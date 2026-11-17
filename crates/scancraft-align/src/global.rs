//! Global registration without an initial guess: FPFH features matched between the passes,
//! RANSAC over feature correspondences (3-point samples with an edge-length consistency test),
//! the best hypotheses refined by point-to-plane ICP and ranked by overlap and visibility.

use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::Transform;
use scancraft_slam::kabsch::kabsch_align;
use scancraft_slam::point_to_plane::{align, PlaneTarget};

use crate::cloud::{downsample, downsample_to, extent, Index, Rng};
use crate::features::{correspondences, fpfh};
use crate::quality::{evaluate_sampled, QualityConfig};
use crate::{rigid, AlignError, Alignment, Oriented, PassCloud, Quality};

#[derive(Debug, Clone)]
pub struct GlobalConfig {
    /// Feature grid; `None` picks it from the object size (about 1/60 of the diagonal).
    pub voxel_mm: Option<f32>,
    /// Upper bound on the points features are computed for, per pass.
    pub max_feature_points: usize,
    pub ransac_iterations: usize,
    /// Hypotheses refined by ICP and scored.
    pub candidates: usize,
    /// Grid of the final ICP refinement.
    pub fine_voxel_mm: f32,
    pub seed: u64,
}

impl Default for GlobalConfig {
    fn default() -> Self {
        GlobalConfig { voxel_mm: None, max_feature_points: 5000, ransac_iterations: 60_000, candidates: 8, fine_voxel_mm: 1.0, seed: 7 }
    }
}

/// Picks the feature grid for two passes from their size.
pub fn auto_voxel(a: &[Oriented], b: &[Oriented]) -> f32 {
    (extent(a).min(extent(b)) / 60.0).clamp(1.5, 8.0)
}

/// Aligns `source` to `target` (both in their own frames) with no initial guess.
pub fn align_global(source: &PassCloud, target: &PassCloud, config: &GlobalConfig) -> Result<Alignment, AlignError> {
    if source.points.len() < 100 {
        return Err(AlignError::TooFewPoints("moving pass"));
    }
    if target.points.len() < 100 {
        return Err(AlignError::TooFewPoints("reference model"));
    }
    let voxel = config.voxel_mm.unwrap_or_else(|| auto_voxel(&source.points, &target.points));
    let (src, voxel) = downsample_to(&source.points, voxel, config.max_feature_points);
    let tgt = downsample(&target.points, voxel);
    let (tgt, voxel) = if tgt.len() > config.max_feature_points { downsample_to(&target.points, voxel, config.max_feature_points) } else { (tgt, voxel) };
    let src = if src.len() > config.max_feature_points * 2 { downsample(&source.points, voxel) } else { src };
    if src.len() < 30 || tgt.len() < 30 {
        return Err(AlignError::TooFewPoints("after subsampling"));
    }
    let src_index = Index::new(src.iter().map(|p| p.0).collect());
    let tgt_index = Index::new(tgt.iter().map(|p| p.0).collect());
    let radius = 5.0 * voxel;
    let fs = fpfh(&src, &src_index, radius);
    let ft = fpfh(&tgt, &tgt_index, radius);
    let corr = correspondences(&fs, &ft, 60);
    if corr.len() < 3 {
        return Err(AlignError::NotFound);
    }
    let hypotheses = ransac(&src, &tgt, &corr, voxel, config);
    if hypotheses.is_empty() {
        return Err(AlignError::NotFound);
    }

    // Refine each hypothesis on the feature grid and score it.
    let target_plane = PlaneTarget::new(tgt.iter().map(|p| p.0).collect(), tgt.iter().map(|p| p.1).collect());
    let qconfig = QualityConfig::for_sample(voxel);
    let gates = [3.0 * voxel, 2.0 * voxel, 1.2 * voxel];
    let mut ranked: Vec<(Transform, Quality)> = hypotheses
        .par_iter()
        .map(|h| {
            let r = align(&src, &target_plane, *h, &gates, 8);
            let pose = if r.inliers > 0 { r.pose } else { *h };
            (pose, evaluate_sampled(&src, &tgt, &tgt_index, &source.views, &target.views, &pose, qconfig))
        })
        .collect();
    ranked.sort_by(|a, b| b.1.score().total_cmp(&a.1.score()));
    let (coarse, coarse_quality) = ranked[0];
    log::debug!("global alignment: voxel {voxel:.2} mm, {} correspondences, best coarse {:?}", corr.len(), coarse_quality);
    if coarse_quality.inliers == 0 {
        return Err(AlignError::NotFound);
    }
    Ok(refine_fine(source, target, coarse, config.fine_voxel_mm.min(voxel)))
}

/// Fine point-to-plane ICP from `guess` (target <- source) and the final score.
pub fn refine_fine(source: &PassCloud, target: &PassCloud, guess: Transform, fine_mm: f32) -> Alignment {
    let (src, fine) = downsample_to(&source.points, fine_mm, 60_000);
    let tgt = downsample(&target.points, fine);
    let plane = PlaneTarget::new(tgt.iter().map(|p| p.0).collect(), tgt.iter().map(|p| p.1).collect());
    let gates = [6.0 * fine, 3.0 * fine, 2.0 * fine, 1.2 * fine];
    let r = align(&src, &plane, guess, &gates, 10);
    let transform = if r.inliers > 0 { r.pose } else { guess };
    let index = Index::new(tgt.iter().map(|p| p.0).collect());
    let quality = evaluate_sampled(&src, &tgt, &index, &source.views, &target.views, &transform, QualityConfig::for_sample(fine.max(1.0)));
    Alignment { transform, quality }
}

/// RANSAC over feature correspondences; returns the best distinct hypotheses, best first.
fn ransac(src: &[Oriented], tgt: &[Oriented], corr: &[(usize, usize)], voxel: f32, config: &GlobalConfig) -> Vec<Transform> {
    let inlier_sq = (2.0 * voxel) * (2.0 * voxel);
    let min_edge = 3.0 * voxel;
    let chunks = 16usize;
    let per_chunk = config.ransac_iterations.div_ceil(chunks);
    let found: Vec<(usize, Transform)> = (0..chunks)
        .into_par_iter()
        .flat_map_iter(|c| {
            let mut rng = Rng::new(config.seed.wrapping_add(c as u64 * 7919));
            let mut out: Vec<(usize, Transform)> = Vec::new();
            for _ in 0..per_chunk {
                let pick = [rng.below(corr.len()), rng.below(corr.len()), rng.below(corr.len())];
                if pick[0] == pick[1] || pick[1] == pick[2] || pick[0] == pick[2] {
                    continue;
                }
                let s: Vec<Vec3> = pick.iter().map(|&k| src[corr[k].0].0).collect();
                let t: Vec<Vec3> = pick.iter().map(|&k| tgt[corr[k].1].0).collect();
                // Rigid motions keep distances: reject samples whose triangles differ.
                let consistent = [(0, 1), (1, 2), (0, 2)].iter().all(|&(a, b)| {
                    let (ds, dt) = ((s[a] - s[b]).length(), (t[a] - t[b]).length());
                    ds > min_edge && dt > min_edge && ds.min(dt) / ds.max(dt) > 0.9
                });
                if !consistent {
                    continue;
                }
                let Some((rot, trans)) = kabsch_align(&s, &t) else { continue };
                let pose = rigid(rot, trans);
                let m = pose.to_mat4();
                let inliers = corr.iter().filter(|&&(i, j)| m.transform_point3(src[i].0).distance_squared(tgt[j].0) <= inlier_sq).count();
                if inliers >= 3 {
                    out.push((inliers, pose));
                }
            }
            out
        })
        .collect();
    let mut found = found;
    found.sort_by_key(|f| std::cmp::Reverse(f.0));
    // Re-fit each kept hypothesis on its inliers; keep distinct ones.
    let mut kept: Vec<Transform> = Vec::new();
    for (_, pose) in found {
        if kept.len() >= config.candidates {
            break;
        }
        let m = pose.to_mat4();
        let (s, t): (Vec<Vec3>, Vec<Vec3>) = corr.iter().filter(|&&(i, j)| m.transform_point3(src[i].0).distance_squared(tgt[j].0) <= inlier_sq).map(|&(i, j)| (src[i].0, tgt[j].0)).unzip();
        let pose = kabsch_align(&s, &t).map(|(r, t)| rigid(r, t)).unwrap_or(pose);
        let distinct = kept.iter().all(|k| k.rotation.angle_between(pose.rotation) > 0.17 || (k.translation - pose.translation).length() > 4.0 * voxel);
        if distinct {
            kept.push(pose);
        }
    }
    kept
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic;
    use glam::Quat;

    fn object_pose(rot: Quat, t: Vec3) -> Transform {
        rigid(rot, t)
    }

    /// (rotation error in degrees, largest point displacement on the object in mm).
    fn errors(found: &Transform, truth: &Transform, probe: &[Vec3]) -> (f32, f32) {
        (crate::rotation_angle(found.rotation, truth.rotation).to_degrees(), crate::displacement_on(found, truth, probe))
    }

    #[test]
    fn aligns_a_flipped_pass_without_a_guess() {
        let obj = synthetic::object(1.0);
        // Pass A: object upright at the origin, seen from all around, from above.
        let a_pose = Transform::default();
        let a = synthetic::pass(&obj, &a_pose, &synthetic::ring(Vec3::ZERO, 0.6, 0.0, 6.0, 10, 260.0), 0.1, 0.8, 1);
        // Pass B: flipped upside down and moved; seen from above again (so sees the bottom).
        let b_pose = object_pose(Quat::from_rotation_x(std::f32::consts::PI) * Quat::from_rotation_z(0.8), Vec3::new(140.0, -60.0, 25.0));
        let b = synthetic::pass(&obj, &b_pose, &synthetic::ring(Vec3::new(140.0, -60.0, 25.0), 0.6, 0.3, 6.3, 10, 260.0), 0.1, 0.8, 2);
        // Truth: A <- B = A_pose * B_pose^-1.
        let truth = crate::compose(&a_pose, &b_pose.inverse());
        let start = std::time::Instant::now();
        let found = align_global(&b, &a, &GlobalConfig::default()).expect("aligned");
        let elapsed = start.elapsed();
        let (rot, disp) = errors(&found.transform, &truth, &b.points.iter().map(|p| p.0).collect::<Vec<_>>());
        eprintln!("flipped: rotation error {rot:.3} deg, max displacement {disp:.3} mm, {:?}, quality {:?}", elapsed, found.quality);
        assert!(rot < 0.3, "rotation error {rot:.3} deg");
        assert!(disp < 0.6, "displacement {disp:.3} mm");
        assert_eq!(found.quality.verdict(), crate::Verdict::Good, "{:?}", found.quality);
    }

    /// Several random placements with sensor-like noise (0.25 mm) and fewer views: success rate,
    /// accuracy and time.
    #[test]
    fn aligns_from_random_placements_with_noise() {
        let obj = synthetic::object(1.0);
        let a = synthetic::pass(&obj, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.7, 0.0, 6.0, 8, 280.0), 0.25, 0.8, 30);
        let mut rng = crate::cloud::Rng::new(77);
        let (mut worst_rot, mut worst_disp, mut slowest) = (0.0f32, 0.0f32, std::time::Duration::ZERO);
        for k in 0..5 {
            let axis = Vec3::new(rng.normal(), rng.normal(), rng.normal()).normalize();
            let pose = object_pose(Quat::from_axis_angle(axis, 0.8 + 2.3 * rng.unit()), Vec3::new(rng.normal(), rng.normal(), rng.normal()) * 80.0);
            let centre = pose.transform_point3(Vec3::ZERO);
            let b = synthetic::pass(&obj, &pose, &synthetic::ring(centre, 0.7, rng.unit() * 6.0, rng.unit() * 6.0 + 4.0, 6, 280.0), 0.25, 0.8, 31 + k);
            let t0 = std::time::Instant::now();
            let found = align_global(&b, &a, &GlobalConfig::default()).expect("aligned");
            slowest = slowest.max(t0.elapsed());
            let (rot, disp) = errors(&found.transform, &pose.inverse(), &b.points.iter().map(|p| p.0).collect::<Vec<_>>());
            eprintln!("placement {k}: rotation error {rot:.3} deg, max displacement {disp:.3} mm, {:?}", found.quality);
            worst_rot = worst_rot.max(rot);
            worst_disp = worst_disp.max(disp);
        }
        eprintln!("random placements: worst rotation {worst_rot:.3} deg, worst displacement {worst_disp:.3} mm, slowest {slowest:?}");
        assert!(worst_rot < 0.5 && worst_disp < 1.0, "worst rot {worst_rot:.3} deg, disp {worst_disp:.3} mm");
    }

    #[test]
    fn aligns_partially_overlapping_passes() {
        let obj = synthetic::object(1.0);
        // Same object orientation, but each pass covers only half the way round.
        let a = synthetic::pass(&obj, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.5, -0.3, 1.5, 5, 260.0), 0.12, 0.8, 3);
        let b_pose = object_pose(Quat::from_euler(glam::EulerRot::ZXY, 2.2, 0.4, -0.3), Vec3::new(-30.0, 80.0, 10.0));
        let centre = b_pose.transform_point3(Vec3::ZERO);
        // Object-relative azimuths about 2.0..3.8 (the object is turned by 2.2 about Z).
        let b = synthetic::pass(&obj, &b_pose, &synthetic::ring(centre, 0.5, 4.2, 6.0, 5, 260.0), 0.12, 0.8, 4);
        let truth = b_pose.inverse();
        let found = align_global(&b, &a, &GlobalConfig::default()).expect("aligned");
        let (rot, disp) = errors(&found.transform, &truth, &b.points.iter().map(|p| p.0).collect::<Vec<_>>());
        eprintln!("partial: rotation error {rot:.3} deg, max displacement {disp:.3} mm, quality {:?}", found.quality);
        assert!(rot < 0.4 && disp < 0.8, "rot {rot:.3} deg, disp {disp:.3} mm");
        assert!(found.quality.overlap > 0.1 && found.quality.overlap < 0.8, "{:?}", found.quality);
    }

    #[test]
    fn wrong_alignment_scores_poorly() {
        let obj = synthetic::object(1.0);
        let a = synthetic::pass(&obj, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.6, 0.0, 6.0, 8, 260.0), 0.1, 0.8, 5);
        let b = synthetic::pass(&obj, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.6, 0.4, 6.4, 8, 260.0), 0.1, 0.8, 6);
        let q = QualityConfig::for_sample(2.0);
        let right = crate::evaluate(&b, &a, &Transform::default(), q);
        // Upside down in place: much of it lands inside or in front of measured surface.
        let wrong = crate::evaluate(&b, &a, &rigid(Quat::from_rotation_x(std::f32::consts::PI), Vec3::ZERO), q);
        eprintln!("right {right:?}\nwrong {wrong:?}");
        assert_eq!(right.verdict(), crate::Verdict::Good);
        assert!(wrong.visibility_conflict > 0.08, "{wrong:?}");
        assert_ne!(wrong.verdict(), crate::Verdict::Good);
        assert!(wrong.score() < right.score() * 0.3);
    }
}
