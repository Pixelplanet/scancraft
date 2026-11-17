//! Rigid registration of two marker constellations (3D point sets with unknown correspondence):
//! nearest-neighbour association from a pose guess, or — without a guess — triangle matching
//! (side-length signatures) with RANSAC, both finished by Kabsch on all inliers.

use glam::Vec3;
use scancraft_core::Transform;
use scancraft_slam::kabsch::kabsch_align;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MatchConfig {
    /// A source point is an inlier if it lands within this distance of a target point, mm.
    pub inlier_mm: f32,
    /// Association gate around the guess, mm (covers motion between frames the guess misses).
    pub guess_gate_mm: f32,
    /// Side lengths of two triangles agree within this tolerance, mm.
    pub edge_tol_mm: f32,
    /// Triangle sides considered, mm. Short sides are noisy, long ones rarely co-visible.
    pub min_edge_mm: f32,
    pub max_edge_mm: f32,
    /// Smallest triangle height (rejects near-collinear triples), mm.
    pub min_height_mm: f32,
    /// Source triangles tried at most.
    pub max_triangles: usize,
    pub min_inliers: usize,
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self {
            inlier_mm: 1.5,
            guess_gate_mm: 8.0,
            edge_tol_mm: 2.0,
            min_edge_mm: 8.0,
            max_edge_mm: 250.0,
            min_height_mm: 4.0,
            max_triangles: 400,
            min_inliers: 3,
        }
    }
}

/// `transform` maps source points onto target points (target <- source).
#[derive(Debug, Clone, PartialEq)]
pub struct RigidMatch {
    pub transform: Transform,
    /// (source index, target index).
    pub pairs: Vec<(usize, usize)>,
    pub rms_mm: f32,
}

impl RigidMatch {
    pub fn inliers(&self) -> usize {
        self.pairs.len()
    }
}

/// Registers `source` onto `target`. With `guess`, tries guess-based association first and falls
/// back to triangle matching if that yields too few inliers.
pub fn register(source: &[Vec3], target: &[Vec3], cfg: &MatchConfig, guess: Option<&Transform>) -> Option<RigidMatch> {
    if source.len() < cfg.min_inliers || target.len() < cfg.min_inliers {
        return None;
    }
    if let Some(g) = guess {
        if let Some(m) = refine(source, target, g, cfg.guess_gate_mm, cfg) {
            if m.inliers() >= cfg.min_inliers.max(source.len().min(4)) {
                return Some(m);
            }
        }
    }
    register_global(source, target, cfg)
}

/// Triangle-signature RANSAC without a pose guess.
pub fn register_global(source: &[Vec3], target: &[Vec3], cfg: &MatchConfig) -> Option<RigidMatch> {
    if source.len() < 3 || target.len() < 3 {
        return None;
    }
    // Target neighbour lists with distances, for triangle lookup.
    let neighbours: Vec<Vec<(usize, f32)>> = target
        .iter()
        .map(|p| {
            let mut n: Vec<(usize, f32)> = target
                .iter()
                .enumerate()
                .map(|(j, q)| (j, p.distance(*q)))
                .filter(|&(_, d)| d >= cfg.min_edge_mm - cfg.edge_tol_mm && d <= cfg.max_edge_mm + cfg.edge_tol_mm)
                .collect();
            n.sort_by(|a, b| a.1.total_cmp(&b.1));
            n
        })
        .collect();

    // Source triangles, largest (most stable) first.
    let mut triangles: Vec<([usize; 3], f32)> = Vec::new();
    let n = source.len();
    for a in 0..n {
        for b in a + 1..n {
            let dab = source[a].distance(source[b]);
            if dab < cfg.min_edge_mm || dab > cfg.max_edge_mm {
                continue;
            }
            for c in b + 1..n {
                let (dbc, dca) = (source[b].distance(source[c]), source[c].distance(source[a]));
                if dbc < cfg.min_edge_mm || dbc > cfg.max_edge_mm || dca < cfg.min_edge_mm || dca > cfg.max_edge_mm {
                    continue;
                }
                let area2 = (source[b] - source[a]).cross(source[c] - source[a]).length();
                let longest = dab.max(dbc).max(dca);
                if area2 / longest < cfg.min_height_mm {
                    continue;
                }
                triangles.push(([a, b, c], area2));
            }
        }
    }
    triangles.sort_by(|x, y| y.1.total_cmp(&x.1));
    triangles.truncate(cfg.max_triangles);

    let mut best: Option<(usize, f32, Transform)> = None;
    for ([a, b, c], _) in &triangles {
        let (dab, dbc, dca) = (source[*a].distance(source[*b]), source[*b].distance(source[*c]), source[*c].distance(source[*a]));
        // Neighbour lists are sorted by distance: only the window around the wanted side length.
        let window = |list: &[(usize, f32)], d: f32| -> std::ops::Range<usize> {
            list.partition_point(|e| e.1 < d - cfg.edge_tol_mm)..list.partition_point(|e| e.1 <= d + cfg.edge_tol_mm)
        };
        for i in 0..target.len() {
            for &(j, _) in &neighbours[i][window(&neighbours[i], dab)] {
                for &(k, _) in &neighbours[j][window(&neighbours[j], dbc)] {
                    if k == i {
                        continue;
                    }
                    if (target[k].distance(target[i]) - dca).abs() > cfg.edge_tol_mm {
                        continue;
                    }
                    let src = [source[*a], source[*b], source[*c]];
                    let tgt = [target[i], target[j], target[k]];
                    let Some((q, t)) = kabsch_align(&src, &tgt) else { continue };
                    let tf = Transform::from_rotation_translation(q, t);
                    // Mirror solutions pass the side lengths but not the Kabsch residual.
                    if src.iter().zip(&tgt).any(|(s, g)| tf.transform_point3(*s).distance(*g) > 2.0 * cfg.edge_tol_mm) {
                        continue;
                    }
                    let (count, sq) = score(source, target, &tf, cfg.inlier_mm);
                    let better = best.as_ref().is_none_or(|(bc, bs, _)| count > *bc || (count == *bc && sq < *bs));
                    if better {
                        best = Some((count, sq, tf));
                    }
                }
            }
        }
        if let Some((count, _, _)) = &best {
            if *count >= 4 && (*count == source.len() || *count >= 12) {
                break;
            }
        }
    }
    let (count, _, tf) = best?;
    if count < cfg.min_inliers {
        return None;
    }
    refine(source, target, &tf, cfg.inlier_mm * 2.0, cfg)
}

/// Associates by nearest neighbour within `gate` from `start`, then re-solves with Kabsch and
/// re-associates with the inlier tolerance (a few rounds).
fn refine(source: &[Vec3], target: &[Vec3], start: &Transform, gate: f32, cfg: &MatchConfig) -> Option<RigidMatch> {
    let mut tf = *start;
    let mut pairs = associate(source, target, &tf, gate);
    for round in 0..4 {
        if pairs.len() < 3 {
            return None;
        }
        let src: Vec<Vec3> = pairs.iter().map(|&(s, _)| source[s]).collect();
        let tgt: Vec<Vec3> = pairs.iter().map(|&(_, t)| target[t]).collect();
        let (q, t) = kabsch_align(&src, &tgt)?;
        tf = Transform::from_rotation_translation(q, t);
        let tol = if round == 0 { gate.min(cfg.inlier_mm * 3.0) } else { cfg.inlier_mm };
        let next = associate(source, target, &tf, tol);
        if next == pairs {
            break;
        }
        pairs = next;
    }
    if pairs.len() < cfg.min_inliers {
        return None;
    }
    let rms = (pairs.iter().map(|&(s, t)| tf.transform_point3(source[s]).distance_squared(target[t])).sum::<f32>() / pairs.len() as f32).sqrt();
    Some(RigidMatch { transform: tf, pairs, rms_mm: rms })
}

/// Mutual nearest neighbours within `tol` after transforming the source.
fn associate(source: &[Vec3], target: &[Vec3], tf: &Transform, tol: f32) -> Vec<(usize, usize)> {
    let moved: Vec<Vec3> = source.iter().map(|p| tf.transform_point3(*p)).collect();
    let nearest = |p: Vec3, set: &[Vec3]| -> Option<(usize, f32)> {
        set.iter().enumerate().map(|(i, q)| (i, p.distance(*q))).min_by(|a, b| a.1.total_cmp(&b.1))
    };
    let mut pairs = Vec::new();
    for (s, p) in moved.iter().enumerate() {
        let Some((t, d)) = nearest(*p, target) else { continue };
        if d > tol {
            continue;
        }
        if nearest(target[t], &moved).is_some_and(|(back, _)| back == s) {
            pairs.push((s, t));
        }
    }
    pairs
}

fn score(source: &[Vec3], target: &[Vec3], tf: &Transform, tol: f32) -> (usize, f32) {
    let mut count = 0;
    let mut sq = 0.0;
    for p in source {
        let m = tf.transform_point3(*p);
        let d = target.iter().map(|q| m.distance_squared(*q)).fold(f32::MAX, f32::min);
        if d <= tol * tol {
            count += 1;
            sq += d;
        }
    }
    (count, sq)
}

/// Rotation angle between two transforms, radians.
pub fn rotation_delta(a: &Transform, b: &Transform) -> f32 {
    (a.rotation * b.rotation.inverse()).normalize().to_axis_angle().1.abs().min(std::f32::consts::TAU)
}

/// `a ∘ b` (apply b, then a).
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    Transform::from_rotation_translation((a.rotation * b.rotation).normalize(), a.rotation * b.translation + a.translation)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::detect::tests::Lcg;

    /// Marker-block-like constellation: a few blocks with irregular dot layouts.
    pub fn constellation(seed: u64, n: usize) -> Vec<Vec3> {
        let mut rng = Lcg(seed);
        let mut pts: Vec<Vec3> = Vec::new();
        while pts.len() < n {
            let p = Vec3::new(rng.next_f32() * 160.0 - 80.0, 250.0 + rng.next_f32() * 120.0, rng.next_f32() * 100.0 - 50.0);
            if pts.iter().all(|q| q.distance(p) > 12.0) {
                pts.push(p);
            }
        }
        pts
    }

    use glam::Quat;

    pub fn pose(rx: f32, ry: f32, rz: f32, t: Vec3) -> Transform {
        Transform::from_rotation_translation(Quat::from_euler(glam::EulerRot::XYZ, rx, ry, rz), t)
    }

    #[test]
    fn registers_without_guess_with_outliers_and_partial_overlap() {
        let world = constellation(1, 30);
        let truth = pose(0.3, -0.2, 0.5, Vec3::new(20.0, -15.0, 8.0)); // world <- sensor
        let inv = truth.inverse();
        let mut rng = Lcg(5);
        // The sensor sees 12 of the world markers (noisy) plus 3 spurious detections.
        let mut obs: Vec<Vec3> = world[5..17]
            .iter()
            .map(|p| inv.transform_point3(*p) + Vec3::new(rng.next_f32() - 0.5, rng.next_f32() - 0.5, rng.next_f32() - 0.5) * 0.2)
            .collect();
        obs.push(Vec3::new(0.0, 300.0, 0.0));
        obs.push(Vec3::new(37.0, 280.0, -12.0));
        obs.push(Vec3::new(-50.0, 330.0, 41.0));
        let m = register(&obs, &world, &MatchConfig::default(), None).expect("match");
        assert_eq!(m.inliers(), 12, "{:?}", m.pairs);
        assert!(m.rms_mm < 0.2);
        assert!((m.transform.translation - truth.translation).length() < 0.3);
        assert!(rotation_delta(&m.transform, &truth) < 0.003);
    }

    #[test]
    fn guess_based_association_tracks_small_motion() {
        let world = constellation(2, 10);
        let truth = pose(0.02, 0.01, -0.03, Vec3::new(2.0, 1.0, -3.0));
        let obs: Vec<Vec3> = world.iter().map(|p| truth.inverse().transform_point3(*p)).collect();
        let m = register(&obs, &world, &MatchConfig::default(), Some(&Transform::default())).expect("match");
        assert_eq!(m.inliers(), 10);
        assert!((m.transform.translation - truth.translation).length() < 1e-2);
    }

    #[test]
    fn too_few_points_or_unrelated_sets_do_not_match() {
        let world = constellation(3, 20);
        assert!(register(&world[..2], &world, &MatchConfig::default(), None).is_none());
        let other = constellation(99, 8);
        let m = register(&other, &world, &MatchConfig::default(), None);
        assert!(m.as_ref().is_none_or(|m| m.inliers() <= 3), "{m:?}");
    }
}
