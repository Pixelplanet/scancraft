//! Alignment from user-picked point pairs: Kabsch on the pairs gives the starting transform,
//! point-to-plane ICP on the surfaces refines it (picks are only accurate to a few mm).

use glam::Vec3;
use scancraft_slam::kabsch::kabsch_align;
use scancraft_slam::point_to_plane::{align, PlaneTarget};

use crate::cloud::downsample;
use crate::global::refine_fine;
use crate::{rigid, AlignError, Alignment, PassCloud};

/// Aligns `source` to `target` from `pairs` of (point on source, same point on target), each in
/// its pass's frame. Needs at least three pairs that are not on a line.
pub fn align_point_pairs(source: &PassCloud, target: &PassCloud, pairs: &[(Vec3, Vec3)], fine_mm: f32) -> Result<Alignment, AlignError> {
    if pairs.len() < 3 {
        return Err(AlignError::TooFewPairs);
    }
    if source.points.len() < 100 || target.points.len() < 100 {
        return Err(AlignError::TooFewPoints("pass"));
    }
    let (s, t): (Vec<Vec3>, Vec<Vec3>) = pairs.iter().copied().unzip();
    if !spans_a_plane(&s) || !spans_a_plane(&t) {
        return Err(AlignError::Degenerate);
    }
    let (rot, trans) = kabsch_align(&s, &t).ok_or(AlignError::Degenerate)?;
    // Coarse ICP with wide gates first: picks can be several millimetres off.
    let coarse_mm = 3.0f32.max(fine_mm);
    let src = downsample(&source.points, coarse_mm);
    let tgt = downsample(&target.points, coarse_mm);
    let plane = PlaneTarget::new(tgt.iter().map(|p| p.0).collect(), tgt.iter().map(|p| p.1).collect());
    let guess = rigid(rot, trans);
    let r = align(&src, &plane, guess, &[24.0, 12.0, 6.0, 3.0], 8);
    let guess = if r.inliers > 0 { r.pose } else { guess };
    Ok(refine_fine(source, target, guess, fine_mm))
}

/// Whether the points span more than a line (the triangle of the farthest points has some area).
fn spans_a_plane(points: &[Vec3]) -> bool {
    let a = points[0];
    let Some(b) = points.iter().copied().max_by(|p, q| (*p - a).length_squared().total_cmp(&(*q - a).length_squared())) else { return false };
    let ab = b - a;
    if ab.length() < 5.0 {
        return false;
    }
    let off = points.iter().map(|p| ab.cross(*p - a).length() / ab.length()).fold(0.0f32, f32::max);
    off > 3.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Rng;
    use crate::synthetic;
    use glam::Quat;
    use scancraft_core::Transform;

    #[test]
    fn rough_point_pairs_are_refined_onto_the_surface() {
        let obj = synthetic::object(1.0);
        let a = synthetic::pass(&obj, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.6, 0.0, 6.0, 8, 260.0), 0.1, 0.8, 11);
        let b_pose = rigid(Quat::from_rotation_y(2.5) * Quat::from_rotation_z(-0.7), Vec3::new(60.0, 200.0, -40.0));
        let centre = b_pose.transform_point3(Vec3::ZERO);
        let b = synthetic::pass(&obj, &b_pose, &synthetic::ring(centre, 0.6, 0.0, 6.0, 8, 260.0), 0.1, 0.8, 12);
        // Picks on recognisable spots (dome top, peg end, tab, lug), each off by up to ~3 mm.
        let spots = [Vec3::new(18.0, 5.0, 36.0), Vec3::new(-72.0, -8.0, -4.0), Vec3::new(-15.0, -44.0, 8.0), Vec3::new(30.0, 40.0, -6.0)];
        let mut rng = Rng::new(3);
        let mut jitter = || Vec3::new(rng.normal(), rng.normal(), rng.normal()) * 1.5;
        let pairs: Vec<(Vec3, Vec3)> = spots.iter().map(|&x| (b_pose.transform_point3(x) + jitter(), x + jitter())).collect();
        let found = align_point_pairs(&b, &a, &pairs, 1.0).expect("aligned");
        let truth = b_pose.inverse();
        let rot = crate::rotation_angle(found.transform.rotation, truth.rotation).to_degrees();
        let disp = crate::displacement_on(&found.transform, &truth, &b.points.iter().map(|p| p.0).collect::<Vec<_>>());
        eprintln!("point pairs: rotation error {rot:.3} deg, max displacement {disp:.3} mm, {:?}", found.quality);
        assert!(rot < 0.3 && disp < 0.6, "rot {rot:.3} deg, disp {disp:.3} mm");
    }

    #[test]
    fn collinear_pairs_are_rejected() {
        let obj = synthetic::object(2.0);
        let a = synthetic::pass(&obj, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.6, 0.0, 6.0, 4, 260.0), 0.1, 1.0, 1);
        let line: Vec<(Vec3, Vec3)> = (0..4).map(|i| (Vec3::X * i as f32 * 10.0, Vec3::X * i as f32 * 10.0)).collect();
        assert_eq!(align_point_pairs(&a, &a, &line, 1.0).unwrap_err(), AlignError::Degenerate);
        assert_eq!(align_point_pairs(&a, &a, &line[..2], 1.0).unwrap_err(), AlignError::TooFewPairs);
    }
}
