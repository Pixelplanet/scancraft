//! Alignment from marker dots. Only markers rigidly attached to the object move with it when it
//! is flipped, so only those tagged [`MarkerPlacement::OnObject`] are used; markers on the table
//! or on marker blocks around the object ([`MarkerPlacement::Environment`], the default) are
//! ignored here (they serve tracking inside a pass).
//!
//! With ids (e.g. coded targets) the correspondences are known; without, they are found by
//! RANSAC over candidate matches with similar neighbour-distance signatures.

use glam::Vec3;
use scancraft_core::Transform;
use scancraft_slam::kabsch::kabsch_align;

use crate::cloud::Rng;
use crate::{rigid, AlignError};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MarkerPlacement {
    OnObject,
    #[default]
    Environment,
}

/// A marker's 3D position in its pass's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Marker {
    pub position: Vec3,
    /// Identity if the marker is coded / was matched across passes already.
    pub id: Option<u32>,
    pub placement: MarkerPlacement,
}

#[derive(Debug, Clone)]
pub struct MarkerAlignment {
    /// target <- source.
    pub transform: Transform,
    /// (source index, target index) of the markers used.
    pub matches: Vec<(usize, usize)>,
    /// RMS distance of the matched markers after alignment, mm.
    pub rms_mm: f32,
}

/// Aligns from markers seen in both passes. `tolerance_mm` is how far a matched marker may be
/// from its counterpart (detection accuracy, ~0.5-1 mm).
pub fn align_markers(source: &[Marker], target: &[Marker], tolerance_mm: f32) -> Result<MarkerAlignment, AlignError> {
    let src: Vec<usize> = (0..source.len()).filter(|&i| source[i].placement == MarkerPlacement::OnObject).collect();
    let tgt: Vec<usize> = (0..target.len()).filter(|&i| target[i].placement == MarkerPlacement::OnObject).collect();
    if src.len() < 3 || tgt.len() < 3 {
        return Err(AlignError::TooFewMarkers(src.len().min(tgt.len())));
    }
    // Known identities first.
    let by_id: Vec<(usize, usize)> = src
        .iter()
        .filter_map(|&i| source[i].id.and_then(|id| tgt.iter().find(|&&j| target[j].id == Some(id)).map(|&j| (i, j))))
        .collect();
    if by_id.len() >= 3 {
        return solve(source, target, &by_id, tolerance_mm).and_then(|a| {
            // Drop id matches that do not fit (a misread code), then solve again.
            let good: Vec<(usize, usize)> = by_id.iter().copied().filter(|&(i, j)| a.transform.transform_point3(source[i].position).distance(target[j].position) <= 3.0 * tolerance_mm).collect();
            if good.len() >= 3 && good.len() < by_id.len() { solve(source, target, &good, tolerance_mm) } else { Ok(a) }
        });
    }
    unlabelled(source, target, &src, &tgt, tolerance_mm)
}

fn solve(source: &[Marker], target: &[Marker], matches: &[(usize, usize)], _tolerance_mm: f32) -> Result<MarkerAlignment, AlignError> {
    let (s, t): (Vec<Vec3>, Vec<Vec3>) = matches.iter().map(|&(i, j)| (source[i].position, target[j].position)).unzip();
    let (rot, trans) = kabsch_align(&s, &t).ok_or(AlignError::Degenerate)?;
    let transform = rigid(rot, trans);
    let sq: f32 = s.iter().zip(&t).map(|(a, b)| transform.transform_point3(*a).distance_squared(*b)).sum();
    Ok(MarkerAlignment { transform, matches: matches.to_vec(), rms_mm: (sq / s.len() as f32).sqrt() })
}

/// Sorted distances to the nearest few other markers of the same set.
fn signature(set: &[Marker], idx: &[usize], i: usize) -> Vec<f32> {
    let mut d: Vec<f32> = idx.iter().filter(|&&j| j != i).map(|&j| set[i].position.distance(set[j].position)).collect();
    d.sort_by(f32::total_cmp);
    d.truncate(4);
    d
}

fn unlabelled(source: &[Marker], target: &[Marker], src: &[usize], tgt: &[usize], tol: f32) -> Result<MarkerAlignment, AlignError> {
    let sig_s: Vec<Vec<f32>> = src.iter().map(|&i| signature(source, src, i)).collect();
    let sig_t: Vec<Vec<f32>> = tgt.iter().map(|&j| signature(target, tgt, j)).collect();
    // Candidate matches: for each source marker, the target markers with the closest signatures.
    let mut cand: Vec<(usize, usize)> = Vec::new();
    for (a, ss) in sig_s.iter().enumerate() {
        let mut scored: Vec<(f32, usize)> = sig_t
            .iter()
            .enumerate()
            .map(|(b, st)| (ss.iter().zip(st).map(|(x, y)| (x - y).abs()).sum::<f32>() / ss.len().min(st.len()).max(1) as f32, b))
            .collect();
        scored.sort_by(|x, y| x.0.total_cmp(&y.0));
        cand.extend(scored.iter().take(4).map(|&(_, b)| (src[a], tgt[b])));
    }
    let mut rng = Rng::new(11);
    let mut best: Option<(usize, Transform)> = None;
    for _ in 0..4000 {
        let k = [rng.below(cand.len()), rng.below(cand.len()), rng.below(cand.len())];
        let m: Vec<(usize, usize)> = k.iter().map(|&k| cand[k]).collect();
        let distinct = m[0].0 != m[1].0 && m[1].0 != m[2].0 && m[0].0 != m[2].0 && m[0].1 != m[1].1 && m[1].1 != m[2].1 && m[0].1 != m[2].1;
        if !distinct {
            continue;
        }
        let ok = [(0, 1), (1, 2), (0, 2)].iter().all(|&(a, b)| {
            let ds = source[m[a].0].position.distance(source[m[b].0].position);
            let dt = target[m[a].1].position.distance(target[m[b].1].position);
            ds > 5.0 && (ds - dt).abs() <= 2.0 * tol
        });
        if !ok {
            continue;
        }
        let (s, t): (Vec<Vec3>, Vec<Vec3>) = m.iter().map(|&(i, j)| (source[i].position, target[j].position)).unzip();
        let Some((rot, trans)) = kabsch_align(&s, &t) else { continue };
        let pose = rigid(rot, trans);
        let inliers = src.iter().filter(|&&i| tgt.iter().any(|&j| pose.transform_point3(source[i].position).distance(target[j].position) <= 2.0 * tol)).count();
        if best.as_ref().is_none_or(|(b, _)| inliers > *b) {
            best = Some((inliers, pose));
        }
    }
    let Some((inliers, pose)) = best else { return Err(AlignError::NotFound) };
    if inliers < 3 {
        return Err(AlignError::NotFound);
    }
    // Final matches: nearest counterpart within tolerance, one-to-one.
    let mut matches = Vec::new();
    for &i in src {
        let p = pose.transform_point3(source[i].position);
        if let Some(&j) = tgt.iter().filter(|&&j| p.distance(target[j].position) <= 2.0 * tol).min_by(|&&a, &&b| p.distance(target[a].position).total_cmp(&p.distance(target[b].position))) {
            if !matches.iter().any(|&(_, jj)| jj == j) {
                matches.push((i, j));
            }
        }
    }
    if matches.len() < 3 {
        return Err(AlignError::TooFewMarkers(matches.len()));
    }
    solve(source, target, &matches, tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    fn markers(rng: &mut Rng, n: usize, placement: MarkerPlacement, spread: f32, offset: Vec3) -> Vec<Marker> {
        (0..n).map(|i| Marker { position: offset + Vec3::new(rng.unit() - 0.5, rng.unit() - 0.5, rng.unit() - 0.5) * spread, id: Some(i as u32), placement }).collect()
    }

    fn observed(markers: &[Marker], pose: &Transform, rng: &mut Rng, noise: f32) -> Vec<Marker> {
        markers.iter().map(|m| Marker { position: pose.transform_point3(m.position) + Vec3::new(rng.normal(), rng.normal(), rng.normal()) * noise, ..*m }).collect()
    }

    /// Object markers move with the flipped object; environment markers stay on the table.
    fn scene(with_ids: bool) -> (Vec<Marker>, Vec<Marker>, Transform) {
        let mut rng = Rng::new(5);
        let on_object = markers(&mut rng, 9, MarkerPlacement::OnObject, 90.0, Vec3::ZERO);
        let mut env = markers(&mut rng, 8, MarkerPlacement::Environment, 250.0, Vec3::new(0.0, 0.0, -60.0));
        for (k, m) in env.iter_mut().enumerate() {
            m.id = Some(100 + k as u32);
        }
        let flip = rigid(Quat::from_rotation_x(3.1) * Quat::from_rotation_z(0.6), Vec3::new(20.0, -10.0, 5.0));
        let mut a = observed(&on_object, &Transform::default(), &mut rng, 0.05);
        a.extend(observed(&env, &Transform::default(), &mut rng, 0.05));
        // In pass B the object is flipped; the table markers did not move (both passes keep the
        // table's frame here, so they would pull towards identity if they were used).
        let mut b = observed(&on_object, &flip, &mut rng, 0.05);
        b.extend(observed(&env, &Transform::default(), &mut rng, 0.05));
        // Only part of the markers is seen in each pass.
        a.remove(2);
        b.remove(6);
        if !with_ids {
            a.iter_mut().chain(b.iter_mut()).for_each(|m| m.id = None);
        }
        (a, b, flip.inverse())
    }

    #[test]
    fn solves_from_object_markers_only() {
        for with_ids in [true, false] {
            let (a, b, truth) = scene(with_ids);
            let found = align_markers(&b, &a, 0.5).expect("aligned");
            let rot = crate::rotation_angle(found.transform.rotation, truth.rotation).to_degrees();
            let disp = crate::displacement_on(&found.transform, &truth, &[Vec3::ZERO, Vec3::X * 60.0, Vec3::Z * 60.0]);
            eprintln!("markers (ids {with_ids}): {} matched, rms {:.3} mm, rotation error {rot:.3} deg, displacement {disp:.3} mm", found.matches.len(), found.rms_mm);
            assert!(found.matches.len() >= 6, "{:?}", found.matches);
            assert!(found.matches.iter().all(|&(i, _)| b[i].placement == MarkerPlacement::OnObject));
            assert!(rot < 0.2 && disp < 0.3, "rot {rot} disp {disp}");
        }
    }

    #[test]
    fn environment_markers_alone_are_not_enough() {
        let mut rng = Rng::new(9);
        let env = markers(&mut rng, 10, MarkerPlacement::Environment, 200.0, Vec3::ZERO);
        assert_eq!(align_markers(&env, &env, 0.5).unwrap_err(), AlignError::TooFewMarkers(0));
    }
}
