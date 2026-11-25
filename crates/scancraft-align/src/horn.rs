//! Closed-form absolute orientation from picked point pairs: Horn's (1987) unit-quaternion
//! method gives the best-fit rigid transform directly (no iteration), and `PairSpread` says
//! how well the picks span space so the app can warn about near-collinear picks.

use glam::{Mat3, Quat, Vec3};
use scancraft_core::Transform;

use crate::eigen::eigen4;
use crate::{rigid, AlignError};

/// How well picked points span space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairSpread {
    /// Largest distance between two picks, mm.
    pub extent_mm: f32,
    /// Largest distance of a pick from the line through the two farthest picks, mm.
    pub off_line_mm: f32,
}

impl PairSpread {
    /// The spread of a pick set (farthest pair, then the greatest distance off their line).
    pub fn of(points: &[Vec3]) -> PairSpread {
        let (mut extent, mut a, mut b) = (0.0f32, 0usize, 0usize);
        for i in 0..points.len() {
            for j in (i + 1)..points.len() {
                let d = points[i].distance_squared(points[j]);
                if d > extent {
                    extent = d;
                    (a, b) = (i, j);
                }
            }
        }
        let extent = extent.sqrt();
        let line = points[b] - points[a];
        let off_line = if extent < 1e-9 {
            0.0
        } else {
            points.iter().map(|p| line.cross(*p - points[a]).length() / extent).fold(0.0, f32::max)
        };
        PairSpread { extent_mm: extent, off_line_mm: off_line }
    }

    /// Too small or too flat to fix a rotation (the thresholds `pairs` used before).
    pub fn degenerate(&self) -> bool {
        self.extent_mm < 5.0 || self.off_line_mm < 3.0
    }

    /// Not degenerate, but nearly on a line: the fit exists, the fix around that line is weak.
    pub fn nearly_collinear(&self) -> bool {
        !self.degenerate() && self.off_line_mm < 0.15 * self.extent_mm
    }
}

/// A point-pair fit: the transform and how well it explains the pairs.
#[derive(Debug, Clone)]
pub struct PairFit {
    /// target <- source.
    pub transform: Transform,
    /// RMS of the per-pair residuals after the fit, mm.
    pub rms_mm: f32,
    /// Per-pair residual after the fit, mm.
    pub residuals_mm: Vec<f32>,
    /// The worse (smaller off-line/extent ratio) of the source and the target spread.
    pub spread: PairSpread,
}

/// Horn's (1987) closed-form best-fit rotation for corresponding point pairs, as
/// `target <- source`. The 3x3 cross-covariance of the centred points goes into a symmetric
/// 4x4 matrix whose eigenvector of the largest eigenvalue is the rotation's unit quaternion;
/// the translation follows from the centroids. None for fewer than three pairs or a
/// degenerate spread (the pairs fix no unique rotation).
pub fn horn(pairs: &[(Vec3, Vec3)]) -> Option<Transform> {
    if pairs.len() < 3 {
        return None;
    }
    let (src, tgt): (Vec<Vec3>, Vec<Vec3>) = pairs.iter().copied().unzip();
    if PairSpread::of(&src).degenerate() || PairSpread::of(&tgt).degenerate() {
        return None;
    }
    let n = src.len() as f64;
    let cs = src.iter().map(|p| p.as_dvec3()).sum::<glam::DVec3>() / n;
    let ct = tgt.iter().map(|p| p.as_dvec3()).sum::<glam::DVec3>() / n;
    // Cross-covariance S = sum of source' * target'^T: the rotation maps source to target.
    let mut s = [[0.0f64; 3]; 3];
    for (sp, tp) in src.iter().zip(&tgt) {
        let (u, v) = (sp.as_dvec3() - cs, tp.as_dvec3() - ct);
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += u[i] * v[j];
            }
        }
    }
    let (sxx, sxy, sxz) = (s[0][0], s[0][1], s[0][2]);
    let (syx, syy, syz) = (s[1][0], s[1][1], s[1][2]);
    let (szx, szy, szz) = (s[2][0], s[2][1], s[2][2]);
    // Horn's symmetric 4x4: the eigenvector of the largest eigenvalue is the quaternion
    // (w, x, y, z) of the optimal rotation.
    let qm = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ];
    let (_, vectors) = eigen4(qm);
    let q = Quat::from_xyzw(vectors[1][3] as f32, vectors[2][3] as f32, vectors[3][3] as f32, vectors[0][3] as f32).normalize();
    // Guard against a reflection sneaking in (it cannot for a symmetric N, but be explicit).
    debug_assert!(Mat3::from_quat(q).determinant() > 0.0);
    let translation = (ct - q.as_dquat() * cs).as_vec3();
    Some(rigid(q, translation))
}

/// Fits the pairs by [`horn`] and reports the residuals and the pick spread.
pub fn fit_pairs(pairs: &[(Vec3, Vec3)]) -> Result<PairFit, AlignError> {
    if pairs.len() < 3 {
        return Err(AlignError::TooFewPairs);
    }
    let transform = horn(pairs).ok_or(AlignError::Degenerate)?;
    let residuals_mm: Vec<f32> = pairs.iter().map(|&(s, t)| (transform.transform_point3(s) - t).length()).collect();
    let rms_mm = (residuals_mm.iter().map(|r| r * r).sum::<f32>() / residuals_mm.len() as f32).sqrt();
    let (s, t): (Vec<Vec3>, Vec<Vec3>) = pairs.iter().copied().unzip();
    let (ss, ts) = (PairSpread::of(&s), PairSpread::of(&t));
    let spread = if ss.off_line_mm * ts.extent_mm <= ts.off_line_mm * ss.extent_mm { ss } else { ts };
    Ok(PairFit { transform, rms_mm, residuals_mm, spread })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Rng;
    use scancraft_slam::kabsch::kabsch_align;

    fn picks(rng: &mut Rng, count: usize, extent: f32) -> Vec<Vec3> {
        (0..count)
            .map(|i| {
                if i == 0 {
                    Vec3::ZERO
                } else {
                    Vec3::new(rng.unit() * extent, rng.unit() * extent, rng.unit() * extent)
                }
            })
            .collect()
    }

    #[test]
    fn three_exact_pairs_recover_the_transform() {
        let mut rng = Rng::new(9);
        let truth = rigid(Quat::from_rotation_y(0.6) * Quat::from_rotation_z(-0.9), Vec3::new(30.0, -70.0, 45.0));
        let src = picks(&mut rng, 3, 80.0);
        let pairs: Vec<(Vec3, Vec3)> = src.iter().map(|&s| (s, truth.transform_point3(s))).collect();
        let fit = fit_pairs(&pairs).expect("3 pairs fit");
        let err = crate::rotation_angle(fit.transform.rotation, truth.rotation).to_degrees();
        eprintln!("3 pairs: rotation error {err:.5} deg, rms {:.5} mm, extent {:.1} mm", fit.rms_mm, fit.spread.extent_mm);
        assert!(err < 0.01 && fit.rms_mm < 1e-3, "rotation error {err}, rms {}", fit.rms_mm);
    }

    #[test]
    fn noisy_pairs_agree_with_kabsch() {
        let mut rng = Rng::new(17);
        let truth = rigid(Quat::from_rotation_x(-0.8) * Quat::from_rotation_y(0.4), Vec3::new(-25.0, 90.0, 10.0));
        let jitter = |rng: &mut Rng| Vec3::new(rng.normal(), rng.normal(), rng.normal());
        let pairs: Vec<(Vec3, Vec3)> = picks(&mut rng, 6, 120.0)
            .iter()
            .map(|&s| (s + jitter(&mut rng), truth.transform_point3(s) + jitter(&mut rng)))
            .collect();
        let fit = fit_pairs(&pairs).expect("fits");
        let (src, tgt): (Vec<Vec3>, Vec<Vec3>) = pairs.iter().copied().unzip();
        let (kr, kt) = kabsch_align(&src, &tgt).expect("kabsch");
        let rot = crate::rotation_angle(fit.transform.rotation, kr).to_degrees();
        let off = (fit.transform.translation - kt).length();
        eprintln!("noisy pairs: horn vs kabsch {rot:.4} deg / {off:.4} mm, rms {:.3} mm", fit.rms_mm);
        assert!(rot < 0.05 && off < 0.05, "horn vs kabsch: {rot} deg, {off} mm");
    }

    #[test]
    fn mirrored_targets_still_give_a_rotation_and_a_bad_fit() {
        let mut rng = Rng::new(23);
        let src = picks(&mut rng, 5, 100.0);
        // Mirrored at x = 0: no rigid transform explains these targets.
        let pairs: Vec<(Vec3, Vec3)> = src.iter().map(|&s| (s, Vec3::new(-s.x, s.y, s.z))).collect();
        let fit = fit_pairs(&pairs).expect("a rotation, not a reflection");
        let det = Mat3::from_quat(fit.transform.rotation).determinant();
        eprintln!("mirror: det {det:.4}, rms {:.2} mm", fit.rms_mm);
        assert!((det - 1.0).abs() < 1e-4 && fit.rms_mm > 30.0, "det {det}, rms {}", fit.rms_mm);
    }

    #[test]
    fn collinear_and_nearly_collinear_spreads() {
        let line: Vec<Vec3> = (0..4).map(|i| Vec3::new(i as f32 * 30.0, 0.1 * i as f32, 0.0)).collect();
        assert!(PairSpread::of(&line).degenerate());
        // 100 mm long, 8 mm off the line: fits, but nearly collinear.
        let wide = vec![Vec3::ZERO, Vec3::new(100.0, 0.0, 0.0), Vec3::new(50.0, 8.0, 0.0), Vec3::new(25.0, -7.0, 1.0)];
        let spread = PairSpread::of(&wide);
        eprintln!("wide spread: extent {:.1} mm, off-line {:.1} mm", spread.extent_mm, spread.off_line_mm);
        assert!(!spread.degenerate() && spread.nearly_collinear());
        let truth = rigid(Quat::from_rotation_z(1.1), Vec3::new(10.0, 20.0, -5.0));
        let pairs: Vec<(Vec3, Vec3)> = wide.iter().map(|&s| (s, truth.transform_point3(s))).collect();
        let fit = fit_pairs(&pairs).expect("fits");
        assert!(fit.spread.nearly_collinear());
        // Exactly collinear pairs degenerate.
        let col: Vec<(Vec3, Vec3)> = line.iter().map(|&s| (s, s)).collect();
        assert_eq!(fit_pairs(&col).unwrap_err(), AlignError::Degenerate);
        assert_eq!(fit_pairs(&col[..2]).unwrap_err(), AlignError::TooFewPairs);
    }
}
