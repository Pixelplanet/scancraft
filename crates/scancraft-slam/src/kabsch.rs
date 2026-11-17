use glam::{Mat3, Quat, Vec3};

/// Compute the best-fit rigid transformation (Rotation and Translation)
/// aligning source points `P` to target points `Q` using the Kabsch algorithm.
///
/// Finds R in SO(3) and t in R^3 minimizing sum || (R * p_i + t) - q_i ||^2
pub fn kabsch_align(source: &[Vec3], target: &[Vec3]) -> Option<(Quat, Vec3)> {
    let n = source.len();
    if n < 3 || n != target.len() {
        return None;
    }

    // 1. Centroids
    let mut mean_p = Vec3::ZERO;
    let mut mean_q = Vec3::ZERO;
    for (p, q) in source.iter().zip(target.iter()) {
        mean_p += *p;
        mean_q += *q;
    }
    let inv_n = 1.0 / (n as f32);
    mean_p *= inv_n;
    mean_q *= inv_n;

    // 2. Cross-covariance matrix H = sum_i (p_i - mean_p) * (q_i - mean_q)^T
    let mut h = Mat3::ZERO;
    for (p, q) in source.iter().zip(target.iter()) {
        let dp = *p - mean_p;
        let dq = *q - mean_q;

        // Outer product: dp * dq^T
        h.x_axis += dp * dq.x;
        h.y_axis += dp * dq.y;
        h.z_axis += dp * dq.z;
    }

    // 3. SVD of 3x3 matrix H = U * S * V^T using Two-Sided Jacobi SVD
    let (u, _s, v) = svd_3x3(h);

    // 4. Optimal rotation R = V * U^T
    let mut r = v * u.transpose();

    // Check for reflection (det(R) < 0)
    if r.determinant() < 0.0 {
        // Negate the 3rd column of V
        let mut v_adj = v;
        v_adj.z_axis = -v_adj.z_axis;
        r = v_adj * u.transpose();
    }

    // Normalize rotation matrix
    let quat = Quat::from_mat3(&r).normalize();

    // 5. Optimal translation t = mean_q - R * mean_p
    let t = mean_q - quat.mul_vec3(mean_p);

    Some((quat, t))
}

/// Compute SVD of a 3x3 matrix A = U * S * V^T using Jacobi rotations.
/// Returns (U, S, V) where U, V are orthogonal 3x3 matrices and S is singular values.
pub fn svd_3x3(a: Mat3) -> (Mat3, Vec3, Mat3) {
    let mut b = a;
    let mut v = Mat3::IDENTITY;

    const MAX_SWEEPS: usize = 30;
    const EPS: f32 = 1e-7;

    for _ in 0..MAX_SWEEPS {
        let mut converged = true;

        for p in 0..3 {
            for q in (p + 1)..3 {
                // Symmetrize off-diagonal via Jacobi rotations
                let (ap, aq) = (col(&b, p), col(&b, q));
                let alpha = ap.dot(ap);
                let beta = aq.dot(aq);
                let gamma = ap.dot(aq);

                if gamma.abs() > EPS * (alpha * beta).sqrt().max(EPS) {
                    converged = false;
                    let zeta = (beta - alpha) / (2.0 * gamma);
                    let t = if zeta >= 0.0 {
                        1.0 / (zeta + (1.0 + zeta * zeta).sqrt())
                    } else {
                        -1.0 / (-zeta + (1.0 + zeta * zeta).sqrt())
                    };
                    let c = 1.0 / (1.0 + t * t).sqrt();
                    let s = t * c;

                    // Rotate columns of B and V
                    rotate_cols(&mut b, p, q, c, s);
                    rotate_cols(&mut v, p, q, c, s);
                }
            }
        }

        if converged {
            break;
        }
    }

    // Compute singular values as norms of columns of B
    let s0 = col(&b, 0).length();
    let s1 = col(&b, 1).length();
    let mut s2 = col(&b, 2).length();

    let u0 = if s0 > EPS { col(&b, 0) / s0 } else { Vec3::X };
    let u1 = if s1 > EPS { col(&b, 1) / s1 } else { Vec3::Y };
    let mut u2 = if s2 > EPS { col(&b, 2) / s2 } else { Vec3::Z };

    // Ensure right-handed coordinate system
    if u0.cross(u1).dot(u2) < 0.0 {
        u2 = -u2;
        s2 = -s2;
    }

    let u = Mat3::from_cols(u0, u1, u2);

    (u, Vec3::new(s0, s1, s2), v)
}

fn col(m: &Mat3, idx: usize) -> Vec3 {
    match idx {
        0 => m.x_axis,
        1 => m.y_axis,
        _ => m.z_axis,
    }
}

fn set_col(m: &mut Mat3, idx: usize, val: Vec3) {
    match idx {
        0 => m.x_axis = val,
        1 => m.y_axis = val,
        _ => m.z_axis = val,
    }
}

fn rotate_cols(m: &mut Mat3, p: usize, q: usize, c: f32, s: f32) {
    let col_p = col(m, p);
    let col_q = col(m, q);
    set_col(m, p, col_p * c - col_q * s);
    set_col(m, q, col_p * s + col_q * c);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kabsch_identity() {
        let pts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 0.0, 10.0),
        ];

        let (r, t) = kabsch_align(&pts, &pts).expect("Should find transform");
        assert!(t.length() < 1e-4);
        assert!((r.w.abs() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_kabsch_translation_and_rotation() {
        let angle = std::f32::consts::FRAC_PI_4;
        let expected_rot = Quat::from_rotation_z(angle);
        let expected_trans = Vec3::new(5.0, -3.0, 2.0);

        let source = vec![
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(0.0, 0.0, 3.0),
            Vec3::new(2.0, 1.0, -1.0),
        ];

        let target: Vec<Vec3> = source
            .iter()
            .map(|&p| expected_rot.mul_vec3(p) + expected_trans)
            .collect();

        let (r, t) = kabsch_align(&source, &target).expect("Should find transform");
        assert!((t - expected_trans).length() < 1e-3);
        let diff_angle = (r.inverse() * expected_rot).to_axis_angle().1;
        assert!(diff_angle.abs() < 1e-3);
    }
}
