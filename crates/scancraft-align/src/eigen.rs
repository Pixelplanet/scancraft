//! Small symmetric eigensolvers (own code, textbook cyclic Jacobi): the workspace has no
//! general eigen crate, and only 3x3 and 4x4 symmetric systems are needed.

/// Eigenpairs of a symmetric NxN matrix (N = 3 or 4), f64, by cyclic Jacobi sweeps.
/// Returns (eigenvalues, eigenvectors as columns of the second array), sorted by eigenvalue,
/// smallest first.
pub(crate) fn jacobi<const N: usize>(a: [[f64; N]; N]) -> ([f64; N], [[f64; N]; N]) {
    let mut a = a;
    let mut v = [[0.0f64; N]; N];
    for i in 0..N {
        v[i][i] = 1.0;
    }
    for _sweep in 0..100 {
        let mut off = 0.0f64;
        for p in 0..N {
            for q in (p + 1)..N {
                off = off.max(a[p][q].abs());
            }
        }
        if off < 1e-14 {
            break;
        }
        for p in 0..N {
            for q in (p + 1)..N {
                if a[p][q].abs() < 1e-17 {
                    continue;
                }
                // Rotation angle that zeroes a[p][q] (stable form).
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                let (app, aqq, apq) = (a[p][p], a[q][q], a[p][q]);
                a[p][p] = app - t * apq;
                a[q][q] = aqq + t * apq;
                a[p][q] = 0.0;
                a[q][p] = 0.0;
                for k in 0..N {
                    if k == p || k == q {
                        continue;
                    }
                    let (akp, akq) = (a[k][p], a[k][q]);
                    a[k][p] = c * akp - s * akq;
                    a[p][k] = a[k][p];
                    a[k][q] = s * akp + c * akq;
                    a[q][k] = a[k][q];
                }
                for k in 0..N {
                    let (vkp, vkq) = (v[k][p], v[k][q]);
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    let mut order: Vec<usize> = (0..N).collect();
    order.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let mut values = [0.0f64; N];
    let mut vectors = [[0.0f64; N]; N];
    for (new, &old) in order.iter().enumerate() {
        values[new] = a[old][old];
        for k in 0..N {
            vectors[k][new] = v[k][old];
        }
    }
    (values, vectors)
}

/// Eigenpairs of a symmetric 3x3: (values smallest first, vectors as columns).
pub(crate) fn eigen3(m: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    jacobi(m)
}

/// Eigenpairs of a symmetric 4x4: (values smallest first, vectors as columns).
pub(crate) fn eigen4(m: [[f64; 4]; 4]) -> ([f64; 4], [[f64; 4]; 4]) {
    jacobi(m)
}

/// The unit eigenvector of `eigen3`'s result for eigenvalue `slot` (0 = smallest), as Vec3.
pub(crate) fn vec3_of(vectors: &[[f64; 3]; 3], slot: usize) -> glam::Vec3 {
    glam::Vec3::new(vectors[0][slot] as f32, vectors[1][slot] as f32, vectors[2][slot] as f32).normalize_or_zero()
}
