//! How much a point cloud looks like a real scan: point density and spacing, normal error
//! against a local plane fit, the spatial structure of the noise, and the point density round
//! each listed marker. Works on any cloud (a synthetic pass or Revo's `raw_preview.ply`), so the
//! generator can be checked against the real scans with the same numbers
//! (`examples/synth_realism.rs`).
//!
//! The measures follow the review of 2026-11-28 that compared the synthetic passes with S1-S3:
//!
//! - density: median number of points within 1 mm, divided by pi (points per mm²);
//! - nearest-neighbour distance p10 / p50 / p90;
//! - normal error: angle between the stored normal and a PCA plane over r 2 mm, on smooth
//!   patches only (smallest eigenvalue below 1 % of the middle one, plane rms below 0.05 mm);
//! - noise structure: residuals to a quadratic surface over r 3 mm on smooth patches, their std,
//!   and the mean of r_i r_j / sigma² for pairs 0.2-0.5 mm and 0.5-0.9 mm apart (white noise
//!   gives about 0, real Revo noise 0.4-0.5 and about 0.1);
//! - marker rings: points per mm² in rings round each listed marker that lies on the cloud.

use std::collections::HashMap;

use glam::DVec3;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::rng::Rng;
use crate::scan::smallest_eigenvector;

/// Points on a hash grid for radius queries.
pub struct Grid<'a> {
    points: &'a [DVec3],
    cell: f64,
    map: HashMap<(i64, i64, i64), Vec<u32>>,
}

impl<'a> Grid<'a> {
    pub fn new(points: &'a [DVec3], cell: f64) -> Self {
        let mut map: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
        for (k, p) in points.iter().enumerate() {
            let c = (*p / cell).floor();
            map.entry((c.x as i64, c.y as i64, c.z as i64)).or_default().push(k as u32);
        }
        Grid { points, cell, map }
    }

    /// Indices of the points within `r` of `p`.
    pub fn within(&self, p: DVec3, r: f64) -> Vec<u32> {
        let lo = ((p - DVec3::splat(r)) / self.cell).floor();
        let hi = ((p + DVec3::splat(r)) / self.cell).floor();
        let mut out = Vec::new();
        for x in lo.x as i64..=hi.x as i64 {
            for y in lo.y as i64..=hi.y as i64 {
                for z in lo.z as i64..=hi.z as i64 {
                    if let Some(l) = self.map.get(&(x, y, z)) {
                        out.extend(l.iter().copied().filter(|&j| self.points[j as usize].distance_squared(p) <= r * r));
                    }
                }
            }
        }
        out
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Realism {
    pub points: usize,
    /// Median points within 1 mm / pi.
    pub density_per_mm2: f64,
    /// sqrt(1 / density).
    pub pitch_mm: f64,
    pub nn_p10_mm: f64,
    pub nn_p50_mm: f64,
    pub nn_p90_mm: f64,
    /// Smooth patches used for the normal error.
    pub normal_samples: usize,
    pub normal_err_median_deg: f64,
    pub normal_err_p90_deg: f64,
    pub normal_err_p99_deg: f64,
    /// Share of the points on smooth patches whose normal is more than 20° off.
    pub normal_err_over_20_share: f64,
    pub noise_patches: usize,
    /// Median over patches of the residual std (mm).
    pub residual_std_mm: f64,
    /// Mean r_i r_j / sigma² for pairs 0.2-0.5 mm apart.
    pub corr_02_05: f64,
    /// ... 0.5-0.9 mm apart.
    pub corr_05_09: f64,
    pub markers: Vec<MarkerRings>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MarkerRings {
    pub centre: [f64; 3],
    /// Distance of the nearest point to the centre.
    pub nearest_mm: f64,
    /// Points per mm² in the rings of [`RING_EDGES`] (within 1.5 mm along the normal).
    pub density: Vec<f64>,
}

/// Ring edges (mm) of [`MarkerRings::density`].
pub const RING_EDGES: [f64; 9] = [0.0, 1.5, 3.0, 4.5, 5.5, 6.5, 8.0, 10.0, 12.0];

fn pct(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((sorted.len() - 1) as f64 * q).round() as usize]
}

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

/// PCA of `pts`: (mean, eigenvalues ascending, normal of the smallest).
fn pca(pts: &[DVec3]) -> (DVec3, [f64; 3], DVec3) {
    let n = pts.len().max(1) as f64;
    let mean = pts.iter().copied().sum::<DVec3>() / n;
    let mut m = [[0.0f64; 3]; 3];
    for q in pts {
        let d = *q - mean;
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j] / n;
            }
        }
    }
    let normal = smallest_eigenvector(m);
    // Eigenvalues from the characteristic polynomial (symmetric 3 x 3).
    let p1 = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
    let q = (m[0][0] + m[1][1] + m[2][2]) / 3.0;
    let p2 = (m[0][0] - q).powi(2) + (m[1][1] - q).powi(2) + (m[2][2] - q).powi(2) + 2.0 * p1;
    let p = (p2 / 6.0).sqrt().max(1e-300);
    let b = |i: usize, j: usize| (m[i][j] - if i == j { q } else { 0.0 }) / p;
    let det = b(0, 0) * (b(1, 1) * b(2, 2) - b(1, 2) * b(2, 1)) - b(0, 1) * (b(1, 0) * b(2, 2) - b(1, 2) * b(2, 0)) + b(0, 2) * (b(1, 0) * b(2, 1) - b(1, 1) * b(2, 0));
    let phi = (det / 2.0).clamp(-1.0, 1.0).acos() / 3.0;
    let e1 = q + 2.0 * p * phi.cos();
    let e3 = q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos();
    let e2 = 3.0 * q - e1 - e3;
    let mut e = [e1, e2, e3];
    e.sort_by(f64::total_cmp);
    (mean, e, normal)
}

/// Least-squares quadratic z = a + b u + c v + d u² + e uv + f v² (normal equations, 6 x 6).
fn quadric_residuals(uvz: &[(f64, f64, f64)]) -> Option<Vec<f64>> {
    let mut a = [[0.0f64; 7]; 6];
    for &(u, v, z) in uvz {
        let row = [1.0, u, v, u * u, u * v, v * v];
        for i in 0..6 {
            for j in 0..6 {
                a[i][j] += row[i] * row[j];
            }
            a[i][6] += row[i] * z;
        }
    }
    // Gauss-Jordan.
    for c in 0..6 {
        let piv = (c..6).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[piv][c].abs() < 1e-12 {
            return None;
        }
        a.swap(c, piv);
        for r in 0..6 {
            if r != c {
                let f = a[r][c] / a[c][c];
                for k in c..7 {
                    a[r][k] -= f * a[c][k];
                }
            }
        }
    }
    let coef: Vec<f64> = (0..6).map(|i| a[i][6] / a[i][i]).collect();
    Some(uvz.iter().map(|&(u, v, z)| z - (coef[0] + coef[1] * u + coef[2] * v + coef[3] * u * u + coef[4] * u * v + coef[5] * v * v)).collect())
}

/// Measures `points` (with their stored `normals`); `markers` are (centre, normal) of listed
/// markers in the same frame. `samples` patch centres are drawn (deterministically).
pub fn measure(points: &[DVec3], normals: &[DVec3], markers: &[(DVec3, DVec3)], samples: usize) -> Realism {
    let grid = Grid::new(points, 1.0);
    let n = points.len();
    let mut rng = Rng::new(0x7265_616c);
    let picks: Vec<usize> = (0..samples.min(n)).map(|_| (rng.uniform() * n as f64) as usize % n.max(1)).collect();

    // Density and nearest neighbours.
    let dens: Vec<(f64, f64)> = picks
        .par_iter()
        .map(|&i| {
            let p = points[i];
            let nb = grid.within(p, 1.0);
            let nn = nb.iter().filter(|&&j| j as usize != i).map(|&j| points[j as usize].distance(p)).fold(f64::INFINITY, f64::min);
            (nb.len() as f64 / std::f64::consts::PI, nn)
        })
        .collect();
    let mut d: Vec<f64> = dens.iter().map(|x| x.0).collect();
    d.sort_by(f64::total_cmp);
    let mut nn: Vec<f64> = dens.iter().map(|x| x.1).filter(|x| x.is_finite()).collect();
    nn.sort_by(f64::total_cmp);
    let density = pct(&d, 0.5);

    // Normal error on smooth patches.
    let errs: Vec<f64> = picks
        .par_iter()
        .filter_map(|&i| {
            let p = points[i];
            let nb: Vec<DVec3> = grid.within(p, 2.0).into_iter().map(|j| points[j as usize]).collect();
            if nb.len() < 12 {
                return None;
            }
            let (_, e, pn) = pca(&nb);
            (e[0] < 0.01 * e[1] && e[0].max(0.0).sqrt() < 0.05).then(|| normals[i].normalize_or_zero().dot(pn).abs().clamp(0.0, 1.0).acos().to_degrees())
        })
        .collect();
    let mut es = errs.clone();
    es.sort_by(f64::total_cmp);

    // Noise structure.
    let noise: Vec<(f64, f64, f64)> = picks
        .par_iter()
        .take(samples / 3 + 1)
        .filter_map(|&i| {
            let p = points[i];
            let idx = grid.within(p, 3.0);
            if idx.len() < 30 {
                return None;
            }
            let nb: Vec<DVec3> = idx.iter().map(|&j| points[j as usize]).collect();
            let (mean, e, pn) = pca(&nb);
            if !(e[0] < 0.01 * e[1]) {
                return None;
            }
            let (u, v) = pn.any_orthonormal_pair();
            let uvz: Vec<(f64, f64, f64)> = nb.iter().map(|q| {
                let d = *q - mean;
                (d.dot(u), d.dot(v), d.dot(pn))
            }).collect();
            let res = quadric_residuals(&uvz)?;
            let var = res.iter().map(|r| r * r).sum::<f64>() / res.len() as f64;
            if var.sqrt() > 0.1 || var <= 0.0 {
                return None;
            }
            let (mut s1, mut c1, mut s2, mut c2) = (0.0, 0, 0.0, 0);
            for a in 0..nb.len() {
                for b in a + 1..nb.len() {
                    let du = uvz[a].0 - uvz[b].0;
                    let dv = uvz[a].1 - uvz[b].1;
                    let lag = (du * du + dv * dv).sqrt();
                    if (0.2..0.5).contains(&lag) {
                        s1 += res[a] * res[b];
                        c1 += 1;
                    } else if (0.5..0.9).contains(&lag) {
                        s2 += res[a] * res[b];
                        c2 += 1;
                    }
                }
            }
            (c1 > 0 && c2 > 0).then(|| (var.sqrt(), s1 / c1 as f64 / var, s2 / c2 as f64 / var))
        })
        .collect();
    let mut stds: Vec<f64> = noise.iter().map(|x| x.0).collect();
    stds.sort_by(f64::total_cmp);
    let m = |f: fn(&(f64, f64, f64)) -> f64| if noise.is_empty() { 0.0 } else { noise.iter().map(f).sum::<f64>() / noise.len() as f64 };

    // Marker rings.
    let rings: Vec<MarkerRings> = markers
        .iter()
        .filter_map(|&(c, mn)| {
            let mn = mn.normalize_or_zero();
            let near = grid.within(c, 12.5);
            let nearest = near.iter().map(|&j| points[j as usize].distance(c)).fold(f64::INFINITY, f64::min);
            if nearest > 1.0 {
                // Not on this cloud (or cut out): only the nearest distance.
                return Some(MarkerRings { centre: c.to_array(), nearest_mm: r3(nearest.min(99.0)), density: Vec::new() });
            }
            let mut count = vec![0usize; RING_EDGES.len() - 1];
            for &j in &near {
                let d = points[j as usize] - c;
                let axial = d.dot(mn);
                if axial.abs() > 1.5 {
                    continue;
                }
                let lateral = (d - mn * axial).length();
                if let Some(k) = RING_EDGES.windows(2).position(|w| lateral >= w[0] && lateral < w[1]) {
                    count[k] += 1;
                }
            }
            let density = RING_EDGES.windows(2).zip(&count).map(|(w, &k)| r3(k as f64 / (std::f64::consts::PI * (w[1] * w[1] - w[0] * w[0])))).collect();
            Some(MarkerRings { centre: c.to_array(), nearest_mm: r3(nearest), density })
        })
        .collect();

    Realism {
        points: n,
        density_per_mm2: r3(density),
        pitch_mm: r3(if density > 0.0 { (1.0 / density).sqrt() } else { 0.0 }),
        nn_p10_mm: r3(pct(&nn, 0.1)),
        nn_p50_mm: r3(pct(&nn, 0.5)),
        nn_p90_mm: r3(pct(&nn, 0.9)),
        normal_samples: es.len(),
        normal_err_median_deg: r3(pct(&es, 0.5)),
        normal_err_p90_deg: r3(pct(&es, 0.9)),
        normal_err_p99_deg: r3(pct(&es, 0.99)),
        normal_err_over_20_share: r3(es.iter().filter(|e| **e > 20.0).count() as f64 / es.len().max(1) as f64),
        noise_patches: noise.len(),
        residual_std_mm: r3(pct(&stds, 0.5)),
        corr_02_05: r3(m(|x| x.1)),
        corr_05_09: r3(m(|x| x.2)),
        markers: rings,
    }
}

/// A one-line summary.
pub fn summary(r: &Realism) -> String {
    format!(
        "{} pts | {:.1} pts/mm² (pitch {:.2}) NN p10/p50/p90 {:.2}/{:.2}/{:.2} | normals median {:.1}° p90 {:.1}° p99 {:.1}° >20° {:.1}% | noise std {:.3} corr {:.2} / {:.2}",
        r.points,
        r.density_per_mm2,
        r.pitch_mm,
        r.nn_p10_mm,
        r.nn_p50_mm,
        r.nn_p90_mm,
        r.normal_err_median_deg,
        r.normal_err_p90_deg,
        r.normal_err_p99_deg,
        100.0 * r.normal_err_over_20_share,
        r.residual_std_mm,
        r.corr_02_05,
        r.corr_05_09
    )
}

/// Marker ring densities as a compact table: one line per marker on the cloud.
pub fn ring_table(r: &Realism) -> String {
    let mut s = String::from("ring edges mm: 0 1.5 3 4.5 5.5 6.5 8 10 12\n");
    let (mut on, mut off) = (0, 0);
    for m in &r.markers {
        if m.density.is_empty() {
            off += 1;
            continue;
        }
        on += 1;
        s += &format!("  ({:7.1} {:7.1} {:7.1}) nearest {:.2}: {}\n", m.centre[0], m.centre[1], m.centre[2], m.nearest_mm, m.density.iter().map(|d| format!("{d:4.1}")).collect::<Vec<_>>().join(" "));
    }
    s += &format!("  {on} markers on the cloud, {off} without a point within 1 mm\n");
    s
}
