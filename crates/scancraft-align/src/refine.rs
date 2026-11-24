//! Refinement of alignments:
//!
//! - [`refine_global`], after every pass is aligned: point-to-plane ICP between each pair of
//!   overlapping passes gives relative-pose measurements, and a pose graph over all passes (the
//!   reference fixed) spreads the residual errors instead of accumulating them;
//! - [`refine_shell`], for one pair: point-to-plane ICP that also uses the two faces of a wall
//!   (an inside scan against an outside scan), pulling them to the wall thickness, which it
//!   estimates as a seventh unknown.

use glam::{DQuat, DVec3, Vec3};
use rayon::prelude::*;
use scancraft_core::Transform;
use scancraft_slam::point_to_plane::{align, PlaneTarget};

use crate::cloud::{downsample, downsample_to, transform_oriented, Index};
use crate::quality::{evaluate_sampled, QualityConfig};
use crate::{Alignment, Oriented, PassCloud};

pub struct RefineInput<'a> {
    pub cloud: &'a PassCloud,
    /// Current alignment: project <- pass.
    pub pose: Transform,
    /// Held in place (the reference pass).
    pub fixed: bool,
}

#[derive(Debug, Clone)]
pub struct RefineConfig {
    pub sample_mm: f32,
    /// Pairs whose overlap (fraction of the smaller pass) is below this are not linked.
    pub min_overlap: f32,
    pub iterations: usize,
    /// Robust pose graph: a link whose residual displacement exceeds this (mm) is down-weighted
    /// (Huber), so one bad pairwise fit cannot pull the others. `None`: plain least squares.
    pub huber_mm: Option<f32>,
    /// Pairwise ICP also pairs the two faces of a wall ([`refine_shell`]'s model), for passes
    /// that see a part from inside and outside.
    pub shell: bool,
}

impl Default for RefineConfig {
    fn default() -> Self {
        RefineConfig { sample_mm: 1.5, min_overlap: 0.05, iterations: 15, huber_mm: None, shell: false }
    }
}

/// One pairwise link of the graph.
#[derive(Debug, Clone, Copy)]
pub struct RefineEdge {
    pub a: usize,
    pub b: usize,
    pub overlap: f32,
    pub rmse_mm: f32,
    /// How far the pair's overlap moved apart from the ICP fit, before and after refinement, mm.
    pub residual_before_mm: f32,
    pub residual_after_mm: f32,
    /// Wall thickness between the pair (with [`RefineConfig::shell`] and enough wall pairs), mm.
    pub thickness_mm: Option<f32>,
}

#[derive(Debug, Clone)]
pub struct RefineResult {
    /// New project <- pass transforms, in input order.
    pub poses: Vec<Transform>,
    pub edges: Vec<RefineEdge>,
}

impl RefineResult {
    pub fn rms_before(&self) -> f32 {
        rms(self.edges.iter().map(|e| e.residual_before_mm))
    }
    pub fn rms_after(&self) -> f32 {
        rms(self.edges.iter().map(|e| e.residual_after_mm))
    }
}

fn rms(v: impl Iterator<Item = f32>) -> f32 {
    let (s, n) = v.fold((0.0f32, 0usize), |(s, n), x| (s + x * x, n + 1));
    if n == 0 { 0.0 } else { (s / n as f32).sqrt() }
}

#[derive(Clone, Copy, Debug)]
struct Pose {
    r: DQuat,
    t: DVec3,
}

impl Pose {
    fn of(t: &Transform) -> Self {
        Pose { r: t.rotation.as_dquat().normalize(), t: t.translation.as_dvec3() }
    }
    fn to_transform(self) -> Transform {
        crate::rigid(self.r.as_quat(), self.t.as_vec3())
    }
    fn mul(self, o: Pose) -> Pose {
        Pose { r: (self.r * o.r).normalize(), t: self.r * o.t + self.t }
    }
    fn inv(self) -> Pose {
        let r = self.r.inverse();
        Pose { r, t: -(r * self.t) }
    }
    fn apply(self, p: DVec3) -> DVec3 {
        self.r * p + self.t
    }
    /// exp(delta) ∘ self, delta = (rotation vector, translation).
    fn perturbed(self, d: &[f64]) -> Pose {
        let w = DVec3::new(d[0], d[1], d[2]);
        let dr = if w.length() > 1e-15 { DQuat::from_scaled_axis(w) } else { DQuat::IDENTITY };
        Pose { r: (dr * self.r).normalize(), t: dr * self.t + DVec3::new(d[3], d[4], d[5]) }
    }
}

/// A relative-pose measurement: pass `a` in the frame of pass `b` (`b <- a`).
struct Measurement {
    a: usize,
    b: usize,
    z: Pose,
    weight: f64,
    /// Overlap centroid in `a`'s frame and its radius: residuals are measured there.
    centre: DVec3,
    radius: f64,
    overlap: f32,
    rmse: f32,
    thickness: Option<f32>,
}

/// Refines the poses of all passes jointly.
pub fn refine_global(inputs: &[RefineInput], config: &RefineConfig) -> RefineResult {
    let n = inputs.len();
    let poses: Vec<Pose> = inputs.iter().map(|i| Pose::of(&i.pose)).collect();
    if n < 2 {
        return RefineResult { poses: inputs.iter().map(|i| i.pose).collect(), edges: Vec::new() };
    }
    // Samples in the project frame at the current poses.
    let samples: Vec<Vec<Oriented>> = inputs.par_iter().map(|i| transform_oriented(&downsample_to(&i.cloud.points, config.sample_mm, 40_000).0, &i.pose)).collect();
    let planes: Vec<PlaneTarget> = samples.par_iter().map(|s| PlaneTarget::new(s.iter().map(|p| p.0).collect(), s.iter().map(|p| p.1).collect())).collect();
    let indices: Vec<Index> = samples.par_iter().map(|s| Index::new(s.iter().map(|p| p.0).collect())).collect();

    // Pairwise measurements.
    let pairs: Vec<(usize, usize)> = (0..n).flat_map(|a| (a + 1..n).map(move |b| (a, b))).collect();
    let gate = 3.0 * config.sample_mm;
    let measurements: Vec<Measurement> = pairs
        .par_iter()
        .filter_map(|&(a, b)| {
            // Overlap at the current poses: the smaller pass moves onto the larger.
            let (src, dst) = if samples[a].len() <= samples[b].len() { (a, b) } else { (b, a) };
            let close: Vec<Vec3> = samples[src].iter().filter(|(p, _)| indices[dst].nearest(*p).is_some_and(|(d, _)| d <= gate * gate)).map(|(p, _)| *p).collect();
            let overlap = close.len() as f32 / samples[src].len().max(1) as f32;
            if overlap < config.min_overlap || close.len() < 200 {
                return None;
            }
            let gates = [gate, 2.0 * config.sample_mm, 1.2 * config.sample_mm];
            let (fit, inliers, rmse, thickness) = if config.shell {
                let icp = shell_icp(&samples[src], &samples[dst], &indices[dst], Transform::default(), &gates, &ShellConfig::default());
                (icp.pose, icp.same + icp.wall, icp.rmse(), (icp.wall >= 100).then_some(icp.thickness))
            } else {
                let r = align(&samples[src], &planes[dst], Transform::default(), &gates, 10);
                (r.pose, r.inliers, r.rmse_mm, None)
            };
            if inliers < 200 || rmse > 2.0 {
                return None;
            }
            // The fit corrects src in the project frame: project <- src' = delta * T_src.
            // Measurement of src in dst's frame: T_dst^-1 * delta * T_src.
            let delta = Pose::of(&fit);
            let z = poses[dst].inv().mul(delta).mul(poses[src]);
            let c = close.iter().copied().sum::<Vec3>() / close.len() as f32;
            let radius = (close.iter().map(|p| (*p - c).length_squared()).sum::<f32>() / close.len() as f32).sqrt().max(10.0);
            let centre = poses[src].inv().apply(c.as_dvec3());
            Some(Measurement { a: src, b: dst, z, weight: inliers as f64, centre, radius: radius as f64, overlap, rmse, thickness })
        })
        .collect();

    let residual = |m: &Measurement, p: &[Pose]| -> [f64; 6] {
        // Where a's overlap lands via the graph vs. via the measurement, both in b's frame.
        let predicted = p[m.b].inv().mul(p[m.a]);
        let e = m.z.inv().mul(predicted);
        let w = e.r.to_scaled_axis() * m.radius;
        let d = e.apply(m.centre) - m.centre;
        let s = m.weight.sqrt();
        [w.x * s, w.y * s, w.z * s, d.x * s, d.y * s, d.z * s]
    };
    let displacement = |m: &Measurement, p: &[Pose]| -> f32 {
        let predicted = p[m.b].inv().mul(p[m.a]);
        let e = m.z.inv().mul(predicted);
        // Largest displacement over the overlap: centre shift plus rotation over the radius.
        ((e.apply(m.centre) - m.centre).length() + e.r.to_scaled_axis().length() * m.radius) as f32
    };

    let free: Vec<usize> = (0..n).filter(|&i| !inputs[i].fixed).collect();
    let mut current = poses.clone();
    if !free.is_empty() && !measurements.is_empty() {
        let dim = 6 * free.len();
        let slot = |i: usize| free.iter().position(|&f| f == i);
        let mut lambda = 1e-6;
        for _ in 0..config.iterations {
            // Huber weights from the current residuals (fixed within the step).
            let scale: Vec<f64> = measurements
                .iter()
                .map(|m| match config.huber_mm {
                    Some(k) => {
                        let d = displacement(m, &current) as f64;
                        if d <= k as f64 { 1.0 } else { (k as f64 / d).sqrt() }
                    }
                    None => 1.0,
                })
                .collect();
            let weighted = |k: usize, p: &[Pose]| residual(&measurements[k], p).map(|x| x * scale[k]);
            let mut h = vec![0.0f64; dim * dim];
            let mut g = vec![0.0f64; dim];
            let mut cost = 0.0;
            for (k, m) in measurements.iter().enumerate() {
                let r0 = weighted(k, &current);
                cost += r0.iter().map(|x| x * x).sum::<f64>();
                // Numeric Jacobian with respect to the two endpoints.
                let mut cols: Vec<(usize, [f64; 6])> = Vec::new();
                for node in [m.a, m.b] {
                    let Some(s) = slot(node) else { continue };
                    for j in 0..6 {
                        let mut d = [0.0; 6];
                        let eps = if j < 3 { 1e-6 } else { 1e-4 };
                        d[j] = eps;
                        let mut p = current.clone();
                        p[node] = current[node].perturbed(&d);
                        let r1 = weighted(k, &p);
                        let mut col = [0.0; 6];
                        for q in 0..6 {
                            col[q] = (r1[q] - r0[q]) / eps;
                        }
                        cols.push((6 * s + j, col));
                    }
                }
                for &(ia, ca) in &cols {
                    g[ia] += (0..6).map(|q| ca[q] * r0[q]).sum::<f64>();
                    for &(ib, cb) in &cols {
                        h[ia * dim + ib] += (0..6).map(|q| ca[q] * cb[q]).sum::<f64>();
                    }
                }
            }
            let trace: f64 = (0..dim).map(|i| h[i * dim + i]).sum::<f64>() / dim as f64;
            for i in 0..dim {
                h[i * dim + i] += lambda * trace + 1e-9;
            }
            let Some(step) = solve(h, g.iter().map(|x| -x).collect(), dim) else { break };
            let mut next = current.clone();
            for (s, &i) in free.iter().enumerate() {
                next[i] = current[i].perturbed(&step[6 * s..6 * s + 6]);
            }
            let next_cost: f64 = (0..measurements.len()).map(|k| weighted(k, &next).iter().map(|x| x * x).sum::<f64>()).sum();
            if next_cost < cost {
                current = next;
                lambda = (lambda * 0.3).max(1e-9);
                if cost - next_cost < 1e-9 * cost.max(1.0) {
                    break;
                }
            } else {
                lambda *= 10.0;
                if lambda > 1e3 {
                    break;
                }
            }
        }
    }
    let edges = measurements
        .iter()
        .map(|m| RefineEdge {
            a: m.a,
            b: m.b,
            overlap: m.overlap,
            rmse_mm: m.rmse,
            residual_before_mm: displacement(m, &poses),
            residual_after_mm: displacement(m, &current),
            thickness_mm: m.thickness,
        })
        .collect();
    RefineResult { poses: current.into_iter().map(Pose::to_transform).collect(), edges }
}

/// Settings for [`refine_shell`].
#[derive(Debug, Clone, Copy)]
pub struct ShellConfig {
    /// Grid both passes are sampled on, mm.
    pub sample_mm: f32,
    /// Upper bound on the moving pass's samples (the grid is coarsened to stay below it).
    pub max_points: usize,
    /// Largest distance along the target normal between the two faces of a wall pair, mm.
    pub shell_range_mm: f32,
    /// Residuals up to this get full weight; larger ones are down-weighted (Huber), mm.
    pub huber_mm: f32,
    /// Gauss-Newton steps per correspondence gate.
    pub iterations: usize,
    /// Starting wall thickness, mm. `None`: the median gap of the wall pairs at the guess.
    pub thickness_mm: Option<f32>,
    /// Hold the thickness at `thickness_mm` (a known wall) instead of estimating it.
    pub fixed_thickness: bool,
}

impl Default for ShellConfig {
    fn default() -> Self {
        ShellConfig { sample_mm: 1.0, max_points: 60_000, shell_range_mm: 10.0, huber_mm: 0.5, iterations: 10, thickness_mm: None, fixed_thickness: false }
    }
}

/// The result of [`refine_shell`].
#[derive(Debug, Clone, Copy)]
pub struct ShellFit {
    /// target <- source, with its quality.
    pub alignment: Alignment,
    /// Estimated wall thickness, mm (the start value when too few wall pairs were found).
    pub thickness_mm: f32,
    /// Pairs of surfaces facing the same way, and pairs of the two faces of a wall, in the last
    /// step.
    pub same_pairs: usize,
    pub wall_pairs: usize,
    /// RMS residual of the same-facing pairs and of the wall pairs about the thickness, mm.
    pub same_rmse_mm: f32,
    pub wall_rmse_mm: f32,
}

/// Fine alignment from `guess` (target <- source) of passes that may see opposite faces of one
/// wall. Surfaces facing the same way are pulled together (point-to-plane); a source surface
/// facing against a target surface within [`ShellConfig::shell_range_mm`] is pulled to lie the
/// wall thickness `t` behind it, with `t` estimated with the pose (7 unknowns). Huber weights.
pub fn refine_shell(source: &PassCloud, target: &PassCloud, guess: Transform, config: &ShellConfig) -> ShellFit {
    let (src, fine) = downsample_to(&source.points, config.sample_mm, config.max_points);
    let tgt = downsample(&target.points, fine);
    let index = Index::new(tgt.iter().map(|p| p.0).collect());
    let icp = shell_icp(&src, &tgt, &index, guess, &[4.0 * fine, 2.5 * fine, 1.5 * fine], config);
    let quality = evaluate_sampled(&src, &tgt, &index, source, target, &icp.pose, QualityConfig::for_sample(fine.max(1.0)));
    ShellFit {
        alignment: Alignment { transform: icp.pose, quality },
        thickness_mm: icp.thickness,
        same_pairs: icp.same,
        wall_pairs: icp.wall,
        same_rmse_mm: icp.same_rmse,
        wall_rmse_mm: icp.wall_rmse,
    }
}

/// Result of [`shell_icp`].
struct ShellIcp {
    pose: Transform,
    thickness: f32,
    same: usize,
    wall: usize,
    same_rmse: f32,
    wall_rmse: f32,
}

impl ShellIcp {
    /// RMS residual over both kinds of pairs.
    fn rmse(&self) -> f32 {
        let sq = |n: usize, rmse: f32| if n > 0 { n as f32 * rmse * rmse } else { 0.0 };
        let n = self.same + self.wall;
        if n == 0 { f32::INFINITY } else { ((sq(self.same, self.same_rmse) + sq(self.wall, self.wall_rmse)) / n as f32).sqrt() }
    }
}

/// One correspondence of [`shell_icp`]: moved source point, target point and normal, and
/// whether it pairs the two faces of a wall.
type ShellPair = (DVec3, DVec3, DVec3, bool);

/// Correspondences of the moved `src` (rotation, translation) with `tgt`: the nearest target
/// point when it faces the same way within `gate`, otherwise the opposite-facing target point
/// within `gate` sideways and `range` along its normal.
fn shell_pairs(src: &[Oriented], tgt: &[Oriented], index: &Index, rot: DQuat, trans: DVec3, gate: f32, range: f32) -> Vec<ShellPair> {
    let gate_sq = gate * gate;
    src.par_iter()
        .filter_map(|&(p, n)| {
            let pw = rot * p.as_dvec3() + trans;
            let (pf, nw) = (pw.as_vec3(), (rot * n.as_dvec3()).as_vec3());
            let (d, i) = index.nearest(pf)?;
            let (q, nt) = tgt[i];
            if d <= gate_sq && nw.dot(nt) > 0.5 {
                return Some((pw, q.as_dvec3(), nt.as_dvec3(), false));
            }
            if d > gate_sq + range * range {
                return None;
            }
            index
                .nearest_n(pf, 8)
                .into_iter()
                .filter_map(|(_, j)| {
                    let (q, nt) = tgt[j];
                    if nw.dot(nt) >= -0.5 {
                        return None;
                    }
                    let v = pf - q;
                    let offset = nt.dot(v);
                    let side = (v - nt * offset).length();
                    (offset.abs() <= range && side <= gate).then_some((side, q, nt))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, q, nt)| (pw, q.as_dvec3(), nt.as_dvec3(), true))
        })
        .collect()
}

/// Gauss-Newton over (rotation, translation, wall thickness) on sampled surfaces (`index` over
/// `tgt`), one stage per correspondence gate.
fn shell_icp(src: &[Oriented], tgt: &[Oriented], index: &Index, guess: Transform, gates: &[f32], config: &ShellConfig) -> ShellIcp {
    let range = config.shell_range_mm;
    let mut rot = guess.rotation.as_dquat().normalize();
    let mut trans = guess.translation.as_dvec3();
    let mut t = match config.thickness_mm {
        Some(t) => t as f64,
        None => {
            // Median gap of the wall pairs at the guess.
            let mut gaps: Vec<f64> = shell_pairs(src, tgt, index, rot, trans, gates.first().copied().unwrap_or(3.0), range).iter().filter(|p| p.3).map(|&(pw, q, n, _)| -n.dot(pw - q)).collect();
            gaps.sort_by(f64::total_cmp);
            gaps.get(gaps.len() / 2).copied().unwrap_or(0.0).clamp(0.0, range as f64)
        }
    };
    for &gate in gates {
        let huber = (gate as f64 * 0.25).max(config.huber_mm as f64);
        for _ in 0..config.iterations {
            let pairs = shell_pairs(src, tgt, index, rot, trans, gate, range);
            if pairs.len() < 12 {
                break;
            }
            let estimate = !config.fixed_thickness && pairs.iter().filter(|p| p.3).count() >= 30;
            let system = ShellSystem::build(&pairs, t, huber, estimate);
            let c = system.centre;
            let Some(x) = solve(system.h, system.g, 7) else { break };
            let omega = DVec3::new(x[0], x[1], x[2]);
            let dt = DVec3::new(x[3], x[4], x[5]);
            let d_rot = if omega.length() > 1e-12 { DQuat::from_scaled_axis(omega) } else { DQuat::IDENTITY };
            // The step rotates about the pairs' centroid: p' = dR (p - c) + c + dt.
            rot = (d_rot * rot).normalize();
            trans = d_rot * (trans - c) + c + dt;
            t = (t + x[6]).clamp(0.0, range as f64);
            if omega.length() < 1e-6 && dt.length() < 1e-4 && x[6].abs() < 1e-4 {
                break;
            }
        }
    }
    // Statistics at the finest gate.
    let pairs = shell_pairs(src, tgt, index, rot, trans, gates.last().copied().unwrap_or(1.0), range);
    let (mut same, mut wall, mut same_sq, mut wall_sq) = (0usize, 0usize, 0.0f64, 0.0f64);
    for &(pw, q, n, is_wall) in &pairs {
        let r = n.dot(pw - q);
        if is_wall {
            wall += 1;
            wall_sq += (r + t).powi(2);
        } else {
            same += 1;
            same_sq += r * r;
        }
    }
    let rms = |sq: f64, n: usize| if n > 0 { (sq / n as f64).sqrt() as f32 } else { f32::INFINITY };
    ShellIcp {
        pose: crate::rigid(rot.as_quat(), trans.as_vec3()),
        thickness: t as f32,
        same,
        wall,
        same_rmse: rms(same_sq, same),
        wall_rmse: rms(wall_sq, wall),
    }
}

/// The damped Gauss-Newton system of one [`shell_icp`] step over (rotation, translation, wall
/// thickness).
struct ShellSystem {
    /// 7 x 7, row major.
    h: Vec<f64>,
    g: Vec<f64>,
    /// The rotation is linearised about this point (the centroid of the moved source points), so
    /// the step does not depend on where the frame's origin is.
    centre: DVec3,
}

impl ShellSystem {
    /// Point-to-plane rows for `pairs` (Huber weights at `huber`); wall pairs are offset by the
    /// thickness `t`, which is a free unknown only with `estimate`.
    fn build(pairs: &[ShellPair], t: f64, huber: f64, estimate: bool) -> Self {
        let centre = pairs.iter().map(|p| p.0).sum::<DVec3>() / pairs.len().max(1) as f64;
        let mut h = vec![0.0f64; 49];
        let mut g = vec![0.0f64; 7];
        for &(pw, q, n, wall) in pairs {
            let r = n.dot(pw - q) + if wall { t } else { 0.0 };
            let w = if r.abs() <= huber { 1.0 } else { huber / r.abs() };
            let c = (pw - centre).cross(n);
            let j = [c.x, c.y, c.z, n.x, n.y, n.z, if wall && estimate { 1.0 } else { 0.0 }];
            for a in 0..7 {
                g[a] -= w * j[a] * r;
                for b in 0..7 {
                    h[a * 7 + b] += w * j[a] * j[b];
                }
            }
        }
        // Light damping, per block so that the rotation terms (which grow with the size of the
        // overlap) do not hold back the translation; an unused thickness stays put.
        let block = |range: std::ops::Range<usize>, h: &[f64]| range.map(|a| h[a * 7 + a]).sum::<f64>() / 3.0;
        let (rot_damp, trans_damp) = (1e-6 * block(0..3, &h), 1e-6 * block(3..6, &h));
        for a in 0..6 {
            h[a * 7 + a] += (if a < 3 { rot_damp } else { trans_damp }) + 1e-9;
        }
        h[48] += 1e-6 * h[48] + 1e-9 + if estimate { 0.0 } else { 1.0 };
        ShellSystem { h, g, centre }
    }
}

/// Gaussian elimination with partial pivoting on a dense `dim` x `dim` system.
fn solve(mut a: Vec<f64>, mut b: Vec<f64>, dim: usize) -> Option<Vec<f64>> {
    for col in 0..dim {
        let pivot = (col..dim).max_by(|&i, &j| a[i * dim + col].abs().total_cmp(&a[j * dim + col].abs()))?;
        if a[pivot * dim + col].abs() < 1e-14 {
            return None;
        }
        if pivot != col {
            for k in 0..dim {
                a.swap(col * dim + k, pivot * dim + k);
            }
            b.swap(col, pivot);
        }
        for row in col + 1..dim {
            let f = a[row * dim + col] / a[col * dim + col];
            if f == 0.0 {
                continue;
            }
            for k in col..dim {
                a[row * dim + k] -= f * a[col * dim + k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = vec![0.0; dim];
    for row in (0..dim).rev() {
        let s: f64 = (row + 1..dim).map(|k| a[row * dim + k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row * dim + row];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Rng;
    use crate::compose;
    use crate::synthetic;
    use glam::Quat;

    #[test]
    fn spreads_errors_over_a_ring_of_passes() {
        let obj = synthetic::object(1.0);
        // Four passes, each a quarter-plus of the way round, overlapping their neighbours.
        let mut clouds = Vec::new();
        let mut truths = Vec::new();
        for k in 0..4 {
            let pose = crate::rigid(Quat::from_rotation_z(0.9 * k as f32) * Quat::from_rotation_x(0.2 * k as f32), Vec3::new(30.0 * k as f32, -20.0 * k as f32, 5.0 * k as f32));
            let centre = pose.transform_point3(Vec3::ZERO);
            let a0 = 1.6 * k as f32;
            clouds.push(synthetic::pass(&obj, &pose, &synthetic::ring(centre, 0.55, a0, a0 + 2.4, 6, 260.0), 0.1, 0.8, 20 + k as u64));
            truths.push(pose.inverse()); // project (= object) <- pass
        }
        // Start from the truth disturbed by ~1 degree and ~2 mm, as chained pairwise
        // alignments would leave it.
        let mut rng = Rng::new(42);
        let start: Vec<Transform> = truths
            .iter()
            .enumerate()
            .map(|(k, t)| {
                if k == 0 {
                    return *t;
                }
                let w = Vec3::new(rng.normal(), rng.normal(), rng.normal()).normalize() * 0.018;
                let d = crate::rigid(Quat::from_scaled_axis(w), Vec3::new(rng.normal(), rng.normal(), rng.normal()) * 1.2);
                compose(&d, t)
            })
            .collect();
        let probe: Vec<Vec3> = obj.iter().step_by(40).map(|p| p.0).collect();
        let err = |poses: &[Transform]| -> f32 {
            (1..4).map(|k| crate::displacement_on(&compose(&poses[k], &truths[k].inverse()), &Transform::default(), &probe)).fold(0.0, f32::max)
        };
        let before = err(&start);
        let inputs: Vec<RefineInput> = clouds.iter().zip(&start).enumerate().map(|(k, (c, p))| RefineInput { cloud: c, pose: *p, fixed: k == 0 }).collect();
        for (name, config) in [("plain", RefineConfig::default()), ("huber", RefineConfig { huber_mm: Some(1.0), ..Default::default() })] {
            let t0 = std::time::Instant::now();
            let result = refine_global(&inputs, &config);
            let after = err(&result.poses);
            eprintln!("refine {name}: {} edges, max object displacement {before:.2} -> {after:.3} mm, edge rms {:.2} -> {:.3} mm, {:?}", result.edges.len(), result.rms_before(), result.rms_after(), t0.elapsed());
            assert!(result.edges.len() >= 4, "{name}: neighbours are linked: {:?}", result.edges);
            assert!(after < 0.5, "{name}: after refinement {after:.3} mm (before {before:.2})");
            assert_eq!(result.poses[0], start[0], "the reference stays");
        }
    }

    /// The truth for the cover's first inside pass (outside <- inside) tilted by `degrees` about
    /// an axis through the middle of the part.
    fn tilted(truth: &Transform, degrees: f32) -> Transform {
        let centre = Vec3::new(0.0, 0.0, 30.0);
        let r = Quat::from_axis_angle(Vec3::new(1.0, 0.6, 0.0).normalize(), degrees.to_radians());
        compose(&crate::rigid(r, centre - r * centre), truth)
    }

    #[test]
    fn shell_refinement_finds_the_wall_and_removes_a_tilt() {
        const WALL: f32 = 3.0;
        let (passes, poses) = synthetic::cover_passes(WALL);
        let truth = poses[1].inverse();
        let points: Vec<Vec3> = passes[1].points.iter().map(|p| p.0).collect();
        let error = |t: &Transform| (crate::rotation_angle(t.rotation, truth.rotation).to_degrees(), crate::displacement_on(t, &truth, &points));
        let guess = tilted(&truth, 1.0);
        let t0 = std::time::Instant::now();
        let fit = refine_shell(&passes[1], &passes[0], guess, &ShellConfig::default());
        let (rot, disp) = error(&fit.alignment.transform);
        eprintln!(
            "shell refine: start {:?}, end {rot:.3} deg / {disp:.3} mm, wall {:.3} mm, {} same / {} wall pairs, rms {:.3} / {:.3} mm, {:?}, {:?}",
            error(&guess),
            fit.thickness_mm,
            fit.same_pairs,
            fit.wall_pairs,
            fit.same_rmse_mm,
            fit.wall_rmse_mm,
            t0.elapsed(),
            fit.alignment.quality
        );
        // Plain point-to-plane ICP has only the rim to hold on to.
        let plain = crate::global::refine_fine(&passes[1], &passes[0], guess, 1.0);
        eprintln!("plain refine: {:?}", error(&plain.transform));
        assert!((fit.thickness_mm - WALL).abs() < 0.2, "wall {:.3} mm", fit.thickness_mm);
        assert!(rot < 0.15 && disp < 0.5, "{rot:.3} deg, {disp:.3} mm");
        assert!(fit.wall_pairs > 1000 && fit.wall_rmse_mm < 0.3, "{fit:?}");
        assert_eq!(fit.alignment.quality.verdict(), crate::Verdict::Good, "{:?}", fit.alignment.quality);
        // A known wall can be held fixed: a wrong one leaves a visible offset in the residuals.
        let held = refine_shell(&passes[1], &passes[0], guess, &ShellConfig { thickness_mm: Some(4.0), fixed_thickness: true, ..Default::default() });
        assert_eq!(held.thickness_mm, 4.0);
        assert!(held.wall_rmse_mm > fit.wall_rmse_mm, "{held:?}");
    }

    /// Real scans sit far from their frame's origin (the scanner's): the fit must not depend on
    /// where the origin is.
    #[test]
    fn shell_refinement_does_not_depend_on_the_frame_origin() {
        const WALL: f32 = 3.0;
        let (passes, poses) = synthetic::cover_passes(WALL);
        let truth = poses[1].inverse();
        let guess = tilted(&truth, 1.0);
        let points: Vec<Vec3> = passes[1].points.iter().map(|p| p.0).collect();
        let near = refine_shell(&passes[1], &passes[0], guess, &ShellConfig::default());
        // The same scene 900 mm away from the origin.
        let shift = crate::rigid(Quat::IDENTITY, Vec3::new(250.0, -400.0, 900.0));
        let far_passes = [crate::quality::moved(&passes[0], &shift), crate::quality::moved(&passes[1], &shift)];
        let conj = |t: &Transform| compose(&compose(&shift, t), &shift.inverse());
        let far = refine_shell(&far_passes[1], &far_passes[0], conj(&guess), &ShellConfig::default());
        let far_back = compose(&compose(&shift.inverse(), &far.alignment.transform), &shift);
        let error = |t: &Transform| crate::displacement_on(t, &truth, &points);
        eprintln!("origin: near {:.3} mm, far {:.3} mm, wall {:.3} / {:.3} mm", error(&near.alignment.transform), error(&far_back), near.thickness_mm, far.thickness_mm);
        assert!(error(&far_back) < 0.5, "far from the origin: {:.3} mm", error(&far_back));
        assert!(crate::displacement_on(&far_back, &near.alignment.transform, &points) < 0.05, "the fit moved with the origin");
        assert!((far.thickness_mm - near.thickness_mm).abs() < 0.02);
    }

    #[test]
    fn global_refinement_links_inside_and_outside_with_shell_pairs() {
        const WALL: f32 = 3.0;
        let (passes, poses) = synthetic::cover_passes(WALL);
        let truth = poses[1].inverse();
        let inputs = [RefineInput { cloud: &passes[0], pose: Transform::default(), fixed: true }, RefineInput { cloud: &passes[1], pose: tilted(&truth, 0.7), fixed: false }];
        let points: Vec<Vec3> = passes[1].points.iter().map(|p| p.0).collect();
        let result = refine_global(&inputs, &RefineConfig { sample_mm: 1.0, shell: true, ..Default::default() });
        eprintln!("shell edges {:?}", result.edges);
        let disp = crate::displacement_on(&result.poses[1], &truth, &points);
        assert_eq!(result.edges.len(), 1);
        assert!(result.edges[0].thickness_mm.is_some_and(|t| (t - WALL).abs() < 0.3), "{:?}", result.edges);
        assert!(disp < 0.6, "{disp:.3} mm");
    }
}
