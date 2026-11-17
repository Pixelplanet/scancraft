//! Global refinement after every pass is aligned: point-to-plane ICP between each pair of
//! overlapping passes gives relative-pose measurements, and a pose graph over all passes
//! (the reference fixed) spreads the residual errors instead of accumulating them.

use glam::{DQuat, DVec3, Vec3};
use rayon::prelude::*;
use scancraft_core::Transform;
use scancraft_slam::point_to_plane::{align, PlaneTarget};

use crate::cloud::{downsample_to, transform_oriented, Index};
use crate::{Oriented, PassCloud};

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
}

impl Default for RefineConfig {
    fn default() -> Self {
        RefineConfig { sample_mm: 1.5, min_overlap: 0.05, iterations: 15 }
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
            let r = align(&samples[src], &planes[dst], Transform::default(), &[gate, 2.0 * config.sample_mm, 1.2 * config.sample_mm], 10);
            if r.inliers < 200 || r.rmse_mm > 2.0 {
                return None;
            }
            // r.pose corrects src in the project frame: project <- src' = delta * T_src.
            // Measurement of src in dst's frame: T_dst^-1 * delta * T_src.
            let delta = Pose::of(&r.pose);
            let z = poses[dst].inv().mul(delta).mul(poses[src]);
            let c = close.iter().copied().sum::<Vec3>() / close.len() as f32;
            let radius = (close.iter().map(|p| (*p - c).length_squared()).sum::<f32>() / close.len() as f32).sqrt().max(10.0);
            let centre = poses[src].inv().apply(c.as_dvec3());
            Some(Measurement { a: src, b: dst, z, weight: r.inliers as f64, centre, radius: radius as f64, overlap, rmse: r.rmse_mm })
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
            let mut h = vec![0.0f64; dim * dim];
            let mut g = vec![0.0f64; dim];
            let mut cost = 0.0;
            for m in &measurements {
                let r0 = residual(m, &current);
                cost += r0.iter().map(|x| x * x).sum::<f64>();
                // Numeric Jacobian with respect to the two endpoints.
                let mut cols: Vec<(usize, [f64; 6])> = Vec::new();
                for node in [m.a, m.b] {
                    let Some(s) = slot(node) else { continue };
                    for k in 0..6 {
                        let mut d = [0.0; 6];
                        let eps = if k < 3 { 1e-6 } else { 1e-4 };
                        d[k] = eps;
                        let mut p = current.clone();
                        p[node] = current[node].perturbed(&d);
                        let r1 = residual(m, &p);
                        let mut col = [0.0; 6];
                        for q in 0..6 {
                            col[q] = (r1[q] - r0[q]) / eps;
                        }
                        cols.push((6 * s + k, col));
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
            let next_cost: f64 = measurements.iter().map(|m| residual(m, &next).iter().map(|x| x * x).sum::<f64>()).sum();
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
        .map(|m| RefineEdge { a: m.a, b: m.b, overlap: m.overlap, rmse_mm: m.rmse, residual_before_mm: displacement(m, &poses), residual_after_mm: displacement(m, &current) })
        .collect();
    RefineResult { poses: current.into_iter().map(Pose::to_transform).collect(), edges }
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
        let t0 = std::time::Instant::now();
        let result = refine_global(&inputs, &RefineConfig::default());
        let after = err(&result.poses);
        eprintln!("refine: {} edges, max object displacement {before:.2} -> {after:.3} mm, edge rms {:.2} -> {:.3} mm, {:?}", result.edges.len(), result.rms_before(), result.rms_after(), t0.elapsed());
        assert!(result.edges.len() >= 4, "neighbours are linked: {:?}", result.edges);
        assert!(after < 0.5, "after refinement {after:.3} mm (before {before:.2})");
        assert_eq!(result.poses[0], start[0], "the reference stays");
    }
}
