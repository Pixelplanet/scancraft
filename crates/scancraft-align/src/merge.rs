//! Merging aligned passes into one model (`scan-sessions.md` sections 3 and 6).
//!
//! 1. **Weights**: each pass carries a weight by expected accuracy (laser lines > Full-field HD >
//!    VCSEL). Where passes overlap (same small cell, surface facing the same way), contributions
//!    from passes much less accurate than the best one there are dropped, so a precise pass wins
//!    instead of being blurred by a noisy one.
//! 2. **Fusion**: the remaining points are fused into surfels on a regular pitch (merging into a
//!    neighbouring surfel when one is within a pitch, as the tracker does), averaged with weight
//!    x observation count.
//! 3. **Confidence**: a surfel observed only once is kept when its neighbours agree (enough
//!    surface around it) and dropped when isolated, so legitimate edge-of-coverage surface stays.
//! 4. **Statistical outliers**: points whose mean distance to their nearest neighbours is far
//!    above the average are removed.
//!
//! Per-frame noise handling (flying pixels, free-space carving) belongs to the passes themselves
//! and happens before this.

use std::collections::HashMap;

use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Point3D};

use crate::cloud::Index;

/// One pass to merge, already in the project frame: (position, normal, observation count).
pub struct MergeInput {
    pub points: Vec<(Vec3, Vec3, u32)>,
    pub weight: f32,
}

#[derive(Debug, Clone)]
pub struct MergeConfig {
    /// Pitch of the merged surface.
    pub pitch_mm: f32,
    /// In a cell where passes overlap, a pass is used only if its weight is at least this
    /// fraction of the best weight there.
    pub dominance: f32,
    /// Size of those cells, in pitches.
    pub dominance_cell: f32,
    /// A single-observation surfel needs this many neighbours within `support_radius` pitches.
    pub min_support: usize,
    pub support_radius: f32,
    /// Statistical outlier removal: neighbours and threshold in standard deviations; `None`
    /// skips it.
    pub outliers: Option<(usize, f32)>,
}

impl Default for MergeConfig {
    fn default() -> Self {
        MergeConfig { pitch_mm: 0.5, dominance: 0.6, dominance_cell: 4.0, min_support: 6, support_radius: 3.0, outliers: Some((12, 3.0)) }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MergeResult {
    pub points: Vec<Point3D>,
    pub input_points: usize,
    /// Dropped because a more accurate pass covered the same surface.
    pub outweighed: usize,
    /// Single observations without surface around them.
    pub isolated: usize,
    pub outliers: usize,
}

struct Acc {
    p: Vec3,
    n: Vec3,
    w: f32,
    obs: u32,
}

impl Acc {
    fn mean(&self) -> Vec3 {
        self.p / self.w
    }
}

fn normal_bin(n: Vec3) -> i32 {
    let a = n.abs();
    if a.x >= a.y && a.x >= a.z {
        if n.x >= 0.0 { 0 } else { 1 }
    } else if a.y >= a.z {
        if n.y >= 0.0 { 2 } else { 3 }
    } else if n.z >= 0.0 {
        4
    } else {
        5
    }
}

fn cell(p: Vec3, size: f32) -> [i32; 3] {
    let k = (p / size).floor();
    [k.x as i32, k.y as i32, k.z as i32]
}

pub fn merge(inputs: &[MergeInput], config: &MergeConfig) -> MergeResult {
    let pitch = config.pitch_mm.max(0.05);
    let input_points = inputs.iter().map(|i| i.points.len()).sum();

    // 1. The best weight per (cell, facing).
    let dom = pitch * config.dominance_cell;
    let mut best: HashMap<([i32; 3], i32), f32> = HashMap::new();
    for input in inputs {
        for &(p, n, _) in &input.points {
            let e = best.entry((cell(p, dom), normal_bin(n))).or_insert(0.0);
            *e = e.max(input.weight);
        }
    }

    // 2. Fuse, most accurate passes first so they place the surfels.
    let mut order: Vec<usize> = (0..inputs.len()).collect();
    order.sort_by(|&a, &b| inputs[b].weight.total_cmp(&inputs[a].weight));
    let mut cells: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    let mut accs: Vec<Acc> = Vec::new();
    let mut outweighed = 0;
    for &k in &order {
        let input = &inputs[k];
        for &(p, n, obs) in &input.points {
            let top = best.get(&(cell(p, dom), normal_bin(n))).copied().unwrap_or(0.0);
            if input.weight < config.dominance * top {
                outweighed += 1;
                continue;
            }
            let w = input.weight * obs.clamp(1, 8) as f32;
            let [kx, ky, kz] = cell(p, pitch);
            let mut target: Option<(f32, usize)> = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(list) = cells.get(&[kx + dx, ky + dy, kz + dz]) {
                            for &i in list {
                                let a = &accs[i];
                                let d = a.mean().distance_squared(p);
                                if d <= pitch * pitch && a.n.normalize_or_zero().dot(n) > 0.5 && target.is_none_or(|(b, _)| d < b) {
                                    target = Some((d, i));
                                }
                            }
                        }
                    }
                }
            }
            match target {
                Some((_, i)) => {
                    let a = &mut accs[i];
                    a.p += p * w;
                    a.n += n * w;
                    a.w += w;
                    a.obs += obs.max(1);
                }
                None => {
                    cells.entry([kx, ky, kz]).or_default().push(accs.len());
                    accs.push(Acc { p: p * w, n: n * w, w, obs: obs.max(1) });
                }
            }
        }
    }
    let fused: Vec<(Vec3, Vec3, u32)> = accs.iter().map(|a| (a.mean(), a.n.normalize_or_zero(), a.obs)).collect();

    // 3. Single observations need surface support.
    let index = Index::new(fused.iter().map(|f| f.0).collect());
    let radius = config.support_radius * pitch;
    let keep: Vec<bool> = fused.par_iter().map(|&(p, _, obs)| obs > 1 || index.within(p, radius).len() > config.min_support).collect();
    let isolated = keep.iter().filter(|k| !**k).count();
    let mut kept: Vec<(Vec3, Vec3, u32)> = fused.into_iter().zip(keep).filter_map(|(f, k)| k.then_some(f)).collect();

    // 4. Statistical outliers.
    let mut outliers = 0;
    if let Some((k, sigma)) = config.outliers {
        if kept.len() > k + 1 {
            let index = Index::new(kept.iter().map(|f| f.0).collect());
            let mean: Vec<f32> = kept
                .par_iter()
                .map(|&(p, _, _)| {
                    let nn = index.nearest_n(p, k + 1);
                    let d: f32 = nn.iter().skip(1).map(|(d, _)| d.sqrt()).sum();
                    d / (nn.len().max(2) - 1) as f32
                })
                .collect();
            let mu = mean.iter().sum::<f32>() / mean.len() as f32;
            let sd = (mean.iter().map(|m| (m - mu) * (m - mu)).sum::<f32>() / mean.len() as f32).sqrt();
            let limit = mu + sigma * sd;
            let before = kept.len();
            kept = kept.into_iter().zip(mean).filter_map(|(f, m)| (m <= limit).then_some(f)).collect();
            outliers = before - kept.len();
        }
    }

    let points = kept
        .into_iter()
        .map(|(p, n, obs)| {
            let mut pt = Point3D::new(p.to_array()).with_normal(n.to_array()).with_color(ColorRgba::GRAY);
            pt.confidence = obs as f32;
            pt
        })
        .collect();
    MergeResult { points, input_points, outweighed, isolated, outliers }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Rng;
    use crate::synthetic;
    use scancraft_core::Transform;

    fn input(cloud: &crate::PassCloud, weight: f32) -> MergeInput {
        MergeInput { points: cloud.points.iter().map(|&(p, n)| (p, n, 3)).collect(), weight }
    }

    fn mean_error(points: &[Point3D]) -> f32 {
        points.iter().map(|p| synthetic::surface_distance(Vec3::from(p.position))).sum::<f32>() / points.len().max(1) as f32
    }

    #[test]
    fn accurate_pass_wins_and_noise_is_removed() {
        let obj = synthetic::object(0.5);
        let views = synthetic::ring(Vec3::ZERO, 0.6, 0.0, 6.0, 8, 260.0);
        // Same surface: a precise pass (laser-like) and a noisy one (VCSEL-like).
        let precise = synthetic::pass(&obj, &Transform::default(), &views[..5], 0.05, 0.5, 1);
        let noisy = synthetic::pass(&obj, &Transform::default(), &views[2..], 0.6, 0.5, 2);
        let mut noisy_in = input(&noisy, 1.0);
        // Stray points, as flying pixels leave them.
        let mut rng = Rng::new(4);
        for _ in 0..300 {
            let p = Vec3::new(rng.unit() - 0.5, rng.unit() - 0.5, rng.unit() - 0.5) * 220.0;
            if synthetic::surface_distance(p) > 5.0 {
                noisy_in.points.push((p, Vec3::Z, 1));
            }
        }
        let strays = noisy_in.points.len() - noisy.points.len();
        let inputs = [input(&precise, 3.0), noisy_in];
        let t0 = std::time::Instant::now();
        let merged = merge(&inputs, &MergeConfig::default());
        let elapsed = t0.elapsed();
        // An unweighted union for comparison.
        let union = merge(&[input(&precise, 1.0), input(&noisy, 1.0)], &MergeConfig { outliers: None, ..MergeConfig::default() });
        let far = merged.points.iter().filter(|p| synthetic::surface_distance(Vec3::from(p.position)) > 3.0).count();
        eprintln!(
            "merge: {} -> {} points in {elapsed:?}; outweighed {}, isolated {}, outliers {}; mean error {:.3} mm (unweighted {:.3} mm); {far} of {strays} strays left",
            merged.input_points,
            merged.points.len(),
            merged.outweighed,
            merged.isolated,
            merged.outliers,
            mean_error(&merged.points),
            mean_error(&union.points)
        );
        assert!(merged.outweighed > 1000);
        assert!(far <= strays / 20, "{far} strays survived");
        assert!(mean_error(&merged.points) < 0.75 * mean_error(&union.points));
    }
}
