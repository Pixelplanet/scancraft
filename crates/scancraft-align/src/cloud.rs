//! Point-set helpers: a nearest-neighbour index, voxel subsampling, transforms.

use glam::Vec3;
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use scancraft_core::Transform;
use scancraft_slam::VoxelMap;

use crate::Oriented;

/// Nearest-neighbour index over 3D points.
pub struct Index {
    points: Vec<Vec3>,
    tree: Option<ImmutableKdTree<f32, 3>>,
}

impl Index {
    pub fn new(points: Vec<Vec3>) -> Self {
        if points.is_empty() {
            return Index { points, tree: None };
        }
        // kiddo's immutable tree panics when many points share one coordinate on an axis; break
        // exact ties with a deterministic jitter of a few micrometres (see the tracker's target).
        let jittered: Vec<[f32; 3]> = points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let h = (i as u32).wrapping_mul(2654435761);
                let j = |shift: u32| ((h >> shift) & 0xff) as f32 * 4e-6;
                [p.x + j(0), p.y + j(8), p.z + j(16)]
            })
            .collect();
        Index { tree: Some(ImmutableKdTree::new_from_slice(&jittered)), points }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn point(&self, i: usize) -> Vec3 {
        self.points[i]
    }

    /// (squared distance, index) of the nearest point.
    pub fn nearest(&self, p: Vec3) -> Option<(f32, usize)> {
        let nn = self.tree.as_ref()?.nearest_one::<SquaredEuclidean>(&p.to_array());
        Some((nn.distance, nn.item as usize))
    }

    /// Indices of the points within `radius`.
    pub fn within(&self, p: Vec3, radius: f32) -> Vec<usize> {
        match &self.tree {
            Some(t) => t.within_unsorted::<SquaredEuclidean>(&p.to_array(), radius * radius).into_iter().map(|n| n.item as usize).collect(),
            None => Vec::new(),
        }
    }

    /// (squared distance, index) of the `k` nearest points, nearest first.
    pub fn nearest_n(&self, p: Vec3, k: usize) -> Vec<(f32, usize)> {
        match &self.tree {
            Some(t) if k > 0 => t.nearest_n::<SquaredEuclidean>(&p.to_array(), k).into_iter().map(|n| (n.distance, n.item as usize)).collect(),
            _ => Vec::new(),
        }
    }
}

/// One point (mean position, normalised mean normal) per occupied voxel.
pub fn downsample(points: &[Oriented], voxel_mm: f32) -> Vec<Oriented> {
    let map = VoxelMap::from_points(voxel_mm, points.iter().copied().filter(|(_, n)| n.length_squared() > 0.25));
    map.surfels().iter().filter_map(|s| { let n = s.normal.normalize_or_zero(); (n != Vec3::ZERO).then_some((s.position, n)) }).collect()
}

/// Like [`downsample`], coarsening the grid until at most `max_points` remain. Returns the
/// points and the voxel size used.
pub fn downsample_to(points: &[Oriented], voxel_mm: f32, max_points: usize) -> (Vec<Oriented>, f32) {
    let mut voxel = voxel_mm;
    loop {
        let out = downsample(points, voxel);
        if out.len() <= max_points || voxel > 1000.0 {
            return (out, voxel);
        }
        voxel *= (out.len() as f32 / max_points as f32).sqrt().max(1.1);
    }
}

pub fn transform_oriented(points: &[Oriented], t: &Transform) -> Vec<Oriented> {
    let m = t.to_mat4();
    points.iter().map(|&(p, n)| (m.transform_point3(p), (t.rotation * n).normalize_or_zero())).collect()
}

/// Diagonal of the axis-aligned bounds.
pub fn extent(points: &[Oriented]) -> f32 {
    if points.is_empty() {
        return 0.0;
    }
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (p, _) in points {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    (hi - lo).length()
}

pub fn centroid(points: &[Oriented]) -> Vec3 {
    if points.is_empty() {
        return Vec3::ZERO;
    }
    points.iter().map(|(p, _)| *p).sum::<Vec3>() / points.len() as f32
}

/// Small deterministic xorshift generator (reproducible RANSAC and synthetic data).
#[derive(Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform in [0, n).
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n.max(1) as u64) as usize
    }

    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Approximately normal (sum of uniforms), mean 0, standard deviation 1.
    pub fn normal(&mut self) -> f32 {
        (0..6).map(|_| self.unit()).sum::<f32>() * std::f32::consts::SQRT_2 - 6.0 * std::f32::consts::FRAC_1_SQRT_2
    }
}
