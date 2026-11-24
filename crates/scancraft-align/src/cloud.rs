//! Point-set helpers: a nearest-neighbour index, voxel subsampling, transforms.

use glam::Vec3;
use scancraft_core::Transform;
use scancraft_slam::VoxelMap;

use crate::Oriented;

/// Points per kd-tree leaf.
const LEAF: usize = 12;

/// Nearest-neighbour index over 3D points: a balanced kd-tree split at the median of the widest
/// axis, with exact answers.
///
/// It replaces kiddo's immutable tree, which grid-quantised clouds (Revo Scan exports, flat
/// axis-aligned surfaces) break: when many points share a coordinate, kiddo 4.2's construction
/// assumes equal keys end up next to each other after `select_nth_unstable`, picks a split value
/// that does not separate its two halves, and then panics or returns wrong neighbours. Here ties
/// are harmless: points equal to a split value may lie on either side, and the search bounds hold
/// for both.
pub struct Index {
    points: Vec<Vec3>,
    /// Point indices in tree order: the node over `order[lo..hi]` splits at `mid = (lo + hi) / 2`.
    order: Vec<u32>,
    /// Split axis of the node whose split point sits at `order[mid]`.
    axis: Vec<u8>,
}

/// Search state for [`Index::nearest_n`]: the best `k` so far, nearest first.
struct Best {
    k: usize,
    found: Vec<(f32, usize)>,
}

impl Best {
    fn bound(&self) -> f32 {
        if self.found.len() < self.k { f32::INFINITY } else { self.found[self.k - 1].0 }
    }

    fn offer(&mut self, d: f32, i: usize) {
        if d >= self.bound() {
            return;
        }
        let at = self.found.partition_point(|&(e, j)| (e, j) < (d, i));
        self.found.insert(at, (d, i));
        self.found.truncate(self.k);
    }
}

impl Index {
    pub fn new(points: Vec<Vec3>) -> Self {
        let mut order: Vec<u32> = (0..points.len() as u32).collect();
        let mut axis = vec![0u8; points.len()];
        build(&points, &mut order, &mut axis);
        Index { points, order, axis }
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
        self.nearest_n(p, 1).first().copied()
    }

    /// Indices of the points within `radius`.
    pub fn within(&self, p: Vec3, radius: f32) -> Vec<usize> {
        let mut out = Vec::new();
        self.collect_within(0, self.order.len(), p, radius * radius, &mut out);
        out
    }

    /// (squared distance, index) of the `k` nearest points, nearest first.
    pub fn nearest_n(&self, p: Vec3, k: usize) -> Vec<(f32, usize)> {
        if k == 0 || self.points.is_empty() {
            return Vec::new();
        }
        let mut best = Best { k, found: Vec::with_capacity(k + 1) };
        self.search(0, self.order.len(), p, &mut best);
        best.found
    }

    fn search(&self, lo: usize, hi: usize, p: Vec3, best: &mut Best) {
        if hi - lo <= LEAF {
            for &i in &self.order[lo..hi] {
                best.offer(self.points[i as usize].distance_squared(p), i as usize);
            }
            return;
        }
        let mid = (lo + hi) / 2;
        let i = self.order[mid] as usize;
        best.offer(self.points[i].distance_squared(p), i);
        let a = self.axis[mid] as usize;
        let diff = p[a] - self.points[i][a];
        let (near, far) = if diff < 0.0 { ((lo, mid), (mid + 1, hi)) } else { ((mid + 1, hi), (lo, mid)) };
        self.search(near.0, near.1, p, best);
        if diff * diff < best.bound() {
            self.search(far.0, far.1, p, best);
        }
    }

    fn collect_within(&self, lo: usize, hi: usize, p: Vec3, r_sq: f32, out: &mut Vec<usize>) {
        if hi - lo <= LEAF {
            out.extend(self.order[lo..hi].iter().map(|&i| i as usize).filter(|&i| self.points[i].distance_squared(p) <= r_sq));
            return;
        }
        let mid = (lo + hi) / 2;
        let i = self.order[mid] as usize;
        if self.points[i].distance_squared(p) <= r_sq {
            out.push(i);
        }
        let a = self.axis[mid] as usize;
        let diff = p[a] - self.points[i][a];
        if diff <= 0.0 || diff * diff <= r_sq {
            self.collect_within(lo, mid, p, r_sq, out);
        }
        if diff >= 0.0 || diff * diff <= r_sq {
            self.collect_within(mid + 1, hi, p, r_sq, out);
        }
    }
}

/// Arranges `order` (a node's points; `axis` the matching slice) into a kd-tree: the median on
/// the widest axis in the middle, smaller-or-equal before it, greater-or-equal after it. Large
/// halves are built in parallel.
fn build(points: &[Vec3], order: &mut [u32], axis: &mut [u8]) {
    if order.len() <= LEAF {
        return;
    }
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for &i in order.iter() {
        lo = lo.min(points[i as usize]);
        hi = hi.max(points[i as usize]);
    }
    let ext = hi - lo;
    let a = if ext.x >= ext.y && ext.x >= ext.z { 0 } else if ext.y >= ext.z { 1 } else { 2 };
    let mid = order.len() / 2;
    order.select_nth_unstable_by(mid, |&i, &j| points[i as usize][a].total_cmp(&points[j as usize][a]));
    axis[mid] = a as u8;
    let (left, rest) = order.split_at_mut(mid);
    let (axis_left, axis_rest) = axis.split_at_mut(mid);
    if left.len() > 20_000 {
        rayon::join(|| build(points, left, axis_left), || build(points, &mut rest[1..], &mut axis_rest[1..]));
    } else {
        build(points, left, axis_left);
        build(points, &mut rest[1..], &mut axis_rest[1..]);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A grid-quantised scene like a Revo Scan export far from the origin: a flat table (one exact
    /// z for 40 000 points), a wall at one exact x, every point stored three times, and a pile of
    /// 2000 identical points.
    fn quantised_scene() -> Vec<Vec3> {
        let mut pts = Vec::new();
        for i in 0..200 {
            for j in 0..200 {
                pts.push(Vec3::new(420.0 + i as f32 * 0.25, -310.0 + j as f32 * 0.25, 812.5));
            }
        }
        for j in 0..100 {
            for k in 0..100 {
                pts.push(Vec3::new(420.0, -310.0 + j as f32 * 0.5, 812.5 + k as f32 * 0.5));
            }
        }
        let copies = pts.clone();
        pts.extend_from_slice(&copies);
        pts.extend_from_slice(&copies);
        pts.extend(std::iter::repeat_n(Vec3::new(450.0, -300.0, 812.5), 2000));
        pts
    }

    #[test]
    fn index_over_quantised_cloud_builds_and_answers_exactly() {
        let pts = quantised_scene();
        let index = Index::new(pts.clone());
        assert_eq!(index.len(), pts.len());
        let mut rng = Rng::new(5);
        for k in 0..200 {
            // Half the queries close to the table and the wall, where the ties are.
            let mut q = Vec3::new(415.0 + rng.unit() * 60.0, -315.0 + rng.unit() * 60.0, 808.0 + rng.unit() * 60.0);
            if k % 2 == 0 {
                q.z = 811.5 + rng.unit() * 2.0;
            }
            if k % 4 == 1 {
                q.x = 419.0 + rng.unit() * 2.0;
            }
            let mut brute: Vec<f32> = pts.iter().map(|p| p.distance_squared(q)).collect();
            brute.sort_by(f32::total_cmp);
            let (d, i) = index.nearest(q).expect("non-empty");
            assert_eq!(d, pts[i].distance_squared(q));
            assert_eq!(d, brute[0], "nearest");
            let knn = index.nearest_n(q, 7);
            assert_eq!(knn.iter().map(|n| n.0).collect::<Vec<_>>(), brute[..7].to_vec(), "7 nearest");
            let r = 1.3;
            let mut within = index.within(q, r);
            within.sort_unstable();
            let expected: Vec<usize> = (0..pts.len()).filter(|&k| pts[k].distance_squared(q) <= r * r).collect();
            assert_eq!(within, expected);
        }
        // Every copy of a duplicated point is found.
        let pile = index.within(Vec3::new(450.0, -300.0, 812.5), 1e-3);
        assert!(pile.len() >= 2000 + 3, "{} points at the pile", pile.len());
    }

    #[test]
    fn index_handles_empty_and_single_points() {
        let empty = Index::new(Vec::new());
        assert!(empty.nearest(Vec3::ZERO).is_none() && empty.within(Vec3::ZERO, 1.0).is_empty() && empty.nearest_n(Vec3::ZERO, 3).is_empty());
        let one = Index::new(vec![Vec3::new(1.0, 2.0, 3.0)]);
        assert_eq!(one.nearest(Vec3::new(1.0, 2.0, 4.0)), Some((1.0, 0)));
        assert_eq!(one.nearest_n(Vec3::ZERO, 4).len(), 1);
    }
}
