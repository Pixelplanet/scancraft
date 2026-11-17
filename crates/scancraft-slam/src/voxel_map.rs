//! Sparse voxel hash of oriented points: averages every observation that falls in a voxel.

use glam::Vec3;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy)]
pub struct Surfel {
    pub position: Vec3,
    pub normal: Vec3,
    pub count: u32,
}

/// Points binned on a regular grid; each occupied voxel keeps the mean position and normal.
#[derive(Debug, Clone)]
pub struct VoxelMap {
    voxel_mm: f32,
    cells: HashMap<[i32; 3], usize>,
    surfels: Vec<Surfel>,
    /// Per surfel: how often a later view saw through it (see [`carve`](Self::carve)). Shorter
    /// than `surfels` when the newest surfels have never been carved at.
    misses: Vec<u32>,
}

impl VoxelMap {
    pub fn new(voxel_mm: f32) -> Self {
        Self { voxel_mm: voxel_mm.max(1e-3), cells: HashMap::new(), surfels: Vec::new(), misses: Vec::new() }
    }

    pub fn voxel_mm(&self) -> f32 {
        self.voxel_mm
    }

    pub fn len(&self) -> usize {
        self.surfels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.surfels.is_empty()
    }

    pub fn surfels(&self) -> &[Surfel] {
        &self.surfels
    }

    pub fn clear(&mut self) {
        self.cells.clear();
        self.surfels.clear();
        self.misses.clear();
    }

    fn key(&self, p: Vec3) -> [i32; 3] {
        let k = (p / self.voxel_mm).floor();
        [k.x as i32, k.y as i32, k.z as i32]
    }

    /// Adds one observation. Returns the surfel index if the voxel was empty before.
    pub fn insert(&mut self, position: Vec3, normal: Vec3) -> Option<usize> {
        let key = self.key(position);
        match self.cells.get(&key) {
            Some(&i) => {
                self.merge(i, position, normal);
                None
            }
            None => {
                let i = self.surfels.len();
                self.cells.insert(key, i);
                self.surfels.push(Surfel { position, normal, count: 1 });
                Some(i)
            }
        }
    }

    /// Like [`insert`](Self::insert), but merges into the nearest surfel in the surrounding
    /// voxels when one lies within one voxel size. Plain binning lets sensor noise straddle
    /// voxel boundaries, so a surface observed repeatedly would keep spawning new voxels and
    /// "thicken"; merging keeps roughly one surfel per voxel-size spacing.
    pub fn insert_merging(&mut self, position: Vec3, normal: Vec3) -> Option<usize> {
        let [kx, ky, kz] = self.key(position);
        let mut best: Option<(f32, usize)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(&i) = self.cells.get(&[kx + dx, ky + dy, kz + dz]) {
                        let d = self.surfels[i].position.distance_squared(position);
                        if best.is_none_or(|(bd, _)| d < bd) {
                            best = Some((d, i));
                        }
                    }
                }
            }
        }
        match best {
            Some((d, i)) if d <= self.voxel_mm * self.voxel_mm => {
                self.merge(i, position, normal);
                None
            }
            _ => {
                let key = [kx, ky, kz];
                if let Some(&i) = self.cells.get(&key) {
                    self.merge(i, position, normal);
                    return None;
                }
                let i = self.surfels.len();
                self.cells.insert(key, i);
                self.surfels.push(Surfel { position, normal, count: 1 });
                Some(i)
            }
        }
    }

    fn merge(&mut self, i: usize, position: Vec3, normal: Vec3) {
        let s = &mut self.surfels[i];
        s.count += 1;
        let w = 1.0 / s.count as f32;
        s.position += (position - s.position) * w;
        let n = if normal.dot(s.normal) < 0.0 { -normal } else { normal };
        s.normal = (s.normal + (n - s.normal) * w).normalize_or(s.normal);
    }

    /// Free-space carving: each surfel for which `seen_through` holds (a view measured surface
    /// clearly behind it) collects a miss; surfels with at least as many misses as observations
    /// are removed, so a well-observed surface survives an occasional contradiction while a
    /// one-off noise point does not. Surfel indices change when anything is removed. Returns the
    /// number of surfels removed.
    pub fn carve(&mut self, mut seen_through: impl FnMut(Vec3) -> bool) -> usize {
        self.misses.resize(self.surfels.len(), 0);
        let mut removed = 0;
        for (s, m) in self.surfels.iter().zip(self.misses.iter_mut()) {
            if seen_through(s.position) {
                *m += 1;
                if *m >= s.count {
                    removed += 1;
                }
            }
        }
        if removed > 0 {
            let (surfels, misses): (Vec<Surfel>, Vec<u32>) =
                self.surfels.iter().zip(&self.misses).filter(|(s, &m)| m < s.count).map(|(s, &m)| (*s, m)).unzip();
            self.surfels = surfels;
            self.misses = misses;
            self.cells.clear();
            for i in 0..self.surfels.len() {
                let key = self.key(self.surfels[i].position);
                self.cells.entry(key).or_insert(i);
            }
        }
        removed
    }

    /// Bins a batch of oriented points into a fresh map (used to subsample frames).
    pub fn from_points(voxel_mm: f32, points: impl IntoIterator<Item = (Vec3, Vec3)>) -> Self {
        let mut map = Self::new(voxel_mm);
        for (p, n) in points {
            map.insert(p, n);
        }
        map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn averages_observations_per_voxel() {
        let mut map = VoxelMap::new(2.0);
        assert!(map.insert(Vec3::new(0.5, 0.5, 0.5), Vec3::Z).is_some());
        assert!(map.insert(Vec3::new(1.5, 1.5, 1.5), Vec3::Z).is_none());
        assert!(map.insert(Vec3::new(2.5, 0.5, 0.5), Vec3::Z).is_some());
        assert_eq!(map.len(), 2);
        let s = map.surfels()[0];
        assert_eq!(s.count, 2);
        assert!((s.position - Vec3::splat(1.0)).length() < 1e-6);
        // Flipped normals don't cancel out.
        map.insert(Vec3::new(1.0, 1.0, 1.0), -Vec3::Z);
        assert!(map.surfels()[0].normal.dot(Vec3::Z) > 0.99);
    }

    #[test]
    fn carving_removes_points_seen_through_more_often_than_seen() {
        let mut map = VoxelMap::new(1.0);
        for _ in 0..3 {
            map.insert(Vec3::new(0.5, 0.5, 0.5), Vec3::Z); // observed three times
        }
        map.insert(Vec3::new(5.5, 0.5, 0.5), Vec3::Z); // observed once
        let through = |p: Vec3| p.x < 10.0;
        assert_eq!(map.carve(through), 1, "the single observation goes at the first contradiction");
        assert_eq!(map.len(), 1);
        assert_eq!(map.carve(through), 0);
        assert_eq!(map.carve(through), 1, "the well-observed one only after three");
        assert!(map.is_empty());
        // Carved voxels can be observed again.
        assert!(map.insert(Vec3::new(0.5, 0.5, 0.5), Vec3::Z).is_some());
    }

    #[test]
    fn merging_absorbs_noise_across_voxel_boundaries() {
        let mut map = VoxelMap::new(0.5);
        // A point near a voxel corner, then noisy re-observations just across the boundaries.
        map.insert_merging(Vec3::new(0.49, 0.49, 0.49), Vec3::Z);
        for d in [0.02f32, -0.03, 0.04, 0.01] {
            map.insert_merging(Vec3::new(0.5 + d, 0.5 - d, 0.5 + d), Vec3::Z);
        }
        assert_eq!(map.len(), 1);
        // A genuinely new point one voxel away still creates a surfel.
        assert!(map.insert_merging(Vec3::new(1.3, 0.5, 0.5), Vec3::Z).is_some());
    }
}
