//! Uniform hash grid over points for radius queries, and a fast hasher for integer keys.

use glam::{IVec3, Vec3};
use rayon::prelude::*;
use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

/// Multiplicative hasher for integer keys (std's SipHash is several times slower here).
#[derive(Default, Clone, Copy)]
pub struct FxHasher(u64);

impl Hasher for FxHasher {
    #[inline]
    fn finish(&self) -> u64 {
        self.0
    }
    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.write_u64(*b as u64);
        }
    }
    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.0 = (self.0.rotate_left(5) ^ i).wrapping_mul(0x51_7c_c1_b7_27_22_0a_95);
    }
    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.write_u64(i as u64);
    }
    #[inline]
    fn write_i32(&mut self, i: i32) {
        self.write_u64(i as u32 as u64);
    }
}

pub type FxMap<K, V> = HashMap<K, V, BuildHasherDefault<FxHasher>>;

/// Packs a signed cell coordinate (|c| < 2^20) into a u64 key.
#[inline]
pub fn cell_key(c: IVec3) -> u64 {
    const O: i64 = 1 << 20;
    (((c.x as i64 + O) as u64) << 42) | (((c.y as i64 + O) as u64) << 21) | ((c.z as i64 + O) as u64)
}

/// Points bucketed into cubic cells of side `cell`.
pub struct PointGrid {
    inv: f32,
    /// Point indices sorted by cell.
    order: Vec<u32>,
    cells: FxMap<u64, (u32, u32)>,
}

impl PointGrid {
    pub fn new(points: &[Vec3], cell: f32) -> Self {
        let inv = 1.0 / cell;
        let mut keyed: Vec<(u64, u32)> = points
            .par_iter()
            .enumerate()
            .map(|(i, p)| (cell_key((*p * inv).floor().as_ivec3()), i as u32))
            .collect();
        keyed.par_sort_unstable();
        let mut cells: FxMap<u64, (u32, u32)> = FxMap::default();
        cells.reserve(keyed.len() / 4 + 16);
        let mut s = 0usize;
        while s < keyed.len() {
            let k = keyed[s].0;
            let mut e = s + 1;
            while e < keyed.len() && keyed[e].0 == k {
                e += 1;
            }
            cells.insert(k, (s as u32, e as u32));
            s = e;
        }
        let order = keyed.into_iter().map(|e| e.1).collect();
        Self { inv, order, cells }
    }

    /// Calls `f(index)` for every point whose cell overlaps the cube around `p` with half-side `r`
    /// (a superset of the points within distance `r`; callers test the distance themselves).
    #[inline]
    pub fn for_each_near(&self, p: Vec3, r: f32, mut f: impl FnMut(u32)) {
        let lo = ((p - Vec3::splat(r)) * self.inv).floor().as_ivec3();
        let hi = ((p + Vec3::splat(r)) * self.inv).floor().as_ivec3();
        for z in lo.z..=hi.z {
            for y in lo.y..=hi.y {
                for x in lo.x..=hi.x {
                    if let Some(&(s, e)) = self.cells.get(&cell_key(IVec3::new(x, y, z))) {
                        for &i in &self.order[s as usize..e as usize] {
                            f(i);
                        }
                    }
                }
            }
        }
    }

    /// Calls `f(index, distance²)` for every point within distance `r` of `p`.
    #[inline]
    pub fn for_each_within(&self, points: &[Vec3], p: Vec3, r: f32, mut f: impl FnMut(u32, f32)) {
        let r2 = r * r;
        self.for_each_near(p, r, |i| {
            let d2 = points[i as usize].distance_squared(p);
            if d2 <= r2 {
                f(i, d2);
            }
        });
    }

    /// One representative point per occupied cell (the first in index order), sorted.
    pub fn representatives(&self) -> Vec<u32> {
        let mut v: Vec<u32> = self
            .cells
            .values()
            .map(|&(s, e)| *self.order[s as usize..e as usize].iter().min().unwrap())
            .collect();
        v.sort_unstable();
        v
    }
}
