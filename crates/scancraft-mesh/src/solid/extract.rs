//! Welded marching cubes over the band field. Nodes outside the band count as χ = 0, so every
//! cell is fully known and the surface is closed.

use super::grid::{cell_key, FxMap};
use super::poisson::{local_index, BandGrid, B, BV};
use crate::tables::{EDGE_TABLE, TRI_TABLE};
use glam::{IVec3, Vec3};
use rayon::prelude::*;
use std::collections::HashSet;

const CORNERS: [[i32; 3]; 8] =
    [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];
const EDGES: [(usize, usize, u64); 12] = [
    (0, 1, 0),
    (1, 2, 1),
    (3, 2, 0),
    (0, 3, 1),
    (4, 5, 0),
    (5, 6, 1),
    (7, 6, 0),
    (4, 7, 1),
    (0, 4, 2),
    (1, 5, 2),
    (2, 6, 2),
    (3, 7, 2),
];
const COORD_BITS: u32 = 20;

#[inline]
fn pack(c: IVec3) -> u64 {
    ((c.x as u64) << (2 * COORD_BITS)) | ((c.y as u64) << COORD_BITS) | (c.z as u64)
}

/// Iso-surface `χ = iso` with triangles wound counter-clockwise seen from outside (χ < iso).
pub fn extract(grid: &BandGrid, chi: &[f32], iso: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let lookup: &FxMap<u64, u32> = &grid.lookup;
    // Cells whose lower corner is in a band block, or in a block just below one.
    let mut todo: HashSet<IVec3> = HashSet::with_capacity(grid.blocks.len() * 2);
    for b in &grid.blocks {
        for o in 0..8 {
            todo.insert(*b - IVec3::new(o & 1, (o >> 1) & 1, (o >> 2) & 1));
        }
    }
    let mut todo: Vec<IVec3> = todo.into_iter().collect();
    todo.sort_unstable_by_key(|b| cell_key(*b));

    let h = grid.h;
    let origin = grid.origin;
    let per_block: Vec<(Vec<(u64, Vec3)>, Vec<[u64; 3]>)> = todo
        .par_iter()
        .map(|&b| {
            const S: usize = (B + 1) as usize;
            // f = iso − χ: negative inside the material.
            let mut vals = [iso; S * S * S];
            let mut any_in = false;
            for o in 0..8 {
                let off = IVec3::new(o & 1, (o >> 1) & 1, (o >> 2) & 1);
                let Some(&bi) = lookup.get(&cell_key(b + off)) else { continue };
                let f = &chi[bi as usize * BV..(bi as usize + 1) * BV];
                let lo = off * B;
                let hi = (lo + IVec3::splat(B - 1)).min(IVec3::splat(B));
                for z in lo.z..=hi.z {
                    for y in lo.y..=hi.y {
                        for x in lo.x..=hi.x {
                            let l = IVec3::new(x, y, z) - lo;
                            let mut v = iso - f[local_index(l.x, l.y, l.z)];
                            if v.abs() < 1e-7 {
                                v = 1e-7;
                            }
                            if v < 0.0 {
                                any_in = true;
                            }
                            vals[(z as usize * S + y as usize) * S + x as usize] = v;
                        }
                    }
                }
            }
            let mut verts: Vec<(u64, Vec3)> = Vec::new();
            let mut tris: Vec<[u64; 3]> = Vec::new();
            if !any_in {
                return (verts, tris);
            }
            let at = |x: i32, y: i32, z: i32| vals[(z as usize * S + y as usize) * S + x as usize];
            let mut seen: FxMap<u64, ()> = FxMap::default();
            let gbase = b * B;
            for z in 0..B {
                for y in 0..B {
                    for x in 0..B {
                        let mut v = [0f32; 8];
                        let mut idx = 0usize;
                        for (i, c) in CORNERS.iter().enumerate() {
                            v[i] = at(x + c[0], y + c[1], z + c[2]);
                            if v[i] < 0.0 {
                                idx |= 1 << i;
                            }
                        }
                        if EDGE_TABLE[idx] == 0 {
                            continue;
                        }
                        let cell = gbase + IVec3::new(x, y, z);
                        let mut ek = [0u64; 12];
                        for (e, &(a, bb, axis)) in EDGES.iter().enumerate() {
                            if EDGE_TABLE[idx] & (1 << e) == 0 {
                                continue;
                            }
                            let ca = cell + IVec3::from_array(CORNERS[a]);
                            let key = (pack(ca) << 2) | axis;
                            ek[e] = key;
                            if seen.insert(key, ()).is_none() {
                                let (va, vb) = (v[a], v[bb]);
                                let t = (va / (va - vb)).clamp(1e-3, 1.0 - 1e-3);
                                let mut p = ca.as_vec3();
                                p[axis as usize] += t;
                                verts.push((key, origin + p * h));
                            }
                        }
                        let row = &TRI_TABLE[idx];
                        let mut t = 0;
                        while t < 16 && row[t] >= 0 {
                            tris.push([ek[row[t] as usize], ek[row[t + 2] as usize], ek[row[t + 1] as usize]]);
                            t += 3;
                        }
                    }
                }
            }
            (verts, tris)
        })
        .collect();

    let mut verts: Vec<(u64, Vec3)> = Vec::with_capacity(per_block.iter().map(|b| b.0.len()).sum());
    let mut tri_keys: Vec<[u64; 3]> = Vec::with_capacity(per_block.iter().map(|b| b.1.len()).sum());
    for (v, t) in per_block {
        verts.extend(v);
        tri_keys.extend(t);
    }
    verts.par_sort_unstable_by_key(|e| e.0);
    verts.dedup_by_key(|e| e.0);
    let find = |k: u64| verts.binary_search_by_key(&k, |e| e.0).expect("edge vertex") as u32;
    let tris: Vec<[u32; 3]> = tri_keys.par_iter().map(|t| [find(t[0]), find(t[1]), find(t[2])]).collect();
    (verts.into_iter().map(|e| e.1).collect(), tris)
}
