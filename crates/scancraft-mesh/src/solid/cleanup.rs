//! Mesh clean-up after marching cubes: collapse of sliver edges, detection of self-intersecting
//! triangle pairs and folded edges, and a local repair of both.
//!
//! Marching cubes puts a vertex very close to a grid node whenever the field there is almost
//! exactly at the iso value. The triangles around such a node become needles a few micrometres
//! wide, and in float arithmetic neighbouring needles can cross each other. Collapsing the short
//! edges (with the link condition, so the mesh stays a closed manifold of the same genus) removes
//! them; what still intersects or folds afterwards is relaxed locally.

use super::grid::{cell_key, FxMap};
use glam::{DVec3, IVec3, Vec3};
use rayon::prelude::*;

/// Vertex → incident triangles.
fn incidence(nv: usize, tris: &[[u32; 3]], alive: &[bool]) -> Vec<Vec<u32>> {
    let mut inc: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (t, tri) in tris.iter().enumerate() {
        if alive[t] {
            for &v in tri {
                inc[v as usize].push(t as u32);
            }
        }
    }
    inc
}

fn face_normal(pos: &[Vec3], t: &[u32; 3]) -> DVec3 {
    let (a, b, c) = (pos[t[0] as usize].as_dvec3(), pos[t[1] as usize].as_dvec3(), pos[t[2] as usize].as_dvec3());
    (b - a).cross(c - a)
}

/// Collapses edges shorter than `min_len` into their midpoint, shortest first, when the link
/// condition holds and no surviving triangle turns over or becomes degenerate. Returns the
/// compacted mesh and the number of collapses.
pub fn collapse_short_edges(pos: Vec<Vec3>, tris: Vec<[u32; 3]>, min_len: f32) -> (Vec<Vec3>, Vec<[u32; 3]>, usize) {
    let min2 = min_len * min_len;
    let mut cand: Vec<(f32, u32, u32)> = tris
        .par_iter()
        .flat_map_iter(|t| {
            [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
                .into_iter()
                .filter(|(a, b)| a < b)
                .map(|(a, b)| (pos[a as usize].distance_squared(pos[b as usize]), a, b))
                .filter(move |e| e.0 < min2)
                .collect::<Vec<_>>()
        })
        .collect();
    if cand.is_empty() {
        return (pos, tris, 0);
    }
    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (mut pos, mut tris) = (pos, tris);
    let nv = pos.len();
    let mut alive = vec![true; tris.len()];
    let mut inc = incidence(nv, &tris, &alive);
    let mut dead_v = vec![false; nv];
    let mut done = 0usize;
    for &(_, u, v) in &cand {
        let (u, v) = (u as usize, v as usize);
        if dead_v[u] || dead_v[v] {
            continue;
        }
        if pos[u].distance_squared(pos[v]) >= min2 {
            continue;
        }
        // Triangles on the edge and the link condition.
        let shared: Vec<u32> = inc[u].iter().copied().filter(|&t| tris[t as usize].contains(&(v as u32))).collect();
        if shared.len() != 2 {
            continue;
        }
        let opp: Vec<u32> = shared
            .iter()
            .map(|&t| *tris[t as usize].iter().find(|&&x| x as usize != u && x as usize != v).unwrap())
            .collect();
        if opp[0] == opp[1] {
            continue;
        }
        let ring = |w: usize| -> Vec<u32> {
            let mut r: Vec<u32> = inc[w].iter().flat_map(|&t| tris[t as usize]).filter(|&x| x as usize != w).collect();
            r.sort_unstable();
            r.dedup();
            r
        };
        let (ru, rv) = (ring(u), ring(v));
        let common = ru.iter().filter(|x| rv.binary_search(x).is_ok()).count();
        if common != 2 {
            continue;
        }
        let target = (pos[u] + pos[v]) * 0.5;
        // Flip / degeneracy check on the triangles that survive.
        let mut ok = true;
        for &t in inc[u].iter().chain(&inc[v]) {
            if shared.contains(&t) {
                continue;
            }
            let tri = tris[t as usize];
            let before = face_normal(&pos, &tri);
            let mut moved = tri;
            for x in moved.iter_mut() {
                if *x as usize == v {
                    *x = u as u32;
                }
            }
            let (a, b, c) = (
                if moved[0] as usize == u { target } else { pos[moved[0] as usize] },
                if moved[1] as usize == u { target } else { pos[moved[1] as usize] },
                if moved[2] as usize == u { target } else { pos[moved[2] as usize] },
            );
            let after = (b.as_dvec3() - a.as_dvec3()).cross(c.as_dvec3() - a.as_dvec3());
            if after.length_squared() < 1e-24 || before.dot(after) <= 0.2 * before.length() * after.length() {
                ok = false;
                break;
            }
        }
        if !ok {
            continue;
        }
        // Apply.
        for &t in &shared {
            alive[t as usize] = false;
            for &o in &opp {
                inc[o as usize].retain(|&x| x != t);
            }
        }
        let vt: Vec<u32> = std::mem::take(&mut inc[v]);
        for t in vt {
            if !alive[t as usize] {
                continue;
            }
            for x in tris[t as usize].iter_mut() {
                if *x as usize == v {
                    *x = u as u32;
                }
            }
            inc[u].push(t);
        }
        inc[u].retain(|&t| alive[t as usize]);
        pos[u] = target;
        dead_v[v] = true;
        done += 1;
    }
    // Compact.
    let mut remap = vec![u32::MAX; nv];
    let mut out_pos = Vec::with_capacity(nv);
    let mut out_tris = Vec::with_capacity(tris.len());
    for (t, tri) in tris.iter().enumerate() {
        if !alive[t] {
            continue;
        }
        let mut nt = [0u32; 3];
        for k in 0..3 {
            let v = tri[k] as usize;
            if remap[v] == u32::MAX {
                remap[v] = out_pos.len() as u32;
                out_pos.push(pos[v]);
            }
            nt[k] = remap[v];
        }
        out_tris.push(nt);
    }
    (out_pos, out_tris, done)
}

/// Segment p→q against triangle (a, b, c), strictly inside both (Möller–Trumbore).
fn segment_hits_triangle(p: DVec3, q: DVec3, a: DVec3, b: DVec3, c: DVec3) -> bool {
    let d = q - p;
    let (e1, e2) = (b - a, c - a);
    let h = d.cross(e2);
    let det = e1.dot(h);
    let scale = e1.length() * e2.length() * d.length();
    if det.abs() <= 1e-12 * scale {
        return false;
    }
    let f = 1.0 / det;
    let s = p - a;
    let u = f * s.dot(h);
    if u <= 1e-9 || u >= 1.0 - 1e-9 {
        return false;
    }
    let qq = s.cross(e1);
    let v = f * d.dot(qq);
    if v <= 1e-9 || u + v >= 1.0 - 1e-9 {
        return false;
    }
    let t = f * e2.dot(qq);
    t > 1e-9 && t < 1.0 - 1e-9
}

fn triangles_intersect(t1: [DVec3; 3], t2: [DVec3; 3]) -> bool {
    for k in 0..3 {
        if segment_hits_triangle(t1[k], t1[(k + 1) % 3], t2[0], t2[1], t2[2])
            || segment_hits_triangle(t2[k], t2[(k + 1) % 3], t1[0], t1[1], t1[2])
        {
            return true;
        }
    }
    false
}

/// Pairs of triangles that share no vertex and cross each other. Triangles are bucketed by
/// bounding box into cubic cells; a pair is tested in the cell holding the low corner of the
/// overlap of the two boxes only, so each pair is counted once.
pub fn self_intersections(pos: &[Vec3], tris: &[[u32; 3]]) -> Vec<(u32, u32)> {
    if tris.is_empty() {
        return Vec::new();
    }
    let mut lens: Vec<f32> =
        tris.iter().step_by((tris.len() / 20000).max(1)).map(|t| pos[t[0] as usize].distance(pos[t[1] as usize])).collect();
    let m = lens.len() / 2;
    lens.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
    let cell = (2.0 * lens[m]).max(1e-3);
    let inv = 1.0 / cell;
    let boxes: Vec<(Vec3, Vec3)> = tris
        .par_iter()
        .map(|t| {
            let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
            (a.min(b).min(c), a.max(b).max(c))
        })
        .collect();
    let mut keyed: Vec<(u64, u32)> = boxes
        .par_iter()
        .enumerate()
        .flat_map_iter(|(i, (lo, hi))| {
            let (l, h) = ((*lo * inv).floor().as_ivec3(), (*hi * inv).floor().as_ivec3());
            let mut v = Vec::new();
            for z in l.z..=h.z {
                for y in l.y..=h.y {
                    for x in l.x..=h.x {
                        v.push((cell_key(IVec3::new(x, y, z)), i as u32));
                    }
                }
            }
            v
        })
        .collect();
    keyed.par_sort_unstable();
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut s = 0;
    while s < keyed.len() {
        let mut e = s + 1;
        while e < keyed.len() && keyed[e].0 == keyed[s].0 {
            e += 1;
        }
        if e - s > 1 {
            runs.push((s, e));
        }
        s = e;
    }
    let mut pairs: Vec<(u32, u32)> = runs
        .par_iter()
        .flat_map_iter(|&(s, e)| {
            let key = keyed[s].0;
            let mut out = Vec::new();
            for a in s..e {
                let i = keyed[a].1 as usize;
                for b in a + 1..e {
                    let j = keyed[b].1 as usize;
                    let (ti, tj) = (tris[i], tris[j]);
                    if ti.iter().any(|x| tj.contains(x)) {
                        continue;
                    }
                    let (bi, bj) = (boxes[i], boxes[j]);
                    let lo = bi.0.max(bj.0);
                    let hi = bi.1.min(bj.1);
                    if lo.x > hi.x || lo.y > hi.y || lo.z > hi.z {
                        continue;
                    }
                    if cell_key((lo * inv).floor().as_ivec3()) != key {
                        continue;
                    }
                    let f = |t: [u32; 3]| t.map(|k| pos[k as usize].as_dvec3());
                    if triangles_intersect(f(ti), f(tj)) {
                        out.push((i.min(j) as u32, i.max(j) as u32));
                    }
                }
            }
            out
        })
        .collect();
    pairs.sort_unstable();
    pairs.dedup();
    pairs
}

/// Interior edges whose two triangles face against each other (normal dot below `dot_max`).
pub fn folded_edges(pos: &[Vec3], tris: &[[u32; 3]], dot_max: f64) -> Vec<(u32, u32)> {
    let mut edges: FxMap<u64, u32> = FxMap::default();
    edges.reserve(tris.len() * 2);
    let mut out = Vec::new();
    for (t, tri) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let key = ((a.min(b) as u64) << 32) | a.max(b) as u64;
            if let Some(o) = edges.insert(key, t as u32) {
                let (n1, n2) = (face_normal(pos, &tris[o as usize]), face_normal(pos, tri));
                let l = n1.length() * n2.length();
                if l > 0.0 && n1.dot(n2) < dot_max * l {
                    out.push((o, t as u32));
                }
            }
        }
    }
    out
}

/// Relaxes the neighbourhood (two vertex rings) of every self-intersecting pair and folded edge
/// with a few Laplacian steps, re-checking up to `rounds` times. Returns (intersecting pairs,
/// folded edges) found before the repair and the counts left after it.
pub fn repair(pos: &mut [Vec3], tris: &[[u32; 3]], rounds: usize) -> ((usize, usize), (usize, usize)) {
    let mut first = None;
    let mut last = (0, 0);
    let nv = pos.len();
    let mut nbr: Option<Vec<Vec<u32>>> = None;
    for round in 0..=rounds {
        let si = self_intersections(pos, tris);
        let fo = folded_edges(pos, tris, -0.9);
        last = (si.len(), fo.len());
        if first.is_none() {
            first = Some(last);
        }
        if (si.is_empty() && fo.is_empty()) || round == rounds {
            break;
        }
        let nb = nbr.get_or_insert_with(|| {
            let mut n: Vec<Vec<u32>> = vec![Vec::new(); nv];
            for t in tris {
                for k in 0..3 {
                    n[t[k] as usize].push(t[(k + 1) % 3]);
                    n[t[k] as usize].push(t[(k + 2) % 3]);
                }
            }
            for l in n.iter_mut() {
                l.sort_unstable();
                l.dedup();
            }
            n
        });
        let mut mark = vec![false; nv];
        let mut seeds: Vec<u32> = Vec::new();
        for &(a, b) in si.iter().chain(&fo) {
            seeds.extend(tris[a as usize]);
            seeds.extend(tris[b as usize]);
        }
        for _ in 0..2 {
            let mut next = Vec::new();
            for &s in &seeds {
                if !mark[s as usize] {
                    mark[s as usize] = true;
                    next.extend(nb[s as usize].iter().copied());
                }
            }
            seeds = next;
        }
        let verts: Vec<usize> = (0..nv).filter(|&i| mark[i]).collect();
        for _ in 0..3 {
            let upd: Vec<Vec3> = verts
                .iter()
                .map(|&i| {
                    let l = &nb[i];
                    let c = l.iter().map(|&j| pos[j as usize]).sum::<Vec3>() / l.len().max(1) as f32;
                    pos[i] + (c - pos[i]) * 0.5
                })
                .collect();
            for (&i, p) in verts.iter().zip(upd) {
                pos[i] = p;
            }
        }
    }
    (first.unwrap_or((0, 0)), last)
}
