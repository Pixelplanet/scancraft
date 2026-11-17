//! Quadric-error-metric edge-collapse decimation (Garland & Heckbert) on a welded mesh.
//!
//! Each collapse is checked against the link condition and for face flips, so a manifold input
//! stays manifold (and a watertight input stays watertight). Boundary edges carry extra
//! constraint quadrics so open outlines are preserved.

use glam::{DMat3, DVec3};
use scancraft_core::{Mesh, Point3D, Triangle};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Copy, Default)]
struct Quadric([f64; 10]);

impl Quadric {
    fn plane(n: DVec3, d: f64, w: f64) -> Self {
        let (a, b, c) = (n.x, n.y, n.z);
        Self([a * a * w, a * b * w, a * c * w, a * d * w, b * b * w, b * c * w, b * d * w, c * c * w, c * d * w, d * d * w])
    }
    fn add(&mut self, o: &Quadric) {
        for i in 0..10 {
            self.0[i] += o.0[i];
        }
    }
    fn sum(a: &Quadric, b: &Quadric) -> Quadric {
        let mut q = *a;
        q.add(b);
        q
    }
    fn eval(&self, p: DVec3) -> f64 {
        let q = &self.0;
        let (x, y, z) = (p.x, p.y, p.z);
        q[0] * x * x + 2.0 * q[1] * x * y + 2.0 * q[2] * x * z + 2.0 * q[3] * x + q[4] * y * y + 2.0 * q[5] * y * z
            + 2.0 * q[6] * y
            + q[7] * z * z
            + 2.0 * q[8] * z
            + q[9]
    }
    fn optimum(&self) -> Option<DVec3> {
        let q = &self.0;
        let m = DMat3::from_cols(DVec3::new(q[0], q[1], q[2]), DVec3::new(q[1], q[4], q[5]), DVec3::new(q[2], q[5], q[7]));
        let det = m.determinant();
        let scale = (q[0] + q[4] + q[7]).powi(3);
        if !(det.abs() > 1e-6 * scale.abs().max(1e-30)) {
            return None;
        }
        let p = m.inverse() * DVec3::new(-q[3], -q[6], -q[8]);
        p.is_finite().then_some(p)
    }
}

#[derive(Clone, Copy)]
struct Candidate {
    cost: f64,
    u: u32,
    v: u32,
    stamp_u: u32,
    stamp_v: u32,
    target: DVec3,
}

impl PartialEq for Candidate {
    fn eq(&self, o: &Self) -> bool {
        self.cost == o.cost
    }
}
impl Eq for Candidate {}
impl PartialOrd for Candidate {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Candidate {
    fn cmp(&self, o: &Self) -> Ordering {
        // Min-heap on cost.
        o.cost.total_cmp(&self.cost)
    }
}

pub struct MeshDecimator {
    pub target_ratio: f32, // e.g. 0.5 for 50% polygon count
}

impl Default for MeshDecimator {
    fn default() -> Self {
        Self { target_ratio: 0.5 }
    }
}

struct State {
    pos: Vec<DVec3>,
    quad: Vec<Quadric>,
    tris: Vec<[u32; 3]>,
    alive: Vec<bool>,
    vtris: Vec<Vec<u32>>,
    stamp: Vec<u32>,
    removed: Vec<bool>,
    boundary: Vec<bool>,
}

impl State {
    fn neighbors(&self, v: u32, out: &mut Vec<u32>) {
        out.clear();
        for &t in &self.vtris[v as usize] {
            for &w in &self.tris[t as usize] {
                if w != v && !out.contains(&w) {
                    out.push(w);
                }
            }
        }
    }

    fn candidate(&self, u: u32, v: u32) -> Candidate {
        let q = Quadric::sum(&self.quad[u as usize], &self.quad[v as usize]);
        let (pu, pv) = (self.pos[u as usize], self.pos[v as usize]);
        let mid = (pu + pv) * 0.5;
        let mut best = (q.eval(mid), mid);
        for p in [pu, pv] {
            let c = q.eval(p);
            if c < best.0 {
                best = (c, p);
            }
        }
        if let Some(p) = q.optimum() {
            // Only trust the optimum near the edge; far-away optima come from flat regions.
            let len = pu.distance(pv);
            if p.distance(mid) <= len {
                let c = q.eval(p);
                if c < best.0 {
                    best = (c, p);
                }
            }
        }
        Candidate {
            cost: best.0.max(0.0),
            u,
            v,
            stamp_u: self.stamp[u as usize],
            stamp_v: self.stamp[v as usize],
            target: best.1,
        }
    }
}

impl MeshDecimator {
    pub fn new(target_ratio: f32) -> Self {
        Self { target_ratio: target_ratio.clamp(0.01, 1.0) }
    }

    pub fn decimate(&self, mesh: &Mesh) -> Mesh {
        let orig = mesh.triangles.len();
        let nv = mesh.vertices.len();
        if orig < 10 || self.target_ratio >= 1.0 {
            return mesh.clone();
        }
        let target = ((orig as f64 * self.target_ratio as f64) as usize).max(4);

        let tris: Vec<[u32; 3]> = mesh
            .triangles
            .iter()
            .map(|t| t.indices)
            .filter(|t| t.iter().all(|&i| (i as usize) < nv) && t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
            .collect();
        let pos: Vec<DVec3> = mesh.vertices.iter().map(|v| DVec3::from(v.position.map(|x| x as f64))).collect();

        let mut st = State {
            quad: vec![Quadric::default(); nv],
            alive: vec![true; tris.len()],
            vtris: vec![Vec::new(); nv],
            stamp: vec![0; nv],
            removed: vec![false; nv],
            boundary: vec![false; nv],
            pos,
            tris,
        };

        // Face quadrics (area weighted) and vertex → triangle lists.
        for (ti, t) in st.tris.iter().enumerate() {
            let (p0, p1, p2) = (st.pos[t[0] as usize], st.pos[t[1] as usize], st.pos[t[2] as usize]);
            let c = (p1 - p0).cross(p2 - p0);
            let len = c.length();
            for &i in t {
                st.vtris[i as usize].push(ti as u32);
            }
            if len < 1e-20 {
                continue;
            }
            let n = c / len;
            let q = Quadric::plane(n, -n.dot(p0), 0.5 * len);
            for &i in t {
                st.quad[i as usize].add(&q);
            }
        }

        // Unique edges, boundary detection and boundary constraint quadrics.
        let mut edges: Vec<(u32, u32, u32)> = Vec::with_capacity(st.tris.len() * 3);
        for (ti, t) in st.tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                edges.push((a.min(b), a.max(b), ti as u32));
            }
        }
        edges.sort_unstable();
        let mut unique: Vec<(u32, u32)> = Vec::with_capacity(edges.len() / 2 + 1);
        let mut s = 0;
        while s < edges.len() {
            let mut e = s + 1;
            while e < edges.len() && (edges[e].0, edges[e].1) == (edges[s].0, edges[s].1) {
                e += 1;
            }
            let (a, b, ti) = edges[s];
            if e - s == 1 {
                st.boundary[a as usize] = true;
                st.boundary[b as usize] = true;
                let t = st.tris[ti as usize];
                let (p0, p1, p2) = (st.pos[t[0] as usize], st.pos[t[1] as usize], st.pos[t[2] as usize]);
                let fnorm = (p1 - p0).cross(p2 - p0).normalize_or_zero();
                let (pa, pb) = (st.pos[a as usize], st.pos[b as usize]);
                let dir = pb - pa;
                let n = dir.cross(fnorm).normalize_or_zero();
                if n != DVec3::ZERO {
                    let q = Quadric::plane(n, -n.dot(pa), 10.0 * dir.length_squared());
                    st.quad[a as usize].add(&q);
                    st.quad[b as usize].add(&q);
                }
            }
            unique.push((a, b));
            s = e;
        }
        drop(edges);

        let mut heap: BinaryHeap<Candidate> = unique.iter().map(|&(a, b)| st.candidate(a, b)).collect();
        drop(unique);

        let mut alive_count = st.tris.len();
        let mut nu = Vec::new();
        let mut nv_ = Vec::new();
        let mut shared = Vec::new();

        while alive_count > target {
            let Some(c) = heap.pop() else { break };
            let (u, v) = (c.u, c.v);
            if st.removed[u as usize]
                || st.removed[v as usize]
                || st.stamp[u as usize] != c.stamp_u
                || st.stamp[v as usize] != c.stamp_v
            {
                continue;
            }
            // Triangles on the edge.
            shared.clear();
            for &t in &st.vtris[u as usize] {
                if st.tris[t as usize].contains(&v) {
                    shared.push(t);
                }
            }
            if shared.is_empty() || shared.len() > 2 {
                continue;
            }
            // Link condition: common neighbours must be exactly the opposite vertices.
            st.neighbors(u, &mut nu);
            st.neighbors(v, &mut nv_);
            let common = nu.iter().filter(|w| **w != v && nv_.contains(w)).count();
            if common != shared.len() {
                continue;
            }
            // An interior edge between two boundary vertices would pinch the surface.
            if shared.len() == 2 && st.boundary[u as usize] && st.boundary[v as usize] {
                continue;
            }
            // Never collapse a tetrahedron-like configuration.
            if nu.len() <= 3 && nv_.len() <= 3 {
                continue;
            }
            // Face-flip check.
            let p = c.target;
            let mut ok = true;
            for (w, list) in [(u, &st.vtris[u as usize]), (v, &st.vtris[v as usize])] {
                for &t in list.iter() {
                    if shared.contains(&t) {
                        continue;
                    }
                    let tri = st.tris[t as usize];
                    let pts = tri.map(|i| st.pos[i as usize]);
                    let old = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
                    let mut np = pts;
                    for k in 0..3 {
                        if tri[k] == w {
                            np[k] = p;
                        }
                    }
                    let new = (np[1] - np[0]).cross(np[2] - np[0]);
                    let (lo, ln) = (old.length(), new.length());
                    if ln < 1e-12 || old.dot(new) < 0.2 * lo * ln {
                        ok = false;
                        break;
                    }
                }
                if !ok {
                    break;
                }
            }
            if !ok {
                continue;
            }

            // Collapse v into u.
            for &t in &shared {
                st.alive[t as usize] = false;
            }
            alive_count -= shared.len();
            let vt = std::mem::take(&mut st.vtris[v as usize]);
            for t in vt {
                if !st.alive[t as usize] {
                    continue;
                }
                for k in 0..3 {
                    if st.tris[t as usize][k] == v {
                        st.tris[t as usize][k] = u;
                    }
                }
                st.vtris[u as usize].push(t);
            }
            let alive = &st.alive;
            st.vtris[u as usize].retain(|&t| alive[t as usize]);
            // Only the vertices opposite the edge lost triangles.
            for &t in &shared {
                for w in st.tris[t as usize] {
                    if w != u && w != v {
                        let al = &st.alive;
                        st.vtris[w as usize].retain(|&t| al[t as usize]);
                    }
                }
            }
            st.removed[v as usize] = true;
            st.boundary[u as usize] |= st.boundary[v as usize];
            st.pos[u as usize] = p;
            let qv = st.quad[v as usize];
            st.quad[u as usize].add(&qv);
            st.stamp[u as usize] += 1;

            st.neighbors(u, &mut nu);
            for &w in &nu {
                // Edges around u changed cost; invalidate neighbours' older entries lazily by
                // pushing fresh ones (stale ones fail the stamp test for u).
                heap.push(st.candidate(u, w));
            }
        }

        // Compact.
        let mut remap = vec![u32::MAX; nv];
        let mut out = Mesh::new();
        for (t, &a) in st.tris.iter().zip(&st.alive) {
            if !a {
                continue;
            }
            let mut idx = [0u32; 3];
            for k in 0..3 {
                let i = t[k] as usize;
                if remap[i] == u32::MAX {
                    let mut v: Point3D = mesh.vertices[i];
                    v.position = st.pos[i].as_vec3().to_array();
                    remap[i] = out.add_vertex(v);
                }
                idx[k] = remap[i];
            }
            out.triangles.push(Triangle::new(idx[0], idx[1], idx[2]));
        }
        out.recompute_normals();
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metrics::MeshMetrics;
    use scancraft_core::Point3D;

    #[test]
    fn test_decimate() {
        let mut mesh = Mesh::new();
        for x in 0..11 {
            for y in 0..11 {
                let z = ((x as f32) * 0.5).sin() * 0.3;
                mesh.add_vertex(Point3D::new([x as f32, y as f32, z]));
            }
        }
        for x in 0..10 {
            for y in 0..10 {
                let i0 = (x * 11 + y) as u32;
                let i1 = ((x + 1) * 11 + y) as u32;
                let i2 = ((x + 1) * 11 + (y + 1)) as u32;
                let i3 = (x * 11 + (y + 1)) as u32;
                mesh.add_triangle(i0, i1, i2);
                mesh.add_triangle(i0, i2, i3);
            }
        }
        let before = MeshMetrics::analyze(&mesh);
        let decimated = MeshDecimator::new(0.5).decimate(&mesh);
        let after = MeshMetrics::analyze(&decimated);
        assert!(decimated.triangles.len() <= 100 && decimated.triangles.len() > 80, "{}", decimated.triangles.len());
        assert_eq!(after.non_manifold_edges, 0);
        assert_eq!(after.inconsistent_edges, 0);
        assert!((after.surface_area_mm2 - before.surface_area_mm2).abs() / before.surface_area_mm2 < 0.05);
    }
}
