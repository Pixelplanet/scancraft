//! Topology checks for closed meshes (edge and vertex manifoldness, boundary loops, components,
//! genus) and component filtering.

use glam::{DVec3, Vec3};
use rayon::prelude::*;

#[derive(Clone, Debug, Default)]
pub struct ComponentStats {
    pub triangles: usize,
    pub vertices: usize,
    pub edges: usize,
    pub euler: i64,
    /// (2 − χ) / 2 for a closed component.
    pub genus: i64,
    pub signed_volume_mm3: f64,
    pub area_mm2: f64,
    pub closed: bool,
}

#[derive(Clone, Debug, Default)]
pub struct TopologyStats {
    pub vertices: usize,
    pub triangles: usize,
    pub degenerate_triangles: usize,
    /// Edges used by one triangle.
    pub boundary_edges: usize,
    /// Edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Interior edges traversed twice in the same direction.
    pub inconsistent_edges: usize,
    /// Vertices whose incident triangles form more than one fan.
    pub non_manifold_vertices: usize,
    pub boundary_loops: usize,
    pub components: Vec<ComponentStats>,
    pub watertight: bool,
    pub manifold: bool,
    pub total_volume_mm3: f64,
    pub total_area_mm2: f64,
}

impl TopologyStats {
    pub fn genus_total(&self) -> i64 {
        self.components.iter().map(|c| c.genus).sum()
    }
}

struct Dsu(Vec<u32>);
impl Dsu {
    fn new(n: usize) -> Self {
        Self((0..n as u32).collect())
    }
    fn find(&mut self, mut i: u32) -> u32 {
        while self.0[i as usize] != i {
            let g = self.0[self.0[i as usize] as usize];
            self.0[i as usize] = g;
            i = g;
        }
        i
    }
    fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            self.0[hi as usize] = lo;
        }
    }
}

/// Component label (root vertex) per triangle.
fn triangle_components(nv: usize, tris: &[[u32; 3]]) -> Vec<u32> {
    let mut d = Dsu::new(nv);
    for t in tris {
        d.union(t[0], t[1]);
        d.union(t[0], t[2]);
    }
    tris.iter().map(|t| d.find(t[0])).collect()
}

pub fn analyze(pos: &[Vec3], tris: &[[u32; 3]]) -> TopologyStats {
    let nv = pos.len();
    let mut st = TopologyStats { vertices: nv, triangles: tris.len(), ..Default::default() };
    let valid = |t: &[u32; 3]| t.iter().all(|&i| (i as usize) < nv) && t[0] != t[1] && t[1] != t[2] && t[0] != t[2];
    st.degenerate_triangles = tris.iter().filter(|t| !valid(t)).count();

    // Directed edges sorted by undirected key.
    let mut edges: Vec<(u64, bool, u32)> = tris
        .par_iter()
        .enumerate()
        .filter(|(_, t)| valid(t))
        .flat_map_iter(|(ti, t)| {
            [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])].into_iter().map(move |(u, v)| {
                let (lo, hi, fwd) = if u < v { (u, v, true) } else { (v, u, false) };
                (((lo as u64) << 32) | hi as u64, fwd, ti as u32)
            })
        })
        .collect();
    edges.par_sort_unstable();
    let mut boundary: Vec<(u32, u32)> = Vec::new();
    let mut edge_tri_count: Vec<u32> = Vec::new(); // per unique edge: component lookup later
    let mut unique_edges: Vec<(u64, u32)> = Vec::new(); // (key, a triangle)
    let mut s = 0;
    while s < edges.len() {
        let mut e = s + 1;
        while e < edges.len() && edges[e].0 == edges[s].0 {
            e += 1;
        }
        let c = e - s;
        match c {
            1 => {
                st.boundary_edges += 1;
                let k = edges[s].0;
                boundary.push(((k >> 32) as u32, k as u32));
            }
            2 => {
                if edges[s].1 == edges[s + 1].1 {
                    st.inconsistent_edges += 1;
                }
            }
            _ => st.non_manifold_edges += 1,
        }
        unique_edges.push((edges[s].0, edges[s].2));
        edge_tri_count.push(c as u32);
        s = e;
    }

    // Boundary loops: connected components of the boundary edge graph.
    if !boundary.is_empty() {
        let mut d = Dsu::new(nv);
        for &(a, b) in &boundary {
            d.union(a, b);
        }
        let mut roots: Vec<u32> = boundary.iter().map(|&(a, _)| d.find(a)).collect();
        roots.sort_unstable();
        roots.dedup();
        st.boundary_loops = roots.len();
    }

    // Vertex manifoldness: the link of every vertex must be one fan.
    let mut inc_count = vec![0u32; nv + 1];
    for t in tris.iter().filter(|t| valid(t)) {
        for &v in t {
            inc_count[v as usize + 1] += 1;
        }
    }
    for i in 0..nv {
        inc_count[i + 1] += inc_count[i];
    }
    let mut fill = inc_count.clone();
    let mut inc = vec![0u32; inc_count[nv] as usize];
    for (ti, t) in tris.iter().enumerate().filter(|(_, t)| valid(t)) {
        for &v in t {
            inc[fill[v as usize] as usize] = ti as u32;
            fill[v as usize] += 1;
        }
    }
    st.non_manifold_vertices = (0..nv)
        .into_par_iter()
        .filter(|&v| {
            let ts = &inc[inc_count[v] as usize..inc_count[v + 1] as usize];
            if ts.len() <= 1 {
                return false;
            }
            // Union triangles sharing an edge through v.
            let mut parent: Vec<usize> = (0..ts.len()).collect();
            fn f(p: &mut [usize], mut i: usize) -> usize {
                while p[i] != i {
                    p[i] = p[p[i]];
                    i = p[i];
                }
                i
            }
            let others = |t: &[u32; 3]| -> [u32; 2] {
                let k = t.iter().position(|&x| x as usize == v).unwrap();
                [t[(k + 1) % 3], t[(k + 2) % 3]]
            };
            for i in 0..ts.len() {
                let oi = others(&tris[ts[i] as usize]);
                for j in i + 1..ts.len() {
                    let oj = others(&tris[ts[j] as usize]);
                    if oi.iter().any(|a| oj.contains(a)) {
                        let (a, b) = (f(&mut parent, i), f(&mut parent, j));
                        if a != b {
                            parent[a] = b;
                        }
                    }
                }
            }
            let r0 = f(&mut parent, 0);
            (1..ts.len()).any(|i| f(&mut parent, i) != r0)
        })
        .count();

    // Components.
    let comp = triangle_components(nv, tris);
    let mut labels: Vec<u32> = comp.clone();
    labels.sort_unstable();
    labels.dedup();
    let li = |r: u32| labels.binary_search(&r).unwrap();
    let mut cs = vec![ComponentStats { closed: true, ..Default::default() }; labels.len()];
    let centre = pos.iter().fold(DVec3::ZERO, |a, p| a + p.as_dvec3()) / nv.max(1) as f64;
    let mut vert_comp = vec![u32::MAX; nv];
    for (ti, t) in tris.iter().enumerate() {
        if !valid(t) {
            continue;
        }
        let c = li(comp[ti]);
        cs[c].triangles += 1;
        let (p0, p1, p2) = (pos[t[0] as usize].as_dvec3() - centre, pos[t[1] as usize].as_dvec3() - centre, pos[t[2] as usize].as_dvec3() - centre);
        cs[c].signed_volume_mm3 += p0.dot(p1.cross(p2)) / 6.0;
        cs[c].area_mm2 += 0.5 * (p1 - p0).cross(p2 - p0).length();
        for &v in t {
            vert_comp[v as usize] = c as u32;
        }
    }
    for &c in &vert_comp {
        if c != u32::MAX {
            cs[c as usize].vertices += 1;
        }
    }
    for (k, (key, ti)) in unique_edges.iter().enumerate() {
        let _ = key;
        let c = li(comp[*ti as usize]);
        cs[c].edges += 1;
        if edge_tri_count[k] != 2 {
            cs[c].closed = false;
        }
    }
    for c in cs.iter_mut() {
        c.euler = c.vertices as i64 - c.edges as i64 + c.triangles as i64;
        c.genus = if c.closed { (2 - c.euler) / 2 } else { -1 };
    }
    cs.sort_by(|a, b| b.triangles.cmp(&a.triangles));
    st.total_volume_mm3 = cs.iter().map(|c| c.signed_volume_mm3).sum();
    st.total_area_mm2 = cs.iter().map(|c| c.area_mm2).sum();
    st.components = cs;
    st.manifold = st.non_manifold_edges == 0 && st.non_manifold_vertices == 0 && st.degenerate_triangles == 0;
    st.watertight = st.manifold && st.boundary_edges == 0 && st.inconsistent_edges == 0;
    st
}

/// Keeps closed components with positive volume that have at least `min_fraction` of the
/// largest component's triangles; drops floating fragments and closed void shells (negative
/// volume: a bubble inside the material, which is thereby filled). Returns the compacted mesh
/// and (dropped fragments, dropped voids).
pub fn filter_components(pos: Vec<Vec3>, tris: Vec<[u32; 3]>, min_fraction: f32) -> (Vec<Vec3>, Vec<[u32; 3]>, usize, usize) {
    let nv = pos.len();
    let comp = triangle_components(nv, &tris);
    let mut count = vec![0usize; nv];
    let mut vol = vec![0f64; nv];
    let c0 = pos.first().copied().unwrap_or(Vec3::ZERO).as_dvec3();
    for (t, &r) in tris.iter().zip(&comp) {
        count[r as usize] += 1;
        let (p0, p1, p2) = (pos[t[0] as usize].as_dvec3() - c0, pos[t[1] as usize].as_dvec3() - c0, pos[t[2] as usize].as_dvec3() - c0);
        vol[r as usize] += p0.dot(p1.cross(p2)) / 6.0;
    }
    let largest = count.iter().copied().max().unwrap_or(0);
    let thr = ((largest as f32 * min_fraction) as usize).max(1);
    let (mut frag, mut voids) = (0, 0);
    let mut keep_root = vec![false; nv];
    for r in 0..nv {
        if count[r] == 0 {
            continue;
        }
        if vol[r] < 0.0 {
            voids += 1;
        } else if count[r] < thr {
            frag += 1;
        } else {
            keep_root[r] = true;
        }
    }
    let mut remap = vec![u32::MAX; nv];
    let mut out_pos = Vec::with_capacity(nv);
    let mut out_tris = Vec::with_capacity(tris.len());
    for (t, &r) in tris.iter().zip(&comp) {
        if !keep_root[r as usize] {
            continue;
        }
        let mut nt = [0u32; 3];
        for k in 0..3 {
            let v = t[k] as usize;
            if remap[v] == u32::MAX {
                remap[v] = out_pos.len() as u32;
                out_pos.push(pos[v]);
            }
            nt[k] = remap[v];
        }
        out_tris.push(nt);
    }
    (out_pos, out_tris, frag, voids)
}
