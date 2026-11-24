//! Hole filling for welded triangle meshes, after Liepa 2003 ("Filling Holes in Meshes").
//!
//! A hole is a closed loop of boundary edges (edges used by exactly one triangle). Each hole that
//! is selected for filling gets a patch in three steps:
//!
//! 1. **Triangulation** of the boundary loop. Loops of up to [`DP_MAX_EDGES`] edges get the
//!    minimum-weight triangulation by dynamic programming (Barequet–Sharir / Liepa weight: first
//!    the largest dihedral angle inside the patch and against the mesh triangle on every boundary
//!    edge, then the total area). This keeps folds out of the patch and makes it continue the
//!    surface across the boundary. Larger loops are cut recursively along the chord with the best
//!    ratio of length to the shorter arc (a cut across the narrowest part of the hole) until every
//!    piece fits the dynamic programme (O(n³) time and O(n²) memory: on a 2000-edge loop over a
//!    billion steps). The cuts carry no dihedral term; refinement flips them away later.
//! 2. **Refinement** to the density of the surrounding mesh (Liepa §3.2): every boundary vertex
//!    gets a scale σ (mean length of its edges in the original mesh), triangles are split at
//!    their centroid while the centroid is more than σ/√2 away from all three corners, and
//!    interior edges are flipped to the Delaunay configuration (circumsphere test) after every
//!    split and every round.
//! 3. **Fairing** of the new vertices: they minimise the summed squared umbrella Laplacian
//!    (a discrete thin-plate energy, Σ d_v |Δv|²) with the boundary loop and its one ring in the
//!    original mesh fixed, so the patch meets the surface with matching tangent and curvature.
//!    The energy is quadratic, so the minimiser solves a symmetric positive definite system
//!    (a two-ring bi-Laplacian). It is solved exactly by sparse Cholesky in nested-dissection
//!    order: iterative solvers converge far too slowly on it (condition ~n²; preconditioned
//!    conjugate gradients still left vertices half an edge length off after 1000 iterations on a
//!    19 000-vertex patch, which the direct solves fill completely in ~0.5 s). Conjugate gradients remain as
//!    the fallback above [`FAIR_DIRECT_MAX`] unknowns. Fairing runs during refinement too, so the
//!    splits measure the curved patch rather than the flat initial triangulation.
//!
//! Patches only ever add triangles and vertices; the original mesh is not changed (apart from the
//! normals of the boundary vertices). Every patch is checked before it is applied: no degenerate
//! triangles, every patch edge used once in each direction except the boundary loop (used once,
//! opposite to the mesh triangle), and no new edge that already exists in the mesh. A patch that
//! fails is skipped rather than applied.
//!
//! Performance: one vertex → triangle table is built for the whole mesh; boundary detection and
//! loop pairing are local queries on it (O(n) plus sorting the boundary edges), patches are built
//! in parallel from the unchanged mesh and applied one after the other. Holes that share two
//! vertices are rebuilt against the edges of the patches already applied, so two patches never
//! add the same edge.

use glam::{DVec3, Vec3};
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Mesh, Point3D, Triangle};
use std::collections::{HashMap, HashSet};
use std::hash::{BuildHasherDefault, Hasher};

/// Largest loop (in edges) triangulated directly by the O(n³) dynamic programme.
const DP_MAX_EDGES: usize = 200;
/// Refinement rounds (each one roughly triples the triangle count where it splits).
const MAX_REFINE_ROUNDS: usize = 40;
/// Safety cap on the size of one patch (a σ of almost zero would otherwise split forever).
const MAX_PATCH_TRIANGLES: usize = 4_000_000;
/// Cost of a degenerate triangle in the dynamic programme: worse than any dihedral cost
/// (1 − cos θ ∈ [0, 2]), so it is used only when no other triangulation exists.
const DEGENERATE_COST: f64 = 3.0;
/// Largest fairing system solved directly (sparse Cholesky); larger ones use conjugate gradients.
const FAIR_DIRECT_MAX: usize = 150_000;
/// Conjugate-gradient limits of the fallback. The bi-Laplacian is badly conditioned (~n²): on a
/// 19 000-vertex patch 1000 iterations still left vertices half an edge length from the minimum,
/// which is why the direct solve is the default.
const FAIR_MAX_ITERATIONS: usize = 5000;
const FAIR_TOLERANCE: f64 = 1e-6;

/// One boundary loop of a mesh.
#[derive(Debug, Clone)]
pub struct HoleInfo {
    /// Loop vertices in the direction of the mesh's boundary half-edges: for every `i` the
    /// directed edge `vertices[i] → vertices[i + 1]` (cyclic) belongs to an existing triangle.
    /// With counter-clockwise triangles this runs clockwise around the hole seen from outside.
    pub vertices: Vec<u32>,
    pub edge_count: usize,
    pub perimeter_mm: f32,
    /// Largest distance between two loop vertices (exact up to 2048 vertices, a two-sweep
    /// farthest-point estimate above that).
    pub diameter_mm: f32,
    /// Mean of the loop vertices.
    pub centroid: [f32; 3],
}

/// Result of [`HoleFiller::fill`] / [`HoleFiller::fill_hole`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FillReport {
    pub filled: usize,
    /// Holes left open: larger than `max_perimeter_mm`, the kept outer border, or holes whose
    /// patch failed the validity checks.
    pub skipped: usize,
    pub added_triangles: usize,
    pub added_vertices: usize,
}

/// Fills holes of a triangle mesh with refined, faired patches (see the module documentation).
#[derive(Debug, Clone)]
pub struct HoleFiller {
    /// Holes with a longer boundary stay open.
    pub max_perimeter_mm: f32,
    /// Never fill the boundary loop with the largest perimeter: the outer border of an open scan.
    pub keep_largest: bool,
}

impl Default for HoleFiller {
    /// Fill every hole (closes the mesh).
    fn default() -> Self {
        Self { max_perimeter_mm: f32::INFINITY, keep_largest: false }
    }
}

impl HoleFiller {
    pub fn new(max_perimeter_mm: f32, keep_largest: bool) -> Self {
        Self { max_perimeter_mm, keep_largest }
    }

    /// All boundary loops of `mesh`, largest perimeter first.
    pub fn find_holes(mesh: &Mesh) -> Vec<HoleInfo> {
        let vt = VertexTris::build(mesh);
        find_holes_with(mesh, &vt)
    }

    /// Fills every hole with a perimeter up to `max_perimeter_mm` (except the largest loop when
    /// `keep_largest` is set). New vertices take the mean colour and confidence of their hole's
    /// boundary; normals are updated for the new and the boundary vertices only.
    pub fn fill(&self, mesh: &mut Mesh) -> FillReport {
        let vt = VertexTris::build(mesh);
        let holes = find_holes_with(mesh, &vt);
        let selected: Vec<&HoleInfo> = holes
            .iter()
            .enumerate()
            .filter(|(i, h)| !(self.keep_largest && *i == 0) && h.perimeter_mm <= self.max_perimeter_mm)
            .map(|(_, h)| h)
            .collect();
        let mut report = FillReport { skipped: holes.len() - selected.len(), ..Default::default() };

        let none = FxMap::default();
        let patches: Vec<Option<Patch>> =
            selected.par_iter().map(|h| build_patch(mesh, &vt, &h.vertices, &none)).collect();

        let first_new_tri = mesh.triangles.len();
        let mut added_diagonals: FxMap<u32, Vec<u32>> = FxMap::default();
        let mut affected = Vec::new();
        for (hole, patch) in selected.iter().zip(patches) {
            let conflict = |p: &Patch| {
                p.diagonals.iter().any(|&(a, b)| added_diagonals.get(&a).is_some_and(|l| l.contains(&b)))
            };
            let patch = match patch {
                Some(p) if conflict(&p) => build_patch(mesh, &vt, &hole.vertices, &added_diagonals),
                p => p,
            };
            match patch {
                Some(p) => {
                    for &(a, b) in &p.diagonals {
                        added_diagonals.entry(a).or_default().push(b);
                        added_diagonals.entry(b).or_default().push(a);
                    }
                    apply_patch(mesh, &p, &mut report, &mut affected);
                }
                None => report.skipped += 1,
            }
        }
        update_normals(mesh, &vt, first_new_tri, &affected);
        report
    }

    /// Fills one hole (from [`HoleFiller::find_holes`] on the current mesh), ignoring
    /// `max_perimeter_mm` and `keep_largest`. A hole that no longer matches the mesh (edges not
    /// on the boundary, or reversed) is skipped. Builds the vertex → triangle table each call, so
    /// use [`HoleFiller::fill`] for many holes.
    pub fn fill_hole(&self, mesh: &mut Mesh, hole: &HoleInfo) -> FillReport {
        let vt = VertexTris::build(mesh);
        let mut report = FillReport::default();
        let first_new_tri = mesh.triangles.len();
        let mut affected = Vec::new();
        match build_patch(mesh, &vt, &hole.vertices, &FxMap::default()) {
            Some(p) => apply_patch(mesh, &p, &mut report, &mut affected),
            None => report.skipped = 1,
        }
        update_normals(mesh, &vt, first_new_tri, &affected);
        report
    }
}

// ---------------------------------------------------------------------------------------------
// Shared helpers

#[derive(Default, Clone, Copy)]
struct Fx(u64);

impl Hasher for Fx {
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
}

type FxMap<K, V> = HashMap<K, V, BuildHasherDefault<Fx>>;
type FxSet<K> = HashSet<K, BuildHasherDefault<Fx>>;

#[inline]
fn dkey(a: u32, b: u32) -> u64 {
    ((a as u64) << 32) | b as u64
}

#[inline]
fn ukey(a: u32, b: u32) -> u64 {
    dkey(a.min(b), a.max(b))
}

#[inline]
fn valid_tri(t: &[u32; 3], n: usize) -> bool {
    t.iter().all(|&i| (i as usize) < n) && t[0] != t[1] && t[1] != t[2] && t[0] != t[2]
}

#[inline]
fn has_directed(t: &[u32; 3], a: u32, b: u32) -> bool {
    (t[0] == a && t[1] == b) || (t[1] == a && t[2] == b) || (t[2] == a && t[0] == b)
}

#[inline]
fn third(t: [u32; 3], a: u32, b: u32) -> u32 {
    t.into_iter().find(|&x| x != a && x != b).unwrap_or(a)
}

/// True for a (near) zero-area triangle: twice the area is at most 1e-6 of the summed squared
/// edge lengths (a sliver whose height is ~2e-6 of its length), or no area at all.
#[inline]
fn degenerate(cross_len: f64, a: DVec3, b: DVec3, c: DVec3) -> bool {
    let s = (b - a).length_squared() + (c - b).length_squared() + (a - c).length_squared();
    cross_len.is_nan() || cross_len <= 1e-6 * s || cross_len < 1e-14
}

fn dpos(mesh: &Mesh, i: u32) -> DVec3 {
    let p = mesh.vertices[i as usize].position;
    DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64)
}

/// Vertex → incident triangles (CSR) of the valid triangles of a mesh.
struct VertexTris {
    offsets: Vec<u32>,
    tris: Vec<u32>,
}

impl VertexTris {
    fn build(mesh: &Mesh) -> Self {
        let n = mesh.vertices.len();
        let mut offsets = vec![0u32; n + 1];
        for t in &mesh.triangles {
            if valid_tri(&t.indices, n) {
                for &i in &t.indices {
                    offsets[i as usize + 1] += 1;
                }
            }
        }
        for i in 0..n {
            offsets[i + 1] += offsets[i];
        }
        let mut fill = offsets[..n].to_vec();
        let mut tris = vec![0u32; offsets[n] as usize];
        for (ti, t) in mesh.triangles.iter().enumerate() {
            if valid_tri(&t.indices, n) {
                for &i in &t.indices {
                    tris[fill[i as usize] as usize] = ti as u32;
                    fill[i as usize] += 1;
                }
            }
        }
        Self { offsets, tris }
    }

    #[inline]
    fn of(&self, v: u32) -> &[u32] {
        let v = v as usize;
        if v + 1 >= self.offsets.len() {
            return &[];
        }
        &self.tris[self.offsets[v] as usize..self.offsets[v + 1] as usize]
    }

    /// Number of triangles using the undirected edge {a, b}.
    fn edge_uses(&self, mesh: &Mesh, a: u32, b: u32) -> usize {
        self.of(a).iter().filter(|&&t| mesh.triangles[t as usize].indices.contains(&b)).count()
    }

    /// The triangle containing the directed edge a → b.
    fn half_edge_tri(&self, mesh: &Mesh, a: u32, b: u32) -> Option<u32> {
        self.of(a).iter().copied().find(|&t| has_directed(&mesh.triangles[t as usize].indices, a, b))
    }

    /// Sorted unique neighbours of `v`.
    fn neighbours(&self, mesh: &Mesh, v: u32, out: &mut Vec<u32>) {
        out.clear();
        for &t in self.of(v) {
            out.extend(mesh.triangles[t as usize].indices.iter().copied().filter(|&x| x != v));
        }
        out.sort_unstable();
        out.dedup();
    }
}

// ---------------------------------------------------------------------------------------------
// Boundary loops

fn find_holes_with(mesh: &Mesh, vt: &VertexTris) -> Vec<HoleInfo> {
    let n = mesh.vertices.len();
    // Boundary half-edges (from, to) in the direction of their triangle.
    let mut he: Vec<(u32, u32)> = mesh
        .triangles
        .par_iter()
        .filter(|t| valid_tri(&t.indices, n))
        .flat_map_iter(|t| {
            let [a, b, c] = t.indices;
            [(a, b), (b, c), (c, a)].into_iter().filter(move |&(u, v)| vt.edge_uses(mesh, u, v) == 1)
        })
        .collect();
    if he.is_empty() {
        return Vec::new();
    }
    he.par_sort_unstable();
    let nh = he.len();
    let out_range = |v: u32| he.partition_point(|e| e.0 < v)..he.partition_point(|e| e.0 <= v);
    let mut by_to: Vec<u32> = (0..nh as u32).collect();
    by_to.par_sort_unstable_by_key(|&i| (he[i as usize].1, he[i as usize].0));
    let in_range =
        |v: u32| by_to.partition_point(|&i| he[i as usize].1 < v)..by_to.partition_point(|&i| he[i as usize].1 <= v);

    // Successor of every half-edge: the outgoing boundary edge at its end vertex. Vertices with
    // one incoming and one outgoing boundary edge are unambiguous.
    const NONE: u32 = u32::MAX;
    let mut succ: Vec<u32> = (0..nh)
        .into_par_iter()
        .map(|h| {
            let v = he[h].1;
            let o = out_range(v);
            if o.len() == 1 && in_range(v).len() == 1 {
                o.start as u32
            } else {
                NONE
            }
        })
        .collect();

    // Non-manifold boundary vertices (several fans around the vertex, e.g. two holes touching at
    // one vertex): sort all boundary edges at the vertex by angle around its normal and pair each
    // incoming edge with the next outgoing edge counter-clockwise, i.e. across the same gap.
    // Following the triangle fan instead would merge the touching holes into one figure-eight.
    let mut multi: Vec<u32> = Vec::new();
    for w in he.windows(2) {
        if w[0].0 == w[1].0 {
            multi.push(w[0].0);
        }
    }
    for w in by_to.windows(2) {
        if he[w[0] as usize].1 == he[w[1] as usize].1 {
            multi.push(he[w[0] as usize].1);
        }
    }
    multi.sort_unstable();
    multi.dedup();
    let pos = |i: u32| Vec3::from_array(mesh.vertices[i as usize].position);
    let pairs: Vec<(u32, u32)> = multi
        .par_iter()
        .flat_map_iter(|&v| {
            let pv = pos(v);
            let mut nrm = Vec3::ZERO;
            for &t in vt.of(v) {
                let [a, b, c] = mesh.triangles[t as usize].indices;
                nrm += (pos(b) - pos(a)).cross(pos(c) - pos(a)).normalize_or_zero();
            }
            let nrm = nrm.try_normalize().unwrap_or(Vec3::Z);
            let e1 = nrm.any_orthonormal_vector();
            let e2 = nrm.cross(e1);
            let angle = |p: Vec3| {
                let d = p - pv;
                d.dot(e2).atan2(d.dot(e1))
            };
            // (angle, is_outgoing, half-edge)
            let mut items: Vec<(f32, bool, u32)> = Vec::new();
            for &h in &by_to[in_range(v)] {
                items.push((angle(pos(he[h as usize].0)), false, h));
            }
            for h in out_range(v) {
                items.push((angle(pos(he[h].1)), true, h as u32));
            }
            items.sort_by(|a, b| a.0.total_cmp(&b.0));
            let m = items.len();
            let mut claimed = vec![false; m];
            let mut out = Vec::new();
            for s in 0..m {
                if items[s].1 {
                    continue;
                }
                if let Some(k) = (1..m).map(|o| (s + o) % m).find(|&k| items[k].1 && !claimed[k]) {
                    claimed[k] = true;
                    out.push((items[s].2, items[k].2));
                }
            }
            out
        })
        .collect();
    for (a, b) in pairs {
        succ[a as usize] = b;
    }

    // Follow the successors. `succ` is injective, so a walk either closes on its start or runs
    // into a dead end (an open chain from inconsistent orientation), which is dropped.
    let mut visited = vec![false; nh];
    let mut loops: Vec<Vec<u32>> = Vec::new();
    let mut path: Vec<u32> = Vec::new();
    for h0 in 0..nh {
        if visited[h0] {
            continue;
        }
        path.clear();
        let mut h = h0;
        let closed = loop {
            visited[h] = true;
            path.push(he[h].0);
            let s = succ[h];
            if s == NONE {
                break false;
            }
            if s as usize == h0 {
                break true;
            }
            if visited[s as usize] {
                break false;
            }
            h = s as usize;
        };
        if closed {
            split_simple(&path, &mut loops);
        }
    }

    let mut holes: Vec<HoleInfo> = loops.into_par_iter().map(|l| hole_info(mesh, l)).collect();
    holes.sort_by(|a, b| b.perimeter_mm.total_cmp(&a.perimeter_mm));
    holes
}

/// Splits a closed vertex cycle that passes a vertex more than once into simple loops; loops of
/// fewer than three vertices are dropped.
fn split_simple(cycle: &[u32], out: &mut Vec<Vec<u32>>) {
    let mut stack: Vec<u32> = Vec::with_capacity(cycle.len());
    let mut at: FxMap<u32, usize> = FxMap::default();
    for &v in cycle {
        if let Some(&k) = at.get(&v) {
            let sub = stack[k..].to_vec();
            for x in &stack[k + 1..] {
                at.remove(x);
            }
            stack.truncate(k + 1);
            if sub.len() >= 3 {
                out.push(sub);
            }
        } else {
            at.insert(v, stack.len());
            stack.push(v);
        }
    }
    if stack.len() >= 3 {
        out.push(stack);
    }
}

fn hole_info(mesh: &Mesh, vertices: Vec<u32>) -> HoleInfo {
    let p: Vec<DVec3> = vertices.iter().map(|&i| dpos(mesh, i)).collect();
    let n = p.len();
    let perimeter: f64 = (0..n).map(|i| p[i].distance(p[(i + 1) % n])).sum();
    let centroid = p.iter().copied().sum::<DVec3>() / n as f64;
    let diameter = if n <= 2048 {
        let mut d2 = 0f64;
        for i in 0..n {
            for j in i + 1..n {
                d2 = d2.max(p[i].distance_squared(p[j]));
            }
        }
        d2.sqrt()
    } else {
        let far = |q: DVec3| {
            p.iter().copied().max_by(|a, b| a.distance_squared(q).total_cmp(&b.distance_squared(q))).unwrap()
        };
        let a = far(p[0]);
        let b = far(a);
        a.distance(b)
    };
    HoleInfo {
        edge_count: n,
        perimeter_mm: perimeter as f32,
        diameter_mm: diameter as f32,
        centroid: centroid.as_vec3().to_array(),
        vertices,
    }
}

// ---------------------------------------------------------------------------------------------
// Patch construction

/// A finished patch in local indices: `0..ring.len()` are the boundary loop (patch orientation,
/// i.e. the reversed hole loop), the rest are `new_pos`.
struct Patch {
    ring: Vec<u32>,
    new_pos: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    /// New edges between two boundary vertices (global ids).
    diagonals: Vec<(u32, u32)>,
}

/// Builds the patch for the hole loop `lp` (mesh half-edge order), or `None` if the loop does
/// not describe a hole of `mesh` or no valid patch was found. `extra` lists edges between
/// original vertices that earlier patches added (they must not be added again).
fn build_patch(mesh: &Mesh, vt: &VertexTris, lp: &[u32], extra: &FxMap<u32, Vec<u32>>) -> Option<Patch> {
    let l = lp.len();
    if l < 3 {
        return None;
    }
    let nverts = mesh.vertices.len();
    // Patch orientation is opposite to the hole loop: patch edge ring[i] → ring[i + 1] pairs with
    // the mesh half-edge ring[i + 1] → ring[i].
    let ring: Vec<u32> = lp.iter().rev().copied().collect();
    let mut local: FxMap<u32, u32> = FxMap::default();
    for (i, &g) in ring.iter().enumerate() {
        if g as usize >= nverts || local.insert(g, i as u32).is_some() {
            return None;
        }
    }

    // Mesh triangle across every boundary edge, for the dihedral term.
    let mut opp_n: Vec<Option<DVec3>> = Vec::with_capacity(l);
    let mut outside: Vec<u32> = Vec::with_capacity(l);
    for i in 0..l {
        let (a, b) = (ring[(i + 1) % l], ring[i]);
        let t = vt.half_edge_tri(mesh, a, b)?;
        if vt.edge_uses(mesh, a, b) != 1 {
            return None;
        }
        let [x, y, z] = mesh.triangles[t as usize].indices;
        let c = (dpos(mesh, y) - dpos(mesh, x)).cross(dpos(mesh, z) - dpos(mesh, x));
        opp_n.push(c.try_normalize());
        outside.push(t);
    }
    if l == 3 && outside[0] == outside[1] && outside[1] == outside[2] {
        return None; // an isolated triangle, not a hole
    }

    // Scale, the fixed one ring, and the forbidden diagonals (edges that already exist).
    let mut pos: Vec<DVec3> = ring.iter().map(|&g| dpos(mesh, g)).collect();
    let mut sigma: Vec<f64> = Vec::with_capacity(l);
    let mut fixed: Vec<Vec<(u32, DVec3)>> = Vec::with_capacity(l);
    let mut forbidden: FxSet<u64> = FxSet::default();
    let mut nbuf = Vec::new();
    for i in 0..l {
        vt.neighbours(mesh, ring[i], &mut nbuf);
        let mut sum = 0.0;
        let mut f = Vec::with_capacity(nbuf.len());
        for &g in &nbuf {
            let q = dpos(mesh, g);
            sum += q.distance(pos[i]);
            let k = local.get(&g).copied().unwrap_or(u32::MAX);
            f.push((k, q));
        }
        sigma.push(if nbuf.is_empty() { 0.0 } else { sum / nbuf.len() as f64 });
        fixed.push(f);
        let partners = nbuf.iter().chain(extra.get(&ring[i]).into_iter().flatten());
        for g in partners {
            if let Some(&k) = local.get(g) {
                let k = k as usize;
                if k != (i + 1) % l && k != (i + l - 1) % l {
                    forbidden.insert(ukey(i as u32, k as u32));
                }
            }
        }
    }
    let smax = sigma.iter().copied().fold(0.0, f64::max);
    if smax <= 0.0 {
        return None;
    }
    for s in &mut sigma {
        *s = s.max(1e-3 * smax);
    }

    let lu = l as u32;
    let forb = |a: u32, b: u32| a < lu && b < lu && forbidden.contains(&ukey(a, b));
    let mut tris: Vec<[u32; 3]> = Vec::new();
    triangulate(&mut pos, &mut sigma, (0..lu).collect(), opp_n, &forb, &mut tris);

    let mut w = Work { pos, sigma, l: lu, tris, he: FxMap::default(), forbidden: &forbidden };
    for t in 0..w.tris.len() as u32 {
        w.add_edges(t);
    }
    if !w.validate() {
        log::debug!("hole of {l} edges: initial triangulation invalid, skipped");
        return None;
    }
    w.relax_all();
    // Fair whenever the patch has grown by a quarter since the last solve, and at the end. The
    // splits then measure distances on the curved patch, not on the flat initial triangulation,
    // so the density matches the surroundings (fairing only on doubling left a 60 mm sphere cap
    // 12 % sparser and 25 % further off the sphere); the late rounds add only a few vertices
    // each, and solving after every round would cost a full solve per round.
    let mut faired_at = l;
    for _ in 0..MAX_REFINE_ROUNDS {
        if !w.split_round() {
            break;
        }
        w.relax_all();
        if w.pos.len() * 4 > faired_at * 5 {
            w.fair(&fixed);
            faired_at = w.pos.len();
        }
        if w.tris.len() > MAX_PATCH_TRIANGLES {
            break;
        }
    }
    if faired_at != w.pos.len() {
        w.fair(&fixed);
    }
    if !w.validate() {
        log::debug!("hole of {l} edges: refined patch invalid, skipped");
        return None;
    }

    let mut diagonals = Vec::new();
    for t in &w.tris {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let ring_edge = b == (a + 1) % lu || a == (b + 1) % lu;
            if a < lu && b < lu && a < b && !ring_edge {
                diagonals.push((ring[a as usize], ring[b as usize]));
            }
        }
    }
    diagonals.sort_unstable();
    diagonals.dedup();
    Some(Patch {
        new_pos: w.pos[l..].iter().map(|p| p.as_vec3()).collect(),
        tris: w.tris,
        ring,
        diagonals,
    })
}

/// Triangulates the polygon `poly` (local ids, patch orientation). `opp[e]` is the unit normal of
/// the triangle on the other side of polygon edge `e` (`poly[e] → poly[e + 1]`, cyclic), `None`
/// for a cut through the hole. `forb` tells whether a diagonal already exists as an edge.
fn triangulate(
    pos: &mut Vec<DVec3>,
    sigma: &mut Vec<f64>,
    poly: Vec<u32>,
    opp: Vec<Option<DVec3>>,
    forb: &(dyn Fn(u32, u32) -> bool + Sync),
    out: &mut Vec<[u32; 3]>,
) {
    let n = poly.len();
    if n <= DP_MAX_EDGES {
        if !dp_triangulate(pos, &poly, &opp, forb, out) {
            fan(pos, sigma, &poly, out);
        }
        return;
    }
    let Some((a, b)) = best_cut(pos, &poly, forb) else {
        fan(pos, sigma, &poly, out);
        return;
    };
    let p1 = poly[a..=b].to_vec();
    let mut o1 = opp[a..b].to_vec();
    o1.push(None);
    let mut p2 = poly[b..].to_vec();
    p2.extend_from_slice(&poly[..=a]);
    let mut o2 = opp[b..].to_vec();
    o2.extend_from_slice(&opp[..a]);
    o2.push(None);
    triangulate(pos, sigma, p1, o1, forb, out);
    triangulate(pos, sigma, p2, o2, forb, out);
}

/// Cut for a large loop: the chord (a, b) with the smallest ratio of its length to the shorter
/// arc, i.e. across the narrowest part of the hole. Both sides keep at least a quarter of the
/// vertices so the recursion stays O(n² log n) on zig-zag borders, where neighbouring vertices
/// two steps apart would otherwise win.
fn best_cut(pos: &[DVec3], poly: &[u32], forb: &(dyn Fn(u32, u32) -> bool + Sync)) -> Option<(usize, usize)> {
    let n = poly.len();
    let p: Vec<DVec3> = poly.iter().map(|&i| pos[i as usize]).collect();
    let mut s = vec![0f64; n + 1];
    for i in 0..n {
        s[i + 1] = s[i] + p[i].distance(p[(i + 1) % n]);
    }
    let per = s[n];
    let min_side = (n / 4).max(2);
    (0..n)
        .into_par_iter()
        .filter_map(|a| {
            let mut best: Option<(f64, usize, usize)> = None;
            for b in a + min_side..n {
                if n - (b - a) < min_side {
                    break;
                }
                if forb(poly[a], poly[b]) {
                    continue;
                }
                let arc = (s[b] - s[a]).min(per - (s[b] - s[a])).max(1e-12);
                let score = p[a].distance(p[b]) / arc;
                if best.is_none_or(|x| score < x.0) {
                    best = Some((score, a, b));
                }
            }
            best
        })
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, a, b)| (a, b))
}

/// Fallback when no triangulation without existing edges exists: a fan around a new centre
/// vertex (it is faired with the rest of the patch).
fn fan(pos: &mut Vec<DVec3>, sigma: &mut Vec<f64>, poly: &[u32], out: &mut Vec<[u32; 3]>) {
    let n = poly.len();
    let c = poly.iter().map(|&i| pos[i as usize]).sum::<DVec3>() / n as f64;
    let sc = poly.iter().map(|&i| sigma[i as usize]).sum::<f64>() / n as f64;
    let ci = pos.len() as u32;
    pos.push(c);
    sigma.push(sc);
    for i in 0..n {
        out.push([poly[i], poly[(i + 1) % n], ci]);
    }
}

/// Minimum-weight triangulation (Liepa §2): weight = (largest dihedral cost, area),
/// lexicographic. The dihedral cost 1 − cos θ is monotone in the angle and needs no `acos`.
/// Returns false (and adds nothing) when every triangulation uses an existing edge or needs a
/// degenerate triangle.
fn dp_triangulate(
    pos: &[DVec3],
    poly: &[u32],
    opp: &[Option<DVec3>],
    forb: &(dyn Fn(u32, u32) -> bool + Sync),
    out: &mut Vec<[u32; 3]>,
) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    if n == 3 {
        let [a, b, c] = [0, 1, 2].map(|i| pos[poly[i] as usize]);
        if degenerate((b - a).cross(c - a).length(), a, b, c) {
            return false;
        }
        out.push([poly[0], poly[1], poly[2]]);
        return true;
    }
    let p = |i: usize| pos[poly[i] as usize];
    let ix = |i: usize, k: usize| i * n + k;
    let mut cost = vec![f64::INFINITY; n * n];
    let mut area = vec![0f64; n * n];
    let mut lam = vec![u32::MAX; n * n];
    // Unit normal of the triangle chosen for (i, k); zero for a degenerate one (no dihedral).
    let mut nrm = vec![DVec3::ZERO; n * n];
    for i in 0..n - 1 {
        cost[ix(i, i + 1)] = 0.0;
    }
    for d in 2..n {
        for i in 0..n - d {
            let k = i + d;
            let top = i == 0 && k == n - 1;
            if !top && forb(poly[i], poly[k]) {
                continue;
            }
            let (pi, pk) = (p(i), p(k));
            let mut best = (f64::INFINITY, f64::INFINITY, u32::MAX, DVec3::ZERO);
            for m in i + 1..k {
                let (cl, cr) = (cost[ix(i, m)], cost[ix(m, k)]);
                if !cl.is_finite() || !cr.is_finite() {
                    continue;
                }
                let pm = p(m);
                let c = (pm - pi).cross(pk - pi);
                let len = c.length();
                let mut w = cl.max(cr);
                let mut nt = DVec3::ZERO;
                if degenerate(len, pi, pm, pk) {
                    w = w.max(DEGENERATE_COST);
                } else {
                    nt = c / len;
                    let nl = if m == i + 1 { opp[i] } else { Some(nrm[ix(i, m)]) };
                    let nr = if k == m + 1 { opp[m] } else { Some(nrm[ix(m, k)]) };
                    let nc = if top { opp[n - 1] } else { None };
                    for o in [nl, nr, nc].into_iter().flatten() {
                        if o != DVec3::ZERO {
                            w = w.max(1.0 - nt.dot(o));
                        }
                    }
                }
                let a = area[ix(i, m)] + area[ix(m, k)] + 0.5 * len;
                if w < best.0 || (w == best.0 && a < best.1) {
                    best = (w, a, m as u32, nt);
                }
            }
            let j = ix(i, k);
            (cost[j], area[j], lam[j], nrm[j]) = best;
        }
    }
    if cost[ix(0, n - 1)].is_nan() || cost[ix(0, n - 1)] >= DEGENERATE_COST {
        return false;
    }
    let mut stack = vec![(0usize, n - 1)];
    while let Some((i, k)) = stack.pop() {
        if k - i < 2 {
            continue;
        }
        let m = lam[ix(i, k)] as usize;
        out.push([poly[i], poly[m], poly[k]]);
        stack.push((i, m));
        stack.push((m, k));
    }
    true
}

/// A patch under refinement, in local indices (`0..l` boundary loop, the rest new vertices).
struct Work<'a> {
    pos: Vec<DVec3>,
    sigma: Vec<f64>,
    l: u32,
    tris: Vec<[u32; 3]>,
    /// Directed edge → triangle.
    he: FxMap<u64, u32>,
    /// Edges of the original mesh between two boundary vertices (undirected local keys).
    forbidden: &'a FxSet<u64>,
}

impl Work<'_> {
    fn add_edges(&mut self, t: u32) {
        let [a, b, c] = self.tris[t as usize];
        for (u, v) in [(a, b), (b, c), (c, a)] {
            self.he.insert(dkey(u, v), t);
        }
    }

    fn remove_edges(&mut self, t: u32) {
        let [a, b, c] = self.tris[t as usize];
        for (u, v) in [(a, b), (b, c), (c, a)] {
            if self.he.get(&dkey(u, v)) == Some(&t) {
                self.he.remove(&dkey(u, v));
            }
        }
    }

    fn edge_exists(&self, a: u32, b: u32) -> bool {
        self.he.contains_key(&dkey(a, b))
            || self.he.contains_key(&dkey(b, a))
            || (a < self.l && b < self.l && self.forbidden.contains(&ukey(a, b)))
    }

    /// Flips the interior edge a–b to the other diagonal of its quad if the opposite vertex lies
    /// inside the circumsphere of (a, b, c) (Liepa's 3D Delaunay test) and the flip neither
    /// creates an existing edge nor folds the quad. Returns the new edge (c, d).
    fn relax_edge(&mut self, a: u32, b: u32) -> Option<(u32, u32)> {
        let t1 = *self.he.get(&dkey(a, b))?;
        let t2 = *self.he.get(&dkey(b, a))?;
        let c = third(self.tris[t1 as usize], a, b);
        let d = third(self.tris[t2 as usize], b, a);
        if c == d || self.edge_exists(c, d) {
            return None;
        }
        let (pa, pb, pc, pd) = (self.pos[a as usize], self.pos[b as usize], self.pos[c as usize], self.pos[d as usize]);
        let u = pb - pa;
        let w = pc - pa;
        let nn = u.cross(w);
        let n2 = nn.length_squared();
        let s = u.length_squared() + w.length_squared();
        if n2 > 1e-24 * s * s {
            let centre = pa + (w.length_squared() * nn.cross(u) + u.length_squared() * w.cross(nn)) / (2.0 * n2);
            let r2 = centre.distance_squared(pa);
            if centre.distance_squared(pd) >= r2 * (1.0 - 1e-9) {
                return None;
            }
        } // else: (a, b, c) is degenerate, flipping is always welcome if the result is valid
        let o1 = nn;
        let o2 = (pa - pb).cross(pd - pb);
        let n1 = (pd - pa).cross(pc - pa);
        let n2v = (pb - pd).cross(pc - pd);
        let (l1, l2) = (n1.length(), n2v.length());
        if degenerate(l1, pa, pd, pc) || degenerate(l2, pd, pb, pc) {
            return None;
        }
        let old = o1 + o2;
        if n1.dot(old) <= 0.0 || n2v.dot(old) <= 0.0 {
            return None;
        }
        // Do not trade shape for a sharper crease: the new pair may bend by at most
        // max(old bend, 30°).
        let cos_new = n1.dot(n2v) / (l1 * l2);
        let (lo1, lo2) = (o1.length(), o2.length());
        let cos_old = if lo1 > 0.0 && lo2 > 0.0 { o1.dot(o2) / (lo1 * lo2) } else { -1.0 };
        if cos_new < cos_old.min(0.866) - 1e-9 {
            return None;
        }
        self.remove_edges(t1);
        self.remove_edges(t2);
        self.tris[t1 as usize] = [a, d, c];
        self.tris[t2 as usize] = [d, b, c];
        self.add_edges(t1);
        self.add_edges(t2);
        Some((c, d))
    }

    fn relax_all(&mut self) {
        let mut queue: Vec<(u32, u32)> = self
            .he
            .keys()
            .map(|&k| ((k >> 32) as u32, k as u32))
            .filter(|&(a, b)| a < b && self.he.contains_key(&dkey(b, a)))
            .collect();
        queue.sort_unstable();
        let cap = 20 * queue.len() + 1000;
        let mut steps = 0;
        while let Some((a, b)) = queue.pop() {
            steps += 1;
            if steps > cap {
                break;
            }
            if let Some((c, d)) = self.relax_edge(a, b) {
                queue.extend([(a, d), (d, b), (b, c), (c, a)]);
            }
        }
    }

    /// One round of Liepa's centroid splits; true if any triangle was split.
    fn split_round(&mut self) -> bool {
        let count = self.tris.len();
        let mut created = false;
        for t in 0..count {
            let [a, b, c] = self.tris[t];
            let (pa, pb, pc) = (self.pos[a as usize], self.pos[b as usize], self.pos[c as usize]);
            let cen = (pa + pb + pc) / 3.0;
            let sc = (self.sigma[a as usize] + self.sigma[b as usize] + self.sigma[c as usize]) / 3.0;
            let far = [(a, pa), (b, pb), (c, pc)].iter().all(|&(m, pm)| {
                let d = std::f64::consts::SQRT_2 * cen.distance(pm);
                d > sc && d > self.sigma[m as usize]
            });
            if !far {
                continue;
            }
            let m = self.pos.len() as u32;
            self.pos.push(cen);
            self.sigma.push(sc);
            let t = t as u32;
            self.remove_edges(t);
            self.tris[t as usize] = [a, b, m];
            let t2 = self.tris.len() as u32;
            self.tris.push([b, c, m]);
            self.tris.push([c, a, m]);
            for x in [t, t2, t2 + 1] {
                self.add_edges(x);
            }
            for (u, v) in [(a, b), (b, c), (c, a)] {
                self.relax_edge(u, v);
            }
            created = true;
            if self.tris.len() > MAX_PATCH_TRIANGLES {
                break;
            }
        }
        created
    }

    /// Moves the new vertices to the minimum of Σ_v d_v |Δv|² (uniform umbrella Δ) over all patch
    /// vertices, with the boundary loop and its original one ring fixed.
    fn fair(&mut self, fixed: &[Vec<(u32, DVec3)>]) {
        let nv = self.pos.len();
        let l = self.l as usize;
        if nv == l {
            return;
        }
        let mut nb: Vec<Vec<u32>> = vec![Vec::new(); nv];
        for t in &self.tris {
            for (u, v) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                nb[u as usize].push(v);
                nb[v as usize].push(u);
            }
        }
        for x in &mut nb {
            x.sort_unstable();
            x.dedup();
        }
        // Boundary vertices: original neighbours that are not also patch neighbours are fixed.
        let mut fsum = vec![DVec3::ZERO; l];
        let mut deg = vec![0f64; nv];
        for i in 0..l {
            let mut c = 0usize;
            for &(k, q) in &fixed[i] {
                if k != u32::MAX && nb[i].binary_search(&k).is_ok() {
                    continue;
                }
                fsum[i] += q;
                c += 1;
            }
            deg[i] = (nb[i].len() + c) as f64;
        }
        for v in l..nv {
            deg[v] = nb[v].len() as f64;
        }
        let ni = nv - l;

        // Right-hand side: minus the energy gradient at the current positions,
        // g = Kᵀ D⁻¹ (K x + f) on the unknowns (f: the fixed one-ring sums).
        let x0 = &self.pos;
        let q: Vec<DVec3> = (0..nv)
            .map(|v| {
                let mut s: DVec3 = nb[v].iter().map(|&w| x0[w as usize]).sum();
                if v < l {
                    s += fsum[v];
                }
                (s - deg[v] * x0[v]) / deg[v]
            })
            .collect();
        let mut r: Vec<DVec3> = (l..nv)
            .map(|u| {
                let s: DVec3 = nb[u].iter().map(|&v| q[v as usize]).sum();
                deg[u] * q[u] - s
            })
            .collect();

        // H = Kᵀ D⁻¹ K on the unknowns, assembled (two-ring stencil, CSR with sorted columns):
        // H_uw = Σ_v K_vu K_vw / d_v with K_vv = −d_v and K_vw = 1 for neighbours.
        let mut row_ptr = Vec::with_capacity(ni + 1);
        let mut cols: Vec<u32> = Vec::with_capacity(ni * 20);
        let mut vals: Vec<f64> = Vec::with_capacity(ni * 20);
        let mut diag = vec![0f64; ni];
        let mut acc = vec![0f64; nv];
        let mut mark = vec![usize::MAX; nv];
        let mut touched: Vec<usize> = Vec::new();
        row_ptr.push(0);
        for u in l..nv {
            touched.clear();
            let mut add = |w: usize, val: f64, acc: &mut [f64], touched: &mut Vec<usize>| {
                if w >= l {
                    if mark[w] != u {
                        mark[w] = u;
                        acc[w] = 0.0;
                        touched.push(w);
                    }
                    acc[w] += val;
                }
            };
            for v in std::iter::once(u).chain(nb[u].iter().map(|&v| v as usize)) {
                let s = if v == u { -deg[u] } else { 1.0 } / deg[v];
                add(v, -s * deg[v], &mut acc, &mut touched);
                for &w in &nb[v] {
                    add(w as usize, s, &mut acc, &mut touched);
                }
            }
            touched.sort_unstable();
            for &w in &touched {
                if w == u {
                    diag[u - l] = acc[w];
                } else {
                    cols.push((w - l) as u32);
                    vals.push(acc[w]);
                }
            }
            row_ptr.push(cols.len());
        }
        let h = SymSparse { row_ptr, cols, vals, diag };
        let pts: Vec<DVec3> = self.pos[l..].to_vec();
        let direct = if ni <= FAIR_DIRECT_MAX { h.cholesky_solve(&r, &pts) } else { None };
        let x = direct.unwrap_or_else(|| {
            let sbar = self.sigma.iter().sum::<f64>() / nv as f64;
            h.pcg(&mut r, sbar)
        });
        if x.iter().all(|d| d.is_finite()) {
            for (i, d) in x.iter().enumerate() {
                self.pos[l + i] += *d;
            }
        }
    }

    /// The patch is a valid disc spanning the hole: finite positions, no degenerate triangles,
    /// every directed edge at most once, every edge used from both sides except the boundary
    /// loop (used once, in patch direction), and no boundary-to-boundary edge that the mesh
    /// already has.
    fn validate(&self) -> bool {
        let l = self.l;
        if !self.pos.iter().all(|p| p.is_finite()) {
            return false;
        }
        let mut dir: FxSet<u64> = FxSet::default();
        for t in &self.tris {
            let [a, b, c] = t.map(|i| self.pos[i as usize]);
            if degenerate((b - a).cross(c - a).length(), a, b, c) {
                return false;
            }
            for (u, v) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                if !dir.insert(dkey(u, v)) {
                    return false;
                }
            }
        }
        for &k in &dir {
            let (a, b) = ((k >> 32) as u32, k as u32);
            let ring_edge = a < l && b < l && b == (a + 1) % l;
            if ring_edge {
                if dir.contains(&dkey(b, a)) {
                    return false;
                }
            } else {
                if !dir.contains(&dkey(b, a)) {
                    return false;
                }
                if a < l && b < l && self.forbidden.contains(&ukey(a, b)) {
                    return false;
                }
            }
        }
        (0..l).all(|i| dir.contains(&dkey(i, (i + 1) % l)))
    }
}

/// Symmetric sparse matrix: CSR rows with sorted off-diagonal columns, diagonal separately.
struct SymSparse {
    row_ptr: Vec<usize>,
    cols: Vec<u32>,
    vals: Vec<f64>,
    diag: Vec<f64>,
}

impl SymSparse {
    fn n(&self) -> usize {
        self.diag.len()
    }

    fn row(&self, i: usize) -> impl Iterator<Item = (usize, f64)> + '_ {
        let r = self.row_ptr[i]..self.row_ptr[i + 1];
        self.cols[r.clone()].iter().map(|&c| c as usize).zip(self.vals[r].iter().copied())
    }

    /// Solves H x = b by sparse Cholesky (up-looking, as CSparse's `cs_chol`) in a geometric
    /// nested-dissection order of the unknowns' positions `pts`, which keeps the fill near
    /// O(n log n) for a surface patch. `None` if H is not numerically positive definite.
    fn cholesky_solve(&self, b: &[DVec3], pts: &[DVec3]) -> Option<Vec<DVec3>> {
        const NONE: usize = usize::MAX;
        let n = self.n();
        let perm = self.nested_dissection(pts);
        let mut inv = vec![0usize; n];
        for (k, &o) in perm.iter().enumerate() {
            inv[o as usize] = k;
        }
        // Upper part of the permuted matrix by column: entries (i, k) with i < k.
        let mut up_ptr = vec![0usize; n + 1];
        let mut up: Vec<(u32, f64)> = Vec::with_capacity(self.cols.len() / 2 + 1);
        for k in 0..n {
            let row = self.row(perm[k] as usize).map(|(c, v)| (inv[c], v));
            up.extend(row.filter(|&(i, _)| i < k).map(|(i, v)| (i as u32, v)));
            up_ptr[k + 1] = up.len();
        }
        // Elimination tree (with path compression through `ancestor`).
        let mut parent = vec![NONE; n];
        let mut ancestor = vec![NONE; n];
        for k in 0..n {
            for &(i, _) in &up[up_ptr[k]..up_ptr[k + 1]] {
                let mut i = i as usize;
                while i != NONE && i < k {
                    let next = ancestor[i];
                    ancestor[i] = k;
                    if next == NONE {
                        parent[i] = k;
                    }
                    i = next;
                }
            }
        }
        // Row by row: the pattern of row k of L is the elimination-tree reach of column k's
        // upper entries; every column of L receives its entries in increasing row order.
        let mut lcol: Vec<Vec<(u32, f64)>> = vec![Vec::new(); n];
        let mut ldiag = vec![0f64; n];
        let mut x = vec![0f64; n];
        let mut mark = vec![NONE; n];
        let mut stack: Vec<usize> = Vec::new();
        let mut path: Vec<usize> = Vec::new();
        for k in 0..n {
            stack.clear();
            mark[k] = k;
            for &(i, v) in &up[up_ptr[k]..up_ptr[k + 1]] {
                let mut i = i as usize;
                x[i] = v;
                path.clear();
                while mark[i] != k {
                    path.push(i);
                    mark[i] = k;
                    i = parent[i];
                }
                stack.extend(path.iter().rev());
            }
            let mut d = self.diag[perm[k] as usize];
            // `stack` is a sequence of reversed etree paths; read backwards it is topological.
            for &i in stack.iter().rev() {
                let lki = x[i] / ldiag[i];
                x[i] = 0.0;
                for &(j, lji) in &lcol[i] {
                    x[j as usize] -= lji * lki;
                }
                d -= lki * lki;
                lcol[i].push((k as u32, lki));
            }
            if d.is_nan() || d <= 0.0 {
                return None;
            }
            ldiag[k] = d.sqrt();
        }
        // L Lᵀ y = P b, then undo the permutation.
        let mut y: Vec<DVec3> = perm.iter().map(|&o| b[o as usize]).collect();
        for j in 0..n {
            y[j] /= ldiag[j];
            let yj = y[j];
            for &(i, l) in &lcol[j] {
                y[i as usize] -= l * yj;
            }
        }
        for j in (0..n).rev() {
            let mut s = y[j];
            for &(i, l) in &lcol[j] {
                s -= l * y[i as usize];
            }
            y[j] = s / ldiag[j];
        }
        let mut out = vec![DVec3::ZERO; n];
        for (k, &o) in perm.iter().enumerate() {
            out[o as usize] = y[k];
        }
        Some(out)
    }

    /// Elimination order: split the unknowns at the median of their widest coordinate, number
    /// both halves recursively and the separator (vertices of one half coupled to the other)
    /// last. On a surface patch this is the classic nested dissection of a 2D grid.
    fn nested_dissection(&self, pts: &[DVec3]) -> Vec<u32> {
        enum Item {
            Split(Vec<u32>),
            Emit(Vec<u32>),
        }
        let n = self.n();
        let mut order: Vec<u32> = Vec::with_capacity(n);
        let mut side = vec![0u32; n];
        let mut stamp = 0u32;
        let mut work = vec![Item::Split((0..n as u32).collect())];
        while let Some(item) = work.pop() {
            let mut ids = match item {
                Item::Emit(ids) => {
                    order.extend(ids);
                    continue;
                }
                Item::Split(ids) if ids.len() <= 64 => {
                    order.extend(ids);
                    continue;
                }
                Item::Split(ids) => ids,
            };
            let (mut lo, mut hi) = (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY));
            for &i in &ids {
                lo = lo.min(pts[i as usize]);
                hi = hi.max(pts[i as usize]);
            }
            let e = hi - lo;
            let axis = if e.x >= e.y && e.x >= e.z { 0 } else if e.y >= e.z { 1 } else { 2 };
            let mid = ids.len() / 2;
            ids.select_nth_unstable_by(mid, |&a, &b| pts[a as usize][axis].total_cmp(&pts[b as usize][axis]));
            let upper = ids.split_off(mid);
            stamp += 1;
            for &i in &ids {
                side[i as usize] = stamp;
            }
            let (sep, rest): (Vec<u32>, Vec<u32>) =
                upper.into_iter().partition(|&i| self.row(i as usize).any(|(c, _)| side[c] == stamp));
            // Popped in reverse: lower half, upper half, then the separator.
            work.push(Item::Emit(sep));
            work.push(Item::Split(rest));
            work.push(Item::Split(ids));
        }
        order
    }

    /// Fallback for very large patches: conjugate gradients from zero with a symmetric
    /// Gauss–Seidel preconditioner until the residual dropped by `FAIR_TOLERANCE` (`scale` sets
    /// an absolute floor). `r` (the right-hand side) is overwritten with the residual.
    fn pcg(&self, r: &mut [DVec3], scale: f64) -> Vec<DVec3> {
        let n = self.n();
        let split: Vec<usize> = (0..n)
            .map(|i| {
                self.row_ptr[i] + self.cols[self.row_ptr[i]..self.row_ptr[i + 1]].partition_point(|&c| (c as usize) < i)
            })
            .collect();
        let hmul = |p: &[DVec3], out: &mut [DVec3]| {
            for i in 0..n {
                out[i] = self.diag[i] * p[i] + self.row(i).map(|(c, v)| v * p[c]).sum::<DVec3>();
            }
        };
        let sgs = |r: &[DVec3], z: &mut [DVec3]| {
            for i in 0..n {
                let mut s = r[i];
                for k in self.row_ptr[i]..split[i] {
                    s -= self.vals[k] * z[self.cols[k] as usize];
                }
                z[i] = s / self.diag[i];
            }
            for (zi, d) in z.iter_mut().zip(&self.diag) {
                *zi *= *d;
            }
            for i in (0..n).rev() {
                let mut s = z[i];
                for k in split[i]..self.row_ptr[i + 1] {
                    s -= self.vals[k] * z[self.cols[k] as usize];
                }
                z[i] = s / self.diag[i];
            }
        };
        let dot = |a: &[DVec3], b: &[DVec3]| -> DVec3 { a.iter().zip(b).map(|(x, y)| *x * *y).sum() };
        let safe_div = |a: DVec3, b: DVec3| {
            DVec3::new(
                if b.x > 0.0 { a.x / b.x } else { 0.0 },
                if b.y > 0.0 { a.y / b.y } else { 0.0 },
                if b.z > 0.0 { a.z / b.z } else { 0.0 },
            )
        };
        let mut x = vec![DVec3::ZERO; n];
        let mut z = vec![DVec3::ZERO; n];
        sgs(r, &mut z);
        let mut p = z.clone();
        let mut hp = vec![DVec3::ZERO; n];
        let mut rz = dot(r, &z);
        let abs2 = (1e-9 * scale).powi(2) * n as f64;
        let limit = (dot(r, r) * FAIR_TOLERANCE * FAIR_TOLERANCE).max(DVec3::splat(abs2));
        for _ in 0..FAIR_MAX_ITERATIONS {
            if dot(r, r).cmple(limit).all() {
                break;
            }
            hmul(&p, &mut hp);
            let alpha = safe_div(rz, dot(&p, &hp));
            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * hp[i];
            }
            sgs(r, &mut z);
            let rz_new = dot(r, &z);
            let beta = safe_div(rz_new, rz);
            rz = rz_new;
            for i in 0..n {
                p[i] = z[i] + beta * p[i];
            }
        }
        x
    }
}

// ---------------------------------------------------------------------------------------------
// Applying patches

fn apply_patch(mesh: &mut Mesh, p: &Patch, report: &mut FillReport, affected: &mut Vec<u32>) {
    let base = mesh.vertices.len() as u32;
    let l = p.ring.len() as u32;
    let (mut rgba, mut conf) = ([0u32; 4], 0f32);
    for &g in &p.ring {
        let v = &mesh.vertices[g as usize];
        for (s, c) in rgba.iter_mut().zip(v.color.to_array()) {
            *s += c as u32;
        }
        conf += v.confidence;
    }
    let color = rgba.map(|s| (s / l) as u8);
    let color = ColorRgba::new(color[0], color[1], color[2], color[3]);
    let confidence = conf / l as f32;
    for q in &p.new_pos {
        mesh.add_vertex(Point3D { position: q.to_array(), normal: [0.0, 0.0, 1.0], color, confidence });
    }
    let map = |i: u32| if i < l { p.ring[i as usize] } else { base + i - l };
    for t in &p.tris {
        mesh.triangles.push(Triangle::new(map(t[0]), map(t[1]), map(t[2])));
    }
    affected.extend_from_slice(&p.ring);
    affected.extend(base..base + p.new_pos.len() as u32);
    report.filled += 1;
    report.added_triangles += p.tris.len();
    report.added_vertices += p.new_pos.len();
}

/// Normals of the boundary and new vertices from all their triangles (sum of unit face
/// normals, as `Mesh::recompute_normals`), without touching the rest of the mesh.
fn update_normals(mesh: &mut Mesh, vt: &VertexTris, first_new_tri: usize, affected: &[u32]) {
    if affected.is_empty() {
        return;
    }
    let face = |mesh: &Mesh, t: usize| {
        let [a, b, c] = mesh.triangles[t].indices.map(|i| Vec3::from_array(mesh.vertices[i as usize].position));
        (b - a).cross(c - a).normalize_or_zero()
    };
    let mut acc: FxMap<u32, Vec3> = affected.iter().map(|&v| (v, Vec3::ZERO)).collect();
    for t in first_new_tri..mesh.triangles.len() {
        let f = face(mesh, t);
        for i in mesh.triangles[t].indices {
            if let Some(a) = acc.get_mut(&i) {
                *a += f;
            }
        }
    }
    for (&v, a) in acc.iter_mut() {
        for &t in vt.of(v) {
            *a += face(mesh, t as usize);
        }
    }
    for (v, a) in acc {
        mesh.vertices[v as usize].normal = a.try_normalize().unwrap_or(Vec3::Z).to_array();
    }
}

// ---------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metrics::MeshMetrics;

    /// Icosphere of radius `r`, counter-clockwise seen from outside.
    fn icosphere(r: f32, subdivisions: u32) -> Mesh {
        let t = (1.0 + 5f32.sqrt()) / 2.0;
        let mut v: Vec<Vec3> = [
            [-1.0, t, 0.0],
            [1.0, t, 0.0],
            [-1.0, -t, 0.0],
            [1.0, -t, 0.0],
            [0.0, -1.0, t],
            [0.0, 1.0, t],
            [0.0, -1.0, -t],
            [0.0, 1.0, -t],
            [t, 0.0, -1.0],
            [t, 0.0, 1.0],
            [-t, 0.0, -1.0],
            [-t, 0.0, 1.0],
        ]
        .iter()
        .map(|p| Vec3::from_array(*p).normalize())
        .collect();
        let mut f: Vec<[u32; 3]> = vec![
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        for _ in 0..subdivisions {
            let mut mid: HashMap<u64, u32> = HashMap::new();
            let mut nf = Vec::with_capacity(f.len() * 4);
            let mut m = |a: u32, b: u32, v: &mut Vec<Vec3>| {
                *mid.entry(ukey(a, b)).or_insert_with(|| {
                    v.push(((v[a as usize] + v[b as usize]) * 0.5).normalize());
                    v.len() as u32 - 1
                })
            };
            for [a, b, c] in f {
                let ab = m(a, b, &mut v);
                let bc = m(b, c, &mut v);
                let ca = m(c, a, &mut v);
                nf.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
            }
            f = nf;
        }
        let mut mesh = Mesh::from_geometry(
            v.iter().map(|p| Point3D::new((*p * r).to_array())).collect(),
            f.iter().map(|t| Triangle::new(t[0], t[1], t[2])).collect(),
        );
        if MeshMetrics::analyze(&mesh).signed_volume_mm3 < 0.0 {
            for t in &mut mesh.triangles {
                t.indices.swap(1, 2);
            }
        }
        mesh.recompute_normals();
        mesh
    }

    /// Removes the triangles whose centroid lies within `radius` (straight-line distance) of
    /// `centre`.
    fn cut(mesh: &mut Mesh, centre: Vec3, radius: f32) {
        let v = &mesh.vertices;
        mesh.triangles.retain(|t| {
            let c = t.indices.iter().map(|&i| Vec3::from_array(v[i as usize].position)).sum::<Vec3>() / 3.0;
            c.distance(centre) > radius
        });
    }

    fn grid(nx: u32, ny: u32, h: f32) -> Mesh {
        let mut m = Mesh::new();
        for y in 0..=ny {
            for x in 0..=nx {
                m.add_vertex(Point3D::new([x as f32 * h, y as f32 * h, 0.0]));
            }
        }
        let w = nx + 1;
        for y in 0..ny {
            for x in 0..nx {
                let i = y * w + x;
                m.add_triangle(i, i + 1, i + w + 1);
                m.add_triangle(i, i + w + 1, i + w);
            }
        }
        m
    }

    fn mean_edge(mesh: &Mesh, tris: &[Triangle]) -> f32 {
        let p = |i: u32| Vec3::from_array(mesh.vertices[i as usize].position);
        let s: f32 = tris
            .iter()
            .map(|t| {
                let [a, b, c] = t.indices;
                p(a).distance(p(b)) + p(b).distance(p(c)) + p(c).distance(p(a))
            })
            .sum();
        s / (3 * tris.len()) as f32
    }

    /// Manifold, consistently oriented, no duplicate or zero-area triangles.
    fn assert_valid(mesh: &Mesh) {
        let a = MeshMetrics::analyze(mesh);
        assert_eq!(a.non_manifold_edges, 0, "{a:?}");
        assert_eq!(a.inconsistent_edges, 0, "{a:?}");
        assert_eq!(a.degenerate_triangles, 0, "{a:?}");
        let mut seen = HashSet::new();
        for t in &mesh.triangles {
            let mut k = t.indices;
            k.sort_unstable();
            assert!(seen.insert(k), "duplicate triangle {:?}", t.indices);
            let [p0, p1, p2] = t.indices.map(|i| Vec3::from_array(mesh.vertices[i as usize].position));
            assert!((p1 - p0).cross(p2 - p0).length() > 1e-8, "zero-area triangle {:?}", t.indices);
        }
    }

    #[test]
    fn sphere_cap_is_filled_on_the_sphere() {
        let r = 50.0;
        let mut mesh = icosphere(r, 5);
        let orig_tris = mesh.triangles.len();
        let dir = Vec3::new(0.3, 0.2, 1.0).normalize();
        cut(&mut mesh, dir * r, 15.0);
        let before = mesh.triangles.len();
        let base_edge = mean_edge(&mesh, &mesh.triangles);

        let holes = HoleFiller::find_holes(&mesh);
        assert_eq!(holes.len(), 1);
        let h = &holes[0];
        let circle = 2.0 * std::f32::consts::PI * 15.0;
        assert!(h.perimeter_mm > 0.9 * circle && h.perimeter_mm < 1.4 * circle, "{h:?}");
        assert!((h.diameter_mm - 30.0).abs() < 4.0, "{h:?}");
        assert!(Vec3::from_array(h.centroid).normalize().dot(dir) > 0.99);

        let nverts = mesh.vertices.len();
        let rep = HoleFiller::default().fill(&mut mesh);
        assert_eq!(rep.filled, 1, "{rep:?}");
        assert_eq!(rep.added_triangles, mesh.triangles.len() - before);
        let a = MeshMetrics::analyze(&mesh);
        assert!(a.is_watertight, "{a:?}");
        assert_valid(&mesh);
        assert!(HoleFiller::find_holes(&mesh).is_empty());
        let vol = 4.0 / 3.0 * std::f32::consts::PI * r.powi(3);
        assert!((a.volume_mm3 / vol - 1.0).abs() < 0.01, "{a:?}");

        let mut worst = 0f32;
        for v in &mesh.vertices[nverts..] {
            worst = worst.max((Vec3::from_array(v.position).length() - r).abs());
        }
        eprintln!("cap: worst {worst:.3} mm, {rep:?}");
        assert!(worst < 1.0, "patch vertex {worst} mm off the sphere");
        let patch_edge = mean_edge(&mesh, &mesh.triangles[before..]);
        let ratio = patch_edge / base_edge;
        assert!(ratio > 0.5 && ratio < 2.0, "patch edge {patch_edge}, mesh edge {base_edge}");
        // Roughly the density of the removed cap.
        let removed = orig_tris - before;
        assert!(rep.added_triangles > removed / 3 && rep.added_triangles < removed * 3, "{rep:?} vs {removed}");
    }

    #[test]
    fn perimeter_limit_and_keep_largest() {
        let r = 50.0;
        let mut base = icosphere(r, 5);
        cut(&mut base, Vec3::X * r, 4.0);
        cut(&mut base, Vec3::Y * r, 8.0);
        cut(&mut base, Vec3::Z * r, 20.0);
        let holes = HoleFiller::find_holes(&base);
        assert_eq!(holes.len(), 3);
        assert!(holes[0].perimeter_mm > holes[1].perimeter_mm && holes[1].perimeter_mm > holes[2].perimeter_mm);
        assert!(holes[0].perimeter_mm > 100.0 && holes[1].perimeter_mm < 80.0, "{holes:?}");

        let mut m = base.clone();
        let rep = HoleFiller::new(80.0, false).fill(&mut m);
        assert_eq!((rep.filled, rep.skipped), (2, 1), "{rep:?}");
        let left = HoleFiller::find_holes(&m);
        assert_eq!(left.len(), 1);
        assert!(left[0].perimeter_mm > 100.0);
        assert_eq!(MeshMetrics::analyze(&m).open_edges, left[0].edge_count);
        assert_valid(&m);

        let mut m = base.clone();
        let rep = HoleFiller::new(f32::INFINITY, true).fill(&mut m);
        assert_eq!((rep.filled, rep.skipped), (2, 1), "{rep:?}");
        assert_eq!(HoleFiller::find_holes(&m).len(), 1);
        assert_valid(&m);

        // An open disc: the outer border stays, the inner hole is filled.
        let mut disc = grid(40, 40, 1.0);
        let c = Vec3::new(20.0, 20.0, 0.0);
        let v = disc.vertices.clone();
        disc.triangles.retain(|t| {
            let d = t.indices.iter().map(|&i| Vec3::from_array(v[i as usize].position)).sum::<Vec3>() / 3.0;
            let d = d.distance(c);
            d < 19.0 && d > 4.0
        });
        assert_eq!(HoleFiller::find_holes(&disc).len(), 2);
        let rep = HoleFiller::new(f32::INFINITY, true).fill(&mut disc);
        assert_eq!((rep.filled, rep.skipped), (1, 1), "{rep:?}");
        let left = HoleFiller::find_holes(&disc);
        assert_eq!(left.len(), 1);
        assert!(left[0].diameter_mm > 35.0);
        assert_valid(&disc);
    }

    #[test]
    fn flat_patch_stays_planar() {
        let mut m = grid(30, 30, 1.0);
        let v = m.vertices.clone();
        let c = Vec3::new(14.3, 15.6, 0.0);
        m.triangles.retain(|t| {
            let d = t.indices.iter().map(|&i| Vec3::from_array(v[i as usize].position)).sum::<Vec3>() / 3.0;
            (d.x - c.x).abs() > 6.0 || (d.y - c.y).abs() > 4.0 || d.distance(c) > 6.5
        });
        let nv = m.vertices.len();
        let before = m.triangles.len();
        let rep = HoleFiller::new(f32::INFINITY, true).fill(&mut m);
        assert_eq!(rep.filled, 1, "{rep:?}");
        assert!(rep.added_vertices > 0, "{rep:?}");
        for p in &m.vertices[nv..] {
            assert!(p.position[2].abs() < 1e-4, "{:?}", p.position);
            assert!((p.normal[2] - 1.0).abs() < 1e-4, "{:?}", p.normal);
        }
        for t in &m.triangles[before..] {
            let [a, b, c] = t.indices.map(|i| Vec3::from_array(m.vertices[i as usize].position));
            assert!((b - a).cross(c - a).z > 0.0, "flipped patch triangle");
        }
        let ratio = mean_edge(&m, &m.triangles[before..]) / mean_edge(&m, &m.triangles[..before]);
        assert!(ratio > 0.5 && ratio < 2.0, "{ratio}");
        assert_valid(&m);
        assert_eq!(MeshMetrics::analyze(&m).open_edges, 4 * 30);
    }

    #[test]
    fn holes_touching_at_one_vertex() {
        // Remove the cells (9, 9) and (10, 10): they share only the vertex (10, 10), which then
        // has two triangle fans and two boundary gaps.
        let w = 20u32;
        let mut m = grid(w, w, 1.0);
        let cell = |x: u32, y: u32| (2 * (y * w + x)) as usize;
        let mut drop = [cell(9, 9), cell(9, 9) + 1, cell(10, 10), cell(10, 10) + 1];
        drop.sort_unstable();
        for i in drop.iter().rev() {
            m.triangles.remove(*i);
        }
        let holes = HoleFiller::find_holes(&m);
        assert_eq!(holes.len(), 3, "{holes:?}");
        assert_eq!(holes.iter().filter(|h| h.edge_count == 4).count(), 2, "{holes:?}");
        let pinch = 10 * (w + 1) + 10;
        assert!(holes.iter().filter(|h| h.vertices.contains(&pinch)).count() == 2);

        let rep = HoleFiller::new(f32::INFINITY, true).fill(&mut m);
        assert_eq!(rep.filled, 2, "{rep:?}");
        assert_valid(&m);
        assert_eq!(MeshMetrics::analyze(&m).open_edges, 4 * w as usize);

        // Same on a closed surface: two caps of a sphere that touch at one vertex.
        let mut s = icosphere(50.0, 4);
        let tri_ids: Vec<usize> = (0..s.triangles.len()).filter(|&t| s.triangles[t].indices.contains(&0)).collect();
        // Two opposite triangles of the umbrella around vertex 0 plus their neighbourhoods.
        let a = tri_ids[0];
        let [x, y, z] = s.triangles[a].indices;
        let ca = [x, y, z].map(|i| Vec3::from_array(s.vertices[i as usize].position)).iter().sum::<Vec3>() / 3.0;
        let far = *tri_ids
            .iter()
            .max_by(|&&p, &&q| {
                let c = |t: usize| {
                    let p = s.triangles[t].indices.map(|i| Vec3::from_array(s.vertices[i as usize].position));
                    p.iter().sum::<Vec3>() / 3.0
                };
                c(p).distance(ca).total_cmp(&c(q).distance(ca))
            })
            .unwrap();
        let (ka, kb) = (a.min(far), a.max(far));
        s.triangles.remove(kb);
        s.triangles.remove(ka);
        let holes = HoleFiller::find_holes(&s);
        assert_eq!(holes.len(), 2, "{holes:?}");
        assert!(holes.iter().all(|h| h.vertices.contains(&0) && h.edge_count == 3));
        let rep = HoleFiller::default().fill(&mut s);
        assert_eq!(rep.filled, 2, "{rep:?}");
        assert!(MeshMetrics::analyze(&s).is_watertight);
        assert_valid(&s);
    }

    #[test]
    fn large_hole_uses_cut_fallback() {
        let r = 50.0;
        let mut mesh = icosphere(r, 6);
        let before_cut = mesh.triangles.len();
        cut(&mut mesh, Vec3::new(-0.2, 0.5, 0.8).normalize() * r, 30.0);
        let removed = before_cut - mesh.triangles.len();
        let holes = HoleFiller::find_holes(&mesh);
        assert_eq!(holes.len(), 1);
        assert!(holes[0].edge_count > DP_MAX_EDGES, "{}", holes[0].edge_count);
        let nverts = mesh.vertices.len();
        let t = std::time::Instant::now();
        let rep = HoleFiller::default().fill(&mut mesh);
        let dt = t.elapsed();
        assert_eq!(rep.filled, 1, "{rep:?}");
        let a = MeshMetrics::analyze(&mesh);
        assert!(a.is_watertight, "{a:?}");
        assert_valid(&mesh);
        let mut worst = 0f32;
        for v in &mesh.vertices[nverts..] {
            worst = worst.max((Vec3::from_array(v.position).length() - r).abs());
        }
        eprintln!(
            "large hole: {} edges, {} triangles removed, {rep:?}, worst {worst:.3} mm, {dt:.2?}",
            holes[0].edge_count, removed
        );
        assert!(worst < 2.0, "patch vertex {worst} mm off the sphere");
    }

    #[test]
    fn marching_cubes_cube_with_holes_on_faces_edges_and_corners() {
        // Marching-cubes borders zig-zag and run straight along the grid (collinear vertices).
        let (side, h) = (40.0f32, 20.0f32);
        let mut cloud = scancraft_core::PointCloud::new();
        let k = 80;
        for axis in 0..3 {
            for sign in [-1.0f32, 1.0] {
                for i in 0..=k {
                    for j in 0..=k {
                        let mut p = [0.0; 3];
                        p[axis] = sign * h;
                        p[(axis + 1) % 3] = -h + i as f32 * side / k as f32;
                        p[(axis + 2) % 3] = -h + j as f32 * side / k as f32;
                        let mut pt = Point3D::new(p);
                        pt.normal = [0.0; 3];
                        pt.normal[axis] = sign;
                        cloud.push(pt);
                    }
                }
            }
        }
        let mut mesh = crate::MarchingCubes::new(1.0).reconstruct(&cloud);
        assert!(MeshMetrics::analyze(&mesh).is_watertight);
        let vol0 = MeshMetrics::analyze(&mesh).volume_mm3;
        cut(&mut mesh, Vec3::new(0.0, 0.0, h), 5.0); // face
        cut(&mut mesh, Vec3::new(3.0, -6.0, -h), 2.5); // small face hole
        cut(&mut mesh, Vec3::new(h, h, 0.0), 6.0); // across an edge
        cut(&mut mesh, Vec3::new(-h, -h, -h), 7.0); // corner
        let holes = HoleFiller::find_holes(&mesh);
        assert_eq!(holes.len(), 4, "{holes:?}");
        let rep = HoleFiller::default().fill(&mut mesh);
        assert_eq!(rep.filled, 4, "{rep:?}");
        let a = MeshMetrics::analyze(&mesh);
        assert!(a.is_watertight, "{a:?}");
        assert_valid(&mesh);
        // The patches round the edge and the corner off, but stay close to the cut volume.
        assert!((a.volume_mm3 / vol0 - 1.0).abs() < 0.01, "{} vs {vol0}", a.volume_mm3);
    }

    #[test]
    fn many_random_overlapping_holes() {
        // Overlapping cuts leave pinched vertices, thin bridges and odd loop shapes.
        let sphere = icosphere(50.0, 5);
        for seed in 1..=12u64 {
            random_holes_case(&sphere, seed);
        }
    }

    fn random_holes_case(sphere: &Mesh, seed: u64) {
        let r = 50.0;
        let mut mesh = sphere.clone();
        let mut s = 0x2545_f491_4f6c_dd1du64.wrapping_mul(seed);
        let mut rnd = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / (1u64 << 24) as f32
        };
        for _ in 0..80 {
            let d = Vec3::new(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5).normalize_or(Vec3::X);
            cut(&mut mesh, d * r, 1.0 + 6.0 * rnd());
        }
        // A triangle left floating (all three neighbours cut away) is not a hole when it is
        // alone: it stays as it is (a back face would make a zero-volume pillow; removing stray
        // components is the caller's job). One that still touches the mesh at a vertex is part
        // of the neighbouring hole's loop and gets filled in.
        let vt = VertexTris::build(&mesh);
        let floating = mesh
            .triangles
            .iter()
            .filter(|t| {
                let [a, b, c] = t.indices;
                [(a, b), (b, c), (c, a)].iter().all(|&(u, v)| vt.edge_uses(&mesh, u, v) == 1)
            })
            .count();
        let holes = HoleFiller::find_holes(&mesh);
        assert!(holes.len() > 20, "seed {seed}: {}", holes.len());
        let rep = HoleFiller::default().fill(&mut mesh);
        assert_eq!(rep.filled + rep.skipped, holes.len(), "seed {seed}: {rep:?}");
        assert!(rep.skipped <= floating, "seed {seed}: {rep:?}, {floating} floating triangles");
        let a = MeshMetrics::analyze(&mesh);
        assert_eq!(a.open_edges, 3 * rep.skipped, "seed {seed}: {a:?}");
        assert_valid(&mesh);
    }

    #[test]
    fn fill_hole_rejects_stale_or_reversed_loops() {
        let mut mesh = icosphere(50.0, 4);
        cut(&mut mesh, Vec3::Z * 50.0, 10.0);
        let h = HoleFiller::find_holes(&mesh).remove(0);
        let mut rev = h.clone();
        rev.vertices.reverse();
        let f = HoleFiller::default();
        assert_eq!(f.fill_hole(&mut mesh, &rev).skipped, 1);
        assert_eq!(f.fill_hole(&mut mesh, &h).filled, 1);
        assert!(MeshMetrics::analyze(&mesh).is_watertight);
        // The same hole again: its edges are no longer on the boundary.
        assert_eq!(f.fill_hole(&mut mesh, &h).skipped, 1);
    }
}
