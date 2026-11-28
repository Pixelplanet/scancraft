//! Feature-preserving mesh smoothing: fold repair, then bilateral normal filtering with vertex
//! updating (Zheng, Fu, Au and Tai 2011, "Bilateral Normal Filtering for Mesh Denoising"; vertex
//! update after Sun et al. 2007).
//!
//! Plain Laplacian or Taubin smoothing moves every vertex towards its neighbours: it removes
//! grain, but rounds edges and ribs off and shrinks small features at the same rate. Here the
//! face normals are smoothed first, each with its neighbours weighted by distance and by how
//! similar their normals are (so faces across a crease barely influence each other), and the
//! vertices are then moved to fit the smoothed normals. Flat and gently curved areas become
//! smooth; edges stay sharp.
//!
//! Scans also contain crumpled spots (dark ink, shiny patches: a few points scattered off the
//! surface) whose faces fold over. Normal filtering would keep those as "features", so folded
//! vertices are relaxed towards their neighbours first.

use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::Mesh;

/// Settings of [`FeatureSmoother::smooth`].
#[derive(Debug, Clone)]
pub struct FeatureSmoother {
    /// Normal-filtering iterations (strength). 0 skips smoothing (fold repair still runs).
    pub iterations: usize,
    /// How strongly creases are kept: the normal-difference scale of the range weight. Small
    /// (0.2–0.35) keeps edges crisp; large (≥ 1) smooths almost like a Laplacian.
    pub sigma_normal: f32,
    /// Vertex-update steps after each normal-filtering iteration.
    pub vertex_steps: usize,
    /// Relax vertices around folded faces before filtering.
    pub repair_folds: bool,
}

impl Default for FeatureSmoother {
    fn default() -> Self {
        FeatureSmoother { iterations: 4, sigma_normal: 0.35, vertex_steps: 8, repair_folds: true }
    }
}

/// Vertex -> incident faces, in CSR form.
struct VertexFaces {
    offsets: Vec<u32>,
    faces: Vec<u32>,
}

impl VertexFaces {
    fn build(n: usize, tris: &[[u32; 3]]) -> Self {
        let mut deg = vec![0u32; n + 1];
        for t in tris {
            for &v in t {
                deg[v as usize] += 1;
            }
        }
        let mut offsets = vec![0u32; n + 1];
        for i in 0..n {
            offsets[i + 1] = offsets[i] + deg[i];
        }
        let mut fill = offsets.clone();
        let mut faces = vec![0u32; offsets[n] as usize];
        for (f, t) in tris.iter().enumerate() {
            for &v in t {
                faces[fill[v as usize] as usize] = f as u32;
                fill[v as usize] += 1;
            }
        }
        VertexFaces { offsets, faces }
    }

    #[inline]
    fn of(&self, v: usize) -> &[u32] {
        &self.faces[self.offsets[v] as usize..self.offsets[v + 1] as usize]
    }
}

fn face_geometry(pos: &[Vec3], tris: &[[u32; 3]]) -> (Vec<Vec3>, Vec<Vec3>, Vec<f32>) {
    let data: Vec<(Vec3, Vec3, f32)> = tris
        .par_iter()
        .map(|t| {
            let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
            let cr = (b - a).cross(c - a);
            let len = cr.length();
            ((a + b + c) / 3.0, if len > 0.0 { cr / len } else { Vec3::ZERO }, 0.5 * len)
        })
        .collect();
    (data.iter().map(|d| d.0).collect(), data.iter().map(|d| d.1).collect(), data.iter().map(|d| d.2).collect())
}

impl FeatureSmoother {
    pub fn smooth(&self, mesh: &mut Mesh) {
        let tris: Vec<[u32; 3]> = mesh.triangles.iter().map(|t| t.indices).collect();
        let mut pos: Vec<Vec3> = mesh.vertices.iter().map(|v| Vec3::from_array(v.position)).collect();
        if !self.smooth_positions(&mut pos, &tris) {
            return;
        }
        for (v, p) in mesh.vertices.iter_mut().zip(&pos) {
            v.position = p.to_array();
        }
        mesh.update_bbox();
        mesh.recompute_normals();
    }

    /// [`smooth`](Self::smooth) on bare positions and triangles (topology unchanged). Returns
    /// `false` when there was nothing to smooth.
    pub(crate) fn smooth_positions(&self, pos: &mut Vec<Vec3>, tris: &[[u32; 3]]) -> bool {
        let n = pos.len();
        let tris: Vec<[u32; 3]> = tris.iter().copied().filter(|t| t.iter().all(|&i| (i as usize) < n) && t[0] != t[1] && t[1] != t[2] && t[0] != t[2]).collect();
        if n == 0 || tris.is_empty() {
            return false;
        }
        let vf = VertexFaces::build(n, &tris);
        // Vertex neighbours (through faces) for the fold repair.
        if self.repair_folds {
            repair_folds(pos, &tris, &vf);
        }
        if self.iterations > 0 {
            let (centroids, _, _) = face_geometry(pos, &tris);
            // Spatial scale: mean distance between the centroids of faces sharing a vertex.
            let sigma_c = {
                let step = (n / 20_000).max(1);
                let (sum, cnt) = (0..n)
                    .step_by(step)
                    .map(|v| {
                        let f = vf.of(v);
                        let mut s = 0.0f32;
                        let mut c = 0usize;
                        for i in 0..f.len() {
                            for j in i + 1..f.len() {
                                s += centroids[f[i] as usize].distance(centroids[f[j] as usize]);
                                c += 1;
                            }
                        }
                        (s, c)
                    })
                    .fold((0.0f32, 0usize), |a, b| (a.0 + b.0, a.1 + b.1));
                if cnt > 0 { sum / cnt as f32 } else { 1.0 }
            };
            let inv_2c2 = 1.0 / (2.0 * sigma_c * sigma_c);
            let inv_2s2 = 1.0 / (2.0 * self.sigma_normal * self.sigma_normal);
            for _ in 0..self.iterations {
                let (centroids, normals, areas) = face_geometry(pos, &tris);
                // Filter each face normal over the faces sharing a vertex with it.
                let filtered: Vec<Vec3> = (0..tris.len())
                    .into_par_iter()
                    .map(|f| {
                        let (ci, ni) = (centroids[f], normals[f]);
                        let mut sum = Vec3::ZERO;
                        for &v in &tris[f] {
                            for &g in vf.of(v as usize) {
                                let g = g as usize;
                                let dc = ci.distance_squared(centroids[g]);
                                let dn = (ni - normals[g]).length_squared();
                                sum += normals[g] * (areas[g] * (-dc * inv_2c2).exp() * (-dn * inv_2s2).exp());
                            }
                        }
                        let s = sum.normalize_or_zero();
                        if s == Vec3::ZERO { ni } else { s }
                    })
                    .collect();
                // Move vertices so their faces follow the filtered normals.
                for _ in 0..self.vertex_steps {
                    let (centroids, _, _) = face_geometry(pos, &tris);
                    let next: Vec<Vec3> = (0..n)
                        .into_par_iter()
                        .map(|v| {
                            let f = vf.of(v);
                            if f.is_empty() {
                                return pos[v];
                            }
                            let x = pos[v];
                            let mut d = Vec3::ZERO;
                            for &g in f {
                                let nf = filtered[g as usize];
                                d += nf * nf.dot(centroids[g as usize] - x);
                            }
                            x + d / f.len() as f32
                        })
                        .collect();
                    *pos = next;
                }
            }
        }
        true
    }
}

/// Relaxes vertices whose incident faces disagree strongly with each other (folds, spikes):
/// such a vertex and its ring move to the mean of their neighbours, a few rounds.
fn repair_folds(pos: &mut [Vec3], tris: &[[u32; 3]], vf: &VertexFaces) {
    let n = pos.len();
    for _round in 0..4 {
        let (_, normals, areas) = face_geometry(pos, tris);
        // Area-weighted vertex normal, and whether some incident face points well away from it.
        let bad: Vec<bool> = (0..n)
            .into_par_iter()
            .map(|v| {
                let f = vf.of(v);
                let mut nv = Vec3::ZERO;
                for &g in f {
                    nv += normals[g as usize] * areas[g as usize];
                }
                let nv = nv.normalize_or_zero();
                nv != Vec3::ZERO && f.iter().any(|&g| normals[g as usize].dot(nv) < 0.25)
            })
            .collect();
        // Grow by one ring so the relaxed patch blends in.
        let mut mark = bad.clone();
        let mut any = false;
        for (f, t) in tris.iter().enumerate() {
            let _ = f;
            if t.iter().any(|&v| bad[v as usize]) {
                for &v in t {
                    mark[v as usize] = true;
                }
                any = true;
            }
        }
        if !any {
            return;
        }
        for _ in 0..3 {
            let next: Vec<Vec3> = (0..n)
                .into_par_iter()
                .map(|v| {
                    if !mark[v] {
                        return pos[v];
                    }
                    let mut s = Vec3::ZERO;
                    let mut c = 0.0;
                    for &g in vf.of(v) {
                        for &u in &tris[g as usize] {
                            if u as usize != v {
                                s += pos[u as usize];
                                c += 1.0;
                            }
                        }
                    }
                    if c > 0.0 { s / c } else { pos[v] }
                })
                .collect();
            pos.copy_from_slice(&next);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::Point3D;

    /// A grid over [0, size]² with z from `height`, two triangles per cell.
    fn grid(k: u32, size: f32, height: impl Fn(f32, f32) -> f32) -> Mesh {
        let mut m = Mesh::new();
        for j in 0..=k {
            for i in 0..=k {
                let (x, y) = (i as f32 * size / k as f32, j as f32 * size / k as f32);
                m.add_vertex(Point3D::new([x, y, height(x, y)]));
            }
        }
        for j in 0..k {
            for i in 0..k {
                let a = j * (k + 1) + i;
                m.add_triangle(a, a + 1, a + k + 2);
                m.add_triangle(a, a + k + 2, a + k + 1);
            }
        }
        m
    }

    fn noise(x: f32, y: f32) -> f32 {
        let h = ((x * 12.9898 + y * 78.233).sin() * 43758.547).rem_euclid(1.0);
        (h - 0.5) * 0.1
    }

    #[test]
    fn grain_goes_and_a_crease_stays() {
        // A roof: two planes meeting at x = 10 with a 90° crease, plus 0.05 mm noise.
        let roof = |x: f32, _y: f32| 10.0 - (x - 10.0).abs();
        let mut m = grid(60, 20.0, |x, y| roof(x, y) + noise(x, y));
        let err = |m: &Mesh| {
            let (mut e, mut c) = (0.0f32, 0);
            for v in &m.vertices {
                let [x, y, z] = v.position;
                if (x - 10.0).abs() > 1.5 && x > 1.0 && x < 19.0 && y > 1.0 && y < 19.0 {
                    e += (z - roof(x, y)).abs();
                    c += 1;
                }
            }
            e / c as f32
        };
        let before = err(&m);
        FeatureSmoother::default().smooth(&mut m);
        let after = err(&m);
        let ridge = m.vertices.iter().filter(|v| (v.position[0] - 10.0).abs() < 0.2 && v.position[1] > 5.0 && v.position[1] < 15.0).map(|v| v.position[2]).fold(f32::MIN, f32::max);
        assert!(after < 0.4 * before, "noise left on the planes: {after} mm (input {before})");
        assert!(ridge > 9.75, "the crease was rounded off: ridge at {ridge} (true 10)");
    }

    #[test]
    fn a_fold_is_relaxed() {
        let mut m = grid(20, 20.0, |_, _| 0.0);
        // Throw one vertex far below its neighbours: its faces fold over.
        m.vertices[10 * 21 + 10].position[2] = -3.0;
        FeatureSmoother::default().smooth(&mut m);
        let z = m.vertices[10 * 21 + 10].position[2];
        assert!(z.abs() < 0.3, "the spike is still {z} mm deep");
    }
}
