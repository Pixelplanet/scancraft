use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::Mesh;

/// Taubin (λ|μ) smoothing on the welded mesh: each iteration is a shrinking Laplacian step with
/// factor `lambda` followed by an inflating step with factor `μ < −λ`, which removes noise without
/// the volume loss of plain Laplacian smoothing. Boundary vertices are smoothed along the boundary
/// only, so open patches keep their outline instead of curling inwards. Topology is unchanged.
pub struct LaplacianSmoother {
    pub iterations: usize,
    pub lambda: f32, // Smoothing factor, typically 0.2 to 0.5
}

impl Default for LaplacianSmoother {
    fn default() -> Self {
        Self { iterations: 2, lambda: 0.35 }
    }
}

/// Vertex adjacency in CSR form. Boundary vertices list only their boundary neighbours.
pub(crate) struct Adjacency {
    pub offsets: Vec<u32>,
    pub neighbors: Vec<u32>,
}

impl Adjacency {
    pub fn build(mesh: &Mesh) -> Self {
        let n = mesh.vertices.len();
        let mut edges: Vec<(u32, u32)> = mesh
            .triangles
            .par_iter()
            .filter(|t| t.indices.iter().all(|&i| (i as usize) < n))
            .flat_map_iter(|t| {
                let [a, b, c] = t.indices;
                [(a, b), (b, c), (c, a)].into_iter().filter(|(u, v)| u != v).map(|(u, v)| (u.min(v), u.max(v)))
            })
            .collect();
        edges.par_sort_unstable();

        // Unique edges with their use count.
        let mut uniq: Vec<(u32, u32, bool)> = Vec::with_capacity(edges.len() / 2 + 1);
        let mut boundary = vec![false; n];
        let mut s = 0;
        while s < edges.len() {
            let mut e = s + 1;
            while e < edges.len() && edges[e] == edges[s] {
                e += 1;
            }
            let is_b = e - s == 1;
            let (u, v) = edges[s];
            if is_b {
                boundary[u as usize] = true;
                boundary[v as usize] = true;
            }
            uniq.push((u, v, is_b));
            s = e;
        }

        let keep = |u: u32, is_b: bool| !boundary[u as usize] || is_b;
        let mut deg = vec![0u32; n + 1];
        for &(u, v, b) in &uniq {
            if keep(u, b) {
                deg[u as usize] += 1;
            }
            if keep(v, b) {
                deg[v as usize] += 1;
            }
        }
        let mut offsets = vec![0u32; n + 1];
        for i in 0..n {
            offsets[i + 1] = offsets[i] + deg[i];
        }
        let mut fill = offsets.clone();
        let mut neighbors = vec![0u32; offsets[n] as usize];
        for &(u, v, b) in &uniq {
            if keep(u, b) {
                neighbors[fill[u as usize] as usize] = v;
                fill[u as usize] += 1;
            }
            if keep(v, b) {
                neighbors[fill[v as usize] as usize] = u;
                fill[v as usize] += 1;
            }
        }
        Self { offsets, neighbors }
    }

    #[inline]
    pub fn of(&self, i: usize) -> &[u32] {
        &self.neighbors[self.offsets[i] as usize..self.offsets[i + 1] as usize]
    }
}

impl LaplacianSmoother {
    pub fn new(iterations: usize, lambda: f32) -> Self {
        Self { iterations, lambda: lambda.clamp(0.01, 1.0) }
    }

    pub fn smooth(&self, mesh: &mut Mesh) {
        let n = mesh.vertices.len();
        if n == 0 || self.iterations == 0 || mesh.triangles.is_empty() {
            return;
        }
        let adj = Adjacency::build(mesh);
        // Pass-band kPB = 0.1: 1/λ + 1/μ = kPB.
        let lambda = self.lambda;
        let mu = 1.0 / (0.1 - 1.0 / lambda);

        let mut pos: Vec<Vec3> = mesh.vertices.iter().map(|v| Vec3::from_array(v.position)).collect();
        let mut next = pos.clone();
        let step = |src: &[Vec3], dst: &mut [Vec3], f: f32| {
            dst.par_iter_mut().enumerate().for_each(|(i, d)| {
                let nb = adj.of(i);
                let p = src[i];
                if nb.is_empty() {
                    *d = p;
                    return;
                }
                let mut sum = Vec3::ZERO;
                for &j in nb {
                    sum += src[j as usize];
                }
                *d = p + f * (sum / nb.len() as f32 - p);
            });
        };
        for _ in 0..self.iterations {
            step(&pos, &mut next, lambda);
            std::mem::swap(&mut pos, &mut next);
            step(&pos, &mut next, mu);
            std::mem::swap(&mut pos, &mut next);
        }
        for (v, p) in mesh.vertices.iter_mut().zip(pos) {
            v.position = p.to_array();
        }
        mesh.update_bbox();
        mesh.recompute_normals();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::Point3D;

    #[test]
    fn test_laplacian_smooth() {
        // A flat 5x5 grid with one noisy interior vertex.
        let mut mesh = Mesh::new();
        for y in 0..5 {
            for x in 0..5 {
                let z = if (x, y) == (2, 2) { 5.0 } else { 0.0 };
                mesh.add_vertex(Point3D::new([x as f32, y as f32, z]));
            }
        }
        for y in 0..4u32 {
            for x in 0..4u32 {
                let i = y * 5 + x;
                mesh.add_triangle(i, i + 1, i + 6);
                mesh.add_triangle(i, i + 6, i + 5);
            }
        }
        LaplacianSmoother::new(3, 0.5).smooth(&mut mesh);
        assert!(mesh.vertices[12].position[2] < 2.5, "spike reduced: {:?}", mesh.vertices[12].position);
        // Boundary vertices slide along the (straight) boundary only, so they stay in the plane.
        assert!(mesh.vertices[0].position[2].abs() < 1e-4);
    }
}
