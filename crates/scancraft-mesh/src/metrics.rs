use glam::DVec3;
use rayon::prelude::*;
use scancraft_core::Mesh;

#[derive(Debug, Clone, Default)]
pub struct MeshAnalysis {
    pub vertex_count: usize,
    pub triangle_count: usize,
    pub surface_area_mm2: f32,
    /// Enclosed volume (absolute value of the signed volume). Only meaningful when watertight.
    pub volume_mm3: f32,
    /// Closed, edge-manifold and consistently oriented.
    pub is_watertight: bool,
    /// Boundary edges: edges used by exactly one triangle.
    pub open_edges: usize,
    /// Edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Interior edges whose two triangles traverse them in the same direction (flipped faces).
    pub inconsistent_edges: usize,
    /// Triangles with repeated or out-of-range indices (ignored by the other counts).
    pub degenerate_triangles: usize,
    /// Signed volume by the divergence theorem; positive when faces are wound counter-clockwise
    /// seen from outside.
    pub signed_volume_mm3: f32,
}

pub struct MeshMetrics;

impl MeshMetrics {
    pub fn analyze(mesh: &Mesh) -> MeshAnalysis {
        let v_count = mesh.vertices.len();
        let t_count = mesh.triangles.len();
        let mut out = MeshAnalysis { vertex_count: v_count, triangle_count: t_count, ..Default::default() };
        if v_count < 3 || t_count == 0 {
            return out;
        }

        let valid = |t: &[u32; 3]| {
            t.iter().all(|&i| (i as usize) < v_count) && t[0] != t[1] && t[1] != t[2] && t[0] != t[2]
        };
        // Reference point for numerically stable volume sums.
        let c = {
            let b = &mesh.bbox;
            let c = (b.min + b.max) * 0.5;
            if c.is_finite() { c.as_dvec3() } else { DVec3::ZERO }
        };
        let pos = |i: u32| DVec3::from(mesh.vertices[i as usize].position.map(|x| x as f64)) - c;

        let (area, vol, degenerate) = mesh
            .triangles
            .par_iter()
            .fold(
                || (0f64, 0f64, 0usize),
                |(a, v, d), tri| {
                    if !valid(&tri.indices) {
                        return (a, v, d + 1);
                    }
                    let [i0, i1, i2] = tri.indices;
                    let (p0, p1, p2) = (pos(i0), pos(i1), pos(i2));
                    (a + 0.5 * (p1 - p0).cross(p2 - p0).length(), v + p0.dot(p1.cross(p2)) / 6.0, d)
                },
            )
            .reduce(|| (0.0, 0.0, 0), |x, y| (x.0 + y.0, x.1 + y.1, x.2 + y.2));

        // Directed edges (from, to); sorted by undirected key so both directions are adjacent.
        let mut edges: Vec<(u64, bool)> = mesh
            .triangles
            .par_iter()
            .filter(|t| valid(&t.indices))
            .flat_map_iter(|t| {
                let [a, b, c] = t.indices;
                [(a, b), (b, c), (c, a)].into_iter().map(|(u, v)| {
                    let (lo, hi, fwd) = if u < v { (u, v, true) } else { (v, u, false) };
                    (((lo as u64) << 32) | hi as u64, fwd)
                })
            })
            .collect();
        edges.par_sort_unstable();

        let (mut open, mut nonman, mut inconsistent) = (0usize, 0usize, 0usize);
        let mut s = 0;
        while s < edges.len() {
            let mut e = s + 1;
            while e < edges.len() && edges[e].0 == edges[s].0 {
                e += 1;
            }
            match e - s {
                1 => open += 1,
                2 => {
                    if edges[s].1 == edges[s + 1].1 {
                        inconsistent += 1;
                    }
                }
                _ => nonman += 1,
            }
            s = e;
        }

        out.surface_area_mm2 = area as f32;
        out.signed_volume_mm3 = vol as f32;
        out.volume_mm3 = vol.abs() as f32;
        out.open_edges = open;
        out.non_manifold_edges = nonman;
        out.inconsistent_edges = inconsistent;
        out.degenerate_triangles = degenerate;
        out.is_watertight = open == 0 && nonman == 0 && inconsistent == 0 && degenerate == 0;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::Point3D;

    fn tetra() -> Mesh {
        let mut m = Mesh::new();
        for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
            m.add_vertex(Point3D::new(p));
        }
        // Counter-clockwise seen from outside.
        m.add_triangle(0, 2, 1);
        m.add_triangle(0, 1, 3);
        m.add_triangle(0, 3, 2);
        m.add_triangle(1, 2, 3);
        m
    }

    #[test]
    fn tetrahedron_is_closed() {
        let a = MeshMetrics::analyze(&tetra());
        assert!(a.is_watertight, "{a:?}");
        assert_eq!(a.open_edges, 0);
        assert!((a.signed_volume_mm3 - 1.0 / 6.0).abs() < 1e-5, "{a:?}");
    }

    #[test]
    fn open_and_flipped_faces_are_detected() {
        let mut m = tetra();
        m.triangles.pop();
        let a = MeshMetrics::analyze(&m);
        assert!(!a.is_watertight);
        assert_eq!(a.open_edges, 3);

        let mut m = tetra();
        m.triangles[3].indices = [1, 3, 2];
        let a = MeshMetrics::analyze(&m);
        assert!(!a.is_watertight);
        assert_eq!(a.inconsistent_edges, 3);
    }
}
