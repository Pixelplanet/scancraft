//! The reference mesh of a true part: surface nets on a grid of its distance function, every
//! vertex moved onto the exact surface. Sharp edges come out bevelled by up to about one cell;
//! the shape in `truth.json` is the exact reference.

use std::collections::HashMap;

use glam::DVec3;
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Mesh, Point3D, Triangle};

use crate::sdf::{project_to_surface, sdf_normal, v3, V3};

/// Largest grid the reference mesh is made on (grid points); a coarser cell is used beyond it.
pub const MAX_GRID_POINTS: usize = 40_000_000;

/// Surface-nets mesh of the zero set of `f` within `bounds`, cell `cell` mm (made coarser if the
/// grid would exceed [`MAX_GRID_POINTS`]). Returns the mesh and the cell used.
pub fn reference_mesh(f: &(impl Fn(DVec3) -> f64 + Sync), bounds: [V3; 2], cell: f64) -> (Mesh, f64) {
    let mut cell = cell;
    let size = v3(bounds[1]) - v3(bounds[0]);
    loop {
        let n = (size / cell).ceil() + DVec3::splat(5.0);
        if n.x * n.y * n.z <= MAX_GRID_POINTS as f64 {
            break;
        }
        cell *= 1.25;
    }
    let lo = v3(bounds[0]) - DVec3::splat(2.0 * cell);
    let dims = [0, 1, 2].map(|i| (size[i] / cell).ceil() as usize + 5);
    let [nx, ny, nz] = dims;
    let node = |i: usize, j: usize, k: usize| lo + DVec3::new(i as f64, j as f64, k as f64) * cell;
    let values: Vec<f32> = (0..nz)
        .into_par_iter()
        .flat_map_iter(|k| {
            let mut slab = Vec::with_capacity(nx * ny);
            for j in 0..ny {
                for i in 0..nx {
                    slab.push(f(node(i, j, k)) as f32);
                }
            }
            slab.into_iter()
        })
        .collect();
    let at = |i: usize, j: usize, k: usize| values[(k * ny + j) * nx + i];

    // One vertex per cell with a sign change: mean of its edge crossings, then onto the surface.
    const CORNERS: [[usize; 3]; 8] = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]];
    const EDGES: [[usize; 2]; 12] = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
    let cell_vertices: Vec<(usize, DVec3)> = (0..nz - 1)
        .into_par_iter()
        .flat_map_iter(|k| {
            let mut out = Vec::new();
            for j in 0..ny - 1 {
                for i in 0..nx - 1 {
                    let v: [f32; 8] = CORNERS.map(|c| at(i + c[0], j + c[1], k + c[2]));
                    let inside = v.iter().filter(|x| **x < 0.0).count();
                    if inside == 0 || inside == 8 {
                        continue;
                    }
                    let mut sum = DVec3::ZERO;
                    let mut count = 0.0;
                    for [a, b] in EDGES {
                        if (v[a] < 0.0) != (v[b] < 0.0) {
                            let t = (v[a] / (v[a] - v[b])) as f64;
                            let pa = node(i + CORNERS[a][0], j + CORNERS[a][1], k + CORNERS[a][2]);
                            let pb = node(i + CORNERS[b][0], j + CORNERS[b][1], k + CORNERS[b][2]);
                            sum += pa + (pb - pa) * t;
                            count += 1.0;
                        }
                    }
                    let mean = sum / count;
                    let projected = project_to_surface(f, mean);
                    let p = if (projected - mean).length() < 1.5 * cell && f(projected).abs() < 1e-3 { projected } else { mean };
                    out.push(((k * ny + j) * nx + i, p));
                }
            }
            out.into_iter()
        })
        .collect();
    let index: HashMap<usize, u32> = cell_vertices.iter().enumerate().map(|(n, (c, _))| (*c, n as u32)).collect();

    // A quad round every grid edge with a sign change, facing out of the material.
    let cell_id = |c: [usize; 3]| (c[2] * ny + c[1]) * nx + c[0];
    let quads: Vec<[u32; 4]> = (0..nz)
        .into_par_iter()
        .flat_map_iter(|k| {
            let mut out = Vec::new();
            for j in 0..ny {
                for i in 0..nx {
                    let base = [i, j, k];
                    for axis in 0..3 {
                        let mut end = base;
                        end[axis] += 1;
                        if end[axis] >= dims[axis] {
                            continue;
                        }
                        let (b, c) = ((axis + 1) % 3, (axis + 2) % 3);
                        if base[b] == 0 || base[c] == 0 || base[b] >= dims[b] - 1 || base[c] >= dims[c] - 1 {
                            continue;
                        }
                        let v0 = at(base[0], base[1], base[2]);
                        let v1 = at(end[0], end[1], end[2]);
                        if (v0 < 0.0) == (v1 < 0.0) {
                            continue;
                        }
                        let around = [(1, 1), (0, 1), (0, 0), (1, 0)].map(|(db, dc)| {
                            let mut q = base;
                            q[b] -= db;
                            q[c] -= dc;
                            index.get(&cell_id(q)).copied()
                        });
                        let [Some(a0), Some(a1), Some(a2), Some(a3)] = around else { continue };
                        // (b, c) counter-clockwise round +axis; the outside is +axis when v0 is inside.
                        out.push(if v0 < 0.0 { [a0, a1, a2, a3] } else { [a3, a2, a1, a0] });
                    }
                }
            }
            out.into_iter()
        })
        .collect();

    let mut mesh = Mesh::new();
    mesh.vertices = cell_vertices
        .iter()
        .map(|(_, p)| {
            let n = sdf_normal(f, *p);
            Point3D { position: p.as_vec3().to_array(), normal: n.as_vec3().to_array(), color: ColorRgba::new(200, 200, 200, 255), confidence: 1.0 }
        })
        .collect();
    let pos = |i: u32| cell_vertices[i as usize].1;
    for [a, b, c, d] in quads {
        if (pos(a) - pos(c)).length_squared() <= (pos(b) - pos(d)).length_squared() {
            mesh.triangles.push(Triangle::new(a, b, c));
            mesh.triangles.push(Triangle::new(a, c, d));
        } else {
            mesh.triangles.push(Triangle::new(a, b, d));
            mesh.triangles.push(Triangle::new(b, c, d));
        }
    }
    (mesh, cell)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::Shape;

    #[test]
    fn sphere_mesh_is_on_the_surface_closed_and_outward() {
        let s = Shape::Sphere { center: [1.0, 2.0, 3.0], radius: 10.0 };
        let f = |p: DVec3| s.sdf(p);
        let (mesh, cell) = reference_mesh(&f, [[-9.0, -8.0, -7.0], [11.0, 12.0, 13.0]], 0.5);
        assert_eq!(cell, 0.5);
        assert!(mesh.triangles.len() > 5000);
        for v in &mesh.vertices {
            assert!(s.sdf(DVec3::from_array(v.position.map(f64::from))).abs() < 1e-4);
        }
        // Every edge used twice (closed), triangles face out.
        let mut edges: HashMap<(u32, u32), i32> = HashMap::new();
        let mut volume = 0.0;
        for t in &mesh.triangles {
            let [a, b, c] = t.indices;
            for (x, y) in [(a, b), (b, c), (c, a)] {
                *edges.entry((x.min(y), x.max(y))).or_default() += 1;
            }
            let p = |i: u32| DVec3::from_array(mesh.vertices[i as usize].position.map(f64::from)) - DVec3::new(1.0, 2.0, 3.0);
            volume += p(a).dot(p(b).cross(p(c))) / 6.0;
        }
        assert!(edges.values().all(|n| *n == 2));
        let exact = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
        assert!((volume - exact).abs() / exact < 0.01, "{volume} vs {exact}");
    }
}
