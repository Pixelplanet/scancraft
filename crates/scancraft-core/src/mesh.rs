use crate::bbox::BoundingBox;
use crate::point::Point3D;
use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Triangle {
    pub indices: [u32; 3],
}

impl Triangle {
    pub fn new(i0: u32, i1: u32, i2: u32) -> Self {
        Self { indices: [i0, i1, i2] }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Mesh {
    pub vertices: Vec<Point3D>,
    pub triangles: Vec<Triangle>,
    pub uv_coordinates: Option<Vec<[f32; 2]>>,
    pub bbox: BoundingBox,
}

impl Mesh {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_geometry(vertices: Vec<Point3D>, triangles: Vec<Triangle>) -> Self {
        let mut bbox = BoundingBox::default();
        for v in &vertices {
            bbox.extend(Vec3::from_slice(&v.position));
        }
        Self {
            vertices,
            triangles,
            uv_coordinates: None,
            bbox,
        }
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty() || self.triangles.is_empty()
    }

    pub fn add_vertex(&mut self, point: Point3D) -> u32 {
        let idx = self.vertices.len() as u32;
        self.bbox.extend(Vec3::from_slice(&point.position));
        self.vertices.push(point);
        idx
    }

    pub fn add_triangle(&mut self, i0: u32, i1: u32, i2: u32) {
        self.triangles.push(Triangle::new(i0, i1, i2));
    }

    /// Recompute smooth vertex normals by accumulating adjacent face normals
    pub fn recompute_normals(&mut self) {
        if self.vertices.is_empty() {
            return;
        }

        let mut acc_normals = vec![Vec3::ZERO; self.vertices.len()];

        for tri in &self.triangles {
            let i0 = tri.indices[0] as usize;
            let i1 = tri.indices[1] as usize;
            let i2 = tri.indices[2] as usize;

            if i0 < self.vertices.len() && i1 < self.vertices.len() && i2 < self.vertices.len() {
                let v0 = Vec3::from_slice(&self.vertices[i0].position);
                let v1 = Vec3::from_slice(&self.vertices[i1].position);
                let v2 = Vec3::from_slice(&self.vertices[i2].position);

                let face_normal = (v1 - v0).cross(v2 - v0);
                let len = face_normal.length();
                if len > 1e-6 {
                    let norm = face_normal / len;
                    acc_normals[i0] += norm;
                    acc_normals[i1] += norm;
                    acc_normals[i2] += norm;
                }
            }
        }

        for (i, v) in self.vertices.iter_mut().enumerate() {
            let n = acc_normals[i];
            let len = n.length();
            if len > 1e-6 {
                let normalized = n / len;
                v.normal = [normalized.x, normalized.y, normalized.z];
            } else {
                v.normal = [0.0, 0.0, 1.0];
            }
        }
    }

    /// Recompute axis-aligned bounding box from vertex positions
    pub fn update_bbox(&mut self) {
        self.bbox = BoundingBox::default();
        for v in &self.vertices {
            self.bbox.extend(Vec3::from_slice(&v.position));
        }
    }
}
