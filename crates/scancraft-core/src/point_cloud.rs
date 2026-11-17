use crate::bbox::BoundingBox;
use crate::point::Point3D;
use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PointCloud {
    pub points: Vec<Point3D>,
    pub bbox: BoundingBox,
}

impl PointCloud {
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            bbox: BoundingBox::default(),
        }
    }

    pub fn from_points(points: Vec<Point3D>) -> Self {
        let mut bbox = BoundingBox::default();
        for p in &points {
            bbox.extend(Vec3::from_slice(&p.position));
        }
        Self { points, bbox }
    }

    pub fn push(&mut self, point: Point3D) {
        self.bbox.extend(Vec3::from_slice(&point.position));
        self.points.push(point);
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn clear(&mut self) {
        self.points.clear();
        self.bbox = BoundingBox::default();
    }

    pub fn bounding_box(&self) -> BoundingBox {
        self.bbox
    }

    /// Recompute bounding box
    pub fn recompute_bbox(&mut self) {
        let mut bbox = BoundingBox::default();
        for p in &self.points {
            bbox.extend(Vec3::from_slice(&p.position));
        }
        self.bbox = bbox;
    }
}
