use scancraft_core::{BoundingBox, PointCloud};

/// Axis-aligned 3D Region of Interest (ROI) crop filter
pub struct RoiCropFilter {
    pub bounds: BoundingBox,
}

impl RoiCropFilter {
    pub fn new(bounds: BoundingBox) -> Self {
        Self { bounds }
    }

    pub fn filter(&self, cloud: &PointCloud) -> PointCloud {
        let mut out = PointCloud::new();
        for p in &cloud.points {
            if self.bounds.contains(p.position) {
                out.push(p.clone());
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::{ColorRgba, Point3D};

    #[test]
    fn test_roi_crop() {
        let mut cloud = PointCloud::new();
        cloud.push(Point3D {
            position: [0.0, 0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::WHITE,
            confidence: 1.0,
        });
        cloud.push(Point3D {
            position: [50.0, 50.0, 50.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::WHITE,
            confidence: 1.0,
        });
        cloud.push(Point3D {
            position: [100.0, 100.0, 100.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::WHITE,
            confidence: 1.0,
        });

        let roi = RoiCropFilter::new(BoundingBox::from_arrays([-10.0, -10.0, -10.0], [60.0, 60.0, 60.0]));
        let filtered = roi.filter(&cloud);
        assert_eq!(filtered.len(), 2);
    }
}
