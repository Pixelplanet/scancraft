use glam::Vec3;
use scancraft_core::PointCloud;

/// 3D Plane representation: Ax + By + Cz + D = 0
/// where normal = [A, B, C] is normalized.
#[derive(Debug, Clone, Copy)]
pub struct Plane {
    pub normal: Vec3,
    pub d: f32,
}

impl Plane {
    pub fn from_points(p1: Vec3, p2: Vec3, p3: Vec3) -> Option<Self> {
        let v1 = p2 - p1;
        let v2 = p3 - p1;
        let n = v1.cross(v2);
        let len = n.length();
        if len < 1e-6 {
            return None;
        }
        let normal = n / len;
        let d = -normal.dot(p1);
        Some(Self { normal, d })
    }

    /// Signed distance from point to plane
    pub fn distance(&self, pt: Vec3) -> f32 {
        self.normal.dot(pt) + self.d
    }

    /// Absolute distance from point to plane
    pub fn abs_distance(&self, pt: Vec3) -> f32 {
        self.distance(pt).abs()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneFilterMode {
    /// Prune points within distance tolerance of the plane (removes the plane itself)
    RemoveInliers,
    /// Keep only points above the plane (removes turntable and anything underneath)
    ClipBelow,
}

/// RANSAC based plane detection filter (e.g., turntable or floor removal)
pub struct RansacPlaneFilter {
    pub distance_threshold_mm: f32,
    pub max_iterations: usize,
    pub mode: PlaneFilterMode,
}

impl Default for RansacPlaneFilter {
    fn default() -> Self {
        Self {
            distance_threshold_mm: 2.0,
            max_iterations: 150,
            mode: PlaneFilterMode::ClipBelow,
        }
    }
}

impl RansacPlaneFilter {
    pub fn new(distance_threshold_mm: f32, max_iterations: usize, mode: PlaneFilterMode) -> Self {
        Self {
            distance_threshold_mm,
            max_iterations,
            mode,
        }
    }

    /// Detect the dominant plane using RANSAC
    pub fn detect_plane(&self, cloud: &PointCloud) -> Option<Plane> {
        let n = cloud.len();
        if n < 3 {
            return None;
        }

        let mut best_plane = None;
        let mut max_inliers = 0;

        // Fast deterministic pseudo-random generator
        let mut rng = FastRng(0x8542_9b1a_4e33_1207);

        for _ in 0..self.max_iterations {
            let i1 = rng.gen_range(n);
            let mut i2 = rng.gen_range(n);
            while i2 == i1 {
                i2 = rng.gen_range(n);
            }
            let mut i3 = rng.gen_range(n);
            while i3 == i1 || i3 == i2 {
                i3 = rng.gen_range(n);
            }

            let p1 = Vec3::from_slice(&cloud.points[i1].position);
            let p2 = Vec3::from_slice(&cloud.points[i2].position);
            let p3 = Vec3::from_slice(&cloud.points[i3].position);

            if let Some(plane) = Plane::from_points(p1, p2, p3) {
                // Count inliers
                let mut inliers = 0;
                for p in &cloud.points {
                    let pt = Vec3::from_slice(&p.position);
                    if plane.abs_distance(pt) <= self.distance_threshold_mm {
                        inliers += 1;
                    }
                }

                if inliers > max_inliers {
                    max_inliers = inliers;
                    best_plane = Some(plane);
                }
            }
        }

        best_plane
    }

    /// Detect dominant plane and filter point cloud based on selected mode
    pub fn filter(&self, cloud: &PointCloud) -> (PointCloud, Option<Plane>) {
        if let Some(mut plane) = self.detect_plane(cloud) {
            // Orient plane normal towards the point cloud center (where scanned object resides)
            let center = cloud.bbox.center();
            if plane.distance(center) < 0.0 {
                plane.normal = -plane.normal;
                plane.d = -plane.d;
            }

            let mut out = PointCloud::new();
            for p in &cloud.points {
                let pt = Vec3::from_slice(&p.position);
                let signed_dist = plane.distance(pt);

                match self.mode {
                    PlaneFilterMode::RemoveInliers => {
                        if signed_dist.abs() > self.distance_threshold_mm {
                            out.push(p.clone());
                        }
                    }
                    PlaneFilterMode::ClipBelow => {
                        // Keep points strictly above the plane
                        if signed_dist > self.distance_threshold_mm {
                            out.push(p.clone());
                        }
                    }
                }
            }

            (out, Some(plane))
        } else {
            (cloud.clone(), None)
        }
    }
}

struct FastRng(u64);

impl FastRng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn gen_range(&mut self, max: usize) -> usize {
        (self.next_u64() % (max as u64)) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::{ColorRgba, Point3D};

    #[test]
    fn test_ransac_turntable_removal() {
        let mut cloud = PointCloud::new();

        // 1. Turntable plane at Z = 0.0 (100 points)
        for x in -5..=5 {
            for y in -5..=5 {
                cloud.push(Point3D {
                    position: [x as f32 * 5.0, y as f32 * 5.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    color: ColorRgba::GRAY,
                    confidence: 1.0,
                });
            }
        }

        // 2. Scanned object resting on top at Z = 10.0 to 30.0 (20 points)
        for z in 1..=5 {
            cloud.push(Point3D {
                position: [0.0, 0.0, z as f32 * 5.0],
                normal: [0.0, 1.0, 0.0],
                color: ColorRgba::GREEN,
                confidence: 1.0,
            });
        }

        let filter = RansacPlaneFilter::new(1.0, 200, PlaneFilterMode::ClipBelow);
        let (filtered, plane) = filter.filter(&cloud);

        assert!(plane.is_some());
        let p = plane.unwrap();
        assert!((p.normal.z.abs() - 1.0).abs() < 0.1);

        // Turntable at Z=0 should be clipped, object at Z >= 5 should remain
        assert!(filtered.len() >= 5);
        assert!(filtered.points.iter().all(|pt| pt.position[2] >= 4.0));
    }
}
