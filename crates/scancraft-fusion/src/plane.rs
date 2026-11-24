use glam::{Mat3, Vec3};
use scancraft_core::PointCloud;

/// 3D Plane representation: Ax + By + Cz + D = 0
/// where normal = [A, B, C] is normalized.
#[derive(Debug, Clone, Copy, PartialEq)]
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
    /// Preferred "up" direction and the smallest |normal . up| of a preferred plane: the
    /// largest such plane wins over larger ones (walls). `None`: the largest plane.
    pub up: Option<(Vec3, f32)>,
    /// The side the kept points are on (for example a clicked object point). `None`: along
    /// `up` if set, else the bounding-box centre.
    pub keep_side: Option<Vec3>,
}

impl Default for RansacPlaneFilter {
    fn default() -> Self {
        Self {
            distance_threshold_mm: 2.0,
            max_iterations: 150,
            mode: PlaneFilterMode::ClipBelow,
            up: None,
            keep_side: None,
        }
    }
}

impl RansacPlaneFilter {
    pub fn new(distance_threshold_mm: f32, max_iterations: usize, mode: PlaneFilterMode) -> Self {
        Self {
            distance_threshold_mm,
            max_iterations,
            mode,
            up: None,
            keep_side: None,
        }
    }

    /// Prefers planes facing `up` (|normal . up| >= `min_up`) and keeps the points above them.
    pub fn with_up(mut self, up: Vec3, min_up: f32) -> Self {
        self.up = up.try_normalize().map(|u| (u, min_up));
        self
    }

    /// Keeps the side of the plane that holds `point`.
    pub fn keeping_side_of(mut self, point: Vec3) -> Self {
        self.keep_side = Some(point);
        self
    }

    /// Detect the dominant plane using RANSAC
    pub fn detect_plane(&self, cloud: &PointCloud) -> Option<Plane> {
        let n = cloud.len();
        if n < 3 {
            return None;
        }

        let mut best_plane = None;
        let mut max_inliers = 0;
        let mut best_up: Option<(usize, Plane)> = None;

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
                if let Some((up, min_up)) = self.up {
                    if plane.normal.dot(up).abs() >= min_up && best_up.is_none_or(|(c, _)| inliers > c) {
                        best_up = Some((inliers, plane));
                    }
                }
            }
        }

        best_up.map(|(_, p)| p).or(best_plane)
    }

    /// Detect dominant plane and filter point cloud based on selected mode
    pub fn filter(&self, cloud: &PointCloud) -> (PointCloud, Option<Plane>) {
        if let Some(mut plane) = self.detect_plane(cloud) {
            // Orient the normal towards the kept side: the given point, else up, else the point
            // cloud centre (where the scanned object usually is).
            let flip = match (self.keep_side, self.up) {
                (Some(p), _) => plane.distance(p) < 0.0,
                (None, Some((up, _))) => plane.normal.dot(up) < 0.0,
                (None, None) => plane.distance(cloud.bbox.center()) < 0.0,
            };
            if flip {
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

/// How [`ransac_plane`] turns each candidate normal before it is scored and compared.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NormalOrientation {
    /// Keep the sign the three sampled points give.
    Free,
    /// Towards this point (for example the sensor origin: the side the sensor sees).
    TowardsPoint(Vec3),
    /// Along this direction (for example an "up" hint).
    Along(Vec3),
}

/// Settings of [`ransac_plane`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneSearch {
    pub iterations: usize,
    /// Points closer than this to a candidate plane are its inliers.
    pub inlier_mm: f32,
    /// With normals: an inlier's normal must also have at least this |cos| to the plane normal.
    pub normal_cos: f32,
    pub orientation: NormalOrientation,
    /// Preferred planes: oriented normal . `dir` >= `min`. The largest preferred plane wins over
    /// larger planes that are not; without any preferred candidate the largest plane is returned.
    pub prefer: Option<(Vec3, f32)>,
    /// Seed of the deterministic sampler.
    pub seed: u32,
}

impl Default for PlaneSearch {
    fn default() -> Self {
        Self {
            iterations: 400,
            inlier_mm: 2.0,
            normal_cos: 0.8,
            orientation: NormalOrientation::Free,
            prefer: None,
            seed: 0x9e37_79b9,
        }
    }
}

/// Result of [`ransac_plane`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneCandidate {
    pub plane: Plane,
    /// Inliers among the searched points.
    pub inliers: usize,
    /// Whether the plane meets [`PlaneSearch::prefer`].
    pub preferred: bool,
}

/// RANSAC plane search on `points` (usually a subsample). `normals`, if given, must match
/// `points`; they make inliers agree in direction as well as position.
pub fn ransac_plane(points: &[Vec3], normals: Option<&[Vec3]>, search: &PlaneSearch) -> Option<PlaneCandidate> {
    if points.len() < 3 {
        return None;
    }
    let normals = normals.filter(|n| n.len() == points.len());
    let mut seed = search.seed.max(1);
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as usize % n
    };
    let mut best_preferred: Option<(usize, Plane)> = None;
    let mut best_any: Option<(usize, Plane)> = None;
    for _ in 0..search.iterations {
        let (a, b, c) = (points[rand(points.len())], points[rand(points.len())], points[rand(points.len())]);
        let mut n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let flip = match search.orientation {
            NormalOrientation::Free => false,
            NormalOrientation::TowardsPoint(o) => n.dot(a - o) > 0.0,
            NormalOrientation::Along(dir) => n.dot(dir) < 0.0,
        };
        if flip {
            n = -n;
        }
        let d = n.dot(a);
        let count = match normals {
            None => points.iter().filter(|p| (n.dot(**p) - d).abs() < search.inlier_mm).count(),
            Some(normals) => points
                .iter()
                .zip(normals)
                .filter(|(p, m)| (n.dot(**p) - d).abs() < search.inlier_mm && n.dot(**m).abs() >= search.normal_cos)
                .count(),
        };
        let plane = Plane { normal: n, d: -d };
        if best_any.is_none_or(|(c0, _)| count > c0) {
            best_any = Some((count, plane));
        }
        if search.prefer.is_some_and(|(dir, min)| n.dot(dir) >= min) && best_preferred.is_none_or(|(c0, _)| count > c0) {
            best_preferred = Some((count, plane));
        }
    }
    match (best_preferred, best_any) {
        (Some((inliers, plane)), _) => Some(PlaneCandidate { plane, inliers, preferred: true }),
        (None, Some((inliers, plane))) => Some(PlaneCandidate { plane, inliers, preferred: false }),
        (None, None) => None,
    }
}

/// Least-squares plane through `points`: (unit normal, centroid). The normal's sign is arbitrary.
pub fn fit_plane_lsq(points: &[Vec3]) -> Option<(Vec3, Vec3)> {
    if points.len() < 3 {
        return None;
    }
    let c = points.iter().sum::<Vec3>() / points.len() as f32;
    let mut cov = Mat3::ZERO;
    for p in points {
        let d = *p - c;
        cov += Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    }
    // Smallest eigenvector by inverse iteration on the (slightly regularised) covariance.
    let inv = (cov + Mat3::IDENTITY * (1e-6 * cov.determinant().abs().cbrt() + 1e-6)).inverse();
    let mut n = Vec3::new(0.3, 0.3, 0.9).normalize();
    for _ in 0..40 {
        n = (inv * n).normalize_or_zero();
        if n == Vec3::ZERO {
            return None;
        }
    }
    Some((n, c))
}

/// Refits `plane` by least squares on the points closer than each band in turn (for example
/// `[2.0, 1.5]`), keeping the normal's direction. `None` if a band holds too few points.
pub fn refine_plane(points: &[Vec3], plane: Plane, bands: &[f32]) -> Option<Plane> {
    let mut plane = plane;
    for &band in bands {
        let inliers: Vec<Vec3> = points.iter().filter(|p| plane.abs_distance(**p) < band).copied().collect();
        let (n, c) = fit_plane_lsq(&inliers)?;
        let normal = if n.dot(plane.normal) < 0.0 { -n } else { n };
        plane = Plane { normal, d: -normal.dot(c) };
    }
    Some(plane)
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

    /// A small table (z = 0) in front of a larger wall (y = 60).
    fn table_and_wall() -> Vec<Vec3> {
        let mut pts = Vec::new();
        for i in -10..=10 {
            for j in -10..=10 {
                pts.push(Vec3::new(i as f32 * 3.0, j as f32 * 3.0, 0.0));
            }
        }
        for i in -20..=20 {
            for k in 1..=20 {
                pts.push(Vec3::new(i as f32 * 3.0, 60.0, k as f32 * 3.0));
            }
        }
        pts
    }

    #[test]
    fn ransac_plane_prefers_the_up_facing_plane() {
        let pts = table_and_wall();
        let largest = ransac_plane(&pts, None, &PlaneSearch::default()).unwrap();
        assert!(largest.plane.normal.y.abs() > 0.99 && !largest.preferred, "{largest:?}");
        let search = PlaneSearch { orientation: NormalOrientation::Along(Vec3::Z), prefer: Some((Vec3::Z, 0.5)), ..PlaneSearch::default() };
        let table = ransac_plane(&pts, None, &search).unwrap();
        assert!(table.preferred && table.plane.normal.z > 0.99, "{table:?}");
        assert!(table.inliers >= 21 * 21, "{table:?}");
        let refined = refine_plane(&pts, table.plane, &[2.0, 1.0]).unwrap();
        assert!(refined.normal.z > 0.9999 && refined.d.abs() < 1e-4, "{refined:?}");
    }

    #[test]
    fn ransac_plane_orients_towards_a_point() {
        let pts = table_and_wall();
        let search = PlaneSearch { orientation: NormalOrientation::TowardsPoint(Vec3::new(0.0, 0.0, 50.0)), ..PlaneSearch::default() };
        let wall = ransac_plane(&pts, None, &search).unwrap();
        assert!(wall.plane.normal.y < -0.99, "faces the point: {wall:?}");
    }

    #[test]
    fn filter_with_up_removes_the_table_not_the_wall() {
        let cloud = PointCloud::from_points(table_and_wall().iter().map(|p| Point3D::new(p.to_array())).collect());
        let (kept, plane) = RansacPlaneFilter::new(1.0, 300, PlaneFilterMode::ClipBelow).with_up(Vec3::Z, 0.5).filter(&cloud);
        assert!(plane.unwrap().normal.z > 0.99);
        assert_eq!(kept.len(), 41 * 20, "the wall above the table stays");
        // Keeping the other side of the wall instead.
        let (kept, plane) = RansacPlaneFilter::new(1.0, 300, PlaneFilterMode::ClipBelow).keeping_side_of(Vec3::new(0.0, 100.0, 0.0)).filter(&cloud);
        assert!(plane.unwrap().normal.y > 0.99);
        assert!(kept.is_empty());
    }
}
