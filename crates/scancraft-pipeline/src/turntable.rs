//! Turntable capture (docs/architecture/scan-sessions.md section 7).
//!
//! With the scanner fixed and the object turning, the static table and background dominate
//! geometric tracking, which then reports "no motion" and smears the object. [`TurntableFilter`]
//! wraps any [`DepthSource`]: from the first frames it fits the table / turntable plane and finds
//! the object standing on it, then passes on only the points inside a cylinder above the plane
//! (the object and anything turning with it). The depth image is passed on unchanged: it is still
//! a valid free-space measurement for the tracker's visibility check and carving.

use crate::{DepthSource, FrameGeometry};
use glam::{Vec2, Vec3};
use scancraft_core::{PointCloud, SensorFrame};
use scancraft_fusion::{cell_components, ransac_plane, refine_plane, CellComponents, NormalOrientation, PlaneSearch};
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
pub struct TurntableConfig {
    /// Frames whose points are pooled to find the plane and the object.
    pub init_frames: usize,
    /// Points closer than this to the plane belong to it and are removed.
    pub plane_margin_mm: f32,
    /// Cylinder radius around the turntable axis; `None`: estimated from the object.
    pub radius_mm: Option<f32>,
    /// Cylinder height above the plane; `None`: estimated from the object.
    pub height_mm: Option<f32>,
    /// Estimated radius = object extent around its centre * `radius_factor` + `auto_margin_mm`
    /// (the object shows other sides as it turns, and its centre is not exactly on the axis).
    pub radius_factor: f32,
    /// Added to estimated radius and height.
    pub auto_margin_mm: f32,
    /// Upper bound of the estimated radius.
    pub max_radius_mm: f32,
    /// The table normal must have at least this component along the sensor's up axis (+Z), so a
    /// wall facing the scanner is not taken for the table. Lower it for steep top-down views.
    pub min_up: f32,
}

impl Default for TurntableConfig {
    fn default() -> Self {
        Self {
            init_frames: 3,
            plane_margin_mm: 3.0,
            radius_mm: None,
            height_mm: None,
            radius_factor: 1.5,
            auto_margin_mm: 25.0,
            max_radius_mm: 250.0,
            min_up: 0.3,
        }
    }
}

/// The capture volume, in the (static) sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurntableRegion {
    /// Unit normal of the turntable plane, pointing to the object side.
    pub normal: Vec3,
    /// The plane is `normal . p = offset`.
    pub offset: f32,
    /// Where the cylinder axis meets the plane.
    pub center: Vec3,
    pub radius_mm: f32,
    pub height_mm: f32,
    /// Plane margin used by [`contains`](Self::contains).
    pub plane_margin_mm: f32,
}

impl TurntableRegion {
    /// Signed height above the plane.
    pub fn height_of(&self, p: Vec3) -> f32 {
        self.normal.dot(p) - self.offset
    }

    pub fn contains(&self, p: Vec3) -> bool {
        let h = self.height_of(p);
        if h <= self.plane_margin_mm || h > self.height_mm {
            return false;
        }
        let radial = (p - self.center) - self.normal * self.normal.dot(p - self.center);
        radial.length_squared() <= self.radius_mm * self.radius_mm
    }
}

/// Depth source wrapper for turntable capture (see the module docs).
pub struct TurntableFilter {
    inner: Box<dyn DepthSource>,
    config: TurntableConfig,
    region: Option<TurntableRegion>,
    pool: Vec<Vec3>,
    pooled_frames: usize,
}

impl TurntableFilter {
    pub fn new(inner: Box<dyn DepthSource>, config: TurntableConfig) -> Self {
        Self { inner, config, region: None, pool: Vec::new(), pooled_frames: 0 }
    }

    /// Uses a known capture volume instead of estimating one (e.g. adjusted by the user).
    pub fn with_region(mut self, region: TurntableRegion) -> Self {
        self.region = Some(region);
        self
    }

    /// The capture volume, once estimated.
    pub fn region(&self) -> Option<&TurntableRegion> {
        self.region.as_ref()
    }
}

impl DepthSource for TurntableFilter {
    fn name(&self) -> &'static str {
        "turntable"
    }

    fn sensor_static(&self) -> bool {
        true
    }

    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry> {
        let geometry = self.inner.process(frame)?;
        if self.region.is_none() {
            let step = (geometry.cloud.len() / 6000).max(1);
            self.pool.extend(geometry.cloud.points.iter().step_by(step).map(|p| Vec3::from(p.position)));
            self.pooled_frames += 1;
            if self.pooled_frames < self.config.init_frames.max(1) {
                return None;
            }
            self.region = estimate_region(&self.pool, &self.config);
            self.pool.clear();
            self.pooled_frames = 0;
            match &self.region {
                Some(r) => log::info!(
                    "turntable: plane normal {:.2?}, centre {:.1?}, radius {:.0} mm, height {:.0} mm",
                    r.normal,
                    r.center,
                    r.radius_mm,
                    r.height_mm
                ),
                None => {
                    log::warn!("turntable: no table plane with an object on it found yet");
                    return None;
                }
            }
        }
        let region = self.region?;
        let mut cloud = PointCloud::new();
        for p in geometry.cloud.points.iter().filter(|p| region.contains(Vec3::from(p.position))) {
            cloud.push(*p);
        }
        Some(FrameGeometry { cloud, depth: geometry.depth })
    }
}

fn median(values: &mut [f32]) -> f32 {
    if values.is_empty() {
        return f32::MAX;
    }
    let mid = values.len() / 2;
    *values.select_nth_unstable_by(mid, f32::total_cmp).1
}

/// Finds the turntable plane and the object on it in pooled sensor-frame points.
pub fn estimate_region(points: &[Vec3], config: &TurntableConfig) -> Option<TurntableRegion> {
    if points.len() < 500 {
        return None;
    }
    // 1. Plane: RANSAC, preferring planes whose normal points up in the sensor frame, oriented
    //    towards the sensor (origin): the object side. Then refined on all inliers.
    const INLIER_MM: f32 = 2.0;
    let sample: Vec<Vec3> = points.iter().step_by((points.len() / 4000).max(1)).copied().collect();
    let search = PlaneSearch {
        iterations: 400,
        inlier_mm: INLIER_MM,
        orientation: NormalOrientation::TowardsPoint(Vec3::ZERO),
        prefer: Some((Vec3::Z, config.min_up)),
        seed: 0x9e37_79b9,
        ..PlaneSearch::default()
    };
    let plane = refine_plane(points, ransac_plane(&sample, None, &search)?.plane, &[INLIER_MM; 2])?;
    let normal = plane.normal;
    let mut offset = -plane.d;

    // 2. Object: points above the plane, clustered on a grid in plane coordinates, starting from
    //    the one nearest the optical axis (the user aims at the object).
    let margin = config.plane_margin_mm;
    let above: Vec<(Vec3, f32)> = points
        .iter()
        .map(|p| (*p, normal.dot(*p) - offset))
        .filter(|(_, h)| *h > margin && *h < 1000.0)
        .collect();
    if above.len() < 100 {
        return None;
    }
    let u_axis = normal.any_orthonormal_vector();
    let v_axis = normal.cross(u_axis);
    let flat = |p: Vec3| Vec2::new(u_axis.dot(p), v_axis.dot(p));
    const CELL_MM: f32 = 5.0;
    let cell = |q: Vec2| [(q.x / CELL_MM).floor() as i32, (q.y / CELL_MM).floor() as i32];
    let mut occupancy: HashMap<[i32; 2], usize> = HashMap::new();
    for (p, _) in &above {
        *occupancy.entry(cell(flat(*p))).or_default() += 1;
    }
    // Connected components of occupied cells. Gaps of up to one empty cell are bridged: parts of
    // one object often do not touch in view (occlusion, dropped silhouette pixels), while the
    // background is usually further away. Cells with a single point are noise and bridge nothing.
    let CellComponents { label: component, sizes } = cell_components(&occupancy, 2, 2);
    // The object: the user aims at it, and it stands in front of any background. Among the
    // sizeable components (at least 5 % of the points above the plane) reaching within 15 degrees
    // of the optical axis, take the nearest one (median distance); failing that, the component
    // closest to the axis.
    let off_axis = |p: Vec3| (p.x * p.x + p.z * p.z).sqrt() / p.y.max(1e-3);
    let mut per_component: Vec<(f32, Vec<f32>)> = vec![(f32::MAX, Vec::new()); sizes.len()];
    for (p, _) in above.iter().filter(|(p, _)| p.y > 1.0) {
        if let Some(&id) = component.get(&cell(flat(*p))) {
            per_component[id].0 = per_component[id].0.min(off_axis(*p));
            per_component[id].1.push(p.length());
        }
    }
    let min_size = above.len() / 20;
    let near_axis = 15f32.to_radians().tan();
    let chosen = (0..sizes.len())
        .filter(|&id| sizes[id] >= min_size.max(20) && per_component[id].0 <= near_axis)
        .min_by(|&a, &b| median(&mut per_component[a].1.clone()).total_cmp(&median(&mut per_component[b].1.clone())))
        .or_else(|| (0..sizes.len()).min_by(|&a, &b| per_component[a].0.total_cmp(&per_component[b].0)))?;
    let mut object: Vec<(Vec3, f32)> =
        above.iter().filter(|(p, _)| component.get(&cell(flat(*p))) == Some(&chosen)).copied().collect();
    if object.len() < 50 {
        return None;
    }

    // 3. A turntable disc standing on the table: a thin, dominant layer of constant height low in
    //    the cluster (curved object surfaces spread over many heights). Raise the plane to it if
    //    enough of the cluster still stands above it.
    let mut heights: Vec<f32> = object.iter().map(|(_, h)| *h).collect();
    heights.sort_by(f32::total_cmp);
    let median = heights[heights.len() / 2];
    let mut bins: HashMap<i32, usize> = HashMap::new();
    for h in heights.iter().filter(|h| **h < median) {
        *bins.entry(h.floor() as i32).or_default() += 1;
    }
    if let Some((&bin, &peak)) = bins.iter().max_by_key(|(b, c)| (**c, -**b)) {
        let centre = bin as f32 + 0.5;
        let layer: Vec<f32> = heights.iter().copied().filter(|h| (h - centre).abs() <= 1.5).collect();
        let top = layer.iter().sum::<f32>() / layer.len() as f32;
        let standing = heights.iter().filter(|h| **h > top + margin).count();
        let n = heights.len();
        if peak * 10 >= n && layer.len() * 4 >= n && standing * 10 >= n * 3 {
            offset += top;
            object = object.into_iter().map(|(p, h)| (p, h - top)).filter(|(_, h)| *h > margin).collect();
        }
    }

    // 4. Cylinder around the object.
    let centre_2d = object.iter().map(|(p, _)| flat(*p)).sum::<Vec2>() / object.len() as f32;
    let mut radii: Vec<f32> = object.iter().map(|(p, _)| (flat(*p) - centre_2d).length()).collect();
    radii.sort_by(f32::total_cmp);
    let extent = radii[(radii.len() * 98 / 100).min(radii.len() - 1)];
    let mut tops: Vec<f32> = object.iter().map(|(_, h)| *h).collect();
    tops.sort_by(f32::total_cmp);
    let top = tops[(tops.len() * 98 / 100).min(tops.len() - 1)];
    let radius = config
        .radius_mm
        .unwrap_or_else(|| (extent * config.radius_factor + config.auto_margin_mm).min(config.max_radius_mm));
    let height = config.height_mm.unwrap_or(top + config.auto_margin_mm);
    let center = u_axis * centre_2d.x + v_axis * centre_2d.y + normal * offset;
    Some(TurntableRegion { normal, offset, center, radius_mm: radius, height_mm: height, plane_margin_mm: margin })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::OnDeviceDepth;
    use glam::{Mat3, Quat};
    use scancraft_core::{DepthMap, PinholeIntrinsics, Transform};
    use scancraft_slam::{Tracker, TrackingState};

    const W: usize = 160;
    const H: usize = 128;
    const DISC_R: f32 = 100.0;
    const DISC_TOP: f32 = 15.0;

    /// Spheres (object frame: turntable axis = z, disc top at z = 15) making up the object.
    fn object() -> [(Vec3, f32); 3] {
        [
            (Vec3::new(24.0, 6.0, DISC_TOP + 24.0), 24.0),
            (Vec3::new(-12.0, -12.0, DISC_TOP + 17.0), 17.0),
            (Vec3::new(2.0, 26.0, DISC_TOP + 12.0), 12.0),
        ]
    }

    fn intrinsics() -> PinholeIntrinsics {
        PinholeIntrinsics { fx: 185.0, fy: 185.0, cx: 79.5, cy: 63.5 }
    }

    /// Sensor pose in the world (world <- sensor): 300 mm from the axis, looking 35 degrees down.
    fn sensor_pose() -> Transform {
        let target = Vec3::new(0.0, 0.0, 35.0);
        let elevation = 35f32.to_radians();
        let position = target + Vec3::new(0.0, -elevation.cos(), elevation.sin()) * 300.0;
        let forward = (target - position).normalize();
        let right = forward.cross(Vec3::Z).normalize();
        let up = right.cross(forward);
        Transform { rotation: Quat::from_mat3(&Mat3::from_cols(right, forward, up)), translation: position, scale: Vec3::ONE }
    }

    fn ray_sphere(o: Vec3, d: Vec3, c: Vec3, r: f32) -> Option<f32> {
        let oc = o - c;
        let (a, b, cc) = (d.dot(d), 2.0 * oc.dot(d), oc.dot(oc) - r * r);
        let disc = b * b - 4.0 * a * cc;
        (disc >= 0.0).then(|| (-b - disc.sqrt()) / (2.0 * a)).filter(|t| *t > 0.0)
    }

    /// Depth image of the scene with the turntable turned by `angle` (radians): table (z = 0), a
    /// back wall (y = 180), the disc and the object on it.
    fn render(angle: f32) -> DepthMap {
        let k = intrinsics();
        let pose = sensor_pose();
        let o = pose.translation;
        let to_object = Quat::from_rotation_z(-angle);
        let mut data = vec![0u16; W * H];
        for v in 0..H {
            for u in 0..W {
                // Unnormalised ray with unit forward component: the ray parameter is the depth.
                let d_sensor = Vec3::new((u as f32 - k.cx) / k.fx, 1.0, -(v as f32 - k.cy) / k.fy);
                let d = pose.rotation * d_sensor;
                let mut t = f32::MAX;
                if d.z < 0.0 {
                    t = t.min(-o.z / d.z); // table
                }
                if d.y > 0.0 {
                    t = t.min((180.0 - o.y) / d.y); // wall
                }
                // Disc top.
                if d.z < 0.0 {
                    let td = (DISC_TOP - o.z) / d.z;
                    let p = o + d * td;
                    if p.truncate().length() <= DISC_R {
                        t = t.min(td);
                    }
                }
                // Disc side (cylinder r = 100, 0 <= z <= 15).
                let (a, b, c) = (d.x * d.x + d.y * d.y, 2.0 * (o.x * d.x + o.y * d.y), o.x * o.x + o.y * o.y - DISC_R * DISC_R);
                let disc = b * b - 4.0 * a * c;
                if disc >= 0.0 {
                    let ts = (-b - disc.sqrt()) / (2.0 * a);
                    let z = o.z + d.z * ts;
                    if ts > 0.0 && (0.0..=DISC_TOP).contains(&z) {
                        t = t.min(ts);
                    }
                }
                let (oo, dd) = (to_object * o, to_object * d);
                for (c, r) in object() {
                    if let Some(ts) = ray_sphere(oo, dd, c, r) {
                        t = t.min(ts);
                    }
                }
                if t < 600.0 {
                    data[v * W + u] = (t * 100.0).round() as u16;
                }
            }
        }
        DepthMap::new(W as u32, H as u32, 0.01, data).with_intrinsics(k)
    }

    fn frame(i: u64, angle: f32) -> SensorFrame {
        let mut f = SensorFrame::new(i, i * 66);
        f.depth = Some(render(angle));
        f
    }

    const STEP: f32 = 0.035; // 2 degrees per frame

    #[test]
    fn region_keeps_only_the_object() {
        let mut filter = TurntableFilter::new(Box::new(OnDeviceDepth::new(150.0, 600.0)), TurntableConfig::default());
        for i in 0..2 {
            assert!(filter.process(&frame(i, 0.0)).is_none(), "still pooling");
        }
        let geometry = filter.process(&frame(2, 0.0)).expect("region found after three frames");
        let region = *filter.region().unwrap();

        // The plane is the disc top (world z = 15), seen from the sensor.
        let to_sensor = sensor_pose().inverse();
        let world_up = to_sensor.rotation * Vec3::Z;
        assert!(region.normal.dot(world_up) > 0.999, "normal {:?} vs up {world_up:?}", region.normal);
        let disc_top = to_sensor.transform_point3(Vec3::new(0.0, 0.0, DISC_TOP));
        assert!(region.height_of(disc_top).abs() < 1.0, "plane at the disc top: {}", region.height_of(disc_top));
        // The axis estimate is near the turntable centre, and the cylinder holds the object.
        let axis_error = (region.center - disc_top).length();
        assert!(axis_error < 25.0, "centre {axis_error:.1} mm off the axis");
        // The object reaches 49 mm from the axis: it must stay inside while turning.
        assert!(region.radius_mm > 49.0 + axis_error && region.radius_mm < 140.0, "radius {}", region.radius_mm);
        assert!(region.height_mm > 48.0 && region.height_mm < 90.0, "height {}", region.height_mm);

        // Only object points pass: everything left lies on a sphere.
        let full = render(0.0).to_point_cloud(150.0, 600.0).len();
        assert!(geometry.cloud.len() > 500 && geometry.cloud.len() < full / 3, "{} of {full}", geometry.cloud.len());
        let to_world = sensor_pose();
        for p in &geometry.cloud.points {
            let w = to_world.transform_point3(Vec3::from(p.position));
            let off_sphere = object().iter().map(|(c, r)| ((w - *c).length() - r).abs()).fold(f32::MAX, f32::min);
            assert!(off_sphere < 1.5, "point {w:?} is {off_sphere:.2} mm off the object");
        }
        assert!(geometry.depth.is_some(), "the depth image is passed on for visibility checks");
    }

    /// Relative rotation angle between two poses.
    fn angle_between(a: &Transform, b: &Transform) -> f32 {
        a.rotation.angle_between(b.rotation)
    }

    #[test]
    fn turntable_mode_tracks_the_object_not_the_background() {
        let mut pipeline = crate::PassPipeline {
            depth: Box::new(TurntableFilter::new(Box::new(OnDeviceDepth::new(150.0, 600.0)), TurntableConfig::default())),
            pose: None,
            color: None,
        };
        let mut tracker = Tracker::new();
        let mut first: Option<(usize, Transform)> = None;
        for i in 0..22usize {
            let Some(status) = pipeline.process(&frame(i as u64, i as f32 * STEP), &mut tracker) else { continue };
            assert_ne!(status.state, TrackingState::Lost, "frame {i}");
            let (i0, p0) = *first.get_or_insert((i, status.current_pose));
            let truth = (i - i0) as f32 * STEP;
            let got = angle_between(&status.current_pose, &p0);
            assert!((got - truth).abs() < 0.6f32.to_radians(), "frame {i}: turned {:.2} deg, truth {:.2} deg", got.to_degrees(), truth.to_degrees());
        }
        let (i0, p0) = first.expect("tracking started");
        assert_eq!(i0, 2, "starts once the region is known");
        // The sensor orbits the object about the turntable axis (world up, in sensor coordinates).
        let step = (tracker.current_pose.rotation * p0.rotation.inverse()).normalize();
        let (axis, _) = step.to_axis_angle();
        let up = sensor_pose().inverse().rotation * Vec3::Z;
        assert!(axis.dot(up).abs() > 0.98, "rotation axis {axis:?} vs turntable axis {up:?}");
    }

    #[test]
    fn without_turntable_mode_the_background_wins() {
        let mut pipeline = crate::PassPipeline { depth: Box::new(OnDeviceDepth::new(150.0, 600.0)), pose: None, color: None };
        let mut tracker = Tracker::new();
        let mut poses = Vec::new();
        for i in 0..12usize {
            if let Some(status) = pipeline.process(&frame(i as u64, i as f32 * STEP), &mut tracker) {
                poses.push(status.current_pose);
            }
        }
        // The documented failure: the static scene holds the pose still while the object turns.
        let turned = angle_between(poses.last().unwrap(), &poses[0]);
        assert!(turned < 11.0 * STEP / 4.0, "turned {:.1} deg", turned.to_degrees());
    }
}
