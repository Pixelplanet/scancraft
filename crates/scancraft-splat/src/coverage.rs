//! RGB coverage of the initial point cloud.
//!
//! The merged cloud may contain geometry that no colour keyframe saw (geometry-only passes, the
//! underside of an object). Those points carry no appearance information, so they are excluded
//! from the splat: a point survives if at least `min_views` images see it unoccluded (and inside
//! the mask, when the image has one).
//!
//! Occlusion is tested against a coarse z-buffer splatted from the cloud itself, which is cheap
//! and needs no mesh.

use crate::types::PosedImage;
use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::PointCloud;

/// Long edge of the occlusion z-buffer, in cells.
const ZBUF_LONG_EDGE: u32 = 192;

#[derive(Debug, Clone, Copy)]
pub struct CoverageConfig {
    /// Views that must see a point for it to stay.
    pub min_views: u32,
    /// A point counts as visible if it is at most this much behind the nearest point in its
    /// z-buffer cell, relative to depth (0.02 = 2 %).
    pub depth_tolerance: f32,
    /// Absolute floor of the depth tolerance, world units (mm).
    pub min_depth_tolerance: f32,
}

impl Default for CoverageConfig {
    fn default() -> Self {
        Self { min_views: 1, depth_tolerance: 0.02, min_depth_tolerance: 2.0 }
    }
}

/// Per point: how many images see it, and the mean colour (0..1) those images observe there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointCoverage {
    pub views: u32,
    pub mean_rgb: [f32; 3],
}

/// Coverage of every point of `cloud` by `images`.
pub fn analyse(cloud: &PointCloud, images: &[PosedImage], cfg: &CoverageConfig) -> Vec<PointCoverage> {
    let positions: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from_array(p.position)).collect();
    let per_image: Vec<Vec<Option<[u8; 3]>>> =
        images.par_iter().map(|img| visible_in(&positions, img, cfg)).collect();
    (0..positions.len())
        .map(|i| {
            let mut sum = [0u32; 3];
            let mut views = 0;
            for c in per_image.iter().filter_map(|v| v[i]) {
                views += 1;
                for k in 0..3 {
                    sum[k] += c[k] as u32;
                }
            }
            let n = views.max(1) as f32 * 255.0;
            PointCoverage { views, mean_rgb: [sum[0] as f32 / n, sum[1] as f32 / n, sum[2] as f32 / n] }
        })
        .collect()
}

/// Number of images that see each point.
pub fn view_counts(cloud: &PointCloud, images: &[PosedImage], cfg: &CoverageConfig) -> Vec<u32> {
    analyse(cloud, images, cfg).into_iter().map(|c| c.views).collect()
}

/// The points of `cloud` that at least `cfg.min_views` images see.
pub fn filter_covered(cloud: &PointCloud, images: &[PosedImage], cfg: &CoverageConfig) -> PointCloud {
    let counts = view_counts(cloud, images, cfg);
    let kept = cloud
        .points
        .iter()
        .zip(counts)
        .filter(|(_, c)| *c >= cfg.min_views)
        .map(|(p, _)| *p)
        .collect();
    PointCloud::from_points(kept)
}

/// The image colour at each point it sees unoccluded, `None` elsewhere.
fn visible_in(points: &[Vec3], img: &PosedImage, cfg: &CoverageConfig) -> Vec<Option<[u8; 3]>> {
    let projected: Vec<Option<(usize, usize, f32)>> = points
        .iter()
        .map(|&p| {
            let (u, v, d) = img.project(p)?;
            if u < 0.0 || v < 0.0 || u >= img.width as f32 || v >= img.height as f32 {
                return None;
            }
            if let Some(mask) = &img.mask {
                if mask[v as usize * img.width as usize + u as usize] < 128 {
                    return None;
                }
            }
            Some((u as usize, v as usize, d))
        })
        .collect();

    // Cell size: at least ZBUF_LONG_EDGE cells across, but never finer than the projected point
    // spacing, or a sparse cloud lets far surfaces show through the gaps of near ones. Roughly
    // half the points face away, so the front surface has ~n/2 points over the footprint.
    let (mut lo, mut hi, mut n) = ((f32::MAX, f32::MAX), (0.0f32, 0.0f32), 0usize);
    for &(u, v, _) in projected.iter().flatten() {
        lo = (lo.0.min(u as f32), lo.1.min(v as f32));
        hi = (hi.0.max(u as f32), hi.1.max(v as f32));
        n += 1;
    }
    let spacing = if n > 0 { ((hi.0 - lo.0 + 1.0) * (hi.1 - lo.1 + 1.0) * 2.0 / n as f32).sqrt() } else { 1.0 };
    let cell = (img.width.max(img.height) as f32 / ZBUF_LONG_EDGE as f32).max(spacing * 1.5).max(1.0);
    let gw = (img.width as f32 / cell).ceil() as usize;
    let gh = (img.height as f32 / cell).ceil() as usize;
    let mut zbuf = vec![f32::INFINITY; gw * gh];

    for &(u, v, d) in projected.iter().flatten() {
        let idx = (v as f32 / cell) as usize * gw + (u as f32 / cell) as usize;
        zbuf[idx] = zbuf[idx].min(d);
    }

    projected
        .iter()
        .map(|pr| {
            let (u, v, d) = (*pr)?;
            let (cx, cy) = ((u as f32 / cell) as usize, (v as f32 / cell) as usize);
            // Nearest surface around the cell: a 3x3 neighbourhood keeps sparse clouds from
            // showing far surfaces through gaps between near points.
            let mut nearest = f32::INFINITY;
            for y in cy.saturating_sub(1)..(cy + 2).min(gh) {
                for x in cx.saturating_sub(1)..(cx + 2).min(gw) {
                    nearest = nearest.min(zbuf[y * gw + x]);
                }
            }
            if d > nearest + (nearest * cfg.depth_tolerance).max(cfg.min_depth_tolerance) {
                return None;
            }
            let o = (v * img.width as usize + u) * 3;
            Some([img.rgb[o], img.rgb[o + 1], img.rgb[o + 2]])
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{CameraAxes, Intrinsics};
    use glam::Affine3A;
    use scancraft_core::Point3D;

    #[test]
    fn hidden_and_out_of_view_points_are_dropped() {
        // Camera at the origin looking along +Y (ScanCraft axes).
        let img = PosedImage {
            name: "v".into(),
            width: 64,
            height: 64,
            rgb: vec![0; 64 * 64 * 3],
            mask: None,
            intrinsics: Intrinsics { fx: 64.0, fy: 64.0, cx: 32.0, cy: 32.0, distortion: None },
            camera_to_world: Affine3A::IDENTITY,
            axes: CameraAxes::ScanCraft,
        };
        // A wall of points at y = 100 and one point straight behind it at y = 200.
        let mut pts = Vec::new();
        for i in -10..=10 {
            for j in -10..=10 {
                pts.push(Point3D::new([i as f32 * 2.0, 100.0, j as f32 * 2.0]));
            }
        }
        pts.push(Point3D::new([0.0, 200.0, 0.0])); // occluded
        pts.push(Point3D::new([0.0, -100.0, 0.0])); // behind the camera
        pts.push(Point3D::new([500.0, 100.0, 0.0])); // outside the image
        let cloud = PointCloud::from_points(pts);
        let counts = view_counts(&cloud, std::slice::from_ref(&img), &CoverageConfig::default());
        let n = counts.len();
        assert!(counts[..n - 3].iter().all(|&c| c == 1));
        assert_eq!(&counts[n - 3..], &[0, 0, 0]);
        assert_eq!(filter_covered(&cloud, &[img], &CoverageConfig::default()).points.len(), n - 3);
    }
}
