//! How plausible an alignment between two passes is: how much of the moving pass lands on the
//! other one (overlap), how closely (RMS point-to-plane), and whether it contradicts what either
//! pass's scanner actually saw (visibility: surface placed in front of a measured surface, where
//! the scanner would have seen it).

use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::Transform;

use crate::cloud::{downsample, Index};
use crate::{compose, Oriented, PassCloud, View};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quality {
    /// Fraction of the moving pass's surface within `gate` of the other pass, facing the same way.
    pub overlap: f32,
    /// RMS point-to-plane distance over the overlapping part, mm.
    pub rmse_mm: f32,
    /// Of the moved surface points that the other pass's views (and vice versa) can judge, the
    /// fraction lying clearly in front of what was measured. A right alignment has only a few
    /// percent (silhouettes); a wrong one hides measured surface behind invented surface.
    pub visibility_conflict: f32,
    /// Overlapping sample points.
    pub inliers: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Good,
    Fair,
    Poor,
}

impl Verdict {
    pub fn label(self) -> &'static str {
        match self {
            Verdict::Good => "Good",
            Verdict::Fair => "Uncertain",
            Verdict::Poor => "Poor",
        }
    }
}

impl Quality {
    pub const NONE: Quality = Quality { overlap: 0.0, rmse_mm: f32::INFINITY, visibility_conflict: 1.0, inliers: 0 };

    /// One number to rank candidates: overlap, penalised by conflict and residual.
    pub fn score(&self) -> f32 {
        if !self.rmse_mm.is_finite() {
            return 0.0;
        }
        let conflict = (1.0 - self.visibility_conflict / 0.2).clamp(0.0, 1.0);
        self.overlap * conflict * conflict / (1.0 + self.rmse_mm)
    }

    pub fn verdict(&self) -> Verdict {
        if self.overlap < 0.05 || self.visibility_conflict > 0.15 || self.rmse_mm > 1.5 || self.inliers < 100 {
            Verdict::Poor
        } else if self.overlap >= 0.12 && self.visibility_conflict <= 0.06 && self.rmse_mm <= 0.8 {
            Verdict::Good
        } else {
            Verdict::Fair
        }
    }
}

/// Settings for [`evaluate`].
#[derive(Debug, Clone, Copy)]
pub struct QualityConfig {
    /// Grid the surfaces are sampled on.
    pub sample_mm: f32,
    /// Distance within which a point counts as overlapping.
    pub gate_mm: f32,
    /// Depth agreement tolerance for the visibility check.
    pub depth_tolerance_mm: f32,
}

impl QualityConfig {
    pub fn for_sample(sample_mm: f32) -> Self {
        QualityConfig { sample_mm, gate_mm: (1.5 * sample_mm).max(1.5), depth_tolerance_mm: (1.5 * sample_mm).max(3.0) }
    }
}

/// Scores `transform` (target <- source).
pub fn evaluate(source: &PassCloud, target: &PassCloud, transform: &Transform, config: QualityConfig) -> Quality {
    let src = downsample(&source.points, config.sample_mm);
    let tgt = downsample(&target.points, config.sample_mm);
    if src.is_empty() || tgt.is_empty() {
        return Quality::NONE;
    }
    let index = Index::new(tgt.iter().map(|p| p.0).collect());
    evaluate_sampled(&src, &tgt, &index, &source.views, &target.views, transform, config)
}

/// [`evaluate`] on already subsampled surfaces (`index` over `tgt`).
pub fn evaluate_sampled(src: &[Oriented], tgt: &[Oriented], index: &Index, src_views: &[View], tgt_views: &[View], transform: &Transform, config: QualityConfig) -> Quality {
    let m = transform.to_mat4();
    let gate_sq = config.gate_mm * config.gate_mm;
    let (inliers, sq) = src
        .par_iter()
        .map(|&(p, n)| {
            let pw = m.transform_point3(p);
            match index.nearest(pw) {
                Some((d, i)) if d <= gate_sq && (transform.rotation * n).dot(tgt[i].1) > 0.5 => {
                    let r = tgt[i].1.dot(pw - tgt[i].0);
                    (1usize, (r * r) as f64)
                }
                _ => (0, 0.0),
            }
        })
        .reduce(|| (0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1));
    // Visibility both ways: the moved source against the target's views, and the target against
    // the source's views (in the source frame).
    let moved: Vec<Vec3> = src.iter().map(|(p, _)| m.transform_point3(*p)).collect();
    let (a1, c1) = visibility(&moved, tgt_views, config.depth_tolerance_mm);
    let inv = transform.inverse().to_mat4();
    let back: Vec<Vec3> = tgt.iter().map(|(p, _)| inv.transform_point3(*p)).collect();
    let (a2, c2) = visibility(&back, src_views, config.depth_tolerance_mm);
    let judged = a1 + c1 + a2 + c2;
    Quality {
        overlap: inliers as f32 / src.len() as f32,
        rmse_mm: if inliers > 0 { (sq / inliers as f64).sqrt() as f32 } else { f32::INFINITY },
        visibility_conflict: if judged >= 100 { (c1 + c2) as f32 / judged as f32 } else { 0.0 },
        inliers,
    }
}

/// Angular depth image of one view: the nearest measured range per direction bin.
struct RangeImage {
    to_sensor: glam::Mat4,
    ranges: Vec<f32>,
}

const BIN_RAD: f32 = 0.008;
const HALF_BINS: i32 = 100; // +-0.8 rad
const WIDTH: usize = (2 * HALF_BINS) as usize;

fn bin_of(q: Vec3) -> Option<(i32, i32)> {
    if q.y <= 1.0 {
        return None;
    }
    let u = (q.x.atan2(q.y) / BIN_RAD).floor() as i32 + HALF_BINS;
    let v = (q.z.atan2(q.y) / BIN_RAD).floor() as i32 + HALF_BINS;
    (u >= 0 && v >= 0 && u < 2 * HALF_BINS && v < 2 * HALF_BINS).then_some((u, v))
}

impl RangeImage {
    fn new(view: &View) -> Self {
        let mut ranges = vec![f32::INFINITY; WIDTH * WIDTH];
        let stride = (view.points.len() / 60_000).max(1);
        for q in view.points.iter().step_by(stride) {
            if let Some((u, v)) = bin_of(*q) {
                let i = v as usize * WIDTH + u as usize;
                ranges[i] = ranges[i].min(q.length());
            }
        }
        RangeImage { to_sensor: view.pose.inverse().to_mat4(), ranges }
    }

    /// The measured range in the direction of `p` (view's frame), unless that direction is
    /// unmeasured or at a depth discontinuity (silhouettes are not evidence).
    fn measured(&self, q: Vec3) -> Option<f32> {
        let (u, v) = bin_of(q)?;
        if u == 0 || v == 0 || u + 1 >= 2 * HALF_BINS || v + 1 >= 2 * HALF_BINS {
            return None;
        }
        let at = |du: i32, dv: i32| self.ranges[(v + dv) as usize * WIDTH + (u + du) as usize];
        let r = at(0, 0);
        if !r.is_finite() {
            return None;
        }
        for (du, dv) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
            let n = at(du, dv);
            if !n.is_finite() || (n - r).abs() > 6.0 {
                return None;
            }
        }
        Some(r)
    }
}

/// (agreeing, conflicting) counts of `points` (in the views' frame) over a spread of `views`.
pub fn visibility(points: &[Vec3], views: &[View], tolerance_mm: f32) -> (usize, usize) {
    if views.is_empty() || points.is_empty() {
        return (0, 0);
    }
    let stride = (views.len() / 24).max(1);
    let images: Vec<RangeImage> = views.iter().step_by(stride).filter(|v| !v.points.is_empty()).map(RangeImage::new).collect();
    let pstride = (points.len() / 20_000).max(1);
    images
        .par_iter()
        .map(|img| {
            let (mut agree, mut conflict) = (0usize, 0usize);
            for p in points.iter().step_by(pstride) {
                let q = img.to_sensor.transform_point3(*p);
                let Some(r) = img.measured(q) else { continue };
                let diff = q.length() - r;
                if diff.abs() <= tolerance_mm {
                    agree += 1;
                } else if diff < -2.0 * tolerance_mm {
                    conflict += 1;
                }
            }
            (agree, conflict)
        })
        .reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1))
}

/// A pass expressed in another frame (`t`: new <- old), views included.
pub fn moved(pass: &PassCloud, t: &Transform) -> PassCloud {
    PassCloud {
        points: crate::cloud::transform_oriented(&pass.points, t),
        views: pass.views.iter().map(|v| View { pose: compose(t, &v.pose), points: v.points.clone() }).collect(),
    }
}
