//! How plausible an alignment between two passes is:
//!
//! - overlap: how much of the moving pass lands on the other one facing the same way, and how
//!   closely (RMS point-to-plane);
//! - shell: where one pass shows the other face of a wall (an inside and an outside scan of a
//!   shell part), the offset between the faces (the wall thickness), how even it is, and how much
//!   of the moving pass pokes through the other surface;
//! - visibility: whether it contradicts what either pass's scanner actually saw (surface placed
//!   in front of a measured surface, where the scanner would have seen it);
//! - support: whether it puts one pass below the table the other one stood on;
//! - slide: whether the matched surfaces pin the pose down in every direction or let it slide
//!   (a plane, a sphere, a smooth shell).

use glam::{Mat4, Vec3};
use rayon::prelude::*;
use scancraft_core::Transform;

use crate::cloud::{downsample, Index};
use crate::{compose, Oriented, PassCloud, Plane, View};

/// [`Quality::slide_mm`] above this counts as sliding.
pub const SLIDE_LIMIT_MM: f32 = 10.0;

/// The residual rise that defines [`Quality::slide_mm`], mm RMS.
const SLIDE_RISE_MM: f64 = 0.5;

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
    /// Pairs of facing-apart surfaces (the two faces of a wall); `None` when too few were found.
    pub shell: Option<Shell>,
    /// Fraction of either pass's object points (those above its own support plane) that the
    /// transform puts below the other pass's support plane. 0 when neither pass has one.
    pub below_support: f32,
    /// How far the pose can move in its least constrained direction (RMS displacement of the
    /// matched points, mm) before the RMS point-to-plane residual of all matched pairs rises by
    /// 0.5 mm. From the smallest eigenvalue of the 6x6 point-to-plane normal matrix. Large when
    /// the surfaces let the fit slide (a plane, a sphere); infinite with too few pairs; 0 when not
    /// measured ([`Quality::NONE`], stored results).
    pub slide_mm: f32,
}

/// Opposite-facing pairs: moved source points with a target surface facing the other way within
/// [`QualityConfig::shell_range_mm`] along its normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shell {
    pub pairs: usize,
    /// `pairs` over the moving pass's sample points.
    pub fraction: f32,
    /// Median signed offset along the target normal, mm. Negative: the moving surface lies behind
    /// the target surface, as the other face of a wall does (-3 = a 3 mm wall).
    pub offset_mm: f32,
    /// Interquartile range of the offsets, mm.
    pub iqr_mm: f32,
    /// Fraction of the moving pass's sample points (not of the pairs) in a pair lying more than
    /// [`QualityConfig::pierce_mm`] in front of the target surface: one face poking through the
    /// other.
    pub piercing: f32,
}

impl Shell {
    /// Enough pairs to judge the fit by.
    pub fn significant(&self) -> bool {
        self.pairs >= 100 && self.fraction >= 0.05
    }

    /// Wall thickness implied by the offset, mm.
    pub fn thickness_mm(&self) -> f32 {
        -self.offset_mm
    }

    /// The pairs describe one wall: a narrow peak of 1..6 mm behind the target, with (almost)
    /// nothing poking through.
    pub fn is_wall(&self) -> bool {
        let t = self.thickness_mm();
        self.significant() && (1.0..=6.0).contains(&t) && self.iqr_mm <= (0.3 * t).max(0.6) && self.piercing < 0.01
    }
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
    pub const NONE: Quality = Quality { overlap: 0.0, rmse_mm: f32::INFINITY, visibility_conflict: 1.0, inliers: 0, shell: None, below_support: 0.0, slide_mm: 0.0 };

    /// The shell pairs, when they describe a wall (see [`Shell::is_wall`]).
    pub fn wall(&self) -> Option<Shell> {
        self.shell.filter(Shell::is_wall)
    }

    /// More than 3 % of the moving pass pokes through the other surface.
    pub fn pierces(&self) -> bool {
        self.shell.is_some_and(|s| s.piercing > 0.03)
    }

    /// The fit is weakly constrained in some direction ([`Quality::slide_mm`] above
    /// [`SLIDE_LIMIT_MM`]).
    pub fn sliding(&self) -> bool {
        self.slide_mm > SLIDE_LIMIT_MM
    }

    /// Fraction of the moving pass that supports the fit: same-side overlap plus the other face
    /// of a wall.
    pub fn contact(&self) -> f32 {
        self.overlap + self.wall().map_or(0.0, |s| s.fraction)
    }

    /// One number to rank candidates: contact, penalised by conflict, piercing, support
    /// violations and residual.
    pub fn score(&self) -> f32 {
        let wall = self.wall();
        let residual = if self.inliers >= 100 && self.rmse_mm.is_finite() {
            self.rmse_mm
        } else if let Some(w) = wall {
            w.iqr_mm / 1.349
        } else if self.rmse_mm.is_finite() {
            self.rmse_mm
        } else {
            return 0.0;
        };
        let conflict = (1.0 - self.visibility_conflict / 0.2).clamp(0.0, 1.0);
        let pierce = self.shell.map_or(1.0, |s| (1.0 - s.piercing / 0.05).clamp(0.0, 1.0));
        let support = (1.0 - self.below_support / 0.05).clamp(0.0, 1.0);
        self.contact() * conflict * conflict * pierce * pierce * support / (1.0 + residual)
    }

    /// The fit contradicts the data: it puts surface below the other pass's table, through the
    /// other surface, or in front of what the other scanner measured. Any of these makes it Poor,
    /// however much it overlaps.
    pub fn contradicted(&self) -> bool {
        self.below_support > 0.02 || self.pierces() || self.visibility_conflict > 0.15
    }

    pub fn verdict(&self) -> Verdict {
        let contact = self.contact();
        let pairs = self.inliers + self.wall().map_or(0, |s| s.pairs);
        let loose = self.inliers >= 100 && self.rmse_mm > 1.5;
        if self.contradicted() || contact < 0.05 || loose || pairs < 100 {
            return Verdict::Poor;
        }
        let tight = self.rmse_mm <= 0.8 || self.inliers < 100;
        let clean = self.shell.is_none_or(|s| s.piercing <= 0.01);
        if contact >= 0.12 && self.visibility_conflict <= 0.06 && tight && clean && !self.sliding() {
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
    /// Distance within which a point counts as overlapping; also the largest sideways distance
    /// of a shell pair.
    pub gate_mm: f32,
    /// Depth agreement tolerance for the visibility check.
    pub depth_tolerance_mm: f32,
    /// How far behind or in front of the target surface an opposite-facing surface still counts
    /// as a shell pair.
    pub shell_range_mm: f32,
    /// A shell pair further than this in front of the target surface pierces it.
    pub pierce_mm: f32,
    /// How far below a support plane a point may lie (noise, alignment error) before it counts.
    pub support_tolerance_mm: f32,
}

impl QualityConfig {
    pub fn for_sample(sample_mm: f32) -> Self {
        QualityConfig {
            sample_mm,
            gate_mm: (1.5 * sample_mm).max(1.5),
            depth_tolerance_mm: (1.5 * sample_mm).max(3.0),
            shell_range_mm: 10.0,
            pierce_mm: (0.15 * sample_mm).max(0.7),
            support_tolerance_mm: (2.0 * sample_mm).max(4.0),
        }
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
    evaluate_sampled(&src, &tgt, &index, source, target, transform, config)
}

/// How one moved source sample relates to the target.
enum Match {
    /// Same-facing surface within the gate; point-to-plane residual.
    Same(f32),
    /// Opposite-facing surface; signed offset along the target normal.
    Opposite(f32),
}

/// [`evaluate`] on already subsampled surfaces (`index` over `tgt`); `source` and `target` give
/// the views and support planes.
pub fn evaluate_sampled(src: &[Oriented], tgt: &[Oriented], index: &Index, source: &PassCloud, target: &PassCloud, transform: &Transform, config: QualityConfig) -> Quality {
    let m = transform.to_mat4();
    let gate_sq = config.gate_mm * config.gate_mm;
    let reach_sq = gate_sq + config.shell_range_mm * config.shell_range_mm;
    // (moved point, target normal, match) per source sample.
    let matches: Vec<(Vec3, Vec3, Match)> = src
        .par_iter()
        .filter_map(|&(p, n)| {
            let pw = m.transform_point3(p);
            let nw = transform.rotation * n;
            let (d, i) = index.nearest(pw)?;
            let (q, nt) = tgt[i];
            if d <= gate_sq && nw.dot(nt) > 0.5 {
                return Some((pw, nt, Match::Same(nt.dot(pw - q))));
            }
            if d > reach_sq {
                return None;
            }
            // The other face of a wall: the opposite-facing target point most nearly along the
            // target normal.
            index
                .nearest_n(pw, 8)
                .into_iter()
                .filter_map(|(_, j)| {
                    let (q, nt) = tgt[j];
                    if nw.dot(nt) >= -0.5 {
                        return None;
                    }
                    let v = pw - q;
                    let offset = nt.dot(v);
                    let side = (v - nt * offset).length();
                    (offset.abs() <= config.shell_range_mm && side <= config.gate_mm).then_some((side, nt, offset))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, nt, offset)| (pw, nt, Match::Opposite(offset)))
        })
        .collect();
    let (mut inliers, mut sq) = (0usize, 0.0f64);
    let mut offsets = Vec::new();
    for (_, _, k) in &matches {
        match *k {
            Match::Same(r) => {
                inliers += 1;
                sq += (r * r) as f64;
            }
            Match::Opposite(o) => offsets.push(o),
        }
    }
    let shell = shell_stats(&mut offsets, src.len(), config.pierce_mm);
    let slide_mm = slide(&matches);
    // Visibility both ways: the moved source against the target's views, and the target against
    // the source's views (in the source frame).
    let moved: Vec<Vec3> = src.iter().map(|(p, _)| m.transform_point3(*p)).collect();
    let (a1, c1) = visibility(&moved, &target.views, config.depth_tolerance_mm);
    let inv = transform.inverse().to_mat4();
    let back: Vec<Vec3> = tgt.iter().map(|(p, _)| inv.transform_point3(*p)).collect();
    let (a2, c2) = visibility(&back, &source.views, config.depth_tolerance_mm);
    let judged = a1 + c1 + a2 + c2;
    let below_support = below_support(src, source.support, target.support, m, config.support_tolerance_mm).max(below_support(tgt, target.support, source.support, inv, config.support_tolerance_mm));
    Quality {
        overlap: inliers as f32 / src.len() as f32,
        rmse_mm: if inliers > 0 { (sq / inliers as f64).sqrt() as f32 } else { f32::INFINITY },
        visibility_conflict: if judged >= 100 { (c1 + c2) as f32 / judged as f32 } else { 0.0 },
        inliers,
        shell,
        below_support,
        slide_mm,
    }
}

/// Median, interquartile range and piercing fraction of the shell offsets (sorted in place);
/// `None` below 20 pairs.
fn shell_stats(offsets: &mut [f32], samples: usize, pierce_mm: f32) -> Option<Shell> {
    if offsets.len() < 20 {
        return None;
    }
    offsets.sort_by(f32::total_cmp);
    let at = |q: f32| offsets[((offsets.len() - 1) as f32 * q).round() as usize];
    let samples = samples.max(1) as f32;
    let piercing = offsets.iter().filter(|&&o| o > pierce_mm).count() as f32 / samples;
    Some(Shell { pairs: offsets.len(), fraction: offsets.len() as f32 / samples, offset_mm: at(0.5), iqr_mm: at(0.75) - at(0.25), piercing })
}

/// Fraction of `points` above their own `own` support plane that `m` moves below `other`'s.
fn below_support(points: &[Oriented], own: Option<Plane>, other: Option<Plane>, m: Mat4, tolerance_mm: f32) -> f32 {
    let Some(other) = other else { return 0.0 };
    let (object, below) = points
        .par_iter()
        .filter(|(p, _)| own.is_none_or(|pl| pl.distance(*p) > tolerance_mm))
        .map(|(p, _)| (1usize, usize::from(other.distance(m.transform_point3(*p)) < -tolerance_mm)))
        .reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
    if object == 0 { 0.0 } else { below as f32 / object as f32 }
}

/// [`Quality::slide_mm`] of the matched pairs. The 6x6 point-to-plane normal matrix is averaged
/// over the pairs, rotations taken about their centroid and scaled by their RMS radius, so a unit
/// step in any direction moves the points by about 1 mm RMS; its smallest eigenvalue is then the
/// mean squared residual added per mm² of motion in the weakest direction. Below 12 pairs:
/// infinite.
fn slide(matches: &[(Vec3, Vec3, Match)]) -> f32 {
    if matches.len() < 12 {
        return f32::INFINITY;
    }
    let n = matches.len() as f64;
    let c = matches.iter().map(|m| m.0.as_dvec3()).sum::<glam::DVec3>() / n;
    let radius = (matches.iter().map(|m| (m.0.as_dvec3() - c).length_squared()).sum::<f64>() / n).sqrt().max(1e-6);
    let mut h = [[0.0f64; 6]; 6];
    for (p, nt, _) in matches {
        let nt = nt.as_dvec3();
        let r = (p.as_dvec3() - c).cross(nt) / radius;
        let j = [r.x, r.y, r.z, nt.x, nt.y, nt.z];
        for a in 0..6 {
            for b in 0..6 {
                h[a][b] += j[a] * j[b] / n;
            }
        }
    }
    let weakest = sym_eigenvalues(h).into_iter().fold(f64::MAX, f64::min).max(0.0);
    (SLIDE_RISE_MM / weakest.sqrt()) as f32
}

/// Eigenvalues of a symmetric matrix (cyclic Jacobi), unsorted.
fn sym_eigenvalues<const N: usize>(mut a: [[f64; N]; N]) -> [f64; N] {
    let scale: f64 = (0..N).flat_map(|i| (0..N).map(move |j| (i, j))).map(|(i, j)| a[i][j] * a[i][j]).sum::<f64>().max(1e-300);
    for _ in 0..60 {
        let off: f64 = (0..N).flat_map(|i| (i + 1..N).map(move |j| (i, j))).map(|(i, j)| a[i][j] * a[i][j]).sum();
        if off <= 1e-24 * scale {
            break;
        }
        for p in 0..N {
            for q in p + 1..N {
                if a[p][q] == 0.0 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for row in a.iter_mut() {
                    let (kp, kq) = (row[p], row[q]);
                    row[p] = c * kp - s * kq;
                    row[q] = s * kp + c * kq;
                }
                let (top, bottom) = a.split_at_mut(q);
                for (pk, qk) in top[p].iter_mut().zip(bottom[0].iter_mut()) {
                    (*pk, *qk) = (c * *pk - s * *qk, s * *pk + c * *qk);
                }
            }
        }
    }
    std::array::from_fn(|i| a[i][i])
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

/// A pass expressed in another frame (`t`: new <- old), views and support plane included.
pub fn moved(pass: &PassCloud, t: &Transform) -> PassCloud {
    PassCloud {
        points: crate::cloud::transform_oriented(&pass.points, t),
        views: pass.views.iter().map(|v| View { pose: compose(t, &v.pose), points: v.points.clone() }).collect(),
        support: pass.support.map(|s| s.transformed(t)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::{self, SHELL_AXES};
    use crate::{rigid, Plane};
    use glam::Quat;

    const WALL: f32 = 3.0;

    /// The shell part scanned twice: outside (standing on its flange, pass frame = part frame)
    /// and inside (upside down, turned and moved). Returns (outside, inside, outside <- inside).
    fn shell_passes() -> (PassCloud, PassCloud, Transform) {
        let (outer, inner) = synthetic::shell_part(1.0, WALL);
        let mut outside = synthetic::pass(&outer, &Transform::default(), &synthetic::ring(Vec3::ZERO, 0.7, 0.0, 6.0, 8, 320.0), 0.08, 1.0, 40);
        outside.support = Some(Plane::through(Vec3::new(0.0, 0.0, -WALL), Vec3::Z));
        let pose = rigid(Quat::from_rotation_x(std::f32::consts::PI) * Quat::from_rotation_z(0.7), Vec3::new(40.0, -25.0, 60.0));
        let centre = pose.transform_point3(Vec3::ZERO);
        let mut inside = synthetic::pass(&inner, &pose, &synthetic::ring(centre, 1.0, 0.0, 6.0, 8, 320.0), 0.08, 1.0, 41);
        // Upside down it rests on the dome's apex.
        inside.support = Some(Plane::through(pose.transform_point3(Vec3::new(0.0, 0.0, SHELL_AXES.z)), Vec3::Z));
        (outside, inside, pose.inverse())
    }

    fn shift(by: Vec3, t: &Transform) -> Transform {
        compose(&rigid(Quat::IDENTITY, by), t)
    }

    #[test]
    fn inside_nested_in_outside_is_good_with_the_wall_thickness() {
        let (outside, inside, truth) = shell_passes();
        let config = QualityConfig::for_sample(1.0);
        for (name, q) in [("inside -> outside", evaluate(&inside, &outside, &truth, config)), ("outside -> inside", evaluate(&outside, &inside, &truth.inverse(), config))] {
            eprintln!("{name}: {q:?}, score {:.3}", q.score());
            let s = q.shell.expect("shell pairs");
            assert!((s.thickness_mm() - WALL).abs() < 0.3, "{name}: wall {:.2} mm", s.thickness_mm());
            assert!(s.iqr_mm < 0.5 && s.piercing < 0.01 && s.is_wall(), "{name}: {s:?}");
            assert!(q.below_support < 0.01 && !q.sliding(), "{name}: {q:?}");
            assert_eq!(q.verdict(), Verdict::Good, "{name}: {q:?}");
        }
    }

    #[test]
    fn surfaces_crossing_the_wall_are_not_good() {
        let (outside, inside, truth) = shell_passes();
        let config = QualityConfig::for_sample(1.0);
        let right = evaluate(&inside, &outside, &truth, config);
        // Moved sideways by more than the wall: one side pokes through the outside.
        let sideways = evaluate(&inside, &outside, &shift(Vec3::new(5.0, 0.0, 0.0), &truth), config);
        // Tilted by 3 degrees about the part's centre: the ends cross the wall.
        let tilt = compose(&compose(&rigid(Quat::IDENTITY, Vec3::new(0.0, 0.0, 20.0)), &rigid(Quat::from_rotation_y(3f32.to_radians()), Vec3::new(0.0, 0.0, -20.0))), &truth);
        let tilted = evaluate(&inside, &outside, &tilt, config);
        // The inner face moved out through the skin, 2 mm beyond the outer face.
        let mut through = inside.clone();
        through.points.iter_mut().for_each(|(p, n)| *p -= *n * (WALL + 2.0));
        let pierced = evaluate(&through, &outside, &truth, config);
        for (name, q) in [("sideways", sideways), ("tilted", tilted), ("pierced", pierced)] {
            eprintln!("{name}: {q:?}, score {:.3}", q.score());
            assert_ne!(q.verdict(), Verdict::Good, "{name}: {q:?}");
            assert!(q.score() < 0.5 * right.score(), "{name}: score {} vs {}", q.score(), right.score());
        }
        assert!(pierced.pierces() && pierced.verdict() == Verdict::Poor, "{pierced:?}");
        assert!(tilted.pierces() && tilted.verdict() == Verdict::Poor, "{tilted:?}");
        assert!(sideways.shell.is_some_and(|s| s.piercing > 0.01) && sideways.verdict() == Verdict::Poor, "{sideways:?}");
    }

    #[test]
    fn poses_through_the_table_are_poor() {
        let (outside, inside, truth) = shell_passes();
        let config = QualityConfig::for_sample(1.0);
        let under = evaluate(&inside, &outside, &shift(Vec3::new(0.0, 0.0, -30.0), &truth), config);
        eprintln!("under the table: {under:?}");
        assert!(under.below_support > 0.02 && under.verdict() == Verdict::Poor, "{under:?}");
        // The right pose, but a support plane that cuts through the other pass: vetoed in both
        // directions.
        let mut raised = outside.clone();
        raised.support = Some(Plane::through(Vec3::new(0.0, 0.0, 20.0), Vec3::Z));
        let a = evaluate(&inside, &raised, &truth, config);
        let b = evaluate(&raised, &inside, &truth.inverse(), config);
        eprintln!("raised table: {a:?}\n{b:?}");
        assert!(a.below_support > 0.02 && a.verdict() == Verdict::Poor, "{a:?}");
        assert!(b.below_support > 0.02 && b.verdict() == Verdict::Poor, "{b:?}");
    }

    #[test]
    fn flat_and_round_surfaces_slide() {
        let config = QualityConfig::for_sample(1.0);
        let pass_of = |points: Vec<Oriented>| PassCloud { points, views: Vec::new(), support: None };
        let flat = pass_of((0..60).flat_map(|i| (0..60).map(move |j| (Vec3::new(i as f32, j as f32, 0.0), Vec3::Z))).collect());
        let sphere = pass_of((0..4000).map(|i| {
            let z = 1.0 - 2.0 * (i as f32 + 0.5) / 4000.0;
            let a = 2.399_963 * i as f32;
            let n = Vec3::new((1.0 - z * z).sqrt() * a.cos(), (1.0 - z * z).sqrt() * a.sin(), z);
            (n * 40.0, n)
        }).collect());
        let object = pass_of(synthetic::object(1.0));
        let (outside, inside, truth) = shell_passes();
        for (name, cloud, slides) in [("plane", &flat, true), ("sphere", &sphere, true), ("object", &object, false)] {
            let q = evaluate(cloud, cloud, &Transform::default(), config);
            eprintln!("{name}: slide {:.2} mm, {:?}", q.slide_mm, q.verdict());
            assert_eq!(q.sliding(), slides, "{name}: {q:?}");
            if slides {
                assert_ne!(q.verdict(), Verdict::Good, "{name}");
            }
        }
        let nested = evaluate(&inside, &outside, &truth, config);
        eprintln!("shell nesting: slide {:.2} mm", nested.slide_mm);
        assert!(!nested.sliding(), "{nested:?}");
    }

    #[test]
    fn jacobi_finds_the_eigenvalues() {
        // R diag(1..6) R^T for a fixed orthonormal R (products of plane rotations).
        let mut m = [[0.0f64; 6]; 6];
        for (i, row) in m.iter_mut().enumerate() {
            row[i] = (i + 1) as f64;
        }
        for (p, q, angle) in [(0, 1, 0.3), (2, 4, -1.1), (1, 5, 0.7), (3, 4, 2.0), (0, 5, -0.4)] {
            let (c, s) = (f64::cos(angle), f64::sin(angle));
            let mut r = [[0.0f64; 6]; 6];
            for (i, row) in r.iter_mut().enumerate() {
                row[i] = 1.0;
            }
            r[p][p] = c;
            r[q][q] = c;
            r[p][q] = -s;
            r[q][p] = s;
            let mul = |a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]| -> [[f64; 6]; 6] { std::array::from_fn(|i| std::array::from_fn(|j| (0..6).map(|k| a[i][k] * b[k][j]).sum())) };
            let rt: [[f64; 6]; 6] = std::array::from_fn(|i| std::array::from_fn(|j| r[j][i]));
            m = mul(&mul(&r, &m), &rt);
        }
        let mut ev = sym_eigenvalues(m);
        ev.sort_by(f64::total_cmp);
        for (i, v) in ev.iter().enumerate() {
            assert!((v - (i + 1) as f64).abs() < 1e-9, "{ev:?}");
        }
    }

    #[test]
    fn plane_moves_with_its_frame() {
        let plane = Plane::through(Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 2.0));
        let t = rigid(Quat::from_rotation_x(0.8), Vec3::new(5.0, -4.0, 9.0));
        let moved = plane.transformed(&t);
        for p in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, -3.0, 7.0)] {
            assert!((moved.distance(t.transform_point3(p)) - plane.distance(p)).abs() < 1e-4);
        }
    }
}
