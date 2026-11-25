//! Auto-orientation of a part to its own faces: an area-weighted histogram of the surface
//! normals finds the face directions, the wall normals' scatter a main axis (turned and
//! extruded parts), and the open boundary a rim plane. The up direction comes from the rim,
//! else the axis, else the strongest face, else the shortest extent; a square face or the
//! long principal axis sets the heading. The result sits the part square to the axes with
//! its base at z = 0 and the bounding-box centre at x = y = 0.

use glam::{Mat3, Quat, Vec3};

use crate::cloud::{downsample, Index, Rng};
use crate::eigen::{eigen3, vec3_of};
use crate::{rigid, Oriented, Plane};
use scancraft_core::Transform;

/// Where the up direction came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpSource {
    /// An open rim (the base a shell part stands on).
    Rim,
    /// The main axis (turned and extruded parts).
    Axis,
    /// The largest face.
    Face,
    /// The shortest principal extent (featureless parts).
    Extent,
}

impl UpSource {
    /// The UI label.
    pub fn label(&self) -> &'static str {
        match self {
            UpSource::Rim => "open rim",
            UpSource::Axis => "main axis",
            UpSource::Face => "largest face",
            UpSource::Extent => "extent",
        }
    }
}

/// A dominant face direction and its share of the surface.
#[derive(Debug, Clone, Copy)]
pub struct FaceDirection {
    /// Unit direction in the old frame.
    pub dir: Vec3,
    /// Fraction of the surface area facing within 10 degrees of it.
    pub share: f32,
}

/// The open rim the part stands on.
#[derive(Debug, Clone)]
pub struct Rim {
    /// The rim's plane, normal pointing to the part.
    pub plane: Plane,
    /// Boundary samples on it.
    pub points: usize,
    /// RMS of the rim samples from the plane, mm.
    pub rms_mm: f32,
    /// Share of all boundary samples on it.
    pub share: f32,
}

/// How [`auto_orient`] placed the part.
#[derive(Debug, Clone)]
pub struct AutoOrient {
    /// new <- old: the part square to the axes, its base at z = 0, the bounding-box centre at
    /// x = y = 0.
    pub transform: Transform,
    /// The old-frame direction that becomes +Z.
    pub up: Vec3,
    /// Where `up` came from.
    pub up_source: UpSource,
    /// Dominant face directions, strongest first (at most 6).
    pub faces: Vec<FaceDirection>,
    /// The main axis from wall normals, when the part has one (old frame).
    pub axis: Option<Vec3>,
    /// The open rim, when found.
    pub rim: Option<Rim>,
    /// The rotation angle of `transform`, degrees.
    pub turned_deg: f32,
}

/// Tuning of [`auto_orient`].
#[derive(Debug, Clone, Copy)]
pub struct OrientConfig {
    /// Minimum point count.
    pub min_points: usize,
    /// The part's extent divided by this is the sampling voxel (~120).
    pub voxel_div: f32,
    /// Number of histogram directions on the sphere (~2500 Fibonacci points).
    pub dirs: usize,
    /// Each sample votes for directions within this angle of its normal, degrees.
    pub spread_deg: f32,
    /// Non-maximum suppression radius for histogram peaks, degrees.
    pub nms_deg: f32,
    /// A peak is refined to the mean of normals within this angle, degrees.
    pub refine_deg: f32,
    /// Minimum area share a peak needs to be a face direction.
    pub face_share: f32,
    /// The part has a main axis when the smallest eigenvalue's share of the normal-scatter
    /// trace is below this and clearly separated from the middle one (half).
    pub axis_share: f32,
    /// A sample is a boundary point when its neighbours' tangent directions leave an
    /// angular gap over this, degrees.
    pub gap_deg: f32,
    /// Boundary neighbours are collected within this multiple of the voxel.
    pub rim_neighbours: f32,
}

impl Default for OrientConfig {
    fn default() -> Self {
        OrientConfig {
            min_points: 500,
            voxel_div: 120.0,
            dirs: 2500,
            spread_deg: 8.0,
            nms_deg: 25.0,
            refine_deg: 10.0,
            face_share: 0.03,
            axis_share: 0.08,
            gap_deg: 150.0,
            rim_neighbours: 3.0,
        }
    }
}

/// `n` nearly-even directions on the sphere (Fibonacci spiral).
fn fibonacci(n: usize) -> Vec<Vec3> {
    let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
    (0..n)
        .map(|i| {
            let z = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
            let r = (1.0 - z * z).max(0.0).sqrt();
            let a = golden * i as f32;
            Vec3::new(r * a.cos(), r * a.sin(), z)
        })
        .collect()
}

/// Angular chord distance: the 3D distance of two unit vectors `deg` apart.
fn chord(deg: f32) -> f32 {
    2.0 * (deg.to_radians() / 2.0).sin()
}

/// Dominant face directions of `samples`: a histogram of `dirs` sphere directions, peaks by
/// non-maximum suppression, each refined to the mean normal within `refine` degrees. Keeps
/// peaks holding at least `min_share` of the samples, strongest first, at most 6.
fn face_directions(samples: &[Oriented], dirs: &[Vec3], config: &OrientConfig) -> Vec<FaceDirection> {
    let dir_index = Index::new(dirs.to_vec());
    let spread = chord(config.spread_deg);
    let mut hist = vec![0.0f32; dirs.len()];
    for (_, n) in samples {
        for d in dir_index.within(*n, spread) {
            hist[d] += 1.0;
        }
    }
    let mut out = Vec::new();
    let nms = chord(config.nms_deg);
    let refine = chord(config.refine_deg);
    loop {
        let Some(peak) = (0..hist.len()).max_by(|&i, &j| hist[i].total_cmp(&hist[j])) else { break };
        if hist[peak] <= 0.0 {
            break;
        }
        // Refine to the mean of the normals close to this direction.
        let d = dirs[peak];
        let mut mean = Vec3::ZERO;
        let mut count = 0usize;
        for (_, n) in samples {
            if n.distance(d) <= refine {
                mean += *n;
                count += 1;
            }
        }
        let share = count as f32 / samples.len() as f32;
        let dir = mean.normalize_or_zero();
        if dir != Vec3::ZERO && share >= config.face_share {
            out.push(FaceDirection { dir, share });
        }
        for s in dir_index.within(d, nms) {
            hist[s] = 0.0;
        }
        if out.len() >= 6 {
            break;
        }
    }
    out
}

/// The unit vector minimising the mean of (n . a)^2 over the samples — the direction the
/// walls of a turned or extruded part face square to — when the smallest eigenvalue is a
/// clearly separated small share of the trace.
fn main_axis(samples: &[Oriented], config: &OrientConfig) -> Option<Vec3> {
    let mut s = [[0.0f64; 3]; 3];
    for (_, n) in samples {
        let v = n.as_dvec3();
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += v[i] * v[j];
            }
        }
    }
    let (values, vectors) = eigen3(s);
    let trace = values.iter().sum::<f64>();
    if trace <= 0.0 {
        return None;
    }
    if values[0] / trace < config.axis_share as f64 && values[0] < 0.5 * values[1] {
        Some(vec3_of(&vectors, 0))
    } else {
        None
    }
}

/// Indices of the boundary samples: those whose neighbours within `radius`, projected into
/// the tangent plane, leave an angular gap over `gap_deg` (or that have too few neighbours
/// to show a gap).
fn boundary_samples(samples: &[Oriented], index: &Index, radius: f32, gap_deg: f32) -> Vec<usize> {
    let mut out = Vec::new();
    for (i, (p, n)) in samples.iter().enumerate() {
        let around: Vec<usize> = index.within(*p, radius).into_iter().filter(|&j| j != i).collect();
        if around.len() < 4 {
            out.push(i);
            continue;
        }
        let u = if n.x.abs() < n.y.abs() { n.cross(Vec3::X) } else { n.cross(Vec3::Y) }.normalize();
        let v = n.cross(u);
        let mut angles: Vec<f32> = around.iter().map(|&j| { let w = samples[j].0 - *p; (w.dot(v)).atan2(w.dot(u)) }).collect();
        angles.sort_by(f32::total_cmp);
        let mut gap = 0.0f32;
        for k in 0..angles.len() {
            let next = if k + 1 == angles.len() { angles[0] + std::f32::consts::TAU } else { angles[k + 1] };
            gap = gap.max(next - angles[k]);
        }
        if gap.to_degrees() > gap_deg {
            out.push(i);
        }
    }
    out
}

/// What the open-rim search saw, for diagnosing why a rim was or was not found (see
/// [`rim_report`]): every check's numbers plus which one rejected the candidate.
#[derive(Clone, Debug)]
pub struct RimReport {
    /// Samples the search ran on (the voxel grid).
    pub samples: usize,
    /// Voxel size the samples were taken at, mm.
    pub voxel_mm: f32,
    /// Boundary samples found (angular gap in the tangent plane).
    pub boundary: usize,
    /// Plane tolerance for inliers and the one-sidedness test, mm.
    pub tolerance_mm: f32,
    /// Boundary samples on the best plane found.
    pub inliers: usize,
    /// Their share of all boundary samples.
    pub share: f32,
    /// RMS distance of the inliers from the best plane, mm.
    pub rms_mm: f32,
    /// All samples more than `tolerance_mm` on each side of the plane (before its normal is
    /// oriented towards the part).
    pub sides: (usize, usize),
    /// In-plane spread of the inliers along the two plane axes, mm.
    pub spread_mm: [f32; 2],
    /// The part's extent the spreads are compared with, mm.
    pub extent_mm: f32,
    /// Which check rejected the rim: "few boundary samples", "share", "one-sided", "spread".
    pub rejected_by: Option<&'static str>,
    /// The rim when every check passed.
    pub rim: Option<Rim>,
}

/// The open rim: a RANSAC plane through the boundary samples is a rim when it holds at least
/// 30 % of them (and at least 60 — interior holes and ragged scan borders are boundary
/// samples too and dilute the share), they spread over half the part's extent in two
/// directions, and nearly all samples lie on one side of it. Its normal points to the part.
fn rim_search(samples: &[Oriented], index: &Index, extent: f32, voxel: f32, config: &OrientConfig) -> RimReport {
    let boundary = boundary_samples(samples, index, config.rim_neighbours * voxel, config.gap_deg);
    let tol = (2.0 * voxel).max(0.005 * extent);
    let mut report = RimReport {
        samples: samples.len(),
        voxel_mm: voxel,
        boundary: boundary.len(),
        tolerance_mm: tol,
        inliers: 0,
        share: 0.0,
        rms_mm: 0.0,
        sides: (0, 0),
        spread_mm: [0.0; 2],
        extent_mm: extent,
        rejected_by: None,
        rim: None,
    };
    if boundary.len() < 60 {
        report.rejected_by = Some("few boundary samples");
        return report;
    }
    let mut rng = Rng::new(71053);
    let mut best: Option<(Plane, Vec<usize>)> = None;
    for _ in 0..300 {
        let (i, j, k) = (boundary[rng.below(boundary.len())], boundary[rng.below(boundary.len())], boundary[rng.below(boundary.len())]);
        if i == j || j == k || i == k {
            continue;
        }
        let (a, b, c) = (samples[i].0, samples[j].0, samples[k].0);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let pl = Plane::through(a, n);
        let inliers: Vec<usize> = boundary.iter().copied().filter(|&i| pl.distance(samples[i].0).abs() <= tol).collect();
        if best.as_ref().map_or(true, |(_, v)| inliers.len() > v.len()) {
            best = Some((pl, inliers));
        }
    }
    let Some((mut pl, inliers)) = best else { return report };
    report.inliers = inliers.len();
    report.share = inliers.len() as f32 / boundary.len() as f32;
    report.rms_mm = (inliers.iter().map(|&i| pl.distance(samples[i].0).powi(2)).sum::<f32>() / inliers.len() as f32).sqrt();
    // Nearly all samples on one side; orient the normal so they are on its +side.
    let below = samples.iter().filter(|(p, _)| pl.distance(*p) < -tol).count();
    let above = samples.iter().filter(|(p, _)| pl.distance(*p) > tol).count();
    report.sides = (below, above);
    // Spread over half the extent in two directions within the plane.
    let un = if below > above { -pl.normal } else { pl.normal };
    let u = if un.x.abs() < un.y.abs() { un.cross(Vec3::X) } else { un.cross(Vec3::Y) }.normalize();
    let v = un.cross(u);
    let extent_in = |d: Vec3| inliers.iter().map(|&i| samples[i].0.dot(d)).fold((f32::MAX, f32::MIN), |(lo, hi), t| (lo.min(t), hi.max(t)));
    let (a, b) = (extent_in(u), extent_in(v));
    report.spread_mm = [a.1 - a.0, b.1 - b.0];
    // The checks, in order: share, one-sidedness (the minority side keeps under ~3 %), spread.
    if inliers.len() < 60 || inliers.len() * 10 < boundary.len() * 3 {
        report.rejected_by = Some("share");
        return report;
    }
    if below.min(above) * 33 > samples.len() {
        report.rejected_by = Some("one-sided");
        return report;
    }
    if below > above {
        pl.normal = -pl.normal;
        pl.d = -pl.d;
    }
    if report.spread_mm[0] < 0.5 * extent || report.spread_mm[1] < 0.5 * extent {
        report.rejected_by = Some("spread");
        return report;
    }
    report.rim = Some(Rim { plane: pl, points: inliers.len(), rms_mm: report.rms_mm, share: report.share });
    report
}

fn find_rim(samples: &[Oriented], index: &Index, extent: f32, voxel: f32, config: &OrientConfig) -> Option<Rim> {
    rim_search(samples, index, extent, voxel, config).rim
}

/// Runs the open-rim search [`auto_orient`] does (same code path) and reports what it saw:
/// for diagnosing why a part did or did not stand on its rim. None for fewer than
/// `config.min_points` points.
pub fn rim_report(points: &[Oriented], config: &OrientConfig) -> Option<RimReport> {
    if points.len() < config.min_points {
        return None;
    }
    let extent = crate::cloud::extent(points);
    if extent < 1e-3 {
        return None;
    }
    let voxel = extent / config.voxel_div;
    let samples = downsample(points, voxel);
    let index = Index::new(samples.iter().map(|p| p.0).collect());
    Some(rim_search(&samples, &index, extent, voxel, config))
}

/// Auto-orients a part to its own faces: returns the `new <- old` transform that sits it
/// square to the axes with the base at z = 0 and the bounding-box centre at x = y = 0, plus
/// the evidence the choice was made from (face directions, main axis, open rim). None for
/// fewer than `config.min_points` points.
///
/// The samples are a voxel grid (extent / `voxel_div`) so every sample stands for about the
/// same area — that is the area weighting.
pub fn auto_orient(points: &[Oriented], config: &OrientConfig) -> Option<AutoOrient> {
    if points.len() < config.min_points {
        return None;
    }
    let extent = crate::cloud::extent(points);
    if extent < 1e-3 {
        return None;
    }
    let voxel = extent / config.voxel_div;
    let samples = downsample(points, voxel);
    if samples.len() < 50 {
        return None;
    }
    let index = Index::new(samples.iter().map(|p| p.0).collect());
    let dirs = fibonacci(config.dirs);
    let faces = face_directions(&samples, &dirs, config);
    let axis = main_axis(&samples, config);
    let rim = find_rim(&samples, &index, extent, voxel, config);
    // Up: the rim's normal, else the main axis, else the face direction closest to the
    // current +Z or -Z, else the shortest principal extent. Not from the rim: the sign
    // closer to the current +Z.
    let toward_z = |d: Vec3| if d.z < 0.0 { -d } else { d };
    let (up, up_source) = if let Some(r) = &rim {
        (r.plane.normal, UpSource::Rim)
    } else if let Some(a) = axis {
        (toward_z(a), UpSource::Axis)
    } else if let Some(f) = faces.iter().max_by(|a, b| a.dir.z.abs().total_cmp(&b.dir.z.abs())) {
        (toward_z(f.dir), UpSource::Face)
    } else {
        // A blob: stand it on its shortest principal extent.
        let mut s = [[0.0f64; 3]; 3];
        let c = crate::cloud::centroid(&samples);
        for (p, _) in &samples {
            let v = (*p - c).as_dvec3();
            for i in 0..3 {
                for j in 0..3 {
                    s[i][j] += v[i] * v[j];
                }
            }
        }
        let (_, vecs) = eigen3(s);
        (toward_z(vec3_of(&vecs, 0)), UpSource::Extent)
    };
    // Heading: the strongest face direction square to up (projected); among it, its opposite
    // and the two perpendiculars, the one closest to the current +X becomes +X. Else the long
    // principal axis on the floor (extent ratio > 1.15), else the current +X projected.
    let on_floor = |d: Vec3| (d - up * d.dot(up)).normalize_or_zero();
    let square10 = 10.0f32.to_radians().sin();
    let mut x = faces
        .iter()
        .find(|f| f.dir.dot(up).abs() < square10)
        .map(|f| {
            let d = on_floor(f.dir);
            let perp = up.cross(d);
            [d, -d, perp, -perp].into_iter().max_by(|a, b| a.x.total_cmp(&b.x)).unwrap()
        });
    if x.is_none() {
        // Principal extents of the samples projected on the floor.
        let (u, v) = { let u = if up.x.abs() < up.y.abs() { up.cross(Vec3::X) } else { up.cross(Vec3::Y) }.normalize(); (u, up.cross(u)) };
        let c = crate::cloud::centroid(&samples);
        let (mut sxx, mut sxy, mut syy) = (0.0f64, 0.0f64, 0.0f64);
        for (p, _) in &samples {
            let (a, b) = ((*p - c).dot(u) as f64, (*p - c).dot(v) as f64);
            sxx += a * a;
            sxy += a * b;
            syy += b * b;
        }
        // The 2x2 eigenvector of the bigger eigenvalue, and the spread ratio along it.
        let (t, det, dd) = ((sxx + syy) / 2.0, sxx * syy - sxy * sxy, (((sxx - syy) / 2.0).powi(2) + sxy * sxy).sqrt());
        let _ = det;
        let e = t + dd;
        let dir2 = if (sxy).abs() > 1e-9 { (e - syy, sxy) } else if sxx >= syy { (1.0, 0.0) } else { (0.0, 1.0) };
        let long = (u * dir2.0 as f32 + v * dir2.1 as f32).normalize_or_zero();
        if long != Vec3::ZERO {
            // Extent ratio along vs square to `long` on the floor.
            let perp = up.cross(long);
            let range = |d: Vec3| samples.iter().map(|(p, _)| p.dot(d)).fold((f32::MAX, f32::MIN), |(l, h), t| (l.min(t), h.max(t)));
            let (l, w) = (range(long), range(perp));
            if (w.1 - w.0) > 1e-6 && (l.1 - l.0) / (w.1 - w.0) > 1.15 {
                x = Some(if long.x >= 0.0 { long } else { -long });
            }
        }
        if x.is_none() {
            x = Some(if on_floor(Vec3::X) != Vec3::ZERO { on_floor(Vec3::X) } else { u });
        }
    }
    let x = x.unwrap();
    let y = up.cross(x);
    let rot = Quat::from_mat3(&Mat3::from_cols(x, y, up).transpose());
    // Translation: base at z = 0 (the rim plane, else the 0.5 % quantile of rotated z), and
    // the bounding-box centre at x = y = 0.
    let mut zs: Vec<f32> = Vec::with_capacity(samples.len());
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (p, _) in &samples {
        let r = rot * *p;
        zs.push(r.z);
        lo = lo.min(r);
        hi = hi.max(r);
    }
    let t = if let Some(r) = &rim {
        let mut tt = Vec3::new(-(lo.x + hi.x) / 2.0, -(lo.y + hi.y) / 2.0, 0.0);
        // The rim plane maps to z = 0: rotated plane offset is -d (normal -> +Z).
        tt.z = r.plane.d;
        tt
    } else {
        zs.sort_by(f32::total_cmp);
        let q = zs[(samples.len() / 200).min(zs.len() - 1)];
        Vec3::new(-(lo.x + hi.x) / 2.0, -(lo.y + hi.y) / 2.0, -q)
    };
    let transform = rigid(rot, t);
    let turned_deg = crate::rotation_angle(rot, Quat::IDENTITY).to_degrees();
    Some(AutoOrient { transform, up, up_source, faces, axis, rim, turned_deg })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Rng;
    use crate::synthetic;

    /// A box (half extents) sampled on all faces, noise `sigma` along the normal, `rot` the
    /// part's attitude in the world.
    fn box_cloud(half: Vec3, spacing: f32, rot: Quat, sigma: f32, seed: u64) -> Vec<Oriented> {
        let mut rng = Rng::new(seed);
        let mut out = Vec::new();
        for axis in 0..3 {
            for sign in [-1.0f32, 1.0] {
                let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
                let (na, nb) = ((2.0 * half[a] / spacing).ceil() as i32, (2.0 * half[b] / spacing).ceil() as i32);
                for i in 0..=na {
                    for j in 0..=nb {
                        let mut p = Vec3::ZERO;
                        p[axis] = sign * half[axis];
                        p[a] = -half[a] + 2.0 * half[a] * i as f32 / na as f32;
                        p[b] = -half[b] + 2.0 * half[b] * j as f32 / nb as f32;
                        let mut n = Vec3::ZERO;
                        n[axis] = sign;
                        out.push((rot * (p + n * rng.normal() * sigma), rot * n));
                    }
                }
            }
        }
        out
    }

    /// A tube (thin wall, both ends open): outer wall normals out, inner wall normals in,
    /// annular end faces, along `rot`'s +Z.
    fn tube_cloud(r: f32, wall: f32, len: f32, spacing: f32, rot: Quat, sigma: f32, seed: u64) -> Vec<Oriented> {
        let mut rng = Rng::new(seed);
        let mut out = Vec::new();
        let around = (std::f32::consts::TAU * r / spacing).ceil() as usize;
        let along = (len / spacing).ceil() as usize;
        for i in 0..around {
            let a = std::f32::consts::TAU * i as f32 / around as f32;
            let n = Vec3::new(a.cos(), a.sin(), 0.0);
            for j in 0..=along {
                let z = -len / 2.0 + len * j as f32 / along as f32;
                out.push((rot * (n * r + Vec3::Z * z + n * rng.normal() * sigma), rot * n));
                out.push((rot * (n * (r - wall) + Vec3::Z * z) - rot * n * rng.normal() * sigma, rot * -n));
            }
            // The annular end faces.
            let steps = (wall / spacing).ceil() as usize;
            for k in 0..=steps {
                let rr = r - wall + wall * k as f32 / steps.max(1) as f32;
                out.push((rot * (Vec3::new(rr * a.cos(), rr * a.sin(), len / 2.0) + Vec3::Z * rng.normal() * sigma), rot * Vec3::Z));
                out.push((rot * (Vec3::new(rr * a.cos(), rr * a.sin(), -len / 2.0) - Vec3::Z * rng.normal() * sigma), rot * -Vec3::Z));
            }
        }
        out
    }

    /// A Fibonacci sphere of `radius`, noise `sigma` along the normal.
    fn sphere_cloud(radius: f32, spacing: f32, sigma: f32, seed: u64) -> Vec<Oriented> {
        let mut rng = Rng::new(seed);
        let count = (4.0 * std::f32::consts::PI * radius * radius / (spacing * spacing)) as usize;
        let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
        (0..count)
            .map(|i| {
                let z = 1.0 - 2.0 * (i as f32 + 0.5) / count as f32;
                let r = (1.0 - z * z).sqrt();
                let a = golden * i as f32;
                let n = Vec3::new(r * a.cos(), r * a.sin(), z);
                (n * radius + n * rng.normal() * sigma, n)
            })
            .collect()
    }

    /// Apply `t` to a cloud (positions and normals).
    fn moved(points: &[Oriented], t: &Transform) -> Vec<Oriented> {
        points.iter().map(|&(p, n)| (t.transform_point3(p), (t.rotation * n).normalize_or_zero())).collect()
    }

    #[test]
    fn a_tilted_box_stands_back_on_its_faces() {
        let half = Vec3::new(40.0, 25.0, 15.0);
        let rot = Quat::from_rotation_y(0.6) * Quat::from_rotation_z(0.4);
        let points = box_cloud(half, 1.0, rot, 0.1, 3);
        let result = auto_orient(&points, &OrientConfig::default()).expect("oriented");
        eprintln!("box: up {:?} {} ({} faces, axis {:?}, rim {:?}), turned {:.2} deg", result.up_source, result.up, result.faces.len(), result.axis, result.rim.is_some(), result.turned_deg);
        // The tilt is undone: every face normal sits on a world axis now.
        let world = [Vec3::X, Vec3::Y, Vec3::Z, -Vec3::X, -Vec3::Y, -Vec3::Z];
        for (i, f) in result.faces.iter().enumerate() {
            let d = result.transform.rotation * f.dir;
            let err = world.iter().map(|w| d.angle_between(*w).to_degrees()).fold(f32::MAX, f32::min);
            eprintln!("  face {i}: dir {:?} -> {d}, {err:.3} deg off, share {:.2}", f.dir, f.share);
            assert!(err < 1.0, "face {i} {err} deg off an axis");
        }
        assert_eq!(result.up_source, UpSource::Face);
        // The chosen up was the face direction closest to the old +Z (a small tilt undone,
        // not a swap).
        let best = result.faces.iter().max_by(|a, b| a.dir.z.abs().total_cmp(&b.dir.z.abs())).unwrap().dir;
        let best = if best.z < 0.0 { -best } else { best };
        assert!(result.up.angle_between(best).to_degrees() < 1.0, "up {} vs {best}", result.up);
        // Base at z = 0.
        let flat = moved(&points, &result.transform);
        let min_z = flat.iter().map(|(p, _)| p.z).fold(f32::MAX, f32::min);
        eprintln!("  min z {min_z:.3} mm");
        assert!(min_z.abs() < 0.35, "min z {min_z}");
        let (lo, hi) = flat.iter().map(|(p, _)| *p).fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(l, h), p| (l.min(p), h.max(p)));
        assert!(((lo.x + hi.x) / 2.0).abs() < 0.6 && ((lo.y + hi.y) / 2.0).abs() < 0.6, "centre {lo} {hi}");
    }

    #[test]
    fn an_already_square_box_barely_turns() {
        let points = box_cloud(Vec3::new(40.0, 25.0, 15.0), 1.2, Quat::IDENTITY, 0.05, 4);
        let result = auto_orient(&points, &OrientConfig::default()).expect("oriented");
        eprintln!("square box: turned {:.3} deg, up {:?}", result.turned_deg, result.up_source);
        assert!(result.turned_deg < 1.0, "turned {}", result.turned_deg);
    }

    #[test]
    fn a_shell_part_stands_on_its_rim() {
        let (outside, inside) = synthetic::shell_part(0.9, 3.0);
        let mut part: Vec<Oriented> = outside.iter().chain(inside.iter()).copied().collect();
        let tilt = rigid(Quat::from_rotation_y(0.45) * Quat::from_rotation_x(-0.3) * Quat::from_rotation_z(0.2), Vec3::new(60.0, -40.0, 150.0));
        part = moved(&part, &tilt);
        let result = auto_orient(&part, &OrientConfig::default()).expect("oriented");
        let rim = result.rim.as_ref().expect("a rim");
        eprintln!("shell: up {:?} {}, rim plane n {} d {:.2} ({} pts, share {:.2}, rms {:.2} mm)", result.up_source, result.up, rim.plane.normal, rim.plane.d, rim.points, rim.share, rim.rms_mm);
        assert_eq!(result.up_source, UpSource::Rim);
        // The rim plane maps to z = 0 within 1 deg / half a mm.
        let rn = result.transform.rotation * rim.plane.normal;
        assert!(rn.angle_between(Vec3::Z).to_degrees() < 1.0, "rim normal {rn}");
        let on_rim = |p: Vec3| (result.transform.transform_point3(p)).z;
        let rim_pt = rim.plane.normal * -rim.plane.d;
        assert!(on_rim(rim_pt).abs() < 0.5, "rim z {}", on_rim(rim_pt));
        // The dome is above.
        let dz = part.iter().take(outside.len()).map(|&(p, _)| p).sum::<Vec3>() / outside.len() as f32;
        assert!(result.transform.transform_point3(dz).z > 0.0);
        // Turned upside down first: still dome up.
        let flipped = moved(&part, &rigid(Quat::from_rotation_x(std::f32::consts::PI) * Quat::from_rotation_z(0.9), Vec3::new(-30.0, 70.0, 90.0)));
        let result2 = auto_orient(&flipped, &OrientConfig::default()).expect("oriented");
        eprintln!("flipped: up {:?} {}, rim {:?}", result2.up_source, result2.up, result2.rim.is_some());
        assert_eq!(result2.up_source, UpSource::Rim);
        let dz2 = flipped.iter().take(outside.len()).map(|&(p, _)| p).sum::<Vec3>() / outside.len() as f32;
        assert!(result2.transform.transform_point3(dz2).z > 0.0, "dome down after flip");
    }

    #[test]
    fn a_shell_part_with_holes_in_the_dome_still_stands_on_its_rim() {
        // Hole borders are boundary samples like the rim and dilute its share (real scans
        // look like this): the rim must still be found when it holds well under half.
        let (outside, inside) = synthetic::shell_part(0.9, 3.0);
        let (a, b, c) = (synthetic::SHELL_AXES.x, synthetic::SHELL_AXES.y, synthetic::SHELL_AXES.z);
        let dome = |phi_deg: f32, theta_deg: f32| {
            let (p, t) = (phi_deg.to_radians(), theta_deg.to_radians());
            Vec3::new(a * p.cos() * t.cos(), b * p.cos() * t.sin(), c * p.sin())
        };
        let mut holes = Vec::new();
        for k in 0..8 {
            holes.push(dome(35.0, k as f32 * 45.0));
            holes.push(dome(60.0, k as f32 * 45.0 + 22.5));
        }
        for k in 0..4 {
            holes.push(dome(80.0, k as f32 * 90.0 + 45.0));
        }
        let r = 12.0;
        let kept_out: Vec<Oriented> = outside.iter().copied().filter(|(p, _)| holes.iter().all(|h| p.distance(*h) > r)).collect();
        let kept_in: Vec<Oriented> = inside.iter().copied().filter(|(p, _)| holes.iter().all(|h| p.distance(*h) > r)).collect();
        let mut part: Vec<Oriented> = kept_out.iter().chain(kept_in.iter()).copied().collect();
        let tilt = rigid(Quat::from_rotation_y(0.45) * Quat::from_rotation_x(-0.3) * Quat::from_rotation_z(0.2), Vec3::new(60.0, -40.0, 150.0));
        part = moved(&part, &tilt);
        let report = rim_report(&part, &OrientConfig::default()).expect("a report");
        eprintln!("holed shell: {} samples, {} boundary, {} rim inliers (share {:.2}), rms {:.2} mm, rejected {:?}", report.samples, report.boundary, report.inliers, report.share, report.rms_mm, report.rejected_by);
        let result = auto_orient(&part, &OrientConfig::default()).expect("oriented");
        let rim = result.rim.as_ref().expect("a rim under the hole borders");
        eprintln!("  up {:?} {}, rim plane n {} d {:.2}", result.up_source, result.up, rim.plane.normal, rim.plane.d);
        assert_eq!(result.up_source, UpSource::Rim);
        let rn = result.transform.rotation * rim.plane.normal;
        assert!(rn.angle_between(Vec3::Z).to_degrees() < 1.0, "rim normal {rn}");
        // The dome is above the rim still.
        let dz = part.iter().take(kept_out.len()).map(|&(p, _)| p).sum::<Vec3>() / kept_out.len() as f32;
        assert!(result.transform.transform_point3(dz).z > 0.0);
    }

    #[test]
    fn a_tube_stands_on_its_axis() {
        let rot = Quat::from_rotation_y(0.5) * Quat::from_rotation_z(-0.4);
        let points = tube_cloud(20.0, 2.0, 60.0, 1.0, rot, 0.05, 6);
        let result = auto_orient(&points, &OrientConfig::default()).expect("oriented");
        eprintln!("tube: up {:?} {}, axis {:?}, rim {:?}", result.up_source, result.up, result.axis, result.rim.is_some());
        let axis = result.axis.expect("a main axis");
        let err = axis.angle_between(rot * Vec3::Z).min(axis.angle_between(rot * -Vec3::Z)).to_degrees();
        eprintln!("  axis {err:.3} deg off, up {:?}", result.up_source);
        assert!(err < 1.0, "axis {err} deg");
        assert!(matches!(result.up_source, UpSource::Axis | UpSource::Rim));
        // Whatever the source, up became +Z: the axis direction is vertical now.
        let up_img = result.transform.rotation * axis;
        assert!(up_img.z.abs() > 0.999, "axis -> Z: {up_img}");
    }

    #[test]
    fn a_sphere_and_too_few_points() {
        let sphere = sphere_cloud(30.0, 0.8, 0.1, 8);
        let result = auto_orient(&sphere, &OrientConfig::default()).expect("oriented");
        eprintln!("sphere: up {:?} {}, turned {:.2} deg, faces {}", result.up_source, result.up, result.turned_deg, result.faces.len());
        assert!(result.turned_deg.is_finite());
        assert!(auto_orient(&sphere[..400], &OrientConfig::default()).is_none());
    }

    #[test]
    fn half_a_million_points_orient_in_time() {
        let t0 = std::time::Instant::now();
        let mut part = sphere_cloud(60.0, 0.55, 0.05, 9);
        part.extend(box_cloud(Vec3::new(35.0, 35.0, 25.0), 0.5, Quat::IDENTITY, 0.05, 10).into_iter().filter(|(p, _)| p.z < -20.0));
        // ~500k points.
        while part.len() < 500_000 {
            part.extend_from_within(..);
        }
        part.truncate(500_000);
        let result = auto_orient(&part, &OrientConfig::default());
        eprintln!("500k points ({} after sampling): {:?} in {:?}", part.len(), result.as_ref().map(|r| r.up_source), t0.elapsed());
        assert!(result.is_some() && t0.elapsed().as_secs_f32() < 2.0, "took {:?}", t0.elapsed());
    }
}
