//! The simulated scanner: rays from every view of a pass, cast against the scene (part, stickers,
//! the pass's table) with occlusion; noise along the ray that grows at grazing incidence;
//! coverage lost past a maximum incidence angle; the views fused into one cloud at the pass's
//! point spacing; normals from a plane fit of the noisy neighbours; then the pass's faults
//! (marker cut-outs, dropout, laser strays, offsets).

use std::collections::HashMap;

use glam::DVec3;
use rayon::prelude::*;
use serde::Serialize;

use crate::rng::{value_noise, Rng};
use crate::scene::{NormalMode, PassSpec, Scene, Sticker};
use crate::sdf::{sdf_normal, v3};

/// What a scanned point lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum PointKind {
    Part,
    /// On sticker `i` of the scene.
    Sticker(u32),
    Table,
    /// A laser-line stray behind the surface.
    Stray,
}

/// One point of a scanned pass (part frame).
#[derive(Debug, Clone, Copy)]
pub struct ScanPoint {
    /// As measured (noise, bias, offset).
    pub pos: DVec3,
    /// As the pipeline would have it (see [`NormalMode`]).
    pub normal: DVec3,
    /// Where the ray really hit.
    pub true_pos: DVec3,
    pub true_normal: DVec3,
    /// Direction from the scanner.
    pub ray: DVec3,
    /// Cosine of the incidence angle.
    pub cos: f64,
    /// Rays of all views that fell into this point's cell.
    pub hits: u32,
    pub kind: PointKind,
}

/// A marker as the scanner software lists it (part frame).
#[derive(Debug, Clone, Copy)]
pub struct ListedMarker {
    pub sticker: usize,
    pub centre: DVec3,
    pub normal: DVec3,
    pub radius_mm: f32,
    /// Points of this pass on the sticker's dot.
    pub dot_points: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, serde::Deserialize)]
pub struct PassStats {
    pub rays: usize,
    pub hits: usize,
    pub lost_grazing: usize,
    pub fused: usize,
    pub clipped: usize,
    pub dropout: usize,
    pub cut: usize,
    pub strays: usize,
    pub part: usize,
    pub sticker: usize,
    pub table: usize,
    pub points: usize,
}

pub struct PassScan {
    pub points: Vec<ScanPoint>,
    pub listed: Vec<ListedMarker>,
    pub stats: PassStats,
}

#[derive(Clone, Copy)]
struct Hit {
    true_pos: DVec3,
    true_normal: DVec3,
    ray: DVec3,
    cos: f64,
    pos: DVec3,
    kind: PointKind,
}

/// Distance of the scene as pass `pass` sees it: part, its table, the stickers on both.
pub fn pass_sdf<'a>(scene: &'a Scene, pass: usize) -> impl Fn(DVec3) -> f64 + Sync + 'a {
    let table = scene.passes[pass].table.as_ref();
    let stickers: Vec<&Sticker> = scene.stickers.iter().filter(|s| s.on_table_of_pass.is_none_or(|p| p == pass)).collect();
    move |p: DVec3| {
        let mut d = scene.part.sdf(p);
        if let Some(t) = table {
            d = d.min(t.sdf(p));
        }
        d - stickers.iter().map(|s| s.thickness(p)).sum::<f64>()
    }
}

/// First crossing of the zero set along o + t d in [t0, t1].
pub fn march(f: &impl Fn(DVec3) -> f64, o: DVec3, d: DVec3, t0: f64, t1: f64) -> Option<f64> {
    let mut t = t0;
    let mut prev_t = t0;
    let mut prev_d = f(o + d * t);
    if prev_d <= 0.0 {
        return None;
    }
    for _ in 0..4000 {
        t += (prev_d * 0.6).max(0.002);
        if t > t1 {
            return None;
        }
        let dv = f(o + d * t);
        if dv <= 1e-6 {
            if dv >= 0.0 {
                return Some(t);
            }
            let (mut a, mut b) = (prev_t, t);
            for _ in 0..40 {
                let m = 0.5 * (a + b);
                if f(o + d * m) > 0.0 {
                    a = m;
                } else {
                    b = m;
                }
            }
            return Some(0.5 * (a + b));
        }
        prev_t = t;
        prev_d = dv;
    }
    None
}

fn ray_box(o: DVec3, d: DVec3, lo: DVec3, hi: DVec3) -> Option<(f64, f64)> {
    let mut t0: f64 = 0.0;
    let mut t1 = f64::INFINITY;
    for i in 0..3 {
        if d[i].abs() < 1e-12 {
            if o[i] < lo[i] || o[i] > hi[i] {
                return None;
            }
            continue;
        }
        let (a, b) = ((lo[i] - o[i]) / d[i], (hi[i] - o[i]) / d[i]);
        t0 = t0.max(a.min(b));
        t1 = t1.min(a.max(b));
    }
    (t0 < t1).then_some((t0, t1))
}

/// The box rays are cast into for pass `pass`: the part's bounds, and room round it for a table.
pub fn march_box(scene: &Scene, pass: usize) -> (DVec3, DVec3) {
    let (mut lo, mut hi) = (v3(scene.bounds[0]) - DVec3::splat(2.0), v3(scene.bounds[1]) + DVec3::splat(2.0));
    if scene.passes[pass].table.is_some() {
        lo -= DVec3::new(30.0, 30.0, 6.0);
        hi += DVec3::new(30.0, 30.0, 0.0);
    }
    (lo, hi)
}

fn spacing_at(pass: &PassSpec, p: DVec3) -> (u8, f64) {
    for (k, r) in pass.density.iter().enumerate() {
        if r.region.contains(p) {
            return (k as u8 + 1, r.spacing_mm);
        }
    }
    (0, pass.spacing_mm)
}

/// A fusion cell: density region and grid index.
type CellKey = (u8, i64, i64, i64);

/// Scans pass `pass` of `scene`.
pub fn scan_pass(scene: &Scene, pass_index: usize) -> PassScan {
    let pass = &scene.passes[pass_index];
    let f = pass_sdf(scene, pass_index);
    let (lo, hi) = march_box(scene, pass_index);
    let min_spacing = pass.density.iter().map(|r| r.spacing_mm).fold(pass.spacing_mm, f64::min);
    let cos_max = pass.max_incidence_deg.to_radians().cos();
    let thin_from = (pass.max_incidence_deg - 6.0).to_radians().cos();
    let mut stats = PassStats::default();

    // 1. Rays of every view.
    let mut hits: Vec<Hit> = Vec::new();
    for (vi, view) in pass.views.iter().enumerate() {
        let eye = v3(view.eye);
        let target = v3(view.target);
        let forward = (target - eye).normalize();
        let hint = if forward.z.abs() > 0.95 { DVec3::Y } else { DVec3::Z };
        let right = forward.cross(hint).normalize();
        let up = right.cross(forward);
        let (mut tx0, mut tx1, mut ty0, mut ty1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..8 {
            let c = DVec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z });
            let v = c - eye;
            let z = v.dot(forward).max(1.0);
            tx0 = tx0.min(v.dot(right) / z);
            tx1 = tx1.max(v.dot(right) / z);
            ty0 = ty0.min(v.dot(up) / z);
            ty1 = ty1.max(v.dot(up) / z);
        }
        let pitch = 0.7 * min_spacing / (target - eye).length();
        let nx = ((tx1 - tx0) / pitch).ceil() as usize + 1;
        let ny = ((ty1 - ty0) / pitch).ceil() as usize + 1;
        stats.rays += nx * ny;
        let rows: Vec<Vec<Hit>> = (0..ny)
            .into_par_iter()
            .map(|iy| {
                let mut row = Vec::new();
                for ix in 0..nx {
                    let d = (forward + right * (tx0 + ix as f64 * pitch) + up * (ty0 + iy as f64 * pitch)).normalize();
                    let Some((t0, t1)) = ray_box(eye, d, lo, hi) else { continue };
                    let Some(t) = march(&f, eye, d, t0, t1) else { continue };
                    let p = eye + d * t;
                    let n = sdf_normal(&f, p);
                    let cos = -d.dot(n);
                    let mut rng = Rng::for_parts(&[scene.seed, pass_index as u64, vi as u64, (iy * nx + ix) as u64]);
                    if cos < cos_max {
                        row.push(Hit { true_pos: p, true_normal: n, ray: d, cos: -1.0, pos: p, kind: PointKind::Part });
                        continue;
                    }
                    if cos < thin_from && rng.uniform() > (cos - cos_max) / (thin_from - cos_max) {
                        row.push(Hit { true_pos: p, true_normal: n, ray: d, cos: -1.0, pos: p, kind: PointKind::Part });
                        continue;
                    }
                    let kind = classify(scene, pass_index, p);
                    let sigma = pass.noise_mm / cos.max(0.15).powf(pass.grazing_noise);
                    let pos = p + d * (sigma * rng.gauss() + pass.depth_bias_mm);
                    row.push(Hit { true_pos: p, true_normal: n, ray: d, cos, pos, kind });
                }
                row
            })
            .collect();
        for row in rows {
            for h in row {
                if h.cos < 0.0 {
                    stats.lost_grazing += 1;
                } else {
                    hits.push(h);
                }
            }
        }
    }
    stats.hits = hits.len() + stats.lost_grazing;

    // 2. Fuse: one point per cell (and sheet side), the most frontal one.
    // Several sheets can share a cell (both faces of a thin wall): a hit joins the entry of the
    // cell whose normal faces the same way.
    let mut cells: HashMap<CellKey, Vec<(usize, u32)>> = HashMap::new();
    for (k, h) in hits.iter().enumerate() {
        let (region, s) = spacing_at(pass, h.true_pos);
        let c = (h.true_pos / s).floor();
        let list = cells.entry((region, c.x as i64, c.y as i64, c.z as i64)).or_default();
        match list.iter_mut().find(|e| hits[e.0].true_normal.dot(h.true_normal) > 0.0) {
            Some(e) => {
                e.1 += 1;
                if h.cos > hits[e.0].cos {
                    e.0 = k;
                }
            }
            None => list.push((k, 1)),
        }
    }
    let mut kept: Vec<(usize, u32)> = cells.into_values().flatten().collect();
    kept.sort_unstable();
    stats.fused = kept.len();

    let mut points: Vec<ScanPoint> = Vec::with_capacity(kept.len());
    for (k, count) in kept {
        let h = hits[k];
        if !pass.clip.iter().all(|r| r.contains(h.true_pos)) {
            stats.clipped += 1;
            continue;
        }
        if let Some(d) = pass.dropout {
            let applies = d.only_z_above.is_none_or(|z| h.true_pos.z > z);
            if applies && value_noise(scene.seed ^ 0xd209 ^ pass_index as u64, h.true_pos.x, h.true_pos.y, h.true_pos.z, d.cell_mm) < d.fraction {
                stats.dropout += 1;
                continue;
            }
        }
        points.push(ScanPoint { pos: h.pos, normal: h.true_normal, true_pos: h.true_pos, true_normal: h.true_normal, ray: h.ray, cos: h.cos, hits: count, kind: h.kind });
    }
    drop(hits);

    // 3. Normals.
    if pass.normals == NormalMode::Estimated {
        estimate_normals(&mut points, pass);
    }

    // 4. The markers the scanner software would list (it saw the dot), before any cut.
    let listed = list_markers(scene, pass_index, &points);

    // 5. Marker cut-outs.
    let cuts: Vec<&Sticker> = scene.stickers.iter().filter(|s| s.cut_radius.is_some() && s.on_table_of_pass.is_none_or(|p| p == pass_index)).collect();
    if !cuts.is_empty() {
        let before = points.len();
        points.retain(|p| {
            !cuts.iter().any(|s| {
                let (lateral, axial) = s.local(p.true_pos);
                lateral < s.cut_radius.unwrap_or(0.0) && axial > -s.back_limit && axial < s.height + 1.0
            })
        });
        stats.cut = before - points.len();
    }

    // 6. Laser-line strays behind the surface.
    if let Some(st) = &pass.strays {
        let mut extra = Vec::new();
        for (k, p) in points.iter().enumerate() {
            if p.kind != PointKind::Part || !st.region.contains(p.true_pos) {
                continue;
            }
            let on_line = st.families.iter().any(|fam| {
                let t = v3(fam.normal).normalize().dot(p.true_pos) / fam.spacing_mm + fam.phase;
                (t - t.round()).abs() * fam.spacing_mm < 0.5 * st.width_mm
            });
            if !on_line {
                continue;
            }
            let mut rng = Rng::for_parts(&[scene.seed, pass_index as u64, 0x5743, k as u64]);
            if rng.uniform() >= st.fraction {
                continue;
            }
            let depth = rng.range(st.depth_mm[0], st.depth_mm[1]);
            let at = p.true_pos + p.ray * (depth + st.noise_mm * rng.gauss());
            let normal = if rng.uniform() < st.wrong_way { -p.normal } else { p.normal };
            extra.push(ScanPoint { pos: at, normal, true_pos: at, true_normal: p.true_normal, ray: p.ray, cos: p.cos, hits: 1, kind: PointKind::Stray });
        }
        stats.strays = extra.len();
        points.extend(extra);
    }

    // 7. Registration error of the whole pass.
    let offset = v3(pass.offset_mm);
    if offset != DVec3::ZERO {
        for p in &mut points {
            p.pos += offset;
        }
    }

    for p in &points {
        match p.kind {
            PointKind::Part => stats.part += 1,
            PointKind::Sticker(_) => stats.sticker += 1,
            PointKind::Table => stats.table += 1,
            PointKind::Stray => {}
        }
    }
    stats.points = points.len();
    PassScan { points, listed, stats }
}

fn classify(scene: &Scene, pass: usize, p: DVec3) -> PointKind {
    let mut best = (0.01, None);
    for (i, s) in scene.stickers.iter().enumerate() {
        if s.on_table_of_pass.is_some_and(|q| q != pass) {
            continue;
        }
        let t = s.thickness(p);
        if t > best.0 {
            best = (t, Some(i));
        }
    }
    if let Some(i) = best.1 {
        return PointKind::Sticker(i as u32);
    }
    match &scene.passes[pass].table {
        Some(t) if t.sdf(p).abs() < scene.part.sdf(p).abs() => PointKind::Table,
        _ => PointKind::Part,
    }
}

/// Plane fit over the neighbours on the same sheet (true normals within 60°, as a depth image's
/// neighbours would be), oriented towards the scanner.
fn estimate_normals(points: &mut [ScanPoint], pass: &PassSpec) {
    let radius_of = |p: &ScanPoint| 2.5 * spacing_at(pass, p.true_pos).1;
    let cell = points.iter().map(radius_of).fold(0.0, f64::max).max(1e-3);
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for (k, p) in points.iter().enumerate() {
        let c = (p.pos / cell).floor();
        grid.entry((c.x as i64, c.y as i64, c.z as i64)).or_default().push(k as u32);
    }
    let normals: Vec<DVec3> = points
        .par_iter()
        .map(|p| {
            let r = radius_of(p);
            let c = (p.pos / cell).floor();
            let (cx, cy, cz) = (c.x as i64, c.y as i64, c.z as i64);
            let mut nb: Vec<DVec3> = Vec::with_capacity(48);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(list) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                            for &j in list {
                                let q = &points[j as usize];
                                if (q.pos - p.pos).length_squared() <= r * r && q.true_normal.dot(p.true_normal) > 0.5 {
                                    nb.push(q.pos);
                                }
                            }
                        }
                    }
                }
            }
            if nb.len() < 5 {
                return p.true_normal;
            }
            let mean = nb.iter().copied().sum::<DVec3>() / nb.len() as f64;
            let mut m = [[0.0f64; 3]; 3];
            for q in &nb {
                let d = *q - mean;
                for i in 0..3 {
                    for j in 0..3 {
                        m[i][j] += d[i] * d[j];
                    }
                }
            }
            let n = smallest_eigenvector(m);
            if n.dot(-p.ray) < 0.0 {
                -n
            } else {
                n
            }
        })
        .collect();
    for (p, n) in points.iter_mut().zip(normals) {
        p.normal = n;
    }
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3 x 3 matrix (Jacobi rotations).
pub fn smallest_eigenvector(mut a: [[f64; 3]; 3]) -> DVec3 {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut big) = (0, 1, a[0][1].abs());
        if a[0][2].abs() > big {
            (p, q, big) = (0, 2, a[0][2].abs());
        }
        if a[1][2].abs() > big {
            (p, q, big) = (1, 2, a[1][2].abs());
        }
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if big <= 1e-15 * scale.max(1e-300) {
            break;
        }
        let theta = 0.5 * (a[q][q] - a[p][p]) / a[p][q];
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let t = if theta == 0.0 { 1.0 } else { t };
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        let mut b = a;
        for k in 0..3 {
            b[k][p] = c * a[k][p] - s * a[k][q];
            b[k][q] = s * a[k][p] + c * a[k][q];
        }
        let mut d = b;
        for k in 0..3 {
            d[p][k] = c * b[p][k] - s * b[q][k];
            d[q][k] = s * b[p][k] + c * b[q][k];
        }
        a = d;
        for row in &mut v {
            let (vp, vq) = (row[p], row[q]);
            row[p] = c * vp - s * vq;
            row[q] = s * vp + c * vq;
        }
    }
    let k = (0..3).min_by(|&i, &j| a[i][i].total_cmp(&a[j][j])).unwrap_or(0);
    DVec3::new(v[0][k], v[1][k], v[2][k]).normalize_or_zero()
}

fn list_markers(scene: &Scene, pass: usize, points: &[ScanPoint]) -> Vec<ListedMarker> {
    let mut dots = vec![0usize; scene.stickers.len()];
    for p in points {
        if let PointKind::Sticker(i) = p.kind {
            let s = &scene.stickers[i as usize];
            if s.local(p.true_pos).0 < s.dot_radius {
                dots[i as usize] += 1;
            }
        }
    }
    let offset = v3(scene.passes[pass].offset_mm);
    scene
        .stickers
        .iter()
        .enumerate()
        .filter(|(i, _)| dots[*i] >= 6)
        .map(|(i, s)| {
            let mut rng = Rng::for_parts(&[scene.seed, pass as u64, 0x4d4b, i as u64]);
            let n = v3(s.normal);
            // Revo's centres sit about 0.1-0.2 mm above the surface (inside the sticker), its
            // normals are within about 1.5° of the surface's.
            let centre = v3(s.center) + n * (0.4 * s.height) + DVec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.03 + offset;
            let normal = (n + DVec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.02).normalize();
            ListedMarker { sticker: i, centre, normal, radius_mm: if rng.uniform() < 0.5 { 3.0 } else { 3.05 }, dot_points: dots[i] }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eigenvector_of_a_flat_cloud_is_its_normal() {
        let mut m = [[0.0; 3]; 3];
        m[0][0] = 4.0;
        m[1][1] = 2.0;
        m[2][2] = 0.01;
        m[0][1] = 0.3;
        m[1][0] = 0.3;
        let n = smallest_eigenvector(m);
        assert!(n.z.abs() > 0.999, "{n}");
        // A tilted plane.
        let normal = DVec3::new(0.3, -0.2, 0.9).normalize();
        let (u, w) = (normal.any_orthonormal_pair().0, normal.any_orthonormal_pair().1);
        let mut m = [[0.0; 3]; 3];
        for (a, b) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.5), (0.7, -0.9), (0.2, 0.3)] {
            let d = u * a + w * b;
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] += d[i] * d[j];
                }
            }
        }
        let n = smallest_eigenvector(m);
        assert!(n.dot(normal).abs() > 0.9999, "{n} {normal}");
    }
}
