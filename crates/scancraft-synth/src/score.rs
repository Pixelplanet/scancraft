//! Checks a mesh against a scene's truth: distance of its vertices to the true surface and every
//! feature probe of `truth.json`. The mesh must be in the part frame (pass 1's frame, which is
//! the project frame when pass 1 is the reference).

use std::collections::HashMap;

use glam::DVec3;
use rayon::prelude::*;
use scancraft_core::Mesh;
use serde::{Deserialize, Serialize};

use crate::scene::ProbeKind;
use crate::sdf::v3;
use crate::write::Truth;

/// Triangles binned on a grid for nearest-distance and line queries.
pub struct MeshIndex {
    tris: Vec<[DVec3; 3]>,
    grid: HashMap<(i64, i64, i64), Vec<u32>>,
    cell: f64,
}

fn key(p: DVec3, cell: f64) -> (i64, i64, i64) {
    let c = (p / cell).floor();
    (c.x as i64, c.y as i64, c.z as i64)
}

/// Closest point of triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_triangle(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> DVec3 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

impl MeshIndex {
    pub fn new(mesh: &Mesh, cell: f64) -> Self {
        let v = |i: u32| DVec3::from_array(mesh.vertices[i as usize].position.map(f64::from));
        let tris: Vec<[DVec3; 3]> = mesh.triangles.iter().map(|t| t.indices.map(v)).collect();
        let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
        for (k, t) in tris.iter().enumerate() {
            let lo = key(t[0].min(t[1]).min(t[2]), cell);
            let hi = key(t[0].max(t[1]).max(t[2]), cell);
            for x in lo.0..=hi.0 {
                for y in lo.1..=hi.1 {
                    for z in lo.2..=hi.2 {
                        grid.entry((x, y, z)).or_default().push(k as u32);
                    }
                }
            }
        }
        MeshIndex { tris, grid, cell }
    }

    fn candidates(&self, lo: DVec3, hi: DVec3) -> Vec<u32> {
        let (a, b) = (key(lo, self.cell), key(hi, self.cell));
        let mut out = Vec::new();
        for x in a.0..=b.0 {
            for y in a.1..=b.1 {
                for z in a.2..=b.2 {
                    if let Some(l) = self.grid.get(&(x, y, z)) {
                        out.extend_from_slice(l);
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Distance from `p` to the nearest triangle, if one is within `max`.
    pub fn nearest(&self, p: DVec3, max: f64) -> Option<f64> {
        self.candidates(p - DVec3::splat(max), p + DVec3::splat(max))
            .into_iter()
            .map(|k| {
                let [a, b, c] = self.tris[k as usize];
                (closest_on_triangle(p, a, b, c) - p).length()
            })
            .filter(|d| *d <= max)
            .min_by(f64::total_cmp)
    }

    /// Signed positions t (|t| <= range) where the line p + t n crosses the mesh.
    pub fn crossings(&self, p: DVec3, n: DVec3, range: f64) -> Vec<f64> {
        let (e0, e1) = (p - n * range, p + n * range);
        let mut out: Vec<f64> = self
            .candidates(e0.min(e1), e0.max(e1))
            .into_iter()
            .filter_map(|k| {
                let [a, b, c] = self.tris[k as usize];
                let (ab, ac) = (b - a, c - a);
                let h = n.cross(ac);
                let det = ab.dot(h);
                if det.abs() < 1e-12 {
                    return None;
                }
                let s = p - a;
                let u = s.dot(h) / det;
                let q = s.cross(ab);
                let v = n.dot(q) / det;
                if !(-1e-9..=1.0 + 1e-9).contains(&u) || v < -1e-9 || u + v > 1.0 + 1e-9 {
                    return None;
                }
                let t = ac.dot(q) / det;
                (t.abs() <= range).then_some(t)
            })
            .collect();
        out.sort_by(f64::total_cmp);
        out
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceStats {
    /// Vertices within 3 mm of the true surface (the others are counted as extraneous).
    pub vertices: usize,
    pub extraneous: usize,
    pub mean_signed_mm: f64,
    pub rms_mm: f64,
    pub p50_mm: f64,
    pub p95_mm: f64,
    pub p99_mm: f64,
    pub max_mm: f64,
    /// Share of vertices more than 0.1 mm off.
    pub over_0_1mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbeResult {
    pub note: String,
    pub point: [f64; 3],
    pub check: ProbeKind,
    pub passed: bool,
    /// On: distance to the mesh; Clear: distance of the nearest mesh (None = clear);
    /// Height: the crossing nearest the expected one (None = no surface along the line).
    pub value: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureResult {
    pub id: String,
    pub kind: String,
    pub expect: String,
    pub passed: usize,
    pub probes: usize,
    pub failed: Vec<ProbeResult>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Report {
    pub scene: String,
    pub vertices: usize,
    pub triangles: usize,
    pub distance: DistanceStats,
    pub probes_passed: usize,
    pub probes: usize,
    pub features: Vec<FeatureResult>,
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((sorted.len() - 1) as f64 * q).round() as usize]
}

/// Scores `mesh` (part frame) against `truth`.
pub fn score(truth: &Truth, mesh: &Mesh) -> Report {
    let d: Vec<f64> = mesh.vertices.par_iter().map(|v| truth.part.sdf(DVec3::from_array(v.position.map(f64::from)))).collect();
    let near: Vec<f64> = d.iter().copied().filter(|x| x.abs() <= 3.0).collect();
    let mut abs: Vec<f64> = near.iter().map(|x| x.abs()).collect();
    abs.sort_by(f64::total_cmp);
    let n = near.len().max(1) as f64;
    let distance = DistanceStats {
        vertices: near.len(),
        extraneous: d.len() - near.len(),
        mean_signed_mm: near.iter().sum::<f64>() / n,
        rms_mm: (near.iter().map(|x| x * x).sum::<f64>() / n).sqrt(),
        p50_mm: percentile(&abs, 0.5),
        p95_mm: percentile(&abs, 0.95),
        p99_mm: percentile(&abs, 0.99),
        max_mm: abs.last().copied().unwrap_or(0.0),
        over_0_1mm: abs.iter().filter(|x| **x > 0.1).count() as f64 / n,
    };

    let index = MeshIndex::new(mesh, 1.0);
    let mut features = Vec::new();
    let (mut passed_all, mut total) = (0, 0);
    for f in &truth.features {
        let results: Vec<ProbeResult> = f
            .probes
            .par_iter()
            .map(|p| {
                let at = v3(p.point);
                let (passed, value) = match &p.check {
                    ProbeKind::On { tol } => {
                        let d = index.nearest(at, tol.max(1.0));
                        (d.is_some_and(|d| d <= *tol), d)
                    }
                    ProbeKind::Clear { radius } => {
                        let d = index.nearest(at, *radius);
                        (d.is_none(), d)
                    }
                    ProbeKind::Height { normal, expect, tol, range } => {
                        let c = index.crossings(at, v3(*normal).normalize(), *range);
                        let best = c.into_iter().min_by(|a, b| (a - expect).abs().total_cmp(&(b - expect).abs()));
                        (best.is_some_and(|t| (t - expect).abs() <= *tol), best)
                    }
                };
                ProbeResult { note: p.note.clone(), point: p.point, check: p.check.clone(), passed, value }
            })
            .collect();
        let passed = results.iter().filter(|r| r.passed).count();
        passed_all += passed;
        total += results.len();
        features.push(FeatureResult { id: f.id.clone(), kind: f.kind.clone(), expect: f.expect.clone(), passed, probes: results.len(), failed: results.into_iter().filter(|r| !r.passed).collect() });
    }
    Report { scene: truth.scene.clone(), vertices: mesh.vertices.len(), triangles: mesh.triangles.len(), distance, probes_passed: passed_all, probes: total, features }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closest_point_and_crossings_on_a_triangle() {
        let mut mesh = Mesh::new();
        for p in [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 4.0, 0.0]] {
            mesh.add_vertex(scancraft_core::Point3D::new(p));
        }
        mesh.add_triangle(0, 1, 2);
        let index = MeshIndex::new(&mesh, 1.0);
        assert!((index.nearest(DVec3::new(1.0, 1.0, 0.3), 1.0).unwrap() - 0.3).abs() < 1e-9);
        assert!((index.nearest(DVec3::new(-0.3, -0.4, 0.0), 1.0).unwrap() - 0.5).abs() < 1e-9);
        assert!(index.nearest(DVec3::new(1.0, 1.0, 2.0), 1.0).is_none());
        let c = index.crossings(DVec3::new(1.0, 1.0, 0.25), DVec3::Z, 2.0);
        assert_eq!(c.len(), 1);
        assert!((c[0] + 0.25).abs() < 1e-9);
    }
}
