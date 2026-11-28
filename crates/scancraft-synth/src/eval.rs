//! Measures a mesh against a scene's truth: distances both ways, topology, and what happened to
//! each feature of `truth.json` (marker sites, holes and slots, relief, edges, laser strays,
//! pass offsets), with a verdict and the reasons for a failure.
//!
//! All measurements are made against the exact true shape (`Truth::part`), never against the
//! reference mesh, except where the true surface has to be sampled (the reference mesh's
//! vertices lie on it).

use std::collections::HashMap;

use glam::{DVec3, Vec3};
use rayon::prelude::*;
use scancraft_core::Mesh;
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::scene::{Feature, ProbeKind};
use crate::score::{score, MeshIndex};
use crate::sdf::{project_to_surface, v3, V3};
use crate::write::Truth;

// ------------------------------------------------------------------------------------ helpers

fn frame(n: DVec3) -> (DVec3, DVec3) {
    let a = if n.x.abs() < 0.8 { DVec3::X } else { DVec3::Y };
    let e1 = n.cross(a).normalize();
    (e1, n.cross(e1))
}

fn median(mut v: Vec<f64>) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    v.sort_by(f64::total_cmp);
    Some(v[v.len() / 2])
}

fn mean(v: &[f64]) -> Option<f64> {
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

fn rms(v: &[f64]) -> Option<f64> {
    (!v.is_empty()).then(|| (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt())
}

fn pct(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((sorted.len() - 1) as f64 * q).round() as usize]
}

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

fn get_v3(v: &Value, key: &str) -> Option<DVec3> {
    let a = v.get(key)?.as_array()?;
    Some(DVec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2).and_then(|x| x.as_f64()).unwrap_or(0.0)))
}

fn get_v2(v: &Value, key: &str) -> Option<[f64; 2]> {
    let a = v.get(key)?.as_array()?;
    Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?])
}

fn get_f(v: &Value, key: &str) -> Option<f64> {
    v.get(key)?.as_f64()
}

/// Every root of the true distance along o + t d, t in [a, b] (the distance is exact, so steps of
/// |f| never jump over a surface).
pub fn truth_roots(truth: &Truth, o: DVec3, d: DVec3, a: f64, b: f64) -> Vec<f64> {
    let f = |t: f64| truth.part.sdf(o + d * t);
    let mut out = Vec::new();
    let (mut t, mut ft) = (a, f(a));
    while t < b {
        let step = (ft.abs() * 0.9).max(0.003);
        let t2 = (t + step).min(b);
        let f2 = f(t2);
        if (ft < 0.0) != (f2 < 0.0) {
            let (mut lo, mut hi, flo) = (t, t2, ft);
            for _ in 0..50 {
                let m = 0.5 * (lo + hi);
                if (f(m) < 0.0) == (flo < 0.0) {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            out.push(0.5 * (lo + hi));
        }
        if t2 >= b {
            break;
        }
        t = t2;
        ft = f2;
    }
    out
}

/// Signed position of the mesh along the true normal at a true surface point (nearest to 0).
fn dev_at(index: &MeshIndex, q: DVec3, n: DVec3, range: f64) -> Option<f64> {
    index.crossings(q, n, range).into_iter().min_by(|a, b| a.abs().total_cmp(&b.abs()))
}

/// Crossings of the mesh along the segment o + t d, t in [a, b].
fn mesh_line(index: &MeshIndex, o: DVec3, d: DVec3, a: f64, b: f64) -> Vec<f64> {
    let mid = 0.5 * (a + b);
    let half = 0.5 * (b - a);
    index.crossings(o + d * mid, d, half).into_iter().map(|t| t + mid).collect()
}

/// The two crossings either side of t = 0 (walls of a hole seen from its middle).
fn walls(c: &[f64]) -> (Option<f64>, Option<f64>) {
    let neg = c.iter().copied().filter(|t| *t < 0.0).max_by(f64::total_cmp);
    let pos = c.iter().copied().filter(|t| *t > 0.0).min_by(f64::total_cmp);
    (neg, pos)
}

/// The two face normals at an edge point of the true part (normals sampled on a sphere round it
/// and split into two groups), and the angle between them in degrees.
pub fn edge_faces(truth: &Truth, e: DVec3, radius: f64) -> Option<(DVec3, DVec3, f64)> {
    let f = |p: DVec3| truth.part.sdf(p);
    let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
    let mut normals = Vec::new();
    for k in 0..96 {
        let z = 1.0 - 2.0 * (k as f64 + 0.5) / 96.0;
        let r = (1.0 - z * z).sqrt();
        let a = golden * k as f64;
        let u = DVec3::new(r * a.cos(), r * a.sin(), z);
        let q = project_to_surface(f, e + u * radius);
        if f(q).abs() < 1e-5 && (q - e).length() > 0.6 * radius {
            normals.push(truth.part.normal(q));
        }
    }
    let first = *normals.first()?;
    let other = *normals.iter().min_by(|a, b| a.dot(first).total_cmp(&b.dot(first)))?;
    let (mut s1, mut s2) = (DVec3::ZERO, DVec3::ZERO);
    for n in &normals {
        if n.dot(first) >= n.dot(other) {
            s1 += *n;
        } else {
            s2 += *n;
        }
    }
    let (n1, n2) = (s1.try_normalize()?, s2.try_normalize()?);
    Some((n1, n2, n1.dot(n2).clamp(-1.0, 1.0).acos().to_degrees()))
}

/// The radius of a round that would leave the mesh `dist` mm off a sharp edge whose faces meet
/// at `angle_deg` (between the normals).
fn rounding_radius(dist: f64, angle_deg: f64) -> Option<f64> {
    let k = 1.0 / (0.5 * angle_deg.to_radians()).cos() - 1.0;
    (angle_deg > 20.0 && k > 1e-6).then(|| dist / k)
}

// ---------------------------------------------------------------------------- the context

/// Truth surface samples for the global and coverage measurements.
pub struct TruthSamples {
    /// (point on the true surface, outward normal).
    pub points: Vec<(DVec3, DVec3)>,
    /// Per sample: which passes (bit k = pass k+1) have a part or sticker point within 0.75 mm.
    pub covered: Vec<u32>,
    pub coverage: Coverage,
    /// The same points within 0.75 mm: a mesh vertex this close to a measured point is supported
    /// by the data; the rest was made up (fills, the closing walls of a solid).
    pub support: Coverage,
}

/// The passes' points (part frame) on a grid, to tell which passes saw a place.
pub struct Coverage {
    grids: Vec<HashMap<(i32, i32, i32), Vec<Vec3>>>,
    r: f32,
}

impl Coverage {
    pub fn new(pass_points: &[Vec<Vec3>], r: f32) -> Self {
        let grids = pass_points
            .iter()
            .map(|pts| {
                let mut g: HashMap<(i32, i32, i32), Vec<Vec3>> = HashMap::new();
                for p in pts {
                    let k = (*p / r).floor();
                    g.entry((k.x as i32, k.y as i32, k.z as i32)).or_default().push(*p);
                }
                g
            })
            .collect();
        Coverage { grids, r }
    }

    /// Bit k set: pass k+1 has a point within `r` of `p`.
    pub fn bits(&self, p: DVec3) -> u32 {
        let (q, r) = (p.as_vec3(), self.r);
        let k = (q / r).floor();
        let mut bits = 0u32;
        for (i, g) in self.grids.iter().enumerate() {
            'search: for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(l) = g.get(&(k.x as i32 + dx, k.y as i32 + dy, k.z as i32 + dz)) {
                            if l.iter().any(|s| s.distance_squared(q) < r * r) {
                                bits |= 1 << i;
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        bits
    }
}

/// Thinned true surface samples (reference mesh vertices, one per `cell`) and which passes cover
/// each. `pass_points` are the passes' points in the part frame (true poses).
pub fn truth_samples(truth: &Truth, reference: &Mesh, cell: f64, pass_points: &[Vec<Vec3>]) -> TruthSamples {
    let mut seen: HashMap<(i64, i64, i64), ()> = HashMap::new();
    let mut points = Vec::new();
    for v in &reference.vertices {
        let p = DVec3::from_array(v.position.map(f64::from));
        let k = (p / cell).floor();
        if seen.insert((k.x as i64, k.y as i64, k.z as i64), ()).is_none() {
            points.push(p);
        }
    }
    let points: Vec<(DVec3, DVec3)> = points.into_par_iter().map(|p| (p, truth.part.normal(p))).collect();
    let coverage = Coverage::new(pass_points, 0.75);
    let covered = points.par_iter().map(|(p, _)| coverage.bits(*p)).collect();
    TruthSamples { points, covered, coverage, support: Coverage::new(pass_points, 0.75) }
}

// ------------------------------------------------------------------------------ results

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Dist {
    pub samples: usize,
    pub mean_mm: f64,
    pub rms_mm: f64,
    pub p50_mm: f64,
    pub p95_mm: f64,
    pub p99_mm: f64,
    pub max_mm: f64,
}

fn dist(values: &[(f64, f64)]) -> Dist {
    // (value, weight)
    let w: f64 = values.iter().map(|v| v.1).sum::<f64>().max(1e-12);
    let mut abs: Vec<f64> = values.iter().map(|v| v.0.abs()).collect();
    abs.sort_by(f64::total_cmp);
    Dist {
        samples: values.len(),
        mean_mm: r3(values.iter().map(|v| v.0 * v.1).sum::<f64>() / w),
        rms_mm: r3((values.iter().map(|v| v.0 * v.0 * v.1).sum::<f64>() / w).sqrt()),
        p50_mm: r3(pct(&abs, 0.5)),
        p95_mm: r3(pct(&abs, 0.95)),
        p99_mm: r3(pct(&abs, 0.99)),
        max_mm: r3(abs.last().copied().unwrap_or(0.0)),
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Topology {
    pub vertices: usize,
    pub triangles: usize,
    pub watertight: bool,
    pub manifold: bool,
    pub components: usize,
    pub closed_components: usize,
    pub genus: i64,
    pub boundary_loops: usize,
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub volume_mm3: f64,
    pub area_mm2: f64,
}

pub fn topology(mesh: &Mesh) -> Topology {
    let pos: Vec<Vec3> = mesh.vertices.iter().map(|v| Vec3::from(v.position)).collect();
    let tris: Vec<[u32; 3]> = mesh.triangles.iter().map(|t| t.indices).collect();
    let t = scancraft_mesh::solid::topo::analyze(&pos, &tris);
    Topology {
        vertices: t.vertices,
        triangles: t.triangles,
        watertight: t.watertight,
        manifold: t.manifold,
        components: t.components.len(),
        closed_components: t.components.iter().filter(|c| c.closed).count(),
        genus: t.genus_total(),
        boundary_loops: t.boundary_loops,
        boundary_edges: t.boundary_edges,
        non_manifold_edges: t.non_manifold_edges,
        volume_mm3: t.total_volume_mm3.round(),
        area_mm2: t.total_area_mm2.round(),
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Global {
    /// Output -> truth: signed distance of the mesh's vertices (area weighted) within 0.75 mm of a
    /// measured point (part or sticker), within 3 mm of the truth.
    pub out_to_truth: Dist,
    /// The same for the rest: surface made up where nothing was measured (marker-site fills,
    /// closed holes, the bottom of a plate seen from above, a solid's wall behind a one-sided
    /// scan). Not an error by itself: shown so that the closing is visible.
    pub out_to_truth_unseen: Dist,
    pub unseen_area_mm2: f64,
    /// Area of the mesh more than 3 mm from the true part (tables, blocks, debris), mm².
    pub extraneous_area_mm2: f64,
    /// Area of the mesh 0.3–2 mm inside the material near measured points (stray layers, merged
    /// walls), mm².
    pub inside_area_mm2: f64,
    /// Truth -> output over the true surface the passes saw: distance to the nearest mesh.
    pub truth_to_out: Dist,
    /// Share of the seen true surface with no mesh within 1 mm.
    pub missing_share: f64,
    /// Share of the seen true surface whose nearest mesh is more than 0.1 mm away.
    pub over_0_1mm_share: f64,
}

pub fn global(truth: &Truth, mesh: &Mesh, index: &MeshIndex, samples: &TruthSamples) -> Global {
    let mut area = vec![0.0f64; mesh.vertices.len()];
    for t in &mesh.triangles {
        let [a, b, c] = t.indices.map(|i| DVec3::from_array(mesh.vertices[i as usize].position.map(f64::from)));
        let ar = (b - a).cross(c - a).length() / 6.0;
        for i in t.indices {
            area[i as usize] += ar;
        }
    }
    // (signed distance, seen)
    let d: Vec<(f64, bool)> = mesh
        .vertices
        .par_iter()
        .map(|v| {
            let p = DVec3::from_array(v.position.map(f64::from));
            let d = truth.part.sdf(p);
            let seen = d.abs() <= 3.0 && samples.support.bits(p) != 0;
            (d, seen)
        })
        .collect();
    let pick = |seen: bool| -> Vec<(f64, f64)> { d.iter().zip(&area).filter(|((x, s), _)| x.abs() <= 3.0 && *s == seen).map(|((x, _), a)| (*x, *a)).collect() };
    let extraneous: f64 = d.iter().zip(&area).filter(|((x, _), _)| x.abs() > 3.0).map(|(_, a)| a).sum();
    let inside: f64 = d.iter().zip(&area).filter(|((x, s), _)| *s && *x < -0.3 && *x > -2.0).map(|(_, a)| a).sum();
    let unseen = pick(false);
    let seen: Vec<DVec3> = samples.points.iter().zip(&samples.covered).filter(|(_, c)| **c != 0).map(|(p, _)| p.0).collect();
    let back: Vec<Option<f64>> = seen.par_iter().map(|p| index.nearest(*p, 1.0).or_else(|| index.nearest(*p, 5.0))).collect();
    let present: Vec<(f64, f64)> = back.iter().map(|b| (b.unwrap_or(5.0), 1.0)).collect();
    let n = seen.len().max(1) as f64;
    Global {
        out_to_truth: dist(&pick(true)),
        unseen_area_mm2: unseen.iter().map(|x| x.1).sum::<f64>().round(),
        out_to_truth_unseen: dist(&unseen),
        extraneous_area_mm2: extraneous.round(),
        inside_area_mm2: (inside * 10.0).round() / 10.0,
        truth_to_out: dist(&present),
        missing_share: r3(back.iter().filter(|b| b.is_none_or(|x| x > 1.0)).count() as f64 / n),
        over_0_1mm_share: r3(back.iter().filter(|b| b.is_none_or(|x| x > 0.1)).count() as f64 / n),
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FeatureEval {
    pub id: String,
    pub kind: String,
    pub expect: String,
    pub probes_passed: usize,
    pub probes: usize,
    /// Failed probes: "note: value".
    pub failed_probes: Vec<String>,
    pub metrics: serde_json::Map<String, Value>,
    /// Why the feature failed (empty = passed).
    pub fails: Vec<String>,
    /// Where to look at it: centre, view direction (towards the viewer) and crop radius.
    pub view: View,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct View {
    pub center: V3,
    pub normal: V3,
    pub radius: f64,
}

impl FeatureEval {
    fn put(&mut self, key: &str, v: impl Into<Value>) {
        self.metrics.insert(key.into(), v.into());
    }
    fn put_opt(&mut self, key: &str, v: Option<f64>) {
        self.metrics.insert(key.into(), v.map(r3).map_or(Value::Null, Value::from));
    }
    fn fail(&mut self, why: String) {
        self.fails.push(why);
    }
}

/// What the pipeline knew about stray points (part frame), for the laser-stray check.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StrayCounts {
    /// Stray points (label 3) in the passes after isolation.
    pub input: usize,
    /// Points 0.3–1.5 mm inside the material after Merge.
    pub after_merge: usize,
    /// ... after Reconstruct's preparation.
    pub after_prepare: usize,
    /// Make Solid's own count of dropped laser strays.
    pub solid_dropped: usize,
}

/// Everything needed besides the mesh.
pub struct Context<'a> {
    pub truth: &'a Truth,
    pub samples: &'a TruthSamples,
    pub strays: Option<&'a StrayCounts>,
    /// The mesh is Make Solid's (closed; the bottom of a plate seen from one side is filled).
    pub solid: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MeshEval {
    pub mesh: String,
    pub global: Global,
    pub topology: Topology,
    pub probes_passed: usize,
    pub probes: usize,
    pub features: Vec<FeatureEval>,
    pub pass_offset: Option<Value>,
}

impl MeshEval {
    pub fn features_failed(&self) -> usize {
        self.features.iter().filter(|f| !f.fails.is_empty()).count()
    }
}

/// Evaluates `mesh` (part frame) against the scene.
pub fn evaluate(name: &str, mesh: &Mesh, cx: &Context) -> MeshEval {
    let index = MeshIndex::new(mesh, 1.0);
    let report = score(cx.truth, mesh);
    let global = global(cx.truth, mesh, &index, cx.samples);
    let topology = topology(mesh);
    let features = cx.truth.features.iter().zip(&report.features).map(|(f, r)| feature(f, r, &index, cx)).collect();
    let pass_offset = (cx.truth.passes.len() > 1).then(|| pass_offset(&index, cx));
    MeshEval { mesh: name.into(), global, topology, probes_passed: report.probes_passed, probes: report.probes, features, pass_offset }
}

fn feature(f: &Feature, r: &crate::score::FeatureResult, index: &MeshIndex, cx: &Context) -> FeatureEval {
    let mut e = FeatureEval {
        id: f.id.clone(),
        kind: f.kind.clone(),
        expect: f.expect.clone(),
        probes_passed: r.passed,
        probes: r.probes,
        failed_probes: r.failed.iter().map(|p| format!("{}: {}", p.note, p.value.map_or("none".into(), |v| format!("{:.3}", v)))).collect(),
        ..Default::default()
    };
    e.view = default_view(f, cx.truth);
    match f.kind.as_str() {
        "marker_site" | "marker_hole" => marker(&mut e, f, index, cx),
        "through_slot" => slot(&mut e, f, index, cx),
        "through_holes" | "through_hole" => holes(&mut e, f, index, cx),
        "blind_groove" | "blind_pocket" => blind(&mut e, f, index, cx),
        "screw_boss" => boss(&mut e, f, index, cx),
        "real_relief" | "curvature_band" => relief(&mut e, f, index, cx),
        "laser_strays" => strays(&mut e, f, index, cx),
        _ => {}
    }
    if f.kind == "thin_wall" && f.params.get("stray_window").is_some() {
        strays(&mut e, f, index, cx);
    }
    if f.kind == "flat" {
        flatness(&mut e, index, cx);
    }
    edges(&mut e, f, index, cx);
    if r.passed < r.probes {
        let why = format!("{} of {} probes failed", r.probes - r.passed, r.probes);
        e.fails.insert(0, why);
    }
    e
}

fn default_view(f: &Feature, truth: &Truth) -> View {
    let p = &f.params;
    let center = get_v3(p, "center").filter(|_| p.get("center").and_then(|v| v.as_array()).is_some_and(|a| a.len() == 3));
    let normal = get_v3(p, "normal");
    let probe_mean = || {
        let n = f.probes.len().max(1) as f64;
        f.probes.iter().map(|q| v3(q.point)).sum::<DVec3>() / n
    };
    let c = center.unwrap_or_else(|| {
        let m = probe_mean();
        // 2D centre (x, y) of a plate feature: on the top surface.
        match get_v2(p, "center") {
            Some(c2) => DVec3::new(c2[0], c2[1], m.z),
            None => m,
        }
    });
    let spread = f.probes.iter().map(|q| (v3(q.point) - c).length()).fold(0.0, f64::max);
    let n = normal.unwrap_or_else(|| {
        let q = project_to_surface(|x| truth.part.sdf(x), c);
        let n = truth.part.normal(q);
        if n == DVec3::ZERO { DVec3::Z } else { n }
    });
    let radius = match f.kind.as_str() {
        "marker_site" | "marker_hole" => 10.0,
        _ => (spread + 4.0).clamp(6.0, 45.0),
    };
    View { center: c.to_array(), normal: n.normalize().to_array(), radius }
}

// ---------------------------------------------------------------------------- marker sites

fn marker(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let truth = cx.truth;
    let (Some(c), Some(n)) = (get_v3(&f.params, "center"), get_v3(&f.params, "normal")) else { return };
    let n = n.normalize();
    let cut = get_f(&f.params, "cut_radius_mm");
    let sticker = truth.stickers.iter().min_by(|a, b| (v3(a.sticker.center) - c).length().total_cmp(&(v3(b.sticker.center) - c).length())).map(|s| &s.sticker);
    let outer = sticker.map_or(5.6, |s| s.outer_radius);
    let flat_r = sticker.map_or(4.3, |s| s.flat_radius);
    let site = cut.unwrap_or(0.0).max(outer);
    let r_max = (site + 2.5).max(9.0);
    let (e1, e2) = frame(n);
    let f_sdf = |p: DVec3| truth.part.sdf(p);
    // (radius, true sag along n, deviation or None)
    let samples: Vec<(f64, f64, Option<f64>)> = (0..=((r_max / 0.25) as usize))
        .into_par_iter()
        .flat_map_iter(|i| {
            let r = i as f64 * 0.25;
            let count = if i == 0 { 1 } else { 32 };
            (0..count)
                .filter_map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / count as f64;
                    let q0 = c + (e1 * a.cos() + e2 * a.sin()) * r;
                    let q = project_to_surface(f_sdf, q0);
                    let qn = truth.part.normal(q);
                    // Not within 1 mm of an edge of the part (the edge's own rounding is
                    // measured with the edges).
                    let clear = f_sdf(q - qn * 1.0) < -0.97;
                    ((q - q0).length() < 1.5 && f_sdf(q).abs() < 1e-4 && qn.dot(n) > 0.3 && clear).then(|| (r, (q - c).dot(n), dev_at(index, q, qn, 1.5)))
                })
                .collect::<Vec<_>>()
        })
        .collect();
    let within = |lo: f64, hi: f64| samples.iter().filter(move |s| s.0 >= lo && s.0 <= hi);
    let centre: Vec<f64> = within(0.0, 1.0).filter_map(|s| s.2).collect();
    let site_d: Vec<f64> = within(0.0, outer).filter_map(|s| s.2).collect();
    let all_site = within(0.0, site + 0.5).count().max(1);
    let open = within(0.0, site + 0.5).filter(|s| s.2.is_none()).count();
    // Profile: mean deviation per 0.5 mm annulus.
    let bins = (r_max / 0.5).ceil() as usize;
    let mut profile = vec![Vec::new(); bins];
    for s in &samples {
        if let Some(d) = s.2 {
            profile[((s.0 / 0.5) as usize).min(bins - 1)].push(d);
        }
    }
    let prof: Vec<Option<f64>> = profile.iter().map(|v| mean(v)).collect();
    let outside: Vec<f64> = within(site + 1.0, r_max).filter_map(|s| s.2).collect();
    let base = mean(&outside);
    let ring = base.and_then(|b| prof.iter().enumerate().filter(|(i, _)| *i as f64 * 0.5 >= 3.0).filter_map(|(_, m)| m.map(|m| (m - b).abs())).reduce(f64::max));
    let step = prof.windows(2).enumerate().filter(|(i, _)| *i as f64 * 0.5 >= 2.0).filter_map(|(_, w)| Some((w[1]? - w[0]?).abs())).reduce(f64::max);
    // Curvature followed: regress d on the true sag over the site.
    let fit: Vec<(f64, f64)> = within(0.0, outer).filter_map(|s| Some((s.1, s.2?))).collect();
    let sag = within(outer - 0.6, outer).map(|s| s.1.abs()).fold(0.0, f64::max);
    let follow = (sag > 0.03 && fit.len() > 20).then(|| {
        let ms = fit.iter().map(|x| x.0).sum::<f64>() / fit.len() as f64;
        let md = fit.iter().map(|x| x.1).sum::<f64>() / fit.len() as f64;
        let cov: f64 = fit.iter().map(|x| (x.0 - ms) * (x.1 - md)).sum();
        let var: f64 = fit.iter().map(|x| (x.0 - ms) * (x.0 - ms)).sum();
        1.0 + cov / var.max(1e-12)
    });
    let centre_dev = mean(&centre);
    let site_rms = rms(&site_d);
    let site_max = site_d.iter().map(|d| d.abs()).fold(None, |m: Option<f64>, x| Some(m.map_or(x, |m| m.max(x))));
    let sticker_ring: Vec<f64> = within(flat_r, outer).filter_map(|s| s.2).collect();
    e.put_opt("centre_dev_mm", centre_dev);
    e.put_opt("site_mean_mm", mean(&site_d));
    e.put_opt("site_rms_mm", site_rms);
    e.put_opt("site_max_mm", site_max);
    e.put_opt("sticker_ring_mean_mm", mean(&sticker_ring));
    e.put_opt("surround_mean_mm", base);
    e.put_opt("ring_mm", ring);
    e.put_opt("ring_step_mm", step);
    e.put_opt("true_sag_mm", Some(sag));
    e.put_opt("curvature_followed", follow);
    e.put("open_share", r3(open as f64 / all_site as f64));
    e.put_opt("cut_radius_mm", cut);
    e.put("profile_mm", prof.iter().map(|m| m.map(r3)).collect::<Vec<_>>());
    if open as f64 > 0.02 * all_site as f64 {
        e.fail(format!("site left open ({:.0} % of it)", 100.0 * open as f64 / all_site as f64));
    }
    if let Some(d) = centre_dev.filter(|d| d.abs() > 0.05) {
        e.fail(format!("centre {:+.3} mm off", d));
    }
    if let Some(d) = site_rms.filter(|d| *d > 0.05) {
        e.fail(format!("site rms {:.3} mm", d));
    }
    if let Some(d) = ring.filter(|d| *d > 0.05) {
        e.fail(format!("visible ring {:.3} mm", d));
    }
    if let Some(k) = follow.filter(|k| !(0.6..=1.4).contains(k)) {
        e.fail(format!("curvature followed {:.2} (1 = as the true surface, 0 = flat)", k));
    }
    e.view = View { center: c.to_array(), normal: n.to_array(), radius: (site + 3.0).max(10.0) };
}

// ------------------------------------------------------------------------- holes and slots

/// Top and bottom of the plate along the vertical through `beside` (first two true roots from
/// above).
fn plate_span(truth: &Truth, beside: DVec3, z_hint: f64) -> Option<(f64, f64)> {
    let o = DVec3::new(beside.x, beside.y, z_hint + 30.0);
    let roots = truth_roots(truth, o, -DVec3::Z, 0.0, 80.0);
    Some((o.z - roots.first()?, o.z - roots.get(1)?))
}

struct Station {
    open: bool,
    width_mid: Option<f64>,
    width_top: Option<f64>,
}

/// A through opening at (x, y) with walls across direction `across` (unit, horizontal) between
/// `top` and `bottom`.
fn station(index: &MeshIndex, x: f64, y: f64, across: DVec3, half: f64, top: f64, bottom: f64) -> Station {
    let o = DVec3::new(x, y, 0.0);
    let vert = mesh_line(index, o, DVec3::Z, bottom - 0.5, top + 0.5);
    let w = |z: f64| {
        let c = mesh_line(index, DVec3::new(x, y, z), across, -(half + 2.5), half + 2.5);
        match walls(&c) {
            (Some(a), Some(b)) => Some(b - a),
            _ => None,
        }
    };
    Station { open: vert.is_empty(), width_mid: w(0.5 * (top + bottom)), width_top: w(top - 0.3) }
}

fn summarise_stations(e: &mut FeatureEval, st: &[Station], true_width: f64, label: &str) {
    let open = st.iter().filter(|s| s.open).count();
    let mid: Vec<f64> = st.iter().filter_map(|s| s.width_mid).collect();
    let top: Vec<f64> = st.iter().filter_map(|s| s.width_top).collect();
    let wm = median(mid);
    e.put("stations", st.len());
    e.put("open", open);
    e.put(&format!("true_{label}_mm"), true_width);
    e.put_opt(&format!("{label}_mid_mm"), wm);
    e.put_opt(&format!("{label}_top_mm"), median(top));
    if open < st.len() {
        e.fail(format!("closed at {} of {} places", st.len() - open, st.len()));
    }
    match wm {
        Some(w) if (w - true_width).abs() > 0.2 => e.fail(format!("{label} {w:.2} mm (true {true_width:.2})")),
        None => e.fail(format!("{label} not measurable (no walls)")),
        _ => {}
    }
}

fn slot(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let p = &f.params;
    let (Some(c), Some(ax), Some(len), Some(w)) = (get_v2(p, "center"), get_v2(p, "axis"), get_f(p, "length_mm"), get_f(p, "width_mm")) else { return };
    let axis = DVec3::new(ax[0], ax[1], 0.0).normalize();
    let across = DVec3::Z.cross(axis);
    let centre = DVec3::new(c[0], c[1], 0.0);
    let Some((top, bottom)) = plate_span(cx.truth, centre + across * (w / 2.0 + 1.2), 0.0) else { return };
    let reach = (len / 2.0 - w / 2.0 - 0.5).max(0.0) * 0.9;
    let st: Vec<Station> = (0..5)
        .map(|k| {
            let s = -reach + 2.0 * reach * k as f64 / 4.0;
            let q = centre + axis * s;
            station(index, q.x, q.y, across, w / 2.0, top, bottom)
        })
        .collect();
    summarise_stations(e, &st, w, "width");
}

fn holes(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let p = &f.params;
    let mut centres = Vec::new();
    let mut radius = 0.0;
    if let Some(prof) = p.get("profile") {
        if let (Some(count), Some(first), Some(pitch), Some(r)) = (get_v2(prof, "count"), get_v2(prof, "first"), get_v2(prof, "pitch"), get_f(prof, "radius")) {
            radius = r;
            for i in 0..count[0] as usize {
                for j in 0..count[1] as usize {
                    centres.push([first[0] + i as f64 * pitch[0], first[1] + j as f64 * pitch[1]]);
                }
            }
        }
    } else if let (Some(c), Some(d)) = (get_v2(p, "center"), get_f(p, "diameter_mm")) {
        centres.push(c);
        radius = d / 2.0;
    }
    if centres.is_empty() {
        return;
    }
    // Keep the centres where the truth is open (a grid can be clipped by the plate).
    let probe_z = f.probes.iter().map(|q| q.point[2]).fold(f64::MIN, f64::max);
    let mut st = Vec::new();
    for c in centres {
        let q = DVec3::new(c[0], c[1], 0.0);
        let Some((top, bottom)) = plate_span(cx.truth, q + DVec3::X * (radius + 1.0), probe_z.max(0.0)) else { continue };
        let open_truth = truth_roots(cx.truth, DVec3::new(c[0], c[1], top + 0.5), -DVec3::Z, 0.0, top - bottom + 1.0).is_empty();
        if !open_truth {
            continue;
        }
        let sx = station(index, c[0], c[1], DVec3::X, radius, top, bottom);
        let sy = station(index, c[0], c[1], DVec3::Y, radius, top, bottom);
        let avg = |a: Option<f64>, b: Option<f64>| match (a, b) {
            (Some(a), Some(b)) => Some(0.5 * (a + b)),
            (a, b) => a.or(b),
        };
        st.push(Station { open: sx.open, width_mid: avg(sx.width_mid, sy.width_mid), width_top: avg(sx.width_top, sy.width_top) });
    }
    summarise_stations(e, &st, 2.0 * radius, "diameter");
}

fn blind(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let p = &f.params;
    let depth = get_f(p, "depth_mm").unwrap_or(1.0);
    let mut points = Vec::new();
    let (mut across, mut width) = (DVec3::Y, 0.0);
    if let (Some(a), Some(b), Some(w)) = (get_v2(p, "from"), get_v2(p, "to"), get_f(p, "width_mm")) {
        let (a, b) = (DVec3::new(a[0], a[1], 0.0), DVec3::new(b[0], b[1], 0.0));
        across = DVec3::Z.cross((b - a).normalize());
        width = w;
        for k in 0..5 {
            points.push(a.lerp(b, 0.1 + 0.8 * k as f64 / 4.0));
        }
    } else if let Some(prof) = p.get("profile") {
        if let (Some(c), Some(h)) = (get_v2(prof, "center"), get_v2(prof, "half")) {
            across = DVec3::X;
            width = 2.0 * h[0];
            let c = DVec3::new(c[0], c[1], 0.0);
            for (dx, dy) in [(0.0, 0.0), (-0.5, -0.4), (0.5, 0.4), (-0.5, 0.4), (0.5, -0.4)] {
                points.push(c + DVec3::new(dx * h[0], dy * h[1], 0.0));
            }
        }
    }
    let mut kept = Vec::new();
    let mut widths = Vec::new();
    for q in points {
        let o = DVec3::new(q.x, q.y, 10.0);
        let truth_floor = truth_roots(cx.truth, o, -DVec3::Z, 0.0, 30.0).first().map(|t| o.z - t);
        let mesh_floor = mesh_line(index, o, -DVec3::Z, 0.0, 30.0).into_iter().min_by(f64::total_cmp).map(|t| o.z - t);
        let (Some(tf), Some(mf)) = (truth_floor, mesh_floor) else { continue };
        let top = tf + depth;
        kept.push((top - mf) / depth);
        let c = mesh_line(index, DVec3::new(q.x, q.y, top - depth / 2.0), across, -(width / 2.0 + 2.5), width / 2.0 + 2.5);
        if let (Some(a), Some(b)) = walls(&c) {
            widths.push(b - a);
        }
    }
    let k = median(kept);
    let w = median(widths);
    e.put("true_depth_mm", depth);
    e.put_opt("depth_kept", k);
    e.put("true_width_mm", width);
    e.put_opt("width_mid_mm", w);
    match k {
        Some(k) if k < 0.8 => e.fail(format!("only {:.0} % of the depth kept", 100.0 * k)),
        None => e.fail("floor not found".into()),
        _ => {}
    }
    if let Some(w) = w.filter(|w| (w - width).abs() > 0.2) {
        e.fail(format!("width {w:.2} mm (true {width:.2})"));
    }
}

fn boss(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let p = &f.params;
    let (Some(c), Some(pd)) = (get_v2(p, "center"), get_f(p, "pilot_diameter_mm")) else { return };
    // Along the pilot's axis: the true surfaces and the mesh's.
    let o = DVec3::new(c[0], c[1], 60.0);
    let tr = truth_roots(cx.truth, o, -DVec3::Z, 0.0, 120.0);
    let me = mesh_line(index, o, -DVec3::Z, 0.0, 120.0);
    let matched = |t: f64| me.iter().map(|m| (m - t).abs()).fold(f64::MAX, f64::min);
    let errs: Vec<f64> = tr.iter().map(|t| matched(*t)).collect();
    let extra = me.iter().filter(|m| tr.iter().all(|t| (*m - t).abs() > 0.5)).count();
    e.put("true_surfaces_on_axis", tr.len());
    e.put("mesh_surfaces_on_axis", me.len());
    e.put("extra_surfaces_on_axis", extra);
    e.put_opt("axis_worst_mm", errs.iter().copied().reduce(f64::max));
    // The pilot's diameter halfway down its first open stretch.
    let pilot_top = o.z - tr.first().copied().unwrap_or(60.0);
    let z = pilot_top - 1.5;
    let cx_ = mesh_line(index, DVec3::new(c[0], c[1], z), DVec3::X, -(pd / 2.0 + 2.0), pd / 2.0 + 2.0);
    let dm = match walls(&cx_) {
        (Some(a), Some(b)) => Some(b - a),
        _ => None,
    };
    e.put("true_pilot_mm", pd);
    e.put_opt("pilot_mm", dm);
    if extra > 0 {
        e.fail(format!("{extra} extra surface(s) across the pilot (capped or bridged)"));
    }
    if let Some(w) = errs.iter().copied().reduce(f64::max).filter(|w| *w > 0.3) {
        e.fail(format!("a surface on the pilot axis is {w:.2} mm off"));
    }
    if let Some(d) = dm.filter(|d| (d - pd).abs() > 0.3) {
        e.fail(format!("pilot {d:.2} mm (true {pd:.2})"));
    }
}

// ---------------------------------------------------------------------------------- relief

fn relief(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let p = &f.params;
    let h = match (get_f(p, "height_mm"), get_f(p, "depth_mm")) {
        (Some(h), _) => h,
        (None, Some(d)) => -d,
        _ => return,
    };
    let truth = cx.truth;
    let fs = |q: DVec3| truth.part.sdf(q);
    let mut kept = Vec::new();
    for probe in &f.probes {
        let note = probe.note.to_lowercase();
        let is_top = ["top", "peak", "floor", "crest", "ridge"].iter().any(|w| note.contains(w)) && !note.contains("beside");
        let ProbeKind::Height { normal, .. } = &probe.check else { continue };
        if !is_top {
            continue;
        }
        let at = v3(probe.point);
        let n = v3(*normal).normalize();
        let Some(d_top) = dev_at(index, at, n, 1.5) else {
            kept.push(0.0);
            continue;
        };
        // The base round it: true surface points about h below the top (above, for a groove).
        let (e1, e2) = frame(n);
        let mut base = Vec::new();
        for ri in 0..8 {
            let r = 0.8 + 0.3 * ri as f64;
            for k in 0..16 {
                let a = std::f64::consts::TAU * k as f64 / 16.0;
                let q = project_to_surface(fs, at - n * h + (e1 * a.cos() + e2 * a.sin()) * r);
                let level = (q - at).dot(n) + h;
                if fs(q).abs() < 1e-4 && level.abs() < 0.03 && truth.part.normal(q).dot(n) > 0.95 {
                    if let Some(d) = dev_at(index, q, truth.part.normal(q), 1.5) {
                        base.push(d);
                    }
                }
            }
        }
        let d_base = median(base).unwrap_or(0.0);
        kept.push((h + d_top - d_base) / h);
    }
    let k = median(kept.clone());
    let worst = kept.iter().copied().reduce(f64::min);
    e.put("true_height_mm", h);
    e.put_opt("height_kept", k);
    e.put_opt("height_kept_worst", worst);
    let limit = if h < 0.0 { 0.7 } else { 0.8 };
    if let Some(k) = k.filter(|k| *k < limit) {
        e.fail(format!("only {:.0} % of the {} kept", 100.0 * k, if h < 0.0 { "depth" } else { "height" }));
    }
}

// ---------------------------------------------------------------------------------- edges

/// Deviation profile across an edge on both faces, relative to each face's level 2.5-3 mm from
/// the edge: the highest bulge (mm, outwards) from 0.3 mm on, and how far from the edge the mesh
/// is more than 0.03 mm off that level.
fn edge_profile(truth: &Truth, index: &MeshIndex, e: DVec3, n1: DVec3, n2: DVec3) -> (Option<f64>, Option<f64>) {
    let fs = |q: DVec3| truth.part.sdf(q);
    let (mut bead, mut band): (Option<f64>, Option<f64>) = (None, None);
    for (na, nb) in [(n1, n2), (n2, n1)] {
        let Some(t) = (nb - na * nb.dot(na)).try_normalize() else { continue };
        // Along face a away from the edge: whichever direction stays on that face.
        let dir = [-t, t].into_iter().find(|d| {
            let q = project_to_surface(fs, e + *d * 1.5);
            (q - (e + *d * 1.5)).length() < 0.05 && truth.part.normal(q).dot(na) > 0.98
        });
        let Some(dir) = dir else { continue };
        // Deviations along the face, then taken relative to the face's level 2.5-3 mm away (so
        // a pass that sits too deep does not count as a bead).
        let mut prof = Vec::new();
        for k in 1..=30 {
            let s = 0.1 * k as f64;
            let q = project_to_surface(fs, e + dir * s);
            if truth.part.normal(q).dot(na) < 0.98 {
                break;
            }
            if let Some(d) = dev_at(index, q, na, 1.0) {
                prof.push((s, d));
            }
        }
        let far: Vec<f64> = prof.iter().filter(|p| p.0 >= 2.45).map(|p| p.1).collect();
        let Some(level) = mean(&far) else { continue };
        for (s, d) in prof {
            let d = d - level;
            if s >= 0.3 {
                bead = Some(bead.map_or(d, |b| b.max(d)));
            }
            if d.abs() > 0.03 {
                band = Some(band.map_or(s, |b| b.max(s)));
            }
        }
    }
    (bead, band)
}

fn edges(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let mut rows = Vec::new();
    let (mut beads, mut bands) = (Vec::new(), Vec::new());
    for probe in &f.probes {
        let note = probe.note.to_lowercase();
        let at = v3(probe.point);
        match &probe.check {
            ProbeKind::On { tol } if ["edge", "corner", "rim"].iter().any(|w| note.contains(w)) => {
                let d = index.nearest(at, 2.0);
                let faces = edge_faces(cx.truth, at, 1.2);
                let r = faces.and_then(|(_, _, ang)| d.and_then(|d| rounding_radius(d, ang)));
                if let Some((n1, n2, ang)) = faces.filter(|x| x.2 > 30.0) {
                    let _ = ang;
                    let (b, w) = edge_profile(cx.truth, index, at, n1, n2);
                    beads.extend(b);
                    bands.push(w.unwrap_or(0.0));
                }
                rows.push((d, r, faces.map(|x| x.2), *tol, 0.0));
            }
            ProbeKind::Height { normal, .. } if f.kind == "fillet" && note.contains("middle") => {
                let rad = get_f(&f.params, "radius_mm").unwrap_or(0.0);
                let d = dev_at(index, at, v3(*normal).normalize(), 1.5);
                // On a 90° round of radius R the middle is R (sec 45° - 1) from the virtual corner.
                let r = d.map(|d| rad - d / (2f64.sqrt() - 1.0));
                rows.push((d.map(f64::abs), r, Some(90.0), 0.05, rad));
            }
            _ => {}
        }
    }
    if rows.is_empty() {
        return;
    }
    let dists: Vec<f64> = rows.iter().filter_map(|r| r.0).collect();
    let radii: Vec<f64> = rows.iter().filter_map(|r| r.1).collect();
    let missing = rows.iter().filter(|r| r.0.is_none()).count();
    e.put("edge_points", rows.len());
    e.put_opt("edge_dist_median_mm", median(dists.clone()));
    e.put_opt("edge_dist_max_mm", dists.iter().copied().reduce(f64::max));
    e.put_opt("rounding_radius_median_mm", median(radii.clone()));
    e.put_opt("rounding_radius_max_mm", radii.iter().copied().reduce(f64::max));
    e.put_opt("edge_angle_deg", median(rows.iter().filter_map(|r| r.2).collect()));
    if missing > 0 {
        e.put("edge_points_without_mesh", missing);
    }
    let bead = beads.iter().copied().reduce(f64::max);
    e.put_opt("edge_bead_max_mm", bead);
    e.put_opt("edge_band_median_mm", median(bands));
    if let Some(b) = bead.filter(|b| *b > 0.05) {
        e.fail(format!("bead {b:.3} mm proud beside an edge"));
    }
    let fillet = rows.iter().find(|r| r.4 > 0.0).map(|r| r.4);
    if let (Some(rad), Some(r)) = (fillet, median(radii)) {
        e.put("true_radius_mm", rad);
        if (r - rad).abs() > 0.3 {
            e.fail(format!("round of radius {r:.2} mm (true {rad:.2})"));
        }
    }
}

// --------------------------------------------------------------------------- laser strays

fn strays(e: &mut FeatureEval, f: &Feature, index: &MeshIndex, cx: &Context) {
    let p = &f.params;
    let (c, r) = match p.get("stray_window") {
        Some(w) => (get_v3(w, "center"), get_f(w, "radius_mm")),
        None => (get_v3(p, "window_center"), get_f(p, "window_radius_mm")),
    };
    let (Some(c), Some(r)) = (c, r) else { return };
    let inside: Vec<&(DVec3, DVec3)> = cx.samples.points.iter().zip(&cx.samples.covered).filter(|((q, _), cov)| **cov != 0 && (*q - c).length() <= r).map(|(s, _)| s).collect();
    let rows: Vec<(Option<f64>, bool)> = inside
        .par_iter()
        .map(|(q, n)| {
            let all = index.crossings(*q, *n, 1.6);
            let d = all.iter().copied().min_by(|a, b| a.abs().total_cmp(&b.abs()));
            let inner = all.iter().any(|t| *t < -0.3 && *t > -1.4);
            (d, inner)
        })
        .collect();
    let devs: Vec<f64> = rows.iter().filter_map(|r| r.0).collect();
    let mut sorted = devs.clone();
    sorted.sort_by(f64::total_cmp);
    let inner = rows.iter().filter(|r| r.1).count() as f64 / rows.len().max(1) as f64;
    e.put("window_samples", rows.len());
    e.put_opt("window_mean_mm", mean(&devs));
    e.put_opt("window_p01_mm", (!sorted.is_empty()).then(|| pct(&sorted, 0.01)));
    e.put("inner_sheet_share", r3(inner));
    if let Some(s) = cx.strays {
        e.put("strays_input", s.input);
        e.put("strays_after_merge", s.after_merge);
        if cx.solid {
            e.put("solid_dropped_laser_strays", s.solid_dropped);
        } else {
            e.put("strays_after_prepare", s.after_prepare);
            if s.input > 0 && s.after_prepare as f64 > 0.05 * s.input as f64 {
                e.fail(format!("{} of {} stray points left after preparation", s.after_prepare, s.input));
            }
        }
    }
    if inner > 0.01 {
        e.fail(format!("a stray layer inside the material under {:.0} % of the window", 100.0 * inner));
    }
    if let Some(p1) = (!sorted.is_empty()).then(|| pct(&sorted, 0.01)).filter(|p| *p < -0.05) {
        e.fail(format!("pits down to {p1:.3} mm"));
    }
}

fn flatness(e: &mut FeatureEval, index: &MeshIndex, cx: &Context) {
    // The plate's top: seen samples facing +z at z = 0, away from the edges.
    let b = cx.truth.bounds;
    let tops: Vec<&(DVec3, DVec3)> = cx
        .samples
        .points
        .iter()
        .zip(&cx.samples.covered)
        .filter(|((q, n), cov)| **cov != 0 && n.z > 0.999 && q.z.abs() < 1e-3 && q.x > b[0][0] + 5.0 && q.x < b[1][0] - 5.0 && q.y > b[0][1] + 5.0 && q.y < b[1][1] - 5.0)
        .map(|(s, _)| s)
        .collect();
    // Away from marker sites (measured on their own).
    let far = |q: DVec3| cx.truth.stickers.iter().all(|s| (v3(s.sticker.center) - q).length() > 10.0);
    let devs: Vec<f64> = tops.par_iter().filter(|(q, _)| far(*q)).filter_map(|(q, n)| dev_at(index, *q, *n, 1.0)).collect();
    if devs.len() < 50 {
        return;
    }
    let m = mean(&devs).unwrap_or(0.0);
    let sd = rms(&devs.iter().map(|d| d - m).collect::<Vec<_>>()).unwrap_or(0.0);
    let mut abs: Vec<f64> = devs.iter().map(|d| (d - m).abs()).collect();
    abs.sort_by(f64::total_cmp);
    e.put("top_samples", devs.len());
    e.put_opt("top_mean_mm", Some(m));
    e.put_opt("top_relief_std_mm", Some(sd));
    e.put_opt("top_relief_p99_mm", Some(pct(&abs, 0.99)));
    if sd > 0.01 {
        e.fail(format!("waviness: relief std {sd:.3} mm"));
    }
    if pct(&abs, 0.99) > 0.03 {
        e.fail(format!("relief p99 {:.3} mm", pct(&abs, 0.99)));
    }
}

// ---------------------------------------------------------------------------- pass offsets

fn pass_offset(index: &MeshIndex, cx: &Context) -> Value {
    let groups: [(&str, Box<dyn Fn(u32) -> bool + Sync>); 3] = [("pass_1_only", Box::new(|c| c == 1)), ("other_passes_only", Box::new(|c| c != 0 && c & 1 == 0)), ("overlap", Box::new(|c| c & 1 != 0 && c & !1 != 0))];
    let mut out = serde_json::Map::new();
    let mut means = Vec::new();
    for (name, keep) in &groups {
        let pts: Vec<&(DVec3, DVec3)> = cx.samples.points.iter().zip(&cx.samples.covered).filter(|(_, c)| keep(**c)).map(|(s, _)| s).collect();
        let devs: Vec<f64> = pts.par_iter().filter_map(|(q, n)| dev_at(index, *q, *n, 1.0)).collect();
        let m = mean(&devs);
        let spread = m.and_then(|m| rms(&devs.iter().map(|d| d - m).collect::<Vec<_>>()));
        out.insert(name.to_string(), serde_json::json!({"samples": pts.len(), "measured": devs.len(), "mean_mm": m.map(r3), "spread_mm": spread.map(r3)}));
        means.push(m);
    }
    let biases: Vec<Value> = cx.truth.passes.iter().map(|p| serde_json::json!({"pass": p.id, "depth_bias_mm": p.spec.depth_bias_mm, "offset_mm": p.spec.offset_mm})).collect();
    out.insert("pass_faults".into(), Value::from(biases));
    let step = match (means[0], means[2]) {
        (Some(a), Some(b)) => Some((a - b).abs()),
        _ => None,
    };
    let step2 = match (means[1], means[2]) {
        (Some(a), Some(b)) => Some((a - b).abs()),
        _ => None,
    };
    let worst = [step, step2].into_iter().flatten().reduce(f64::max);
    out.insert("step_mm".into(), worst.map(r3).map_or(Value::Null, Value::from));
    out.insert("handled".into(), Value::from(worst.is_none_or(|s| s <= 0.05)));
    Value::Object(out)
}

/// A one-line summary of a mesh evaluation.
pub fn summary_line(m: &MeshEval) -> String {
    let g = &m.global;
    format!(
        "o->t rms {:.3} p99 {:.3} | t->o p99 {:.3} miss {:.1}% | probes {}/{} | features {}/{} ok | {} comp, genus {}, {}",
        g.out_to_truth.rms_mm,
        g.out_to_truth.p99_mm,
        g.truth_to_out.p99_mm,
        100.0 * g.missing_share,
        m.probes_passed,
        m.probes,
        m.features.len() - m.features_failed(),
        m.features.len(),
        m.topology.components,
        m.topology.genus,
        if m.topology.watertight { "watertight" } else { "open" }
    )
}
