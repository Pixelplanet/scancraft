use super::markers::{self, Quadric};
use super::*;
use glam::Vec3;
use std::f32::consts::PI;

const R_OUT: f32 = 30.0;
const WALL: f32 = 3.0;
/// Polar angle of the open rim (cap around +Y).
const THETA_MAX: f32 = 70.0 * PI / 180.0;

/// Fibonacci samples of a sphere of radius `r`, restricted to the cap, normals ±radial.
fn cap(r: f32, spacing: f32, outward: bool, scan: u8) -> Vec<SolidPoint> {
    let n = (4.0 * PI * r * r / (spacing * spacing)) as usize;
    let golden = PI * (3.0 - 5f32.sqrt());
    let mut v = Vec::new();
    for i in 0..n {
        let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
        if y < THETA_MAX.cos() {
            continue;
        }
        let rad = (1.0 - y * y).sqrt();
        let th = golden * i as f32;
        let d = Vec3::new(rad * th.cos(), y, rad * th.sin());
        v.push(SolidPoint { position: d * r, normal: if outward { d } else { -d }, scan });
    }
    v
}

fn dir(theta_deg: f32, phi_deg: f32) -> Vec3 {
    let (t, p) = (theta_deg.to_radians(), phi_deg.to_radians());
    Vec3::new(t.sin() * p.cos(), t.cos(), t.sin() * p.sin())
}

struct Shell {
    points: Vec<SolidPoint>,
    hole_dir: Vec3,
    disc_dir: Vec3,
}

/// Outer cap (scan 1) with a 5 mm radius hole and a raised disc (r 4.5 mm, +0.4 mm),
/// inner cap 3 mm inside (scan 2).
fn shell(spacing: f32) -> Shell {
    let hole_dir = dir(35.0, 20.0);
    let disc_dir = dir(30.0, 200.0);
    let mut points = Vec::new();
    for mut p in cap(R_OUT, spacing, true, 1) {
        let d = p.position / R_OUT;
        if (d.angle_between(hole_dir) * R_OUT) < 5.0 {
            continue;
        }
        if (d.angle_between(disc_dir) * R_OUT) < 4.5 {
            p.position = d * (R_OUT + 0.4);
        }
        points.push(p);
    }
    points.extend(cap(R_OUT - WALL, spacing, false, 2));
    Shell { points, hole_dir, disc_dir }
}

fn test_params() -> SolidConfig {
    SolidConfig { cell_mm: 0.5, ..Default::default() }
}

fn radial_errors(mesh: &scancraft_core::Mesh, pred: impl Fn(Vec3) -> bool, r: f32) -> Vec<f32> {
    let mut e: Vec<f32> = mesh
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .filter(|p| pred(*p))
        .map(|p| (p.length() - r).abs())
        .collect();
    e.sort_by(|a, b| a.total_cmp(b));
    e
}

fn pct(e: &[f32], q: f32) -> f32 {
    e[((e.len() - 1) as f32 * q) as usize]
}

/// Away from the rim, angle from +Y in degrees.
fn polar_deg(p: Vec3) -> f32 {
    (p.y / p.length()).clamp(-1.0, 1.0).acos().to_degrees()
}

#[test]
fn quadric_fit_recovers_a_paraboloid() {
    let mut s = Vec::new();
    for i in -10..=10 {
        for j in -10..=10 {
            let (u, v) = (i as f64 * 0.7, j as f64 * 0.7);
            let z = 0.3 + 0.01 * u - 0.02 * v + 0.004 * u * u + 0.001 * u * v - 0.003 * v * v;
            s.push((u, v, z + if (i + j) % 7 == 0 { 0.5 } else { 0.0 }));
        }
    }
    let (q, rms, _) = Quadric::fit_robust(&s).unwrap();
    assert!(rms < 1e-6, "rms {rms}");
    assert!((q.eval(1.0, 2.0) - (0.3 + 0.01 - 0.04 + 0.004 + 0.002 - 0.012)).abs() < 1e-6);
}

#[test]
fn topology_of_a_closed_tetrahedron_and_an_open_triangle() {
    let pos = vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z];
    let tris = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
    let t = topo::analyze(&pos, &tris);
    assert!(t.watertight && t.manifold);
    assert_eq!(t.components.len(), 1);
    assert_eq!(t.components[0].genus, 0);
    assert!(t.total_volume_mm3 > 0.0);
    let t = topo::analyze(&pos, &[[0, 1, 2]]);
    assert!(!t.watertight);
    assert_eq!(t.boundary_edges, 3);
    assert_eq!(t.boundary_loops, 1);
}

#[test]
fn detector_finds_the_raised_disc_and_the_hole() {
    let sh = shell(0.4);
    let pts: Vec<WorkPoint> = sh
        .points
        .iter()
        .map(|p| WorkPoint { p: p.position, n: p.normal, scan: p.scan, kind: PointKind::Measured, area: 0.0 })
        .collect();
    let found = markers::detect(&pts, &DetectConfig::default());
    let disc = found.iter().find(|m| m.source == MarkerSource::DetectedDisc).expect("disc detected");
    assert!(disc.center.distance(sh.disc_dir * R_OUT) < 1.0, "disc at {:?}", disc.center);
    assert!((disc.radius_mm - 4.5).abs() < 1.0, "disc radius {}", disc.radius_mm);
    let hole = found.iter().find(|m| m.source == MarkerSource::DetectedHole).expect("hole detected");
    assert!(hole.center.distance(sh.hole_dir * R_OUT) < 1.0, "hole at {:?}", hole.center);
    assert!((hole.radius_mm - 5.0).abs() < 1.0, "hole radius {}", hole.radius_mm);
    // Nothing else on a clean cap.
    assert_eq!(found.len(), 2, "{found:?}");
}

#[test]
fn solid_of_a_thick_cap_is_closed_and_accurate() {
    let sh = shell(0.45);
    let res = build_solid(&sh.points, &[], &test_params());
    let rep = &res.report;
    let t = &rep.topology;
    assert!(t.watertight, "{t:?}");
    assert!(t.manifold, "{t:?}");
    assert_eq!(t.components.len(), 1, "{:?}", t.components);
    assert_eq!(t.components[0].genus, 0);
    assert!((rep.wall_median_mm - WALL).abs() < 0.1, "wall {}", rep.wall_median_mm);

    // Outer and inner faces away from the rim and the two sites.
    let away = |p: Vec3| {
        let d = p.normalize();
        polar_deg(p) < 60.0 && d.angle_between(sh.hole_dir) * R_OUT > 9.0 && d.angle_between(sh.disc_dir) * R_OUT > 9.0
    };
    let outer = radial_errors(&res.mesh, |p| p.length() > R_OUT - WALL / 2.0 && away(p), R_OUT);
    let inner = radial_errors(&res.mesh, |p| p.length() < R_OUT - WALL / 2.0 && away(p), R_OUT - WALL);
    assert!(pct(&outer, 0.5) < 0.05 && pct(&outer, 0.99) < 0.12, "outer p50 {} p99 {}", pct(&outer, 0.5), pct(&outer, 0.99));
    assert!(pct(&inner, 0.5) < 0.05 && pct(&inner, 0.99) < 0.12, "inner p50 {} p99 {}", pct(&inner, 0.5), pct(&inner, 0.99));

    // The disc is gone and the hole is spanned, both close to the true sphere.
    let site = |c: Vec3, r: f32| move |p: Vec3| p.length() > R_OUT - WALL / 2.0 && p.normalize().angle_between(c) * R_OUT < r;
    let disc = radial_errors(&res.mesh, site(sh.disc_dir, 5.5), R_OUT);
    let hole = radial_errors(&res.mesh, site(sh.hole_dir, 5.0), R_OUT);
    assert!(!disc.is_empty() && !hole.is_empty());
    eprintln!(
        "cap: outer p50 {:.4} p99 {:.4} | inner p50 {:.4} p99 {:.4} | disc max {:.4} | hole max {:.4} | cg {} (+{} coarse) | {:?}",
        pct(&outer, 0.5),
        pct(&outer, 0.99),
        pct(&inner, 0.5),
        pct(&inner, 0.99),
        disc.last().unwrap(),
        hole.last().unwrap(),
        rep.cg_iterations,
        rep.cg_coarse_iterations,
        rep.stages
    );
    assert!(*disc.last().unwrap() < 0.08, "disc site max {}", disc.last().unwrap());
    assert!(*hole.last().unwrap() < 0.12, "hole site max {}", hole.last().unwrap());
    assert!(rep.markers_detected.len() == 2, "{:?}", rep.markers_detected);
}

#[test]
fn listed_marker_and_one_sided_gap_are_handled() {
    // Outer face missing over a large patch (only completion can close it) and a listed marker.
    let mut sh = shell(0.45);
    let gap_dir = dir(45.0, 110.0);
    sh.points.retain(|p| p.scan != 1 || (p.position / R_OUT).angle_between(gap_dir) * R_OUT > 12.0);
    let m = Marker {
        id: "M1".into(),
        center: sh.disc_dir * R_OUT,
        normal: -sh.disc_dir, // wrong way round on purpose: re-oriented from the data
        scan: Some(1),
        source: MarkerSource::List,
        radius_mm: 0.0,
        score: 0.0,
    };
    let prm = SolidConfig { detect: None, ..test_params() };
    let res = build_solid(&sh.points, &[m], &prm);
    let t = &res.report.topology;
    assert!(t.watertight && t.manifold, "{t:?}");
    assert_eq!(t.components.len(), 1);
    assert!(res.report.completion_points > 1000, "completion {}", res.report.completion_points);
    let o = &res.report.marker_outcomes[0];
    assert!(o.cut_points > 100 && o.fill_points > 100, "{o:?}");

    let gap = radial_errors(
        &res.mesh,
        |p| p.length() > R_OUT - WALL / 2.0 && p.normalize().angle_between(gap_dir) * R_OUT < 10.0,
        R_OUT,
    );
    assert!(!gap.is_empty());
    eprintln!("gap: p50 {:.4} max {:.4} ({} completion points)", pct(&gap, 0.5), gap.last().unwrap(), res.report.completion_points);
    assert!(pct(&gap, 0.5) < 0.08 && *gap.last().unwrap() < 0.25, "gap p50 {} max {}", pct(&gap, 0.5), gap.last().unwrap());
    let disc = radial_errors(
        &res.mesh,
        |p| p.length() > R_OUT - WALL / 2.0 && p.normalize().angle_between(sh.disc_dir) * R_OUT < 5.5,
        R_OUT,
    );
    assert!(*disc.last().unwrap() < 0.08, "disc site max {}", disc.last().unwrap());
}

#[test]
fn hole_through_both_faces_is_filled() {
    // An 8 mm radius gap through both faces: without the hole filler it stays a tunnel.
    let mut sh = shell(0.45);
    let gap_dir = dir(40.0, 300.0);
    sh.points.retain(|p| (p.position.normalize()).angle_between(gap_dir) * R_OUT > 8.0);
    let prm = SolidConfig { detect: None, ..test_params() };
    let open = build_solid(&sh.points, &[], &SolidConfig { hole_fill: None, ..prm.clone() });
    assert_eq!(open.report.topology.genus_total(), 1, "without filling the gap is a tunnel");
    let res = build_solid(&sh.points, &[], &prm);
    let t = &res.report.topology;
    assert!(t.watertight && t.manifold, "{t:?}");
    assert_eq!(t.components.len(), 1);
    assert_eq!(t.genus_total(), 0, "hole filled: {:?}", t.components);
    let site = |r: f32| {
        radial_errors(&res.mesh, |p| p.length() > R_OUT - WALL / 2.0 && p.normalize().angle_between(gap_dir) * R_OUT < 6.0, r)
    };
    let e = site(R_OUT);
    eprintln!("two-sided hole: outer max {:.4} ({} holes, {} fill points)", e.last().unwrap(), res.report.holes_filled, res.report.hole_fill_points);
    assert!(*e.last().unwrap() < 0.15, "outer max {}", e.last().unwrap());
}

#[test]
fn thick_cap_vertex_origins_are_tagged() {
    let sh = shell(0.45);
    let res = build_solid(&sh.points, &[], &test_params());
    let c = res.report.origin_counts;
    assert_eq!(c.iter().sum::<usize>(), res.mesh.vertices.len());
    assert_eq!(res.vertex_origin.len(), res.mesh.vertices.len());
    let f = res.report.origin_fractions();
    // Both faces are measured almost everywhere; the lip at the rim is completed surface and
    // the detected disc is marker fill.
    assert!(f[0] > 0.8, "{f:?}");
    assert!(c[1] > 0 && c[3] > 0, "{c:?}");
    for (v, o) in res.mesh.vertices.iter().zip(&res.vertex_origin) {
        assert_eq!(v.color, o.color());
    }
    assert_eq!(res.report.defects_after, (0, 0), "{:?}", res.report.defects_before);
}

#[test]
fn covered_layers_keep_the_trusted_scan() {
    // Scan 1 covers x < 10 of a plane; scan 3 covers the whole plane 0.25 mm above.
    let mut pts = Vec::new();
    for i in 0..80 {
        for j in 0..40 {
            let (x, y) = (i as f32 * 0.25, j as f32 * 0.25);
            if x < 10.0 {
                pts.push(WorkPoint { p: Vec3::new(x, y, 0.0), n: Vec3::Z, scan: 1, kind: PointKind::Measured, area: 0.0 });
            }
            pts.push(WorkPoint { p: Vec3::new(x + 0.1, y + 0.1, 0.25), n: Vec3::Z, scan: 3, kind: PointKind::Measured, area: 0.0 });
        }
    }
    let dropped = prep::drop_covered_layers(&mut pts, &[1], 1.0, 0.5);
    assert!(dropped > 1000, "dropped {dropped}");
    // Scan 3 survives only where scan 1 is absent (allowing a 1 mm transition).
    assert!(pts.iter().filter(|p| p.scan == 3).all(|p| p.p.x > 9.0));
    assert!(pts.iter().filter(|p| p.scan == 3 && p.p.x > 11.0).count() > 1300);
    assert_eq!(pts.iter().filter(|p| p.scan == 1).count(), 40 * 40);
    // No priority: nothing is dropped.
    let n = pts.len();
    assert_eq!(prep::drop_covered_layers(&mut pts, &[], 1.0, 0.5), 0);
    assert_eq!(pts.len(), n);
}

#[test]
fn remove_markers_restores_the_sphere_and_leaves_the_inner_face() {
    let sh = shell(0.4);
    // The same sticker listed once per scan, 1.5 mm apart: merged into one site.
    let m = |id: &str, off: Vec3| Marker {
        id: id.into(),
        center: sh.disc_dir * R_OUT + off,
        normal: sh.disc_dir,
        scan: Some(1),
        source: MarkerSource::List,
        radius_mm: 0.0,
        score: 0.0,
    };
    let (eu, _) = markers::tangent_frame(sh.disc_dir);
    let out = remove_markers(&sh.points, &[m("M1", Vec3::ZERO), m("M1", eu * 1.5)], &MarkerConfig::default(), None);
    assert_eq!(out.sites.len(), 1, "{:?}", out.sites);
    assert!(out.sites[0].radius_mm > 0.7);
    let o = &out.outcomes[0];
    assert!(o.cut_points > 100 && o.fill_points > 300, "{o:?}");
    let inner_in = sh.points.iter().filter(|p| p.scan == 2).count();
    let inner_out = out.samples.iter().filter(|p| p.scan == 2 && p.kind == PointKind::Measured).count();
    assert_eq!(inner_in, inner_out, "the inner face must not be cut");
    // No raised disc point is left, the patch lies on the sphere.
    assert!(out.samples.iter().filter(|p| p.kind == PointKind::Measured && p.scan == 1).all(|p| (p.position.length() - R_OUT).abs() < 0.2));
    let worst = out.samples.iter().filter(|p| p.kind == PointKind::MarkerFill).map(|p| (p.position.length() - R_OUT).abs()).fold(0f32, f32::max);
    assert!(worst < 0.04, "infill deviates {worst} mm");
}

#[test]
fn unfillable_marker_site_is_kept() {
    // A listed marker over the open rim: the annulus is not covered all round.
    let sh = shell(0.45);
    let d = dir(69.0, 45.0);
    let m = Marker { id: "R".into(), center: d * R_OUT, normal: d, scan: None, source: MarkerSource::List, radius_mm: 0.0, score: 0.0 };
    let out = remove_markers(&sh.points, &[m], &MarkerConfig::default(), None);
    assert!(out.outcomes[0].fill.starts_with("kept"), "{:?}", out.outcomes[0]);
    assert_eq!(out.samples.len(), sh.points.len());
}

#[test]
fn fill_holes_patches_the_enclosed_hole_but_not_the_rim() {
    let sh = shell(0.45);
    let samples: Vec<Sample> =
        sh.points.iter().map(|p| Sample { position: p.position, normal: p.normal, scan: p.scan, kind: PointKind::Measured }).collect();
    let res = fill_holes(&samples, &HoleConfig::default());
    assert!(res.holes >= 1 && !res.fill.is_empty());
    let hole = sh.hole_dir * R_OUT;
    for p in &res.fill {
        assert!(p.position.distance(hole) < 5.6, "fill point {:?} away from the hole", p.position);
        assert!((p.position.length() - R_OUT).abs() < 0.05, "fill off the sphere by {}", p.position.length() - R_OUT);
        assert_eq!(p.kind, PointKind::HoleFill);
    }
}

#[test]
fn sliver_edges_collapse_and_crossings_are_found() {
    // Octahedron with one vertex moved next to another: one collapse, still closed, genus 0.
    let mut pos = vec![Vec3::X, Vec3::NEG_X, Vec3::Y, Vec3::NEG_Y, Vec3::Z, Vec3::NEG_Z];
    pos[0] = Vec3::new(0.01, 1.0, 0.002);
    let tris = vec![[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]];
    let (p2, t2, n) = cleanup::collapse_short_edges(pos, tris, 0.05);
    assert_eq!(n, 1);
    let t = topo::analyze(&p2, &t2);
    assert!(t.watertight && t.manifold, "{t:?}");
    assert_eq!(t.genus_total(), 0);
    // Two triangles crossing each other.
    let pos = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.5, 0.5, -1.0), Vec3::new(0.5, 0.5, 1.0), Vec3::new(1.5, 1.5, 0.0)];
    assert_eq!(cleanup::self_intersections(&pos, &[[0, 1, 2], [3, 4, 5]]).len(), 1);
    assert!(cleanup::self_intersections(&pos[..3], &[[0, 1, 2]]).is_empty());
}

#[test]
fn mesh_fairing_flattens_a_bump() {
    // A flat grid mesh with a smooth 0.3 mm bump at the centre is faired back onto the plane.
    let n = 61;
    let mut verts = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (x, y) = ((i as f32 - 30.0) * 0.4, (j as f32 - 30.0) * 0.4);
            let r = (x * x + y * y).sqrt();
            let z = if r < 4.0 { 0.15 * (1.0 + (PI * r / 4.0).cos()) } else { 0.0 };
            verts.push(scancraft_core::Point3D { position: [x, y, z], normal: [0.0, 0.0, 1.0], color: ColorRgba::WHITE, confidence: 1.0 });
        }
    }
    let mut tris = Vec::new();
    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let a = (j * n + i) as u32;
            tris.push(Triangle::new(a, a + 1, a + n as u32 + 1));
            tris.push(Triangle::new(a, a + n as u32 + 1, a + n as u32));
        }
    }
    let mut mesh = Mesh::from_geometry(verts, tris);
    let site = Marker { id: "B".into(), center: Vec3::ZERO, normal: Vec3::Z, scan: None, source: MarkerSource::List, radius_mm: 0.0, score: 0.0 };
    let out = fair_marker_sites(&mut mesh, &[site], &MarkerConfig::default());
    assert_eq!(out[0].status, "faired", "{:?}", out[0]);
    let worst = mesh.vertices.iter().map(|v| v.position[2].abs()).fold(0f32, f32::max);
    assert!(worst < 0.01, "bump left: {worst} mm");
}

/// Outer cap (scan 1) with raised discs (r 4.5 mm, +0.4 mm) at `discs`, inner cap 3 mm inside
/// (scan 2).
fn cap_with_discs(spacing: f32, discs: &[Vec3]) -> Vec<SolidPoint> {
    let mut points = Vec::new();
    for mut p in cap(R_OUT, spacing, true, 1) {
        let d = p.position / R_OUT;
        if discs.iter().any(|c| d.angle_between(*c) * R_OUT < 4.5) {
            p.position = d * (R_OUT + 0.4);
        }
        points.push(p);
    }
    points.extend(cap(R_OUT - WALL, spacing, false, 2));
    points
}

fn listed(id: &str, c: Vec3, n: Vec3) -> Marker {
    Marker { id: id.into(), center: c, normal: n, scan: Some(1), source: MarkerSource::List, radius_mm: 0.0, score: 0.0 }
}

#[test]
fn tilted_marker_normal_is_reoriented_from_the_data() {
    let d = dir(30.0, 200.0);
    let pts = cap_with_discs(0.4, &[d]);
    // Listed normal 30° off the sheet normal (and flipped): direction and sign come from the data.
    let (eu, _) = markers::tangent_frame(d);
    let tilted = -(d * 30f32.to_radians().cos() + eu * 30f32.to_radians().sin());
    let out = remove_markers(&pts, &[listed("T", d * R_OUT, tilted)], &MarkerConfig::default(), None);
    let n = out.sites[0].normal;
    let err = n.angle_between(d).to_degrees();
    assert!(err < 2.0, "normal still {err:.2} deg off the sheet");
    let worst = out.samples.iter().filter(|p| p.kind == PointKind::MarkerFill).map(|p| (p.position.length() - R_OUT).abs()).fold(0f32, f32::max);
    assert!(worst < 0.04, "infill deviates {worst} mm");
    assert!(out.samples.iter().filter(|p| p.kind == PointKind::Measured && p.scan == 1).all(|p| (p.position.length() - R_OUT).abs() < 0.2));
}

#[test]
fn overlapping_marker_sites_give_one_smooth_patch() {
    // Two stickers 10 mm apart on one sheet: the cuts (r 6.5) overlap, and each annulus reaches
    // over the other sticker.
    let (a, b) = (dir(28.0, 90.0), dir(28.0 + (10.0 / R_OUT).to_degrees(), 90.0));
    let pts = cap_with_discs(0.4, &[a, b]);
    let sites = [listed("A", a * R_OUT, a), listed("B", b * R_OUT, b)];
    let cfg = MarkerConfig::default();
    let out = remove_markers(&pts, &sites, &cfg, None);
    assert_eq!(out.sites.len(), 2, "{:?}", out.sites);
    for o in &out.outcomes {
        assert_eq!(o.fill, "quadric", "{o:?}");
        assert!(o.annulus_rms_mm < 0.01, "annulus of {} takes in the other sticker: {o:?}", o.id);
    }
    let fill: Vec<Vec3> = out.samples.iter().filter(|p| p.kind == PointKind::MarkerFill).map(|p| p.position).collect();
    let worst = fill.iter().map(|p| (p.length() - R_OUT).abs()).fold(0f32, f32::max);
    assert!(worst < 0.04, "infill deviates {worst} mm");
    // One layer in the overlap of the two cuts: about one lattice point per spacing squared.
    let rc = 6.5f32;
    let (r, d) = (rc - 0.3, 10.0f32);
    let in_lens = |p: &Vec3| p.normalize().angle_between(a) * R_OUT < r && p.normalize().angle_between(b) * R_OUT < r;
    let n_lens = fill.iter().filter(|p| in_lens(p)).count() as f32;
    let area = 2.0 * r * r * (d / (2.0 * r)).acos() - 0.5 * d * (4.0 * r * r - d * d).sqrt();
    let density = n_lens / (area / (cfg.fill_spacing_mm * cfg.fill_spacing_mm));
    assert!((0.8..1.25).contains(&density), "lens carries {density:.2} layers ({n_lens} points)");
    // No two fill points closer than a third of the spacing (a second layer would be).
    let grid = grid::PointGrid::new(&fill, 0.3);
    let close = (0..fill.len())
        .filter(|&i| {
            let mut c = 0;
            grid.for_each_within(&fill, fill[i], 0.1, |_, _| c += 1);
            c > 1
        })
        .count();
    assert!(close < fill.len() / 50, "{close} of {} fill points have a twin", fill.len());
}

#[test]
fn overlapping_fair_sites_share_the_vertices() {
    // Flat grid with two 0.3 mm bumps 9 mm apart: both sites move the vertices of the overlap,
    // the result must be flat everywhere (no site undoes the other).
    let n = 91;
    let bump = |x: f32, y: f32| {
        let mut z = 0.0;
        for cx in [-4.5f32, 4.5] {
            let r = ((x - cx).powi(2) + y * y).sqrt();
            if r < 4.0 {
                z += 0.15 * (1.0 + (PI * r / 4.0).cos());
            }
        }
        z
    };
    let mut verts = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (x, y) = ((i as f32 - 45.0) * 0.4, (j as f32 - 45.0) * 0.4);
            verts.push(scancraft_core::Point3D { position: [x, y, bump(x, y)], normal: [0.0, 0.0, 1.0], color: ColorRgba::WHITE, confidence: 1.0 });
        }
    }
    let mut tris = Vec::new();
    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let a = (j * n + i) as u32;
            tris.push(Triangle::new(a, a + 1, a + n as u32 + 1));
            tris.push(Triangle::new(a, a + n as u32 + 1, a + n as u32));
        }
    }
    let mut mesh = Mesh::from_geometry(verts, tris);
    let site = |id: &str, x: f32| Marker { id: id.into(), center: Vec3::new(x, 0.0, 0.0), normal: Vec3::Z, scan: None, source: MarkerSource::List, radius_mm: 0.0, score: 0.0 };
    let out = fair_marker_sites(&mut mesh, &[site("A", -4.5), site("B", 4.5)], &MarkerConfig::default());
    assert!(out.iter().all(|o| o.status == "faired"), "{out:?}");
    let worst = mesh.vertices.iter().map(|v| v.position[2].abs()).fold(0f32, f32::max);
    assert!(worst < 0.01, "bump left: {worst} mm");
}

/// Cap with a 6 mm wide slot along the plane x = 0 out to 60° from the pole: longer than a 24 mm
/// window, so it never looks enclosed there. `outer_only`: only the outer face is cut;
/// `cut_faces`: measured points on the slot walls (a real opening).
fn slotted(outer_only: bool, cut_faces: bool) -> Vec<SolidPoint> {
    let in_slot = |d: Vec3| d.x.abs() * R_OUT < 3.0 && polar_deg(d) < 60.0;
    let mut pts: Vec<SolidPoint> = cap(R_OUT, 0.45, true, 1).into_iter().filter(|p| !in_slot(p.position / R_OUT)).collect();
    pts.extend(cap(R_OUT - WALL, 0.45, false, 2).into_iter().filter(|p| outer_only || !in_slot(p.position / (R_OUT - WALL))));
    if cut_faces {
        let x = 3.0 / R_OUT;
        for side in [-1.0f32, 1.0] {
            for k in 0..=(2.0 * 60f32.to_radians() * R_OUT / 0.4) as i32 {
                let t = -60f32.to_radians() + k as f32 * 0.4 / R_OUT;
                for l in 0..=7 {
                    let r = R_OUT - WALL + l as f32 * WALL / 7.0;
                    let s = (1.0 - x * x).sqrt();
                    let d = Vec3::new(side * x, t.cos() * s, t.sin() * s);
                    pts.push(SolidPoint { position: d * r, normal: Vec3::new(-side, 0.0, 0.0), scan: 1 });
                }
            }
        }
    }
    pts
}

fn slot_fill(pts: &[SolidPoint], bridge_span_mm: f32) -> HoleFilling {
    let samples: Vec<Sample> =
        pts.iter().map(|p| Sample { position: p.position, normal: p.normal, scan: p.scan, kind: PointKind::Measured }).collect();
    fill_holes(&samples, &HoleConfig { windows_mm: vec![24.0], bridge_span_mm, ..Default::default() })
}

/// Fraction of 1° steps along the slot centre line (outer face, ±50° from the pole) with a fill
/// point within 0.6 mm.
fn centre_line_cover(fill: &[Sample]) -> f32 {
    let steps: Vec<Vec3> =
        (-50..=50).map(|k| Vec3::new(0.0, (k as f32).to_radians().cos(), (k as f32).to_radians().sin()) * R_OUT).collect();
    steps.iter().filter(|s| fill.iter().any(|p| p.position.distance(**s) < 0.6)).count() as f32 / steps.len() as f32
}

#[test]
fn long_gap_through_both_faces_is_bridged() {
    let pts = slotted(false, false);
    let none = slot_fill(&pts, 0.0);
    assert!(centre_line_cover(&none.fill) < 0.05, "without bridging the slot is open in a 24 mm window");
    let res = slot_fill(&pts, 40.0);
    let cover = centre_line_cover(&res.fill);
    eprintln!("bridge: cover {cover:.2}, {} patches, {} points", res.holes, res.fill.len());
    assert!(cover > 0.95, "slot centre line covered {cover:.2}");
    // One patch per face; the bridging can leave a stray point or two in the notches of the
    // jagged rim beyond the slot's end, and each of those counts as a patch of its own.
    let strays = res.fill.iter().filter(|p| polar_deg(p.position) > 70.0).count();
    assert!(strays <= 4, "{strays} fill points beyond the rim");
    assert!(res.holes >= 2 && res.holes <= 2 + strays, "{} patches, {strays} strays", res.holes);
    for p in &res.fill {
        assert!(p.position.x.abs() < 3.0, "bridge outside the slot at {:?}", p.position);
        let r = p.position.length();
        assert!((r - R_OUT).abs() < 0.1 || (r - (R_OUT - WALL)).abs() < 0.1, "bridge off both faces: r {r}");
    }
}

#[test]
fn real_opening_and_one_sided_gap_are_not_bridged() {
    // The slot walls are seen: a real opening.
    let res = slot_fill(&slotted(false, true), 40.0);
    assert!(centre_line_cover(&res.fill) < 0.05, "real opening bridged ({} points)", res.fill.len());
    // Only the outer face is missing: the inner face lies behind the gap (left to completion).
    let res = slot_fill(&slotted(true, false), 40.0);
    assert!(centre_line_cover(&res.fill) < 0.05, "one-sided gap bridged ({} points)", res.fill.len());
}

#[test]
fn bridged_slot_gives_a_closed_solid_without_a_tunnel() {
    let pts = slotted(false, false);
    let holes = |span: f32| Some(HoleConfig { windows_mm: vec![24.0], bridge_span_mm: span, ..Default::default() });
    let prm = SolidConfig { detect: None, ..test_params() };
    let open = build_solid(&pts, &[], &SolidConfig { hole_fill: holes(0.0), ..prm.clone() });
    assert_eq!(open.report.topology.genus_total(), 1, "without bridging the slot is a tunnel");
    let res = build_solid(&pts, &[], &SolidConfig { hole_fill: holes(40.0), ..prm });
    let t = &res.report.topology;
    assert!(t.watertight && t.manifold, "{t:?}");
    assert_eq!(t.components.len(), 1);
    assert_eq!(t.genus_total(), 0, "slot bridged: {:?}", t.components);
    let e = radial_errors(&res.mesh, |p| p.length() > R_OUT - WALL / 2.0 && p.x.abs() < 2.0 && polar_deg(p) < 50.0, R_OUT);
    eprintln!(
        "bridged slot: outer p50 {:.4} max {:.4} ({} patches, {} fill points)",
        pct(&e, 0.5),
        e.last().unwrap(),
        res.report.holes_filled,
        res.report.hole_fill_points
    );
    assert!(*e.last().unwrap() < 0.15, "outer max {}", e.last().unwrap());
}

#[test]
fn repair_unfolds_a_folded_vertex() {
    // Flat grid with one vertex pushed across its neighbours (its triangles turn over and cross
    // their neighbours): the local relaxation removes the folds and crossings.
    let n = 21;
    let mut pos: Vec<Vec3> = (0..n * n).map(|k| Vec3::new((k % n) as f32, (k / n) as f32, 0.0)).collect();
    let mut tris = Vec::new();
    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let a = (j * n + i) as u32;
            tris.push([a, a + 1, a + n as u32 + 1]);
            tris.push([a, a + n as u32 + 1, a + n as u32]);
        }
    }
    pos[10 * n + 10] += Vec3::new(1.7, 0.4, 0.3);
    let (before, after) = cleanup::repair(&mut pos, &tris, 4);
    eprintln!("repair: before {before:?} after {after:?}");
    assert!(before.1 > 0, "no fold to repair: {before:?}");
    assert_eq!(after, (0, 0), "before {before:?}");
    assert!(cleanup::folded_edges(&pos, &tris, -0.9).is_empty());
}
