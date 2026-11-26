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
    // A listed marker over the open rim: the annulus is not covered all round. With partly
    // covered sites patched (the default, see `partly_covered_marker_site_is_patched_up_to_the_rim`)
    // this needs fewer than `partial_min_sectors` covered; here that is switched off.
    let sh = shell(0.45);
    let d = dir(69.0, 45.0);
    let m = Marker { id: "R".into(), center: d * R_OUT, normal: d, scan: None, source: MarkerSource::List, radius_mm: 0.0, score: 0.0 };
    let out = remove_markers(&sh.points, &[m], &MarkerConfig { partial_min_sectors: 8, ..MarkerConfig::default() }, None);
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


#[test]
fn partly_covered_marker_site_is_patched_up_to_the_rim() {
    // A sticker (raised disc, r 5.5 mm, 0.4 mm) 2.6 mm from the open rim: its annulus is covered
    // on the inner side only. It is cut and patched where the scan has surface, never past the rim.
    let d = dir(65.0, 45.0);
    let mut pts = Vec::new();
    for mut p in cap(R_OUT, 0.4, true, 1) {
        let u = p.position / R_OUT;
        if u.angle_between(d) * R_OUT < 5.5 {
            p.position = u * (R_OUT + 0.4);
        }
        pts.push(p);
    }
    let m = Marker { id: "E".into(), center: d * R_OUT, normal: d, scan: None, source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    let out = remove_markers(&pts, &[m], &MarkerConfig::default(), None);
    let o = &out.outcomes[0];
    assert!(o.fill.starts_with("quadric, partial"), "{o:?}");
    assert!(o.sectors_covered < 8 && o.fill_points > 100, "{o:?}");
    // Up to one sample spacing (0.4 mm) past the last scanned point: the cut's cells are grown by
    // one lattice cell to close the gaps between its points.
    let rim = THETA_MAX.to_degrees() + (0.45 / R_OUT).to_degrees();
    for p in out.samples.iter().filter(|p| p.kind == PointKind::MarkerFill) {
        assert!(polar_deg(p.position) < rim, "patch point past the rim at {:.2} deg", polar_deg(p.position));
        assert!((p.position.length() - R_OUT).abs() < 0.06, "patch {} mm off the sphere", p.position.length() - R_OUT);
    }
    let raised = out.samples.iter().filter(|p| p.kind == PointKind::Measured && p.position.length() > R_OUT + 0.2).count();
    assert_eq!(raised, 0, "sticker points left");
}

/// A hinge-bracket plate: `w` x `h` mm, 3 mm thick, both faces and the side faces scanned, with a
/// round screw hole (r 3.2 mm) through it at (4, 0), its bore scanned when `bore`.
fn bracket_plate(w: f32, h: f32, bore: bool) -> Vec<SolidPoint> {
    let hole = Vec3::new(4.0, 0.0, 0.0);
    let (rh, s) = (3.2, 0.4);
    let mut v = Vec::new();
    let (nx, ny) = ((w / s) as i32, (h / s) as i32);
    for i in 0..=nx {
        for j in 0..=ny {
            let (x, y) = (-w / 2.0 + i as f32 * s, -h / 2.0 + j as f32 * s);
            if Vec3::new(x - hole.x, y - hole.y, 0.0).length() < rh {
                continue;
            }
            v.push(SolidPoint { position: Vec3::new(x, y, 0.0), normal: Vec3::Z, scan: 1 });
            if Vec3::new(x + 0.2 - hole.x, y + 0.2 - hole.y, 0.0).length() >= rh {
                v.push(SolidPoint { position: Vec3::new(x + 0.2, y + 0.2, -3.0), normal: -Vec3::Z, scan: 2 });
            }
        }
    }
    for k in 0..=7 {
        let z = -k as f32 * 3.0 / 7.0;
        for i in 0..=nx {
            let x = -w / 2.0 + i as f32 * s;
            v.push(SolidPoint { position: Vec3::new(x, -h / 2.0, z), normal: -Vec3::Y, scan: 1 });
            v.push(SolidPoint { position: Vec3::new(x, h / 2.0, z), normal: Vec3::Y, scan: 1 });
        }
        for j in 0..=ny {
            let y = -h / 2.0 + j as f32 * s;
            v.push(SolidPoint { position: Vec3::new(-w / 2.0, y, z), normal: -Vec3::X, scan: 1 });
            v.push(SolidPoint { position: Vec3::new(w / 2.0, y, z), normal: Vec3::X, scan: 1 });
        }
        // The bore: normals point into the hole.
        for a in (0..48).filter(|_| bore) {
            let t = a as f32 / 48.0 * std::f32::consts::TAU;
            let r = Vec3::new(t.cos(), t.sin(), 0.0);
            v.push(SolidPoint { position: hole + r * rh + Vec3::Z * z, normal: -r, scan: 1 });
        }
    }
    v
}

fn work_points(pts: &[SolidPoint]) -> Vec<WorkPoint> {
    pts.iter().map(|p| WorkPoint { p: p.position, n: p.normal, scan: p.scan, kind: PointKind::Measured, area: 0.0 }).collect()
}

#[test]
fn screw_hole_in_a_bracket_plate_is_no_marker_and_stays_open() {
    let hole = Vec3::new(4.0, 0.0, 0.0);
    let holes_found = |pts: &[SolidPoint]| markers::detect(&work_points(pts), &DetectConfig::default()).into_iter().filter(|m| m.source == MarkerSource::DetectedHole).count();
    // A wide plate: round the hole the sheet is plain, like round a cut-out sticker. Without its
    // bore the hole is taken for one (the gap a sticker was cut out of looks the same) ...
    assert!(holes_found(&bracket_plate(40.0, 40.0, false)) > 0, "the hole is not even seen as one");
    // ... with its bore scanned it is a screw hole and no marker.
    let wide = bracket_plate(40.0, 40.0, true);
    assert_eq!(holes_found(&wide), 0, "the screw hole was taken for a cut-out marker");
    // A narrow plate (the hinge bracket): the plate's edge is near all round.
    assert_eq!(holes_found(&bracket_plate(24.0, 14.0, true)), 0);
    // The solid keeps the bore.
    let res = build_solid(&wide, &[], &SolidConfig { cell_mm: 0.3, ..Default::default() });
    let free = res
        .mesh
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .filter(|p| p.z < -0.5 && p.z > -2.5)
        .map(|p| Vec3::new(p.x - hole.x, p.y - hole.y, 0.0).length())
        .fold(f32::MAX, f32::min);
    assert!(free > 2.2, "the bore is closed to a free radius of {free} mm");
}

#[test]
fn through_hole_without_a_scanned_bore_is_not_filled() {
    // The narrow bracket plate with its bore not scanned: nothing on the hole's axis on either
    // side, so the hole filler leaves it open (it is not a gap in one face of a wall).
    let hole = Vec3::new(4.0, 0.0, 0.0);
    let samples: Vec<Sample> = bracket_plate(24.0, 14.0, false).iter().map(|p| Sample { position: p.position, normal: p.normal, scan: p.scan, kind: PointKind::Measured }).collect();
    let in_bore = |cfg: &HoleConfig| fill_holes(&samples, cfg).fill.iter().filter(|p| Vec3::new(p.position.x - hole.x, p.position.y - hole.y, 0.0).length() < 2.5).count();
    assert_eq!(in_bore(&HoleConfig::default()), 0, "fill points in the bore");
    // Switched off, the same hole is patched: the rule is what keeps it.
    assert!(in_bore(&HoleConfig { keep_through_holes: false, ..HoleConfig::default() }) > 0, "the hole is not even seen as one");
}

#[test]
fn scan_offset_is_equalised_and_a_rough_layer_dropped() {
    // Scan 1: a clean plane. Scan 2 over x > 10: 0.12 mm above it (a depth bias), as clean.
    // Scan 3 over x < 10: the plane with ±0.3 mm streaks (a grazing fringe).
    let plane = |x0: f32, x1: f32, z: f32, scan: u8, salt: f32, noise: f32| {
        let mut v = Vec::new();
        let mut k = 0u32;
        for i in 0..((x1 - x0) / 0.3) as i32 {
            for j in 0..100 {
                k = k.wrapping_mul(1664525).wrapping_add(1013904223);
                let e = ((k >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 2.0;
                let (x, y) = (x0 + i as f32 * 0.3 + salt, j as f32 * 0.3 + salt);
                let streak = if noise > 0.0 { noise * (y * 2.1).sin() * (x * 0.7).cos() } else { 0.0 };
                v.push(WorkPoint { p: Vec3::new(x, y, z + 0.01 * e + streak), n: Vec3::Z, scan, kind: PointKind::Measured, area: 0.0 });
            }
        }
        v
    };
    let mut pts = plane(0.0, 30.0, 0.0, 1, 0.0, 0.0);
    pts.extend(plane(10.0, 30.0, 0.12, 2, 0.1, 0.0));
    let shifts = prep::equalise_scan_offsets(&mut pts, 0.03, 0.4, 500);
    assert_eq!(shifts.len(), 1, "{shifts:?}");
    assert_eq!(shifts[0].0, 2);
    assert!((shifts[0].1 + 0.12).abs() < 0.02, "{shifts:?}");
    assert!(prep::scan_offsets(&pts).iter().all(|o| o.1.abs() < 0.03), "{:?}", prep::scan_offsets(&pts));

    let mut pts = plane(0.0, 30.0, 0.0, 1, 0.0, 0.0);
    pts.extend(plane(0.0, 10.0, 0.0, 3, 0.1, 0.3));
    pts.extend(plane(30.0, 36.0, 0.0, 3, 0.1, 0.3));
    let (ones, threes_alone) = (pts.iter().filter(|p| p.scan == 1).count(), pts.iter().filter(|p| p.scan == 3 && p.p.x > 31.5).count());
    let dropped = prep::drop_rough_layers(&mut pts, 1.5, 0.4, 0.06);
    assert_eq!(pts.iter().filter(|p| p.scan == 1).count(), ones, "the clean scan is kept");
    assert!(pts.iter().filter(|p| p.scan == 3 && p.p.x < 9.0).count() * 20 < 30 * 100, "{dropped} dropped: the rough layer over the clean scan stays");
    assert_eq!(pts.iter().filter(|p| p.scan == 3 && p.p.x > 31.5).count(), threes_alone, "the rough scan is kept where it is alone");
}

#[test]
fn laser_strays_behind_a_surface_go_but_edges_and_steps_stay() {
    let w = |p: Vec3, n: Vec3| WorkPoint { p, n, scan: 1, kind: PointKind::Measured, area: 0.0 };
    let mut k = 0u32;
    let mut e = || {
        k = k.wrapping_mul(1664525).wrapping_add(1013904223);
        ((k >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 2.0
    };
    let mut keep = Vec::new();
    // A surface seen from in front, and the laser lines 0.8 mm behind it (2 mm wide, 9 mm apart,
    // most of them facing the other way).
    let mut strays = Vec::new();
    for i in 0..80 {
        for j in 0..60 {
            let (x, y) = (i as f32 * 0.5, j as f32 * 0.5);
            keep.push(w(Vec3::new(x + 0.1 * e(), y + 0.1 * e(), 0.03 * e()), Vec3::Z));
            if (x + 0.03 * (y - 15.0).powi(2)).rem_euclid(9.0) < 2.0 && x > 3.0 && x < 37.0 && y > 3.0 && y < 27.0 {
                let n = if i % 6 == 0 { Vec3::Z } else { -Vec3::Z };
                strays.push(w(Vec3::new(x + 0.2 + 0.1 * e(), y + 0.2 + 0.1 * e(), -0.8 + 0.12 * e()), n));
            }
        }
    }
    // A block's convex edge: top face and side face, the side face reaching up to the edge.
    for i in 0..20 {
        for j in 0..20 {
            keep.push(w(Vec3::new(50.0 + i as f32 * 0.5, j as f32 * 0.5, 0.0), Vec3::Z));
        }
        for d in 1..10 {
            keep.push(w(Vec3::new(60.0, i as f32 * 0.5, -(d as f32) * 0.5), Vec3::X));
        }
    }
    // A step: a floor and, beside it, a top 0.8 mm higher with its flank.
    for i in 0..20 {
        for j in 0..20 {
            let x = 70.0 + i as f32 * 0.5;
            keep.push(w(Vec3::new(x, j as f32 * 0.5, if x > 75.0 { 0.8 } else { 0.0 }), Vec3::Z));
        }
        keep.push(w(Vec3::new(75.2, i as f32 * 0.5, 0.4), -Vec3::X));
    }
    let (n_keep, n_strays) = (keep.len(), strays.len());
    let mut pts = keep;
    pts.extend(strays);
    let dropped = prep::drop_behind_strays(&mut pts, 2.0, 0.4, 1.6);
    let strays_left = pts.iter().filter(|p| p.p.z < -0.4 && p.p.x < 45.0).count();
    let kept = pts.len() - strays_left;
    assert!(strays_left * 20 <= n_strays, "{strays_left} of {n_strays} strays left ({dropped} dropped)");
    assert!(kept * 100 >= n_keep * 99, "{} of {n_keep} surface points dropped", n_keep - kept);
}

/// A sheet z = `surface(x, y)` sampled at 0.35 mm round the origin (radius 20 mm), facing +z,
/// with a sticker (a disc 0.4 mm proud, r 5.5 mm) at the origin and no points at all in the ring
/// `gap` (mm) around it.
fn sheet_with_sticker(surface: impl Fn(f32, f32) -> f32, gap: (f32, f32)) -> Vec<SolidPoint> {
    let mut v = Vec::new();
    let s = 0.35;
    for i in -57..=57 {
        for j in -57..=57 {
            let (x, y) = (i as f32 * s, j as f32 * s);
            let r = (x * x + y * y).sqrt();
            if r > 20.0 || (r >= gap.0 && r < gap.1) {
                continue;
            }
            let e = 0.01;
            let n = Vec3::new(-(surface(x + e, y) - surface(x - e, y)) / (2.0 * e), -(surface(x, y + e) - surface(x, y - e)) / (2.0 * e), 1.0).normalize();
            let lift = 0.4 * ((5.5 - r) / 0.5).clamp(0.0, 1.0);
            v.push(SolidPoint { position: Vec3::new(x, y, surface(x, y) + lift), normal: n, scan: 1 });
        }
    }
    v
}

/// Worst height of the samples within `r` of the origin above `surface`.
fn worst_off(samples: &[Sample], surface: impl Fn(f32, f32) -> f32, r: f32) -> f32 {
    samples.iter().filter(|p| p.position.x.hypot(p.position.y) < r).map(|p| (p.position.z - surface(p.position.x, p.position.y)).abs()).fold(0.0, f32::max)
}

#[test]
fn marker_cut_grows_over_an_empty_ring() {
    // Revo leaves an empty ring 6-9 mm round some stickers: the cut grows over it and the patch
    // closes it on the sheet, so no ring of dimples is left for the solve or the mesher to bridge.
    let dome = |x: f32, y: f32| 0.004 * (x * x + y * y);
    let pts = sheet_with_sticker(dome, (6.0, 9.0));
    let m = Marker { id: "G".into(), center: Vec3::ZERO, normal: Vec3::Z, scan: None, source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    let out = remove_markers(&pts, std::slice::from_ref(&m), &MarkerConfig::default(), None);
    assert!(out.outcomes[0].fill.starts_with("quadric"), "{:?}", out.outcomes[0]);
    let rc = markers::cut_radius(&out.sites[0], &MarkerConfig::default());
    assert!((9.0..=10.5).contains(&rc), "cut radius {rc}");
    assert!(worst_off(&out.samples, dome, 12.0) < 0.03, "{} mm off the dome", worst_off(&out.samples, dome, 12.0));
    // The ring is filled: every 0.5 mm of the ring has a sample.
    for k in 0..360 {
        let t = k as f32 / 360.0 * std::f32::consts::TAU;
        for r in [6.5f32, 7.5, 8.5] {
            let c = Vec3::new(r * t.cos(), r * t.sin(), 0.0);
            assert!(out.samples.iter().any(|p| (p.position - c).truncate().length() < 0.5), "no sample at r {r}, {k} deg");
        }
    }
    // Switched off, the cut stays at 6.5 mm and the ring stays empty.
    let off = remove_markers(&pts, &[m], &MarkerConfig { widen_max_mm: 0.0, ..MarkerConfig::default() }, None);
    assert!(markers::cut_radius(&off.sites[0], &MarkerConfig::default()) < 7.0);
}

#[test]
fn marker_site_on_a_sheet_of_changing_curvature_is_filled_flush() {
    // A sheet whose curvature changes across the site (z = c x^3 + gentle dome): the quadric fitted
    // to the annulus misses the centre by ~0.1 mm, the cubic fit does not.
    let sheet = |x: f32, y: f32| 0.0005 * x * x * x + 0.003 * (x * x + y * y);
    let pts = sheet_with_sticker(sheet, (0.0, 0.0));
    let m = Marker { id: "C".into(), center: Vec3::ZERO, normal: Vec3::Z, scan: None, source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    let out = remove_markers(&pts, &[m], &MarkerConfig::default(), None);
    assert!(out.outcomes[0].fill.starts_with("quadric"), "{:?}", out.outcomes[0]);
    let off = worst_off(&out.samples, sheet, 7.0);
    assert!(off < 0.03, "patch {off} mm off the sheet");
    // What a quadric alone leaves (the reason for the cubic).
    let s: Vec<(f64, f64, f64)> = pts.iter().map(|p| p.position).filter(|p| (6.5..12.5).contains(&p.x.hypot(p.y))).map(|p| (p.x as f64, p.y as f64, p.z as f64)).collect();
    let (q, _, _) = Quadric::fit_robust(&s).unwrap();
    let quad_off = (-60..=60).map(|k| k as f64 * 0.1).map(|x| (q.eval(x, 0.0) - sheet(x as f32, 0.0) as f64).abs()).fold(0.0, f64::max);
    assert!(quad_off > 0.08, "the quadric alone would do: {quad_off}");
}

/// Whether `p`'s direction from the centre lies within `r` mm (on the outer sphere) of `d`, or in
/// the slot `|u| < half_len`, `|v| < half_w` round `d` when `half_len > 0`.
fn in_gap(p: Vec3, d: Vec3, r: f32, half_len: f32, half_w: f32) -> bool {
    let u = p.normalize();
    if half_len > 0.0 {
        let (eu, ev) = markers::tangent_frame(d);
        u.dot(d) > 0.0 && (u.dot(eu) * R_OUT).abs() < half_len && (u.dot(ev) * R_OUT).abs() < half_w
    } else {
        u.angle_between(d) * R_OUT < r
    }
}

fn holes_in(pts: &[SolidPoint]) -> HoleFilling {
    let samples: Vec<Sample> =
        pts.iter().map(|p| Sample { position: p.position, normal: p.normal, scan: p.scan, kind: PointKind::Measured }).collect();
    fill_holes(&samples, &HoleConfig::default())
}

/// Fill points within `r` of `c` (projected onto the outer sphere).
fn fill_near(fill: &[Sample], c: Vec3, r: f32) -> usize {
    fill.iter().filter(|p| (p.position.normalize() * R_OUT).distance(c) < r).count()
}

#[test]
fn gap_in_a_one_sided_face_is_filled_not_kept_as_a_bore() {
    // Sticker-sized and dropout-sized gaps in a face scanned from one side only: nothing measured
    // lies behind them, so their axis is clear like a screw hole's, but nothing shows the plate's
    // other face round them either. They are filled.
    let (a, b, c) = (dir(30.0, 0.0), dir(35.0, 120.0), dir(40.0, 240.0));
    let pts: Vec<SolidPoint> = cap(R_OUT, 0.45, true, 1)
        .into_iter()
        .filter(|p| !in_gap(p.position, a, 2.0, 0.0, 0.0) && !in_gap(p.position, b, 2.75, 0.0, 0.0) && !in_gap(p.position, c, 4.5, 0.0, 0.0))
        .collect();
    let res = holes_in(&pts);
    for (d, r) in [(a, 2.0), (b, 2.75), (c, 4.5)] {
        let n = fill_near(&res.fill, d * R_OUT, r - 0.7);
        assert!(n > 10, "gap of r {r} mm left open ({n} fill points in it)");
    }
}

#[test]
fn round_hole_through_both_faces_stays_open_but_a_slot_is_filled() {
    // Both faces scanned: a round hole through both (a screw hole) stays open; an elongated gap
    // through both (a dropout along a laser line) is not round and is filled.
    let (hole, slot) = (dir(30.0, 0.0), dir(35.0, 180.0));
    let gap = |p: &SolidPoint| in_gap(p.position, hole, 2.5, 0.0, 0.0) || in_gap(p.position, slot, 0.0, 7.0, 2.0);
    let mut pts: Vec<SolidPoint> = cap(R_OUT, 0.45, true, 1).into_iter().filter(|p| !gap(p)).collect();
    pts.extend(cap(R_OUT - WALL, 0.45, false, 2).into_iter().filter(|p| !gap(p)));
    let res = holes_in(&pts);
    assert_eq!(fill_near(&res.fill, hole * R_OUT, 2.0), 0, "the round through hole was filled");
    let n = fill_near(&res.fill, slot * R_OUT, 1.0);
    assert!(n > 10, "the slot was left open ({n} fill points in it)");
    // The same hole in one face only (the other face scanned behind it) is filled.
    let mut pts: Vec<SolidPoint> = cap(R_OUT, 0.45, true, 1).into_iter().filter(|p| !in_gap(p.position, hole, 2.5, 0.0, 0.0)).collect();
    pts.extend(cap(R_OUT - WALL, 0.45, false, 2));
    assert!(fill_near(&holes_in(&pts).fill, hole * R_OUT, 1.5) > 10, "a gap in one face was kept open");
}

#[test]
fn dropout_in_a_thin_one_sided_wall_gives_no_tunnel() {
    // A 1.8 mm wall scanned from outside only, with a 4 mm dropout: the solid is closed there and
    // follows the sphere (a bore kept open here was a tunnel through the wall).
    let gap_dir = dir(35.0, 60.0);
    let pts: Vec<SolidPoint> = cap(R_OUT, 0.45, true, 1).into_iter().filter(|p| !in_gap(p.position, gap_dir, 2.0, 0.0, 0.0)).collect();
    let res = build_solid(&pts, &[], &SolidConfig { detect: None, wall_mm: Some(1.8), ..test_params() });
    let t = &res.report.topology;
    assert!(t.watertight && t.manifold, "{t:?}");
    assert_eq!(t.genus_total(), 0, "tunnel through the wall: {:?}", t.components);
    let e = radial_errors(&res.mesh, |p| p.length() > R_OUT - 0.9 && p.normalize().angle_between(gap_dir) * R_OUT < 3.0, R_OUT);
    assert!(!e.is_empty(), "no outer surface over the dropout");
    eprintln!("thin one-sided dropout: outer max {:.4}", e.last().unwrap());
    assert!(*e.last().unwrap() < 0.15, "outer face over the dropout off by {}", e.last().unwrap());
}

#[test]
fn thin_wall_seen_from_both_sides_is_no_laser_stray() {
    // A plate 0.8-1.5 mm thick seen from both sides by one scan, one face sampled more sparsely
    // (a grazing view): its faces are 0.4-1.6 mm apart with nothing between, like a stray layer
    // behind a surface, but the sparser face is a surface of its own and stays.
    for (t, dense, sparse) in [(1.2f32, 0.3f32, 0.37f32), (0.8, 0.3, 0.4), (1.5, 0.25, 0.32), (1.2, 0.3, 0.45)] {
        let mut pts = Vec::new();
        let mut face = |s: f32, z: f32, n: Vec3, off: f32| {
            let m = (20.0 / s) as i32;
            for i in 0..=m {
                for j in 0..=m {
                    pts.push(WorkPoint { p: Vec3::new(i as f32 * s + off, j as f32 * s + off, z), n, scan: 1, kind: PointKind::Measured, area: 0.0 });
                }
            }
        };
        face(dense, 0.0, Vec3::Z, 0.0);
        face(sparse, -t, -Vec3::Z, 0.1);
        let n = pts.len();
        let dropped = prep::drop_behind_strays(&mut pts, 2.0, 0.4, 1.6);
        assert!(dropped * 100 <= n, "plate {t} mm ({dense}/{sparse} mm spacing): {dropped} of {n} points dropped as strays");
    }
}

#[test]
fn marker_near_a_rounded_rim_leaves_the_rolled_edge() {
    // A flat top (z = 0, x < 0) rolling over a 2 mm radius edge into a side face (x = 2), and a
    // sticker (r 4.5 mm, 0.4 mm proud) 3 mm in from the edge. The site's cut reaches over the
    // rolled edge; the patch must not stand above it, and the rolled edge stays.
    let true_z = |x: f32| if x <= 0.0 { 0.0 } else { -2.0 + (4.0 - x.min(2.0).powi(2)).max(0.0).sqrt() };
    let s = 0.35;
    let mut pts = Vec::new();
    for j in -57..=57 {
        let y = j as f32 * s;
        for i in 0..=71 {
            let x = -(i as f32) * s;
            let lift = if (x + 3.0).hypot(y) < 4.5 { 0.4 } else { 0.0 };
            pts.push(SolidPoint { position: Vec3::new(x, y, lift), normal: Vec3::Z, scan: 1 });
        }
        for k in 1..=17 {
            let phi = k as f32 / 18.0 * std::f32::consts::FRAC_PI_2;
            let n = Vec3::new(phi.sin(), 0.0, phi.cos());
            pts.push(SolidPoint { position: Vec3::new(0.0, y, -2.0) + n * 2.0, normal: n, scan: 1 });
        }
        for k in 0..=23 {
            pts.push(SolidPoint { position: Vec3::new(2.0, y, -2.0 - k as f32 * s), normal: Vec3::X, scan: 1 });
        }
    }
    let m = Marker { id: "R".into(), center: Vec3::new(-3.0, 0.0, 0.0), normal: Vec3::Z, scan: None, source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    let out = remove_markers(&pts, &[m], &MarkerConfig { widen_max_mm: 0.0, ..MarkerConfig::default() }, None);
    let o = &out.outcomes[0];
    assert!(o.fill.starts_with("quadric") && o.sectors_covered < 8 && o.fill_points > 100, "{o:?}");
    for p in out.samples.iter().filter(|p| p.kind == PointKind::MarkerFill) {
        let above = p.position.z - true_z(p.position.x);
        assert!(p.position.x < 2.05 && above < 0.35, "patch point {:.2} mm above the surface at x {:.2}", above, p.position.x);
    }
    let roll = |v: &[Vec3]| v.iter().filter(|p| p.x > 0.0 && p.z < -0.5 && p.z > -2.0 && p.y.abs() < 5.0).count();
    let before = roll(&pts.iter().map(|p| p.position).collect::<Vec<_>>());
    let after = roll(&out.samples.iter().filter(|p| p.kind == PointKind::Measured).map(|p| p.position).collect::<Vec<_>>());
    assert_eq!(after, before, "rolled edge cut");
    let raised = out.samples.iter().filter(|p| p.kind == PointKind::Measured && p.position.z > 0.2).count();
    assert_eq!(raised, 0, "sticker points left");
}
