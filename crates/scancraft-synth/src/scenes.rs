//! The scene catalogue. Every scene is built from a few numbers, so it is the same on every
//! machine; [`Options::spacing_scale`] makes coarser (faster) versions for tests.

use glam::{DVec2, DVec3};
use serde_json::json;

use crate::rng::mix;
use crate::scan::march;
use crate::scene::*;
use crate::sdf::*;

#[derive(Debug, Clone)]
pub struct Options {
    /// Multiplies every pass's point spacing and the reference mesh cell (1 = the full set).
    pub spacing_scale: f64,
}

impl Default for Options {
    fn default() -> Self {
        Options { spacing_scale: 1.0 }
    }
}

/// Names of all scenes, in catalogue order.
pub fn names() -> Vec<&'static str> {
    BUILDERS.iter().map(|(n, _)| *n).collect()
}

/// Builds the scene `name`.
pub fn build(name: &str, options: &Options) -> Option<Scene> {
    BUILDERS.iter().find(|(n, _)| *n == name).map(|(n, f)| finish(f(n), options))
}

/// Builds every scene whose name contains `filter` (all for an empty filter).
pub fn build_matching(filter: &str, options: &Options) -> Vec<Scene> {
    BUILDERS.iter().filter(|(n, _)| n.contains(filter)).map(|(n, f)| finish(f(n), options)).collect()
}

type Builder = fn(&str) -> Scene;

const BUILDERS: &[(&str, Builder)] = &[
    ("flat_plain", flat_plain),
    ("flat_markers", flat_markers),
    ("flat_marker_holes", flat_marker_holes),
    ("flat_vents", flat_vents),
    ("perforated_grille", perforated_grille),
    ("flat_two_pass_offset", flat_two_pass_offset),
    ("flat_two_pass_tilt", flat_two_pass_tilt),
    ("flat_laser_strays", flat_laser_strays),
    ("dome_r60_markers", dome_r60_markers),
    ("dome_r60_marker_holes", dome_r60_marker_holes),
    ("dome_r60_noise_002", |n| dome_noise(n, 0.02)),
    ("dome_r60_noise_005", |n| dome_noise(n, 0.05)),
    ("dome_r60_noise_010", |n| dome_noise(n, 0.10)),
    ("dome_density_steps", dome_density_steps),
    ("ball_r15_markers", |n| ball_r15(n, false)),
    ("ball_r15_marker_holes", |n| ball_r15(n, true)),
    ("cylinder_r15_markers", |n| cylinder_r15(n, false)),
    ("cylinder_r15_marker_holes", |n| cylinder_r15(n, true)),
    ("cylinder_r65_markers", |n| cylinder_r65(n, false)),
    ("cylinder_r65_marker_holes", |n| cylinder_r65(n, true)),
    ("saddle_markers", |n| saddle(n, false)),
    ("saddle_marker_holes", |n| saddle(n, true)),
    ("grazing_cylinder_r30", grazing_cylinder_r30),
    ("shell_two_pass", shell_two_pass),
    ("shell_on_rim", shell_on_rim),
    ("cup_rim_partial", cup_rim_partial),
    ("box_edges", box_edges),
    ("relief_fine", relief_fine),
    ("plate_screws_bosses_ribs", plate_screws_bosses_ribs),
    ("thin_fins", thin_fins),
];

// ---------------------------------------------------------------------------------------------
// Helpers

fn d3(x: f64, y: f64, z: f64) -> DVec3 {
    DVec3::new(x, y, z)
}

fn seed_of(name: &str) -> u64 {
    mix(&name.bytes().map(u64::from).collect::<Vec<_>>())
}

fn new_scene(name: &str, description: &str, part: Shape, bounds: [V3; 2], tags: &[&str]) -> Scene {
    Scene {
        name: name.into(),
        description: description.into(),
        seed: seed_of(name),
        part,
        bounds,
        stickers: Vec::new(),
        passes: Vec::new(),
        features: Vec::new(),
        truth_cell_mm: 0.4,
        tags: tags.iter().map(|t| t.to_string()).collect(),
    }
}

/// First surface point of `shape` along the ray from `from` in `dir`.
pub fn hit(shape: &Shape, from: DVec3, dir: DVec3) -> DVec3 {
    let dir = dir.normalize();
    let f = |p: DVec3| shape.sdf(p);
    let t = march(&f, from, dir, 0.0, 2000.0).unwrap_or_else(|| panic!("no surface along {from} {dir}"));
    from + dir * t
}

fn from_above(shape: &Shape, x: f64, y: f64) -> DVec3 {
    hit(shape, d3(x, y, 400.0), -DVec3::Z)
}

fn from_below(shape: &Shape, x: f64, y: f64) -> DVec3 {
    hit(shape, d3(x, y, -400.0), DVec3::Z)
}

/// A sticker where the ray from `from` along `dir` meets the part.
fn sticker(part: &Shape, id: &str, from: DVec3, dir: DVec3) -> Sticker {
    let p = hit(part, from, dir);
    Sticker::standard(id, p, part.normal(p))
}

/// A sticker on the surface seen straight from above at (x, y).
fn sticker_down(part: &Shape, id: &str, x: f64, y: f64) -> Sticker {
    sticker(part, id, d3(x, y, 400.0), -DVec3::Z)
}

/// A sticker on the surface seen from `dir` (pointing away from the surface) through `towards`.
fn sticker_facing(part: &Shape, id: &str, towards: DVec3, out: DVec3) -> Sticker {
    sticker(part, id, towards + out.normalize() * 60.0, -out)
}

/// Eight views on a ring 48° up plus one from straight above, all at about 225 mm.
fn above(center: DVec3) -> Vec<View> {
    let mut v = ring_views(center, 150.0, 168.0, 8, 22.5);
    v.push(view(center + d3(8.0, 5.0, 228.0), center));
    v
}

/// The same from below (for a pass of the part turned over, in part frame).
fn below(center: DVec3) -> Vec<View> {
    let mut v = ring_views(center, 150.0, -168.0, 8, 22.5);
    v.push(view(center + d3(8.0, 5.0, -228.0), center));
    v
}

/// Low ring (25° up) plus a higher ring of four.
fn around(center: DVec3) -> Vec<View> {
    let mut v = ring_views(center, 200.0, 95.0, 8, 0.0);
    v.extend(ring_views(center, 120.0, 190.0, 4, 45.0));
    v
}

/// Part frame -> frame of a pass of the part turned over onto its face at z = `top`, then turned
/// by `turn_deg` and moved by (`dx`, `dy`) on the table.
fn turned_over(top: f64, turn_deg: f64, dx: f64, dy: f64) -> Rigid {
    Rigid::axis_angle([0.0, 0.0, 1.0], turn_deg, [dx, dy, top]).after(&Rigid::axis_angle([1.0, 0.0, 0.0], 180.0, [0.0; 3]))
}

fn plate(half_x: f64, half_y: f64, thickness: f64) -> Shape {
    Shape::Box { center: [0.0, 0.0, -thickness / 2.0], half: [half_x, half_y, thickness / 2.0], round: 0.0 }
}

fn prism(profile: Profile2, z0: f64, z1: f64) -> Shape {
    Shape::Prism { profile, z0, z1 }
}

fn intersect(parts: Vec<Shape>) -> Shape {
    Shape::Intersect { parts }
}

fn above_z(z: f64) -> Shape {
    Shape::HalfSpace { normal: [0.0, 0.0, -1.0], offset: -z }
}

/// The "marker holes" variant of a scene: the scan as Revo's Fused export gives it. Every
/// sticker site is cut out to exactly 5.5 mm (the export's holes; no ring round them), the passes
/// are denser and smoother ([`PassSpec::fused_export`]) and have no table (the export holds the
/// object only).
fn fused_export(scene: &mut Scene) {
    for s in &mut scene.stickers {
        s.cut_radius = Some(5.5);
    }
    scene.stickers.retain(|s| s.on_table_of_pass.is_none());
    for p in &mut scene.passes {
        *p = p.clone().fused_export();
        p.table = None;
    }
}

/// How the stickers of a raw scan (`raw_preview.ply`) come out: the dot measured, the black ring
/// thin (S1) or empty (S3), some with an empty halo out to 9-12 mm (S2).
fn raw_rings(stickers: &mut [Sticker]) {
    let styles = [RingStyle::Thin, RingStyle::Empty, RingStyle::Thin, RingStyle::Halo];
    for (k, st) in stickers.iter_mut().enumerate() {
        *st = st.clone().styled(styles[k % styles.len()], k);
    }
}

/// A table under pass `pass` with what stands on a real one: a marker block with two stickers
/// on its top, a can and a small box, and four stickers on the table itself. `top` is the
/// table's surface (part frame), `up` (+1 or -1 along z) points from the table to the part; the
/// part touches it where it rests on it. The clutter stands beside the part's `bounds`, within
/// the margin the scanner sees round a table.
fn table_with_clutter(scene: &mut Scene, pass: usize, top: f64, up: f64) {
    let (lo, hi) = (v3(scene.bounds[0]), v3(scene.bounds[1]));
    let c = 0.5 * (lo + hi);
    let m = crate::scan::TABLE_MARGIN_MM;
    let half = [0.5 * (hi.x - lo.x) + m + 2.0, 0.5 * (hi.y - lo.y) + m + 2.0, 2.5];
    let slab = Shape::Box { center: [c.x, c.y, top - up * 2.5], half, round: 0.0 };
    let on = |h: f64| top + up * h;
    let block_c = d3(hi.x + 16.8, lo.y + 12.0, on(6.0));
    let block = Shape::Box { center: block_c.to_array(), half: [11.0, 7.0, 6.0], round: 0.5 };
    let (z0, z1) = if up > 0.0 { (top - 1.0, on(22.0)) } else { (on(22.0), top + 1.0) };
    let can = Shape::Cylinder { a: [lo.x - 15.0, hi.y - 12.0, z0], b: [lo.x - 15.0, hi.y - 12.0, z1], radius: 9.0 };
    let small = Shape::Box { center: [lo.x - 12.0, lo.y - 13.0, on(3.0)], half: [6.0, 4.0, 3.0], round: 0.3 };
    let clutter = vec![slab, block, can, small];
    let table = Shape::Union { parts: clutter };
    scene.passes[pass].table = Some(table);
    let n = d3(0.0, 0.0, up);
    let mut k = 0;
    let mut add = |at: DVec3, scene: &mut Scene| {
        k += 1;
        let mut st = Sticker::standard(format!("T{}-{}", pass + 1, k), at, n).styled(RingStyle::Thin, k);
        st.on_table_of_pass = Some(pass);
        scene.stickers.push(st);
    };
    for dx in [-5.6, 5.6] {
        add(d3(block_c.x + dx, block_c.y, on(12.0)), scene);
    }
    for (x, y) in [(lo.x - 16.0, c.y - 6.0), (hi.x + 16.0, hi.y + 4.0), (c.x + 8.0, lo.y - 18.0), (c.x - 10.0, hi.y + 18.0)] {
        add(d3(x, y, top), scene);
    }
}

/// Rotation by `degrees` about `axis` through `centre`, then `shift` (part frame): an
/// alignment's residual error.
fn tilt(axis: [f64; 3], degrees: f64, centre: DVec3, shift: [f64; 3]) -> Rigid {
    let r = Rigid::axis_angle(axis, degrees, [0.0; 3]);
    let t = centre - r.apply(centre) + v3(shift);
    Rigid::axis_angle(axis, degrees, t.to_array())
}

/// A point on the true surface near `near` (projected), with the outward normal.
fn surface_point(part: &Shape, near: DVec3) -> (DVec3, DVec3) {
    let p = project_to_surface(|q| part.sdf(q), near);
    (p, part.normal(p))
}

fn height_probe(part: &Shape, near: DVec3, tol: f64, note: &str) -> Probe {
    let (p, n) = surface_point(part, near);
    Probe::height(p, n, 0.0, tol, note)
}

/// Marker-site features: the true surface under every sticker on the part (centre and rings at
/// 2.5 and 4.8 mm) must be met by the mesh.
fn marker_features(scene: &mut Scene) {
    let mut features = Vec::new();
    for s in scene.stickers.iter().filter(|s| s.on_table_of_pass.is_none()) {
        let c = v3(s.center);
        let n = v3(s.normal);
        let (u, w) = n.any_orthonormal_pair();
        let mut probes = vec![Probe::height(c, n, 0.0, 0.05, "site centre on the true surface")];
        for r in [2.5, 4.8] {
            for (a, b) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)] {
                let guess = c + (u * a + w * b) * r;
                let (p, pn) = surface_point(&scene.part, guess);
                if (p - guess).length() < 1.5 && pn.dot(n) > 0.7 {
                    probes.push(Probe::height(p, pn, 0.0, 0.05, format!("r {r} mm")));
                }
            }
        }
        let partial = probes.len() < 9;
        let (kind, expect) = match s.cut_radius {
            None => (
                "marker_site",
                "The sticker (0.4 mm proud, 11 mm across; its dot measured, its black ring thin or empty, perhaps an empty \
                 halo round it) is removed and the site is flush with the true surface (within 0.05 mm, no bump, no flat spot \
                 or dent on a curved surface, no ring at the disc edge, nothing left open).",
            ),
            Some(_) => (
                "marker_hole",
                "The cut-out of Revo's Fused export (no points within 5.5 mm) is closed and the fill continues the true \
                 surface (within 0.05 mm, follows the curvature, no flat spot, no dent, no ring).",
            ),
        };
        features.push(Feature {
            id: s.id.clone(),
            kind: kind.into(),
            expect: if partial { format!("{expect} Partial site: the disc reaches an edge or rim, which must stay where it is.") } else { expect.into() },
            params: json!({"center": s.center, "normal": s.normal, "cut_radius_mm": s.cut_radius, "site_radius_mm": s.site_radius(), "partial": partial,
                "scan": {"dot_keep": s.dot_keep, "ring_keep": s.ring_keep, "ring_radius_mm": s.ring_radius, "halo_radius_mm": s.halo_radius, "halo_keep": s.halo_keep}}),
            probes,
        });
    }
    scene.features.splice(0..0, features);
}

/// Validates and scales a built scene, and adds the marker features.
fn finish(mut scene: Scene, options: &Options) -> Scene {
    marker_features(&mut scene);
    let s = options.spacing_scale;
    if s != 1.0 {
        scene.truth_cell_mm *= s;
        for p in &mut scene.passes {
            p.spacing_mm *= s;
            for r in &mut p.density {
                r.spacing_mm *= s;
            }
        }
    }
    scene
}

fn plane_probes(part: &Shape, points: &[(f64, f64)], tol: f64) -> Vec<Probe> {
    points.iter().map(|&(x, y)| height_probe(part, from_above(part, x, y), tol, "flat top")).collect()
}

fn feature(id: &str, kind: &str, expect: &str, params: serde_json::Value, probes: Vec<Probe>) -> Feature {
    Feature { id: id.into(), kind: kind.into(), expect: expect.into(), params, probes }
}

// ---------------------------------------------------------------------------------------------
// Flat plates

const PLATE_BOUNDS: [V3; 2] = [[-50.0, -40.0, -4.0], [50.0, 40.0, 0.0]];

fn flat_base(name: &str, description: &str, tags: &[&str]) -> Scene {
    let part = plate(50.0, 40.0, 4.0);
    let mut scene = new_scene(name, description, part, PLATE_BOUNDS, tags);
    scene.passes.push(PassSpec::new("top", above(DVec3::ZERO)));
    let grid: Vec<(f64, f64)> = [-35.0, 0.0, 35.0].iter().flat_map(|&x| [-28.0, 0.0, 28.0].map(|y| (x + 3.0, y - 2.0))).collect();
    let mut probes = plane_probes(&scene.part, &grid, 0.03);
    for p in [d3(50.0, 10.0, 0.0), d3(-50.0, -10.0, 0.0), d3(15.0, 40.0, 0.0), d3(50.0, 40.0, 0.0)] {
        probes.push(Probe::on(p, 0.15, "top edge / corner, sharp"));
    }
    scene.features.push(feature(
        "plane",
        "flat",
        "One flat sheet: no waviness or hatching (relief std below 0.01 mm, p99 below 0.03 mm), the top edges sharp (within 0.15 mm of the corner line).",
        json!({"size_mm": [100, 80, 4]}),
        probes,
    ));
    scene
}

fn plate_stickers(part: &Shape) -> Vec<Sticker> {
    let mut v: Vec<Sticker> = [(-30.0, -22.0), (30.0, -22.0), (-30.0, 22.0), (28.0, 20.0), (0.0, 0.0), (0.0, -28.0), (-12.0, 30.0)]
        .iter()
        .enumerate()
        .map(|(k, &(x, y))| sticker_down(part, &format!("M{}", k + 1), x, y))
        .collect();
    // Its disc ends 0.4 mm before the plate's edge.
    v.push(sticker_down(part, "M8-edge", 50.0 - 5.6 - 0.4, 2.0));
    v
}

fn flat_plain(name: &str) -> Scene {
    flat_base(name, "Plain plate 100 x 80 x 4 mm, top pass only, noise 0.03 mm. The simplest case: a mesher must give a flat sheet with sharp edges.", &["flat"])
}

fn flat_markers(name: &str) -> Scene {
    let mut s = flat_base(
        name,
        "Plate 100 x 80 x 4 mm with 8 raised stickers (11 mm, 0.4 mm proud), one ending 0.4 mm before the edge; the rings scan thin or empty, two with an empty halo. On a table with a marker block, a can and a box.",
        &["flat", "markers", "table"],
    );
    s.stickers = plate_stickers(&s.part);
    raw_rings(&mut s.stickers);
    table_with_clutter(&mut s, 0, -4.0, 1.0);
    s
}

fn flat_marker_holes(name: &str) -> Scene {
    let mut s = flat_base(name, "Plate 100 x 80 x 4 mm as Revo's Fused export gives it: the 8 sticker sites cut out to 5.5 mm, 0.23 mm spacing, no table.", &["flat", "marker_holes", "fused_export"]);
    s.stickers = plate_stickers(&s.part);
    fused_export(&mut s);
    s
}

fn flat_two_pass_offset(name: &str) -> Scene {
    let mut s = flat_base(
        name,
        "Plate 100 x 80 x 4 mm with stickers, two top passes that overlap over 40 mm (x -20..20); pass 2 sits 0.15 mm too deep and is in its own frame (turned 70°).",
        &["flat", "markers", "multi_pass", "offset"],
    );
    s.stickers = plate_stickers(&s.part);
    raw_rings(&mut s.stickers);
    s.passes[0].clip = vec![Region::HalfSpace { normal: [1.0, 0.0, 0.0], offset: 20.0 }];
    let mut p2 = PassSpec::new("top-right", above(d3(15.0, 0.0, 0.0)));
    p2.clip = vec![Region::HalfSpace { normal: [-1.0, 0.0, 0.0], offset: 20.0 }];
    p2.depth_bias_mm = 0.15;
    p2.scan_pose = Rigid::axis_angle([0.0, 0.0, 1.0], 70.0, [12.0, -30.0, 0.0]);
    s.passes.push(p2);
    let probes = plane_probes(&s.part, &[(-19.0, 5.0), (-10.0, -15.0), (0.0, 12.0), (10.0, -5.0), (19.0, 14.0)], 0.05);
    s.features.push(feature(
        "overlap",
        "pass_offset",
        "Pass 2 is 0.15 mm too deep. After alignment and merging the overlap is one sheet: no step or double layer at x = -20 or x = 20, the sheet within 0.05 mm of the truth.",
        json!({"overlap_x_mm": [-20, 20], "depth_bias_mm": 0.15}),
        probes,
    ));
    s
}

/// Two overlapping top passes; pass 2 is placed with an alignment's residual error (0.25° about
/// the y axis through the plate's middle and 0.1 mm up), so the gap between the passes changes
/// across the overlap from 0.01 to 0.19 mm.
fn flat_two_pass_tilt(name: &str) -> Scene {
    let mut s = flat_base(
        name,
        "Plate 100 x 80 x 4 mm with stickers, two top passes that overlap over 40 mm (x -20..20); pass 2 is placed with an alignment's residual error (turned 0.25° about the y axis through the middle, raised 0.1 mm: the gap grows from 0.01 to 0.19 mm across the overlap) and is in its own frame (turned 70°).",
        &["flat", "markers", "multi_pass", "offset", "tilt"],
    );
    s.stickers = plate_stickers(&s.part);
    raw_rings(&mut s.stickers);
    s.passes[0].clip = vec![Region::HalfSpace { normal: [1.0, 0.0, 0.0], offset: 20.0 }];
    let mut p2 = PassSpec::new("top-right", above(d3(15.0, 0.0, 0.0)));
    p2.clip = vec![Region::HalfSpace { normal: [-1.0, 0.0, 0.0], offset: 20.0 }];
    p2.pose_error = tilt([0.0, 1.0, 0.0], -0.25, DVec3::ZERO, [0.0, 0.0, 0.1]);
    p2.scan_pose = Rigid::axis_angle([0.0, 0.0, 1.0], 70.0, [12.0, -30.0, 0.0]);
    s.passes.push(p2);
    let probes = plane_probes(&s.part, &[(-19.0, 5.0), (-10.0, -15.0), (0.0, 12.0), (10.0, -5.0), (19.0, 14.0)], 0.05);
    s.features.push(feature(
        "overlap",
        "pass_offset",
        "Pass 2 is tilted against pass 1 by 0.25° (gap 0.01 mm at x = -20, 0.19 mm at x = 20). The overlap is one sheet: no step, crease or double layer, within 0.05 mm of pass 1 (the reference) where both overlap; the tilt is not a step at x = 20.",
        json!({"overlap_x_mm": [-20, 20], "tilt_deg": 0.25, "tilt_axis": [0, 1, 0], "shift_mm": [0, 0, 0.1]}),
        probes,
    ));
    s
}

fn flat_laser_strays(name: &str) -> Scene {
    let mut s = flat_base(
        name,
        "Plate 100 x 80 x 4 mm with stickers; inside a 30 mm window two families of cross-line laser strays (70° apart, 2 mm wide lines, 9 and 11 mm apart) lie 0.6-1.0 mm behind the top, most with normals facing the wrong way.",
        &["flat", "markers", "laser_strays", "table"],
    );
    s.stickers = plate_stickers(&s.part).into_iter().filter(|m| m.id != "M5").collect();
    raw_rings(&mut s.stickers);
    s.passes[0].strays = Some(strays_window(d3(0.0, 0.0, 0.0), 30.0));
    table_with_clutter(&mut s, 0, -4.0, 1.0);
    let mut probes = Vec::new();
    for (x, y) in [(-15.0, 5.0), (-5.0, -10.0), (4.0, 7.0), (12.0, -3.0), (0.0, 18.0)] {
        probes.push(height_probe(&s.part, d3(x, y, 0.0), 0.03, "top in the stray window"));
        probes.push(Probe::clear(d3(x, y, -0.8), 0.45, "no surface where the strays are (0.35-1.25 mm below the top)"));
    }
    s.features.push(feature(
        "strays",
        "laser_strays",
        "The strays are dropped: the top stays flat in the window (no pits or lattice, relief std below 0.01 mm), no surface inside the plate.",
        json!({"window_center": [0, 0, 0], "window_radius_mm": 30, "depth_mm": [0.6, 1.0]}),
        probes,
    ));
    s
}

/// Streaks along two families of laser lines in the grazing fringe (past 60°, full at 75°).
fn grazing_streaks() -> crate::scene::Streaks {
    let n1 = DVec2::from_angle(20f64.to_radians());
    let n2 = DVec2::from_angle(90f64.to_radians());
    crate::scene::Streaks {
        families: vec![
            LineFamily { normal: [n1.x, n1.y, 0.3], spacing_mm: 9.0, phase: 0.13 },
            LineFamily { normal: [n2.x, n2.y, 0.3], spacing_mm: 11.0, phase: 0.41 },
        ],
        width_mm: 2.0,
        amplitude_mm: 0.5,
        from_incidence_deg: 60.0,
        full_incidence_deg: 75.0,
        cell_mm: 4.0,
    }
}

/// Two families of lines 70° apart in a window round `center`.
fn strays_window(center: DVec3, radius: f64) -> Strays {
    let n1 = DVec2::from_angle(20f64.to_radians());
    let n2 = DVec2::from_angle(90f64.to_radians());
    Strays {
        region: Region::Sphere { center: center.to_array(), radius },
        families: vec![
            LineFamily { normal: [n1.x, n1.y, 0.0], spacing_mm: 9.0, phase: 0.13 },
            LineFamily { normal: [n2.x, n2.y, 0.0], spacing_mm: 11.0, phase: 0.41 },
        ],
        width_mm: 2.0,
        depth_mm: [0.6, 1.0],
        fraction: 0.6,
        wrong_way: 0.83,
        noise_mm: 0.05,
    }
}

fn flat_vents(name: &str) -> Scene {
    // Plate 120 x 80 x 3, top at z = 0.
    let widths = [1.5, 2.0, 3.0, 4.0, 6.0];
    let xs = [-45.0, -30.0, -15.0, 0.0, 18.0];
    let (slot_y, slot_len) = (10.0, 30.0);
    let mut cuts = Vec::new();
    for (&w, &x) in widths.iter().zip(&xs) {
        let r = w / 2.0;
        cuts.push(prism(Profile2::Stadium { a: [x, slot_y - slot_len / 2.0 + r], b: [x, slot_y + slot_len / 2.0 - r], radius: r }, -10.0, 10.0));
    }
    let grooves = [(2.0, 1.0, -15.0), (1.0, 0.5, -22.0)];
    for &(w, depth, y) in &grooves {
        cuts.push(prism(Profile2::Stadium { a: [-50.0, y], b: [-10.0, y], radius: w / 2.0 }, -depth, 10.0));
    }
    let pocket = Profile2::RoundRect { center: [25.0, -20.0], half: [8.0, 5.0], radius: 2.0 };
    cuts.push(prism(pocket.clone(), -1.5, 10.0));
    let part = Shape::subtract(Shape::Box { center: [0.0, 0.0, -1.5], half: [60.0, 40.0, 1.5], round: 0.0 }, cuts);
    let mut s = new_scene(
        name,
        "Vent plate 120 x 80 x 3 mm: five through slots 1.5 / 2 / 3 / 4 / 6 mm wide (30 mm long, round ends), blind grooves 2 x 1 and 1 x 0.5 mm, a 16 x 10 x 1.5 mm pocket, stickers 1.5 mm from a slot. Top pass and a pass of the plate turned over (0.1 mm too deep, own frame), each lying on a table with clutter (the table shows through the slots).",
        part,
        [[-60.0, -40.0, -3.0], [60.0, 40.0, 0.0]],
        &["flat", "slots", "markers", "multi_pass", "table"],
    );
    s.passes.push(PassSpec::new("top", above(DVec3::ZERO)));
    let mut bottom = PassSpec::new("bottom", below(d3(0.0, 0.0, -3.0)));
    bottom.scan_pose = turned_over(0.0, 35.0, 20.0, 10.0);
    bottom.depth_bias_mm = 0.1;
    s.passes.push(bottom);
    s.stickers = vec![
        sticker_down(&s.part, "M1-near-slot6", 18.0 + 3.0 + 1.5 + 5.6, slot_y),
        sticker_down(&s.part, "M2-near-slot1.5-end", -45.0, slot_y + slot_len / 2.0 + 1.5 + 5.6),
        sticker_down(&s.part, "M3", -20.0, -32.0),
        sticker_down(&s.part, "M4", 46.0, -27.0),
    ];
    raw_rings(&mut s.stickers);
    // On a table, top up and then turned over: the table shows through every slot.
    table_with_clutter(&mut s, 0, -3.0, 1.0);
    table_with_clutter(&mut s, 1, 0.0, -1.0);
    for (&w, &x) in widths.iter().zip(&xs) {
        let r = w / 2.0;
        let mut probes = Vec::new();
        for dy in [-8.0, 0.0, 8.0] {
            probes.push(Probe::clear(d3(x, slot_y + dy, -1.5), r - 0.25, "slot open at mid-depth"));
            probes.push(Probe::clear(d3(x, slot_y + dy, -0.3), r - 0.25, "no lid just under the top"));
            probes.push(Probe::through(d3(x, slot_y + dy, -1.5), DVec3::Z, 1.8, "slot open through (no cap or bottom on the axis)"));
        }
        probes.push(Probe::clear(d3(x, slot_y + slot_len / 2.0 - r, -1.5), r - 0.25, "round end open"));
        for side in [-1.0, 1.0] {
            probes.push(height_probe(&s.part, d3(x + side * (r + 0.8), slot_y, 0.0), 0.05, "top 0.8 mm beside the slot"));
            probes.push(Probe::on(d3(x + side * r, slot_y, 0.0), 0.15, "slot edge, sharp"));
        }
        s.features.push(feature(
            &format!("slot-{w}"),
            "through_slot",
            &format!("Through slot {w} mm wide, 30 mm long with round ends: open all the way through (not bridged or capped), width {w} ± 0.2 mm, edges sharp, the plate flat right up to the edge."),
            json!({"width_mm": w, "length_mm": slot_len, "center": [x, slot_y], "axis": [0, 1], "through": true}),
            probes,
        ));
    }
    for &(w, depth, y) in &grooves {
        let mut probes = Vec::new();
        for x in [-40.0, -30.0, -20.0] {
            probes.push(Probe::height(d3(x, y, -depth), DVec3::Z, 0.0, 0.1, "groove floor"));
            probes.push(Probe::clear(d3(x, y, -depth / 2.0), (w / 2.0 - 0.2).min(depth / 2.0 - 0.05), "groove open"));
        }
        s.features.push(feature(
            &format!("groove-{w}x{depth}"),
            "blind_groove",
            &format!("Blind groove {w} mm wide and {depth} mm deep: kept open to its floor (floor within 0.1 mm), not filled or smoothed over."),
            json!({"width_mm": w, "depth_mm": depth, "from": [-50, y], "to": [-10, y]}),
            probes,
        ));
    }
    s.features.push(feature(
        "pocket",
        "blind_pocket",
        "Pocket 16 x 10 mm, 1.5 mm deep, corner radius 2: floor flat at -1.5 mm, walls and rim sharp.",
        json!({"profile": pocket, "depth_mm": 1.5}),
        vec![
            Probe::height(d3(25.0, -20.0, -1.5), DVec3::Z, 0.0, 0.08, "pocket floor"),
            Probe::height(d3(20.0, -18.0, -1.5), DVec3::Z, 0.0, 0.08, "pocket floor"),
            Probe::clear(d3(25.0, -20.0, -0.75), 0.6, "pocket open"),
            Probe::on(d3(33.0, -20.0, 0.0), 0.15, "pocket rim, sharp"),
        ],
    ));
    s
}

fn perforated_grille(name: &str) -> Scene {
    let small = Profile2::CircleGrid { first: [-33.0, -12.0], pitch: [3.0, 3.0], count: [11, 9], radius: 0.75 };
    let large = Profile2::CircleGrid { first: [10.0, -12.5], pitch: [5.0, 5.0], count: [6, 6], radius: 1.5 };
    let louvres: Vec<Profile2> = (0..6).map(|k| Profile2::Stadium { a: [-30.0 + 5.0 * k as f64, 19.0], b: [-30.0 + 5.0 * k as f64, 26.0], radius: 1.0 }).collect();
    let cuts = vec![prism(small.clone(), -5.0, 5.0), prism(large.clone(), -5.0, 5.0), prism(Profile2::Union { parts: louvres }, -5.0, 5.0)];
    let part = Shape::subtract(plate(40.0, 30.0, 2.0), cuts);
    let mut s = new_scene(
        name,
        "Perforated plate 80 x 60 x 2 mm: 99 holes of 1.5 mm at 3 mm pitch, 36 holes of 3 mm at 5 mm pitch, six 2 x 9 mm louvre slots. Top pass and a pass turned over, each lying on a table with clutter (the table shows through every hole).",
        part,
        [[-40.0, -30.0, -2.0], [40.0, 30.0, 0.0]],
        &["flat", "holes", "slots", "multi_pass", "table"],
    );
    s.passes.push(PassSpec::new("top", above(DVec3::ZERO)));
    let mut bottom = PassSpec::new("bottom", below(d3(0.0, 0.0, -2.0)));
    bottom.scan_pose = turned_over(0.0, -20.0, -5.0, 8.0);
    s.passes.push(bottom);
    table_with_clutter(&mut s, 0, -2.0, 1.0);
    table_with_clutter(&mut s, 1, 0.0, -1.0);
    let mut probes = Vec::new();
    for i in 0..11 {
        for j in 0..9 {
            let at = d3(-33.0 + 3.0 * i as f64, -12.0 + 3.0 * j as f64, -1.0);
            probes.push(Probe::clear(at, 0.5, "1.5 mm hole open"));
            probes.push(Probe::through(at, DVec3::Z, 1.3, "1.5 mm hole open through (no cap on the axis)"));
        }
    }
    s.features.push(feature("holes-1.5", "through_holes", "Every 1.5 mm hole stays open (99 holes, 0.75 mm wall-free radius ± 0.2), the 1.5 mm webs between them kept.", json!({"profile": small}), probes));
    let mut probes = Vec::new();
    for i in 0..6 {
        for j in 0..6 {
            let at = d3(10.0 + 5.0 * i as f64, -12.5 + 5.0 * j as f64, -1.0);
            probes.push(Probe::clear(at, 1.2, "3 mm hole open"));
            probes.push(Probe::through(at, DVec3::Z, 1.3, "3 mm hole open through (no cap on the axis)"));
        }
    }
    probes.push(height_probe(&s.part, d3(12.5, -10.0, 0.0), 0.05, "web between holes"));
    s.features.push(feature("holes-3", "through_holes", "Every 3 mm hole stays open (36 holes, diameter 3 ± 0.2 mm), webs flat.", json!({"profile": large}), probes));
    let probes = (0..6)
        .flat_map(|k| {
            let at = d3(-30.0 + 5.0 * k as f64, 22.5, -1.0);
            [Probe::clear(at, 0.75, "louvre open"), Probe::through(at, DVec3::Z, 1.3, "louvre open through (no cap on the axis)")]
        })
        .collect();
    let centres: Vec<[f64; 2]> = (0..6).map(|k| [-30.0 + 5.0 * k as f64, 22.5]).collect();
    s.features.push(feature("louvres", "through_slot", "Six 2 mm louvre slots, open, 2 ± 0.2 mm wide.", json!({"width_mm": 2, "length_mm": 9, "centers": centres, "axis": [0, 1]}), probes));
    s
}

// ---------------------------------------------------------------------------------------------
// Rounded surfaces

fn dome_part() -> Shape {
    // Sphere R 60 cut at z = 0: base radius 45, height 20.3.
    let zc = -(60.0f64 * 60.0 - 45.0 * 45.0).sqrt();
    intersect(vec![Shape::Sphere { center: [0.0, 0.0, zc], radius: 60.0 }, above_z(0.0)])
}

fn dome_base(name: &str, description: &str, tags: &[&str]) -> Scene {
    let part = dome_part();
    let mut s = new_scene(name, description, part, [[-45.0, -45.0, 0.0], [45.0, 45.0, 20.4]], tags);
    s.passes.push(PassSpec::new("outside", above(DVec3::ZERO)));
    let mut stickers = vec![sticker_down(&s.part, "M1-top", 0.0, 0.0)];
    for k in 0..4 {
        let a = (90.0 * k as f64 + 10.0f64).to_radians();
        stickers.push(sticker_down(&s.part, &format!("M{}-r18", k + 2), 18.0 * a.cos(), 18.0 * a.sin()));
    }
    for k in 0..3 {
        let a = (120.0 * k as f64 + 55.0f64).to_radians();
        stickers.push(sticker_down(&s.part, &format!("M{}-r33", k + 6), 33.0 * a.cos(), 33.0 * a.sin()));
    }
    // Its disc reaches the rim (partial site).
    stickers.push(sticker_down(&s.part, "M9-rim", -41.0, 0.0));
    s.stickers = stickers;
    let probes = (0..8)
        .map(|k| {
            let a = (45.0 * k as f64 + 30.0f64).to_radians();
            Probe::on(d3(45.0 * a.cos(), 45.0 * a.sin(), 0.0), 0.2, "rim edge")
        })
        .collect();
    s.features.push(feature("rim", "rim_edge", "The rim (sharp edge at z = 0, radius 45) stays where it is: no flaring, curling or shrinking (within 0.2 mm).", json!({"radius_mm": 45}), probes));
    let mut probes = Vec::new();
    for (x, y) in [(10.0, -8.0), (-20.0, 22.0), (27.0, 12.0), (-8.0, -30.0)] {
        probes.push(height_probe(&s.part, from_above(&s.part, x, y), 0.04, "dome surface"));
    }
    s.features.push(feature("sphere", "curved_surface", "Sphere R 60: smooth, within 0.04 mm of the truth away from the markers.", json!({"radius_mm": 60}), probes));
    s
}

/// The dome as a raw scan sees it: standing on its rim on a table with clutter, the stickers'
/// rings thin or empty.
fn dome_raw(name: &str, description: &str, tags: &[&str]) -> Scene {
    let mut s = dome_base(name, description, tags);
    raw_rings(&mut s.stickers);
    table_with_clutter(&mut s, 0, 0.0, 1.0);
    s
}

fn dome_r60_markers(name: &str) -> Scene {
    dome_raw(name, "Dome (sphere cap R 60, base 90 mm, height 20 mm) standing on its rim on a table with clutter, 9 raised stickers (rings thin or empty, two with a halo) including one whose disc reaches the rim; one pass from above, noise 0.025 mm.", &["rounded", "markers", "table"])
}

fn dome_r60_marker_holes(name: &str) -> Scene {
    let mut s = dome_base(name, "Dome R 60 as Revo's Fused export gives it: the 9 sticker sites cut out to 5.5 mm (one reaching the rim), 0.23 mm spacing, no table.", &["rounded", "marker_holes", "fused_export"]);
    fused_export(&mut s);
    s
}

fn dome_noise(name: &str, noise: f64) -> Scene {
    let mut s = dome_raw(name, &format!("Dome R 60 on a table with 9 raised stickers, noise {noise} mm along the ray (80 % of it correlated over 0.55 mm, plus hatching; grazing noise grows as 1 / cos)."), &["rounded", "markers", "noise", "table"]);
    s.passes[0].noise_mm = noise;
    s
}

fn dome_density_steps(name: &str) -> Scene {
    let mut s = dome_raw(
        name,
        "Dome R 60 on a table with 9 raised stickers; point spacing 0.25 mm (about 11 points per mm², ScanCraft's own fused passes) for x < -15, 0.4 mm (Revo's preview) in the middle, 0.8 mm for x > 15.",
        &["rounded", "markers", "density", "table"],
    );
    s.passes[0].density = vec![
        DensityRegion { region: Region::HalfSpace { normal: [1.0, 0.0, 0.0], offset: -15.0 }, spacing_mm: 0.25 },
        DensityRegion { region: Region::HalfSpace { normal: [-1.0, 0.0, 0.0], offset: -15.0 }, spacing_mm: 0.8 },
    ];
    let probes = [-15.0, 15.0].iter().flat_map(|&x| [-20.0, 0.0, 20.0].map(|y| height_probe(&s.part, from_above(&s.part, x, y), 0.04, "density step"))).collect();
    s.features.push(feature("density_steps", "density", "No seam, ridge or step where the point spacing changes (x = -15 and x = 15).", json!({"spacing_mm": [0.25, 0.4, 0.8], "steps_x": [-15, 15]}), probes));
    s
}

fn ball_r15(name: &str, holes: bool) -> Scene {
    let part = Shape::union(vec![intersect(vec![Shape::Sphere { center: [0.0; 3], radius: 15.0 }, above_z(-0.5)]), plate(30.0, 30.0, 3.0)]);
    let description = if holes {
        "Ball R 15 (half sphere) on a 60 x 60 x 3 mm plate as Revo's Fused export gives it: 7 sticker sites cut out to 5.5 mm, no table. On R 15 an 11 mm disc spans 42°."
    } else {
        "Ball R 15 (half sphere) on a 60 x 60 x 3 mm plate lying on a table with clutter, 5 raised stickers on the ball (top, 45°, 62° from the top) and 2 on the plate (rings thin or empty). On R 15 an 11 mm disc spans 42°."
    };
    let mut s = new_scene(name, description, part, [[-30.0, -30.0, -3.0], [30.0, 30.0, 15.0]], &["rounded", "small_radius", if holes { "marker_holes" } else { "markers" }]);
    s.passes.push(PassSpec::new("outside", above(DVec3::ZERO)));
    let on_ball = |id: &str, polar: f64, azimuth: f64| {
        let (p, a) = (polar.to_radians(), azimuth.to_radians());
        let n = d3(p.sin() * a.cos(), p.sin() * a.sin(), p.cos());
        let mut st = Sticker::standard(id, n * 15.0, n);
        st.back_limit = 1.5;
        st
    };
    s.stickers = vec![
        on_ball("M1-top", 0.0, 0.0),
        on_ball("M2-45", 45.0, 20.0),
        on_ball("M3-45", 45.0, 140.0),
        on_ball("M4-45", 45.0, 260.0),
        on_ball("M5-62", 62.0, 200.0),
        sticker_down(&s.part, "M6-plate", 22.0, -20.0),
        sticker_down(&s.part, "M7-plate", -21.0, 21.0),
    ];
    if holes {
        fused_export(&mut s);
    } else {
        raw_rings(&mut s.stickers);
        table_with_clutter(&mut s, 0, -3.0, 1.0);
    }
    let probes = (0..6).map(|k| {
        let a = (60.0 * k as f64 + 15.0f64).to_radians();
        Probe::on(d3(15.0 * a.cos(), 15.0 * a.sin(), 0.0), 0.25, "concave edge where the ball meets the plate")
    });
    let mut probes: Vec<Probe> = probes.collect();
    for (polar, az) in [(25.0, 80.0), (70.0, 30.0), (35.0, 300.0)] {
        let (p, a) = (f64::to_radians(polar), f64::to_radians(az));
        let n = d3(p.sin() * a.cos(), p.sin() * a.sin(), p.cos());
        probes.push(Probe::height(n * 15.0, n, 0.0, 0.04, "ball surface"));
    }
    s.features.push(feature("ball", "curved_surface", "Ball R 15: round (within 0.04 mm), the concave corner with the plate kept sharp-ish (within 0.25 mm).", json!({"radius_mm": 15}), probes));
    s
}

fn cylinder_r15(name: &str, holes: bool) -> Scene {
    let part = Shape::Cylinder { a: [-45.0, 0.0, 15.0], b: [45.0, 0.0, 15.0], radius: 15.0 };
    let description = if holes {
        "Cylinder R 15, 90 mm long, lying on its side, as Revo's Fused export gives it: 6 sticker sites (top, 45°, 80° to both sides, one near an end) cut out to 5.5 mm, no table."
    } else {
        "Cylinder R 15, 90 mm long, lying on its side on a table with clutter (line contact); 6 raised stickers (top, 45°, 80° to both sides, one near an end), rings thin or empty."
    };
    let mut s = new_scene(name, description, part, [[-45.0, -15.0, 0.0], [45.0, 15.0, 30.0]], &["rounded", "small_radius", if holes { "marker_holes" } else { "markers" }]);
    s.passes.push(PassSpec::new("outside", above(d3(0.0, 0.0, 15.0))));
    let at = |id: &str, x: f64, deg: f64| {
        let a = deg.to_radians();
        let n = d3(0.0, a.sin(), a.cos());
        let mut st = Sticker::standard(id, d3(x, 0.0, 15.0) + n * 15.0, n);
        st.back_limit = 1.5;
        st
    };
    s.stickers = vec![at("M1-top", -25.0, 0.0), at("M2-45", 0.0, 45.0), at("M3-45", 20.0, -45.0), at("M4-80", -5.0, 80.0), at("M5-80", 30.0, -80.0), at("M6-end", 45.0 - 5.6 - 0.5, 10.0)];
    if holes {
        fused_export(&mut s);
    } else {
        raw_rings(&mut s.stickers);
        table_with_clutter(&mut s, 0, 0.0, 1.0);
    }
    let mut probes: Vec<Probe> = [(-35.0, 20.0), (10.0, -20.0), (25.0, 60.0), (-15.0, -60.0)]
        .iter()
        .map(|&(x, deg): &(f64, f64)| {
            let a = deg.to_radians();
            let n = d3(0.0, a.sin(), a.cos());
            Probe::height(d3(x, 0.0, 15.0) + n * 15.0, n, 0.0, 0.04, "cylinder surface")
        })
        .collect();
    for deg in [0.0f64, 50.0, -50.0] {
        let a = deg.to_radians();
        probes.push(Probe::on(d3(45.0, 15.0 * a.sin(), 15.0 + 15.0 * a.cos()), 0.2, "end edge, sharp"));
    }
    s.features.push(feature("cylinder", "curved_surface", "Cylinder R 15: round across (within 0.04 mm), straight along, the end edges sharp.", json!({"radius_mm": 15, "axis": [1, 0, 0]}), probes));
    s
}

fn cylinder_r65(name: &str, holes: bool) -> Scene {
    let part = intersect(vec![
        Shape::Cylinder { a: [-50.0, 0.0, 0.0], b: [50.0, 0.0, 0.0], radius: 65.0 },
        Shape::Box { center: [0.0, 0.0, 55.0], half: [50.0, 45.0, 15.0], round: 0.0 },
    ]);
    let description = if holes {
        "Barrel panel: the top is a cylinder R 65 (axis along x), 100 x 90 mm, as Revo's Fused export gives it: 7 sticker sites cut out to 5.5 mm, no table."
    } else {
        "Barrel panel: the top is a cylinder R 65 (axis along x), 100 x 90 mm, on a table with clutter, 7 raised stickers (rings thin or empty)."
    };
    let mut s = new_scene(name, description, part, [[-50.0, -45.0, 40.0], [50.0, 45.0, 65.0]], &["rounded", "large_radius", if holes { "marker_holes" } else { "markers" }]);
    s.passes.push(PassSpec::new("outside", above(d3(0.0, 0.0, 55.0))));
    s.stickers = [(0.0, 0.0), (-25.0, 15.0), (25.0, 15.0), (-25.0, -15.0), (25.0, -15.0), (5.0, 32.0), (-5.0, -32.0)]
        .iter()
        .enumerate()
        .map(|(k, &(x, y))| sticker_down(&s.part, &format!("M{}", k + 1), x, y))
        .collect();
    if holes {
        fused_export(&mut s);
    } else {
        raw_rings(&mut s.stickers);
        table_with_clutter(&mut s, 0, 40.0, 1.0);
    }
    let mut probes = plane_probes(&s.part, &[(-40.0, 5.0), (12.0, -24.0), (35.0, 30.0), (-12.0, 40.0)], 0.04);
    for (x, y) in [(50.0f64, 0.0f64), (-50.0, 20.0), (10.0, 45.0)] {
        let p = from_above(&s.part, x - x.signum() * 0.01, y - y.signum() * 0.01);
        probes.push(Probe::on(p, 0.2, "top edge, sharp"));
    }
    s.features.push(feature("barrel", "curved_surface", "Cylinder R 65 top: within 0.04 mm, no flat spots, sharp top edges.", json!({"radius_mm": 65, "axis": [1, 0, 0]}), probes));
    s
}

fn saddle_part() -> Shape {
    Shape::HeightField {
        footprint: Profile2::RoundRect { center: [0.0, 0.0], half: [50.0, 40.0], radius: 3.0 },
        bottom: 0.0,
        base: 20.0,
        height: HeightFn::Saddle { rx: 150.0, ry: 110.0, bands: vec![Band { point: [0.0, 10.0], direction: [1.0, 0.25], height: 0.8, width: 4.0 }] },
    }
}

fn saddle(name: &str, holes: bool) -> Scene {
    let part = saddle_part();
    let description = if holes {
        "Saddle (x² / 300 - y² / 220, 20 mm thick) with a curvature band (a ridge 0.8 mm high, 8 mm wide), as Revo's Fused export gives it: 8 sticker sites cut out to 5.5 mm, two on the band's flank, no table."
    } else {
        "Saddle (x² / 300 - y² / 220, 20 mm thick) with a curvature band (a ridge 0.8 mm high, 8 mm wide) on a table with clutter; 8 raised stickers (rings thin or empty), two on the band's flank."
    };
    let mut s = new_scene(name, description, part, [[-50.0, -40.0, 0.0], [50.0, 40.0, 29.2]], &["rounded", "freeform", if holes { "marker_holes" } else { "markers" }]);
    s.passes.push(PassSpec::new("outside", above(d3(0.0, 0.0, 20.0))));
    s.stickers = [(-30.0, -20.0), (30.0, -22.0), (-32.0, 25.0), (34.0, 28.0), (0.0, -10.0), (8.0, -30.0), (-12.0, 3.5), (16.0, 18.0)]
        .iter()
        .enumerate()
        .map(|(k, &(x, y))| sticker_down(&s.part, &format!("M{}", k + 1), x, y))
        .collect();
    s.stickers[6].id = "M7-band".into();
    s.stickers[7].id = "M8-band".into();
    if holes {
        fused_export(&mut s);
    } else {
        raw_rings(&mut s.stickers);
        table_with_clutter(&mut s, 0, 0.0, 1.0);
    }
    // The band's crest: y = 10 + 0.25 x.
    let probes = [-40.0, -25.0, 0.0, 25.0, 40.0].iter().map(|&x| height_probe(&s.part, from_above(&s.part, x, 10.0 + 0.25 * x), 0.05, "band crest")).collect();
    s.features.push(feature("band", "curvature_band", "Real geometry: the ridge (0.8 mm high, 8 mm wide) is kept at its height (within 0.05 mm at the crest), not flattened.", json!({"height_mm": 0.8, "width_mm": 8, "line": {"point": [0, 10], "direction": [1, 0.25]}}), probes));
    s
}

fn grazing_cylinder_r30(name: &str) -> Scene {
    let part = Shape::Cylinder { a: [-50.0, 0.0, 30.0], b: [50.0, 0.0, 30.0], radius: 30.0 };
    let mut s = new_scene(
        name,
        "Cylinder R 30 lying on a table with clutter, seen only from above (three views): the flanks are grazing, noisier (noise / cos^1.5) with streaks of up to ±0.5 mm along the laser lines past 60° (as S3's fringe), and thin out up to 82°; raised stickers at 0, 30, 50, 65 and 75° from the top.",
        part,
        [[-50.0, -30.0, 0.0], [50.0, 30.0, 60.0]],
        &["rounded", "grazing", "partial", "markers", "streaks", "table"],
    );
    let c = d3(0.0, 0.0, 30.0);
    let mut p = PassSpec::new("above", vec![view(d3(-45.0, 0.0, 250.0), c), view(d3(0.0, 4.0, 250.0), c), view(d3(45.0, 0.0, 250.0), c)]);
    p.grazing_noise = 1.5;
    p.max_incidence_deg = 82.0;
    p.streaks = Some(grazing_streaks());
    s.passes.push(p);
    let at = |id: &str, x: f64, deg: f64| {
        let a = deg.to_radians();
        let n = d3(0.0, a.sin(), a.cos());
        Sticker::standard(id, d3(x, 0.0, 30.0) + n * 30.0, n)
    };
    s.stickers = vec![at("M1-0", -30.0, 0.0), at("M2-30", -10.0, 30.0), at("M3-50", 10.0, -50.0), at("M4-65", 28.0, 65.0), at("M5-75", -28.0, -75.0)];
    raw_rings(&mut s.stickers);
    table_with_clutter(&mut s, 0, 0.0, 1.0);
    let probes = [0.0f64, 30.0, 45.0, 60.0, 70.0]
        .iter()
        .map(|&deg| {
            let a = deg.to_radians();
            let n = d3(0.0, -a.sin(), a.cos());
            Probe::height(d3(0.0, 0.0, 30.0) + n * 30.0, n, 0.0, if deg < 50.0 { 0.04 } else { 0.08 }, format!("{deg}° from the top"))
        })
        .collect();
    s.features.push(feature(
        "flanks",
        "grazing",
        "Within 0.04 mm where the incidence is below 50°, within 0.08 mm up to 70°; where coverage ends near 80° the open boundary does not curl or flare (if closed, the fill continues the cylinder).",
        json!({"radius_mm": 30, "views": "above only", "max_incidence_deg": 82}),
        probes,
    ));
    s
}

// ---------------------------------------------------------------------------------------------
// Thin-walled shell (like the real Revo part)

fn shell_two_pass(name: &str) -> Scene {
    let wall = 2.5;
    let outer = Shape::SmoothIntersect {
        a: Box::new(Shape::Box { center: [0.0, 0.0, 5.0], half: [60.0, 40.0, 30.0], round: 12.0 }),
        b: Box::new(Shape::Sphere { center: [0.0, 0.0, 35.0 - 150.0], radius: 150.0 }),
        k: 6.0,
    };
    let stripe_outline = Profile2::RoundRect { center: [0.0, 0.0], half: [42.0, 25.0], radius: 8.0 };
    let coated = Shape::Coated {
        shape: Box::new(outer.clone()),
        coats: vec![Coat::Stripe { outline: stripe_outline.clone(), half_width: 1.0, height: 0.1, edge: 0.3, min_z: 20.0 }],
    };
    let cavity = Shape::Inset { shape: Box::new(outer.clone()), by: wall };
    let body = Shape::subtract(intersect(vec![coated, above_z(0.0)]), vec![cavity]);
    let bosses = [(20.0, 15.0), (-20.0, -15.0)];
    let mut inside = Vec::new();
    for &(x, y) in &bosses {
        inside.push(Shape::Cylinder { a: [x, y, 12.0], b: [x, y, 40.0], radius: 3.5 });
    }
    inside.push(Shape::Box { center: [0.0, 0.0, 22.0], half: [30.0, 0.6, 10.0], round: 0.0 });
    let additions = intersect(vec![Shape::union(inside), outer.clone()]);
    let mut cuts: Vec<Shape> = bosses.iter().map(|&(x, y)| Shape::Cylinder { a: [x, y, 11.0], b: [x, y, 17.0], radius: 1.25 }).collect();
    let holes = [(40.0, 22.0), (-40.0, -22.0)];
    for &(x, y) in &holes {
        cuts.push(prism(Profile2::Circle { center: [x, y], radius: 1.6 }, 10.0, 40.0));
    }
    let part = Shape::subtract(Shape::union(vec![body, additions]), cuts);
    let mut s = new_scene(
        name,
        "Thin-walled shell like the real Revo part: 120 x 80 x 35 mm, wall 2.5 mm, domed top (R 150), open at the bottom; a painted stripe 0.1 mm high along a rounded rectangle; two screw holes through the top; inside two screw bosses with 2.5 mm pilot recesses and a rib. Pass 1 outside (no table, as S1), pass 2 inside (part turned over onto a table with clutter, own frame) 0.15 mm too deep and placed with an alignment's residual error (0.22°, 0.1 mm), with cross-line laser strays 0.6-1.0 mm behind the inner surface and grazing fringes with streaks along the laser lines. Stickers outside and inside (rings thin or empty).",
        part,
        [[-60.0, -40.0, 0.0], [60.0, 40.0, 35.2]],
        &["shell", "thin_wall", "multi_pass", "offset", "tilt", "laser_strays", "streaks", "markers", "screw_holes", "bosses", "ribs", "real_relief", "table"],
    );
    let mut outside = PassSpec::new("outside", above(d3(0.0, 0.0, 12.0)));
    outside.max_incidence_deg = 80.0;
    s.passes.push(outside);
    let mut inside = PassSpec::new("inside", below(d3(0.0, 0.0, 25.0)));
    inside.scan_pose = turned_over(0.0, 25.0, 15.0, -10.0);
    inside.depth_bias_mm = 0.15;
    inside.grazing_noise = 1.5;
    inside.strays = Some(strays_window(d3(-25.0, 10.0, 30.0), 22.0));
    inside.streaks = Some(grazing_streaks());
    // What Align All leaves between the passes (0.22° and 0.1 mm; the real S1 -> S3 0.27°).
    inside.pose_error = tilt([1.0, 0.4, 0.0], 0.22, d3(0.0, 0.0, 17.0), [0.05, -0.08, 0.06]);
    s.passes.push(inside);

    let mut st = vec![
        sticker_down(&s.part, "O1-top", 0.0, 0.0),
        sticker_down(&s.part, "O2", 25.0, 12.0),
        sticker_down(&s.part, "O3", -25.0, -12.0),
        sticker_down(&s.part, "O4", -28.0, 14.0),
        sticker_facing(&s.part, "O5-side", d3(5.0, -40.0, 12.0), -DVec3::Y),
        sticker_facing(&s.part, "O6-end", d3(60.0, 5.0, 12.0), DVec3::X),
        sticker_facing(&s.part, "O7-side", d3(-15.0, 40.0, 10.0), DVec3::Y),
    ];
    for (id, x, y) in [("I1", -32.0, 18.0), ("I2", 32.0, -18.0), ("I3", 4.0, -22.0)] {
        st.push(sticker(&s.part, id, d3(x, y, 1.0), DVec3::Z));
    }
    for m in &mut st {
        m.back_limit = 1.2;
    }
    raw_rings(&mut st);
    s.stickers = st;
    // The inside pass: the shell lies on its top on a table with marker blocks and clutter (as
    // S2 and S3); the outside pass has no table (as S1's preview).
    let top = from_above(&s.part, 0.0, 0.0).z;
    table_with_clutter(&mut s, 1, top, -1.0);

    let wall_point = |x: f64, y: f64| {
        let outer_p = from_above(&s.part, x, y);
        let n = s.part.normal(outer_p);
        outer_p - n * (wall / 2.0)
    };
    let mut probes = Vec::new();
    for (x, y) in [(-25.0, 10.0), (-35.0, 2.0), (-15.0, 22.0), (-30.0, 24.0), (10.0, -5.0), (30.0, 5.0)] {
        probes.push(Probe::clear(wall_point(x, y), 0.9, "inside the wall: no surface"));
        let inner = from_below(&s.part, x, y);
        probes.push(height_probe(&s.part, inner, 0.05, "inner surface"));
    }
    s.features.push(feature(
        "wall",
        "thin_wall",
        "Wall 2.5 mm: both faces at their true place (inner within 0.05 mm once the 0.15 mm offset of pass 2 is dealt with), no surface inside the wall, no pits or lattice from the laser strays in the window round (-25, 10).",
        json!({"thickness_mm": wall, "stray_window": {"center": [-25, 10, 30], "radius_mm": 22}, "inside_depth_bias_mm": 0.15}),
        probes,
    ));
    let probes = [(42.0, 0.0), (0.0, 25.0), (-42.0, -6.0), (20.0, -25.0)]
        .iter()
        .flat_map(|&(x, y)| {
            let on = from_above(&s.part, x, y);
            let off = from_above(&s.part, x + if x.abs() > 40.0 { -2.5 } else { 0.0 }, y + if y.abs() > 20.0 { -2.5 } else { 0.0 });
            [height_probe(&s.part, on, 0.04, "on the stripe"), height_probe(&s.part, off, 0.04, "beside the stripe")]
        })
        .collect();
    s.features.push(feature("stripe", "real_relief", "Real geometry: the painted stripe (0.1 mm high, 2 mm wide, rounded-rectangle line) is kept, not smoothed away (stripe top within 0.04 mm).", json!({"height_mm": 0.1, "width_mm": 2, "outline": stripe_outline}), probes));
    for (k, &(x, y)) in holes.iter().enumerate() {
        let top = from_above(&s.part, x + 3.0, y);
        let mid = d3(x, y, top.z - wall / 2.0);
        s.features.push(feature(
            &format!("screw_hole-{}", k + 1),
            "through_hole",
            "Screw hole 3.2 mm through the 2.5 mm wall: open (not patched), diameter 3.2 ± 0.3 mm.",
            json!({"center": [x, y], "diameter_mm": 3.2}),
            vec![Probe::clear(mid, 1.25, "hole open"), Probe::clear(mid + d3(0.0, 0.0, 1.0), 1.25, "hole open near the top"), Probe::through(mid, DVec3::Z, wall / 2.0 + 0.6, "hole open through (no cap on the axis)")],
        ));
    }
    for (k, &(x, y)) in bosses.iter().enumerate() {
        s.features.push(feature(
            &format!("boss-{}", k + 1),
            "screw_boss",
            "Screw boss (7 mm across, end at z = 12) with a 2.5 mm pilot recess 5 mm deep: boss wall and end face kept, recess open to its floor.",
            json!({"center": [x, y], "outer_diameter_mm": 7, "end_z": 12, "pilot_diameter_mm": 2.5, "pilot_depth_mm": 5, "pilot_axis": [0, 0, 1]}),
            vec![
                Probe::clear(d3(x, y, 14.5), 0.9, "pilot recess open"),
                Probe::height(d3(x, y, 17.0), -DVec3::Z, 0.0, 0.2, "pilot recess floor"),
                Probe::height(d3(x + 2.4, y, 12.0), -DVec3::Z, 0.0, 0.08, "boss end face"),
                Probe::height(d3(x, y + 3.5, 14.0), DVec3::Y, 0.0, 0.08, "boss wall"),
            ],
        ));
    }
    s.features.push(feature(
        "rib",
        "rib",
        "Rib 1.2 mm thick hanging 10 mm below the inside of the top: kept, both faces and its end at z = 12.",
        json!({"thickness_mm": 1.2, "end_z": 12, "from": [-30, 0], "to": [30, 0]}),
        vec![
            Probe::height(d3(0.0, 0.0, 12.0), -DVec3::Z, 0.0, 0.1, "rib end"),
            Probe::height(d3(10.0, 0.6, 16.0), DVec3::Y, 0.0, 0.08, "rib face"),
            Probe::height(d3(-10.0, -0.6, 16.0), -DVec3::Y, 0.0, 0.08, "rib face"),
        ],
    ));
    let probes = [(60.0f64, 0.0f64), (0.0, 40.0), (-60.0, 10.0), (20.0, -40.0)]
        .iter()
        .flat_map(|&(x, y)| {
            let n = d3(x.signum() * f64::from(x.abs() > 59.0), y.signum() * f64::from(y.abs() > 39.0), 0.0);
            [Probe::on(d3(x, y, 0.0), 0.2, "outer rim edge"), Probe::on(d3(x, y, 0.0) - n * wall, 0.2, "inner rim edge")]
        })
        .collect();
    s.features.push(feature("rim", "rim_edge", "The open rim (z = 0) stays sharp and in place on both faces (within 0.2 mm); the shell stays open there.", json!({"z": 0}), probes));
    s.truth_cell_mm = 0.4;
    s
}

/// The shell standing on its rim on a table with clutter, scanned from outside only: the rim
/// touches the table, so isolation must cut the part off the support plane without eating the
/// rim.
fn shell_on_rim(name: &str) -> Scene {
    let mut s = shell_two_pass(name);
    s.description = "The thin-walled shell of shell_two_pass standing on its open rim on a table with a marker block, a can and a box, scanned from outside only (one pass): the rim touches the table, so isolating the part must cut it off the support plane without eating the rim; the inside is not seen.".into();
    s.tags = ["shell", "thin_wall", "rim", "table", "markers", "screw_holes", "real_relief"].iter().map(|t| t.to_string()).collect();
    s.passes.truncate(1);
    s.stickers.retain(|m| m.id.starts_with('O'));
    table_with_clutter(&mut s, 0, 0.0, 1.0);
    s.features.retain(|f| matches!(f.id.as_str(), "stripe" | "screw_hole-1" | "screw_hole-2"));
    let probes = [(60.0f64, 0.0f64), (0.0, 40.0), (-60.0, 10.0), (20.0, -40.0)].iter().map(|&(x, y)| Probe::on(d3(x, y, 0.0), 0.25, "outer rim edge on the table")).collect();
    s.features.push(feature("rim", "rim_edge", "The outer rim edge where the shell stands on the table stays where it is (within 0.25 mm): the support plane is cut off at z = 0, not higher.", json!({"z": 0}), probes));
    s
}

fn cup_rim_partial(name: &str) -> Scene {
    let part = Shape::subtract(Shape::Cylinder { a: [0.0, 0.0, 0.0], b: [0.0, 0.0, 30.0], radius: 40.0 }, vec![Shape::Cylinder { a: [0.0, 0.0, 3.0], b: [0.0, 0.0, 31.0], radius: 37.5 }]);
    let mut s = new_scene(
        name,
        "Cup: cylinder R 40, 30 mm high, wall 2.5 mm, open top rim, standing on a table with clutter. One pass from all round and above: the inside is seen only in part. Stickers on the outer wall, two of them so close to the rim that their disc ends 0.4 mm below it; their rings scan thin or empty, one with a halo to 12 mm.",
        part,
        [[-40.0, -40.0, 0.0], [40.0, 40.0, 30.0]],
        &["thin_wall", "rim", "partial", "markers", "table"],
    );
    s.passes.push(PassSpec::new("outside", around(d3(0.0, 0.0, 15.0))));
    let on_wall = |id: &str, deg: f64, z: f64| {
        let a = deg.to_radians();
        let n = d3(a.cos(), a.sin(), 0.0);
        let mut st = Sticker::standard(id, n * 40.0 + d3(0.0, 0.0, z), n);
        st.back_limit = 1.2;
        st
    };
    let mut inner = Sticker::standard("M6-inside", d3(-37.5, 0.0, 20.0), DVec3::X);
    inner.back_limit = 1.2;
    s.stickers = vec![on_wall("M1-rim", 0.0, 30.0 - 6.0), on_wall("M2-rim", 130.0, 24.0), on_wall("M3", 60.0, 12.0), on_wall("M4", 250.0, 12.0), on_wall("M5-rim", 200.0, 24.0), inner];
    raw_rings(&mut s.stickers);
    s.stickers[3] = s.stickers[3].clone().styled(RingStyle::Halo, 2);
    table_with_clutter(&mut s, 0, 0.0, 1.0);
    let mut probes = Vec::new();
    for deg in [20.0f64, 100.0, 170.0, 280.0] {
        let a = deg.to_radians();
        let u = d3(a.cos(), a.sin(), 0.0);
        probes.push(Probe::on(u * 40.0 + d3(0.0, 0.0, 30.0), 0.2, "outer rim edge"));
        probes.push(Probe::on(u * 37.5 + d3(0.0, 0.0, 30.0), 0.25, "inner rim edge"));
        probes.push(Probe::height(u * 38.75 + d3(0.0, 0.0, 30.0), DVec3::Z, 0.0, 0.05, "rim top"));
        probes.push(Probe::clear(u * 38.75 + d3(0.0, 0.0, 20.0), 0.9, "inside the wall"));
    }
    s.features.push(feature(
        "rim",
        "rim_edge",
        "Open rim 2.5 mm wide: flat top and both edges in place (within 0.2 mm), the cup stays open (not capped), the wall not thickened where the inside is seen only in part.",
        json!({"z": 30, "outer_radius_mm": 40, "inner_radius_mm": 37.5}),
        probes,
    ));
    s
}

// ---------------------------------------------------------------------------------------------
// Edges, relief, plates with screw features

fn box_edges(name: &str) -> Scene {
    let profile = Profile2::Rect {
        center: [0.0, 15.0],
        half: [35.0, 15.0],
        corners: [Corner::Fillet { radius: 1.5 }, Corner::Sharp, Corner::Fillet { radius: 0.5 }, Corner::Chamfer { size: 1.0 }],
    };
    let block = Shape::ExtrudeY { profile: profile.clone(), y0: -25.0, y1: 25.0 };
    let chamfer_plane = Shape::HalfSpace { normal: [0.0, 1.0, 1.0], offset: (25.0 + 30.0 - 2.0) / 2f64.sqrt() };
    let notch = Shape::Box { center: [-28.5, 0.0, 27.0], half: [7.5, 26.0, 5.0], round: 0.0 };
    let part = Shape::subtract(intersect(vec![block, chamfer_plane]), vec![notch]);
    let mut s = new_scene(
        name,
        "Block 70 x 50 x 30 mm for edge preservation: edges along y are sharp / fillet 1.5 / fillet 0.5 / chamfer 1; a 2 mm chamfer along the top front edge; a 14 x 8 mm step (sharp concave edge). Pass 1 on a table with a marker block, a can, a box and table stickers (Isolate Object should drop them); pass 2 turned over onto its top, on another table with the same, own frame.",
        part,
        [[-35.0, -25.0, 0.0], [35.0, 25.0, 30.0]],
        &["edges", "table", "multi_pass", "markers"],
    );
    s.passes.push(PassSpec::new("upright", above(d3(0.0, 0.0, 10.0))));
    let mut p2 = PassSpec::new("turned_over", below(d3(0.0, 0.0, 20.0)));
    p2.scan_pose = turned_over(30.0, -40.0, 10.0, 5.0);
    s.passes.push(p2);
    table_with_clutter(&mut s, 0, 0.0, 1.0);
    table_with_clutter(&mut s, 1, 30.0, -1.0);
    let along = |x: f64, z: f64, tol: f64, note: &str| -> Vec<Probe> { [-15.0, 0.0, 15.0].iter().map(|&y| Probe::on(d3(x, y, z), tol, note)).collect() };
    let fillet_mid = |cx: f64, cz: f64, r: f64, sx: f64, sz: f64| d3(cx, 0.0, cz) + d3(sx, 0.0, sz).normalize() * r;
    s.features.push(feature("edge-sharp", "sharp_edge", "Sharp 90° edge (x = -35, z = 22, along y): kept sharp, the mesh within 0.1 mm of the edge line.", json!({"edge": "x=-35,z=22"}), along(-35.0, 22.0, 0.1, "sharp edge")));
    let mut probes: Vec<Probe> = [-15.0, 0.0, 15.0].iter().map(|&y| Probe { point: (fillet_mid(33.5, 28.5, 1.5, 1.0, 1.0) + d3(0.0, y, 0.0)).to_array(), check: ProbeKind::Height { normal: d3(1.0, 0.0, 1.0).normalize().to_array(), expect: 0.0, tol: 0.05, range: 2.0 }, note: "fillet 1.5 middle".into() }).collect();
    probes.push(height_probe(&s.part, d3(35.0, 0.0, 20.0), 0.04, "face below the fillet"));
    s.features.push(feature("edge-fillet-1.5", "fillet", "Fillet R 1.5 (x = 35, z = 30): round with its radius (middle within 0.05 mm), not sharpened or flattened.", json!({"radius_mm": 1.5}), probes));
    let probes = [-15.0, 0.0, 15.0].iter().map(|&y| Probe { point: (fillet_mid(-34.5, 0.5, 0.5, -1.0, -1.0) + d3(0.0, y, 0.0)).to_array(), check: ProbeKind::Height { normal: d3(-1.0, 0.0, -1.0).normalize().to_array(), expect: 0.0, tol: 0.06, range: 2.0 }, note: "fillet 0.5 middle".into() }).collect();
    s.features.push(feature("edge-fillet-0.5", "fillet", "Fillet R 0.5 (x = -35, z = 0, seen from pass 2): a small round, within 0.06 mm.", json!({"radius_mm": 0.5}), probes));
    let mut probes = along(35.0, 1.0, 0.1, "chamfer edge on the side face");
    probes.extend(along(34.0, 0.0, 0.1, "chamfer edge on the bottom face"));
    probes.extend([-15.0, 15.0].iter().map(|&y| Probe { point: [34.5, y, 0.5], check: ProbeKind::Height { normal: d3(1.0, 0.0, -1.0).normalize().to_array(), expect: 0.0, tol: 0.05, range: 2.0 }, note: "chamfer face".into() }));
    s.features.push(feature("edge-chamfer-1", "chamfer", "Chamfer 1 mm (x = 35, z = 0): a flat 45° face with two sharp edges.", json!({"size_mm": 1.0}), probes));
    let mut probes: Vec<Probe> = [-15.0, 0.0, 10.0, 25.0].iter().map(|&x| Probe::on(d3(x, 25.0 - 2.0, 30.0), 0.1, "top edge of the 2 mm chamfer")).collect();
    probes.extend([-10.0, 20.0].iter().map(|&x| Probe::on(d3(x, 25.0, 28.0), 0.1, "front edge of the 2 mm chamfer")));
    s.features.push(feature("edge-chamfer-2", "chamfer", "Chamfer 2 mm along the top front edge (y = 25, z = 30): flat, both edges sharp.", json!({"size_mm": 2.0}), probes));
    let mut probes = along(-21.0, 22.0, 0.15, "concave step edge");
    probes.extend(along(-21.0, 30.0, 0.1, "convex step edge"));
    probes.push(Probe::clear(d3(-27.0, 0.0, 26.0), 2.5, "the step is empty"));
    s.features.push(feature("step", "concave_edge", "Step 14 x 8 mm: the concave edge (x = -21, z = 22) stays sharp, not filled with a fillet (within 0.15 mm).", json!({"x": -21, "z": 22}), probes));
    s.features.push(feature(
        "table",
        "support",
        "The tables (z = 0 in pass 1, z = 30 in pass 2, with the clutter and the stickers T1-*, T2-*) are not part of the object: Isolate Object drops them; Make Solid / Reconstruct of the isolated passes has no table.",
        json!({"pass_1_table_top_z": 0, "pass_2_table_top_z": 30}),
        vec![Probe::clear(d3(-50.0, 0.0, -0.5), 4.0, "no table in the result"), Probe::clear(d3(50.0, 30.0, 0.5), 4.0, "no table in the result")],
    ));
    s.truth_cell_mm = 0.25;
    s
}

fn relief_fine(name: &str) -> Scene {
    let emboss = Profile2::Strokes {
        segments: vec![
            // E
            [[-38.0, 5.0], [-38.0, 20.0]],
            [[-38.0, 20.0], [-30.0, 20.0]],
            [[-38.0, 12.5], [-32.0, 12.5]],
            [[-38.0, 5.0], [-30.0, 5.0]],
            // H
            [[-26.0, 5.0], [-26.0, 20.0]],
            [[-18.0, 5.0], [-18.0, 20.0]],
            [[-26.0, 12.5], [-18.0, 12.5]],
            // Z
            [[-14.0, 20.0], [-6.0, 20.0]],
            [[-6.0, 20.0], [-14.0, 5.0]],
            [[-14.0, 5.0], [-6.0, 5.0]],
        ],
        radius: 0.4,
    };
    let engrave = Profile2::Strokes {
        segments: vec![[[-38.0, -5.0], [-28.0, -18.0]], [[-38.0, -18.0], [-28.0, -5.0]], [[-24.0, -5.0], [-19.0, -18.0]], [[-19.0, -18.0], [-14.0, -5.0]]],
        radius: 0.3,
    };
    let knurl = Shape::HeightField {
        footprint: Profile2::RoundRect { center: [25.0, 12.0], half: [10.0, 10.0], radius: 1.0 },
        bottom: -1.0,
        base: 0.0,
        height: HeightFn::Knurl { center: [25.0, 12.0], pitch: 1.2, depth: 0.25, angle_deg: 45.0 },
    };
    let bump_centers: Vec<V2> = (0..6).map(|k| [10.0 + 5.0 * k as f64, -10.0]).collect();
    let bumps = Shape::HeightField {
        footprint: Profile2::RoundRect { center: [22.5, -10.0], half: [16.0, 2.5], radius: 1.0 },
        bottom: -1.0,
        base: 0.0,
        height: HeightFn::Bumps { centers: bump_centers.clone(), radius: 1.0, height: 0.2 },
    };
    let stripe_line = Profile2::Stadium { a: [-35.0, -24.0], b: [35.0, -24.0], radius: 0.0 };
    let plate_coated = Shape::Coated { shape: Box::new(plate(45.0, 30.0, 4.0)), coats: vec![Coat::Stripe { outline: stripe_line.clone(), half_width: 1.0, height: 0.1, edge: 0.3, min_z: -1.0 }] };
    let part = Shape::subtract(Shape::union(vec![plate_coated, prism(emboss.clone(), -1.0, 0.3), knurl, bumps]), vec![prism(engrave.clone(), -0.4, 2.0)]);
    let mut s = new_scene(
        name,
        "Fine real relief that must be kept (the counter-case to artefact suppression): embossed strokes 0.8 mm wide, 0.3 mm high (E H Z), engraved strokes 0.6 mm wide, 0.4 mm deep (X V), a diamond knurl (pitch 1.2, 0.25 mm deep), six bumps 0.2 mm high, a painted stripe 0.1 mm high. Plate 90 x 60 x 4 on a table with clutter, spacing 0.25 mm (about 11 points per mm², as ScanCraft's own fused passes), noise 0.02 mm.",
        part,
        [[-45.0, -30.0, -4.0], [45.0, 30.0, 0.3]],
        &["flat", "real_relief", "markers", "table"],
    );
    let mut p = PassSpec::new("top", above(DVec3::ZERO));
    p.spacing_mm = 0.25;
    p.noise_mm = 0.02;
    s.passes.push(p);
    s.stickers = vec![sticker_down(&s.part, "M1", 2.0, 24.0), sticker_down(&s.part, "M2", 39.0, -2.0), sticker_down(&s.part, "M3", 2.0, -2.0)];
    raw_rings(&mut s.stickers);
    table_with_clutter(&mut s, 0, -4.0, 1.0);
    let tops = [(-38.0, 15.0), (-34.0, 20.0), (-22.0, 12.5), (-10.0, 12.5), (-10.0, 5.0)];
    let probes = tops.iter().map(|&(x, y)| Probe::height(d3(x, y, 0.3), DVec3::Z, 0.0, 0.06, "stroke top (0.3 above the plate)")).collect();
    s.features.push(feature("emboss", "real_relief", "Embossed strokes 0.8 mm wide, 0.3 mm high: kept at their height (top within 0.06 mm, i.e. at least 0.24 mm high), not smoothed into the plate or dropped as artefacts.", json!({"width_mm": 0.8, "height_mm": 0.3, "profile": emboss}), probes));
    let probes = [(-33.0, -11.5), (-16.5, -11.5), (-21.5, -11.5)]
        .iter()
        .map(|&(x, y)| {
            let p = from_above(&s.part, x, y);
            Probe::height(p, DVec3::Z, 0.0, 0.12, "groove floor (0.4 below the plate)")
        })
        .collect();
    s.features.push(feature("engrave", "real_relief", "Engraved strokes 0.6 mm wide, 0.4 mm deep: kept open (floor within 0.12 mm; at 0.25 mm spacing the floor is barely sampled), not filled.", json!({"width_mm": 0.6, "depth_mm": 0.4, "profile": engrave}), probes));
    let pitch = 1.2;
    let mut probes = Vec::new();
    for (i, j) in [(0.0, 0.0), (2.0, 1.0), (-3.0, 2.0)] {
        // Peaks at half-integer cells of the turned grid, valleys at the lines.
        let (s45, c45) = 45f64.to_radians().sin_cos();
        let (u, v) = ((i + 0.5) * pitch, (j + 0.5) * pitch);
        let peak = DVec2::new(25.0 + c45 * u - s45 * v, 12.0 + s45 * u + c45 * v);
        let (u, v) = (i * pitch, (j + 0.5) * pitch);
        let valley = DVec2::new(25.0 + c45 * u - s45 * v, 12.0 + s45 * u + c45 * v);
        probes.push(Probe::height(from_above(&s.part, peak.x, peak.y), DVec3::Z, 0.0, 0.08, "knurl peak (0.25 above the valleys)"));
        probes.push(Probe::height(from_above(&s.part, valley.x, valley.y), DVec3::Z, 0.0, 0.08, "knurl valley"));
    }
    s.features.push(feature("knurl", "real_relief", "Diamond knurl, pitch 1.2 mm, 0.25 mm deep: the pattern is kept (peaks and valleys within 0.08 mm).", json!({"pitch_mm": pitch, "height_mm": 0.25, "center": [25, 12], "half_mm": [10, 10], "peak_valley": true}), probes));
    let probes = bump_centers.iter().map(|c| Probe::height(d3(c[0], c[1], 0.2), DVec3::Z, 0.0, 0.05, "bump top (0.2 high)")).collect();
    s.features.push(feature("bumps", "real_relief", "Six bumps 2 mm across, 0.2 mm high: kept (tops within 0.05 mm).", json!({"height_mm": 0.2, "diameter_mm": 2, "centers": bump_centers}), probes));
    let probes = [-25.0, 0.0, 25.0].iter().flat_map(|&x| [Probe::height(d3(x, -24.0, 0.1), DVec3::Z, 0.0, 0.04, "stripe top"), Probe::height(d3(x, -21.0, 0.0), DVec3::Z, 0.0, 0.04, "beside the stripe")]).collect();
    s.features.push(feature("stripe", "real_relief", "Painted stripe 2 mm wide, 0.1 mm high: kept (top within 0.04 mm).", json!({"height_mm": 0.1, "width_mm": 2}), probes));
    s.truth_cell_mm = 0.15;
    s
}

fn plate_screws_bosses_ribs(name: &str) -> Scene {
    let bosses = [(-20.0, 15.0), (20.0, 15.0), (0.0, -18.0)];
    let mut solid = vec![plate(50.0, 35.0, 2.0)];
    for &(x, y) in &bosses {
        solid.push(Shape::Cylinder { a: [x, y, -8.0], b: [x, y, -1.0], radius: 3.5 });
    }
    solid.push(Shape::Box { center: [0.0, 0.0, -5.0], half: [35.0, 0.6, 3.0], round: 0.0 });
    solid.push(Shape::Box { center: [-30.0, 0.0, -5.0], half: [0.6, 25.0, 3.0], round: 0.0 });
    let holes = [(-40.0, 27.0, 1.6), (-40.0, -27.0, 1.6), (40.0, 27.0, 2.25), (40.0, -27.0, 2.25)];
    let mut cuts: Vec<Shape> = holes.iter().map(|&(x, y, r)| prism(Profile2::Circle { center: [x, y], radius: r }, -10.0, 10.0)).collect();
    // Pilot recesses 4 mm deep from the boss ends; the middle boss's goes through the plate.
    cuts.push(Shape::Cylinder { a: [-20.0, 15.0, -9.0], b: [-20.0, 15.0, -4.0], radius: 1.25 });
    cuts.push(Shape::Cylinder { a: [20.0, 15.0, -9.0], b: [20.0, 15.0, 1.0], radius: 1.25 });
    cuts.push(Shape::Cylinder { a: [0.0, -18.0, -9.0], b: [0.0, -18.0, -4.0], radius: 1.25 });
    let part = Shape::subtract(Shape::union(solid), cuts);
    let mut s = new_scene(
        name,
        "Thin plate 100 x 70 x 2 mm with four screw holes (3.2 and 4.5 mm) through it; underneath three screw bosses (7 mm, 6 mm high) with 2.5 mm pilot recesses 4 mm deep (one goes through the plate) and two ribs 1.2 mm thick, 6 mm high. Top pass with 5 stickers (standing on its bosses on a table with clutter), bottom pass turned over onto its top on a table (own frame, 0.05 mm too deep).",
        part,
        [[-50.0, -35.0, -8.0], [50.0, 35.0, 0.0]],
        &["flat", "thin_wall", "screw_holes", "bosses", "ribs", "markers", "multi_pass", "table"],
    );
    s.passes.push(PassSpec::new("top", above(DVec3::ZERO)));
    let mut bottom = PassSpec::new("bottom", below(d3(0.0, 0.0, -5.0)));
    bottom.scan_pose = turned_over(0.0, 15.0, -10.0, 4.0);
    bottom.depth_bias_mm = 0.05;
    s.passes.push(bottom);
    s.stickers = [(-25.0, -20.0), (25.0, -20.0), (0.0, 22.0), (-8.0, 2.0), (36.0, 6.0)].iter().enumerate().map(|(k, &(x, y))| sticker_down(&s.part, &format!("M{}", k + 1), x, y)).collect();
    raw_rings(&mut s.stickers);
    // Top up it stands on its bosses and ribs (the table 6 mm below the plate shows through the
    // holes); turned over it lies on its top.
    table_with_clutter(&mut s, 0, -8.0, 1.0);
    table_with_clutter(&mut s, 1, 0.0, -1.0);
    for (k, &(x, y, r)) in holes.iter().enumerate() {
        s.features.push(feature(
            &format!("screw_hole-{}", k + 1),
            "through_hole",
            &format!("Screw hole {} mm through the 2 mm plate: open, diameter ± 0.3 mm, edges sharp.", 2.0 * r),
            json!({"center": [x, y], "diameter_mm": 2.0 * r}),
            vec![Probe::clear(d3(x, y, -1.0), r - 0.3, "hole open"), Probe::through(d3(x, y, -1.0), DVec3::Z, 1.3, "hole open through (no cap on the axis)"), Probe::on(d3(x + r, y, 0.0), 0.15, "hole edge")],
        ));
    }
    for (k, &(x, y)) in bosses.iter().enumerate() {
        let through = k == 1;
        let mut probes = vec![
            Probe::clear(d3(x, y, -6.0), 0.9, "pilot recess open"),
            Probe::height(d3(x + 2.4, y, -8.0), -DVec3::Z, 0.0, 0.08, "boss end face"),
            Probe::height(d3(x - 3.5, y, -5.0), -DVec3::X, 0.0, 0.08, "boss wall"),
        ];
        if through {
            probes.push(Probe::clear(d3(x, y, -1.0), 0.9, "pilot hole open through the plate"));
            probes.push(Probe::through(d3(x, y, -4.0), DVec3::Z, 4.3, "pilot hole open end to end (no cap on the axis)"));
        } else {
            probes.push(Probe::height(d3(x, y, -4.0), -DVec3::Z, 0.0, 0.2, "pilot recess floor"));
        }
        s.features.push(feature(
            &format!("boss-{}", k + 1),
            "screw_boss",
            if through { "Screw boss with a 2.5 mm pilot hole right through boss and plate: open end to end." } else { "Screw boss with a 2.5 mm pilot recess 4 mm deep: recess open to its floor, not filled." },
            json!({"center": [x, y], "outer_diameter_mm": 7, "end_z": -8, "pilot_diameter_mm": 2.5, "pilot_depth_mm": if through { 8.0 } else { 4.0 }, "pilot_axis": [0, 0, 1], "pilot_through": through}),
            probes,
        ));
    }
    s.features.push(feature(
        "ribs",
        "rib",
        "Ribs 1.2 mm thick, 6 mm high: both faces kept (within 0.08 mm), not thickened or merged into one sheet, ends square.",
        json!({"thickness_mm": 1.2, "height_mm": 6}),
        vec![
            Probe::height(d3(10.0, 0.6, -5.0), DVec3::Y, 0.0, 0.08, "rib face"),
            Probe::height(d3(10.0, -0.6, -5.0), -DVec3::Y, 0.0, 0.08, "rib face"),
            Probe::height(d3(12.0, 0.0, -8.0), -DVec3::Z, 0.0, 0.08, "rib end"),
            Probe::clear(d3(12.0, 0.0, -5.0), 0.35, "inside the rib"),
            Probe::height(d3(-30.0, 12.0, -8.0), -DVec3::Z, 0.0, 0.08, "cross rib end"),
        ],
    ));
    s.truth_cell_mm = 0.3;
    s
}

fn thin_fins(name: &str) -> Scene {
    let fins = [(0.6, -24.0), (1.0, -8.0), (1.5, 8.0), (2.5, 24.0)];
    let mut parts = vec![plate(40.0, 25.0, 3.0)];
    for &(t, x) in &fins {
        parts.push(Shape::Box { center: [x, 0.0, 6.0], half: [t / 2.0, 20.0, 6.0], round: 0.0 });
    }
    let part = Shape::union(parts);
    let mut s = new_scene(
        name,
        "Plate 80 x 50 x 3 mm with four fins 12 mm high and 0.6 / 1.0 / 1.5 / 2.5 mm thick, lying on a table with clutter, seen from all round: the thin-wall and ghost-layer logic must keep both faces of every fin.",
        part,
        [[-40.0, -25.0, -3.0], [40.0, 25.0, 12.0]],
        &["thin_wall", "edges", "table"],
    );
    s.passes.push(PassSpec::new("all_round", around(d3(0.0, 0.0, 5.0))));
    table_with_clutter(&mut s, 0, -3.0, 1.0);
    for &(t, x) in &fins {
        let h = t / 2.0;
        s.features.push(feature(
            &format!("fin-{t}"),
            "thin_wall",
            &format!("Fin {t} mm thick: both faces at their place (within 0.08 mm), not collapsed into one sheet, not thickened, not dropped; top edges within 0.2 mm."),
            json!({"thickness_mm": t, "x": x, "height_mm": 12}),
            vec![
                Probe::height(d3(x + h, 5.0, 6.0), DVec3::X, 0.0, 0.08, "+x face"),
                Probe::height(d3(x - h, -5.0, 6.0), -DVec3::X, 0.0, 0.08, "-x face"),
                Probe::clear(d3(x, 0.0, 6.0), (h - 0.12).max(0.12), "inside the fin"),
                Probe::on(d3(x + h, 10.0, 12.0), 0.2, "top edge"),
                Probe::height(d3(x, -10.0, 12.0), DVec3::Z, 0.0, 0.1, "fin top"),
            ],
        ));
    }
    s.truth_cell_mm = 0.2;
    s
}
