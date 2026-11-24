//! Watertight solid of a thin-walled part from merged oriented scans (PLY in, PLY + STL out).
//!
//! cargo run --release -p scancraft-mesh --example solid_ply -- <in.ply> <out_dir>
//!
//! Input: binary little-endian PLY with float x,y,z,nx,ny,nz and optional uchar red,green,blue
//! (the colour gives the scan id, see `SCAN_COLOURS`). Normals point out of the material.
//!
//! Environment (all optional):
//! * `SCANCRAFT_MESH_MARKERS`: one or more CSV files separated by `;`, each with a header holding
//!   marker_id, scan, x, y, z, nx, ny, nz and optionally diameter_mm / disc_diam_mm (scan `S2` =
//!   scan 2 only, `S2+S3` or empty = all scans). Copies of one sticker are merged.
//! * `SCAN_COLOURS`: `r,g,b=id;r,g,b=id;...` (default: the three merged Revo scans).
//! * `SCAN_PRIORITY`: scan ids from the most trusted down, e.g. `1` (default: none).
//! * `SHARP_SCANS`: scans that get the narrow normal kernel, e.g. `1` (default: none).
//! * `CELL` (mm, 0.4), `BAND` (mm, 3), `SCREEN` (0.3), `CONF_MEASURED` (1), `CONF_SHARP` (3), `SMOOTH_SHARP` (1),
//!   `SMOOTH_MEASURED` (2), `SMOOTH_SOFT` (3), `SMOOTH_COMPLETED` (4), `REFINE` (mm, 0.8, 0 = off), `CUT_R` (6.5), `WALL` (measured), `OFFSURF`
//!   (0.15 mm, 0 = off), `DETECT` (1), `COMPLETION` (1), `GHOST` (1), `HOLES` (1), `BRIDGE` (span mm, 40, 0 = off), `FAIR` (1), `FAIR_RMS` (mm, 0.1), `REORIENT_DEG` (10: a listed marker normal more than this off the data takes the data's direction, 180 = sign only),
//!   `REPAIR` (rounds, 4).
//! * `SAMPLES=1` writes samples.ply (solve input coloured by kind), `DUMP_INSIDE=1` writes
//!   inside_nodes.bin, `DECIMATE=ratio` decimates the result, `DISPLAY=ratio` writes a decimated
//!   copy solid_display.ply for viewers.
//!
//! Writes `solid.ply` (binary; vertex normals and colours by origin: grey measured, blue marker
//! fill, green hole fill, orange completed), `solid.stl` and `report.json` to `out_dir`.

use glam::Vec3;
use scancraft_core::Mesh;
use scancraft_mesh::solid::{build_solid, Marker, MarkerSource, PointKind, Sample, SolidConfig, SolidPoint};
use scancraft_mesh::{MeshDecimator, Mesher};
use std::io::{BufWriter, Read, Write};
use std::time::Instant;

fn env_f(k: &str) -> Option<f32> {
    std::env::var(k).ok().and_then(|v| v.parse().ok())
}
fn env_b(k: &str, d: bool) -> bool {
    std::env::var(k).map(|v| v != "0").unwrap_or(d)
}

fn scan_colours() -> Vec<([u8; 3], u8)> {
    let spec = std::env::var("SCAN_COLOURS").unwrap_or_else(|_| "110,160,230=1;90,200,110=2;240,150,60=3".into());
    spec.split(';')
        .filter_map(|e| {
            let (c, id) = e.split_once('=')?;
            let v: Vec<u8> = c.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (v.len() == 3).then(|| ([v[0], v[1], v[2]], id.trim().parse().unwrap_or(0)))
        })
        .collect()
}

fn read_ply(path: &str) -> Vec<SolidPoint> {
    let mut data = Vec::new();
    std::fs::File::open(path).expect("open input").read_to_end(&mut data).unwrap();
    let end = data.windows(11).position(|w| w == b"end_header\n").expect("PLY header") + 11;
    let header = String::from_utf8_lossy(&data[..end]).to_string();
    assert!(header.contains("binary_little_endian"), "only binary little-endian PLY is supported");
    let mut count = 0usize;
    let mut props: Vec<(String, usize)> = Vec::new();
    let mut in_vertex = false;
    for l in header.lines() {
        let t: Vec<&str> = l.split_whitespace().collect();
        match t.as_slice() {
            ["element", "vertex", n] => {
                count = n.parse().unwrap();
                in_vertex = true;
            }
            ["element", ..] => in_vertex = false,
            ["property", ty, name] if in_vertex => {
                let size = match *ty {
                    "float" | "float32" | "int" | "int32" | "uint" | "uint32" => 4,
                    "double" | "float64" => 8,
                    "uchar" | "uint8" | "char" | "int8" => 1,
                    "short" | "ushort" | "int16" | "uint16" => 2,
                    _ => panic!("unsupported property type {ty}"),
                };
                props.push((name.to_string(), size));
            }
            _ => {}
        }
    }
    let stride: usize = props.iter().map(|p| p.1).sum();
    let off = |n: &str| {
        let mut o = 0;
        for (name, s) in &props {
            if name == n {
                return Some(o);
            }
            o += s;
        }
        None
    };
    let (ox, oy, oz) = (off("x").unwrap(), off("y").unwrap(), off("z").unwrap());
    let (onx, ony, onz) = (off("nx").expect("normals required"), off("ny").unwrap(), off("nz").unwrap());
    let (or, og, ob) = (off("red"), off("green"), off("blue"));
    let colours = scan_colours();
    let f = |b: &[u8], o: usize| f32::from_le_bytes(b[o..o + 4].try_into().unwrap());
    (0..count)
        .map(|i| {
            let b = &data[end + i * stride..end + (i + 1) * stride];
            let scan = match (or, og, ob) {
                (Some(r), Some(g), Some(bb)) => {
                    let c = [b[r], b[g], b[bb]];
                    colours.iter().find(|e| e.0 == c).map_or(0, |e| e.1)
                }
                _ => 0,
            };
            SolidPoint {
                position: Vec3::new(f(b, ox), f(b, oy), f(b, oz)),
                normal: Vec3::new(f(b, onx), f(b, ony), f(b, onz)),
                scan,
            }
        })
        .collect()
}

/// Splits one CSV line, honouring double quotes.
fn csv_fields(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut quoted = false;
    for c in line.chars() {
        match c {
            '"' => quoted = !quoted,
            ',' if !quoted => out.push(std::mem::take(&mut cur)),
            _ => cur.push(c),
        }
    }
    out.push(cur);
    out
}

fn read_markers(path: &str) -> Vec<Marker> {
    let text = std::fs::read_to_string(path).expect("read markers");
    let mut lines = text.lines();
    let head = csv_fields(lines.next().unwrap());
    let col = |n: &str| head.iter().position(|h| h.trim() == n);
    let need = |n: &str| col(n).unwrap_or_else(|| panic!("{path}: column {n}"));
    let (ci, cs, cx, cy, cz, cnx, cny, cnz) =
        (need("marker_id"), need("scan"), need("x"), need("y"), need("z"), need("nx"), need("ny"), need("nz"));
    let cd = col("diameter_mm").or_else(|| col("disc_diam_mm"));
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let v = csv_fields(l);
            let num = |i: usize| v[i].trim().parse::<f32>().unwrap();
            let scan = v[cs].trim();
            let scan = if scan.contains('+') || scan.is_empty() { None } else { scan.trim_start_matches('S').parse().ok() };
            Marker {
                id: v[ci].clone(),
                center: Vec3::new(num(cx), num(cy), num(cz)),
                normal: Vec3::new(num(cnx), num(cny), num(cnz)),
                scan,
                source: MarkerSource::List,
                radius_mm: cd.and_then(|c| v.get(c)).and_then(|s| s.trim().parse::<f32>().ok()).map_or(0.0, |d| 0.5 * d),
                score: 0.0,
            }
        })
        .collect()
}

fn write_ply(mesh: &Mesh, path: &str) {
    let mut w = BufWriter::new(std::fs::File::create(path).expect("create output"));
    write!(
        w,
        "ply\nformat binary_little_endian 1.0\ncomment ScanCraft solid (colour = vertex origin)\nelement vertex {}\n\
         property float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\n\
         property uchar red\nproperty uchar green\nproperty uchar blue\nelement face {}\nproperty list uchar int vertex_indices\nend_header\n",
        mesh.vertices.len(),
        mesh.triangles.len()
    )
    .unwrap();
    for v in &mesh.vertices {
        for c in v.position.iter().chain(&v.normal) {
            w.write_all(&c.to_le_bytes()).unwrap();
        }
        w.write_all(&[v.color.r, v.color.g, v.color.b]).unwrap();
    }
    for t in &mesh.triangles {
        w.write_all(&[3u8]).unwrap();
        for i in t.indices {
            w.write_all(&(i as i32).to_le_bytes()).unwrap();
        }
    }
}

fn write_stl(mesh: &Mesh, path: &str) {
    let mut w = BufWriter::new(std::fs::File::create(path).expect("create stl"));
    let mut head = [0u8; 80];
    head[..15].copy_from_slice(b"ScanCraft solid");
    w.write_all(&head).unwrap();
    w.write_all(&(mesh.triangles.len() as u32).to_le_bytes()).unwrap();
    for t in &mesh.triangles {
        let p: Vec<Vec3> = t.indices.iter().map(|&i| Vec3::from_array(mesh.vertices[i as usize].position)).collect();
        let n = (p[1] - p[0]).cross(p[2] - p[0]).normalize_or_zero();
        for v in std::iter::once(n).chain(p) {
            for c in v.to_array() {
                w.write_all(&c.to_le_bytes()).unwrap();
            }
        }
        w.write_all(&[0u8, 0]).unwrap();
    }
}

fn write_samples(samples: &[Sample], path: &str) {
    let mut w = BufWriter::new(std::fs::File::create(path).unwrap());
    write!(
        w,
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\n\
         property float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
        samples.len()
    )
    .unwrap();
    for s in samples {
        for c in s.position.to_array().iter().chain(&s.normal.to_array()) {
            w.write_all(&c.to_le_bytes()).unwrap();
        }
        let col = match s.kind {
            PointKind::Measured => [180u8, 180, 180],
            PointKind::MarkerFill => [230, 40, 40],
            PointKind::Completion => [40, 90, 230],
            PointKind::HoleFill => [40, 200, 60],
        };
        w.write_all(&col).unwrap();
    }
}

#[cfg(windows)]
fn peak_working_set_mb() -> Option<f64> {
    #[repr(C)]
    struct Pmc {
        cb: u32,
        page_fault_count: u32,
        peak_working_set: usize,
        working_set: usize,
        quota_peak_paged: usize,
        quota_paged: usize,
        quota_peak_nonpaged: usize,
        quota_nonpaged: usize,
        pagefile: usize,
        peak_pagefile: usize,
    }
    extern "system" {
        fn GetCurrentProcess() -> isize;
        fn K32GetProcessMemoryInfo(process: isize, pmc: *mut Pmc, cb: u32) -> i32;
    }
    let mut p: Pmc = unsafe { std::mem::zeroed() };
    p.cb = std::mem::size_of::<Pmc>() as u32;
    let ok = unsafe { K32GetProcessMemoryInfo(GetCurrentProcess(), &mut p, p.cb) };
    (ok != 0).then(|| p.peak_working_set as f64 / 1048576.0)
}
#[cfg(not(windows))]
fn peak_working_set_mb() -> Option<f64> {
    None
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: solid_ply <in.ply> <out_dir>";
    let input = args.get(1).expect(usage);
    let out = args.get(2).expect(usage);
    std::fs::create_dir_all(out).unwrap();

    let t0 = Instant::now();
    let pts = read_ply(input);
    let read_s = t0.elapsed().as_secs_f64();
    let mut per_scan = [0usize; 8];
    for p in &pts {
        per_scan[(p.scan as usize).min(7)] += 1;
    }
    println!("read {} points in {:.2} s, per scan {:?}", pts.len(), read_s, &per_scan[..4]);

    let markers: Vec<Marker> = std::env::var("SCANCRAFT_MESH_MARKERS")
        .map(|v| v.split(';').filter(|p| !p.trim().is_empty()).flat_map(|p| read_markers(p.trim())).collect())
        .unwrap_or_default();
    println!("{} listed marker sites", markers.len());

    let mut cfg = SolidConfig::default();
    let set = |v: &mut f32, k: &str| {
        if let Some(x) = env_f(k) {
            *v = x;
        }
    };
    set(&mut cfg.cell_mm, "CELL");
    set(&mut cfg.band_mm, "BAND");
    set(&mut cfg.screen, "SCREEN");
    set(&mut cfg.measured_confidence, "CONF_MEASURED");
    set(&mut cfg.sharp_confidence, "CONF_SHARP");
    set(&mut cfg.markers.cut_radius_mm, "CUT_R");
    set(&mut cfg.markers.fair_rms_max_mm, "FAIR_RMS");
    set(&mut cfg.markers.reorient_min_deg, "REORIENT_DEG");
    set(&mut cfg.off_surface_mm, "OFFSURF");
    set(&mut cfg.refine_radius_mm, "REFINE");
    let seti = |v: &mut usize, k: &str| {
        if let Some(x) = env_f(k) {
            *v = x as usize;
        }
    };
    set(&mut cfg.smooth_sharp, "SMOOTH_SHARP");
    set(&mut cfg.smooth_measured, "SMOOTH_MEASURED");
    set(&mut cfg.smooth_soft, "SMOOTH_SOFT");
    seti(&mut cfg.smooth_completed, "SMOOTH_COMPLETED");
    seti(&mut cfg.repair_rounds, "REPAIR");
    if let Some(v) = env_f("WALL") {
        cfg.wall_mm = Some(v);
    }
    let ids = |k: &str| std::env::var(k).ok().map(|v| v.split(',').filter_map(|s| s.trim().parse().ok()).collect::<Vec<u8>>());
    if let Some(v) = ids("SCAN_PRIORITY") {
        cfg.scan_priority = v;
    }
    if let Some(v) = ids("SHARP_SCANS") {
        cfg.sharp_scans = v;
    }
    if !env_b("DETECT", true) {
        cfg.detect = None;
    }
    cfg.completion = env_b("COMPLETION", true);
    if let (Some(h), Some(v)) = (cfg.hole_fill.as_mut(), env_f("BRIDGE")) {
        h.bridge_span_mm = v;
    }
    if !env_b("HOLES", true) {
        cfg.hole_fill = None;
    }
    cfg.ghost_filter = env_b("GHOST", true);
    cfg.markers.fair_mesh = env_b("FAIR", true);
    cfg.keep_inside_nodes = env_b("DUMP_INSIDE", false);

    let res = build_solid(&pts, &markers, &cfg);
    if let Some((o, h, nodes)) = &res.inside_nodes {
        // Header: origin xyz (f32), h (f32), count (u32); then i32 triples.
        let mut w = BufWriter::new(std::fs::File::create(format!("{out}/inside_nodes.bin")).unwrap());
        for c in [o.x, o.y, o.z, *h] {
            w.write_all(&c.to_le_bytes()).unwrap();
        }
        w.write_all(&(nodes.len() as u32).to_le_bytes()).unwrap();
        for n in nodes {
            for c in n {
                w.write_all(&c.to_le_bytes()).unwrap();
            }
        }
    }
    let mut mesh = res.mesh;
    let rep = res.report;
    for (k, v) in &rep.stages {
        println!("  {k:<16} {v:8.2} s");
    }
    println!(
        "wall {:.3} mm, outliers {}+{} off-surface, covered layers {}, ghosts {}, marker sites {} ({} detected, {} faired), cut {}, fill {}, completion {}, holes {} ({} pts, +{} completion)",
        rep.wall_median_mm,
        rep.dropped_outliers,
        rep.dropped_off_surface,
        rep.dropped_covered_layers,
        rep.dropped_ghosts,
        rep.marker_sites.len(),
        rep.markers_detected.len(),
        rep.sites_faired,
        rep.marker_cut_points,
        rep.marker_fill_points,
        rep.completion_points,
        rep.holes_filled,
        rep.hole_fill_points,
        rep.completion_points_2
    );
    println!(
        "solve: {} samples, {} blocks, {} unknowns, CG {} (+{} coarse) iterations, rel residual {:.2e}, mean χ at samples {:.4}",
        rep.samples, rep.band_blocks, rep.unknowns, rep.cg_iterations, rep.cg_coarse_iterations, rep.cg_rel_residual, rep.mean_chi_measured
    );
    let t = &rep.topology;
    let f = rep.origin_fractions();
    println!(
        "mesh: {} vertices, {} triangles, watertight {}, manifold {}, components {}, genus {}, volume {:.0} mm³ | slivers collapsed {} | self-intersections {} -> {}, folds {} -> {} | origin measured {:.1} % marker {:.1} % hole {:.1} % completed {:.1} %",
        t.vertices,
        t.triangles,
        t.watertight,
        t.manifold,
        t.components.len(),
        t.genus_total(),
        t.total_volume_mm3,
        rep.sliver_collapses,
        rep.defects_before.0,
        rep.defects_after.0,
        rep.defects_before.1,
        rep.defects_after.1,
        100.0 * f[0],
        100.0 * f[1],
        100.0 * f[2],
        100.0 * f[3]
    );

    if let Some(r) = env_f("DECIMATE") {
        let t = Instant::now();
        mesh = MeshDecimator::new(r).decimate(&mesh);
        mesh.recompute_normals();
        println!("decimated to {} triangles in {:.2?}: {:?}", mesh.triangles.len(), t.elapsed(), Mesher::analyze(&mesh));
    }

    let t = Instant::now();
    write_ply(&mesh, &format!("{out}/solid.ply"));
    write_stl(&mesh, &format!("{out}/solid.stl"));
    if env_b("SAMPLES", false) {
        write_samples(&res.samples, &format!("{out}/samples.ply"));
    }
    if let Some(r) = env_f("DISPLAY") {
        let td = Instant::now();
        let mut d = MeshDecimator::new(r).decimate(&mesh);
        d.recompute_normals();
        write_ply(&d, &format!("{out}/solid_display.ply"));
        println!("display copy: {} triangles in {:.2?}", d.triangles.len(), td.elapsed());
    }
    let write_s = t.elapsed().as_secs_f64();
    let peak = peak_working_set_mb();
    let mut json = rep.to_json();
    json.truncate(json.trim_end().len() - 1);
    json.push_str(&format!(
        ",\n  \"read_s\": {read_s:.3},\n  \"write_s\": {write_s:.3},\n  \"wall_clock_s\": {:.3},\n  \"peak_working_set_mb\": {},\n  \"config\": \"{}\"\n}}\n",
        t0.elapsed().as_secs_f64(),
        peak.map_or("null".into(), |p| format!("{p:.0}")),
        format!("{cfg:?}").replace('\\', "/").replace('"', "'")
    ));
    std::fs::write(format!("{out}/report.json"), json).unwrap();
    println!("total {:.1} s, peak working set {:?} MB", t0.elapsed().as_secs_f64(), peak.map(|p| p.round()));
}
