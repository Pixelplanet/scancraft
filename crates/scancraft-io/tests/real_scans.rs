//! Imports real scans from a local folder (not in the repository).
//!
//! Run with: `SCANCRAFT_IMPORT_DATA=<dir> cargo test -p scancraft-io --release --test real_scans -- --ignored --nocapture`
//!
//! Every `.ply`, `.obj` and `.stl` file under the folder (recursively) is imported twice: once
//! as stored, once with its normals ignored so that the estimation runs on real data. The test
//! prints counts and timings and checks that estimated normals agree with the stored ones.

use scancraft_io::{import_bytes, ImportOptions, NormalSource};
use std::path::{Path, PathBuf};
use std::time::Instant;

fn scan_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            scan_files(&p, out);
        } else if matches!(
            p.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref(),
            Some("ply" | "obj" | "stl")
        ) {
            out.push(p);
        }
    }
}

/// Removes the normal properties from a binary PLY whose vertices are `x y z nx ny nz` floats.
fn strip_normals(bytes: &[u8]) -> Option<Vec<u8>> {
    let end = bytes.windows(11).position(|w| w == b"end_header\n")? + 11;
    let header = std::str::from_utf8(&bytes[..end]).ok()?;
    let props: Vec<&str> = header.lines().filter(|l| l.starts_with("property")).collect();
    let wanted = ["x", "y", "z", "nx", "ny", "nz"].map(|n| format!("property float {n}"));
    if props.len() != 6 || props.iter().zip(&wanted).any(|(a, b)| *a != b) || header.contains("element face") {
        return None;
    }
    let mut out: Vec<u8> = header
        .lines()
        .filter(|l| !l.starts_with("property float n"))
        .flat_map(|l| l.bytes().chain(std::iter::once(b'\n')))
        .collect();
    for rec in bytes[end..].chunks_exact(24) {
        out.extend_from_slice(&rec[..12]);
    }
    Some(out)
}

#[test]
#[ignore = "needs SCANCRAFT_IMPORT_DATA (a folder with scans)"]
fn import_real_scans() {
    let Some(dir) = std::env::var_os("SCANCRAFT_IMPORT_DATA") else {
        eprintln!("SCANCRAFT_IMPORT_DATA not set; nothing to do");
        return;
    };
    let mut files = Vec::new();
    scan_files(Path::new(&dir), &mut files);
    files.sort();
    assert!(!files.is_empty(), "no scan files found");

    for path in files {
        let name = path.strip_prefix(&dir).unwrap_or(&path).display().to_string();
        let bytes = std::fs::read(&path).unwrap();
        let ext = path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase);
        let t = Instant::now();
        let scan = import_bytes(&bytes, ext.as_deref(), &ImportOptions::default())
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let ms = t.elapsed().as_secs_f64() * 1e3;
        let (min, max) = scan.bounds().unwrap_or(([0.0; 3], [0.0; 3]));
        println!(
            "{name}: {} points, {} faces, normals {:?}, colours {}, units {:?}, {:.1} MB in {ms:.0} ms; \
             bounds {:.0?} .. {:.0?}",
            scan.len(),
            scan.faces.as_ref().map_or(0, Vec::len),
            scan.normal_source,
            scan.colors.is_some(),
            scan.units,
            bytes.len() as f64 / 1e6,
            min,
            max
        );
        assert!(!scan.is_empty(), "{name}: no points");

        let Some(stripped) = strip_normals(&bytes) else { continue };
        let t = Instant::now();
        let est = import_bytes(&stripped, Some("ply"), &ImportOptions::default()).unwrap();
        let ms = t.elapsed().as_secs_f64() * 1e3;
        assert_eq!(est.normal_source, NormalSource::Estimated);
        assert_eq!(est.len(), scan.len());
        let (file_n, est_n) = (scan.normals.as_ref().unwrap(), est.normals.as_ref().unwrap());
        let mut line = 0usize; // |cos| > 0.9: the same tangent plane
        let mut same_sign = 0usize;
        for (a, b) in file_n.iter().zip(est_n) {
            let c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            line += (c.abs() > 0.9) as usize;
            same_sign += (c > 0.0) as usize;
        }
        let pct = |v: usize| 100.0 * v as f64 / scan.len() as f64;
        // The global sign of a one-sided surface is a guess, so report consistency too.
        println!(
            "    estimated normals in {ms:.0} ms: {:.1} % match the stored plane (|cos| > 0.9), \
             {:.1} % point the same way (sign consistency {:.1} %)",
            pct(line),
            pct(same_sign),
            pct(same_sign).max(100.0 - pct(same_sign))
        );
        assert!(pct(line) > 70.0, "{name}: estimated normals disagree with the stored ones");
    }
}
