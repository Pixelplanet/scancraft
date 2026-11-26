//! Project folders (`docs/architecture/scan-sessions.md`, section 8):
//!
//! ```text
//! <project>/project.json            passes, modes, alignments, settings, output metadata
//! <project>/pass-<id>/keyframes.bin  the pass's keyframes (poses + packed points)
//! <project>/pass-<id>/raw.bin        optional raw recording (referenced, not written here)
//! <project>/pass-<id>/markers.json   optional marker map (pass frame, mm)
//! <project>/outputs/*.ply            derived results (re-computable)
//! ```
//!
//! `project.json` stores scan settings and device information as free-form JSON so the format
//! does not break when those structures grow; unknown fields are ignored on load.

use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use scancraft_core::Transform;
use serde::{Deserialize, Serialize};

use crate::IoError;

pub const PROJECT_FILE: &str = "project.json";
pub const FORMAT_VERSION: u32 = 1;
const KEYFRAMES_MAGIC: &[u8; 4] = b"SCKF";
const KEYFRAMES_VERSION: u32 = 1;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectFile {
    pub format: u32,
    pub name: String,
    /// The pass that defines the project frame.
    pub reference_pass: Option<u32>,
    pub next_pass_id: u32,
    pub passes: Vec<PassRecord>,
    #[serde(default)]
    pub outputs: Vec<OutputRecord>,
    /// Application settings worth restoring (free-form).
    #[serde(default)]
    pub settings: serde_json::Value,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PassRecord {
    pub id: u32,
    pub name: String,
    /// Scan mode and settings the pass was captured with.
    #[serde(default)]
    pub scan_config: serde_json::Value,
    /// Device (and calibration) snapshot.
    #[serde(default)]
    pub device: serde_json::Value,
    #[serde(default)]
    pub frames: u64,
    /// project <- pass.
    pub transform: Transform,
    pub alignment: AlignmentRecord,
    pub weight: f32,
    pub included: bool,
    #[serde(default = "yes")]
    pub visible: bool,
    /// How the pass was started relative to the previous ones: "moved" or "same_position".
    #[serde(default)]
    pub start: String,
    /// Keyframe file, relative to the project folder.
    pub keyframes: String,
    /// Raw sensor recording, if one was kept (relative or absolute).
    #[serde(default)]
    pub raw_recording: Option<String>,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct AlignmentRecord {
    /// "reference", "aligned" or "unaligned".
    pub status: String,
    /// "reference", "auto", "point_pairs", "markers", "same_position", "refined".
    #[serde(default)]
    pub method: Option<String>,
    #[serde(default)]
    pub overlap: Option<f32>,
    #[serde(default)]
    pub rmse_mm: Option<f32>,
    #[serde(default)]
    pub visibility_conflict: Option<f32>,
    /// The rest of the alignment's quality (all optional: older projects lack them and older
    /// versions of the app ignore them).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inliers: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub below_support: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub slide_mm: Option<f32>,
    /// The two faces of a wall found by the alignment, when it found them.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shell: Option<ShellRecord>,
}

/// Facing-apart surfaces measured by an alignment (`scancraft_align::quality::Shell`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShellRecord {
    pub pairs: u64,
    pub fraction: f32,
    pub offset_mm: f32,
    pub iqr_mm: f32,
    pub piercing: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OutputRecord {
    /// "merged", "cleaned" or "mesh".
    pub kind: String,
    /// File relative to the project folder.
    pub file: String,
    #[serde(default)]
    pub points: usize,
    /// Passes it was computed from.
    #[serde(default)]
    pub passes: Vec<u32>,
}

/// One keyframe as stored in `keyframes.bin`: pose (pass <- sensor), fit, and sensor-frame points
/// with normals quantised to `i8`.
#[derive(Debug, Clone, PartialEq)]
pub struct KeyframeRecord {
    pub id: u64,
    pub rotation: [f32; 4],
    pub translation: [f32; 3],
    pub rmse_mm: f32,
    pub visibility_conflict: f32,
    pub new_points: u64,
    pub discarded: bool,
    pub imu: Option<[f32; 4]>,
    pub centroid: [f32; 3],
    pub points: Vec<([f32; 3], [i8; 3])>,
}

/// Keyframes of each pass, by pass id.
pub type PassKeyframes = Vec<(u32, Vec<KeyframeRecord>)>;

pub fn pass_dir(id: u32) -> String {
    format!("pass-{id}")
}

/// The keyframe file of a pass, relative to the project folder.
pub fn keyframes_path(id: u32) -> String {
    format!("{}/keyframes.bin", pass_dir(id))
}

/// A pass's marker map file, relative to the project folder. Optional: a pass without markers has
/// none, and versions that do not know it ignore it.
pub fn markers_path(id: u32) -> String {
    format!("{}/markers.json", pass_dir(id))
}

const MARKERS_VERSION: u32 = 1;

/// One marker sticker of a pass's marker map, in the pass frame (mm).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MarkerRecord {
    pub centre: [f32; 3],
    /// Unit normal out of the surface, when the source gives one (Revo's marker file does, the
    /// tracker's map does not).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub normal: Option<[f32; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radius_mm: Option<f32>,
}

impl MarkerRecord {
    /// The record as it can be stored: `None` when the centre is not finite, a non-finite normal
    /// or radius left out. JSON has no NaN or infinity (serde_json writes them as `null`).
    pub fn finite(self) -> Option<Self> {
        self.centre.iter().all(|v| v.is_finite()).then(|| MarkerRecord {
            centre: self.centre,
            normal: self.normal.filter(|n| n.iter().all(|v| v.is_finite())),
            radius_mm: self.radius_mm.filter(|r| r.is_finite()),
        })
    }
}

#[derive(Serialize)]
struct MarkersFile {
    format: u32,
    markers: Vec<MarkerRecord>,
}

/// What is read back: the entries one by one, so one damaged entry does not lose the others.
#[derive(Deserialize)]
struct MarkersFileIn {
    format: u32,
    markers: Vec<serde_json::Value>,
}

/// Writes a marker map; entries with a non-finite centre are left out (see [`MarkerRecord::finite`]).
pub fn write_markers(path: &Path, markers: &[MarkerRecord]) -> Result<(), IoError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let markers = markers.iter().filter_map(|m| m.finite()).collect();
    let json = serde_json::to_string_pretty(&MarkersFile { format: MARKERS_VERSION, markers }).map_err(|e| IoError::Parse(e.to_string()))?;
    fs::write(path, json)?;
    Ok(())
}

/// Reads a marker map. An entry that cannot be read (for example a `null` coordinate) or has a
/// non-finite centre is dropped, and the other entries are kept.
pub fn read_markers(path: &Path) -> Result<Vec<MarkerRecord>, IoError> {
    let text = fs::read_to_string(path)?;
    let file: MarkersFileIn = serde_json::from_str(&text).map_err(|e| IoError::Parse(format!("{}: {e}", path.display())))?;
    if file.format > MARKERS_VERSION {
        return Err(IoError::Unsupported(format!("{}: marker map format {}", path.display(), file.format)));
    }
    let total = file.markers.len();
    let markers: Vec<MarkerRecord> = file.markers.into_iter().filter_map(|v| serde_json::from_value::<MarkerRecord>(v).ok()?.finite()).collect();
    if markers.len() < total {
        log::warn!("{}: {} of {total} markers could not be read and are left out", path.display(), total - markers.len());
    }
    Ok(markers)
}

pub fn write_keyframes(path: &Path, keyframes: &[KeyframeRecord]) -> Result<(), IoError> {
    let mut w = BufWriter::new(File::create(path)?);
    w.write_all(KEYFRAMES_MAGIC)?;
    w.write_u32::<LittleEndian>(KEYFRAMES_VERSION)?;
    w.write_u64::<LittleEndian>(keyframes.len() as u64)?;
    for k in keyframes {
        w.write_u64::<LittleEndian>(k.id)?;
        for v in k.rotation.iter().chain(&k.translation) {
            w.write_f32::<LittleEndian>(*v)?;
        }
        w.write_f32::<LittleEndian>(k.rmse_mm)?;
        w.write_f32::<LittleEndian>(k.visibility_conflict)?;
        w.write_u64::<LittleEndian>(k.new_points)?;
        w.write_u8(k.discarded as u8 | (k.imu.is_some() as u8) << 1)?;
        for v in k.imu.unwrap_or([0.0, 0.0, 0.0, 1.0]).iter().chain(&k.centroid) {
            w.write_f32::<LittleEndian>(*v)?;
        }
        w.write_u64::<LittleEndian>(k.points.len() as u64)?;
        for (p, n) in &k.points {
            for v in p {
                w.write_f32::<LittleEndian>(*v)?;
            }
            for v in n {
                w.write_i8(*v)?;
            }
        }
    }
    w.flush()?;
    Ok(())
}

pub fn read_keyframes(path: &Path) -> Result<Vec<KeyframeRecord>, IoError> {
    let mut r = BufReader::new(File::open(path)?);
    let mut magic = [0u8; 4];
    r.read_exact(&mut magic)?;
    if &magic != KEYFRAMES_MAGIC {
        return Err(IoError::Parse(format!("{} is not a keyframe file", path.display())));
    }
    let version = r.read_u32::<LittleEndian>()?;
    if version != KEYFRAMES_VERSION {
        return Err(IoError::Parse(format!("unsupported keyframe file version {version}")));
    }
    let count = r.read_u64::<LittleEndian>()?;
    let mut out = Vec::with_capacity(count.min(1 << 20) as usize);
    let f = |r: &mut BufReader<File>, n: usize| -> Result<Vec<f32>, IoError> { (0..n).map(|_| r.read_f32::<LittleEndian>().map_err(IoError::from)).collect() };
    for _ in 0..count {
        let id = r.read_u64::<LittleEndian>()?;
        let pose = f(&mut r, 7)?;
        let rmse_mm = r.read_f32::<LittleEndian>()?;
        let visibility_conflict = r.read_f32::<LittleEndian>()?;
        let new_points = r.read_u64::<LittleEndian>()?;
        let flags = r.read_u8()?;
        let rest = f(&mut r, 7)?;
        let n = r.read_u64::<LittleEndian>()? as usize;
        let mut points = Vec::with_capacity(n.min(1 << 24));
        for _ in 0..n {
            let p = [r.read_f32::<LittleEndian>()?, r.read_f32::<LittleEndian>()?, r.read_f32::<LittleEndian>()?];
            let q = [r.read_i8()?, r.read_i8()?, r.read_i8()?];
            points.push((p, q));
        }
        out.push(KeyframeRecord {
            id,
            rotation: [pose[0], pose[1], pose[2], pose[3]],
            translation: [pose[4], pose[5], pose[6]],
            rmse_mm,
            visibility_conflict,
            new_points,
            discarded: flags & 1 != 0,
            imu: (flags & 2 != 0).then(|| [rest[0], rest[1], rest[2], rest[3]]),
            centroid: [rest[4], rest[5], rest[6]],
            points,
        });
    }
    Ok(out)
}

/// Writes `project.json` and every pass's keyframes into `dir` (created if needed). The JSON is
/// written last, through a temporary file, so an interrupted save leaves the previous project
/// description in place.
pub fn save_project(dir: &Path, project: &ProjectFile, keyframes: &[(u32, Vec<KeyframeRecord>)]) -> Result<(), IoError> {
    fs::create_dir_all(dir)?;
    for (id, kfs) in keyframes {
        let record = project.passes.iter().find(|p| p.id == *id).ok_or_else(|| IoError::Parse(format!("pass {id} is not in the project")))?;
        let path = dir.join(&record.keyframes);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        write_keyframes(&path, kfs)?;
    }
    let json = serde_json::to_string_pretty(project).map_err(|e| IoError::Parse(e.to_string()))?;
    let tmp = dir.join(format!("{PROJECT_FILE}.tmp"));
    fs::write(&tmp, json)?;
    fs::rename(&tmp, dir.join(PROJECT_FILE))?;
    Ok(())
}

/// Reads `project.json` and every pass's keyframes.
pub fn load_project(dir: &Path) -> Result<(ProjectFile, PassKeyframes), IoError> {
    let text = fs::read_to_string(dir.join(PROJECT_FILE))?;
    let project: ProjectFile = serde_json::from_str(&text).map_err(|e| IoError::Parse(format!("{PROJECT_FILE}: {e}")))?;
    if project.format > FORMAT_VERSION {
        return Err(IoError::Parse(format!("the project was saved by a newer version (format {})", project.format)));
    }
    let mut keyframes = Vec::with_capacity(project.passes.len());
    for p in &project.passes {
        keyframes.push((p.id, read_keyframes(&dir.join(&p.keyframes))?));
    }
    Ok((project, keyframes))
}

/// Whether `dir` holds a project.
pub fn is_project(dir: &Path) -> bool {
    dir.join(PROJECT_FILE).is_file()
}

/// Resolves a path stored in the project relative to its folder.
pub fn resolve(dir: &Path, stored: &str) -> PathBuf {
    let p = Path::new(stored);
    if p.is_absolute() { p.to_path_buf() } else { dir.join(p) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, Vec3};

    fn keyframe(id: u64, n: usize) -> KeyframeRecord {
        KeyframeRecord {
            id,
            rotation: Quat::from_rotation_z(0.1 * id as f32).to_array(),
            translation: [id as f32, -2.5, 300.0],
            rmse_mm: 0.21,
            visibility_conflict: 0.03,
            new_points: 1234,
            discarded: id.is_multiple_of(3),
            imu: id.is_multiple_of(2).then_some([0.0, 0.0, 0.38268343, 0.9238795]),
            centroid: [1.0, 250.0, -3.0],
            points: (0..n).map(|i| ([i as f32 * 0.5, 250.0, -(i as f32)], [0, -127, (i % 7) as i8])).collect(),
        }
    }

    #[test]
    fn project_round_trips() {
        let dir = std::env::temp_dir().join(format!("scancraft-project-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        let pass = |id: u32, status: &str| PassRecord {
            id,
            name: format!("Pass {id}"),
            scan_config: serde_json::json!({ "light": "FullFieldHd", "accuracy": "Standard" }),
            device: serde_json::json!({ "model": "POP 4" }),
            frames: 321,
            transform: Transform { translation: Vec3::new(10.0, -3.0, 2.0), rotation: Quat::from_rotation_x(3.0), scale: Vec3::ONE },
            alignment: AlignmentRecord { status: status.into(), method: Some("auto".into()), overlap: Some(0.4), rmse_mm: Some(0.2), visibility_conflict: Some(0.01), ..Default::default() },
            weight: 2.0,
            included: id != 3,
            visible: true,
            start: "moved".into(),
            keyframes: keyframes_path(id),
            raw_recording: None,
        };
        let project = ProjectFile {
            format: FORMAT_VERSION,
            name: "Bracket".into(),
            reference_pass: Some(1),
            next_pass_id: 4,
            passes: vec![pass(1, "reference"), pass(2, "aligned"), pass(3, "unaligned")],
            outputs: vec![OutputRecord { kind: "merged".into(), file: "outputs/merged.ply".into(), points: 42, passes: vec![1, 2] }],
            settings: serde_json::json!({ "min_mm": 150.0 }),
        };
        let kfs = vec![(1, vec![keyframe(0, 10), keyframe(1, 0)]), (2, vec![keyframe(5, 100)]), (3, Vec::new())];
        save_project(&dir, &project, &kfs).unwrap();
        assert!(is_project(&dir));
        let (loaded, loaded_kfs) = load_project(&dir).unwrap();
        assert_eq!(loaded, project);
        assert_eq!(loaded_kfs, kfs);
        // Saving again over it works (the JSON is replaced, not appended).
        save_project(&dir, &loaded, &loaded_kfs).unwrap();
        assert_eq!(load_project(&dir).unwrap().0, project);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn marker_maps_round_trip() {
        let path = std::env::temp_dir().join(format!("scancraft-markers-{}", std::process::id())).join(markers_path(7));
        let markers = vec![
            MarkerRecord { centre: [1.0, -2.5, 300.25], normal: None, radius_mm: None },
            MarkerRecord { centre: [4.0, 5.0, 6.0], normal: Some([0.0, 0.6, -0.8]), radius_mm: Some(3.05) },
        ];
        write_markers(&path, &markers).unwrap();
        assert_eq!(read_markers(&path).unwrap(), markers);
        // A tracker marker stores its centre only.
        let text = fs::read_to_string(&path).unwrap();
        assert_eq!(text.matches("radius_mm").count(), 1, "{text}");
        // A newer format and a damaged file are errors, not a panic or an empty map.
        fs::write(&path, r#"{"format": 9, "markers": []}"#).unwrap();
        assert!(matches!(read_markers(&path), Err(IoError::Unsupported(_))));
        fs::write(&path, "{").unwrap();
        assert!(read_markers(&path).is_err());
        let _ = fs::remove_dir_all(path.parent().unwrap().parent().unwrap());
    }

    #[test]
    fn a_non_finite_marker_does_not_lose_the_others() {
        let path = std::env::temp_dir().join(format!("scancraft-markers-nan-{}", std::process::id())).join(markers_path(3));
        let good = MarkerRecord { centre: [1.0, 2.0, 3.0], normal: Some([0.0, 0.0, 1.0]), radius_mm: Some(3.0) };
        let markers = vec![
            good,
            MarkerRecord { centre: [f32::NAN, 1.0, 2.0], normal: None, radius_mm: None },
            MarkerRecord { centre: [f32::INFINITY, 1.0, 2.0], normal: None, radius_mm: None },
            MarkerRecord { centre: [4.0, 5.0, 6.0], normal: Some([f32::NAN, 0.0, 1.0]), radius_mm: Some(f32::INFINITY) },
        ];
        write_markers(&path, &markers).unwrap();
        let text = fs::read_to_string(&path).unwrap();
        assert!(!text.contains("null"), "{text}");
        let expected = vec![good, MarkerRecord { centre: [4.0, 5.0, 6.0], normal: None, radius_mm: None }];
        assert_eq!(read_markers(&path).unwrap(), expected);
        // A file written before this fix (serde_json's `null` for NaN) keeps its readable entries.
        fs::write(&path, r#"{"format": 1, "markers": [{"centre": [null, 1.0, 2.0]}, {"centre": [1.0, 2.0, 3.0]}, {"centre": [1e300, 0, 0]}]}"#).unwrap();
        assert_eq!(read_markers(&path).unwrap(), vec![MarkerRecord { centre: [1.0, 2.0, 3.0], normal: None, radius_mm: None }]);
        let _ = fs::remove_dir_all(path.parent().unwrap().parent().unwrap());
    }

    #[test]
    fn rejects_foreign_files() {
        let path = std::env::temp_dir().join(format!("scancraft-not-keyframes-{}.bin", std::process::id()));
        fs::write(&path, b"PLY\n....").unwrap();
        assert!(read_keyframes(&path).is_err());
        let _ = fs::remove_file(path);
    }
}
