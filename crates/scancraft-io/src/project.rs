//! Project folders (`docs/architecture/scan-sessions.md`, section 8; `docs/open-issues.md` 4.1):
//!
//! ```text
//! <project>/<name>.scancraft         the project file: passes, modes, alignments, settings,
//!                                    output metadata (JSON; opened by double-click)
//! <project>/pass-<id>/keyframes.bin  the pass's keyframes (poses + packed points)
//! <project>/pass-<id>/surface.ply    an imported pass's surface (app)
//! <project>/pass-<id>/cleaned.ply    a cleaned pass's surface (+ cleaned_observations.bin) (app)
//! <project>/pass-<id>/object.bin     the pass's object labels (app)
//! <project>/pass-<id>/rgb/           colour keyframes: kf-<id>.jpg + keyframes.json (app)
//! <project>/pass-<id>/raw.bin        optional raw recording (referenced, not written here)
//! <project>/pass-<id>/markers.json   optional marker map (pass frame, mm)
//! <project>/outputs/*.ply            derived results (re-computable)
//! <project>/.scancraft.lock          while the project is open ([`crate::lock`])
//! ```
//!
//! The project file stores scan settings and device information as free-form JSON so the format
//! does not break when those structures grow. Fields this version does not know (at the top level
//! and in a pass) are kept: they are read into `other` and a save writes them back, also those of
//! the file a save replaces when the caller did not read them (forward compatibility: a project
//! saved by a later version and saved again here keeps that version's additions). Every path in
//! it is relative to the project folder. One project per folder: a folder with two `.scancraft`
//! files is refused rather than guessed at, since both would share the pass data. A project file
//! keeps its file name when the project is renamed; a new folder's file is named after the
//! project ([`project_file_for_save`]).
//!
//! **Older projects** (before 2026-11-26) have `project.json` instead, with the same content;
//! they keep opening. Saving such a project writes `<name>.scancraft` and then replaces
//! `project.json` with a small *moved* stub (format [`MOVED_STUB_FORMAT`], no passes, `moved_to`
//! naming the new file). An older ScanCraft opening the folder then refuses it as "saved by a
//! newer version" instead of opening the stale copy, and this version ignores the stub when the
//! `.scancraft` file is there (and says that file is missing when it is not). The stub is kept,
//! not deleted, so that the older app's message is that one rather than "holds no project.json".
//! Should a save stop between the two steps, both files are real; the `.scancraft` file is the
//! newer one and is the one used ([`locate`]).
//!
//! Every file of a save is written to a temporary file that is flushed and renamed over the old
//! one, and the project file comes last, so an interrupted save leaves the previous project
//! description in place.
//!
//! A bundle (`<name>.scancraftz`, [`crate::bundle`]) is the same layout in one ZIP file.

use std::fs::{self, File};
use std::io::{BufReader, Read, Write};
use std::path::{Path, PathBuf};

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use scancraft_core::Transform;
use serde::{Deserialize, Serialize};

use crate::{fsutil, IoError};

/// Extension of the project file (form A), without the dot.
pub const PROJECT_EXTENSION: &str = "scancraft";
/// Extension of a project bundle (form B), without the dot.
pub const BUNDLE_EXTENSION: &str = "scancraftz";
/// The project file of folders saved before the `.scancraft` file existed.
pub const LEGACY_PROJECT_FILE: &str = "project.json";
pub const FORMAT_VERSION: u32 = 1;
/// The format number written into a legacy `project.json` that now only points at the
/// `.scancraft` file: higher than any format the older versions read, so they refuse it.
pub const MOVED_STUB_FORMAT: u32 = 2;
/// Key of the stub naming the project file that replaced it.
const MOVED_TO_KEY: &str = "moved_to";
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
    /// Fields this version does not know (written by a later one), kept as they are: a save
    /// writes them back ([`save_project`]).
    #[serde(flatten)]
    pub other: serde_json::Map<String, serde_json::Value>,
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
    /// Fields of the pass this version does not know, kept as they are (see
    /// [`ProjectFile::other`]).
    #[serde(flatten)]
    pub other: serde_json::Map<String, serde_json::Value>,
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

/// An imported pass's surface file, relative to the project folder.
pub fn surface_path(id: u32) -> String {
    format!("{}/surface.ply", pass_dir(id))
}

/// A cleaned pass's surface file, relative to the project folder.
pub fn cleaned_path(id: u32) -> String {
    format!("{}/cleaned.ply", pass_dir(id))
}

/// The observation counts of a cleaned pass's points (one little-endian u32 per point, in order).
pub fn cleaned_counts_path(id: u32) -> String {
    format!("{}/cleaned_observations.bin", pass_dir(id))
}

/// A pass's saved object labels, relative to the project folder.
pub fn object_labels_path(id: u32) -> String {
    format!("{}/object.bin", pass_dir(id))
}

/// The folder of a pass's colour keyframes (`kf-<id>.jpg` and the manifest), relative to the
/// project folder.
pub fn rgb_dir(id: u32) -> String {
    format!("{}/rgb", pass_dir(id))
}

/// The manifest of a pass's colour keyframes (a JSON array, one entry per JPEG).
pub fn rgb_manifest_path(id: u32) -> String {
    format!("{}/keyframes.json", rgb_dir(id))
}

/// The raw recording of a pass when it is kept inside the project folder.
pub fn raw_path(id: u32) -> String {
    format!("{}/raw.bin", pass_dir(id))
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
    fsutil::atomic_write(path, json.as_bytes())
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

/// Writes a keyframe file (through a temporary file renamed over `path`).
pub fn write_keyframes(path: &Path, keyframes: &[KeyframeRecord]) -> Result<(), IoError> {
    fsutil::atomic_write_with(path, |w| write_keyframes_to(w, keyframes))
}

fn write_keyframes_to(w: &mut impl Write, keyframes: &[KeyframeRecord]) -> Result<(), IoError> {
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

/// The keyframe file of a pass without keyframes (what a bundle stores when its keyframes were
/// left out, [`crate::bundle`]).
pub fn empty_keyframes() -> Vec<u8> {
    let mut out = Vec::new();
    write_keyframes_to(&mut out, &[]).expect("writing to a Vec cannot fail");
    out
}

/// Where a project's description lives ([`locate`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProjectLocation {
    /// The project folder: every path stored in the project is relative to it.
    pub root: PathBuf,
    /// The project file: `<name>.scancraft`, or `project.json` of an older project.
    pub file: PathBuf,
    /// Whether `file` is an older `project.json`.
    pub legacy: bool,
}

/// Whether `path` has the extension `ext` (no dot; ASCII case ignored, as Windows does).
pub fn has_extension(path: &Path, ext: &str) -> bool {
    path.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case(ext))
}

/// Whether `path` names a bundle (`.scancraftz`). Bundles are opened through a working folder
/// ([`crate::workdir`]), not with [`load_project`].
pub fn is_bundle(path: &Path) -> bool {
    has_extension(path, BUNDLE_EXTENSION) && path.is_file()
}

/// The `.scancraft` files directly in `dir`, sorted by name.
pub fn project_files_in(dir: &Path) -> Result<Vec<PathBuf>, IoError> {
    let mut out = Vec::new();
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if has_extension(&path, PROJECT_EXTENSION) && entry.file_type()?.is_file() {
            out.push(path);
        }
    }
    out.sort();
    Ok(out)
}

/// What a legacy `project.json` is: a real project, or the stub naming the `.scancraft` file that
/// replaced it (see the module documentation).
fn moved_to(json: &serde_json::Value) -> Option<String> {
    json.get(MOVED_TO_KEY).and_then(|v| v.as_str()).map(str::to_string)
}

/// Finds the project of `path`: a `.scancraft` file, a folder holding exactly one, or an older
/// folder with a `project.json`. A folder with several `.scancraft` files is refused (one project
/// per folder: they would share the pass data). When a folder has both a `.scancraft` file and a
/// real (not moved) `project.json`, the `.scancraft` file is the current one: it is written first
/// on every save, so a `project.json` beside it is at best the state before that save.
pub fn locate(path: &Path) -> Result<ProjectLocation, IoError> {
    if path.is_file() {
        if has_extension(path, PROJECT_EXTENSION) {
            let root = path.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new(".")).to_path_buf();
            return Ok(ProjectLocation { root, file: path.to_path_buf(), legacy: false });
        }
        if has_extension(path, BUNDLE_EXTENSION) {
            return Err(IoError::Unsupported(format!("{} is a bundle: it is opened into a working folder first", path.display())));
        }
        if path.file_name().is_some_and(|n| n.eq_ignore_ascii_case(LEGACY_PROJECT_FILE)) {
            if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
                return locate(parent);
            }
        }
        return Err(IoError::NotAProject(format!("{} is not a ScanCraft project file (.{PROJECT_EXTENSION})", path.display())));
    }
    if !path.is_dir() {
        return Err(IoError::NotAProject(format!("{} does not exist", path.display())));
    }
    let files = project_files_in(path)?;
    let legacy = path.join(LEGACY_PROJECT_FILE);
    match files.as_slice() {
        [file] => {
            if legacy.is_file() && fs::read(&legacy).ok().and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok()).is_some_and(|v| moved_to(&v).is_none()) {
                log::warn!("{} also holds an older project.json; {} is used", path.display(), file.display());
            }
            Ok(ProjectLocation { root: path.to_path_buf(), file: file.clone(), legacy: false })
        }
        [] if legacy.is_file() => {
            let json: serde_json::Value = serde_json::from_slice(&fs::read(&legacy)?).map_err(|e| IoError::Parse(format!("{}: {e}", legacy.display())))?;
            match moved_to(&json) {
                Some(name) => Err(IoError::NotAProject(format!("{}: the project file {name} is missing (project.json only says the project moved there)", path.display()))),
                None => Ok(ProjectLocation { root: path.to_path_buf(), file: legacy, legacy: true }),
            }
        }
        [] => Err(IoError::NotAProject(format!("{} holds no ScanCraft project (no .{PROJECT_EXTENSION} file)", path.display()))),
        several => {
            let names: Vec<String> = several.iter().filter_map(|p| p.file_name()).map(|n| n.to_string_lossy().into_owned()).collect();
            Err(IoError::Unsupported(format!(
                "{} holds several project files ({}); a folder holds one project, since they would share its pass data. Move all but one into folders of their own",
                path.display(),
                names.join(", ")
            )))
        }
    }
}

/// The project folder of `path` (see [`locate`]).
pub fn project_root(path: &Path) -> Result<PathBuf, IoError> {
    locate(path).map(|l| l.root)
}

/// A file name (without extension) for a project name: characters Windows does not allow in file
/// names replaced by `_`, trailing dots and spaces removed, reserved device names avoided, at most
/// 100 characters. `None` when nothing usable is left.
pub fn file_stem_for(name: &str) -> Option<String> {
    let cleaned: String = name.chars().map(|c| if c.is_control() || "<>:\"/\\|?*".contains(c) { '_' } else { c }).take(100).collect();
    let cleaned = cleaned.trim().trim_end_matches(['.', ' ']).to_string();
    if cleaned.is_empty() || cleaned.chars().all(|c| c == '_' || c == '.') {
        return None;
    }
    let base = cleaned.split('.').next().unwrap_or_default().to_ascii_uppercase();
    let reserved = ["CON", "PRN", "AUX", "NUL"].contains(&base.as_str()) || ((base.starts_with("COM") || base.starts_with("LPT")) && base.len() == 4 && base.as_bytes()[3].is_ascii_digit());
    Some(if reserved { format!("{cleaned}_") } else { cleaned })
}

/// The project file a save of `dir` writes: the folder's `.scancraft` file when it has one,
/// otherwise `<project name>.scancraft` (or the folder's name, or `Project`). An existing file
/// keeps its name when the project is renamed. Several `.scancraft` files are refused as in
/// [`locate`].
pub fn project_file_for_save(dir: &Path, name: &str) -> Result<PathBuf, IoError> {
    if dir.is_dir() {
        let files = project_files_in(dir)?;
        match files.len() {
            0 => {}
            1 => return Ok(files[0].clone()),
            _ => return locate(dir).map(|l| l.file),
        }
    }
    let stem = file_stem_for(name).or_else(|| dir.file_name().and_then(|n| file_stem_for(&n.to_string_lossy()))).unwrap_or_else(|| "Project".into());
    Ok(dir.join(format!("{stem}.{PROJECT_EXTENSION}")))
}

/// Writes the project file and every pass's keyframes into `dir` (created if needed); returns the
/// project file written ([`project_file_for_save`]). Each file is written to a temporary file and
/// renamed over the old one, and the project file comes last, so an interrupted or failed save
/// leaves the previous project description in place. An older `project.json` in the folder is
/// then replaced by the *moved* stub (module documentation).
///
/// Fields this version does not know are kept: those of `project` itself, and those of the
/// project file being replaced in `dir` that `project` does not set (matched by pass id for the
/// passes' fields), so a caller that builds the project from scratch does not drop what a later
/// version wrote ([`save_project_carrying`] names another project to take them from when `dir`
/// has none yet).
pub fn save_project(dir: &Path, project: &ProjectFile, keyframes: &[(u32, Vec<KeyframeRecord>)]) -> Result<PathBuf, IoError> {
    save_project_carrying(dir, project, keyframes, None)
}

/// [`save_project`]; when `dir` holds no project file yet (Save As into a new place), the unknown
/// fields come from `previous` (a project folder or project file) instead.
pub fn save_project_carrying(dir: &Path, project: &ProjectFile, keyframes: &[(u32, Vec<KeyframeRecord>)], previous: Option<&Path>) -> Result<PathBuf, IoError> {
    fs::create_dir_all(dir)?;
    let target = project_file_for_save(dir, &project.name)?;
    let mut project = std::borrow::Cow::Borrowed(project);
    let replaced = locate(dir).ok().map(|l| l.file).or_else(|| previous.and_then(|p| locate(p).ok()).map(|l| l.file));
    if let Some(old) = replaced.and_then(|f| read_project_file(&f).ok()) {
        carry_unknown_fields(project.to_mut(), &old);
    }
    let project = project.as_ref();
    for (id, kfs) in keyframes {
        let record = project.passes.iter().find(|p| p.id == *id).ok_or_else(|| IoError::Parse(format!("pass {id} is not in the project")))?;
        if !fsutil::is_safe_relative(&record.keyframes) {
            return Err(IoError::Parse(format!("pass {id}: the keyframe file {} is not inside the project folder", record.keyframes)));
        }
        let path = dir.join(&record.keyframes);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        write_keyframes(&path, kfs)?;
    }
    let json = serde_json::to_string_pretty(project).map_err(|e| IoError::Parse(e.to_string()))?;
    fsutil::atomic_write(&target, json.as_bytes())?;
    retire_legacy_file(dir, &target, project);
    Ok(target)
}

/// Copies the fields this version does not know from `old` into `project` where `project` does
/// not have them: the top-level ones, and each pass's by pass id (a pass deleted since is gone
/// with its fields).
pub fn carry_unknown_fields(project: &mut ProjectFile, old: &ProjectFile) {
    for (k, v) in &old.other {
        project.other.entry(k.clone()).or_insert_with(|| v.clone());
    }
    for pass in &mut project.passes {
        if let Some(o) = old.passes.iter().find(|o| o.id == pass.id) {
            for (k, v) in &o.other {
                pass.other.entry(k.clone()).or_insert_with(|| v.clone());
            }
        }
    }
}

/// The stamp ([`fsutil::FileStamp`]) of the project file of the project at `path` (a folder or
/// its project file), `None` when there is none.
pub fn project_file_stamp(path: &Path) -> Option<fsutil::FileStamp> {
    locate(path).ok().and_then(|l| fsutil::stamp(&l.file))
}

/// `Err(IoError::Conflict)` when the project file of the project at `dir` is no longer the one
/// with stamp `seen` (taken when the project was opened or last saved here): someone else saved
/// it meanwhile, and saving now would replace their version. `seen: None` (not known) and a
/// project file that is gone (saving puts it back) are not conflicts.
pub fn check_unchanged(dir: &Path, seen: Option<fsutil::FileStamp>) -> Result<(), IoError> {
    let Some(seen) = seen else { return Ok(()) };
    let Ok(location) = locate(dir) else { return Ok(()) };
    match fsutil::stamp(&location.file) {
        Some(now) if now != seen => Err(IoError::Conflict(format!("{} was changed by someone else since it was opened here; saving would replace that version", location.file.display()))),
        _ => Ok(()),
    }
}

/// Replaces a real `project.json` in `dir` by the stub pointing at `target`. A failure only means
/// an older ScanCraft could still open the state before this save, so it is logged, not returned.
fn retire_legacy_file(dir: &Path, target: &Path, project: &ProjectFile) {
    let legacy = dir.join(LEGACY_PROJECT_FILE);
    let Ok(bytes) = fs::read(&legacy) else { return };
    if serde_json::from_slice::<serde_json::Value>(&bytes).ok().is_some_and(|v| moved_to(&v).is_some()) {
        return;
    }
    let name = target.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    let stub = serde_json::json!({
        "format": MOVED_STUB_FORMAT,
        "name": project.name,
        "reference_pass": null,
        "next_pass_id": 0,
        "passes": [],
        MOVED_TO_KEY: name,
        "note": "This project is now saved in the .scancraft file named in moved_to. Open that file with ScanCraft 0.2 or newer.",
    });
    let text = serde_json::to_string_pretty(&stub).unwrap_or_default();
    if let Err(e) = fsutil::atomic_write(&legacy, text.as_bytes()) {
        log::warn!("could not replace {} by a pointer to {name}: {e}", legacy.display());
    }
}

/// Reads the project of `path` ([`locate`]: a `.scancraft` file, its folder, or an older folder
/// with `project.json`) and every pass's keyframes.
pub fn load_project(path: &Path) -> Result<(ProjectFile, PassKeyframes), IoError> {
    let location = locate(path)?;
    let project = read_project_file(&location.file)?;
    let mut keyframes = Vec::with_capacity(project.passes.len());
    for p in &project.passes {
        let file = resolve(&location.root, &p.keyframes);
        let kfs = read_keyframes(&file).map_err(|e| match e {
            IoError::Io(io) if io.kind() == std::io::ErrorKind::NotFound => IoError::Parse(format!("{}: the keyframes of {} ({}) are missing", location.root.display(), p.name, p.keyframes)),
            other => other,
        })?;
        keyframes.push((p.id, kfs));
    }
    Ok((project, keyframes))
}

/// Parses project JSON (a `.scancraft` file's content).
pub fn parse_project(bytes: &[u8], what: &str) -> Result<ProjectFile, IoError> {
    let project: ProjectFile = serde_json::from_slice(bytes).map_err(|e| IoError::Parse(format!("{what}: {e}")))?;
    if project.format > FORMAT_VERSION {
        return Err(IoError::Unsupported(format!("{what}: the project was saved by a newer version (format {})", project.format)));
    }
    Ok(project)
}

/// Reads and parses a project file.
pub fn read_project_file(file: &Path) -> Result<ProjectFile, IoError> {
    let name = file.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    parse_project(&fs::read(file)?, &name)
}

/// Whether `path` is a project ([`locate`] finds one).
pub fn is_project(path: &Path) -> bool {
    locate(path).is_ok()
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
            other: Default::default(),
        };
        let project = ProjectFile {
            format: FORMAT_VERSION,
            name: "Bracket".into(),
            reference_pass: Some(1),
            next_pass_id: 4,
            passes: vec![pass(1, "reference"), pass(2, "aligned"), pass(3, "unaligned")],
            outputs: vec![OutputRecord { kind: "merged".into(), file: "outputs/merged.ply".into(), points: 42, passes: vec![1, 2] }],
            settings: serde_json::json!({ "min_mm": 150.0 }),
            other: Default::default(),
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

    fn small_project(name: &str) -> ProjectFile {
        ProjectFile {
            format: FORMAT_VERSION,
            name: name.into(),
            reference_pass: Some(1),
            next_pass_id: 2,
            passes: vec![PassRecord {
                id: 1,
                name: "Pass 1".into(),
                scan_config: serde_json::json!({}),
                device: serde_json::json!({}),
                frames: 1,
                transform: Transform::default(),
                alignment: AlignmentRecord { status: "reference".into(), ..Default::default() },
                weight: 1.0,
                included: true,
                visible: true,
                start: "moved".into(),
                keyframes: keyframes_path(1),
                raw_recording: None,
                other: Default::default(),
            }],
            outputs: Vec::new(),
            settings: serde_json::json!({}),
            other: Default::default(),
        }
    }

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-projectfile-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn the_project_file_is_named_after_the_project_and_found_by_folder_or_file() {
        let dir = temp("named");
        let kfs = vec![(1, vec![keyframe(1, 5)])];
        let file = save_project(&dir, &small_project("Bracket: left / v2"), &kfs).unwrap();
        assert_eq!(file, dir.join("Bracket_ left _ v2.scancraft"));
        assert!(!dir.join(LEGACY_PROJECT_FILE).exists(), "no project.json is written");
        let location = locate(&dir).unwrap();
        assert_eq!((location.file.as_path(), location.legacy), (file.as_path(), false));
        assert_eq!(locate(&file).unwrap().root, dir);
        assert_eq!(load_project(&file).unwrap(), load_project(&dir).unwrap());
        assert!(is_project(&dir) && is_project(&file));
        // A renamed project keeps its file.
        assert_eq!(save_project(&dir, &small_project("Other name"), &kfs).unwrap(), file);
        assert_eq!(load_project(&dir).unwrap().0.name, "Other name");
        // Two project files in one folder are refused, by folder and on save.
        fs::copy(&file, dir.join("Second.scancraft")).unwrap();
        let err = load_project(&dir).unwrap_err().to_string();
        assert!(err.contains("several project files") && err.contains("Second.scancraft"), "{err}");
        assert!(save_project(&dir, &small_project("x"), &kfs).is_err());
        assert!(load_project(&file).is_ok(), "naming the file is not ambiguous");
        // Neither a bundle nor an empty folder is a project.
        let empty = temp("empty");
        fs::create_dir_all(&empty).unwrap();
        assert!(matches!(load_project(&empty), Err(IoError::NotAProject(_))));
        fs::write(empty.join("P.scancraftz"), b"zip").unwrap();
        assert!(matches!(locate(&empty.join("P.scancraftz")), Err(IoError::Unsupported(_))));
        assert_eq!(file_stem_for("  ...  "), None);
        assert_eq!(file_stem_for("con").as_deref(), Some("con_"));
        assert_eq!(file_stem_for("Part.v2.").as_deref(), Some("Part.v2"));
        let _ = fs::remove_dir_all(&dir);
        let _ = fs::remove_dir_all(&empty);
    }

    #[test]
    fn an_older_project_json_folder_opens_and_is_moved_on_save() {
        let dir = temp("legacy");
        let kfs = vec![(1, vec![keyframe(1, 5)])];
        fs::create_dir_all(dir.join("pass-1")).unwrap();
        write_keyframes(&dir.join(keyframes_path(1)), &kfs[0].1).unwrap();
        fs::write(dir.join(LEGACY_PROJECT_FILE), serde_json::to_string_pretty(&small_project("Old")).unwrap()).unwrap();
        let location = locate(&dir).unwrap();
        assert!(location.legacy);
        assert_eq!(locate(&dir.join(LEGACY_PROJECT_FILE)).unwrap(), location);
        let (project, loaded) = load_project(&dir).unwrap();
        assert_eq!((project.name.as_str(), &loaded), ("Old", &kfs));

        let file = save_project(&dir, &project, &loaded).unwrap();
        assert_eq!(file, dir.join("Old.scancraft"));
        assert!(!locate(&dir).unwrap().legacy);
        // project.json is now a stub that versions before the .scancraft file refuse (they read
        // it as a ProjectFile and stop at the format), and that names the new file.
        let stub = fs::read_to_string(dir.join(LEGACY_PROJECT_FILE)).unwrap();
        let as_old_app: ProjectFile = serde_json::from_str(&stub).unwrap();
        assert!(as_old_app.format > FORMAT_VERSION && as_old_app.passes.is_empty());
        assert!(stub.contains("Old.scancraft"));
        assert_eq!(load_project(&dir).unwrap().0, project);
        // A later save leaves the stub as it is.
        save_project(&dir, &project, &loaded).unwrap();
        assert_eq!(fs::read_to_string(dir.join(LEGACY_PROJECT_FILE)).unwrap(), stub);
        // The stub without its file says what is missing.
        fs::remove_file(&file).unwrap();
        let err = load_project(&dir).unwrap_err().to_string();
        assert!(err.contains("Old.scancraft") && err.contains("missing"), "{err}");
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_failed_save_leaves_the_previous_project_intact() {
        let dir = temp("atomic");
        let kfs = vec![(1, vec![keyframe(1, 5)])];
        let file = save_project(&dir, &small_project("Safe"), &kfs).unwrap();
        let before = fs::read(&file).unwrap();
        // The temporary file cannot be created: a folder is in its way.
        fs::create_dir_all(fsutil::tmp_path(&file)).unwrap();
        assert!(save_project(&dir, &small_project("Safe, renamed"), &kfs).is_err());
        assert_eq!(fs::read(&file).unwrap(), before);
        assert_eq!(load_project(&dir).unwrap().0.name, "Safe");
        // Same for a keyframe file: the old one stays and the project still opens.
        fs::remove_dir_all(fsutil::tmp_path(&file)).unwrap();
        let kf = dir.join(keyframes_path(1));
        fs::create_dir_all(fsutil::tmp_path(&kf)).unwrap();
        assert!(save_project(&dir, &small_project("Safe"), &[(1, vec![keyframe(2, 9)])]).is_err());
        assert_eq!(load_project(&dir).unwrap().1, kfs);
        // A missing keyframe file is named.
        fs::remove_dir_all(fsutil::tmp_path(&kf)).unwrap();
        fs::remove_file(&kf).unwrap();
        let err = load_project(&dir).unwrap_err().to_string();
        assert!(err.contains("keyframes of Pass 1") && err.contains("missing"), "{err}");
        // A keyframe path leaving the folder is refused on save.
        let mut escaping = small_project("Safe");
        escaping.passes[0].keyframes = "../outside.bin".into();
        assert!(save_project(&dir, &escaping, &kfs).is_err());
        assert_eq!(empty_keyframes().len(), 16);
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

    #[test]
    fn fields_of_a_later_version_are_kept_on_save() {
        let dir = temp("forward");
        let kfs = vec![(1, vec![keyframe(0, 3)])];
        let file = save_project(&dir, &small_project("Bracket"), &kfs).unwrap();
        // A later version adds fields at the top level and in a pass (and a pass it knows).
        let mut json: serde_json::Value = serde_json::from_slice(&fs::read(&file).unwrap()).unwrap();
        json["thumbnail"] = serde_json::json!({ "file": "thumb.png", "size": [256, 256] });
        json["passes"][0]["lighting"] = serde_json::json!("dome");
        fs::write(&file, serde_json::to_vec_pretty(&json).unwrap()).unwrap();

        // Read here: the unknown fields are held, and written back as they were.
        let (loaded, loaded_kfs) = load_project(&dir).unwrap();
        assert_eq!(loaded.other.get("thumbnail"), Some(&json["thumbnail"]));
        assert_eq!(loaded.passes[0].other.get("lighting"), Some(&serde_json::json!("dome")));
        save_project(&dir, &loaded, &loaded_kfs).unwrap();
        let again: serde_json::Value = serde_json::from_slice(&fs::read(&file).unwrap()).unwrap();
        assert_eq!(again["thumbnail"], json["thumbnail"]);
        assert_eq!(again["passes"][0]["lighting"], "dome");

        // A caller that builds the project from scratch (the app does) keeps them too: they come
        // from the file being replaced. A field the caller sets wins; a deleted pass's are gone.
        let mut fresh = small_project("Bracket");
        fresh.other.insert("thumbnail".into(), serde_json::json!("mine"));
        fresh.passes.push(PassRecord { id: 2, keyframes: keyframes_path(2), ..fresh.passes[0].clone() });
        save_project(&dir, &fresh, &[(1, vec![keyframe(0, 3)]), (2, Vec::new())]).unwrap();
        let (loaded, _) = load_project(&dir).unwrap();
        assert_eq!(loaded.other.get("thumbnail"), Some(&serde_json::json!("mine")));
        assert_eq!(loaded.passes[0].other.get("lighting"), Some(&serde_json::json!("dome")));
        assert!(loaded.passes[1].other.is_empty(), "pass 2 is new: nothing to carry");

        // Save As into a new folder takes them from the previous project.
        let moved = temp("forward-moved");
        save_project_carrying(&moved, &small_project("Bracket"), &kfs, Some(&dir)).unwrap();
        let (copied, _) = load_project(&moved).unwrap();
        assert_eq!(copied.other.get("thumbnail"), Some(&serde_json::json!("mine")));
        assert_eq!(copied.passes[0].other.get("lighting"), Some(&serde_json::json!("dome")));
        // A newer *format* is still refused, unknown fields or not.
        json["format"] = serde_json::json!(FORMAT_VERSION + 1);
        fs::write(&file, serde_json::to_vec(&json).unwrap()).unwrap();
        assert!(matches!(load_project(&dir), Err(IoError::Unsupported(_))));
        let _ = fs::remove_dir_all(&dir);
        let _ = fs::remove_dir_all(&moved);
    }

    #[test]
    fn a_project_file_changed_by_someone_else_is_a_conflict() {
        let dir = temp("conflict");
        let kfs = vec![(1, vec![keyframe(0, 3)])];
        save_project(&dir, &small_project("Bracket"), &kfs).unwrap();
        let seen = project_file_stamp(&dir);
        assert!(seen.is_some());
        assert!(check_unchanged(&dir, seen).is_ok());
        assert!(check_unchanged(&dir, None).is_ok(), "not known: no conflict");
        // Someone else saves (another size, a later time).
        std::thread::sleep(std::time::Duration::from_millis(20));
        save_project(&dir, &small_project("Bracket, edited elsewhere"), &kfs).unwrap();
        match check_unchanged(&dir, seen) {
            Err(IoError::Conflict(why)) => assert!(why.contains("changed by someone else"), "{why}"),
            other => panic!("{other:?}"),
        }
        // Gone: saving puts it back, no conflict.
        let file = locate(&dir).unwrap().file;
        fs::remove_file(&file).unwrap();
        assert!(check_unchanged(&dir, seen).is_ok());
        let _ = fs::remove_dir_all(&dir);
    }
}
