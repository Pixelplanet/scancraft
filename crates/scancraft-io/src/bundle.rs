//! Project bundles: a whole project in one `<name>.scancraftz` file (`docs/open-issues.md` 4.1).
//!
//! A bundle is a plain ZIP (ZIP64 where sizes need it) of the project folder's layout
//! ([`crate::project`]), so any ZIP program opens it:
//!
//! ```text
//! <name>.scancraft          the project file: always the first entry
//! pass-<id>/…, outputs/…    the project's files by their relative path, sorted
//! scancraft-bundle.json     the manifest (last): contents chosen, size and SHA-256 of every entry,
//!                           what was left out
//! ```
//!
//! Entries are *stored*, not compressed (point data barely compresses, and stored entries can be
//! read in place), and never encrypted. A bundle that another ZIP program re-packed (deflated
//! entries, no manifest) still opens; ZIP's own CRC-32 is checked on every read, the manifest's
//! SHA-256 too when there is one.
//!
//! **Contents.** What goes in is chosen per [`Item`] ([`BundleContents`], the settings page's
//! list); [`classify`] maps the project layout's files to the items. Files nothing would miss are
//! never bundled: lock files, temporary files, a working folder's state file, an older
//! `project.json`, Explorer's `Thumbs.db` / `desktop.ini`. A left-out file the app cannot open a
//! project without is replaced by a placeholder: the keyframe file of a pass that has a stored
//! surface (`surface.ply` or `cleaned.ply`) by an empty keyframe file, a colour keyframe manifest
//! by an empty list. The keyframes of a pass whose only surface they are (a captured, uncleaned
//! pass) are always kept ([`BundleReport::kept`]).
//!
//! **Saving.** A bundle is written to `<bundle>.tmp` beside the target, flushed and renamed over
//! it, so a failed or interrupted save leaves the previous bundle intact. Saving a working folder
//! back ([`save_back`]) copies the entries of files that did not change raw from the previous
//! bundle (no re-reading through a decoder, no recompression), writes changed and new files from
//! the working folder and drops the entries of deleted files. "Unchanged" means: the file has the
//! size and modification time recorded when it was extracted (or last saved), or, when only the
//! time differs (the app rewrites files on every save), the same SHA-256; and the old bundle's
//! entry still has the recorded size and CRC-32 (so a bundle replaced behind the app's back is
//! never mixed in).
//!
//! **Extraction** ([`Bundle::extract_to`]) copies every entry into the working folder at disk
//! speed. A later improvement is lazy extraction: stored entries can be read straight from the
//! bundle (their data offset is known), so only files that change would need writing.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use zip::result::ZipError;
use zip::write::FileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

use crate::lock::{self, LockFile};
use crate::project::{self, ProjectFile, BUNDLE_EXTENSION, LEGACY_PROJECT_FILE, PROJECT_EXTENSION};
use crate::{fsutil, IoError};

/// The manifest entry's name.
pub const MANIFEST_ENTRY: &str = "scancraft-bundle.json";
/// Version of the manifest (and of the bundle layout).
pub const BUNDLE_FORMAT: u32 = 1;
/// JSON entries (project file, manifest) larger than this are refused as damaged.
const MAX_JSON_ENTRY: u64 = 64 << 20;
/// Entries at least this large get ZIP64 size fields (they may pass 4 GiB while being written).
const LARGE_ENTRY: u64 = 1 << 31;
const CHUNK: usize = 1 << 20;

/// A group of project files that is bundled or left out together (`docs/open-issues.md` 4.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Item {
    /// The project file, pass surfaces and alignment, cleaned surfaces, object labels, and any
    /// file this version does not know. Always bundled.
    Core,
    /// `pass-<id>/keyframes.bin`.
    Keyframes,
    /// `pass-<id>/rgb/`.
    ColourKeyframes,
    /// `outputs/` (merged model, mesh).
    Results,
    /// `pass-<id>/markers.json`.
    MarkerMaps,
    /// `pass-<id>/raw.bin`, `pass-<id>/raw/`, and a pass's `raw_recording` when it lies inside
    /// the project folder.
    RawRecordings,
}

impl Item {
    pub const ALL: [Item; 6] = [Item::Core, Item::Keyframes, Item::ColourKeyframes, Item::Results, Item::MarkerMaps, Item::RawRecordings];

    /// The name shown in the settings page.
    pub fn label(self) -> &'static str {
        match self {
            Item::Core => "Project file, pass surfaces and alignment, cleaned surfaces, object labels",
            Item::Keyframes => "Keyframes (depth frames and poses)",
            Item::ColourKeyframes => "Colour keyframes",
            Item::Results => "Merged model and mesh",
            Item::MarkerMaps => "Marker maps",
            Item::RawRecordings => "Raw stream recordings",
        }
    }

    /// What the item is needed for (the explanation beside its switch).
    pub fn needed_for(self) -> &'static str {
        match self {
            Item::Core => "Opening the project at all, merging, meshing",
            Item::Keyframes => "Continuing a pass, discarding frames, re-tracking, Same Position start, adding passes from it",
            Item::ColourKeyframes => "Colour, AI object segmentation",
            Item::Results => "Opening with results ready; otherwise Merge / Reconstruct again (minutes)",
            Item::MarkerMaps => "Make Solid marker removal, marker display",
            Item::RawRecordings => "Replaying a scan for analysis or bug reports only; several MB per second",
        }
    }
}

/// Which optional items a bundle holds ([`Item::Core`] always). The default is the settings
/// page's default: everything but raw recordings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct BundleContents {
    pub keyframes: bool,
    pub colour_keyframes: bool,
    pub results: bool,
    pub marker_maps: bool,
    pub raw_recordings: bool,
}

impl Default for BundleContents {
    fn default() -> Self {
        BundleContents { keyframes: true, colour_keyframes: true, results: true, marker_maps: true, raw_recordings: false }
    }
}

impl BundleContents {
    pub fn everything() -> Self {
        BundleContents { raw_recordings: true, ..Default::default() }
    }

    /// No optional item: [`Item::Core`] only.
    pub fn nothing() -> Self {
        BundleContents { keyframes: false, colour_keyframes: false, results: false, marker_maps: false, raw_recordings: false }
    }

    /// Every item that either holds.
    pub fn union(&self, other: &BundleContents) -> BundleContents {
        BundleContents {
            keyframes: self.keyframes || other.keyframes,
            colour_keyframes: self.colour_keyframes || other.colour_keyframes,
            results: self.results || other.results,
            marker_maps: self.marker_maps || other.marker_maps,
            raw_recordings: self.raw_recordings || other.raw_recordings,
        }
    }

    pub fn includes(&self, item: Item) -> bool {
        match item {
            Item::Core => true,
            Item::Keyframes => self.keyframes,
            Item::ColourKeyframes => self.colour_keyframes,
            Item::Results => self.results,
            Item::MarkerMaps => self.marker_maps,
            Item::RawRecordings => self.raw_recordings,
        }
    }

    /// Switches an optional item ([`Item::Core`] cannot be switched off).
    pub fn set(&mut self, item: Item, on: bool) {
        match item {
            Item::Core => {}
            Item::Keyframes => self.keyframes = on,
            Item::ColourKeyframes => self.colour_keyframes = on,
            Item::Results => self.results = on,
            Item::MarkerMaps => self.marker_maps = on,
            Item::RawRecordings => self.raw_recordings = on,
        }
    }
}

fn pass_id(dir: &str) -> Option<u32> {
    dir.strip_prefix("pass-")?.parse().ok()
}

/// The item a project file belongs to (`rel`: `/`-separated, relative to the project folder), or
/// `None` for a file that is never bundled. `raw_recordings` are the passes' raw recording paths
/// that lie inside the project folder ([`raw_recording_paths`]).
pub fn classify(rel: &str, raw_recordings: &[String]) -> Option<Item> {
    let name = rel.rsplit('/').next().unwrap_or(rel).to_ascii_lowercase();
    let top_level = !rel.contains('/');
    let never = name.ends_with(".tmp")
        || name.ends_with(".lock")
        || name.contains(".lock.stale-")
        || name == crate::workdir::STATE_FILE
        || name.ends_with(&format!(".{BUNDLE_EXTENSION}"))
        || matches!(name.as_str(), "thumbs.db" | "desktop.ini" | ".ds_store")
        || (top_level && (name == LEGACY_PROJECT_FILE || name == MANIFEST_ENTRY));
    if never {
        return None;
    }
    if raw_recordings.iter().any(|r| rel == r || rel.strip_prefix(r.as_str()).is_some_and(|rest| rest.starts_with('/'))) {
        return Some(Item::RawRecordings);
    }
    if rel.starts_with("outputs/") {
        return Some(Item::Results);
    }
    if let Some((first, rest)) = rel.split_once('/') {
        if pass_id(first).is_some() {
            return Some(match rest {
                "keyframes.bin" => Item::Keyframes,
                "markers.json" => Item::MarkerMaps,
                "raw.bin" => Item::RawRecordings,
                r if r.starts_with("rgb/") => Item::ColourKeyframes,
                r if r.starts_with("raw/") => Item::RawRecordings,
                _ => Item::Core,
            });
        }
    }
    Some(Item::Core)
}

/// The passes' raw recording paths that lie inside the project folder, `/`-separated (absolute
/// ones, such as a `SCANCRAFT_RECORD_DIR` outside the project, are not part of a bundle).
pub fn raw_recording_paths(project: &ProjectFile) -> Vec<String> {
    project
        .passes
        .iter()
        .filter_map(|p| p.raw_recording.as_deref())
        .map(|r| r.replace('\\', "/"))
        .map(|r| r.trim_start_matches("./").trim_end_matches('/').to_string())
        .filter(|r| fsutil::is_safe_relative(r))
        .collect()
}

/// A file of the working folder as it matches its bundle entry: recorded at extraction and after
/// every save, to tell unchanged files on the next save.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EntryState {
    /// Path relative to the folder, `/`-separated (also the entry name).
    pub path: String,
    pub size: u64,
    /// The file's modification time (ns since 1970) when it last matched the entry.
    pub mtime_ns: u64,
    pub crc32: u32,
    /// Lower-case hex.
    pub sha256: String,
}

/// The manifest entry ([`MANIFEST_ENTRY`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    pub format: u32,
    #[serde(default)]
    pub app: String,
    /// When the bundle was written (s since 1970).
    #[serde(default)]
    pub created: u64,
    /// The project file's entry.
    pub project: String,
    #[serde(default)]
    pub contents: BundleContents,
    #[serde(default)]
    pub entries: Vec<ManifestEntry>,
    /// What the contents setting left out, per item.
    #[serde(default)]
    pub left_out: Vec<LeftOut>,
    /// Entries that hold a placeholder instead of the left-out file (see the module docs).
    #[serde(default)]
    pub placeholders: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManifestEntry {
    pub path: String,
    pub size: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sha256: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeftOut {
    pub item: Item,
    pub files: u64,
    pub bytes: u64,
}

/// What a write or save put into the bundle.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BundleReport {
    pub path: PathBuf,
    /// Entries, the manifest included.
    pub entries: usize,
    /// Size of the bundle file.
    pub bytes: u64,
    /// Entries written from files (or placeholders).
    pub written: usize,
    /// Entries copied raw from the previous bundle.
    pub copied_raw: usize,
    pub left_out: Vec<LeftOut>,
    pub placeholders: Vec<String>,
    /// Files of a left-out item kept because the project cannot open without them.
    pub kept: Vec<String>,
}

/// What [`Bundle::verify`] found.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VerifyReport {
    pub entries: usize,
    pub bytes: u64,
    /// Damaged entries and disagreements with the manifest.
    pub problems: Vec<String>,
    /// Entries the manifest does not list (a bundle changed by another ZIP program).
    pub unlisted: Vec<String>,
}

impl VerifyReport {
    pub fn is_ok(&self) -> bool {
        self.problems.is_empty()
    }
}

/// One file entry of a bundle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BundleEntry {
    /// Index in the ZIP's central directory.
    pub index: usize,
    pub name: String,
    pub size: u64,
    pub compressed_size: u64,
    /// Stored (else deflated by another ZIP program).
    pub stored: bool,
    pub crc32: u32,
    /// From the manifest, when it lists the entry.
    pub sha256: Option<String>,
}

/// An opened bundle: its entries listed and checked for safe names; nothing is read yet.
pub struct Bundle {
    path: PathBuf,
    archive: ZipArchive<BufReader<File>>,
    entries: Vec<BundleEntry>,
    /// Index into `entries`.
    project: usize,
    manifest: Option<Manifest>,
}

fn zip_error(path: &Path, e: ZipError) -> IoError {
    match e {
        ZipError::Io(e) => IoError::Io(e),
        ZipError::InvalidArchive(m) => IoError::Parse(format!("{}: not a readable bundle ({m})", path.display())),
        ZipError::UnsupportedArchive(m) => IoError::Unsupported(format!("{}: {m}", path.display())),
        ZipError::FileNotFound => IoError::Parse(format!("{}: an entry is missing", path.display())),
    }
}

/// Whether an entry name may become a file below the working folder on every system.
fn is_safe_entry_name(name: &str) -> bool {
    fsutil::is_safe_relative(name) && !name.chars().any(|c| c.is_control() || "<>\"|?*".contains(c)) && name.split('/').all(|part| !part.ends_with(['.', ' ']))
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// What copying an entry or a file produced.
struct Copied {
    size: u64,
    crc32: u32,
    sha256: String,
}

/// Copies `reader` to `out`, hashing it. Read errors are `Err(Ok(e))`, write errors `Err(Err(e))`.
fn copy_hashing(reader: &mut dyn Read, out: &mut dyn Write, limit: u64) -> Result<Copied, Result<io::Error, io::Error>> {
    let mut buf = vec![0u8; CHUNK];
    let (mut sha, mut crc, mut size) = (Sha256::new(), crc32fast::Hasher::new(), 0u64);
    loop {
        let n = match reader.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(Ok(e)),
        };
        size += n as u64;
        if size > limit {
            return Err(Ok(io::Error::new(io::ErrorKind::InvalidData, "longer than its entry says")));
        }
        sha.update(&buf[..n]);
        crc.update(&buf[..n]);
        out.write_all(&buf[..n]).map_err(Err)?;
    }
    Ok(Copied { size, crc32: crc.finalize(), sha256: hex(&sha.finalize()) })
}

/// The SHA-256 of a file.
fn hash_file(path: &Path) -> Result<String, IoError> {
    let mut f = BufReader::new(File::open(path)?);
    copy_hashing(&mut f, &mut io::sink(), u64::MAX).map(|c| c.sha256).map_err(|e| IoError::Io(e.unwrap_or_else(|e| e)))
}

impl Bundle {
    /// Opens a bundle and lists it. Refused: not a ZIP, no or several project files, entry names
    /// that would land outside the folder (or are not valid file names), entries that appear
    /// twice (also differing only in case), compression other than stored/deflate, encryption,
    /// a damaged manifest or one of a newer format.
    pub fn open(path: &Path) -> Result<Bundle, IoError> {
        let file = File::open(path)?;
        let mut archive = ZipArchive::new(BufReader::new(file)).map_err(|e| zip_error(path, e))?;
        let mut entries = Vec::new();
        let mut seen = HashSet::new();
        let mut manifest_index = None;
        for index in 0..archive.len() {
            let f = archive.by_index_raw(index).map_err(|e| zip_error(path, e))?;
            let name = f.name().to_string();
            let bad_name = || IoError::Parse(format!("{}: the entry {name:?} is not a safe path inside a project folder", path.display()));
            if let Some(dir) = name.strip_suffix('/') {
                if !is_safe_entry_name(dir) {
                    return Err(bad_name());
                }
                continue;
            }
            if !is_safe_entry_name(&name) {
                return Err(bad_name());
            }
            if !seen.insert(name.to_lowercase()) {
                return Err(IoError::Parse(format!("{}: the entry {name} appears twice", path.display())));
            }
            let stored = match f.compression() {
                CompressionMethod::Stored => true,
                CompressionMethod::Deflated => false,
                other => return Err(IoError::Unsupported(format!("{}: {name} is compressed with {other:?}, which ScanCraft does not read", path.display()))),
            };
            if stored && f.compressed_size() != f.size() {
                return Err(IoError::Unsupported(format!("{}: {name} is encrypted or damaged", path.display())));
            }
            if name == MANIFEST_ENTRY {
                manifest_index = Some((index, f.size(), stored, f.crc32()));
                continue;
            }
            entries.push(BundleEntry { index, name, size: f.size(), compressed_size: f.compressed_size(), stored, crc32: f.crc32(), sha256: None });
        }
        let projects: Vec<usize> = (0..entries.len()).filter(|&k| !entries[k].name.contains('/') && project::has_extension(Path::new(&entries[k].name), PROJECT_EXTENSION)).collect();
        let project = match projects.as_slice() {
            [k] => *k,
            [] => return Err(IoError::NotAProject(format!("{} holds no project file (.{PROJECT_EXTENSION}): it is not a ScanCraft bundle", path.display()))),
            _ => return Err(IoError::Unsupported(format!("{} holds several project files", path.display()))),
        };
        let mut bundle = Bundle { path: path.to_path_buf(), archive, entries, project, manifest: None };
        if let Some((index, size, stored, crc32)) = manifest_index {
            let entry = BundleEntry { index, name: MANIFEST_ENTRY.into(), size, compressed_size: 0, stored, crc32, sha256: None };
            let bytes = bundle.read_entry(&entry, MAX_JSON_ENTRY)?;
            let manifest: Manifest = serde_json::from_slice(&bytes).map_err(|e| IoError::Parse(format!("{}: the manifest is damaged ({e})", path.display())))?;
            if manifest.format > BUNDLE_FORMAT {
                return Err(IoError::Unsupported(format!("{}: the bundle was written by a newer version (format {})", path.display(), manifest.format)));
            }
            let hashes: HashMap<&str, &str> = manifest.entries.iter().filter_map(|e| Some((e.path.as_str(), e.sha256.as_deref()?))).collect();
            for e in &mut bundle.entries {
                e.sha256 = hashes.get(e.name.as_str()).map(|h| h.to_string());
            }
            bundle.manifest = Some(manifest);
        }
        Ok(bundle)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The file entries (the manifest not included), in archive order.
    pub fn entries(&self) -> &[BundleEntry] {
        &self.entries
    }

    pub fn manifest(&self) -> Option<&Manifest> {
        self.manifest.as_ref()
    }

    pub fn project_entry(&self) -> &BundleEntry {
        &self.entries[self.project]
    }

    /// Bytes the entries take when extracted.
    pub fn total_size(&self) -> u64 {
        self.entries.iter().map(|e| e.size).sum()
    }

    /// Reads and parses the project file (refused when a newer version wrote it).
    pub fn read_project(&mut self) -> Result<ProjectFile, IoError> {
        let entry = self.project_entry().clone();
        let bytes = self.read_entry(&entry, MAX_JSON_ENTRY)?;
        project::parse_project(&bytes, &entry.name)
    }

    /// Reads a (small) entry by name.
    pub fn read(&mut self, name: &str) -> Result<Vec<u8>, IoError> {
        let entry = self.entries.iter().find(|e| e.name == name).cloned().ok_or_else(|| IoError::Parse(format!("{}: no entry {name}", self.path.display())))?;
        self.read_entry(&entry, MAX_JSON_ENTRY)
    }

    fn read_entry(&mut self, entry: &BundleEntry, limit: u64) -> Result<Vec<u8>, IoError> {
        if entry.size > limit {
            return Err(IoError::Parse(format!("{}: {} is implausibly large ({} bytes)", self.path.display(), entry.name, entry.size)));
        }
        let mut out = Vec::with_capacity(entry.size.min(1 << 20) as usize);
        self.copy_entry(entry, &mut out)?;
        Ok(out)
    }

    /// Copies an entry's content to `out`, checking its size, CRC-32 and (when the manifest has
    /// it) SHA-256. A damaged entry is `IoError::Parse` naming it; a failed write is `IoError::Io`.
    fn copy_entry(&mut self, entry: &BundleEntry, out: &mut dyn Write) -> Result<Copied, IoError> {
        let path = self.path.clone();
        let damaged = |why: String| IoError::Parse(format!("{}: {} is damaged ({why})", path.display(), entry.name));
        let mut raw = self.archive.by_index_raw(entry.index).map_err(|e| zip_error(&path, e))?;
        let mut reader: Box<dyn Read + '_> = if entry.stored { Box::new(&mut raw) } else { Box::new(flate2::read::DeflateDecoder::new(&mut raw)) };
        let copied = match copy_hashing(&mut reader, out, entry.size) {
            Ok(c) => c,
            Err(Ok(read)) => return Err(damaged(read.to_string())),
            Err(Err(write)) => return Err(IoError::Io(write)),
        };
        if copied.size != entry.size {
            return Err(damaged(format!("{} of {} bytes", copied.size, entry.size)));
        }
        if copied.crc32 != entry.crc32 {
            return Err(damaged("checksum mismatch".into()));
        }
        if entry.sha256.as_deref().is_some_and(|h| !h.eq_ignore_ascii_case(&copied.sha256)) {
            return Err(damaged("SHA-256 differs from the manifest".into()));
        }
        Ok(copied)
    }

    /// Reads every entry and checks it (and the manifest's list).
    pub fn verify(&mut self) -> Result<VerifyReport, IoError> {
        let mut report = VerifyReport::default();
        for entry in self.entries.clone() {
            report.entries += 1;
            match self.copy_entry(&entry, &mut io::sink()) {
                Ok(c) => report.bytes += c.size,
                Err(IoError::Parse(why)) => report.problems.push(why),
                Err(e) => return Err(e),
            }
        }
        if let Some(manifest) = &self.manifest {
            let by_name: HashMap<&str, &BundleEntry> = self.entries.iter().map(|e| (e.name.as_str(), e)).collect();
            for m in &manifest.entries {
                match by_name.get(m.path.as_str()) {
                    None => report.problems.push(format!("{} is listed in the manifest but missing", m.path)),
                    Some(e) if e.size != m.size => report.problems.push(format!("{}: {} bytes, the manifest says {}", m.path, e.size, m.size)),
                    Some(_) => {}
                }
            }
            let listed: HashSet<&str> = manifest.entries.iter().map(|m| m.path.as_str()).collect();
            report.unlisted = self.entries.iter().filter(|e| !listed.contains(e.name.as_str())).map(|e| e.name.clone()).collect();
        }
        Ok(report)
    }

    /// Extracts every entry into `dir` (created if needed; existing files are overwritten) after
    /// checking the drive has room; returns each file's state for a later [`save_back`]. A
    /// damaged entry stops the extraction with an error naming it.
    pub fn extract_to(&mut self, dir: &Path) -> Result<Vec<EntryState>, IoError> {
        fs::create_dir_all(dir)?;
        fsutil::check_free_space(dir, self.total_size())?;
        let mut states = Vec::with_capacity(self.entries.len());
        for entry in self.entries.clone() {
            // Never a lock, a temporary file or a working folder's state from a foreign bundle.
            if classify(&entry.name, &[]).is_none() {
                continue;
            }
            let target = dir.join(&entry.name);
            if let Some(parent) = target.parent() {
                fs::create_dir_all(parent)?;
            }
            let mut out = BufWriter::with_capacity(CHUNK, File::create(&target)?);
            let copied = self.copy_entry(&entry, &mut out)?;
            out.into_inner().map_err(|e| e.into_error())?;
            let mtime_ns = fsutil::mtime_ns(&fs::metadata(&target)?);
            states.push(EntryState { path: entry.name.clone(), size: copied.size, mtime_ns, crc32: copied.crc32, sha256: copied.sha256 });
        }
        Ok(states)
    }
}

/// Writes a bundle of the project at `project` (a project folder, its `.scancraft` file, or an
/// older folder with `project.json`) to `dest`, atomically. Refused when another running
/// ScanCraft has `dest` open.
pub fn write_bundle(project: &Path, dest: &Path, contents: &BundleContents) -> Result<BundleReport, IoError> {
    let location = project::locate(project)?;
    write(&location.root, &location.file, location.legacy, None, &[], dest, contents).map(|(report, _)| report)
}

/// Saves a working folder (`work_dir`, a form-A project extracted from `source`) into the bundle
/// `dest` (`source` itself for *Save*, another path for *Save as Bundle…*). Unchanged files are
/// copied raw from `source` (see the module docs); `recorded` is the folder's state from the
/// extraction or the previous save. Returns the report and the folder's new state.
pub fn save_back(work_dir: &Path, source: &Path, recorded: &[EntryState], dest: &Path, contents: &BundleContents) -> Result<(BundleReport, Vec<EntryState>), IoError> {
    let location = project::locate(work_dir)?;
    write(&location.root, &location.file, location.legacy, Some(source), recorded, dest, contents)
}

enum Source {
    File { path: PathBuf, size: u64, mtime_ns: u64 },
    Bytes(Vec<u8>),
    /// An entry of the source bundle, with the state it matches.
    Raw { entry: BundleEntry, state: EntryState },
}

struct Planned {
    name: String,
    source: Source,
}

impl Planned {
    fn size(&self) -> u64 {
        match &self.source {
            Source::File { size, .. } => *size,
            Source::Bytes(b) => b.len() as u64,
            Source::Raw { entry, .. } => entry.compressed_size,
        }
    }
}

/// What to do with a file of an item that is left out.
enum Leave {
    Keep,
    Placeholder(Vec<u8>),
    Drop,
}

fn leave_out(item: Item, rel: &str, present: &HashSet<&str>) -> Leave {
    let id = rel.split_once('/').and_then(|(first, _)| pass_id(first));
    match (item, id) {
        (Item::Keyframes, Some(id)) => {
            if present.contains(project::surface_path(id).as_str()) || present.contains(project::cleaned_path(id).as_str()) {
                Leave::Placeholder(project::empty_keyframes())
            } else {
                Leave::Keep
            }
        }
        (Item::ColourKeyframes, Some(id)) if rel == project::rgb_manifest_path(id) => Leave::Placeholder(b"[]".to_vec()),
        _ => Leave::Drop,
    }
}

fn zip_options(size: u64, mtime_ns: u64) -> FileOptions {
    let mut options = FileOptions::default().compression_method(CompressionMethod::Stored).large_file(size >= LARGE_ENTRY).unix_permissions(0o644);
    let (y, mo, d, h, mi, s) = fsutil::civil_utc(mtime_ns / 1_000_000_000);
    if let Ok(t) = u16::try_from(y).map_err(drop).and_then(|y| zip::DateTime::from_date_and_time(y, mo as u8, d as u8, h as u8, mi as u8, s as u8)) {
        options = options.last_modified_time(t);
    }
    options
}

#[allow(clippy::too_many_arguments)]
fn write(
    root: &Path,
    project_file: &Path,
    legacy: bool,
    source: Option<&Path>,
    recorded: &[EntryState],
    dest: &Path,
    contents: &BundleContents,
) -> Result<(BundleReport, Vec<EntryState>), IoError> {
    lock::check_not_held_by_other(&LockFile::path_for(dest))?;
    let file_name = project_file.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    let parsed = project::read_project_file(project_file)?;
    let raw_paths = raw_recording_paths(&parsed);
    let project_entry = if legacy {
        let stem = project::file_stem_for(&parsed.name).or_else(|| root.file_name().and_then(|n| project::file_stem_for(&n.to_string_lossy()))).unwrap_or_else(|| "Project".into());
        format!("{stem}.{PROJECT_EXTENSION}")
    } else {
        file_name.clone()
    };
    if !is_safe_entry_name(&project_entry) {
        return Err(IoError::Parse(format!("{project_entry} cannot be stored in a bundle")));
    }

    let mut old = match source.filter(|p| p.is_file()) {
        Some(p) => match Bundle::open(p) {
            Ok(b) => Some(b),
            Err(e) => {
                log::warn!("{} cannot be copied from ({e}); every file is written from the working folder", p.display());
                None
            }
        },
        None => None,
    };
    let old_entries: HashMap<String, BundleEntry> = old.iter().flat_map(|b| b.entries.iter().map(|e| (e.name.clone(), e.clone()))).collect();
    let recorded: HashMap<&str, &EntryState> = recorded.iter().map(|s| (s.path.as_str(), s)).collect();

    let files = fsutil::walk_files(root)?;
    let present: HashSet<&str> = files.iter().map(|f| f.rel.as_str()).collect();
    // Either the file's entry from the old bundle (unchanged) or the file itself.
    let plan_file = |f: &fsutil::WalkedFile, name: &str| -> Result<Planned, IoError> {
        if let (Some(rec), Some(entry)) = (recorded.get(f.rel.as_str()), old_entries.get(name)) {
            if rec.size == f.size && entry.size == rec.size && entry.crc32 == rec.crc32 {
                let same = (rec.mtime_ns == f.mtime_ns && f.mtime_ns != 0) || hash_file(&f.path)? == rec.sha256;
                if same {
                    let state = EntryState { mtime_ns: f.mtime_ns, ..(*rec).clone() };
                    return Ok(Planned { name: name.to_string(), source: Source::Raw { entry: entry.clone(), state } });
                }
            }
        }
        Ok(Planned { name: name.to_string(), source: Source::File { path: f.path.clone(), size: f.size, mtime_ns: f.mtime_ns } })
    };

    let mut report = BundleReport { path: dest.to_path_buf(), ..Default::default() };
    let mut left_out: BTreeMap<Item, LeftOut> = BTreeMap::new();
    let project_walked = files.iter().find(|f| f.rel == file_name).ok_or_else(|| IoError::Parse(format!("{} is not in {}", file_name, root.display())))?;
    let mut plan = vec![plan_file(project_walked, &project_entry)?];
    for f in &files {
        if f.rel == file_name || (!f.rel.contains('/') && project::has_extension(Path::new(&f.rel), PROJECT_EXTENSION)) {
            continue;
        }
        let Some(item) = classify(&f.rel, &raw_paths) else { continue };
        if !contents.includes(item) {
            let leave = leave_out(item, &f.rel, &present);
            if !matches!(leave, Leave::Keep) {
                let tally = left_out.entry(item).or_insert(LeftOut { item, files: 0, bytes: 0 });
                tally.files += 1;
                tally.bytes += f.size;
            }
            match leave {
                Leave::Keep => report.kept.push(f.rel.clone()),
                Leave::Placeholder(bytes) => {
                    report.placeholders.push(f.rel.clone());
                    plan.push(Planned { name: f.rel.clone(), source: Source::Bytes(bytes) });
                    continue;
                }
                Leave::Drop => continue,
            }
        }
        plan.push(plan_file(f, &f.rel)?);
    }
    report.left_out = left_out.into_values().collect();

    let dest_dir = dest.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    fsutil::check_free_space(dest_dir, plan.iter().map(Planned::size).sum())?;

    let tmp = fsutil::tmp_path(dest);
    let mut states = Vec::new();
    let result = (|| -> Result<(), IoError> {
        let mut zw = ZipWriter::new(BufWriter::with_capacity(CHUNK, File::create(&tmp)?));
        let mut manifest_entries = Vec::with_capacity(plan.len());
        for p in &plan {
            let z = |e| zip_error(&tmp, e);
            match &p.source {
                Source::File { path, size, mtime_ns } => {
                    zw.start_file(p.name.clone(), zip_options(*size, *mtime_ns)).map_err(z)?;
                    let mut input = File::open(path)?;
                    let copied = copy_hashing(&mut input, &mut zw, u64::MAX).map_err(|e| IoError::Io(e.unwrap_or_else(|e| e)))?;
                    manifest_entries.push(ManifestEntry { path: p.name.clone(), size: copied.size, sha256: Some(copied.sha256.clone()) });
                    states.push(EntryState { path: p.name.clone(), size: copied.size, mtime_ns: *mtime_ns, crc32: copied.crc32, sha256: copied.sha256 });
                    report.written += 1;
                }
                Source::Bytes(bytes) => {
                    zw.start_file(p.name.clone(), zip_options(bytes.len() as u64, fsutil::system_ns(std::time::SystemTime::now()))).map_err(z)?;
                    zw.write_all(bytes)?;
                    manifest_entries.push(ManifestEntry { path: p.name.clone(), size: bytes.len() as u64, sha256: Some(hex(&Sha256::digest(bytes))) });
                    report.written += 1;
                }
                Source::Raw { entry, state } => {
                    let bundle = old.as_mut().expect("raw entries come from the source bundle");
                    let raw = bundle.archive.by_index_raw(entry.index).map_err(|e| zip_error(&bundle.path, e))?;
                    zw.raw_copy_file(raw).map_err(z)?;
                    manifest_entries.push(ManifestEntry { path: p.name.clone(), size: entry.size, sha256: Some(state.sha256.clone()) });
                    states.push(EntryState { path: p.name.clone(), ..state.clone() });
                    report.copied_raw += 1;
                }
            }
        }
        let manifest = Manifest {
            format: BUNDLE_FORMAT,
            app: format!("ScanCraft {}", env!("CARGO_PKG_VERSION")),
            created: fsutil::unix_now(),
            project: project_entry.clone(),
            contents: *contents,
            entries: manifest_entries,
            left_out: report.left_out.clone(),
            placeholders: report.placeholders.clone(),
        };
        let json = serde_json::to_vec_pretty(&manifest).map_err(|e| IoError::Parse(e.to_string()))?;
        zw.start_file(MANIFEST_ENTRY, zip_options(json.len() as u64, fsutil::system_ns(std::time::SystemTime::now()))).map_err(|e| zip_error(&tmp, e))?;
        zw.write_all(&json)?;
        let file = zw.finish().map_err(|e| zip_error(&tmp, e))?.into_inner().map_err(|e| e.into_error())?;
        file.sync_all()?;
        Ok(())
    })();
    // The source must be closed before it is replaced.
    drop(old);
    let result = result.and_then(|()| fsutil::rename_replace(&tmp, dest).map_err(IoError::from));
    if let Err(e) = result {
        let _ = fs::remove_file(&tmp);
        return Err(e);
    }
    report.entries = plan.len() + 1;
    report.bytes = fs::metadata(dest)?.len();
    // The project entry of a legacy folder has another name than its file: no state for it.
    if legacy {
        states.retain(|s| s.path != project_entry);
    }
    Ok((report, states))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::project::{keyframes_path, AlignmentRecord, KeyframeRecord, PassRecord};
    use scancraft_core::Transform;

    pub(crate) fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-bundle-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn keyframe(id: u64, n: usize) -> KeyframeRecord {
        KeyframeRecord {
            id,
            rotation: [0.0, 0.0, 0.0, 1.0],
            translation: [id as f32, 0.0, 300.0],
            rmse_mm: 0.2,
            visibility_conflict: 0.01,
            new_points: n as u64,
            discarded: false,
            imu: None,
            centroid: [0.0, 0.0, 300.0],
            points: (0..n).map(|i| ([i as f32, 1.0, 2.0], [0, 0, 127])).collect(),
        }
    }

    fn pass(id: u32, raw: Option<&str>) -> PassRecord {
        PassRecord {
            id,
            name: format!("Pass {id}"),
            scan_config: serde_json::json!({}),
            device: serde_json::json!({}),
            frames: 3,
            transform: Transform::default(),
            alignment: AlignmentRecord { status: "aligned".into(), ..Default::default() },
            weight: 1.0,
            included: true,
            visible: true,
            start: "moved".into(),
            keyframes: keyframes_path(id),
            raw_recording: raw.map(str::to_string),
        }
    }

    /// A project folder with every kind of file: pass 1 imported (surface, colour, markers),
    /// pass 2 captured only (keyframes are its surface) with a raw recording, results, and files
    /// that must never be bundled.
    pub(crate) fn sample_project(dir: &Path) -> PathBuf {
        let project = ProjectFile {
            format: project::FORMAT_VERSION,
            name: "Bracket".into(),
            reference_pass: Some(1),
            next_pass_id: 3,
            passes: vec![pass(1, None), pass(2, Some("pass-2/raw.bin"))],
            outputs: Vec::new(),
            settings: serde_json::json!({}),
        };
        let file = project::save_project(dir, &project, &[(1, vec![keyframe(0, 50)]), (2, vec![keyframe(1, 80), keyframe(2, 10)])]).unwrap();
        let put = |rel: &str, bytes: &[u8]| {
            let p = dir.join(rel);
            fs::create_dir_all(p.parent().unwrap()).unwrap();
            fs::write(p, bytes).unwrap();
        };
        put(&project::surface_path(1), &[7u8; 3000]);
        put(&project::object_labels_path(1), b"labels");
        put(&project::rgb_manifest_path(1), br#"[{"jpeg_file": "kf-000001.jpg"}]"#);
        put("pass-1/rgb/kf-000001.jpg", &[0xFF; 500]);
        put(&project::markers_path(1), br#"{"format": 1, "markers": []}"#);
        put("pass-2/raw.bin", &[1u8; 4000]);
        put("outputs/merged.ply", &[2u8; 1500]);
        put("outputs/mesh.ply", &[3u8; 900]);
        put(".scancraft.lock", b"{}");
        put("pass-1/surface.ply.tmp", b"half");
        put("Thumbs.db", b"x");
        file
    }

    fn names(bundle: &Bundle) -> Vec<String> {
        bundle.entries().iter().map(|e| e.name.clone()).collect()
    }

    #[test]
    fn items_of_the_layout() {
        let raw = vec!["recordings/take1".to_string()];
        let cases = [
            ("Part.scancraft", Some(Item::Core)),
            ("pass-3/keyframes.bin", Some(Item::Keyframes)),
            ("pass-3/surface.ply", Some(Item::Core)),
            ("pass-3/cleaned.ply", Some(Item::Core)),
            ("pass-3/cleaned_observations.bin", Some(Item::Core)),
            ("pass-3/object.bin", Some(Item::Core)),
            ("pass-3/rgb/kf-000004.jpg", Some(Item::ColourKeyframes)),
            ("pass-3/rgb/keyframes.json", Some(Item::ColourKeyframes)),
            ("pass-3/markers.json", Some(Item::MarkerMaps)),
            ("pass-3/raw.bin", Some(Item::RawRecordings)),
            ("pass-3/raw/frames-0001.bin", Some(Item::RawRecordings)),
            ("recordings/take1/a.bin", Some(Item::RawRecordings)),
            ("recordings/take10/a.bin", Some(Item::Core)),
            ("outputs/merged.ply", Some(Item::Results)),
            ("outputs/mesh.ply", Some(Item::Results)),
            ("notes.txt", Some(Item::Core)),
            ("passes/keyframes.bin", Some(Item::Core)),
            ("project.json", None),
            (".scancraft.lock", None),
            ("Part.scancraftz.lock", None),
            ("Old.scancraftz", None),
            ("pass-1/keyframes.bin.tmp", None),
            (".scancraft.lock.stale-12-34", None),
            (crate::workdir::STATE_FILE, None),
            (MANIFEST_ENTRY, None),
            ("Thumbs.db", None),
        ];
        for (rel, item) in cases {
            assert_eq!(classify(rel, &raw), item, "{rel}");
        }
        let defaults = BundleContents::default();
        assert!(Item::ALL.iter().all(|&i| defaults.includes(i) == (i != Item::RawRecordings)));
        assert!(Item::ALL.iter().all(|i| !i.label().is_empty() && !i.needed_for().is_empty()));
        let mut c = BundleContents::everything();
        c.set(Item::Core, false);
        c.set(Item::Results, false);
        assert!(c.includes(Item::Core) && !c.includes(Item::Results) && c.includes(Item::RawRecordings));
    }

    #[test]
    fn writes_reads_and_extracts() {
        let dir = temp("write");
        let project = dir.join("Bracket");
        let file = sample_project(&project);
        assert_eq!(file.file_name().unwrap(), "Bracket.scancraft");
        let dest = dir.join("Bracket.scancraftz");
        let report = write_bundle(&project, &dest, &BundleContents::default()).unwrap();
        assert_eq!((report.copied_raw, report.left_out.len()), (0, 1));
        assert_eq!(report.left_out[0], LeftOut { item: Item::RawRecordings, files: 1, bytes: 4000 });
        assert!(!fsutil::tmp_path(&dest).exists());

        let mut bundle = Bundle::open(&dest).unwrap();
        let listed = names(&bundle);
        assert_eq!(listed[0], "Bracket.scancraft", "the project file comes first");
        for absent in ["pass-2/raw.bin", ".scancraft.lock", "pass-1/surface.ply.tmp", "Thumbs.db"] {
            assert!(!listed.iter().any(|n| n == absent), "{absent} must not be bundled");
        }
        let mut sorted = listed[1..].to_vec();
        sorted.sort();
        assert_eq!(sorted, listed[1..].to_vec());
        assert!(bundle.entries().iter().all(|e| e.stored && e.sha256.is_some()));
        let manifest = bundle.manifest().unwrap().clone();
        assert_eq!((manifest.project.as_str(), manifest.entries.len()), ("Bracket.scancraft", listed.len()));
        assert_eq!(bundle.read_project().unwrap().name, "Bracket");
        let verified = bundle.verify().unwrap();
        assert!(verified.is_ok() && verified.unlisted.is_empty(), "{verified:?}");
        assert_eq!(verified.bytes, bundle.total_size());

        // Any ZIP reader sees the same files, stored.
        let mut zip = ZipArchive::new(File::open(&dest).unwrap()).unwrap();
        assert_eq!(zip.by_index(0).unwrap().name(), "Bracket.scancraft");
        assert_eq!(zip.by_index(zip.len() - 1).unwrap().name(), MANIFEST_ENTRY);
        drop(zip);

        let out = dir.join("out");
        let states = bundle.extract_to(&out).unwrap();
        assert_eq!(states.len(), listed.len());
        for s in &states {
            assert_eq!(fs::read(out.join(&s.path)).unwrap(), fs::read(project.join(&s.path)).unwrap(), "{}", s.path);
        }
        let (loaded, kfs) = project::load_project(&out).unwrap();
        assert_eq!((loaded.passes.len(), kfs[1].1.len()), (2, 2));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_content_filter_leaves_out_what_the_project_can_open_without() {
        let dir = temp("filter");
        let project = dir.join("Bracket");
        sample_project(&project);
        let dest = dir.join("Small.scancraftz");
        let contents = BundleContents { keyframes: false, colour_keyframes: false, results: false, marker_maps: false, raw_recordings: true };
        let report = write_bundle(&project.join("Bracket.scancraft"), &dest, &contents).unwrap();
        // Pass 1 has a stored surface: its keyframes become an empty file. Pass 2's keyframes are
        // its only surface: kept.
        assert_eq!(report.kept, vec!["pass-2/keyframes.bin".to_string()]);
        assert_eq!(report.placeholders, vec!["pass-1/keyframes.bin".to_string(), "pass-1/rgb/keyframes.json".to_string()]);
        let mut bundle = Bundle::open(&dest).unwrap();
        let listed = names(&bundle);
        for absent in ["pass-1/rgb/kf-000001.jpg", "pass-1/markers.json", "outputs/merged.ply", "outputs/mesh.ply"] {
            assert!(!listed.iter().any(|n| n == absent), "{absent}");
        }
        assert!(listed.iter().any(|n| n == "pass-2/raw.bin"), "raw recordings were asked for");
        assert_eq!(bundle.read("pass-1/rgb/keyframes.json").unwrap(), b"[]");
        let items: Vec<Item> = bundle.manifest().unwrap().left_out.iter().map(|l| l.item).collect();
        assert_eq!(items, vec![Item::Keyframes, Item::ColourKeyframes, Item::Results, Item::MarkerMaps]);
        assert!(bundle.verify().unwrap().is_ok());
        // The extracted project opens: pass 1 without keyframes, pass 2 with them.
        let out = dir.join("out");
        bundle.extract_to(&out).unwrap();
        let (_, kfs) = project::load_project(&out).unwrap();
        assert_eq!((kfs[0].1.len(), kfs[1].1.len()), (0, 2));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn saving_back_copies_unchanged_entries_raw() {
        let dir = temp("saveback");
        let project = dir.join("Bracket");
        sample_project(&project);
        let bundle_path = dir.join("Bracket.scancraftz");
        write_bundle(&project, &bundle_path, &BundleContents::default()).unwrap();
        let work = dir.join("work");
        let recorded = Bundle::open(&bundle_path).unwrap().extract_to(&work).unwrap();
        let before = names(&Bundle::open(&bundle_path).unwrap());

        // Nothing changed: everything is copied raw, and the bundle's entries stay the same.
        let (report, states) = save_back(&work, &bundle_path, &recorded, &bundle_path, &BundleContents::default()).unwrap();
        assert_eq!((report.written, report.copied_raw), (0, before.len()), "{report:?}");
        assert_eq!(states, recorded);

        // Rewritten with the same bytes (new time stamp), changed, deleted, added.
        std::thread::sleep(std::time::Duration::from_millis(20));
        fs::write(work.join("outputs/mesh.ply"), [3u8; 900]).unwrap();
        fs::write(work.join(project::surface_path(1)), [9u8; 3000]).unwrap();
        fs::remove_file(work.join("outputs/merged.ply")).unwrap();
        fs::write(work.join("pass-1/notes.txt"), b"new").unwrap();
        let (report, states) = save_back(&work, &bundle_path, &states, &bundle_path, &BundleContents::default()).unwrap();
        assert_eq!((report.written, report.copied_raw), (2, before.len() - 2), "{report:?}");
        let mut bundle = Bundle::open(&bundle_path).unwrap();
        let after = names(&bundle);
        assert!(!after.iter().any(|n| n == "outputs/merged.ply") && after.iter().any(|n| n == "pass-1/notes.txt"));
        assert!(bundle.verify().unwrap().is_ok());
        assert_eq!(bundle.read("pass-1/notes.txt").unwrap(), b"new");
        let check = dir.join("check");
        bundle.extract_to(&check).unwrap();
        assert_eq!(fs::read(check.join(project::surface_path(1))).unwrap(), vec![9u8; 3000]);
        // The new state names the rewritten file's new time, so the next save does not hash it.
        let mesh = states.iter().find(|s| s.path == "outputs/mesh.ply").unwrap();
        assert_eq!(mesh.mtime_ns, fsutil::mtime_ns(&fs::metadata(work.join("outputs/mesh.ply")).unwrap()));

        // A bundle replaced behind our back is not copied from where its entries differ.
        let other = dir.join("Other");
        sample_project(&other);
        fs::write(other.join("outputs/mesh.ply"), [4u8; 900]).unwrap();
        write_bundle(&other, &bundle_path, &BundleContents::default()).unwrap();
        let (_, _) = save_back(&work, &bundle_path, &states, &bundle_path, &BundleContents::default()).unwrap();
        let mut bundle = Bundle::open(&bundle_path).unwrap();
        let fresh = dir.join("fresh");
        bundle.extract_to(&fresh).unwrap();
        assert_eq!(fs::read(fresh.join("outputs/mesh.ply")).unwrap(), vec![3u8; 900], "the working folder's file, not the replaced bundle's");
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_failed_write_leaves_the_old_bundle_intact() {
        let dir = temp("atomic");
        let project = dir.join("Bracket");
        sample_project(&project);
        let dest = dir.join("Bracket.scancraftz");
        write_bundle(&project, &dest, &BundleContents::default()).unwrap();
        let old = fs::read(&dest).unwrap();
        // The temporary file cannot be created: a folder is in its way.
        fs::create_dir_all(fsutil::tmp_path(&dest)).unwrap();
        fs::write(project.join("pass-1/extra.bin"), b"more").unwrap();
        assert!(write_bundle(&project, &dest, &BundleContents::default()).is_err());
        assert_eq!(fs::read(&dest).unwrap(), old);
        fs::remove_dir_all(fsutil::tmp_path(&dest)).unwrap();
        // A destination opened by another running ScanCraft is refused.
        let mut other = lock::Owner::this_process();
        other.host = "SOME-OTHER-PC".into();
        fs::write(LockFile::path_for(&dest), serde_json::to_vec(&other).unwrap()).unwrap();
        assert!(matches!(write_bundle(&project, &dest, &BundleContents::default()), Err(IoError::Locked { .. })));
        assert_eq!(fs::read(&dest).unwrap(), old);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_older_project_folder_is_bundled_under_the_new_name() {
        let dir = temp("legacy");
        let project = dir.join("Old");
        let file = sample_project(&project);
        fs::rename(&file, project.join(LEGACY_PROJECT_FILE)).unwrap();
        let dest = dir.join("Old.scancraftz");
        write_bundle(&project, &dest, &BundleContents::default()).unwrap();
        let bundle = Bundle::open(&dest).unwrap();
        assert_eq!(bundle.project_entry().name, "Bracket.scancraft");
        assert!(!names(&bundle).iter().any(|n| n == LEGACY_PROJECT_FILE));
        let _ = fs::remove_dir_all(&dir);
    }

    /// A ZIP with the given (name, bytes) entries, stored.
    fn zip_of(path: &Path, entries: &[(&str, &[u8])]) {
        let mut zw = ZipWriter::new(File::create(path).unwrap());
        for (name, bytes) in entries {
            zw.start_file(*name, FileOptions::default().compression_method(CompressionMethod::Stored)).unwrap();
            zw.write_all(bytes).unwrap();
        }
        zw.finish().unwrap();
    }

    #[test]
    fn malformed_bundles_are_errors_not_panics() {
        let dir = temp("malformed");
        let bad = dir.join("bad.scancraftz");
        let project_json = br#"{"format": 1, "name": "P", "reference_pass": null, "next_pass_id": 1, "passes": []}"#;
        let refuse = |what: &str| {
            let e = Bundle::open(&bad).err();
            assert!(e.is_some(), "{what} must be refused");
            e.unwrap().to_string()
        };
        fs::write(&bad, b"").unwrap();
        refuse("an empty file");
        fs::write(&bad, b"PK\x03\x04 this is not really a zip").unwrap();
        refuse("junk");
        zip_of(&bad, &[("notes.txt", b"hi")]);
        assert!(refuse("no project file").contains("no project file"));
        zip_of(&bad, &[("A.scancraft", project_json), ("B.scancraft", project_json)]);
        refuse("two project files");
        zip_of(&bad, &[("P.scancraft", project_json), ("../evil.txt", b"x")]);
        assert!(refuse("a path leaving the folder").contains("not a safe path"));
        zip_of(&bad, &[("P.scancraft", project_json), ("pass-1/a.bin", b"x"), ("PASS-1/A.bin", b"y")]);
        assert!(refuse("names differing only in case").contains("twice"));
        zip_of(&bad, &[("P.scancraft", project_json), (MANIFEST_ENTRY, b"{not json")]);
        assert!(refuse("a damaged manifest").contains("manifest"));
        zip_of(&bad, &[("P.scancraft", br#"{"format": 99, "name": "P", "next_pass_id": 1, "passes": []}"#)]);
        assert!(matches!(Bundle::open(&bad).unwrap().read_project(), Err(IoError::Unsupported(_))));
        // A bundle without a manifest (packed by hand) opens.
        zip_of(&bad, &[("P.scancraft", project_json), ("pass-1/keyframes.bin", &project::empty_keyframes())]);
        let mut b = Bundle::open(&bad).unwrap();
        assert!(b.manifest().is_none() && b.verify().unwrap().is_ok());
        // A foreign bundle's lock or working-folder state is never unpacked.
        zip_of(&bad, &[("P.scancraft", project_json), (crate::workdir::STATE_FILE, b"{}"), (".scancraft.lock", b"{}")]);
        let states = Bundle::open(&bad).unwrap().extract_to(&dir.join("foreign")).unwrap();
        assert_eq!(states.len(), 1);
        assert!(!dir.join("foreign").join(crate::workdir::STATE_FILE).exists());

        // Damage inside a good bundle: verify names the entry, extraction stops with an error.
        let project = dir.join("Bracket");
        sample_project(&project);
        let good = dir.join("good.scancraftz");
        write_bundle(&project, &good, &BundleContents::default()).unwrap();
        let bytes = fs::read(&good).unwrap();
        let surface = Bundle::open(&good).unwrap().entries().iter().find(|e| e.name == project::surface_path(1)).unwrap().clone();
        let at = (0..bytes.len() - 3000).find(|&i| bytes[i..i + 3000].iter().all(|&b| b == 7)).unwrap();
        let mut flipped = bytes.clone();
        flipped[at + 1234] ^= 0x55;
        fs::write(&bad, &flipped).unwrap();
        let mut b = Bundle::open(&bad).unwrap();
        let report = b.verify().unwrap();
        assert_eq!(report.problems.len(), 1, "{report:?}");
        assert!(report.problems[0].contains(&surface.name));
        assert!(b.extract_to(&dir.join("damaged")).is_err());

        // Truncated bundles and single-byte damage anywhere: never a panic.
        for cut in [10, bytes.len() / 3, bytes.len() / 2, bytes.len() - 30, bytes.len() - 1] {
            fs::write(&bad, &bytes[..cut]).unwrap();
            assert!(Bundle::open(&bad).is_err(), "cut at {cut}");
        }
        let mut rng = 0x9E37_79B9_7F4A_7C15u64;
        for round in 0..400 {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            let mut damaged = bytes.clone();
            // Half the rounds hit the central directory and the end records, where the parser looks.
            let span = if round % 2 == 0 { damaged.len() } else { 600.min(damaged.len()) };
            let at = damaged.len() - 1 - (rng as usize % span);
            damaged[at] ^= (rng >> 32) as u8 | 1;
            fs::write(&bad, &damaged).unwrap();
            let outcome = std::panic::catch_unwind(|| {
                if let Ok(mut b) = Bundle::open(&bad) {
                    let _ = b.read_project();
                    let _ = b.verify();
                    let _ = b.extract_to(&dir.join("fuzz"));
                }
            });
            assert!(outcome.is_ok(), "panic with byte {at} damaged");
        }
        let _ = fs::remove_dir_all(&dir);
    }
}
