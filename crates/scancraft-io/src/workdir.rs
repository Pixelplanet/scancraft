//! Working folders: an opened bundle is unpacked into a folder (a temporary form-A project) and
//! all work goes there; *Save* writes it back into the bundle (`docs/open-issues.md` 4.1).
//!
//! ```text
//! <root>/<bundle name>-<id>/                    the working folder: a project folder
//! <root>/<bundle name>-<id>/.scancraft-work.json   its state (STATE_FILE)
//! ```
//!
//! `root` is a setting; [`default_root`] is under the user's local application data. The state
//! file names the bundle, the bundle's size and time stamp when it was opened (or last saved),
//! the owner (process id, host, process start) and when it was opened, and each extracted file's
//! size, time stamp and hashes ([`EntryState`]), which [`crate::bundle::save_back`] uses to copy
//! unchanged entries raw. It is written before anything is unpacked, so every working folder has
//! one.
//!
//! While a bundle is open its lock is held ([`crate::lock`]), so a second app or PC cannot open
//! it for writing. Closing ([`WorkFolder::close`]) removes the folder. A folder whose owner has
//! ended without closing it (a crash) is a *leftover*: [`leftovers`] lists them on the next start
//! with what changed in them, and the app offers to recover ([`WorkFolder::recover`]) or discard
//! ([`discard`]) each. A folder in use by a running app is listed as such and left alone. A
//! removal that stopped part-way (a file below the folder was held open) is never offered for
//! recovery: the next start finishes it.

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::bundle::{self, Bundle, BundleContents, BundleReport, EntryState, Item};
use crate::lock::{self, Liveness, LockFile, Owner};
use crate::{fsutil, project, IoError};

/// The state file inside a working folder.
pub const STATE_FILE: &str = ".scancraft-work.json";
const STATE_FORMAT: u32 = 1;
/// Suffix of a working folder being removed.
const CLOSING_SUFFIX: &str = ".closing";

/// `<local app data>/ScanCraft/work`.
pub fn default_root() -> PathBuf {
    fsutil::local_data_dir().join("ScanCraft").join("work")
}

/// The state file's content.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkState {
    pub format: u32,
    /// The bundle (absolute path).
    pub bundle: PathBuf,
    /// The bundle's size and time stamp (ns since 1970) when it was opened or last saved: a
    /// different bundle there now was changed by someone else.
    pub bundle_size: u64,
    pub bundle_mtime_ns: u64,
    pub owner: Owner,
    /// When the bundle was opened, and last saved (s since 1970).
    pub opened: u64,
    #[serde(default)]
    pub saved: Option<u64>,
    /// Whether the folder holds the whole project: unpacking finished ([`WorkFolder::open`]),
    /// or the app wrote the project into a new folder ([`WorkFolder::project_written`]). An
    /// incomplete folder holds no work and only part of the bundle: it can only be discarded.
    #[serde(default)]
    pub complete: bool,
    #[serde(default)]
    pub entries: Vec<EntryState>,
    /// What the bundle holds (its manifest's contents when it was opened, or what the last save
    /// wrote); `None`: not known (no manifest, or no save yet).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contents: Option<BundleContents>,
    /// The folder is being removed ([`WorkFolder::close`], [`discard`]): a removal that stopped
    /// part-way is finished at the next start, never offered for recovery.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub closing: bool,
}

impl WorkState {
    /// Whether the bundle file is no longer the one opened or last saved (replaced, changed or
    /// deleted by someone else). A bundle that did not exist when the folder was made (a new
    /// bundle not saved yet) and still does not is unchanged.
    pub fn bundle_changed(&self) -> bool {
        match stamp(&self.bundle) {
            Some(now) => now != (self.bundle_size, self.bundle_mtime_ns),
            None => (self.bundle_size, self.bundle_mtime_ns) != (0, 0),
        }
    }

    /// What the bundle holds: as recorded, or (for a state without it) the items of the
    /// extracted entries.
    pub fn held_contents(&self) -> BundleContents {
        if let Some(c) = self.contents {
            return c;
        }
        let mut held = BundleContents::nothing();
        for item in self.entries.iter().filter_map(|e| bundle::classify(&e.path, &[])) {
            held.set(item, true);
        }
        held
    }
}

fn read_state(dir: &Path) -> Result<WorkState, IoError> {
    let path = dir.join(STATE_FILE);
    let state: WorkState = serde_json::from_slice(&fs::read(&path)?).map_err(|e| IoError::Parse(format!("{}: {e}", path.display())))?;
    if state.format > STATE_FORMAT {
        return Err(IoError::Unsupported(format!("{}: written by a newer version", path.display())));
    }
    Ok(state)
}

fn write_state(dir: &Path, state: &WorkState) -> Result<(), IoError> {
    let json = serde_json::to_vec_pretty(state).map_err(|e| IoError::Parse(e.to_string()))?;
    fsutil::atomic_write(&dir.join(STATE_FILE), &json)
}

/// Size and time stamp of a file, `None` when it does not exist.
fn stamp(path: &Path) -> Option<(u64, u64)> {
    fs::metadata(path).ok().map(|m| (m.len(), fsutil::mtime_ns(&m)))
}

/// Files of `dir` that differ from `entries`: changed, new or deleted (sorted). A file with the
/// recorded size but another time stamp is hashed.
pub fn changed_files(dir: &Path, entries: &[EntryState]) -> Result<Vec<String>, IoError> {
    let mut changed = Vec::new();
    let mut seen = std::collections::HashSet::new();
    let recorded: std::collections::HashMap<&str, &EntryState> = entries.iter().map(|e| (e.path.as_str(), e)).collect();
    for f in fsutil::walk_files(dir)? {
        if bundle::classify(&f.rel, &[]).is_none() {
            continue;
        }
        seen.insert(f.rel.clone());
        let same = match recorded.get(f.rel.as_str()) {
            Some(e) if e.size != f.size => false,
            Some(e) if e.mtime_ns == f.mtime_ns && f.mtime_ns != 0 => true,
            Some(e) => sha256_of(&f.path)? == e.sha256,
            None => false,
        };
        if !same {
            changed.push(f.rel);
        }
    }
    changed.extend(entries.iter().filter(|e| !seen.contains(&e.path)).map(|e| e.path.clone()));
    changed.sort();
    Ok(changed)
}

fn sha256_of(path: &Path) -> Result<String, IoError> {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    std::io::copy(&mut std::io::BufReader::new(fs::File::open(path)?), &mut hasher)?;
    Ok(hasher.finalize().iter().map(|b| format!("{b:02x}")).collect())
}

/// A new, empty folder `<root>/<stem>-<id>`.
fn create_unique(root: &Path, stem: &str) -> Result<PathBuf, IoError> {
    fs::create_dir_all(root)?;
    let seed = fsutil::system_ns(std::time::SystemTime::now()) ^ (u64::from(std::process::id()) << 32);
    for k in 0..1000u64 {
        let id = (seed.wrapping_add(k.wrapping_mul(0x9E37_79B9_7F4A_7C15)) >> 8) as u32;
        let dir = root.join(format!("{stem}-{id:08x}"));
        match fs::create_dir(&dir) {
            Ok(()) => return Ok(dir),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(e.into()),
        }
    }
    Err(IoError::Io(std::io::Error::new(std::io::ErrorKind::AlreadyExists, format!("no free working folder name in {}", root.display()))))
}

/// An opened bundle's working folder. Dropping it without [`WorkFolder::close`] keeps the folder
/// (the next start lists it as a leftover) and releases the bundle's lock.
#[derive(Debug)]
pub struct WorkFolder {
    dir: PathBuf,
    state: WorkState,
    lock: LockFile,
}

impl WorkFolder {
    /// Opens `bundle` into a new working folder under `root`: locks the bundle, checks the
    /// project file (a newer or broken one is refused before anything is unpacked), checks the
    /// drive of `root` has room for the unpacked files, and unpacks everything. On failure
    /// nothing is left behind.
    pub fn open(bundle_path: &Path, root: &Path) -> Result<WorkFolder, IoError> {
        let bundle_path = std::path::absolute(bundle_path)?;
        let lock = LockFile::lock_project(&bundle_path)?;
        WorkFolder::open_with_lock(&bundle_path, root, lock)
    }

    /// [`WorkFolder::open`] of a bundle whose lock `holder` has (on another computer), after the
    /// user confirmed taking it over ([`LockFile::take_over_from`]: refused when someone else
    /// holds it by now, or a running ScanCraft on this computer).
    pub fn open_taking_over(bundle_path: &Path, root: &Path, holder: &Owner) -> Result<WorkFolder, IoError> {
        let bundle_path = std::path::absolute(bundle_path)?;
        let lock = LockFile::take_over_from(&LockFile::path_for(&bundle_path), holder)?;
        WorkFolder::open_with_lock(&bundle_path, root, lock)
    }

    /// [`WorkFolder::open`] with the bundle's lock already taken (by the caller, who may have had
    /// to ask the user first). `lock` must be the bundle's ([`LockFile::path_for`]).
    pub fn open_with_lock(bundle_path: &Path, root: &Path, lock: LockFile) -> Result<WorkFolder, IoError> {
        let bundle_path = std::path::absolute(bundle_path)?;
        if std::path::absolute(lock.path())? != LockFile::path_for(&bundle_path) {
            return Err(IoError::Unsupported(format!("{} is not the lock of {}", lock.path().display(), bundle_path.display())));
        }
        let mut bundle = Bundle::open(&bundle_path)?;
        bundle.read_project()?;
        fs::create_dir_all(root)?;
        fsutil::check_free_space(root, bundle.total_size())?;
        let stem = bundle_path.file_stem().and_then(|s| project::file_stem_for(&s.to_string_lossy())).unwrap_or_else(|| "Project".into());
        let dir = create_unique(root, &stem)?;
        let (bundle_size, bundle_mtime_ns) = stamp(&bundle_path).unwrap_or_default();
        let mut state = WorkState {
            format: STATE_FORMAT,
            bundle: bundle_path,
            bundle_size,
            bundle_mtime_ns,
            owner: lock.owner().clone(),
            opened: fsutil::unix_now(),
            saved: None,
            complete: false,
            entries: Vec::new(),
            contents: bundle.manifest().map(|m| m.contents),
            closing: false,
        };
        let unpacked = (|| {
            write_state(&dir, &state)?;
            state.entries = bundle.extract_to(&dir)?;
            state.complete = true;
            write_state(&dir, &state)
        })();
        if let Err(e) = unpacked {
            let _ = fs::remove_dir_all(&dir);
            return Err(e);
        }
        Ok(WorkFolder { dir, state, lock })
    }

    /// A new, empty working folder under `root` for a bundle that does not exist yet (*Save as
    /// Bundle…* of a project that is not a bundle): locks `dest`, and records it as the folder's
    /// bundle. The app writes the project into [`WorkFolder::dir`], calls
    /// [`WorkFolder::project_written`], then [`WorkFolder::save`] writes the bundle. Until the
    /// project is written the folder counts as incomplete (a crash before that leaves a folder
    /// that can only be discarded); after it, a crash (also one while the bundle is written)
    /// leaves a folder that is offered for recovery, since for a new project it is the only copy.
    /// `dest` may exist: the first save replaces it.
    pub fn create(dest: &Path, root: &Path) -> Result<WorkFolder, IoError> {
        let dest = std::path::absolute(dest)?;
        let lock = LockFile::lock_project(&dest)?;
        let stem = dest.file_stem().and_then(|s| project::file_stem_for(&s.to_string_lossy())).unwrap_or_else(|| "Project".into());
        let dir = create_unique(root, &stem)?;
        let (bundle_size, bundle_mtime_ns) = stamp(&dest).unwrap_or_default();
        let state = WorkState {
            format: STATE_FORMAT,
            bundle: dest,
            bundle_size,
            bundle_mtime_ns,
            owner: lock.owner().clone(),
            opened: fsutil::unix_now(),
            saved: None,
            complete: false,
            entries: Vec::new(),
            contents: None,
            closing: false,
        };
        if let Err(e) = write_state(&dir, &state) {
            let _ = fs::remove_dir_all(&dir);
            return Err(e);
        }
        Ok(WorkFolder { dir, state, lock })
    }

    /// Takes over a leftover working folder (its owner ended without closing it): takes the
    /// bundle's lock (refused while another running app holds it) and becomes its owner. Refused
    /// for a folder still in use and for one whose unpacking did not finish.
    pub fn recover(dir: &Path) -> Result<WorkFolder, IoError> {
        let mut state = read_state(dir)?;
        if !state.owner.is_this_process() && state.owner.liveness() != Liveness::Dead {
            return Err(IoError::Locked { path: dir.to_path_buf(), owner: Some(Box::new(state.owner.clone())) });
        }
        if state.closing {
            return Err(IoError::Unsupported(format!("{}: it was being removed when ScanCraft ended; discard it", dir.display())));
        }
        if !state.complete {
            return Err(IoError::Unsupported(format!("{}: unpacking stopped part-way, so it holds no changes; discard it and open the bundle again", dir.display())));
        }
        let lock = LockFile::lock_project(&state.bundle)?;
        state.owner = lock.owner().clone();
        write_state(dir, &state)?;
        Ok(WorkFolder { dir: dir.to_path_buf(), state, lock })
    }

    /// For a folder made by [`WorkFolder::create`]: the app has written the whole project into
    /// it (its project file last), so from now on it holds work and is recovered after a crash.
    /// Refused while the folder has no project file.
    pub fn project_written(&mut self) -> Result<(), IoError> {
        self.project_file()?;
        self.state.complete = true;
        write_state(&self.dir, &self.state)
    }

    /// The working folder (a project folder: open it with [`project::load_project`]).
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The bundle's lock (it says whose lock it replaced, if any: [`LockFile::replaced`]).
    pub fn lock(&self) -> &LockFile {
        &self.lock
    }

    pub fn bundle(&self) -> &Path {
        &self.state.bundle
    }

    pub fn state(&self) -> &WorkState {
        &self.state
    }

    /// The project file in the working folder.
    pub fn project_file(&self) -> Result<PathBuf, IoError> {
        project::locate(&self.dir).map(|l| l.file)
    }

    /// Files changed since the bundle was opened or last saved.
    pub fn changed_files(&self) -> Result<Vec<String>, IoError> {
        changed_files(&self.dir, &self.state.entries)
    }

    /// Whether the bundle file is no longer the one opened or last saved (replaced, changed or
    /// deleted by someone else).
    pub fn bundle_changed(&self) -> bool {
        self.state.bundle_changed()
    }

    /// What the bundle holds now (see [`WorkState::held_contents`]). A plain *Save* keeps all of
    /// it: the working folder is the only other copy, and it is removed when the tab closes.
    pub fn held_contents(&self) -> BundleContents {
        self.state.held_contents()
    }

    fn check_lock(&self) -> Result<(), IoError> {
        if self.lock.is_held() {
            return Ok(());
        }
        Err(IoError::Locked { path: self.lock.path().to_path_buf(), owner: lock::holder(self.lock.path()).map(Box::new) })
    }

    /// *Save*: writes the working folder back into its bundle. Refused with
    /// [`IoError::Conflict`] when the bundle was changed by someone else since it was opened or
    /// last saved ([`WorkFolder::save_overwriting`] replaces it anyway), and with
    /// [`IoError::Locked`] when another app broke this app's lock.
    pub fn save(&mut self, contents: &BundleContents) -> Result<BundleReport, IoError> {
        self.check_lock()?;
        if self.state.bundle.exists() && self.bundle_changed() {
            return Err(IoError::Conflict(format!("{} was changed by someone else since it was opened; saving would replace that version", self.state.bundle.display())));
        }
        let dest = self.state.bundle.clone();
        self.write_to(&dest, contents)
    }

    /// *Save* over a bundle that was changed by someone else (after the user confirmed).
    pub fn save_overwriting(&mut self, contents: &BundleContents) -> Result<BundleReport, IoError> {
        self.check_lock()?;
        let dest = self.state.bundle.clone();
        self.write_to(&dest, contents)
    }

    /// *Save as Bundle…*: writes the working folder into another bundle (unchanged entries still
    /// copied raw from the current one), which this folder then belongs to; the old bundle is
    /// left as it is and unlocked.
    pub fn save_as(&mut self, dest: &Path, contents: &BundleContents) -> Result<BundleReport, IoError> {
        let dest = std::path::absolute(dest)?;
        if dest == self.state.bundle {
            return self.save(contents);
        }
        let lock = LockFile::lock_project(&dest)?;
        let report = self.write_to(&dest, contents)?;
        self.lock = lock;
        Ok(report)
    }

    fn write_to(&mut self, dest: &Path, contents: &BundleContents) -> Result<BundleReport, IoError> {
        let (report, entries) = bundle::save_back(&self.dir, &self.state.bundle, &self.state.entries, dest, contents)?;
        let (size, mtime) = stamp(dest).unwrap_or_default();
        self.state.bundle = dest.to_path_buf();
        self.state.bundle_size = size;
        self.state.bundle_mtime_ns = mtime;
        self.state.entries = entries;
        self.state.contents = Some(*contents);
        self.state.saved = Some(fsutil::unix_now());
        // A folder made by [`WorkFolder::create`] is complete once its bundle exists.
        self.state.complete = true;
        write_state(&self.dir, &self.state)?;
        Ok(report)
    }

    /// *Save As Project Folder*: copies the working folder into `dest` as a normal project folder
    /// (form A) and returns its project file. `dest` must not hold a project already. The project
    /// file is copied last, so an interrupted copy is not a project. The working folder stays
    /// (the app closes it when it switches the tab to `dest`).
    pub fn save_as_folder(&self, dest: &Path) -> Result<PathBuf, IoError> {
        if project::locate(dest).is_ok() {
            return Err(IoError::Unsupported(format!("{} already holds a project", dest.display())));
        }
        let project_file = self.project_file()?;
        let files: Vec<_> = fsutil::walk_files(&self.dir)?.into_iter().filter(|f| bundle::classify(&f.rel, &[]).is_some()).collect();
        fs::create_dir_all(dest)?;
        fsutil::check_free_space(dest, files.iter().map(|f| f.size).sum())?;
        let (last, rest): (Vec<_>, Vec<_>) = files.into_iter().partition(|f| f.path == project_file);
        for f in rest.iter().chain(&last) {
            let target = dest.join(&f.rel);
            if let Some(parent) = target.parent() {
                fs::create_dir_all(parent)?;
            }
            fs::copy(&f.path, &target)?;
        }
        Ok(dest.join(project_file.file_name().unwrap_or_default()))
    }

    /// Closes the working folder: removes it (unsaved changes are lost) and unlocks the bundle.
    pub fn close(self) -> Result<(), IoError> {
        remove_working_folder(&self.dir)
    }
}

/// A bundle unpacked only to be read (Add Project of a bundle, Open Read-Only): no lock is taken,
/// so it also works while the bundle is open elsewhere, and nothing is ever written back. The
/// folder is removed when this is dropped. Its state file marks it as being removed from the
/// start, so a crash while it exists leaves a folder the next start removes without asking
/// ([`LeftoverStatus::Closing`]); while this app runs it is in use and left alone.
#[derive(Debug)]
pub struct UnpackedCopy {
    dir: PathBuf,
    bundle: PathBuf,
}

impl UnpackedCopy {
    /// Unpacks `bundle_path` into a new folder under `root` (checks as [`WorkFolder::open`]:
    /// a newer or broken project file is refused before anything is unpacked, and the drive
    /// must have room). On failure nothing is left behind.
    pub fn unpack(bundle_path: &Path, root: &Path) -> Result<UnpackedCopy, IoError> {
        let bundle_path = std::path::absolute(bundle_path)?;
        let mut bundle = Bundle::open(&bundle_path)?;
        bundle.read_project()?;
        fs::create_dir_all(root)?;
        fsutil::check_free_space(root, bundle.total_size())?;
        let stem = bundle_path.file_stem().and_then(|s| project::file_stem_for(&s.to_string_lossy())).unwrap_or_else(|| "Project".into());
        let dir = create_unique(root, &format!("{stem}-copy"))?;
        let (bundle_size, bundle_mtime_ns) = stamp(&bundle_path).unwrap_or_default();
        let state = WorkState {
            format: STATE_FORMAT,
            bundle: bundle_path.clone(),
            bundle_size,
            bundle_mtime_ns,
            owner: Owner::this_process(),
            opened: fsutil::unix_now(),
            saved: None,
            complete: false,
            entries: Vec::new(),
            contents: None,
            closing: true,
        };
        let unpacked = write_state(&dir, &state).and_then(|()| bundle.extract_to(&dir).map(drop));
        if let Err(e) = unpacked {
            let _ = fs::remove_dir_all(&dir);
            return Err(e);
        }
        Ok(UnpackedCopy { dir, bundle: bundle_path })
    }

    /// The unpacked project folder.
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    pub fn bundle(&self) -> &Path {
        &self.bundle
    }
}

impl Drop for UnpackedCopy {
    fn drop(&mut self) {
        if let Err(e) = remove_working_folder(&self.dir) {
            log::warn!("could not remove the unpacked copy {}: {e}", self.dir.display());
        }
    }
}

/// Copies the files of `item` (for example [`Item::RawRecordings`], which the app does not write
/// itself) from the project folder `from` into the project folder `to`, where `to` has no file
/// of that name yet. Used when a project is saved as another form (folder to bundle, bundle to
/// folder) from memory: the app writes everything it holds, and this brings along what it does
/// not hold. Returns the number of files copied.
pub fn copy_item_files(from: &Path, to: &Path, item: Item) -> Result<usize, IoError> {
    let raw = match project::locate(from) {
        Ok(l) => project::read_project_file(&l.file).map(|p| bundle::raw_recording_paths(&p)).unwrap_or_default(),
        Err(_) => Vec::new(),
    };
    let files: Vec<_> = fsutil::walk_files(from)?.into_iter().filter(|f| bundle::classify(&f.rel, &raw) == Some(item) && !to.join(&f.rel).exists()).collect();
    fsutil::check_free_space(to, files.iter().map(|f| f.size).sum())?;
    for f in &files {
        let target = to.join(&f.rel);
        if let Some(parent) = target.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::copy(&f.path, &target)?;
    }
    Ok(files.len())
}

/// Removes a working folder: renamed to `<dir>.closing` first, so a removal that stops part-way
/// (a file held open) never leaves something that looks like a working folder with work in it.
/// When the rename keeps failing (on Windows: a file below the folder is open), the folder is
/// marked as closing in its state and removed in place, the state file last: a removal that
/// stops part-way still has its state, and the next start finishes it
/// ([`LeftoverStatus::Closing`]).
fn remove_working_folder(dir: &Path) -> Result<(), IoError> {
    let mut closing = dir.as_os_str().to_os_string();
    closing.push(CLOSING_SUFFIX);
    let closing = PathBuf::from(closing);
    for attempt in 0..3 {
        if fs::rename(dir, &closing).is_ok() {
            fs::remove_dir_all(&closing)?;
            return Ok(());
        }
        if attempt < 2 {
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
    if let Ok(mut state) = read_state(dir) {
        state.closing = true;
        write_state(dir, &state)?;
    }
    remove_in_place(dir)
}

/// Removes everything in `dir` but its state file (as much as can be removed), then the state
/// file and `dir`. On the first error the state file stays.
fn remove_in_place(dir: &Path) -> Result<(), IoError> {
    let mut first: Option<std::io::Error> = None;
    for entry in fs::read_dir(dir)?.flatten() {
        if entry.file_name() == STATE_FILE {
            continue;
        }
        let path = entry.path();
        let removed = if entry.file_type().is_ok_and(|t| t.is_dir()) { fs::remove_dir_all(&path) } else { fs::remove_file(&path) };
        if let Err(e) = removed {
            first.get_or_insert(e);
        }
    }
    if let Some(e) = first {
        return Err(e.into());
    }
    let state = dir.join(STATE_FILE);
    if state.exists() {
        fs::remove_file(&state)?;
    }
    fs::remove_dir(dir)?;
    Ok(())
}

/// A working folder found under the root.
#[derive(Debug, Clone, PartialEq)]
pub struct Leftover {
    pub dir: PathBuf,
    pub state: Option<WorkState>,
    pub status: LeftoverStatus,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LeftoverStatus {
    /// Its owner is running (this app, another one, or on another computer, which cannot be
    /// checked): leave it alone.
    InUse(Owner),
    /// Its owner ended without closing it. `changed`: files changed since the last save (empty:
    /// nothing to recover); `bundle_changed`: the bundle was changed by someone else since.
    Orphaned { changed: Vec<String>, bundle_changed: bool },
    /// Unpacking stopped part-way (or a new bundle's project was never written): nothing to
    /// recover, discard it.
    Incomplete,
    /// A removal that stopped part-way (a file was held open): nothing to recover, remove it.
    Closing,
    /// No readable state: discard it.
    Damaged(String),
}

/// The working folders under `root` (none when it does not exist).
pub fn leftovers(root: &Path) -> Vec<Leftover> {
    let Ok(dirs) = fs::read_dir(root) else { return Vec::new() };
    let mut out: Vec<Leftover> = dirs
        .flatten()
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
        .map(|e| {
            let dir = e.path();
            if dir.to_string_lossy().ends_with(CLOSING_SUFFIX) {
                return Leftover { dir, state: None, status: LeftoverStatus::Closing };
            }
            let state = match read_state(&dir) {
                Ok(s) => s,
                Err(e) => return Leftover { dir, state: None, status: LeftoverStatus::Damaged(e.to_string()) },
            };
            let status = if state.owner.is_this_process() || state.owner.liveness() != Liveness::Dead {
                LeftoverStatus::InUse(state.owner.clone())
            } else if state.closing {
                LeftoverStatus::Closing
            } else if !state.complete {
                LeftoverStatus::Incomplete
            } else {
                let changed = changed_files(&dir, &state.entries).unwrap_or_else(|e| vec![format!("(cannot be checked: {e})")]);
                LeftoverStatus::Orphaned { changed, bundle_changed: state.bundle_changed() }
            };
            Leftover { dir, state: Some(state), status }
        })
        .collect();
    out.sort_by(|a, b| a.dir.cmp(&b.dir));
    out
}

/// Removes a leftover working folder. Refused for one in use, and for a folder that is not a
/// working folder (no state file, not a stopped removal).
pub fn discard(dir: &Path) -> Result<(), IoError> {
    if dir.to_string_lossy().ends_with(CLOSING_SUFFIX) {
        fs::remove_dir_all(dir)?;
        return Ok(());
    }
    if !dir.join(STATE_FILE).is_file() {
        return Err(IoError::NotAProject(format!("{} is not a working folder", dir.display())));
    }
    if let Ok(state) = read_state(dir) {
        if state.owner.is_this_process() || state.owner.liveness() != Liveness::Dead {
            return Err(IoError::Locked { path: dir.to_path_buf(), owner: Some(Box::new(state.owner)) });
        }
    }
    remove_working_folder(dir)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::tests::{sample_project, temp};

    /// A process id that is not running: a child that has already exited.
    fn dead_pid() -> u32 {
        let exe = if cfg!(windows) { "cmd" } else { "true" };
        let args: &[&str] = if cfg!(windows) { &["/C", "exit 0"] } else { &[] };
        let mut child = std::process::Command::new(exe).args(args).spawn().unwrap();
        let pid = child.id();
        child.wait().unwrap();
        pid
    }

    fn sample_bundle(dir: &Path) -> PathBuf {
        let project = dir.join("Bracket");
        sample_project(&project);
        let bundle = dir.join("Bracket.scancraftz");
        bundle::write_bundle(&project, &bundle, &BundleContents::default()).unwrap();
        bundle
    }

    #[test]
    fn open_edit_save_close() {
        let dir = temp("work");
        let bundle_path = sample_bundle(&dir);
        let root = dir.join("root");
        let mut work = WorkFolder::open(&bundle_path, &root).unwrap();
        assert!(work.dir().starts_with(&root) && work.dir().join(STATE_FILE).is_file());
        assert_eq!(work.project_file().unwrap().file_name().unwrap(), "Bracket.scancraft");
        assert!(work.changed_files().unwrap().is_empty());
        // The bundle is locked while it is open.
        assert!(matches!(WorkFolder::open(&bundle_path, &root), Err(IoError::Locked { .. })));
        assert!(LockFile::path_for(&bundle_path).is_file());

        // Work on the project as a normal folder.
        let (mut project, kfs) = project::load_project(work.dir()).unwrap();
        project.name = "Bracket v2".into();
        project::save_project(work.dir(), &project, &kfs).unwrap();
        fs::write(work.dir().join("outputs/mesh.ply"), b"a new mesh").unwrap();
        let changed = work.changed_files().unwrap();
        assert!(changed.contains(&"Bracket.scancraft".to_string()) && changed.contains(&"outputs/mesh.ply".to_string()), "{changed:?}");
        let report = work.save(&BundleContents::default()).unwrap();
        assert!(report.copied_raw > 0 && report.written >= 2, "{report:?}");
        assert!(work.changed_files().unwrap().is_empty());
        assert!(!work.bundle_changed());
        let mut reopened = Bundle::open(&bundle_path).unwrap();
        assert_eq!(reopened.read_project().unwrap().name, "Bracket v2");
        assert_eq!(reopened.read("outputs/mesh.ply").unwrap(), b"a new mesh");
        assert!(reopened.verify().unwrap().is_ok());
        drop(reopened);

        // Someone replaces the bundle: Save refuses, Save overwriting replaces it.
        std::thread::sleep(std::time::Duration::from_millis(20));
        let other = dir.join("Other");
        sample_project(&other);
        let theirs = dir.join("theirs.scancraftz");
        bundle::write_bundle(&other, &theirs, &BundleContents::default()).unwrap();
        fs::copy(&theirs, &bundle_path).unwrap();
        assert!(work.bundle_changed());
        assert!(matches!(work.save(&BundleContents::default()), Err(IoError::Conflict(_))));
        work.save_overwriting(&BundleContents::default()).unwrap();
        assert_eq!(Bundle::open(&bundle_path).unwrap().read_project().unwrap().name, "Bracket v2");

        // Save as Bundle moves the folder (and the lock) to the new bundle.
        let copy = dir.join("Copy.scancraftz");
        work.save_as(&copy, &BundleContents::default()).unwrap();
        assert_eq!(work.bundle(), std::path::absolute(&copy).unwrap());
        assert!(!LockFile::path_for(&bundle_path).exists() && LockFile::path_for(&copy).is_file());
        assert!(Bundle::open(&copy).unwrap().verify().unwrap().is_ok());

        // Save As Project Folder.
        let folder = dir.join("As folder");
        let file = work.save_as_folder(&folder).unwrap();
        assert_eq!(file, folder.join("Bracket.scancraft"));
        assert!(!folder.join(STATE_FILE).exists());
        assert_eq!(project::load_project(&file).unwrap().0.name, "Bracket v2");
        assert!(work.save_as_folder(&folder).is_err(), "never over another project");

        let wd = work.dir().to_path_buf();
        work.close().unwrap();
        assert!(!wd.exists() && !LockFile::path_for(&copy).exists());
        assert!(leftovers(&root).is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn leftovers_after_a_crash_are_recovered_or_discarded() {
        let dir = temp("leftover");
        let bundle_path = sample_bundle(&dir);
        let root = dir.join("root");
        let work = WorkFolder::open(&bundle_path, &root).unwrap();
        let wd = work.dir().to_path_buf();
        // In use by this (running) app: listed as such, never discarded.
        let found = leftovers(&root);
        assert_eq!(found.len(), 1);
        assert!(matches!(&found[0].status, LeftoverStatus::InUse(o) if o.pid == std::process::id()));
        assert!(matches!(discard(&wd), Err(IoError::Locked { .. })));
        assert!(matches!(WorkFolder::recover(&wd), Err(IoError::Locked { .. })));

        // The app "crashes": the folder and the lock stay, their owner is gone.
        fs::write(wd.join("pass-1/notes.txt"), b"unsaved").unwrap();
        let lock_path = LockFile::path_for(&bundle_path);
        std::mem::forget(work);
        let mut state = read_state(&wd).unwrap();
        state.owner.pid = dead_pid();
        state.owner.process_started_ms = None;
        write_state(&wd, &state).unwrap();
        fs::write(&lock_path, serde_json::to_vec(&state.owner).unwrap()).unwrap();
        let found = leftovers(&root);
        assert_eq!(found[0].status, LeftoverStatus::Orphaned { changed: vec!["pass-1/notes.txt".into()], bundle_changed: false });

        // Recover: the stale lock is taken over and the unsaved work saved.
        let mut work = WorkFolder::recover(&wd).unwrap();
        assert!(work.state().owner.is_this_process());
        work.save(&BundleContents::default()).unwrap();
        assert_eq!(Bundle::open(&bundle_path).unwrap().read("pass-1/notes.txt").unwrap(), b"unsaved");
        drop(work);

        // A folder whose unpacking stopped: discard only.
        let mut state = read_state(&wd).unwrap();
        state.owner.pid = dead_pid();
        state.owner.process_started_ms = None;
        state.complete = false;
        write_state(&wd, &state).unwrap();
        assert_eq!(leftovers(&root)[0].status, LeftoverStatus::Incomplete);
        assert!(WorkFolder::recover(&wd).is_err());
        discard(&wd).unwrap();
        assert!(!wd.exists() && leftovers(&root).is_empty());

        // A stopped removal and an unreadable state are damaged; any other folder is not ours.
        fs::create_dir_all(root.join("Part-1234.closing")).unwrap();
        fs::create_dir_all(root.join("Broken")).unwrap();
        fs::write(root.join("Broken").join(STATE_FILE), b"{").unwrap();
        fs::create_dir_all(root.join("Foreign")).unwrap();
        let found = leftovers(&root);
        assert_eq!(found.len(), 3);
        assert_eq!(found.iter().filter(|l| l.status == LeftoverStatus::Closing).count(), 1);
        assert_eq!(found.iter().filter(|l| matches!(l.status, LeftoverStatus::Damaged(_))).count(), 2);
        assert!(discard(&root.join("Foreign")).is_err());
        discard(&root.join("Part-1234.closing")).unwrap();
        discard(&root.join("Broken")).unwrap();
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_new_bundle_is_written_from_a_created_folder() {
        let dir = temp("create");
        let root = dir.join("root");
        let dest = dir.join("New part.scancraftz");
        let mut work = WorkFolder::create(&dest, &root).unwrap();
        assert!(matches!(WorkFolder::create(&dest, &root), Err(IoError::Locked { .. })), "the new bundle is locked");
        // Before the first save it holds nothing to recover.
        assert_eq!(leftovers(&root)[0].status, LeftoverStatus::InUse(work.state().owner.clone()));
        assert!(!work.state().complete);
        sample_project(work.dir());
        fs::create_dir_all(dir.join("Old/pass-1/raw")).unwrap();
        fs::write(dir.join("Old/pass-1/raw/stream.bin"), b"raw").unwrap();
        fs::write(dir.join("Old/pass-1/keyframes.bin"), b"old keyframes").unwrap();
        // Only the item's files that are missing are copied (the keyframes are the app's own).
        let before = fs::read(work.dir().join("pass-1/keyframes.bin")).unwrap();
        assert_eq!(copy_item_files(&dir.join("Old"), work.dir(), Item::RawRecordings).unwrap(), 1);
        assert_eq!(fs::read(work.dir().join("pass-1/raw/stream.bin")).unwrap(), b"raw");
        assert_eq!(fs::read(work.dir().join("pass-1/keyframes.bin")).unwrap(), before);
        let mut everything = BundleContents::default();
        everything.raw_recordings = true;
        work.save(&everything).unwrap();
        assert!(work.state().complete && work.changed_files().unwrap().is_empty());
        let mut b = Bundle::open(&dest).unwrap();
        assert!(b.verify().unwrap().is_ok());
        assert_eq!(b.read("pass-1/raw/stream.bin").unwrap(), b"raw");
        work.close().unwrap();
        assert!(leftovers(&root).is_empty() && !LockFile::path_for(&dest).exists());
        let _ = fs::remove_dir_all(&dir);
    }

    /// Makes the working folder `wd` look as if its app had crashed: owned by a process that is
    /// gone, with the bundle's lock left behind.
    fn orphan(work: WorkFolder) -> PathBuf {
        let wd = work.dir().to_path_buf();
        std::mem::forget(work);
        let mut state = read_state(&wd).unwrap();
        state.owner.pid = dead_pid();
        state.owner.process_started_ms = None;
        write_state(&wd, &state).unwrap();
        fs::write(LockFile::path_for(&state.bundle), serde_json::to_vec(&state.owner).unwrap()).unwrap();
        wd
    }

    #[test]
    fn a_crash_while_the_first_bundle_is_written_keeps_the_project_recoverable() {
        let dir = temp("first-save");
        let root = dir.join("root");
        let dest = dir.join("New part.scancraftz");
        let mut work = WorkFolder::create(&dest, &root).unwrap();
        assert!(work.project_written().is_err(), "no project file yet");
        sample_project(work.dir());
        work.project_written().unwrap();
        // The app ends while the bundle is written: no bundle, a complete project in the folder.
        let wd = orphan(work);
        assert!(!dest.exists());
        let found = leftovers(&root);
        let LeftoverStatus::Orphaned { changed, bundle_changed } = &found[0].status else { panic!("{:?}", found[0].status) };
        assert!(!changed.is_empty() && !bundle_changed, "{changed:?}: the never-written bundle is not 'changed by someone else'");
        // Recover and save: the bundle is written from the folder.
        let mut work = WorkFolder::recover(&wd).unwrap();
        work.save(&BundleContents::everything()).unwrap();
        assert!(Bundle::open(&dest).unwrap().verify().unwrap().is_ok());
        assert!(work.changed_files().unwrap().is_empty());
        work.close().unwrap();
        assert!(leftovers(&root).is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn what_a_bundle_holds_is_known_from_its_manifest_and_its_saves() {
        let dir = temp("held");
        let project = dir.join("Bracket");
        sample_project(&project);
        fs::create_dir_all(project.join("pass-1/raw")).unwrap();
        fs::write(project.join("pass-1/raw/stream.bin"), b"raw").unwrap();
        let bundle_path = dir.join("Bracket.scancraftz");
        bundle::write_bundle(&project, &bundle_path, &BundleContents::everything()).unwrap();
        let root = dir.join("root");
        let mut work = WorkFolder::open(&bundle_path, &root).unwrap();
        assert_eq!(work.held_contents(), BundleContents::everything());
        // A plain Save with settings that leave raw recordings out keeps what the bundle holds.
        let keep = work.held_contents().union(&BundleContents::default());
        work.save(&keep).unwrap();
        assert_eq!(Bundle::open(&bundle_path).unwrap().read("pass-1/raw/stream.bin").unwrap(), b"raw");
        // A save that does leave them out is what the bundle holds from then on.
        work.save(&BundleContents::default()).unwrap();
        assert!(Bundle::open(&bundle_path).unwrap().read("pass-1/raw/stream.bin").is_err());
        assert_eq!(work.held_contents(), BundleContents::default());
        // A state without recorded contents: the items of its entries.
        let state = WorkState { contents: None, ..work.state().clone() };
        assert!(state.held_contents().keyframes && !state.held_contents().raw_recordings);
        work.close().unwrap();
        let _ = fs::remove_dir_all(&dir);
    }

    /// On Windows a file opened without delete sharing stops both the rename and the removal of
    /// the folder: the removal is finished at the next start, never offered as recoverable work.
    #[cfg(windows)]
    #[test]
    fn a_removal_stopped_by_an_open_file_is_finished_later() {
        use std::os::windows::fs::OpenOptionsExt;
        let dir = temp("held-open");
        let bundle_path = sample_bundle(&dir);
        let root = dir.join("root");
        let work = WorkFolder::open(&bundle_path, &root).unwrap();
        let wd = work.dir().to_path_buf();
        let held = wd.join("pass-1/keyframes.bin");
        // FILE_SHARE_READ only, as some viewers and virus scanners open files.
        let handle = fs::OpenOptions::new().read(true).share_mode(1).open(&held).unwrap();
        assert!(work.close().is_err());
        assert!(wd.join(STATE_FILE).is_file(), "the state file is removed last");
        assert!(!wd.join("Bracket.scancraft").exists(), "everything else that could go is gone");
        let mut state = read_state(&wd).unwrap();
        assert!(state.closing);
        state.owner.pid = dead_pid();
        state.owner.process_started_ms = None;
        write_state(&wd, &state).unwrap();
        assert_eq!(leftovers(&root)[0].status, LeftoverStatus::Closing);
        assert!(WorkFolder::recover(&wd).is_err(), "nothing to recover");
        drop(handle);
        discard(&wd).unwrap();
        assert!(!wd.exists() && leftovers(&root).is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_unpacked_copy_needs_no_lock_and_leaves_nothing_behind() {
        let dir = temp("copy");
        let bundle_path = sample_bundle(&dir);
        let root = dir.join("root");
        // The bundle is open (locked) elsewhere: a copy to read from still works.
        let open = WorkFolder::open(&bundle_path, &root).unwrap();
        let copy = UnpackedCopy::unpack(&bundle_path, &root).unwrap();
        assert!(copy.dir().starts_with(&root) && copy.dir() != open.dir());
        assert_eq!(project::load_project(copy.dir()).unwrap().0.name, "Bracket");
        assert!(copy.bundle().ends_with("Bracket.scancraftz"));
        // While this app runs, it is in use (never recovered or removed by another start).
        let listed = leftovers(&root);
        assert!(listed.iter().all(|l| matches!(l.status, LeftoverStatus::InUse(_))), "{listed:?}");
        let copy_dir = copy.dir().to_path_buf();
        drop(copy);
        assert!(!copy_dir.exists(), "removed when dropped");
        assert!(LockFile::path_for(&bundle_path).is_file(), "the other holder's lock is untouched");
        open.close().unwrap();

        // After a crash it is removed without a question: it never held work.
        let copy = UnpackedCopy::unpack(&bundle_path, &root).unwrap();
        let copy_dir = copy.dir().to_path_buf();
        std::mem::forget(copy);
        let mut state = read_state(&copy_dir).unwrap();
        state.owner.pid = dead_pid();
        state.owner.process_started_ms = None;
        write_state(&copy_dir, &state).unwrap();
        assert_eq!(leftovers(&root).iter().map(|l| l.status.clone()).collect::<Vec<_>>(), vec![LeftoverStatus::Closing]);
        assert!(WorkFolder::recover(&copy_dir).is_err());
        discard(&copy_dir).unwrap();
        assert!(leftovers(&root).is_empty());
        // A damaged bundle leaves nothing either.
        let junk = dir.join("junk.scancraftz");
        fs::write(&junk, b"not a zip").unwrap();
        assert!(UnpackedCopy::unpack(&junk, &root).is_err());
        assert!(leftovers(&root).is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_bundle_locked_on_another_computer_is_opened_only_when_taken_over() {
        let dir = temp("take-over");
        let bundle_path = sample_bundle(&dir);
        let root = dir.join("root");
        let mut remote = Owner::this_process();
        remote.host = "OTHER-PC".into();
        fs::write(LockFile::path_for(&bundle_path), serde_json::to_vec(&remote).unwrap()).unwrap();
        match WorkFolder::open(&bundle_path, &root) {
            Err(IoError::Locked { owner: Some(o), .. }) => assert!(o.may_be_taken_over()),
            other => panic!("{other:?}"),
        }
        let work = WorkFolder::open_taking_over(&bundle_path, &root, &remote).unwrap();
        assert_eq!(work.lock().replaced().map(|o| o.host.as_str()), Some("OTHER-PC"));
        assert!(work.lock().is_held());
        // Never from a running app on this computer (this one, here).
        let mine = work.lock().owner().clone();
        assert!(matches!(WorkFolder::open_taking_over(&bundle_path, &root, &mine), Err(IoError::Locked { .. })));
        work.close().unwrap();
        assert!(leftovers(&root).is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn opening_needs_room_and_a_readable_bundle() {
        let dir = temp("open-errors");
        let root = dir.join("root");
        let junk = dir.join("junk.scancraftz");
        fs::write(&junk, b"not a zip").unwrap();
        assert!(WorkFolder::open(&junk, &root).is_err());
        assert!(!LockFile::path_for(&junk).exists(), "the lock is released after a failed open");
        assert!(leftovers(&root).is_empty(), "nothing left behind");
        assert!(matches!(fsutil::check_free_space(&dir, u64::MAX / 4), Err(IoError::NoSpace { .. })));
        let message = fsutil::check_free_space(&dir, u64::MAX / 4).unwrap_err().to_string();
        assert!(message.contains("free space"), "{message}");
        assert!(default_root().ends_with(Path::new("ScanCraft").join("work")));
        let _ = fs::remove_dir_all(&dir);
    }
}
