//! Where an open project lives on disk, and the project-file preferences
//! (`docs/open-issues.md` 4.1, `docs/architecture/scan-sessions.md` section 8).
//!
//! A tab's project is either not saved yet, a project folder (form A: `<name>.scancraft` next to
//! `pass-<id>/` and `outputs/`), or a bundle (form B: one `<name>.scancraftz` file) that is
//! unpacked into a working folder while the tab is open. Either is locked while open
//! (`scancraft_io::lock`), so a second app or PC cannot save over it. Closing the tab releases
//! the lock and removes a bundle's working folder; after a crash the working folder stays and the
//! next start offers to recover it ([`crate::app::ScanCraftApp::check_leftovers`]).
//!
//! The preferences (working-folder root, what a bundle holds) are kept in
//! `<local app data>/ScanCraft/settings.json`. The app has no Settings window yet: they are edited
//! in the Save Project card when saving as a bundle.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use scancraft_io::bundle::BundleContents;
use scancraft_io::lock::LockFile;
use scancraft_io::workdir::{self, WorkFolder};
use serde::{Deserialize, Serialize};

/// The disk side of an open project.
#[derive(Default)]
pub enum Store {
    /// Not saved yet.
    #[default]
    None,
    /// A project folder, locked while the tab is open.
    Folder(LockFile),
    /// A bundle opened into a working folder (shared with a running save job). `dir` is the
    /// working folder, which never changes (read without locking `work`).
    Bundle { work: Arc<Mutex<WorkFolder>>, dir: PathBuf },
}

impl std::fmt::Debug for Store {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Store::None => write!(f, "Store::None"),
            Store::Folder(lock) => write!(f, "Store::Folder({})", lock.path().display()),
            Store::Bundle { dir, .. } => write!(f, "Store::Bundle({})", dir.display()),
        }
    }
}

impl Store {
    pub fn bundle(work: WorkFolder) -> Store {
        let dir = work.dir().to_path_buf();
        Store::Bundle { work: Arc::new(Mutex::new(work)), dir }
    }

    pub fn is_bundle(&self) -> bool {
        matches!(self, Store::Bundle { .. })
    }

    /// The working folder of a bundle tab.
    pub fn work_dir(&self) -> Option<PathBuf> {
        match self {
            Store::Bundle { dir, .. } => Some(dir.clone()),
            _ => None,
        }
    }

    /// The folder the project's files are in right now: the project folder, or a bundle's
    /// working folder.
    pub fn data_dir(&self) -> Option<PathBuf> {
        match self {
            Store::None => None,
            Store::Folder(lock) => Some(scancraft_io::lock::locked_target(lock.path())),
            Store::Bundle { .. } => self.work_dir(),
        }
    }

    /// Whether this store is the project folder `dir` (already locked by this tab).
    pub fn is_folder(&self, dir: &Path) -> bool {
        match self {
            Store::Folder(lock) => same_path(&scancraft_io::lock::locked_target(lock.path()), dir),
            _ => false,
        }
    }

    /// Releases the lock; a bundle's working folder is removed (its unsaved changes are lost).
    /// A working folder still used by a job (it cannot be) is kept and listed at the next start.
    pub fn close(self) {
        if let Store::Bundle { work, .. } = self {
            match Arc::try_unwrap(work) {
                Ok(m) => {
                    let work = m.into_inner().unwrap_or_else(|p| p.into_inner());
                    let dir = work.dir().to_path_buf();
                    if let Err(e) = work.close() {
                        log::warn!("could not remove the working folder {}: {e}", dir.display());
                    }
                }
                Err(_) => log::warn!("a working folder is still in use; it is kept and listed at the next start"),
            }
        }
    }
}

/// Whether two paths name the same file or folder (absolute, case-insensitive on Windows).
pub fn same_path(a: &Path, b: &Path) -> bool {
    let norm = |p: &Path| {
        let p = std::path::absolute(p).unwrap_or_else(|_| p.to_path_buf());
        let s = p.to_string_lossy().trim_end_matches(['\\', '/']).replace('\\', "/");
        if cfg!(windows) {
            s.to_lowercase()
        } else {
            s
        }
    };
    norm(a) == norm(b)
}

/// What a path opens as.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpenTarget {
    /// A project folder (given as the folder, its `.scancraft` file or an older `project.json`).
    Folder(PathBuf),
    /// A bundle file.
    Bundle(PathBuf),
}

impl OpenTarget {
    /// `None` when `path` is neither a project nor a bundle.
    pub fn of(path: &Path) -> Option<OpenTarget> {
        if scancraft_io::project::is_bundle(path) {
            return Some(OpenTarget::Bundle(std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf())));
        }
        // (An older project is also found by its project.json: the Open dialog picks files.)
        let root = scancraft_io::project::project_root(path).ok()?;
        Some(OpenTarget::Folder(std::path::absolute(&root).unwrap_or(root)))
    }

    pub fn path(&self) -> &Path {
        match self {
            OpenTarget::Folder(p) | OpenTarget::Bundle(p) => p,
        }
    }
}

/// A project waiting to be opened once the app is free.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpenRequest {
    /// Open a project folder, project file or bundle.
    Path(PathBuf),
    /// Recover a leftover working folder (crash recovery).
    Recover(PathBuf),
}

/// The project-file preferences (kept across starts).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Prefs {
    /// Where bundles are unpacked while open; `None`: under the user's local app data
    /// ([`workdir::default_root`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub work_root: Option<PathBuf>,
    /// Earlier working-folder roots that may still hold working folders (of bundles open when
    /// the root was changed): crash recovery looks there too, and nothing is saved into them.
    /// A root is forgotten once nothing is left in it.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub past_work_roots: Vec<PathBuf>,
    /// What Save as Bundle… puts into a bundle.
    #[serde(default)]
    pub bundle_contents: BundleContents,
    /// Settings of later versions, kept as they are.
    #[serde(flatten)]
    pub other: serde_json::Map<String, serde_json::Value>,
}

impl Default for Prefs {
    fn default() -> Self {
        Prefs { work_root: None, past_work_roots: Vec::new(), bundle_contents: BundleContents::default(), other: Default::default() }
    }
}

impl Prefs {
    /// `<local app data>/ScanCraft/settings.json`.
    pub fn default_path() -> PathBuf {
        scancraft_io::fsutil::local_data_dir().join("ScanCraft").join("settings.json")
    }

    /// The preferences in `path`; the defaults when it does not exist or cannot be read.
    pub fn load(path: &Path) -> Prefs {
        match std::fs::read(path) {
            Ok(bytes) => serde_json::from_slice(&bytes).unwrap_or_else(|e| {
                log::warn!("{} cannot be read ({e}); using the default settings", path.display());
                Prefs::default()
            }),
            Err(_) => Prefs::default(),
        }
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        let json = serde_json::to_vec_pretty(self).map_err(|e| e.to_string())?;
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        scancraft_io::fsutil::atomic_write(path, &json).map_err(|e| e.to_string())
    }

    /// The working-folder root in use.
    pub fn work_root(&self) -> PathBuf {
        self.work_root.clone().filter(|p| !p.as_os_str().is_empty()).unwrap_or_else(workdir::default_root)
    }

    /// Changes the working-folder root. The one in use so far is remembered (open bundles keep
    /// their working folders there until their tabs close).
    pub fn set_work_root(&mut self, root: PathBuf) {
        let old = self.work_root();
        if !same_path(&old, &root) && !self.past_work_roots.iter().any(|p| same_path(p, &old)) {
            self.past_work_roots.push(old);
        }
        self.past_work_roots.retain(|p| !same_path(p, &root));
        self.work_root = Some(root);
    }

    /// Every root that may hold working folders: the one in use first, then the earlier ones.
    pub fn work_roots(&self) -> Vec<PathBuf> {
        let mut roots = vec![self.work_root()];
        for p in &self.past_work_roots {
            if !roots.iter().any(|r| same_path(r, p)) {
                roots.push(p.clone());
            }
        }
        roots
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-storage-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn prefs_round_trip_and_keep_unknown_settings() {
        let dir = temp("prefs");
        let path = dir.join("settings.json");
        assert_eq!(Prefs::load(&path), Prefs::default(), "missing file: defaults");
        std::fs::write(&path, br#"{"bundle_contents":{"raw_recordings":true},"future":{"x":1}}"#).unwrap();
        let mut prefs = Prefs::load(&path);
        assert!(prefs.bundle_contents.raw_recordings && prefs.bundle_contents.keyframes, "missing fields keep their defaults");
        prefs.work_root = Some(dir.join("work"));
        prefs.save(&path).unwrap();
        let back = Prefs::load(&path);
        assert_eq!(back, prefs);
        assert_eq!(back.other.get("future"), Some(&serde_json::json!({"x": 1})), "a later version's settings are kept");
        assert_eq!(back.work_root(), dir.join("work"));
        std::fs::write(&path, b"{ broken").unwrap();
        assert_eq!(Prefs::load(&path), Prefs::default());
        assert_eq!(Prefs { work_root: Some(PathBuf::new()), ..Prefs::default() }.work_root(), workdir::default_root());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn earlier_working_folder_roots_are_remembered() {
        let mut prefs = Prefs::default();
        let (a, b) = (PathBuf::from("E:/Work"), PathBuf::from("F:/Work"));
        prefs.set_work_root(a.clone());
        assert_eq!(prefs.work_roots(), vec![a.clone(), workdir::default_root()]);
        prefs.set_work_root(b.clone());
        prefs.set_work_root(b.clone());
        assert_eq!(prefs.work_roots(), vec![b.clone(), workdir::default_root(), a.clone()]);
        // Back to an earlier root: it is the one in use again, listed once.
        prefs.set_work_root(a.clone());
        assert_eq!(prefs.work_roots(), vec![a, workdir::default_root(), b]);
    }

    #[test]
    fn open_targets_are_told_apart() {
        let dir = temp("targets");
        assert_eq!(OpenTarget::of(&dir), None, "an empty folder is no project");
        std::fs::write(dir.join("Part.scancraft"), b"{}").unwrap();
        assert_eq!(OpenTarget::of(&dir.join("Part.scancraft")), Some(OpenTarget::Folder(dir.clone())));
        assert_eq!(OpenTarget::of(&dir), Some(OpenTarget::Folder(dir.clone())));
        let old = dir.join("old");
        std::fs::create_dir_all(&old).unwrap();
        std::fs::write(old.join("project.json"), b"{}").unwrap();
        assert_eq!(OpenTarget::of(&old.join("project.json")), Some(OpenTarget::Folder(old.clone())), "an older project by its project.json");
        std::fs::write(dir.join("Part.scancraftz"), b"zip").unwrap();
        assert_eq!(OpenTarget::of(&dir.join("Part.scancraftz")), Some(OpenTarget::Bundle(dir.join("Part.scancraftz"))));
        assert_eq!(OpenTarget::of(&dir.join("Missing.scancraftz")), None);
        assert!(same_path(&dir.join("a"), &dir.join("a").join("")));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
