//! Lock files, so two ScanCraft instances (on one PC, or on two PCs through a shared folder) never
//! save over each other's project (`docs/open-issues.md` 4.1).
//!
//! ```text
//! <project>/.scancraft.lock     a project folder (form A) is open
//! <bundle>.scancraftz.lock      a bundle (form B) is open
//! ```
//!
//! A lock is a small JSON file naming its owner: process id, host, the process's start time and
//! when the lock was taken. It is created with `create_new`, so of two instances racing for it
//! only one wins. A lock whose owner is gone (same host, process no longer running, or the
//! process id now belongs to a later process) is stale and taken over; a live one is refused with
//! who holds it ([`IoError::Locked`]). A lock from another host cannot be checked and counts as
//! live; [`LockFile::take_over`] breaks it once the user confirmed. The lock is removed when its
//! [`LockFile`] is dropped (only if it is still ours).

use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Serialize};

use crate::fsutil;
use crate::IoError;

/// The lock file inside a project folder.
pub const FOLDER_LOCK_FILE: &str = ".scancraft.lock";

/// A lock that cannot be read and is younger than this may be one being written right now.
const UNREADABLE_GRACE_SECS: u64 = 10;

/// Who holds a lock or a working folder.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Owner {
    #[serde(default)]
    pub app: String,
    pub pid: u32,
    pub host: String,
    /// Start of the owning process (ms since 1970), to tell a reused process id from the owner.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub process_started_ms: Option<u64>,
    /// When the lock was taken (s since 1970).
    pub since: u64,
    /// Distinguishes two locks of one process, so a lock is only ever removed by its taker.
    #[serde(default)]
    pub token: String,
}

/// Whether an owner is still running.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Liveness {
    Alive,
    Dead,
    /// On another host: cannot be checked from here.
    Unknown,
}

impl Owner {
    /// This process, now.
    pub fn this_process() -> Owner {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let pid = std::process::id();
        let n = NEXT.fetch_add(1, Ordering::Relaxed);
        Owner {
            app: format!("ScanCraft {}", env!("CARGO_PKG_VERSION")),
            pid,
            host: fsutil::host_name(),
            process_started_ms: process_started_ms(pid),
            since: fsutil::unix_now(),
            token: format!("{pid:x}-{:x}-{n:x}", fsutil::system_ns(std::time::SystemTime::now())),
        }
    }

    /// Whether the owner is this very process.
    pub fn is_this_process(&self) -> bool {
        self.pid == std::process::id() && self.host.eq_ignore_ascii_case(&fsutil::host_name()) && same_start(self.process_started_ms, process_started_ms(self.pid))
    }

    pub fn liveness(&self) -> Liveness {
        if !self.host.eq_ignore_ascii_case(&fsutil::host_name()) {
            return Liveness::Unknown;
        }
        match process_alive(self.pid) {
            false => Liveness::Dead,
            // A running process with the owner's id that started at another time is a later
            // process that got the id after the owner ended.
            true if !same_start(self.process_started_ms, process_started_ms(self.pid)) => Liveness::Dead,
            true => Liveness::Alive,
        }
    }

    /// "ScanCraft 0.2.0 (process 1234 on PC-2, since 2026-11-26 10:15 UTC)".
    pub fn describe(&self) -> String {
        let app = if self.app.is_empty() { "ScanCraft" } else { &self.app };
        format!("{app} (process {} on {}, since {})", self.pid, self.host, fsutil::format_utc(self.since))
    }
}

fn same_start(recorded: Option<u64>, now: Option<u64>) -> bool {
    match (recorded, now) {
        (Some(a), Some(b)) => a.abs_diff(b) <= 1000,
        _ => true,
    }
}

/// A held lock; removed on drop.
#[derive(Debug)]
pub struct LockFile {
    path: PathBuf,
    owner: Owner,
}

impl LockFile {
    /// The lock of a project folder (or of the folder of a `.scancraft` file) or of a bundle (any
    /// other file, existing or not: `<file>.lock` beside it).
    pub fn path_for(project: &Path) -> PathBuf {
        let is_bundle = crate::project::has_extension(project, crate::project::BUNDLE_EXTENSION);
        let is_project_file = crate::project::has_extension(project, crate::project::PROJECT_EXTENSION) && !project.is_dir();
        if is_project_file {
            project.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new(".")).join(FOLDER_LOCK_FILE)
        } else if is_bundle || project.is_file() {
            let mut name = project.file_name().map(|n| n.to_os_string()).unwrap_or_default();
            name.push(".lock");
            project.with_file_name(name)
        } else {
            project.join(FOLDER_LOCK_FILE)
        }
    }

    /// Locks a project folder, a `.scancraft` file's folder or a bundle ([`LockFile::path_for`]).
    pub fn lock_project(project: &Path) -> Result<LockFile, IoError> {
        LockFile::acquire(&LockFile::path_for(project))
    }

    /// Takes the lock at `path`: creates it, or takes over a stale one. A live lock (also one of
    /// this process: the project is already open here) is refused with [`IoError::Locked`].
    pub fn acquire(path: &Path) -> Result<LockFile, IoError> {
        for _ in 0..4 {
            match LockFile::create(path) {
                Ok(lock) => return Ok(lock),
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {}
                Err(e) => return Err(e.into()),
            }
            match read_holder(path) {
                Holder::Gone => continue,
                Holder::Owner(owner) if owner.liveness() == Liveness::Dead => {
                    log::info!("taking over the stale lock {} of {}", path.display(), owner.describe());
                    break_lock(path)?;
                }
                Holder::Unreadable { age_secs } if age_secs > UNREADABLE_GRACE_SECS => {
                    log::warn!("taking over the unreadable lock {}", path.display());
                    break_lock(path)?;
                }
                Holder::Owner(owner) => return Err(IoError::Locked { path: path.to_path_buf(), owner: Some(Box::new(owner)) }),
                Holder::Unreadable { .. } => return Err(IoError::Locked { path: path.to_path_buf(), owner: None }),
            }
        }
        Err(IoError::Locked { path: path.to_path_buf(), owner: holder(path).map(Box::new) })
    }

    /// Takes the lock whoever holds it (after the user confirmed that the other holder, for
    /// example on another PC, is not really using the project).
    pub fn take_over(path: &Path) -> Result<LockFile, IoError> {
        if path.exists() {
            break_lock(path)?;
        }
        LockFile::acquire(path)
    }

    fn create(path: &Path) -> io::Result<LockFile> {
        let owner = Owner::this_process();
        let mut f = OpenOptions::new().write(true).create_new(true).open(path)?;
        let json = serde_json::to_vec_pretty(&owner).map_err(io::Error::other)?;
        let written = f.write_all(&json).and_then(|_| f.sync_all());
        if let Err(e) = written {
            drop(f);
            let _ = fs::remove_file(path);
            return Err(e);
        }
        Ok(LockFile { path: path.to_path_buf(), owner })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn owner(&self) -> &Owner {
        &self.owner
    }

    /// Whether the file on disk still names this lock (someone may have broken it).
    pub fn is_held(&self) -> bool {
        matches!(read_holder(&self.path), Holder::Owner(o) if o.token == self.owner.token)
    }

    /// Releases the lock now (the same as dropping it).
    pub fn release(self) {}
}

impl Drop for LockFile {
    fn drop(&mut self) {
        if self.is_held() {
            let _ = fs::remove_file(&self.path);
        }
    }
}

/// What a lock file locks: the project folder or the bundle.
pub fn locked_target(lock: &Path) -> PathBuf {
    match lock.file_name().and_then(|n| n.to_str()) {
        Some(FOLDER_LOCK_FILE) => lock.parent().map(Path::to_path_buf).unwrap_or_default(),
        Some(name) if name.ends_with(".lock") => lock.with_file_name(&name[..name.len() - 5]),
        _ => lock.to_path_buf(),
    }
}

/// Who holds the lock at `path`, if anyone (and the lock can be read).
pub fn holder(path: &Path) -> Option<Owner> {
    match read_holder(path) {
        Holder::Owner(o) => Some(o),
        _ => None,
    }
}

/// `Err(Locked)` when a live process other than this one holds the lock at `path`.
pub fn check_not_held_by_other(path: &Path) -> Result<(), IoError> {
    match read_holder(path) {
        Holder::Owner(o) if !o.is_this_process() && o.liveness() != Liveness::Dead => Err(IoError::Locked { path: path.to_path_buf(), owner: Some(Box::new(o)) }),
        _ => Ok(()),
    }
}

enum Holder {
    Gone,
    Owner(Owner),
    Unreadable { age_secs: u64 },
}

fn read_holder(path: &Path) -> Holder {
    let text = match fs::read(path) {
        Ok(t) => t,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Holder::Gone,
        Err(_) => return Holder::Unreadable { age_secs: 0 },
    };
    match serde_json::from_slice::<Owner>(&text) {
        Ok(o) => Holder::Owner(o),
        Err(_) => {
            let age = fs::metadata(path).and_then(|m| m.modified()).ok().and_then(|t| t.elapsed().ok()).map_or(0, |d| d.as_secs());
            Holder::Unreadable { age_secs: age }
        }
    }
}

/// Removes a lock file without racing another taker: it is first renamed to a unique name (only
/// one rename of the same file can succeed), then deleted.
fn break_lock(path: &Path) -> Result<(), IoError> {
    let mut name = path.file_name().map(|n| n.to_os_string()).unwrap_or_default();
    name.push(format!(".stale-{}", Owner::this_process().token));
    let aside = path.with_file_name(name);
    match fs::rename(path, &aside) {
        Ok(()) => {
            let _ = fs::remove_file(&aside);
            Ok(())
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(e.into()),
    }
}

/// Whether a process with this id is running on this computer. When that cannot be told (no
/// right to look at it), it counts as running.
#[cfg(windows)]
pub fn process_alive(pid: u32) -> bool {
    use windows_sys::Win32::Foundation::{CloseHandle, ERROR_INVALID_PARAMETER, STILL_ACTIVE};
    use windows_sys::Win32::System::Threading::{GetExitCodeProcess, OpenProcess, PROCESS_QUERY_LIMITED_INFORMATION};
    // SAFETY: plain Win32 calls; the handle is closed before returning.
    unsafe {
        let h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
        if h.is_null() {
            return io::Error::last_os_error().raw_os_error() != Some(ERROR_INVALID_PARAMETER as i32);
        }
        let mut code = 0u32;
        let ok = GetExitCodeProcess(h, &mut code);
        CloseHandle(h);
        ok == 0 || code == STILL_ACTIVE as u32
    }
}

#[cfg(unix)]
pub fn process_alive(pid: u32) -> bool {
    let Ok(pid) = i32::try_from(pid) else { return false };
    // SAFETY: signal 0 only checks that the process exists.
    if unsafe { libc::kill(pid, 0) } == 0 {
        return true;
    }
    io::Error::last_os_error().raw_os_error() == Some(libc::EPERM)
}

#[cfg(not(any(windows, unix)))]
pub fn process_alive(_pid: u32) -> bool {
    true
}

/// Start time of a running process in ms since 1970, where the platform tells it.
#[cfg(windows)]
fn process_started_ms(pid: u32) -> Option<u64> {
    use windows_sys::Win32::Foundation::{CloseHandle, FILETIME};
    use windows_sys::Win32::System::Threading::{GetProcessTimes, OpenProcess, PROCESS_QUERY_LIMITED_INFORMATION};
    // SAFETY: plain Win32 calls; the handle is closed before returning.
    unsafe {
        let h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
        if h.is_null() {
            return None;
        }
        let zero = || FILETIME { dwLowDateTime: 0, dwHighDateTime: 0 };
        let (mut created, mut exited, mut kernel, mut user) = (zero(), zero(), zero(), zero());
        let ok = GetProcessTimes(h, &mut created, &mut exited, &mut kernel, &mut user);
        CloseHandle(h);
        if ok == 0 {
            return None;
        }
        // 100 ns ticks since 1601 -> ms since 1970.
        let ticks = (created.dwHighDateTime as u64) << 32 | created.dwLowDateTime as u64;
        (ticks / 10_000).checked_sub(11_644_473_600_000)
    }
}

#[cfg(not(windows))]
fn process_started_ms(_pid: u32) -> Option<u64> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-lock-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A process id that is not running: a child that has already exited.
    fn dead_pid() -> u32 {
        let exe = if cfg!(windows) { "cmd" } else { "true" };
        let args: &[&str] = if cfg!(windows) { &["/C", "exit 0"] } else { &[] };
        let mut child = std::process::Command::new(exe).args(args).spawn().unwrap();
        let pid = child.id();
        child.wait().unwrap();
        pid
    }

    #[test]
    fn a_live_lock_is_refused_and_released_on_drop() {
        let dir = temp("live");
        let lock = LockFile::lock_project(&dir).unwrap();
        assert!(dir.join(FOLDER_LOCK_FILE).is_file() && lock.is_held());
        assert!(lock.owner().is_this_process());
        assert_eq!(lock.owner().liveness(), Liveness::Alive);
        // This process holding it counts as live too: the project is already open here.
        match LockFile::lock_project(&dir) {
            Err(IoError::Locked { owner: Some(o), .. }) => assert_eq!(o.pid, std::process::id()),
            other => panic!("{other:?}"),
        }
        let message = LockFile::lock_project(&dir).unwrap_err().to_string();
        assert!(message.contains("already open in this window"), "{message}");
        // Another holder is named: process, computer, since when.
        let mut other = Owner::this_process();
        other.host = "SOME-OTHER-PC".into();
        let message = IoError::Locked { path: lock.path().to_path_buf(), owner: Some(Box::new(other)) }.to_string();
        assert!(message.contains(&format!("process {} on SOME-OTHER-PC, since", std::process::id())) && message.contains(&dir.display().to_string()), "{message}");
        drop(lock);
        assert!(!dir.join(FOLDER_LOCK_FILE).exists());
        let again = LockFile::lock_project(&dir.join("Part.scancraft")).unwrap();
        assert_eq!(again.path(), dir.join(FOLDER_LOCK_FILE));
        drop(again);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_stale_lock_is_taken_over() {
        let dir = temp("stale");
        let path = LockFile::path_for(&dir.join("Part.scancraftz"));
        assert_eq!(path, dir.join("Part.scancraftz.lock"));
        let mut dead = Owner::this_process();
        dead.pid = dead_pid();
        dead.process_started_ms = None;
        assert_eq!(dead.liveness(), Liveness::Dead);
        fs::write(&path, serde_json::to_vec(&dead).unwrap()).unwrap();
        let lock = LockFile::acquire(&path).unwrap();
        assert!(lock.is_held());
        drop(lock);
        // A reused process id: our own pid, but a different start time.
        let mut reused = Owner::this_process();
        if let Some(t) = reused.process_started_ms.as_mut() {
            *t -= 60_000;
            fs::write(&path, serde_json::to_vec(&reused).unwrap()).unwrap();
            assert_eq!(reused.liveness(), Liveness::Dead);
            drop(LockFile::acquire(&path).unwrap());
        }
        // Another host cannot be checked: refused, but can be broken on purpose.
        let mut remote = Owner::this_process();
        remote.host = "SOME-OTHER-PC".into();
        fs::write(&path, serde_json::to_vec(&remote).unwrap()).unwrap();
        assert!(matches!(LockFile::acquire(&path), Err(IoError::Locked { .. })));
        let taken = LockFile::take_over(&path).unwrap();
        assert!(taken.is_held());
        drop(taken);
        // A fresh unreadable lock may be one being written: refused. An old one is taken over.
        fs::write(&path, b"").unwrap();
        assert!(matches!(LockFile::acquire(&path), Err(IoError::Locked { owner: None, .. })));
        let old = std::time::SystemTime::now() - std::time::Duration::from_secs(60);
        fs::File::options().write(true).open(&path).unwrap().set_modified(old).unwrap();
        drop(LockFile::acquire(&path).unwrap());
        assert!(!path.exists());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_broken_lock_is_not_removed_by_its_old_holder() {
        let dir = temp("broken");
        let path = dir.join(FOLDER_LOCK_FILE);
        let first = LockFile::acquire(&path).unwrap();
        let second = LockFile::take_over(&path).unwrap();
        assert!(!first.is_held() && second.is_held());
        drop(first);
        assert!(path.is_file(), "the second holder's lock stays");
        drop(second);
        assert!(!path.exists());
        let _ = fs::remove_dir_all(&dir);
    }
}
