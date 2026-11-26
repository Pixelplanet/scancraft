//! File-system helpers shared by project files, bundles, working folders and locks: atomic
//! replace, free space, a sorted walk of a folder, time stamps and the host name.

use std::fs::{self, File, Metadata};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::IoError;

/// Writes `bytes` to `<path>.tmp`, flushes it to disk and renames it over `path`. An interrupted
/// or failed write leaves the previous `path` untouched (at worst a stray `.tmp` beside it).
pub fn atomic_write(path: &Path, bytes: &[u8]) -> Result<(), IoError> {
    atomic_write_with(path, |w| w.write_all(bytes).map_err(IoError::from))
}

/// [`atomic_write`] for content produced by a writer function.
pub fn atomic_write_with(path: &Path, write: impl FnOnce(&mut BufWriter<File>) -> Result<(), IoError>) -> Result<(), IoError> {
    let tmp = tmp_path(path);
    let result = (|| {
        let mut w = BufWriter::with_capacity(1 << 20, File::create(&tmp)?);
        write(&mut w)?;
        let f = w.into_inner().map_err(|e| e.into_error())?;
        f.sync_all()?;
        drop(f);
        rename_replace(&tmp, path).map_err(IoError::from)
    })();
    if result.is_err() {
        let _ = fs::remove_file(&tmp);
    }
    result
}

/// `<path>.tmp`.
pub fn tmp_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().map(|n| n.to_os_string()).unwrap_or_default();
    name.push(".tmp");
    path.with_file_name(name)
}

/// `fs::rename` that replaces `to`, retried for a moment on Windows when another program (a virus
/// scanner, the Explorer preview, a sync client) briefly holds the target open.
pub fn rename_replace(from: &Path, to: &Path) -> io::Result<()> {
    let mut wait = Duration::from_millis(20);
    for attempt in 0.. {
        match fs::rename(from, to) {
            Ok(()) => return Ok(()),
            Err(e) if attempt < 8 && cfg!(windows) && e.kind() == io::ErrorKind::PermissionDenied => {
                std::thread::sleep(wait);
                wait = (wait * 2).min(Duration::from_millis(500));
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}

/// Bytes free for this user on the volume holding `path` (or its nearest existing ancestor).
pub fn free_space(path: &Path) -> io::Result<u64> {
    let mut p = path.to_path_buf();
    while !p.exists() {
        if !p.pop() {
            return Err(io::Error::new(io::ErrorKind::NotFound, format!("{} does not exist", path.display())));
        }
    }
    free_space_of(&p)
}

#[cfg(windows)]
fn free_space_of(path: &Path) -> io::Result<u64> {
    use std::os::windows::ffi::OsStrExt;
    use windows_sys::Win32::Storage::FileSystem::GetDiskFreeSpaceExW;
    let wide: Vec<u16> = path.as_os_str().encode_wide().chain(std::iter::once(0)).collect();
    let mut free = 0u64;
    // SAFETY: `wide` is NUL-terminated and outlives the call; the out-pointers are valid or null.
    let ok = unsafe { GetDiskFreeSpaceExW(wide.as_ptr(), &mut free, std::ptr::null_mut(), std::ptr::null_mut()) };
    if ok == 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(free)
}

#[cfg(unix)]
fn free_space_of(path: &Path) -> io::Result<u64> {
    use std::os::unix::ffi::OsStrExt;
    let c = std::ffi::CString::new(path.as_os_str().as_bytes()).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    // SAFETY: `c` is NUL-terminated; `stat` is plain data written by the call.
    let mut stat: libc::statvfs = unsafe { std::mem::zeroed() };
    if unsafe { libc::statvfs(c.as_ptr(), &mut stat) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(stat.f_bavail as u64 * stat.f_frsize as u64)
}

#[cfg(not(any(windows, unix)))]
fn free_space_of(_path: &Path) -> io::Result<u64> {
    Ok(u64::MAX)
}

/// Head room kept free on top of what a job needs.
pub const SPACE_MARGIN: u64 = 64 << 20;

/// `Err(IoError::NoSpace)` when the volume holding `dir` has less than `needed` (+ a margin) free.
pub fn check_free_space(dir: &Path, needed: u64) -> Result<(), IoError> {
    let free = free_space(dir)?;
    let needed = needed.saturating_add(SPACE_MARGIN);
    if free < needed {
        return Err(IoError::NoSpace { path: dir.to_path_buf(), needed, free });
    }
    Ok(())
}

/// One file found by [`walk_files`].
#[derive(Debug, Clone)]
pub struct WalkedFile {
    /// Path relative to the walked folder, `/`-separated.
    pub rel: String,
    pub path: PathBuf,
    pub size: u64,
    pub mtime_ns: u64,
}

/// Every regular file below `root`, sorted by relative path. Symbolic links are not followed.
pub fn walk_files(root: &Path) -> io::Result<Vec<WalkedFile>> {
    let mut out = Vec::new();
    let mut stack = vec![(root.to_path_buf(), String::new())];
    while let Some((dir, prefix)) = stack.pop() {
        for entry in fs::read_dir(&dir)? {
            let entry = entry?;
            let name = entry.file_name().to_string_lossy().into_owned();
            let rel = if prefix.is_empty() { name } else { format!("{prefix}/{name}") };
            let kind = entry.file_type()?;
            if kind.is_dir() {
                stack.push((entry.path(), rel));
            } else if kind.is_file() {
                let meta = entry.metadata()?;
                out.push(WalkedFile { rel, path: entry.path(), size: meta.len(), mtime_ns: mtime_ns(&meta) });
            }
        }
    }
    out.sort_by(|a, b| a.rel.cmp(&b.rel));
    Ok(out)
}

/// Modification time in nanoseconds since 1970 (0 when the platform has none).
pub fn mtime_ns(meta: &Metadata) -> u64 {
    meta.modified().ok().map(system_ns).unwrap_or(0)
}

pub fn system_ns(t: SystemTime) -> u64 {
    t.duration_since(UNIX_EPOCH).map(|d| d.as_nanos().min(u64::MAX as u128) as u64).unwrap_or(0)
}

/// Seconds since 1970.
pub fn unix_now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// The user's local (not roaming) application data folder: `%LOCALAPPDATA%` on Windows,
/// `$XDG_DATA_HOME` or `~/.local/share` elsewhere, the temporary folder as a last resort.
pub fn local_data_dir() -> PathBuf {
    let env = |k: &str| std::env::var_os(k).filter(|v| !v.is_empty()).map(PathBuf::from);
    if cfg!(windows) {
        if let Some(p) = env("LOCALAPPDATA") {
            return p;
        }
    }
    env("XDG_DATA_HOME").or_else(|| env("HOME").map(|h| h.join(".local").join("share"))).unwrap_or_else(std::env::temp_dir)
}

/// This computer's name, as other ScanCraft instances (on a shared folder) see it in a lock.
pub fn host_name() -> String {
    let from_env = std::env::var("COMPUTERNAME").or_else(|_| std::env::var("HOSTNAME")).ok();
    let from_file = || fs::read_to_string("/etc/hostname").ok().map(|s| s.trim().to_string());
    from_env.or_else(from_file).filter(|s| !s.is_empty()).unwrap_or_else(|| "unknown".into())
}

/// Calendar date (UTC) of a time in seconds since 1970: (year, month, day, hour, minute, second).
pub fn civil_utc(secs: u64) -> (i64, u32, u32, u32, u32, u32) {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // Howard Hinnant's days_from_civil inverse.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day, (rem / 3600) as u32, (rem / 60 % 60) as u32, (rem % 60) as u32)
}

/// "2026-11-26 10:15 UTC".
pub fn format_utc(secs: u64) -> String {
    let (y, mo, d, h, mi, _) = civil_utc(secs);
    format!("{y:04}-{mo:02}-{d:02} {h:02}:{mi:02} UTC")
}

/// "1.4 GB", "820 MB", "12 kB".
pub fn format_bytes(n: u64) -> String {
    let n = n as f64;
    if n >= 1e9 {
        format!("{:.1} GB", n / 1e9)
    } else if n >= 1e6 {
        format!("{:.0} MB", n / 1e6)
    } else {
        format!("{:.0} kB", (n / 1e3).ceil())
    }
}

/// Whether `rel` (a stored, `/`-separated path) stays inside the folder it is relative to: not
/// empty, not absolute, no drive letter, no `..` or `.` parts, no backslash or NUL.
pub fn is_safe_relative(rel: &str) -> bool {
    !rel.is_empty()
        && !rel.starts_with('/')
        && !rel.contains(['\\', ':', '\0'])
        && rel.split('/').all(|part| !part.is_empty() && part != "." && part != "..")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calendar_and_paths() {
        assert_eq!(civil_utc(0), (1970, 1, 1, 0, 0, 0));
        assert_eq!(civil_utc(1_795_679_696), (2026, 11, 26, 7, 54, 56));
        assert_eq!(civil_utc(951_782_400), (2000, 2, 29, 0, 0, 0));
        assert!(is_safe_relative("pass-1/keyframes.bin"));
        for bad in ["", "/etc/passwd", "../x", "a/../b", "a//b", "C:/x", "a\\b", "./a", "a/"] {
            assert!(!is_safe_relative(bad), "{bad}");
        }
        assert_eq!(format_bytes(1_400_000_000), "1.4 GB");
        assert!(free_space(&std::env::temp_dir().join("does/not/exist")).unwrap() > 0);
    }
}
