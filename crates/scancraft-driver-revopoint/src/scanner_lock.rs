//! Cross-process lock for the physical scanner. Only one process may stream from a POP 4 at a
//! time: a second client reconfigures the sensor and silently breaks the first one's stream.
//!
//! The lock is a file in the temp directory created exclusively; its content names the holder.
//! Shell scripts can take it too, e.g. bash: `set -o noclobber; echo "me" > "$TEMP/scancraft-scanner.lock"`
//! (fails if held) and `rm` it when done.

use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, SystemTime};

/// Locks older than this are considered abandoned (a crashed holder) and are taken over.
const STALE_AFTER: Duration = Duration::from_secs(20 * 60);

pub fn lock_path() -> PathBuf {
    std::env::temp_dir().join("scancraft-scanner.lock")
}

#[derive(Debug)]
pub struct ScannerLock {
    path: PathBuf,
}

impl ScannerLock {
    /// Takes the lock or reports who holds it.
    pub fn try_acquire(owner: &str) -> Result<Self, String> {
        let path = lock_path();
        for _ in 0..2 {
            match OpenOptions::new().write(true).create_new(true).open(&path) {
                Ok(mut f) => {
                    let _ = writeln!(f, "{owner} (pid {})", std::process::id());
                    return Ok(Self { path });
                }
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                    let holder = std::fs::read_to_string(&path).unwrap_or_default().trim().to_string();
                    let stale = std::fs::metadata(&path)
                        .and_then(|m| m.modified())
                        .ok()
                        .and_then(|t| SystemTime::now().duration_since(t).ok())
                        .is_some_and(|age| age > STALE_AFTER);
                    if stale {
                        log::warn!("taking over stale scanner lock held by {holder}");
                        let _ = std::fs::remove_file(&path);
                        continue;
                    }
                    return Err(format!("the scanner is in use by {holder} ({})", path.display()));
                }
                Err(e) => return Err(format!("cannot create {}: {e}", path.display())),
            }
        }
        Err("could not take the scanner lock".into())
    }

    /// Waits up to `timeout` for the lock.
    pub fn acquire(owner: &str, timeout: Duration) -> Result<Self, String> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            match Self::try_acquire(owner) {
                Ok(lock) => return Ok(lock),
                Err(e) if std::time::Instant::now() >= deadline => return Err(e),
                Err(_) => std::thread::sleep(Duration::from_millis(500)),
            }
        }
    }

}

/// Marks the lock as still in use so long sessions are not taken for abandoned ones. Rewriting
/// the content updates the modification time without extra dependencies.
pub fn touch() {
    let path = lock_path();
    if let Ok(content) = std::fs::read(&path) {
        let _ = std::fs::write(&path, content);
    }
}

impl Drop for ScannerLock {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn second_holder_is_refused_until_release() {
        // Use a private path so a real lock on this machine is not disturbed.
        let path = std::env::temp_dir().join(format!("scancraft-lock-test-{}", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let first = OpenOptions::new().write(true).create_new(true).open(&path);
        assert!(first.is_ok());
        assert!(OpenOptions::new().write(true).create_new(true).open(&path).is_err());
        std::fs::remove_file(&path).unwrap();
        assert!(OpenOptions::new().write(true).create_new(true).open(&path).is_ok());
        std::fs::remove_file(&path).unwrap();
    }
}
