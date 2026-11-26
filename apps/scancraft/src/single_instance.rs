//! One ScanCraft per user: a second start (a double-click on a project while ScanCraft runs)
//! hands its project to the running app, which opens it as a tab, and ends. Two apps could
//! otherwise both talk to the scanner, which breaks the one-client rule (`docs/HANDOFF.md`
//! section 3; the driver's scanner lock still stops a second *stream*, see
//! `scancraft-driver-revopoint/src/scanner_lock.rs`).
//!
//! ```text
//! <local app data>/ScanCraft/instance.lock   held by the running app (scancraft_io::lock)
//! <local app data>/ScanCraft/instance.json   {pid, port, token}: where it listens
//! ```
//!
//! The running app listens on a loopback TCP port (127.0.0.1 only) chosen by the system. A
//! request is one JSON line `{"token": …, "open": "<path>" | null}` and gets one line back
//! (`ok`, or `refused: <why>`). The token is random per launch and only in `instance.json`, which
//! lies in the user's own local app data (on Windows only the user, SYSTEM and administrators may
//! read it; on Unix the file is made 0600): a request without it is refused. The path must be
//! absolute and name an existing project file (`.scancraft`), bundle (`.scancraftz`) or project
//! folder; nothing else is accepted. `open: null` only brings the window to the front.

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use scancraft_io::lock::LockFile;
use serde::{Deserialize, Serialize};

const LOCK_FILE: &str = "instance.lock";
const INFO_FILE: &str = "instance.json";
/// A request line longer than this is refused.
const MAX_REQUEST: u64 = 64 * 1024;
const IO_TIMEOUT: Duration = Duration::from_secs(3);

/// What a second start asks of the running app.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Handover {
    /// Open this project in a tab (checked: see [`validate_path`]).
    Open(PathBuf),
    /// Only come to the front.
    Focus,
}

#[derive(Debug, Serialize, Deserialize)]
struct Info {
    pid: u32,
    port: u16,
    token: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Request {
    token: String,
    #[serde(default)]
    open: Option<String>,
}

/// How this start goes on.
pub enum Start {
    /// The only ScanCraft: it listens for later starts.
    Primary(Primary),
    /// A running ScanCraft took the request: this start ends.
    HandedOver,
    /// Neither (no running app could be reached, or the files could not be written): run on
    /// alone, without listening.
    Alone(String),
}

/// The listening side, kept by the running app.
pub struct Primary {
    _lock: LockFile,
    info_path: PathBuf,
    token: String,
    /// Requests from later starts.
    pub requests: mpsc::Receiver<Handover>,
    wake: Arc<Mutex<Option<eframe::egui::Context>>>,
}

impl Primary {
    /// The receiver and the slot for the window's context (set once it exists, so a request
    /// wakes the otherwise idle window); the guard stays with `main`.
    pub fn split(self) -> (Guard, mpsc::Receiver<Handover>, Arc<Mutex<Option<eframe::egui::Context>>>) {
        let Primary { _lock, info_path, token, requests, wake } = self;
        (Guard { _lock, info_path, token }, requests, wake)
    }
}

/// Removes `instance.json` (when it is still ours) and the lock when the app ends.
pub struct Guard {
    _lock: LockFile,
    info_path: PathBuf,
    token: String,
}

impl Drop for Guard {
    fn drop(&mut self) {
        let ours = std::fs::read(&self.info_path).ok().and_then(|b| serde_json::from_slice::<Info>(&b).ok()).is_some_and(|i| i.token == self.token);
        if ours {
            let _ = std::fs::remove_file(&self.info_path);
        }
    }
}

/// `<local app data>/ScanCraft`.
pub fn default_dir() -> PathBuf {
    scancraft_io::fsutil::local_data_dir().join("ScanCraft")
}

/// Becomes the running ScanCraft, or hands `open` (or a focus request) to the one that runs.
pub fn start(dir: &Path, open: Option<&Path>) -> Start {
    if let Err(e) = std::fs::create_dir_all(dir) {
        return Start::Alone(format!("cannot create {}: {e}", dir.display()));
    }
    match LockFile::acquire(&dir.join(LOCK_FILE)) {
        Ok(lock) => match listen(dir, lock) {
            Ok(primary) => Start::Primary(primary),
            Err(e) => Start::Alone(e),
        },
        Err(scancraft_io::IoError::Locked { .. }) => {
            // Another ScanCraft runs; it may still be writing where it listens.
            let mut last = String::new();
            for _ in 0..20 {
                match hand_over(dir, open) {
                    Ok(()) => return Start::HandedOver,
                    Err(e) => last = e,
                }
                std::thread::sleep(Duration::from_millis(150));
            }
            Start::Alone(format!("the running ScanCraft did not answer ({last})"))
        }
        Err(e) => Start::Alone(e.to_string()),
    }
}

fn listen(dir: &Path, lock: LockFile) -> Result<Primary, String> {
    let listener = TcpListener::bind(SocketAddr::from((Ipv4Addr::LOCALHOST, 0))).map_err(|e| format!("cannot listen on the loopback address: {e}"))?;
    let port = listener.local_addr().map_err(|e| e.to_string())?.port();
    let token = random_token();
    let info = Info { pid: std::process::id(), port, token: token.clone() };
    let info_path = dir.join(INFO_FILE);
    write_private(&info_path, &serde_json::to_vec_pretty(&info).map_err(|e| e.to_string())?).map_err(|e| format!("cannot write {}: {e}", info_path.display()))?;
    let (tx, requests) = mpsc::channel();
    let wake: Arc<Mutex<Option<eframe::egui::Context>>> = Arc::default();
    let (serve_token, serve_wake) = (token.clone(), wake.clone());
    std::thread::Builder::new()
        .name("single-instance".into())
        .spawn(move || serve(listener, &serve_token, &tx, &serve_wake))
        .map_err(|e| e.to_string())?;
    Ok(Primary { _lock: lock, info_path, token, requests, wake })
}

/// Answers requests until the app ends.
fn serve(listener: TcpListener, token: &str, tx: &mpsc::Sender<Handover>, wake: &Mutex<Option<eframe::egui::Context>>) {
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let answer = match read_request(&stream).and_then(|r| check(&r, token)) {
            Ok(h) => {
                let _ = tx.send(h);
                if let Some(ctx) = wake.lock().ok().and_then(|w| w.clone()) {
                    ctx.request_repaint();
                }
                "ok".to_string()
            }
            Err(why) => {
                log::warn!("refused a request from another start: {why}");
                format!("refused: {why}")
            }
        };
        let _ = writeln!(stream, "{answer}");
    }
}

fn read_request(stream: &TcpStream) -> Result<Request, String> {
    let _ = stream.set_read_timeout(Some(IO_TIMEOUT));
    let _ = stream.set_write_timeout(Some(IO_TIMEOUT));
    let mut line = String::new();
    BufReader::new(stream.take(MAX_REQUEST)).read_line(&mut line).map_err(|e| e.to_string())?;
    if !line.ends_with('\n') {
        return Err("incomplete or too long request".into());
    }
    serde_json::from_str(line.trim()).map_err(|_| "not a request".to_string())
}

/// The token check, then the path check.
fn check(request: &Request, token: &str) -> Result<Handover, String> {
    if !same_token(&request.token, token) {
        return Err("wrong token".into());
    }
    match &request.open {
        None => Ok(Handover::Focus),
        Some(p) => validate_path(p).map(Handover::Open),
    }
}

/// Compares in time independent of where the tokens differ.
fn same_token(a: &str, b: &str) -> bool {
    a.len() == b.len() && a.bytes().zip(b.bytes()).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

/// Accepts only an absolute path of an existing project file (`.scancraft`), bundle
/// (`.scancraftz`) or project folder.
pub fn validate_path(path: &str) -> Result<PathBuf, String> {
    use scancraft_io::project::{has_extension, is_bundle, is_project, BUNDLE_EXTENSION, PROJECT_EXTENSION};
    let p = PathBuf::from(path);
    if path.is_empty() || !p.is_absolute() {
        return Err("not an absolute path".into());
    }
    let ok = if p.is_dir() {
        is_project(&p)
    } else if p.is_file() {
        (has_extension(&p, PROJECT_EXTENSION) || has_extension(&p, BUNDLE_EXTENSION)) && (is_bundle(&p) || is_project(&p))
    } else {
        return Err("no such file or folder".into());
    };
    if ok {
        Ok(p)
    } else {
        Err("not a ScanCraft project file, bundle or project folder".into())
    }
}

/// Sends `open` (or a focus request) to the running app.
fn hand_over(dir: &Path, open: Option<&Path>) -> Result<(), String> {
    let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let request = Request { token: info.token, open: open.map(|p| p.display().to_string()) };
    send(info.port, &serde_json::to_string(&request).map_err(|e| e.to_string())?)
}

/// Sends one request line to `port` and reads the answer.
fn send(port: u16, line: &str) -> Result<(), String> {
    let mut stream = TcpStream::connect_timeout(&SocketAddr::from((Ipv4Addr::LOCALHOST, port)), IO_TIMEOUT).map_err(|e| e.to_string())?;
    let _ = stream.set_read_timeout(Some(IO_TIMEOUT));
    writeln!(stream, "{line}").map_err(|e| e.to_string())?;
    let mut answer = String::new();
    BufReader::new(stream).read_line(&mut answer).map_err(|e| e.to_string())?;
    match answer.trim() {
        "ok" => Ok(()),
        other => Err(other.to_string()),
    }
}

/// 256 random bits as hex. `RandomState` keys come from the operating system's random source
/// (once per thread, then counted on); each key hashes a counter with SipHash, so the result is
/// unpredictable to other processes.
fn random_token() -> String {
    use std::hash::{BuildHasher, Hasher};
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
    (0..4u64)
        .map(|i| {
            let mut h = std::collections::hash_map::RandomState::new().build_hasher();
            h.write_u64(i);
            h.write_u128(now);
            h.write_u32(std::process::id());
            format!("{:016x}", h.finish())
        })
        .collect()
}

/// Writes a file only this user can read (Unix: 0600; Windows: the user's local app data is
/// private already), atomically.
fn write_private(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let tmp = path.with_extension("json.tmp");
    {
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create(true).truncate(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let mut f = options.open(&tmp)?;
        f.write_all(bytes)?;
        f.sync_all()?;
    }
    scancraft_io::fsutil::rename_replace(&tmp, path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-instance-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn recv(rx: &mpsc::Receiver<Handover>) -> Handover {
        rx.recv_timeout(Duration::from_secs(5)).expect("the running app got the request")
    }

    #[test]
    fn a_second_start_hands_its_project_over() {
        let dir = temp("handover");
        let project = dir.join("Part");
        std::fs::create_dir_all(&project).unwrap();
        std::fs::write(project.join("Part.scancraft"), b"{}").unwrap();
        let Start::Primary(primary) = start(&dir, None) else { panic!("the first start runs") };
        // A second start with a project, and one without.
        assert!(matches!(start(&dir, Some(&project.join("Part.scancraft"))), Start::HandedOver));
        assert_eq!(recv(&primary.requests), Handover::Open(project.join("Part.scancraft")));
        assert!(matches!(start(&dir, None), Start::HandedOver));
        assert_eq!(recv(&primary.requests), Handover::Focus);
        // A project that does not exist is refused by the running app: the start runs alone.
        let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).unwrap()).unwrap();
        let missing = Request { token: info.token.clone(), open: Some(dir.join("gone.scancraft").display().to_string()) };
        assert!(send(info.port, &serde_json::to_string(&missing).unwrap()).unwrap_err().contains("no such file"));
        // Without the token nothing is accepted.
        let forged = Request { token: "0".repeat(64), open: Some(project.display().to_string()) };
        assert!(send(info.port, &serde_json::to_string(&forged).unwrap()).unwrap_err().contains("wrong token"));
        assert!(send(info.port, "not json").unwrap_err().contains("not a request"));
        assert!(primary.requests.try_recv().is_err(), "refused requests never reach the app");
        // Ending the app removes its files; the next start is the running one again.
        let (guard, _rx, _wake) = primary.split();
        drop(guard);
        assert!(!dir.join(INFO_FILE).exists() && !dir.join(LOCK_FILE).exists());
        assert!(matches!(start(&dir, None), Start::Primary(_)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn only_absolute_existing_projects_are_accepted() {
        let dir = temp("validate");
        let folder = dir.join("Part");
        std::fs::create_dir_all(&folder).unwrap();
        assert!(validate_path(&folder.display().to_string()).is_err(), "a folder without a project");
        std::fs::write(folder.join("Part.scancraft"), b"{}").unwrap();
        std::fs::write(dir.join("Part.scancraftz"), b"zip").unwrap();
        std::fs::write(dir.join("notes.txt"), b"text").unwrap();
        assert_eq!(validate_path(&folder.display().to_string()), Ok(folder.clone()));
        assert!(validate_path(&folder.join("Part.scancraft").display().to_string()).is_ok());
        assert!(validate_path(&dir.join("Part.scancraftz").display().to_string()).is_ok());
        assert!(validate_path(&dir.join("notes.txt").display().to_string()).is_err());
        assert!(validate_path("Part.scancraft").unwrap_err().contains("absolute"));
        assert!(validate_path("").is_err());
        assert!(validate_path(&dir.join("missing.scancraftz").display().to_string()).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn tokens_are_long_random_and_compared_whole() {
        let (a, b) = (random_token(), random_token());
        assert_eq!(a.len(), 64);
        assert_ne!(a, b);
        assert!(same_token(&a, &a.clone()));
        assert!(!same_token(&a, &b) && !same_token(&a, &a[..63]));
    }
}
