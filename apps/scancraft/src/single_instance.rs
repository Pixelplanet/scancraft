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
//! request is one JSON line `{"token": …, "open": "<path>" | null, "ignored": [options]}`, sent
//! within 3 s, and gets
//! one line back: `ok`, `refused: <why>`, or `closing` (the app is ending: the second start
//! waits for its lock and then runs as the new one). A refused start never runs a second app; the
//! running one shows why the project could not be opened. Each request is answered on its own
//! thread (a few at once), so a slow client cannot hold up the others. The token is random per
//! launch and only in `instance.json`, which
//! lies in the user's own local app data (on Windows only the user, SYSTEM and administrators may
//! read it; on Unix the file is made 0600): a request without it is refused. The path must be
//! absolute and name an existing project file (`.scancraft`), bundle (`.scancraftz`) or project
//! folder; nothing else is accepted. `open: null` only brings the window to the front.
//!
//! Only the project is handed over. The second start's options that set up a new app
//! (`--mock`, `--connect`, …: `Cli::new_app_options`) are not applied to the running app, which
//! may be connected or capturing already; they travel as `ignored` and the running app names them
//! in its status bar, with the result of opening the project when one was given
//! ([`Handover::Ignored`]), since the second start has no window to say so.
//! An older running app that does not know the field simply does not show them.

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use scancraft_io::lock::LockFile;
use serde::{Deserialize, Serialize};

const LOCK_FILE: &str = "instance.lock";
const INFO_FILE: &str = "instance.json";
/// A request line longer than this is refused.
const MAX_REQUEST: u64 = 64 * 1024;
const IO_TIMEOUT: Duration = Duration::from_secs(3);
/// How long a second start waits for the running app to answer or to end.
const HANDOVER_WAIT: Duration = Duration::from_secs(15);
/// Requests answered at the same time; more are dropped (their start tries again).
const MAX_CLIENTS: usize = 8;

/// What a second start asks of the running app.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Handover {
    /// Open this project in a tab (checked: see [`validate_path`]).
    Open(PathBuf),
    /// Only come to the front.
    Focus,
    /// A start with the right token asked to open something that is not a project: come to the
    /// front and say why (`path` as given).
    Refused { path: String, why: String },
    /// The second start's options that were not applied (see the module documentation), at
    /// most [`MAX_IGNORED`], each shortened to [`MAX_IGNORED_LEN`] characters. `project`: an
    /// [`Handover::Open`] of its project follows (the app names the options with the result of
    /// that open, which would otherwise replace them at once).
    Ignored { options: Vec<String>, project: bool },
}

/// Ignored options passed on to the running app (more are dropped).
const MAX_IGNORED: usize = 8;
const MAX_IGNORED_LEN: usize = 200;

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
    /// Options of the second start that the running app does not apply (it names them).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    ignored: Vec<String>,
}

/// How this start goes on.
pub enum Start {
    /// The only ScanCraft: it listens for later starts.
    Primary(Box<Primary>),
    /// A running ScanCraft took the request: this start ends.
    HandedOver,
    /// The running ScanCraft refused the request (it shows why): this start ends too, never a
    /// second app.
    Refused(String),
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
    closing: Arc<AtomicBool>,
}

impl Primary {
    /// The receiver and the slot for the window's context (set once it exists, so a request
    /// wakes the otherwise idle window); the guard stays with `main`.
    pub fn split(self: Box<Self>) -> (Guard, mpsc::Receiver<Handover>, Arc<Mutex<Option<eframe::egui::Context>>>) {
        let Primary { _lock, info_path, token, requests, wake, closing } = *self;
        (Guard { _lock, info_path, token, closing }, requests, wake)
    }
}

/// Removes `instance.json` (when it is still ours) and the lock when the app ends.
pub struct Guard {
    _lock: LockFile,
    info_path: PathBuf,
    token: String,
    closing: Arc<AtomicBool>,
}

impl Guard {
    /// The app is ending (it reads no more requests): later starts are told so, and wait for the
    /// lock instead of handing over.
    pub fn closing_flag(&self) -> Arc<AtomicBool> {
        self.closing.clone()
    }
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

/// Becomes the running ScanCraft, or hands `open` (or a focus request) to the one that runs,
/// with the options this start sets up a new app with (`ignored`: the running app does not apply
/// them, it names them).
pub fn start(dir: &Path, open: Option<&Path>, ignored: &[String]) -> Start {
    if let Err(e) = std::fs::create_dir_all(dir) {
        return Start::Alone(format!("cannot create {}: {e}", dir.display()));
    }
    start_waiting(dir, open, ignored, HANDOVER_WAIT)
}

/// [`start`], waiting up to `wait` for a running app that cannot be reached or is ending.
fn start_waiting(dir: &Path, open: Option<&Path>, ignored: &[String], wait: Duration) -> Start {
    let deadline = Instant::now() + wait;
    loop {
        match LockFile::acquire(&dir.join(LOCK_FILE)) {
            Ok(lock) => {
                return match listen(dir, lock) {
                    Ok(primary) => Start::Primary(Box::new(primary)),
                    Err(e) => Start::Alone(e),
                }
            }
            Err(scancraft_io::IoError::Locked { .. }) => {}
            Err(e) => return Start::Alone(e.to_string()),
        }
        // Another ScanCraft runs; it may still be writing where it listens, or be ending (its
        // lock is free then, and this start becomes the running one).
        let last = match hand_over(dir, open, ignored) {
            Ok(()) => return Start::HandedOver,
            Err(Refusal::Refused(why)) => return Start::Refused(why),
            Err(Refusal::Retry(e)) => e,
        };
        if Instant::now() >= deadline {
            return Start::Alone(format!("the running ScanCraft did not answer ({last})"));
        }
        std::thread::sleep(Duration::from_millis(150));
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
    let closing = Arc::new(AtomicBool::new(false));
    let server = Server { token: token.clone(), tx, wake: wake.clone(), closing: closing.clone(), clients: Arc::default() };
    std::thread::Builder::new().name("single-instance".into()).spawn(move || serve(listener, server)).map_err(|e| e.to_string())?;
    Ok(Primary { _lock: lock, info_path, token, requests, wake, closing })
}

/// What the listening thread shares with the threads that answer requests.
#[derive(Clone)]
struct Server {
    token: String,
    tx: mpsc::Sender<Handover>,
    wake: Arc<Mutex<Option<eframe::egui::Context>>>,
    closing: Arc<AtomicBool>,
    /// Requests being answered now.
    clients: Arc<AtomicUsize>,
}

/// Accepts requests until the app ends, each answered on its own thread.
fn serve(listener: TcpListener, server: Server) {
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        if server.clients.fetch_add(1, Ordering::SeqCst) >= MAX_CLIENTS {
            server.clients.fetch_sub(1, Ordering::SeqCst);
            continue;
        }
        let s = server.clone();
        let spawned = std::thread::Builder::new().name("single-instance-request".into()).spawn(move || {
            answer(stream, &s);
            s.clients.fetch_sub(1, Ordering::SeqCst);
        });
        if spawned.is_err() {
            server.clients.fetch_sub(1, Ordering::SeqCst);
        }
    }
}

/// Reads one request and answers it.
fn answer(mut stream: TcpStream, server: &Server) {
    let answer = match read_request(&stream) {
        Err(why) => format!("refused: {why}"),
        Ok(r) if !same_token(&r.token, &server.token) => "refused: wrong token".to_string(),
        // The app reads no more requests.
        Ok(_) if server.closing.load(Ordering::SeqCst) => "closing".to_string(),
        Ok(r) => {
            let (handover, answer) = match check_path(&r) {
                Ok(h) => (h, "ok".to_string()),
                Err(why) => (Handover::Refused { path: r.open.clone().unwrap_or_default(), why: why.clone() }, format!("refused: {why}")),
            };
            // The options not applied are named first, then the project opens.
            let ignored: Vec<String> = r.ignored.iter().take(MAX_IGNORED).map(|o| o.chars().filter(|c| !c.is_control()).take(MAX_IGNORED_LEN).collect()).collect();
            if !ignored.is_empty() && answer == "ok" {
                let _ = server.tx.send(Handover::Ignored { options: ignored, project: matches!(handover, Handover::Open(_)) });
            }
            if server.tx.send(handover).is_ok() {
                if let Some(ctx) = server.wake.lock().ok().and_then(|w| w.clone()) {
                    ctx.request_repaint();
                }
                answer
            } else {
                "closing".to_string()
            }
        }
    };
    if answer != "ok" {
        log::warn!("a request from another start: {answer}");
    }
    let _ = writeln!(stream, "{answer}");
}

/// Reads one request line, which must arrive whole within [`IO_TIMEOUT`] (a client sending a
/// byte now and then is cut off too).
fn read_request(stream: &TcpStream) -> Result<Request, String> {
    let deadline = Instant::now() + IO_TIMEOUT;
    let _ = stream.set_write_timeout(Some(IO_TIMEOUT));
    let mut line = Vec::new();
    let mut buf = [0u8; 4096];
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Err("the request took too long".into());
        }
        stream.set_read_timeout(Some(left)).map_err(|e| e.to_string())?;
        let n = (&mut &*stream).read(&mut buf).map_err(|e| match e.kind() {
            std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock => "the request took too long".to_string(),
            _ => e.to_string(),
        })?;
        if n == 0 {
            return Err("incomplete request".into());
        }
        line.extend_from_slice(&buf[..n]);
        if let Some(end) = line.iter().position(|&b| b == b'\n') {
            line.truncate(end);
            break;
        }
        if line.len() as u64 > MAX_REQUEST {
            return Err("too long request".into());
        }
    }
    let line = String::from_utf8(line).map_err(|_| "not a request".to_string())?;
    serde_json::from_str(line.trim()).map_err(|_| "not a request".to_string())
}

/// What the running app says about a second start's options it did not apply; `project`:
/// whether that start handed over a project (it opens here).
pub fn ignored_message(options: &[String], project: bool) -> String {
    let options = options.join(" ");
    if project {
        format!("ScanCraft was already running: the project of the second start opens here, its options {options} were not used. Connect a scanner from the Connect stage.")
    } else {
        format!("ScanCraft was already running: the options {options} of the second start were not used. Connect a scanner from the Connect stage.")
    }
}

/// The path check (after the token check).
fn check_path(request: &Request) -> Result<Handover, String> {
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

/// Why a handover did not happen.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Refusal {
    /// The running app answered `refused: <why>`: asking again changes nothing.
    Refused(String),
    /// It could not be reached, or it is ending: try again (or take its lock once free).
    Retry(String),
}

/// Sends `open` (or a focus request) to the running app.
fn hand_over(dir: &Path, open: Option<&Path>, ignored: &[String]) -> Result<(), Refusal> {
    let retry = |e: String| Refusal::Retry(e);
    let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).map_err(|e| retry(e.to_string()))?).map_err(|e| retry(e.to_string()))?;
    let request = Request { token: info.token, open: open.map(|p| p.display().to_string()), ignored: ignored.to_vec() };
    send(info.port, &serde_json::to_string(&request).map_err(|e| retry(e.to_string()))?)
}

/// Sends one request line to `port` and reads the answer.
fn send(port: u16, line: &str) -> Result<(), Refusal> {
    let retry = |e: std::io::Error| Refusal::Retry(e.to_string());
    let mut stream = TcpStream::connect_timeout(&SocketAddr::from((Ipv4Addr::LOCALHOST, port)), IO_TIMEOUT).map_err(retry)?;
    let _ = stream.set_read_timeout(Some(IO_TIMEOUT));
    writeln!(stream, "{line}").map_err(retry)?;
    let mut answer = String::new();
    BufReader::new(stream).read_line(&mut answer).map_err(retry)?;
    match answer.trim() {
        "ok" => Ok(()),
        a => match a.strip_prefix("refused: ") {
            Some(why) => Err(Refusal::Refused(why.to_string())),
            None => Err(Refusal::Retry(if a.is_empty() { "no answer".into() } else { a.to_string() })),
        },
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
        let Start::Primary(primary) = start(&dir, None, &[]) else { panic!("the first start runs") };
        // A second start with a project, and one without.
        assert!(matches!(start(&dir, Some(&project.join("Part.scancraft")), &[]), Start::HandedOver));
        assert_eq!(recv(&primary.requests), Handover::Open(project.join("Part.scancraft")));
        assert!(matches!(start(&dir, None, &[]), Start::HandedOver));
        assert_eq!(recv(&primary.requests), Handover::Focus);
        // A second start's connection options are not applied, but named to the running app
        // before its project opens.
        let options = vec!["--mock".to_string(), "--connect 10.0.0.7".to_string()];
        assert!(matches!(start(&dir, Some(&project), &options), Start::HandedOver));
        assert_eq!(recv(&primary.requests), Handover::Ignored { options: options.clone(), project: true });
        assert_eq!(recv(&primary.requests), Handover::Open(project.clone()));
        assert!(matches!(start(&dir, None, &options), Start::HandedOver));
        assert_eq!(recv(&primary.requests), Handover::Ignored { options: options.clone(), project: false });
        assert_eq!(recv(&primary.requests), Handover::Focus);
        assert!(ignored_message(&options, true).contains("the project of the second start opens here"));
        assert!(!ignored_message(&options, false).contains("project"), "{}", ignored_message(&options, false));
        let junk = vec!["x\u{1b}[2J".repeat(100); 20];
        assert!(matches!(start(&dir, None, &junk), Start::HandedOver));
        match recv(&primary.requests) {
            Handover::Ignored { options: o, project: false } => assert!(o.len() == MAX_IGNORED && o.iter().all(|s| s.chars().count() <= MAX_IGNORED_LEN && !s.contains('\u{1b}'))),
            other => panic!("{other:?}"),
        }
        assert_eq!(recv(&primary.requests), Handover::Focus);
        // A request from an older start (no "ignored" field) still works.
        let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).unwrap()).unwrap();
        assert_eq!(send(info.port, &format!("{{\"token\": \"{}\", \"open\": null}}", info.token)), Ok(()));
        assert_eq!(recv(&primary.requests), Handover::Focus);
        // A project that does not exist is refused by the running app, which says why; the
        // start ends at once and never runs a second app.
        let gone = dir.join("gone.scancraft");
        let t0 = Instant::now();
        assert!(matches!(start(&dir, Some(&gone), &["--mock".to_string()]), Start::Refused(why) if why.contains("no such file")));
        assert!(t0.elapsed() < Duration::from_secs(2), "no retrying a definite refusal");
        assert!(matches!(recv(&primary.requests), Handover::Refused { path, .. } if path == gone.display().to_string()));
        // Without the token nothing is accepted, and nothing reaches the app.
        let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).unwrap()).unwrap();
        let forged = Request { token: "0".repeat(64), open: Some(project.display().to_string()), ignored: Vec::new() };
        assert!(matches!(send(info.port, &serde_json::to_string(&forged).unwrap()), Err(Refusal::Refused(w)) if w.contains("wrong token")));
        assert!(matches!(send(info.port, "not json"), Err(Refusal::Refused(w)) if w.contains("not a request")));
        assert!(primary.requests.try_recv().is_err(), "refused requests never reach the app");
        // Ending the app removes its files; the next start is the running one again.
        let (guard, _rx, _wake) = primary.split();
        drop(guard);
        assert!(!dir.join(INFO_FILE).exists() && !dir.join(LOCK_FILE).exists());
        assert!(matches!(start(&dir, None, &[]), Start::Primary(_)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_ending_app_is_waited_for_instead_of_handed_to() {
        let dir = temp("closing");
        let Start::Primary(primary) = start(&dir, None, &[]) else { panic!("the first start runs") };
        let (guard, rx, _wake) = primary.split();
        let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).unwrap()).unwrap();
        let focus = serde_json::to_string(&Request { token: info.token.clone(), open: None, ignored: Vec::new() }).unwrap();
        // The app no longer reads its requests: never "ok".
        guard.closing_flag().store(true, Ordering::SeqCst);
        assert_eq!(send(info.port, &focus), Err(Refusal::Retry("closing".into())));
        assert!(rx.try_recv().is_err());
        // Its receiver gone (dropped before the guard): "closing" too.
        guard.closing_flag().store(false, Ordering::SeqCst);
        drop(rx);
        assert_eq!(send(info.port, &focus), Err(Refusal::Retry("closing".into())));
        // A start while it ends becomes the running app once the lock is free.
        let ending = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(500));
            drop(guard);
        });
        assert!(matches!(start_waiting(&dir, None, &[], Duration::from_secs(10)), Start::Primary(_)));
        ending.join().unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_slow_client_neither_blocks_others_nor_stays_connected() {
        let dir = temp("slow");
        let Start::Primary(primary) = start(&dir, None, &[]) else { panic!("the first start runs") };
        let info: Info = serde_json::from_slice(&std::fs::read(dir.join(INFO_FILE)).unwrap()).unwrap();
        // A client that sends a byte every 200 ms and never a whole line.
        let slow = TcpStream::connect((Ipv4Addr::LOCALHOST, info.port)).unwrap();
        let mut writer = slow.try_clone().unwrap();
        std::thread::spawn(move || {
            for _ in 0..50 {
                if writer.write_all(b" ").is_err() {
                    break;
                }
                std::thread::sleep(Duration::from_millis(200));
            }
        });
        // Another start is answered meanwhile.
        let t0 = Instant::now();
        assert!(matches!(start(&dir, None, &[]), Start::HandedOver));
        assert!(t0.elapsed() < Duration::from_secs(2));
        assert_eq!(recv(&primary.requests), Handover::Focus);
        // The slow one is cut off after the request time, not after 64 KiB of bytes.
        let _ = slow.set_read_timeout(Some(Duration::from_secs(10)));
        let mut answer = String::new();
        let _ = BufReader::new(&slow).read_line(&mut answer);
        assert!(answer.contains("too long"), "{answer:?}");
        assert!(t0.elapsed() < Duration::from_secs(5), "{:?}", t0.elapsed());
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
