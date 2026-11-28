//! Raw stream recording: the bytes a scanner sends, written to a file while scanning so a pass can
//! be decoded and processed again offline (`docs/architecture/scan-sessions.md` section 8).
//!
//! The streaming side never waits for the disk. [`RawRecorder::push`] hands a record to a writer
//! thread through a queue bounded in bytes; when the queue is full (the disk cannot keep up) the
//! record is dropped and counted, and scanning goes on. Blocking the socket reader instead would
//! stall the scanner's media server, which is what hangs it (`docs/HANDOFF.md` section 3).
//!
//! What a driver records is its own wire framing (the POP 4: `PACKET_MAGIC`, `u32` length,
//! packet); the recorder only appends bytes. [`RecordStats`] is shared with the app, which shows
//! the size, warns about dropped records and can ask the writer to stop (low disk space).

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};

/// Default queue limit: about 30 s of a POP 4 depth stream (several MB/s).
pub const DEFAULT_QUEUE_BYTES: usize = 192 << 20;

/// How long [`RawRecorder::open`] waits for an earlier recorder of the same file to finish
/// writing out its queue.
pub const EARLIER_WRITER_WAIT: std::time::Duration = std::time::Duration::from_secs(90);

/// While the queue is idle, written records are handed to the operating system at once (a crash
/// of the app then loses nothing already written) and synced to the disk this often.
pub const SYNC_INTERVAL: std::time::Duration = std::time::Duration::from_secs(10);

/// The recordings of this process, by file: a second recorder of a file waits for the first.
fn active_writers() -> std::sync::MutexGuard<'static, Vec<(PathBuf, Arc<RecordStats>)>> {
    static ACTIVE: Mutex<Vec<(PathBuf, Arc<RecordStats>)>> = Mutex::new(Vec::new());
    ACTIVE.lock().unwrap_or_else(|e| e.into_inner())
}

/// Counters of one recording, shared between the writer thread and the app.
#[derive(Debug, Default)]
pub struct RecordStats {
    /// Bytes and records written to the file (after the queue).
    pub bytes_written: AtomicU64,
    pub records_written: AtomicU64,
    /// Records the queue could not take (the disk was too slow) or that came after a write
    /// error or a stop request.
    pub records_dropped: AtomicU64,
    pub bytes_dropped: AtomicU64,
    /// Bytes waiting in the queue now, and the most there ever were.
    pub queued_bytes: AtomicU64,
    pub max_queued_bytes: AtomicU64,
    /// Set by the app to end the recording early (e.g. the drive is nearly full). Scanning goes on.
    pub stop: AtomicBool,
    /// The writer thread has written and flushed everything and closed the file.
    pub finished: AtomicBool,
    /// Size of the file when this recording started (> 0 when appending to an earlier one).
    pub start_offset: AtomicU64,
    error: Mutex<Option<String>>,
}

impl RecordStats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Why the recording stopped early, if it did.
    pub fn error(&self) -> Option<String> {
        self.error.lock().ok().and_then(|e| e.clone())
    }

    fn set_error(&self, e: String) {
        if let Ok(mut slot) = self.error.lock() {
            slot.get_or_insert(e);
        }
    }

    /// A copy of the counters (for the capture record).
    pub fn snapshot(&self) -> RecordSnapshot {
        RecordSnapshot {
            bytes_written: self.bytes_written.load(Ordering::Relaxed),
            records_written: self.records_written.load(Ordering::Relaxed),
            records_dropped: self.records_dropped.load(Ordering::Relaxed),
            bytes_dropped: self.bytes_dropped.load(Ordering::Relaxed),
            max_queued_bytes: self.max_queued_bytes.load(Ordering::Relaxed),
            start_offset: self.start_offset.load(Ordering::Relaxed),
            finished: self.finished.load(Ordering::Relaxed),
            stopped_early: self.stop.load(Ordering::Relaxed),
            error: self.error(),
        }
    }
}

/// [`RecordStats`] as saved.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RecordSnapshot {
    pub bytes_written: u64,
    pub records_written: u64,
    pub records_dropped: u64,
    pub bytes_dropped: u64,
    pub max_queued_bytes: u64,
    pub start_offset: u64,
    pub finished: bool,
    pub stopped_early: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Where a device records the streams of its next scan (see `ScannerDevice::set_recording`).
#[derive(Debug, Clone)]
pub struct RecordingTarget {
    /// The main stream (depth / IR), e.g. `pass-<id>/raw.bin`.
    pub main: PathBuf,
    /// The colour stream, when the device sends one separately, e.g. `pass-<id>/raw_rgb.bin`.
    pub rgb: PathBuf,
    /// Append to existing files (a continued pass) instead of replacing them.
    pub append: bool,
    pub main_stats: Arc<RecordStats>,
    pub rgb_stats: Arc<RecordStats>,
    /// Queue limit per stream, bytes.
    pub queue_bytes: usize,
}

impl RecordingTarget {
    pub fn new(main: PathBuf, rgb: PathBuf, append: bool) -> Self {
        Self { main, rgb, append, main_stats: RecordStats::new(), rgb_stats: RecordStats::new(), queue_bytes: DEFAULT_QUEUE_BYTES }
    }
}

/// What a device reports about the scan it just started, for the capture record: how its
/// recorded streams are laid out and decoded, what it sent to set the scan up, and the
/// calibration files needed to decode the recording offline.
#[derive(Debug, Clone, Default)]
pub struct RecordingInfo {
    /// Free-form description of the stream(s) (format id, layout, resolution, framing) and the
    /// setup commands sent, as JSON.
    pub stream: serde_json::Value,
    /// Calibration files as received from the device: (relative file name, bytes).
    pub calibration: Vec<(String, Vec<u8>)>,
    /// The files actually being written for this scan (the target's, or the
    /// `SCANCRAFT_RECORD_DIR` ones), main stream first; empty when nothing is recorded.
    pub files: Vec<PathBuf>,
}

struct Queue {
    items: VecDeque<Vec<u8>>,
    bytes: usize,
    closed: bool,
}

struct Shared {
    queue: Mutex<Queue>,
    ready: Condvar,
}

/// Appends records to a file on a writer thread (see the module docs). Dropping it lets the
/// writer finish the queue, flush and close the file in the background; it never waits.
pub struct RawRecorder {
    shared: Arc<Shared>,
    stats: Arc<RecordStats>,
    limit: usize,
    path: PathBuf,
}

impl RawRecorder {
    /// Opens (or appends to) `path` and starts the writer thread.
    ///
    /// A new recording (`append == false`) never replaces a file that already holds data (a
    /// pass id used again after a crash would otherwise truncate the only copy of a scan): that
    /// is an [`std::io::ErrorKind::AlreadyExists`] error. When an earlier recorder of the same
    /// file is still writing out its queue (a pass continued right after Stop, with a backlog on
    /// a slow disk), this waits for it first, at most [`EARLIER_WRITER_WAIT`], and fails instead
    /// of putting a second writer on the file when it is still writing after that.
    pub fn open(path: &Path, append: bool, stats: Arc<RecordStats>, queue_bytes: usize) -> std::io::Result<Self> {
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir)?;
        }
        let key = std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf());
        let earlier: Vec<Arc<RecordStats>> = active_writers().iter().filter(|(p, s)| *p == key && !s.finished.load(Ordering::SeqCst)).map(|(_, s)| s.clone()).collect();
        if !earlier.is_empty() && !wait_finished(&earlier, EARLIER_WRITER_WAIT) {
            return Err(std::io::Error::new(std::io::ErrorKind::WouldBlock, format!("{} is still being written by the previous recording", path.display())));
        }
        let file = if append {
            OpenOptions::new().create(true).append(true).open(path)?
        } else {
            match OpenOptions::new().write(true).create_new(true).open(path) {
                Ok(f) => f,
                // An empty file (a recording that never got a record) may be replaced.
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                    if std::fs::metadata(path).map(|m| m.len()).unwrap_or(1) > 0 {
                        return Err(std::io::Error::new(std::io::ErrorKind::AlreadyExists, format!("{} already holds a recording; it is not replaced", path.display())));
                    }
                    File::create(path)?
                }
                Err(e) => return Err(e),
            }
        };
        let offset = file.metadata().map(|m| m.len()).unwrap_or(0);
        stats.start_offset.store(offset, Ordering::Relaxed);
        stats.finished.store(false, Ordering::SeqCst);
        let shared = Arc::new(Shared { queue: Mutex::new(Queue { items: VecDeque::new(), bytes: 0, closed: false }), ready: Condvar::new() });
        let (writer_shared, writer_stats) = (shared.clone(), stats.clone());
        let name = path.display().to_string();
        std::thread::Builder::new().name("raw-recorder".into()).spawn(move || write_loop(file, writer_shared, writer_stats, name))?;
        {
            let mut active = active_writers();
            active.retain(|(_, s)| !s.finished.load(Ordering::SeqCst));
            active.push((key, stats.clone()));
        }
        Ok(Self { shared, stats, limit: queue_bytes.max(1 << 20), path: path.to_path_buf() })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn stats(&self) -> &Arc<RecordStats> {
        &self.stats
    }

    /// Queues one record. Never blocks on the disk: a record that does not fit into the queue,
    /// or comes after a write error or a stop request, is dropped and counted.
    pub fn push(&self, record: Vec<u8>) {
        let len = record.len();
        let stopped = self.stats.stop.load(Ordering::Relaxed) || self.stats.error.lock().map(|e| e.is_some()).unwrap_or(true);
        let Ok(mut q) = self.shared.queue.lock() else { return };
        if stopped || q.bytes + len > self.limit {
            drop(q);
            self.stats.records_dropped.fetch_add(1, Ordering::Relaxed);
            self.stats.bytes_dropped.fetch_add(len as u64, Ordering::Relaxed);
            return;
        }
        q.bytes += len;
        q.items.push_back(record);
        let queued = q.bytes as u64;
        drop(q);
        self.stats.queued_bytes.store(queued, Ordering::Relaxed);
        self.stats.max_queued_bytes.fetch_max(queued, Ordering::Relaxed);
        self.shared.ready.notify_one();
    }
}

impl Drop for RawRecorder {
    fn drop(&mut self) {
        if let Ok(mut q) = self.shared.queue.lock() {
            q.closed = true;
        }
        self.shared.ready.notify_all();
    }
}

fn write_loop(file: File, shared: Arc<Shared>, stats: Arc<RecordStats>, name: String) {
    enum Next {
        Item(Vec<u8>),
        /// Nothing queued: time to flush (and maybe sync) what was written.
        Idle,
        Closed,
    }
    let mut out = Some(BufWriter::with_capacity(4 << 20, file));
    // Written but not yet handed to the operating system / not yet synced to the disk.
    let (mut unflushed, mut unsynced) = (false, false);
    let mut last_sync = std::time::Instant::now();
    loop {
        let next = {
            let Ok(mut q) = shared.queue.lock() else { break };
            loop {
                if let Some(item) = q.items.pop_front() {
                    q.bytes -= item.len();
                    stats.queued_bytes.store(q.bytes as u64, Ordering::Relaxed);
                    break Next::Item(item);
                }
                if q.closed {
                    break Next::Closed;
                }
                if unflushed {
                    break Next::Idle;
                }
                if unsynced {
                    let left = SYNC_INTERVAL.saturating_sub(last_sync.elapsed());
                    if left.is_zero() {
                        break Next::Idle;
                    }
                    q = match shared.ready.wait_timeout(q, left) {
                        Ok((q, _)) => q,
                        Err(_) => return,
                    };
                } else {
                    q = match shared.ready.wait(q) {
                        Ok(q) => q,
                        Err(_) => return,
                    };
                }
            }
        };
        let item = match next {
            Next::Closed => break,
            Next::Idle => {
                let sync = unsynced && last_sync.elapsed() >= SYNC_INTERVAL;
                if let Some(w) = out.as_mut() {
                    let done = w.flush().and_then(|_| if sync { w.get_ref().sync_data() } else { Ok(()) });
                    if let Err(e) = done {
                        log::warn!("raw recording {name} stopped: {e}");
                        stats.set_error(e.to_string());
                        out = None;
                    }
                }
                unflushed = false;
                if sync || out.is_none() {
                    unsynced = false;
                    last_sync = std::time::Instant::now();
                }
                continue;
            }
            Next::Item(item) => item,
        };
        let stop = stats.stop.load(Ordering::Relaxed);
        match out.as_mut() {
            Some(w) if !stop => match w.write_all(&item) {
                Ok(()) => {
                    stats.bytes_written.fetch_add(item.len() as u64, Ordering::Relaxed);
                    stats.records_written.fetch_add(1, Ordering::Relaxed);
                    (unflushed, unsynced) = (true, true);
                }
                Err(e) => {
                    log::warn!("raw recording {name} stopped: {e}");
                    stats.set_error(e.to_string());
                    stats.records_dropped.fetch_add(1, Ordering::Relaxed);
                    stats.bytes_dropped.fetch_add(item.len() as u64, Ordering::Relaxed);
                    out = None;
                }
            },
            _ => {
                if stop && out.is_some() {
                    log::warn!("raw recording {name} stopped on request");
                    if let Some(mut w) = out.take() {
                        let _ = w.flush();
                    }
                }
                stats.records_dropped.fetch_add(1, Ordering::Relaxed);
                stats.bytes_dropped.fetch_add(item.len() as u64, Ordering::Relaxed);
            }
        }
    }
    if let Some(mut w) = out.take() {
        if let Err(e) = w.flush().and_then(|_| w.get_ref().sync_data()) {
            log::warn!("raw recording {name}: {e}");
            stats.set_error(e.to_string());
        }
    }
    stats.finished.store(true, Ordering::SeqCst);
}

/// Waits until every recording in `stats` has finished writing, at most `limit`. `true` when all
/// did.
pub fn wait_finished(stats: &[Arc<RecordStats>], limit: std::time::Duration) -> bool {
    let t0 = std::time::Instant::now();
    loop {
        if stats.iter().all(|s| s.finished.load(Ordering::SeqCst)) {
            return true;
        }
        if t0.elapsed() >= limit {
            return false;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-rec-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn records_are_written_in_order_and_appended() {
        let dir = temp("order");
        let path = dir.join("pass-1").join("raw.bin");
        let stats = RecordStats::new();
        let rec = RawRecorder::open(&path, false, stats.clone(), 1 << 20).unwrap();
        for i in 0..100u8 {
            rec.push(vec![i; 100]);
        }
        drop(rec);
        assert!(wait_finished(&[stats.clone()], std::time::Duration::from_secs(5)));
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len(), 10_000);
        assert!(bytes.chunks(100).enumerate().all(|(i, c)| c.iter().all(|&b| b == i as u8)));
        let s = stats.snapshot();
        assert_eq!((s.records_written, s.records_dropped, s.start_offset), (100, 0, 0));

        let again = RecordStats::new();
        let rec = RawRecorder::open(&path, true, again.clone(), 1 << 20).unwrap();
        rec.push(vec![7; 10]);
        drop(rec);
        assert!(wait_finished(&[again.clone()], std::time::Duration::from_secs(5)));
        assert_eq!(std::fs::read(&path).unwrap().len(), 10_010);
        assert_eq!(again.snapshot().start_offset, 10_000, "the continuation starts where the first recording ended");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_full_queue_drops_records_instead_of_blocking() {
        let dir = temp("full");
        let path = dir.join("raw.bin");
        let stats = RecordStats::new();
        let rec = RawRecorder::open(&path, false, stats.clone(), 1 << 20).unwrap();
        // 16 MB at once into a 1 MB queue: what does not fit is dropped, nothing waits.
        let t0 = std::time::Instant::now();
        for _ in 0..64 {
            rec.push(vec![1; 256 << 10]);
        }
        assert!(t0.elapsed() < std::time::Duration::from_secs(2), "push never waits for the disk");
        drop(rec);
        assert!(wait_finished(&[stats.clone()], std::time::Duration::from_secs(10)));
        let s = stats.snapshot();
        assert_eq!(s.records_written + s.records_dropped, 64);
        assert_eq!(std::fs::metadata(&path).unwrap().len(), s.bytes_written);
        assert!(s.max_queued_bytes <= 1 << 20);

        // A stop request ends the writing; later records are counted as dropped.
        let stopped = RecordStats::new();
        let rec = RawRecorder::open(&dir.join("b.bin"), false, stopped.clone(), 1 << 20).unwrap();
        rec.push(vec![1; 10]);
        assert!(wait_until(|| stopped.records_written.load(Ordering::Relaxed) == 1));
        stopped.stop.store(true, Ordering::Relaxed);
        rec.push(vec![1; 10]);
        drop(rec);
        assert!(wait_finished(&[stopped.clone()], std::time::Duration::from_secs(5)));
        let s = stopped.snapshot();
        assert_eq!((s.records_written, s.records_dropped, s.stopped_early), (1, 1, true));
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn wait_until(f: impl Fn() -> bool) -> bool {
        let t0 = std::time::Instant::now();
        while !f() {
            if t0.elapsed() > std::time::Duration::from_secs(5) {
                return false;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        true
    }

    #[test]
    fn a_new_recording_never_replaces_an_existing_one() {
        // A pass id used again after a crash: its raw.bin holds the only copy of that scan.
        let dir = temp("exists");
        let path = dir.join("pass-3").join("raw.bin");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, vec![9u8; 5000]).unwrap();
        let err = RawRecorder::open(&path, false, RecordStats::new(), 1 << 20).err().expect("refused");
        assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists);
        assert_eq!(std::fs::read(&path).unwrap(), vec![9u8; 5000], "the earlier recording is untouched");
        // An empty leftover may be replaced.
        let empty = dir.join("empty.bin");
        std::fs::write(&empty, b"").unwrap();
        let stats = RecordStats::new();
        let rec = RawRecorder::open(&empty, false, stats.clone(), 1 << 20).unwrap();
        rec.push(vec![1; 10]);
        drop(rec);
        assert!(wait_finished(&[stats], std::time::Duration::from_secs(5)));
        assert_eq!(std::fs::read(&empty).unwrap().len(), 10);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_continuation_waits_for_the_earlier_writer_of_its_file() {
        // Stop with a backlog in the queue, then Continue at once: the second recorder must not
        // write while the first one still drains, and starts where the first one ended.
        let dir = temp("drain");
        let path = dir.join("raw.bin");
        let first = RecordStats::new();
        let rec = RawRecorder::open(&path, false, first.clone(), DEFAULT_QUEUE_BYTES).unwrap();
        const RECORDS: usize = 96;
        for i in 0..RECORDS {
            rec.push(vec![(i % 200) as u8; 1 << 20]);
        }
        drop(rec);
        let second = RecordStats::new();
        let rec = RawRecorder::open(&path, true, second.clone(), DEFAULT_QUEUE_BYTES).unwrap();
        assert!(first.finished.load(Ordering::SeqCst), "the earlier writer finished before the file was opened again");
        let written = first.bytes_written.load(Ordering::Relaxed);
        assert_eq!(written, (RECORDS as u64) << 20, "nothing of the first session was dropped");
        assert_eq!(second.start_offset.load(Ordering::Relaxed), written);
        for _ in 0..4 {
            rec.push(vec![0xEE; 1 << 20]);
        }
        drop(rec);
        assert!(wait_finished(&[second.clone()], std::time::Duration::from_secs(30)));
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len() as u64, written + (4 << 20));
        assert!(bytes[written as usize..].iter().all(|&b| b == 0xEE), "the continuation is intact after the first session");
        assert!(bytes[..written as usize].chunks(1 << 20).enumerate().all(|(i, c)| c.iter().all(|&b| b == (i % 200) as u8)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn written_records_reach_the_file_while_recording() {
        // A crash of the app must not lose what was already written: the buffer is handed to the
        // operating system whenever the queue runs empty, not only at the end.
        let dir = temp("flush");
        let path = dir.join("raw.bin");
        let stats = RecordStats::new();
        let rec = RawRecorder::open(&path, false, stats.clone(), 1 << 20).unwrap();
        for i in 0..10u8 {
            rec.push(vec![i; 1000]);
        }
        assert!(wait_until(|| std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0) == 10_000), "the records are in the file before the recording ends");
        assert!(!stats.finished.load(Ordering::SeqCst));
        drop(rec);
        assert!(wait_finished(&[stats], std::time::Duration::from_secs(5)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_unwritable_path_is_an_error_at_open() {
        let dir = temp("bad");
        std::fs::create_dir_all(&dir).unwrap();
        // A folder where the file should be.
        let path = dir.join("raw.bin");
        std::fs::create_dir_all(&path).unwrap();
        assert!(RawRecorder::open(&path, false, RecordStats::new(), 1 << 20).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
