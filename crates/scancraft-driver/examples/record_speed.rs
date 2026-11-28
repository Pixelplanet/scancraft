//! How fast the raw recorder writes to a drive, against the POP 4's stream rate.
//!
//! `cargo run --release -p scancraft-driver --example record_speed -- <folder> [MB] [paced MB/s] [paced s]`
//!
//! 1. Flat out: `MB` (default 512) in 700 kB records (a Full-field HD packet), pushed as fast as
//!    the queue takes them; the time includes flushing and `sync_data`, so the page cache does not
//!    flatter the result.
//! 2. Paced like the scanner: `paced MB/s` (default 8, above the POP 4's 5-8 MB/s) for `paced s`
//!    seconds (default 10): records dropped, most bytes queued.
//!
//! The files are deleted afterwards.

use scancraft_driver::recording::{wait_finished, RawRecorder, RecordStats, DEFAULT_QUEUE_BYTES};
use std::time::{Duration, Instant};

const RECORD: usize = 700 << 10;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = std::path::PathBuf::from(args.get(1).expect("folder"));
    let mb: usize = args.get(2).and_then(|v| v.parse().ok()).unwrap_or(512);
    let paced_rate: f64 = args.get(3).and_then(|v| v.parse().ok()).unwrap_or(8.0);
    let paced_secs: f64 = args.get(4).and_then(|v| v.parse().ok()).unwrap_or(10.0);
    std::fs::create_dir_all(&dir).unwrap();
    let record: Vec<u8> = (0..RECORD).map(|i| (i * 31 % 251) as u8).collect();

    let path = dir.join("record_speed_flat.bin");
    let stats = RecordStats::new();
    let rec = RawRecorder::open(&path, false, stats.clone(), DEFAULT_QUEUE_BYTES).unwrap();
    let n = mb * (1 << 20) / RECORD;
    let t0 = Instant::now();
    let mut pushed = 0;
    while pushed < n {
        // Flat out, but without dropping: wait while the queue is full.
        if stats.queued_bytes.load(std::sync::atomic::Ordering::Relaxed) as usize + RECORD > DEFAULT_QUEUE_BYTES {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        }
        rec.push(record.clone());
        pushed += 1;
    }
    drop(rec);
    assert!(wait_finished(&[stats.clone()], Duration::from_secs(600)));
    let secs = t0.elapsed().as_secs_f64();
    let s = stats.snapshot();
    println!("flat out: {:.0} MB in {secs:.2} s = {:.0} MB/s (dropped {})", s.bytes_written as f64 / 1e6, s.bytes_written as f64 / 1e6 / secs, s.records_dropped);
    let _ = std::fs::remove_file(&path);

    let path = dir.join("record_speed_paced.bin");
    let stats = RecordStats::new();
    let rec = RawRecorder::open(&path, false, stats.clone(), DEFAULT_QUEUE_BYTES).unwrap();
    let interval = Duration::from_secs_f64(RECORD as f64 / (paced_rate * 1e6));
    let t0 = Instant::now();
    let mut next = t0;
    let mut slowest_push = Duration::ZERO;
    while t0.elapsed().as_secs_f64() < paced_secs {
        let p = Instant::now();
        rec.push(record.clone());
        slowest_push = slowest_push.max(p.elapsed());
        next += interval;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
    }
    drop(rec);
    assert!(wait_finished(&[stats.clone()], Duration::from_secs(600)));
    let s = stats.snapshot();
    println!(
        "paced {paced_rate} MB/s for {paced_secs} s: {:.0} MB written, {} records dropped, at most {:.1} MB queued, slowest push {:.2} ms",
        s.bytes_written as f64 / 1e6,
        s.records_dropped,
        s.max_queued_bytes as f64 / 1e6,
        slowest_push.as_secs_f64() * 1000.0
    );
    let _ = std::fs::remove_file(&path);
}
