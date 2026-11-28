//! The app's log: what `RUST_LOG` asks for goes to the console as before (`env_logger`), and
//! warnings plus ScanCraft's own information messages (scanner setup, stream stalls, recording
//! problems) also go to a file, `<local app data>/ScanCraft/logs/scancraft-<UTC time>-<pid>.log`.
//! Capture records name the file (`CaptureRecord::app_log`), so a scan's log can be found later.
//! The 30 newest files are kept.

use std::io::Write;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

static LOG_FILE: OnceLock<PathBuf> = OnceLock::new();
const KEEP: usize = 30;

/// The log file of this run, when there is one.
pub fn path() -> Option<PathBuf> {
    LOG_FILE.get().cloned()
}

struct Logger {
    console: env_logger::Logger,
    file: Mutex<std::io::LineWriter<std::fs::File>>,
}

fn to_file(meta: &log::Metadata) -> bool {
    meta.level() <= log::Level::Warn || (meta.level() <= log::Level::Info && meta.target().starts_with("scancraft"))
}

impl log::Log for Logger {
    fn enabled(&self, meta: &log::Metadata) -> bool {
        to_file(meta) || self.console.enabled(meta)
    }

    fn log(&self, record: &log::Record) {
        if self.console.matches(record) {
            self.console.log(record);
        }
        if to_file(record.metadata()) {
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
            let stamp = format!("{}.{:03}", scancraft_io::fsutil::format_utc(now.as_secs()), now.subsec_millis());
            if let Ok(mut f) = self.file.lock() {
                let _ = writeln!(f, "{stamp} {:<5} {}: {}", record.level(), record.target(), record.args());
            }
        }
    }

    fn flush(&self) {
        self.console.flush();
        if let Ok(mut f) = self.file.lock() {
            let _ = f.flush();
        }
    }
}

/// Sets up logging (call once at start). Without a writable log folder only the console logs.
pub fn init() {
    let console = env_logger::Builder::from_default_env().build();
    let dir = scancraft_io::fsutil::local_data_dir().join("ScanCraft").join("logs");
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let name = format!("scancraft-{}-{}.log", scancraft_io::fsutil::format_utc(secs).replace([' ', ':'], "-"), std::process::id());
    let file = std::fs::create_dir_all(&dir).and_then(|_| std::fs::File::create(dir.join(&name)));
    let max = console.filter().max(log::LevelFilter::Info);
    match file {
        Ok(f) => {
            let _ = LOG_FILE.set(dir.join(&name));
            if log::set_boxed_logger(Box::new(Logger { console, file: Mutex::new(std::io::LineWriter::new(f)) })).is_ok() {
                log::set_max_level(max);
            }
            prune(&dir);
            log::info!("ScanCraft {}", crate::capture::version_label());
        }
        Err(_) => {
            let filter = console.filter();
            if log::set_boxed_logger(Box::new(console)).is_ok() {
                log::set_max_level(filter);
            }
        }
    }
}

/// Keeps the newest [`KEEP`] log files.
fn prune(dir: &std::path::Path) {
    let mut logs: Vec<(std::time::SystemTime, PathBuf)> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("scancraft-") && e.file_name().to_string_lossy().ends_with(".log"))
        .filter_map(|e| Some((e.metadata().ok()?.modified().ok()?, e.path())))
        .collect();
    logs.sort_by(|a, b| b.0.cmp(&a.0));
    for (_, p) in logs.into_iter().skip(KEEP) {
        let _ = std::fs::remove_file(p);
    }
}
