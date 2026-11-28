//! Memory use of this process: the current and peak working set (Windows), and a sampler that
//! records the highest working set between two marks, for per-stage peaks.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

#[cfg(windows)]
fn counters() -> Option<(usize, usize)> {
    #[repr(C)]
    struct Pmc {
        cb: u32,
        page_fault_count: u32,
        peak_working_set: usize,
        working_set: usize,
        quota_peak_paged: usize,
        quota_paged: usize,
        quota_peak_nonpaged: usize,
        quota_nonpaged: usize,
        pagefile: usize,
        peak_pagefile: usize,
    }
    extern "system" {
        fn GetCurrentProcess() -> isize;
        fn K32GetProcessMemoryInfo(process: isize, pmc: *mut Pmc, cb: u32) -> i32;
    }
    // SAFETY: a plain-data struct filled by the OS for the current process.
    let mut p: Pmc = unsafe { std::mem::zeroed() };
    p.cb = std::mem::size_of::<Pmc>() as u32;
    let ok = unsafe { K32GetProcessMemoryInfo(GetCurrentProcess(), &mut p, p.cb) };
    (ok != 0).then_some((p.working_set, p.peak_working_set))
}

#[cfg(not(windows))]
fn counters() -> Option<(usize, usize)> {
    None
}

const MB: f64 = 1048576.0;

/// Current working set, MB.
pub fn working_set_mb() -> Option<f64> {
    counters().map(|c| c.0 as f64 / MB)
}

/// Peak working set of the process so far, MB.
pub fn peak_working_set_mb() -> Option<f64> {
    counters().map(|c| c.1 as f64 / MB)
}

/// Samples the working set every 20 ms; [`Sampler::mark`] returns the highest value since the
/// last mark.
pub struct Sampler {
    peak: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
}

impl Sampler {
    pub fn start() -> Self {
        let peak = Arc::new(AtomicUsize::new(counters().map_or(0, |c| c.0)));
        let stop = Arc::new(AtomicBool::new(false));
        let (p, s) = (Arc::clone(&peak), Arc::clone(&stop));
        std::thread::spawn(move || {
            while !s.load(Ordering::Relaxed) {
                if let Some((ws, _)) = counters() {
                    p.fetch_max(ws, Ordering::Relaxed);
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        });
        Sampler { peak, stop }
    }

    /// The highest working set (MB) since the last mark; starts the next interval.
    pub fn mark(&self) -> f64 {
        let now = counters().map_or(0, |c| c.0);
        let peak = self.peak.swap(now, Ordering::Relaxed).max(now);
        (peak as f64 / MB * 10.0).round() / 10.0
    }
}

impl Drop for Sampler {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}
