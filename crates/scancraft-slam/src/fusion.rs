//! Background fusion of keyframes into the display surface.
//!
//! Merging a keyframe (~100k points at 0.5 mm pitch) into the surfel model costs ~100–150 ms,
//! about a whole frame interval of the POP 4; done on the tracking thread it made the app fall
//! behind whenever the scanner moved. Tracking only needs the coarse registration model, so the
//! fine surface is merged here, on its own thread, and shows up a moment later.

use crate::carve::FreeSpaceView;
use crate::surface::SurfaceModel;
use glam::Vec3;
use scancraft_core::{Point3D, Transform};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

/// One keyframe to merge: points (sensor frame) placed at `pose`, then free-space carving with
/// the keyframe's view.
pub(crate) struct FuseJob {
    pub pose: Transform,
    pub points: Vec<(Vec3, Vec3)>,
    pub view: Option<FreeSpaceView>,
    pub carve_margin_mm: f32,
    pub carve_margin_per_mm: f32,
}

/// Merges a job into the model (also used synchronously for rebuilds, so both give the same
/// surface). Returns the number of new surfels.
pub(crate) fn fuse_into(model: &mut SurfaceModel, job: &FuseJob) -> usize {
    let mut bounds = (job.pose.translation, job.pose.translation);
    let new_points = insert_points(model, &job.pose, &job.points, &mut bounds);
    finish(model, job, bounds);
    new_points
}

/// Inserts points (sensor frame) at `pose`, growing `bounds` (the world-space box this view
/// saw, sensor to measured surface). Returns the number of new surfels.
fn insert_points(model: &mut SurfaceModel, pose: &Transform, points: &[(Vec3, Vec3)], bounds: &mut (Vec3, Vec3)) -> usize {
    let mut new_points = 0;
    for &(p, n) in points {
        // Squarely seen surface is measured better than surface seen at a grazing angle.
        let view_cos = n.dot(p.normalize_or_zero()).abs();
        let w = pose.transform_point3(p);
        bounds.0 = bounds.0.min(w);
        bounds.1 = bounds.1.max(w);
        if model.insert(w, pose.rotation * n, view_cos).is_some() {
            new_points += 1;
        }
    }
    new_points
}

/// Free-space carving with the job's view inside `bounds`, then a visibility refresh.
fn finish(model: &mut SurfaceModel, job: &FuseJob, (lo, hi): (Vec3, Vec3)) {
    let pose = job.pose;
    if let Some(view) = &job.view {
        let to_sensor = pose.inverse();
        let (margin, per_mm) = (job.carve_margin_mm, job.carve_margin_per_mm);
        // Only surfels inside what this view saw can be seen through.
        let pad = Vec3::splat(margin + 10.0);
        model.carve_within(lo - pad, hi + pad, |p: Vec3| {
            let q = to_sensor.transform_point3(p);
            view.sees_through(q, margin + per_mm * q.y)
        });
    }
    model.refresh();
}

/// Points merged per lock hold, so readers of the model never wait for a whole keyframe.
const CHUNK: usize = 16_384;

pub(crate) struct Fusion {
    model: Arc<Mutex<SurfaceModel>>,
    tx: Option<Sender<FuseJob>>,
    /// Jobs submitted but not merged yet, with a condition variable to wait for zero.
    pending: Arc<(Mutex<usize>, Condvar)>,
    /// Output surfels after the last merged job (readable without the lock).
    visible: Arc<AtomicUsize>,
    /// Surface that became visible in merged jobs and was not handed out yet. Filled by the
    /// thread after each job, so taking it never waits for a merge (the model lock is busy
    /// right after each submit, which is exactly when the tracking thread asks).
    outbox: Arc<Mutex<Vec<Point3D>>>,
    /// Model revision (withdrawals) after the last merged job, and the one the last
    /// `take_update` consumer has seen.
    revision: Arc<AtomicU64>,
    seen_revision: AtomicU64,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Fusion {
    pub fn new(model: SurfaceModel) -> Self {
        let model = Arc::new(Mutex::new(model));
        let pending = Arc::new((Mutex::new(0usize), Condvar::new()));
        let visible = Arc::new(AtomicUsize::new(0));
        let outbox = Arc::new(Mutex::new(Vec::new()));
        let revision = Arc::new(AtomicU64::new(model.lock().unwrap().revision()));
        let (tx, rx) = channel::<FuseJob>();
        let thread = {
            let (model, pending, visible) = (model.clone(), pending.clone(), visible.clone());
            let (outbox, revision) = (outbox.clone(), revision.clone());
            std::thread::Builder::new()
                .name("scancraft-fusion".into())
                .spawn(move || {
                    while let Ok(job) = rx.recv() {
                        let mut bounds = (job.pose.translation, job.pose.translation);
                        for chunk in job.points.chunks(CHUNK) {
                            insert_points(&mut model.lock().unwrap(), &job.pose, chunk, &mut bounds);
                        }
                        {
                            let mut m = model.lock().unwrap();
                            finish(&mut m, &job, bounds);
                            visible.store(m.len(), Ordering::Relaxed);
                            revision.store(m.revision(), Ordering::Relaxed);
                            let new = m.take_new();
                            outbox.lock().unwrap().extend(new);
                        }
                        let (count, cv) = &*pending;
                        let mut n = count.lock().unwrap();
                        *n = n.saturating_sub(1);
                        cv.notify_all();
                    }
                })
                .expect("spawn fusion thread")
        };
        let seen_revision = AtomicU64::new(revision.load(Ordering::Relaxed));
        Self { model, tx: Some(tx), pending, visible, outbox, revision, seen_revision, thread: Some(thread) }
    }

    pub fn submit(&self, job: FuseJob) {
        *self.pending.0.lock().unwrap() += 1;
        if let Some(tx) = &self.tx {
            if tx.send(job).is_err() {
                *self.pending.0.lock().unwrap() -= 1;
            }
        }
    }

    /// Waits until every submitted job is merged.
    pub fn flush(&self) {
        let (count, cv) = &*self.pending;
        let mut n = count.lock().unwrap();
        while *n > 0 {
            n = cv.wait(n).unwrap();
        }
    }

    /// The model after all submitted jobs.
    pub fn lock(&self) -> MutexGuard<'_, SurfaceModel> {
        self.flush();
        self.model.lock().unwrap()
    }

    /// Surface merged since the previous call (never waits for a merge in progress).
    pub fn take_new(&self) -> Vec<Point3D> {
        std::mem::take(&mut *self.outbox.lock().unwrap())
    }

    /// Whether points already handed out were withdrawn since the last [`mark_seen`](Self::mark_seen).
    pub fn withdrawn_since_seen(&self) -> bool {
        self.revision.load(Ordering::Relaxed) != self.seen_revision.load(Ordering::Relaxed)
    }

    /// After a consumer took a full copy of `model` (or a rebuild made one current): nothing is
    /// pending any more.
    pub fn mark_seen(&self, model: &mut SurfaceModel) {
        model.mark_all_reported();
        self.outbox.lock().unwrap().clear();
        self.revision.store(model.revision(), Ordering::Relaxed);
        self.seen_revision.store(model.revision(), Ordering::Relaxed);
    }

    pub fn visible(&self) -> usize {
        self.visible.load(Ordering::Relaxed)
    }

    pub fn set_visible(&self, n: usize) {
        self.visible.store(n, Ordering::Relaxed);
    }
}

impl Drop for Fusion {
    fn drop(&mut self) {
        self.tx = None; // ends the thread's loop
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
