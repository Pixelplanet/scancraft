//! Object isolation: which of a pass's fused surfels belong to the scanned object, as opposed to
//! the table it stood on, marker blocks and detached fragments (`scancraft_fusion::segment`).
//!
//! Alignment, refinement and merging work on the object only unless the user asks for the whole
//! scene. A pass's isolation is computed the first time it is needed and cached with a
//! fingerprint of the surfels it was made for, so a pass that gains or loses frames is isolated
//! again while an unchanged one is not.

use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};

use glam::Vec3;
use scancraft_align::{Oriented, PassCloud, Plane};
use scancraft_core::{Point3D, PointCloud};
use scancraft_fusion::{segment_object, SegmentConfig, SegmentLabel, Seeds, Segmentation};
use scancraft_render::GpuPoint;

/// Below this many object points the isolation is taken to have failed and the whole scene is
/// used.
const MIN_OBJECT_POINTS: usize = 200;
/// ...and below this share of the pass.
const MIN_OBJECT_SHARE: f32 = 0.05;

/// The object part of one pass, for one content of its fused model.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectMask {
    /// [`fingerprint`] of the surfels it was computed for.
    pub key: u64,
    /// Per surfel: part of the object.
    pub object: Vec<bool>,
    /// The table the object stood on, normal towards the object (pass frame).
    pub support: Option<Plane>,
    /// Surfels kept as object.
    pub kept: usize,
    /// No object could be told apart: every surfel is kept.
    pub fallback: bool,
}

impl ObjectMask {
    /// Keeps everything (nothing to tell apart).
    pub fn everything(key: u64, len: usize) -> Self {
        ObjectMask { key, object: vec![true; len], support: None, kept: len, fallback: true }
    }

    /// The items whose surfel is part of the object (`items` in surfel order).
    pub fn keep<T>(&self, items: impl IntoIterator<Item = T>) -> Vec<T> {
        items.into_iter().zip(&self.object).filter(|(_, o)| **o).map(|(t, _)| t).collect()
    }

    /// The pass with only its object surfels, and its table as the support plane.
    pub fn apply(&self, cloud: PassCloud) -> PassCloud {
        let PassCloud { points, views, .. } = cloud;
        PassCloud { points: self.keep(points), views, support: self.support }
    }
}

/// One pass's cached isolation, shared with the background jobs that need it.
pub type MaskCache = Arc<Mutex<Option<Arc<ObjectMask>>>>;

pub fn new_cache() -> MaskCache {
    Arc::new(Mutex::new(None))
}

/// A fingerprint of surfel positions: equal for the same model, different (almost surely) once
/// frames are added, discarded or restored.
pub fn fingerprint(positions: impl Iterator<Item = Vec3>) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    let mut n = 0usize;
    for p in positions {
        p.to_array().map(f32::to_bits).hash(&mut h);
        n += 1;
    }
    n.hash(&mut h);
    h.finish()
}

/// Segments `points` (a pass's surfels, its own frame) into object and the rest.
pub fn segment(points: &[Oriented]) -> ObjectMask {
    let positions: Vec<Vec3> = points.iter().map(|p| p.0).collect();
    let normals: Vec<Vec3> = points.iter().map(|p| p.1).collect();
    let key = fingerprint(positions.iter().copied());
    let seg = segment_object(&positions, Some(&normals), &SegmentConfig::default(), None, None);
    from_segmentation(key, &seg)
}

/// The mask of a segmentation, or [`ObjectMask::everything`] when it found no usable object.
fn from_segmentation(key: u64, seg: &Segmentation) -> ObjectMask {
    let object: Vec<bool> = seg.labels.iter().map(|l| *l == SegmentLabel::Object).collect();
    let kept = object.iter().filter(|o| **o).count();
    if kept < MIN_OBJECT_POINTS || (kept as f32) < MIN_OBJECT_SHARE * object.len() as f32 {
        return ObjectMask::everything(key, object.len());
    }
    let support = seg.support.map(|s| Plane::through(s.point, s.normal));
    ObjectMask { key, object, support, kept, fallback: false }
}

/// The mask for `points` from `cache`, segmenting (and caching) when the cached one was made for
/// other surfels.
pub fn mask_for(cache: &MaskCache, points: &[Oriented]) -> Arc<ObjectMask> {
    let key = fingerprint(points.iter().map(|p| p.0));
    if let Some(m) = cache.lock().ok().and_then(|c| c.clone()).filter(|m| m.key == key && m.object.len() == points.len()) {
        return m;
    }
    let mask = Arc::new(segment(points));
    if let Ok(mut c) = cache.lock() {
        *c = Some(Arc::clone(&mask));
    }
    mask
}

/// The last isolation computed for a pass, without waiting or computing (for the interface).
pub fn cached(cache: &MaskCache) -> Option<Arc<ObjectMask>> {
    cache.try_lock().ok().and_then(|c| c.clone())
}

/// A pass as alignment sees it: its object only, with its table, unless `whole_scene`.
pub fn pass_input(cloud: PassCloud, cache: &MaskCache, whole_scene: bool) -> PassCloud {
    if whole_scene {
        return cloud;
    }
    let mask = mask_for(cache, &cloud.points);
    mask.apply(cloud)
}

// ---------------------------------------------------------------------- Isolate Object tool

/// The user's picks in the Isolate Object tool, in order: (point, keep).
pub type Picks = Vec<(Vec3, bool)>;

/// The Isolate Object tool: the cloud it works on, the user's picks and the current result.
pub struct ObjectPicking {
    /// The working cloud when the tool was opened.
    pub input: Arc<PointCloud>,
    /// Picks in order: (point, keep). Keep picks select a cluster, the others exclude one.
    pub picks: Picks,
    /// Labels for `input` from the last segmentation, and the picks it was made for.
    pub result: Option<(Arc<Segmentation>, Picks)>,
    /// `input` coloured by label (or plain before the first result), and its generation.
    pub layer: Arc<Vec<GpuPoint>>,
    pub generation: u64,
    /// Points that can be clicked: a spread subset of `input`.
    pub targets: Arc<Vec<Vec3>>,
    /// The pass or merged model `input` is, which the kept object replaces.
    pub target: crate::model::CleanTarget,
}

/// Object, support and other colours of the Isolate Object preview.
pub const LABEL_COLOURS: [[u8; 3]; 3] = [[120, 200, 110], [125, 130, 142], [215, 90, 80]];

impl ObjectPicking {
    pub fn new(input: PointCloud, generation: u64, target: crate::model::CleanTarget) -> Self {
        let stride = (input.len() / 200_000).max(1);
        let targets = input.points.iter().step_by(stride).map(|p| Vec3::from(p.position)).collect();
        let layer = Arc::new(crate::model::tinted_points(&input, LABEL_COLOURS[0]));
        ObjectPicking { input: Arc::new(input), picks: Vec::new(), result: None, layer, generation, targets: Arc::new(targets), target }
    }

    pub fn seeds(&self) -> Seeds {
        let mut seeds = Seeds::default();
        for (p, keep) in &self.picks {
            if *keep {
                seeds.positive.push(*p);
            } else {
                seeds.negative.push(*p);
            }
        }
        seeds
    }

    /// The last result is for the current picks (the same picks, not just as many).
    pub fn current(&self) -> bool {
        self.result.as_ref().is_some_and(|(_, made_for)| *made_for == self.picks)
    }

    /// Takes a segmentation made for `picks`.
    pub fn set_result(&mut self, seg: Arc<Segmentation>, picks: Picks, generation: u64) {
        self.layer = Arc::new(labelled_points(&self.input, &seg.labels));
        self.generation = generation;
        self.result = Some((seg, picks));
    }

    /// The object points of the last result.
    pub fn object(&self) -> Option<PointCloud> {
        let (seg, _) = self.result.as_ref()?;
        let points: Vec<Point3D> = seg.select(&self.input.points, SegmentLabel::Object);
        (!points.is_empty()).then(|| PointCloud::from_points(points))
    }
}

/// Segments `cloud` with the user's seeds (the Isolate Object tool's job).
pub fn segment_with_seeds(cloud: &PointCloud, seeds: &Seeds) -> Segmentation {
    let seeds = (!seeds.positive.is_empty() || !seeds.negative.is_empty()).then_some(seeds);
    scancraft_fusion::segment_cloud(cloud, &SegmentConfig::default(), seeds, None)
}

/// GPU points coloured by label with a fixed light baked in.
pub fn labelled_points(cloud: &PointCloud, labels: &[SegmentLabel]) -> Vec<GpuPoint> {
    let light = Vec3::new(-0.35, -0.75, 0.55).normalize();
    cloud
        .points
        .iter()
        .zip(labels)
        .map(|(p, label)| {
            let colour = match label {
                SegmentLabel::Object => LABEL_COLOURS[0],
                SegmentLabel::Support => LABEL_COLOURS[1],
                SegmentLabel::Other => LABEL_COLOURS[2],
            };
            let k = 0.35 + 0.65 * Vec3::from(p.normal).dot(light).abs();
            let c = |v: u8| (v as f32 * k).min(255.0) as u8;
            let mut g = GpuPoint::from_point(p);
            g.color = [c(colour[0]), c(colour[1]), c(colour[2]), 255];
            g
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 200 x 200 mm table at z = 0 (normals up) with a 40 mm box standing on it.
    fn scene() -> Vec<Oriented> {
        let mut points = Vec::new();
        let step = 2.0;
        let n = (200.0 / step) as i32;
        for i in 0..n {
            for j in 0..n {
                let p = Vec3::new(-100.0 + i as f32 * step, -100.0 + j as f32 * step, 0.0);
                if p.x.abs() > 20.0 || p.y.abs() > 20.0 {
                    points.push((p, Vec3::Z));
                }
            }
        }
        let m = (40.0 / 1.0) as i32;
        for a in 0..=m {
            for b in 0..=m {
                let (u, v) = (-20.0 + a as f32, b as f32);
                points.push((Vec3::new(u, -20.0, v), -Vec3::Y));
                points.push((Vec3::new(u, 20.0, v), Vec3::Y));
                points.push((Vec3::new(-20.0, u, v), -Vec3::X));
                points.push((Vec3::new(20.0, u, v), Vec3::X));
                points.push((Vec3::new(u, -20.0 + b as f32, 40.0), Vec3::Z));
            }
        }
        points
    }

    #[test]
    fn isolation_drops_the_table_and_keeps_its_plane() {
        let points = scene();
        let mask = segment(&points);
        assert!(!mask.fallback);
        let table = points.iter().zip(&mask.object).filter(|((p, _), o)| **o && (p.x.abs() > 25.0 || p.y.abs() > 25.0)).count();
        assert_eq!(table, 0, "table points kept as object");
        let on_box = points.iter().zip(&mask.object).filter(|((p, _), o)| **o && p.z > 5.0).count();
        assert!(on_box > 5000, "{on_box} box points kept");
        let support = mask.support.expect("table plane");
        assert!(support.normal.dot(Vec3::Z) > 0.99 && support.distance(Vec3::ZERO).abs() < 1.0, "{support:?}");
        let isolated = mask.apply(PassCloud { points: points.clone(), ..Default::default() });
        assert_eq!(isolated.points.len(), mask.kept);
        assert_eq!(isolated.support, mask.support);
    }

    #[test]
    fn cache_is_reused_until_the_surfels_change() {
        let points = scene();
        let cache = new_cache();
        let a = mask_for(&cache, &points);
        let b = mask_for(&cache, &points);
        assert!(Arc::ptr_eq(&a, &b), "the unchanged pass was segmented again");
        let mut changed = points.clone();
        changed.truncate(points.len() - 10);
        let c = mask_for(&cache, &changed);
        assert!(!Arc::ptr_eq(&a, &c));
        assert_eq!(c.object.len(), changed.len());
        assert!(cached(&cache).is_some_and(|m| Arc::ptr_eq(&m, &c)));
    }

    #[test]
    fn whole_scene_keeps_everything_and_nothing_found_falls_back() {
        let points = scene();
        let cache = new_cache();
        let all = pass_input(PassCloud { points: points.clone(), ..Default::default() }, &cache, true);
        assert_eq!(all.points.len(), points.len());
        assert!(all.support.is_none() && cached(&cache).is_none(), "the whole scene needs no isolation");
        // A few scattered points: no object to tell apart, so all are kept.
        let sparse: Vec<Oriented> = (0..50).map(|i| (Vec3::new(i as f32 * 37.0, 0.0, 0.0), Vec3::Z)).collect();
        let mask = segment(&sparse);
        assert!(mask.fallback && mask.kept == sparse.len());
    }

    #[test]
    fn picks_become_seeds_and_results_follow_them() {
        let cloud = PointCloud::from_points(scene().into_iter().map(|(p, n)| Point3D::new(p.to_array()).with_normal(n.to_array())).collect());
        let mut picking = ObjectPicking::new(cloud, 1, crate::model::CleanTarget::Merged);
        picking.picks.push((Vec3::new(0.0, 0.0, 40.0), true));
        picking.picks.push((Vec3::new(90.0, 90.0, 0.0), false));
        let seeds = picking.seeds();
        assert_eq!((seeds.positive.len(), seeds.negative.len()), (1, 1));
        assert!(!picking.current());
        let seg = segment_with_seeds(&picking.input, &seeds);
        picking.set_result(Arc::new(seg), picking.picks.clone(), 2);
        assert!(picking.current());
        let object = picking.object().expect("object");
        assert!(object.points.iter().all(|p| p.position[2] > -1.0 && p.position[0].abs() < 25.0));
        assert_eq!(picking.layer.len(), picking.input.len());
    }

    #[test]
    fn a_result_for_other_picks_of_the_same_count_is_stale() {
        let cloud = PointCloud::from_points(scene().into_iter().map(|(p, n)| Point3D::new(p.to_array()).with_normal(n.to_array())).collect());
        let mut picking = ObjectPicking::new(cloud, 1, crate::model::CleanTarget::Merged);
        let (a, b, c, d) = (Vec3::new(0.0, 0.0, 40.0), Vec3::new(90.0, 90.0, 0.0), Vec3::new(-90.0, 90.0, 0.0), Vec3::new(90.0, -90.0, 0.0));
        picking.picks = vec![(a, true), (b, false), (c, false)];
        let made_for = picking.picks.clone();
        // While that segmentation runs, A is removed and D added: three picks again.
        picking.picks.remove(0);
        picking.picks.push((d, false));
        let seg = segment_with_seeds(&picking.input, &picking.seeds());
        picking.set_result(Arc::new(seg), made_for, 2);
        assert!(!picking.current(), "a result for [A, B, C] is not current for [B, C, D]");
    }
}
