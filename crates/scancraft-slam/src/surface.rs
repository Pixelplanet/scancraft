//! The fused output surface: one surfel per occupied fusion voxel, with a confidence per surfel.
//!
//! One observation creates a surfel, so without further care every stray measurement becomes a
//! permanent point. Each surfel therefore carries a confidence (observations weighted by how
//! squarely the sensor saw the surface, times how well their normals agree), and only surfels that
//! are either confident on their own or *supported* by confident neighbours are part of the output.
//! A surface seen once is kept because its neighbours vouch for it; an isolated point is not.
//! Surfels that a later view sees through (free-space carving) are removed.
//!
//! Support is evaluated on a coarse grid (`coarse_voxels` fusion voxels per cell): a surfel's
//! support is the summed confidence of the 3 x 3 x 3 coarse cells around it. Visibility is kept up
//! to date incrementally: only cells whose content changed (and their neighbours) are re-evaluated.

use crate::voxel_map::Surfel;
use glam::Vec3;
use std::collections::{HashMap, HashSet};

/// Output filtering of the fused surface (see the module docs).
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceFilter {
    /// `false` outputs every surfel that has not been carved away.
    pub enabled: bool,
    /// Confidence at which a surfel is output regardless of its neighbours (about the number of
    /// consistent, head-on observations).
    pub min_confidence: f32,
    /// Summed confidence of the 3 x 3 x 3 coarse-cell neighbourhood at which a surfel is output.
    /// A flat patch seen once head-on fills that neighbourhood with ~140 surfels of confidence ~1.
    pub min_support: f32,
    /// Fusion voxels per coarse cell edge.
    pub coarse_voxels: i32,
}

impl Default for SurfaceFilter {
    fn default() -> Self {
        Self { enabled: true, min_confidence: 2.5, min_support: 24.0, coarse_voxels: 4 }
    }
}

#[derive(Debug, Clone, Copy)]
struct Info {
    /// Sum of |cos| of the angle between surface normal and viewing ray over all observations.
    view_weight: f32,
    /// Sum of the (sign-aligned) observed unit normals.
    normal_sum: Vec3,
    misses: u32,
    /// Confidence currently counted in the coarse cell.
    confidence: f32,
    key: [i32; 3],
    coarse: [i32; 3],
    carved: bool,
    visible: bool,
    reported: bool,
}

#[derive(Debug, Clone, Default)]
struct Coarse {
    members: Vec<u32>,
    confidence: f32,
}

/// What changed in the output surface since the consumer last asked
/// ([`Tracker::take_update`](crate::Tracker::take_update)).
#[derive(Debug, Clone)]
pub enum SurfaceUpdate {
    /// New points to add to the consumer's copy.
    Append(Vec<scancraft_core::Point3D>),
    /// Points were removed (noise carving, keyframe discard): replace the copy with these.
    Replace(Vec<scancraft_core::Point3D>),
}

#[derive(Debug, Clone)]
pub struct SurfaceModel {
    voxel_mm: f32,
    filter: SurfaceFilter,
    surfels: Vec<Surfel>,
    info: Vec<Info>,
    cells: HashMap<[i32; 3], u32>,
    coarse: HashMap<[i32; 3], Coarse>,
    dirty: HashSet<[i32; 3]>,
    /// Surfels that became visible and were not handed out yet.
    unreported: Vec<u32>,
    visible: usize,
    /// Increases whenever handed-out points are withdrawn or the model is cleared.
    revision: u64,
    /// `revision` at the last [`take_update`](Self::take_update).
    update_revision: u64,
    /// Handed-out points withdrawn since the consumer last took a full copy.
    withdrawn_pending: usize,
}

impl SurfaceModel {
    pub fn new(voxel_mm: f32, filter: SurfaceFilter) -> Self {
        Self {
            voxel_mm: voxel_mm.max(1e-3),
            filter,
            surfels: Vec::new(),
            info: Vec::new(),
            cells: HashMap::new(),
            coarse: HashMap::new(),
            dirty: HashSet::new(),
            unreported: Vec::new(),
            visible: 0,
            revision: 0,
            update_revision: 0,
            withdrawn_pending: 0,
        }
    }

    /// Points handed out (by [`take_new`](Self::take_new)) that have since been removed from the
    /// output, counted since the last full copy ([`take_update`](Self::take_update) returning
    /// `Replace`, or [`mark_all_reported`](Self::mark_all_reported)). Lets a consumer that shows
    /// the model live decide when a full refresh is worth it (e.g. above 1 % of its points, or
    /// when the user pauses). A cleared model counts as `usize::MAX`.
    pub fn withdrawn_pending(&self) -> usize {
        self.withdrawn_pending
    }

    /// Output surfels (visible after the last [`refresh`](Self::refresh)).
    pub fn len(&self) -> usize {
        self.visible
    }

    pub fn is_empty(&self) -> bool {
        self.visible == 0
    }

    /// All surfels ever created, carved and filtered ones included.
    pub fn stored(&self) -> usize {
        self.surfels.len()
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Removes everything. Points handed out so far are withdrawn (the revision increases) and
    /// what is fused next counts as already handed out until [`mark_all_reported`] says otherwise.
    /// Adapts the support neighbourhood to the sampling density of the source: coarse cells become
    /// at least four point spacings wide, so a sparsely sampled surface (low resolution, far away)
    /// finds as many neighbours as a densely sampled one. Ignored once surfels exist.
    pub fn set_point_spacing(&mut self, spacing_mm: f32) {
        if !self.surfels.is_empty() || !(spacing_mm > 0.0 && spacing_mm.is_finite()) {
            return;
        }
        let needed = (4.0 * spacing_mm / self.voxel_mm).ceil().min(64.0) as i32;
        self.filter.coarse_voxels = self.filter.coarse_voxels.max(needed);
    }

    pub fn clear(&mut self) {
        let filter = self.filter.clone();
        let (revision, update_revision) = (self.revision + 1, self.update_revision);
        *self = Self::new(self.voxel_mm, filter);
        self.revision = revision;
        self.update_revision = update_revision;
        self.withdrawn_pending = usize::MAX;
    }

    fn key(&self, p: Vec3) -> [i32; 3] {
        let k = (p / self.voxel_mm).floor();
        [k.x as i32, k.y as i32, k.z as i32]
    }

    fn coarse_key(&self, k: [i32; 3]) -> [i32; 3] {
        let c = self.filter.coarse_voxels.max(1);
        [k[0].div_euclid(c), k[1].div_euclid(c), k[2].div_euclid(c)]
    }

    /// Adds one observation (world frame); `view_cos` is |cos| of the angle between the observed
    /// normal and the viewing ray. Merges into the nearest surfel within one voxel size. Returns
    /// the index of a newly created surfel.
    pub fn insert(&mut self, position: Vec3, normal: Vec3, view_cos: f32) -> Option<usize> {
        let key = self.key(position);
        let [kx, ky, kz] = key;
        let mut best: Option<(f32, u32)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(&i) = self.cells.get(&[kx + dx, ky + dy, kz + dz]) {
                        let d = self.surfels[i as usize].position.distance_squared(position);
                        if best.is_none_or(|(bd, _)| d < bd) {
                            best = Some((d, i));
                        }
                    }
                }
            }
        }
        let target = match best {
            Some((d, i)) if d <= self.voxel_mm * self.voxel_mm => Some(i),
            _ => self.cells.get(&key).copied(),
        };
        if let Some(i) = target {
            self.merge(i as usize, position, normal, view_cos);
            return None;
        }
        let i = self.surfels.len();
        let coarse = self.coarse_key(key);
        self.cells.insert(key, i as u32);
        self.surfels.push(Surfel { position, normal, count: 1 });
        self.info.push(Info {
            view_weight: view_cos,
            normal_sum: normal,
            misses: 0,
            confidence: 0.0,
            key,
            coarse,
            carved: false,
            visible: false,
            reported: false,
        });
        self.coarse.entry(coarse).or_default().members.push(i as u32);
        self.update_confidence(i);
        Some(i)
    }

    fn merge(&mut self, i: usize, position: Vec3, normal: Vec3, view_cos: f32) {
        let s = &mut self.surfels[i];
        s.count += 1;
        let w = 1.0 / s.count as f32;
        s.position += (position - s.position) * w;
        let n = if normal.dot(s.normal) < 0.0 { -normal } else { normal };
        s.normal = (s.normal + (n - s.normal) * w).normalize_or(s.normal);
        let info = &mut self.info[i];
        info.view_weight += view_cos;
        info.normal_sum += n;
        self.update_confidence(i);
    }

    /// Observations weighted by viewing angle, times normal agreement (1 when all observations
    /// agree, towards 0 for scattered normals as noise produces).
    fn confidence_of(&self, i: usize) -> f32 {
        let info = &self.info[i];
        if info.carved {
            return 0.0;
        }
        let agreement = info.normal_sum.length() / self.surfels[i].count as f32;
        info.view_weight * agreement
    }

    fn update_confidence(&mut self, i: usize) {
        let c = self.confidence_of(i);
        let info = &mut self.info[i];
        let delta = c - info.confidence;
        info.confidence = c;
        let coarse = info.coarse;
        if let Some(cell) = self.coarse.get_mut(&coarse) {
            cell.confidence += delta;
        }
        self.dirty.insert(coarse);
    }

    /// Free-space carving with one view: surfels for which `seen_through` holds collect a miss and
    /// are removed once the misses reach their confidence (at least one). Returns the number
    /// removed.
    pub fn carve(&mut self, seen_through: impl FnMut(Vec3) -> bool) -> usize {
        self.carve_indices(0..self.surfels.len(), seen_through)
    }

    /// [`carve`](Self::carve) limited to surfels whose coarse cell overlaps the box `[min, max]`
    /// (e.g. the space one view saw). Equivalent to `carve` when nothing outside the box can be
    /// seen through, but costs only the surfels near the view instead of the whole model.
    pub fn carve_within(&mut self, min: Vec3, max: Vec3, seen_through: impl FnMut(Vec3) -> bool) -> usize {
        let size = self.voxel_mm * self.filter.coarse_voxels.max(1) as f32;
        let lo = (min / size).floor();
        let hi = (max / size).floor();
        let inside = |k: &[i32; 3]| {
            (0..3).all(|a| (k[a] as f32) >= lo[a] && (k[a] as f32) <= hi[a])
        };
        let mut indices: Vec<usize> =
            self.coarse.iter().filter(|(k, _)| inside(k)).flat_map(|(_, c)| c.members.iter().map(|&i| i as usize)).collect();
        indices.sort_unstable();
        self.carve_indices(indices.into_iter(), seen_through)
    }

    fn carve_indices(&mut self, indices: impl Iterator<Item = usize>, mut seen_through: impl FnMut(Vec3) -> bool) -> usize {
        let mut removed = 0;
        for i in indices {
            if self.info[i].carved || !seen_through(self.surfels[i].position) {
                continue;
            }
            self.info[i].misses += 1;
            if self.info[i].misses as f32 >= self.info[i].confidence.max(1.0) {
                self.info[i].carved = true;
                let key = self.info[i].key;
                if self.cells.get(&key) == Some(&(i as u32)) {
                    self.cells.remove(&key);
                }
                self.update_confidence(i);
                removed += 1;
            }
        }
        removed
    }

    /// Re-evaluates visibility where anything changed since the last call.
    pub fn refresh(&mut self) {
        if self.dirty.is_empty() {
            return;
        }
        let mut region: HashSet<[i32; 3]> = HashSet::with_capacity(self.dirty.len() * 4);
        for c in self.dirty.drain() {
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let n = [c[0] + dx, c[1] + dy, c[2] + dz];
                        if self.coarse.contains_key(&n) {
                            region.insert(n);
                        }
                    }
                }
            }
        }
        let mut withdrawn = false;
        for c in region {
            let mut support = 0.0;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(cell) = self.coarse.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                            support += cell.confidence;
                        }
                    }
                }
            }
            let cell = &self.coarse[&c];
            for &i in &cell.members {
                let info = &mut self.info[i as usize];
                let show = !info.carved
                    && (!self.filter.enabled
                        || info.confidence >= self.filter.min_confidence
                        || support >= self.filter.min_support);
                if show && !info.visible {
                    info.visible = true;
                    self.visible += 1;
                    if !info.reported {
                        self.unreported.push(i);
                    }
                } else if !show && info.visible {
                    info.visible = false;
                    self.visible -= 1;
                    // Stays `reported`: the consumer still holds it until it takes a full copy, so
                    // should it come back it must not be handed out twice.
                    if info.reported {
                        withdrawn = true;
                        self.withdrawn_pending = self.withdrawn_pending.saturating_add(1);
                    }
                }
            }
        }
        if withdrawn {
            self.revision += 1;
        }
    }

    /// The output surface (as of the last [`refresh`](Self::refresh)).
    pub fn points(&self) -> Vec<scancraft_core::Point3D> {
        self.surfels.iter().zip(&self.info).filter(|(_, i)| i.visible).map(|(s, _)| crate::surfel_point(s)).collect()
    }

    /// The output surfels with their observation counts (as of the last [`refresh`](Self::refresh)).
    pub fn output_surfels(&self) -> Vec<Surfel> {
        self.surfels.iter().zip(&self.info).filter(|(_, i)| i.visible).map(|(s, _)| *s).collect()
    }

    /// Output points not handed out yet; marks them handed out.
    pub fn take_new(&mut self) -> Vec<scancraft_core::Point3D> {
        let mut out = Vec::with_capacity(self.unreported.len());
        for i in self.unreported.drain(..) {
            let info = &mut self.info[i as usize];
            if info.visible && !info.reported {
                info.reported = true;
                out.push(crate::surfel_point(&self.surfels[i as usize]));
            }
        }
        out
    }

    /// Counts every current output point as handed out (after the consumer took a full copy).
    pub fn mark_all_reported(&mut self) {
        self.unreported.clear();
        self.withdrawn_pending = 0;
        for info in &mut self.info {
            info.reported = info.visible;
        }
    }

    /// Additions since the last call, or a full replacement if points were withdrawn meanwhile.
    pub fn take_update(&mut self) -> SurfaceUpdate {
        if self.revision != self.update_revision {
            self.update_revision = self.revision;
            self.mark_all_reported();
            SurfaceUpdate::Replace(self.points())
        } else {
            SurfaceUpdate::Append(self.take_new())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn patch(model: &mut SurfaceModel, origin: Vec3, n: i32, pitch: f32) {
        for i in 0..n {
            for j in 0..n {
                model.insert(origin + Vec3::new(i as f32 * pitch, 0.0, j as f32 * pitch), -Vec3::Y, 1.0);
            }
        }
    }

    #[test]
    fn isolated_single_observations_are_hidden_supported_ones_shown() {
        let mut m = SurfaceModel::new(0.5, SurfaceFilter::default());
        patch(&mut m, Vec3::new(0.0, 100.0, 0.0), 30, 0.6); // a surface seen once
        m.insert(Vec3::new(40.0, 80.0, 40.0), Vec3::X, 1.0); // a stray point
        m.insert(Vec3::new(-30.0, 120.0, 10.0), Vec3::Z, 0.7);
        m.refresh();
        assert_eq!(m.len(), 900, "the patch is shown, the strays are not");
        assert_eq!(m.take_new().len(), 900);

        // A stray point observed consistently several times becomes confident on its own.
        for _ in 0..3 {
            m.insert(Vec3::new(40.0, 80.0, 40.0), Vec3::X, 1.0);
        }
        m.refresh();
        assert_eq!(m.len(), 901);
        assert_eq!(m.take_new().len(), 1);
        assert_eq!(m.revision(), 0, "nothing handed out was withdrawn");
    }

    #[test]
    fn scattered_normals_lower_confidence() {
        let mut m = SurfaceModel::new(0.5, SurfaceFilter::default());
        for n in [Vec3::X, Vec3::Y, Vec3::Z] {
            m.insert(Vec3::new(1.1, 1.1, 1.1), n, 1.0);
        }
        m.refresh();
        assert_eq!(m.len(), 0, "three disagreeing observations are not a confident surface");
        for _ in 0..3 {
            m.insert(Vec3::new(5.1, 1.1, 1.1), Vec3::X, 1.0);
        }
        m.refresh();
        assert_eq!(m.len(), 1, "three agreeing ones are");
    }

    #[test]
    fn carving_withdraws_points_and_bumps_the_revision() {
        let mut m = SurfaceModel::new(0.5, SurfaceFilter::default());
        patch(&mut m, Vec3::new(0.0, 100.0, 0.0), 30, 0.6);
        m.refresh();
        assert!(matches!(m.take_update(), SurfaceUpdate::Append(p) if p.len() == 900));
        // A view sees through the left third of the patch.
        let removed = m.carve(|p| p.x < 5.9);
        m.refresh();
        assert_eq!(removed, 300);
        assert_eq!(m.len(), 600);
        assert_eq!(m.revision(), 1);
        match m.take_update() {
            SurfaceUpdate::Replace(p) => assert_eq!(p.len(), 600),
            other => panic!("expected a replacement, got {other:?}"),
        }
        assert!(matches!(m.take_update(), SurfaceUpdate::Append(p) if p.is_empty()));
        // The carved space can be observed again.
        assert!(m.insert(Vec3::new(1.0, 100.0, 1.0), -Vec3::Y, 1.0).is_some());
    }
}
