//! The marker map of a pass (or a Global Marker library entry): marker positions in the pass /
//! world frame with observation counts and an "environment" / "on object" tag, JSON persistence,
//! and marker sets for aligning passes.

use crate::constellation::{register, MatchConfig, RigidMatch};
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::path::Path;
use std::sync::{Arc, Mutex};

/// Whether a marker moves with the object. Markers on blocks or the table around the object only
/// help tracking inside a pass (and same-position passes); after the object is flipped they no
/// longer match, so pass alignment must use `OnObject` markers only
/// (`docs/architecture/scan-sessions.md` section 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum MarkerTag {
    #[default]
    Environment,
    OnObject,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapMarker {
    /// Stable within one map.
    pub id: u32,
    /// World / pass frame, mm (+X right, +Y forward, +Z up of the pass's first frame).
    pub position: [f32; 3],
    /// Frames that contributed to the position.
    pub observations: u32,
    /// Mean estimated dot diameter, mm.
    #[serde(default)]
    pub diameter_mm: f32,
    #[serde(default)]
    pub tag: MarkerTag,
}

impl MapMarker {
    pub fn pos(&self) -> Vec3 {
        Vec3::from(self.position)
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MarkerMap {
    /// File format version.
    #[serde(default = "format_version")]
    pub version: u32,
    pub markers: Vec<MapMarker>,
    #[serde(default)]
    next_id: u32,
}

fn format_version() -> u32 {
    1
}

/// A map shared between the tracker (inside the pass pipeline) and the app.
pub type SharedMarkerMap = Arc<Mutex<MarkerMap>>;

impl MarkerMap {
    pub fn new() -> Self {
        Self { version: format_version(), markers: Vec::new(), next_id: 0 }
    }

    pub fn len(&self) -> usize {
        self.markers.len()
    }

    pub fn is_empty(&self) -> bool {
        self.markers.is_empty()
    }

    pub fn positions(&self) -> Vec<Vec3> {
        self.markers.iter().map(MapMarker::pos).collect()
    }

    pub fn get(&self, id: u32) -> Option<&MapMarker> {
        self.markers.iter().find(|m| m.id == id)
    }

    /// Adds a marker with one observation and returns its id.
    pub fn add(&mut self, position: Vec3, diameter_mm: f32) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        self.markers.push(MapMarker { id, position: position.to_array(), observations: 1, diameter_mm, tag: MarkerTag::default() });
        id
    }

    /// Folds a new observation into marker `index` (running mean, capped weight so a long pass
    /// still follows slow corrections).
    pub fn observe(&mut self, index: usize, position: Vec3, diameter_mm: f32) {
        let m = &mut self.markers[index];
        let n = m.observations.min(50) as f32;
        m.position = ((m.pos() * n + position) / (n + 1.0)).to_array();
        m.diameter_mm = (m.diameter_mm * n + diameter_mm) / (n + 1.0);
        m.observations += 1;
    }

    pub fn set_tag(&mut self, id: u32, tag: MarkerTag) -> bool {
        match self.markers.iter_mut().find(|m| m.id == id) {
            Some(m) => {
                m.tag = tag;
                true
            }
            None => false,
        }
    }

    pub fn set_all_tags(&mut self, tag: MarkerTag) {
        self.markers.iter_mut().for_each(|m| m.tag = tag);
    }

    /// Drops markers seen fewer than `min_observations` times (spurious detections).
    pub fn prune(&mut self, min_observations: u32) {
        self.markers.retain(|m| m.observations >= min_observations);
    }

    pub fn save_json(&self, path: &Path) -> std::io::Result<()> {
        let json = serde_json::to_string_pretty(self).map_err(std::io::Error::other)?;
        std::fs::write(path, json)
    }

    pub fn load_json(path: &Path) -> std::io::Result<Self> {
        let mut map: Self = serde_json::from_str(&std::fs::read_to_string(path)?).map_err(std::io::Error::other)?;
        map.next_id = map.markers.iter().map(|m| m.id + 1).max().unwrap_or(0).max(map.next_id);
        Ok(map)
    }

    /// Snapshot for pass alignment: positions in the pass frame plus tags.
    pub fn marker_set(&self) -> MarkerSet {
        MarkerSet { markers: self.markers.clone() }
    }
}

/// Which markers take part in a constellation alignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TagFilter {
    /// Markers rigidly attached to the object only: the right choice after the object moved.
    OnObject,
    /// Everything: valid while nothing moved (same-position passes).
    All,
}

/// The markers of one pass in that pass's frame. Owned by the pass; the alignment stage
/// registers sets of two passes with [`align_marker_sets`].
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MarkerSet {
    pub markers: Vec<MapMarker>,
}

impl MarkerSet {
    pub fn positions(&self, filter: TagFilter) -> Vec<Vec3> {
        self.filtered(filter).map(MapMarker::pos).collect()
    }

    pub fn filtered(&self, filter: TagFilter) -> impl Iterator<Item = &MapMarker> {
        self.markers.iter().filter(move |m| filter == TagFilter::All || m.tag == MarkerTag::OnObject)
    }
}

/// Rigid transform taking `source`'s pass frame into `target`'s (target <- source), from the
/// markers both observed. Returns the match with marker ids paired up, or `None` if fewer than
/// `min_inliers` markers agree. Uses triangle-signature matching, so no initial guess is needed.
pub fn align_marker_sets(source: &MarkerSet, target: &MarkerSet, filter: TagFilter, cfg: &MatchConfig) -> Option<MarkerSetAlignment> {
    let src: Vec<&MapMarker> = source.filtered(filter).collect();
    let tgt: Vec<&MapMarker> = target.filtered(filter).collect();
    let m = register(&src.iter().map(|m| m.pos()).collect::<Vec<_>>(), &tgt.iter().map(|m| m.pos()).collect::<Vec<_>>(), cfg, None)?;
    let id_pairs = m.pairs.iter().map(|&(s, t)| (src[s].id, tgt[t].id)).collect();
    Some(MarkerSetAlignment { id_pairs, matched: m })
}

#[derive(Debug, Clone, PartialEq)]
pub struct MarkerSetAlignment {
    /// (source marker id, target marker id).
    pub id_pairs: Vec<(u32, u32)>,
    pub matched: RigidMatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constellation::tests::{constellation, pose};
    use crate::constellation::rotation_delta;

    #[test]
    fn json_round_trip_keeps_ids_and_tags() {
        let mut map = MarkerMap::new();
        for p in constellation(4, 6) {
            map.add(p, 6.0);
        }
        map.set_tag(2, MarkerTag::OnObject);
        let path = std::env::temp_dir().join(format!("scancraft-markers-test-{}.json", std::process::id()));
        map.save_json(&path).unwrap();
        let mut back = MarkerMap::load_json(&path).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(back.markers, map.markers);
        assert_eq!(back.get(2).unwrap().tag, MarkerTag::OnObject);
        assert_eq!(back.add(Vec3::ZERO, 6.0), 6, "new ids continue after loaded ones");
    }

    #[test]
    fn aligns_passes_on_object_markers_only() {
        // Pass A: object markers + environment markers. Object flipped for pass B: object markers
        // moved rigidly, environment markers stayed where they were.
        let object = constellation(10, 8);
        let env = constellation(20, 10);
        let flip = pose(2.5, 0.2, 0.4, Vec3::new(10.0, 40.0, -30.0)); // B frame <- A frame for the object
        let mut a = MarkerSet::default();
        let mut b = MarkerSet::default();
        let mk = |id: u32, p: Vec3, tag| MapMarker { id, position: p.to_array(), observations: 5, diameter_mm: 6.0, tag };
        for (i, p) in object.iter().enumerate() {
            a.markers.push(mk(i as u32, *p, MarkerTag::OnObject));
            b.markers.push(mk(100 + i as u32, flip.transform_point3(*p), MarkerTag::OnObject));
        }
        for (i, p) in env.iter().enumerate() {
            a.markers.push(mk(50 + i as u32, *p, MarkerTag::Environment));
            b.markers.push(mk(150 + i as u32, *p, MarkerTag::Environment));
        }
        let r = align_marker_sets(&a, &b, TagFilter::OnObject, &MatchConfig::default()).expect("aligned");
        assert_eq!(r.id_pairs.len(), 8);
        assert!(r.id_pairs.iter().all(|&(s, t)| t == s + 100));
        assert!(rotation_delta(&r.matched.transform, &flip) < 1e-3);
        assert!((r.matched.transform.translation - flip.translation).length() < 0.05);
    }
}
