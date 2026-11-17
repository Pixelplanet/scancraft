//! POP 4 mode matrix and per-object device presets, from Revo Scan 6's `deviceScanParam.json`
//! (model `0E1`) and its logged command sequences. See `docs/protocol/modes.md`.

use scancraft_driver::{Availability, LightSource, ModeSupport, ObjectType, TrackingMode};

/// Device settings Revo applies for Full-field HD, per object preset (`accuracymode` rows).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FullFieldPreset {
    /// IR exposure, µs (`0x911`).
    pub exposure_us: u32,
    /// Revo gain index 1..=10; sent as `0x903 gain*0x10` (encoding inferred from logged 1 -> 0x10).
    pub gain: u8,
    /// Depth window the device computes, mm (`0x707 zmin zmax`).
    pub z_range_mm: (u32, u32),
    /// Depth-engine threshold (`0x705`).
    pub threshold: u32,
    /// Multi-frame fusion on the device (`0xa04`), used for dark objects.
    pub multi_frame_fusion: bool,
}

pub fn full_field_preset(object: ObjectType) -> Option<FullFieldPreset> {
    let p = |exposure_us, gain, threshold, multi_frame_fusion| FullFieldPreset {
        exposure_us,
        gain,
        z_range_mm: (250, 500),
        threshold,
        multi_frame_fusion,
    };
    match object {
        ObjectType::General => Some(p(3000, 1, 10, false)),
        ObjectType::Dark => Some(p(5400, 4, 5, true)),
        ObjectType::Face => Some(p(4600, 2, 5, false)),
        _ => None,
    }
}

/// What the POP 4 offers, and what ScanCraft implements of it.
pub fn pop4_modes() -> ModeSupport {
    use LightSource::*;
    use ObjectType::*;
    use TrackingMode::*;
    let later = |why: &str| Availability::NotImplemented(why.to_string());

    // Combinations present for model 0E1 in Revo's parameter table.
    let mut combinations = Vec::new();
    for object in [General, Dark, Face] {
        for light in [FullFieldHd, VcselRapid, HybridHd] {
            for tracking in [Feature, Color] {
                combinations.push((light, tracking, object));
            }
        }
    }
    combinations.push((VcselRapid, Feature, Outdoor));
    combinations.push((VcselRapid, Color, Outdoor));
    for tracking in [Marker, GlobalMarker] {
        combinations.push((FullFieldHd, tracking, General));
        for light in [CrossLines, SingleLine] {
            for object in [General, Dark, Reflective, Outdoor] {
                combinations.push((light, tracking, object));
            }
        }
    }
    // Cross Lines + Color runs on the combined lines + dots stream (format 7), as in Revo Scan.
    for object in [General, Dark, Reflective, Outdoor] {
        combinations.push((CrossLines, Color, object));
    }

    ModeSupport {
        light_sources: vec![
            (FullFieldHd, Availability::Available),
            (VcselRapid, Availability::Available),
            (HybridHd, Availability::Available),
            // With Marker / Global Marker tracking (static scene verified 2026-11-10; markers are
            // detected in the raw laser frames and carry the pose).
            (CrossLines, Availability::Available),
            (SingleLine, Availability::Available),
            (ParallelLines, Availability::Unsupported),
            (Turntable, later("Turntable control (Bluetooth) not implemented yet")),
        ],
        tracking_modes: vec![
            (Feature, Availability::Available),
            // With Full-field HD (the only implemented light source they combine with).
            (Marker, Availability::Available),
            (GlobalMarker, Availability::Available),
            (Color, Availability::Available),
        ],
        object_types: vec![
            (General, Availability::Available),
            (Dark, Availability::Available),
            (Face, Availability::Available),
            (Reflective, later("Only with the blue laser lines")),
            (Outdoor, Availability::Available),
            (Head, Availability::Unsupported),
            (Body, Availability::Unsupported),
            (BodyLarge, Availability::Unsupported),
            (Large, Availability::Unsupported),
        ],
        combinations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn implemented_modes_are_scannable() {
        let m = pop4_modes();
        for object in [ObjectType::General, ObjectType::Dark, ObjectType::Face] {
            assert!(m.can_scan(LightSource::FullFieldHd, TrackingMode::Feature, object));
            assert!(m.can_scan(LightSource::FullFieldHd, TrackingMode::Color, object));
            assert!(full_field_preset(object).is_some());
        }
        assert!(m.can_scan(LightSource::CrossLines, TrackingMode::Marker, ObjectType::General));
        assert!(m.can_scan(LightSource::CrossLines, TrackingMode::Color, ObjectType::General));
        // Marker tracking with Full-field HD, General preset only (Revo's table).
        assert!(m.can_scan(LightSource::FullFieldHd, TrackingMode::Marker, ObjectType::General));
        assert!(m.can_scan(LightSource::FullFieldHd, TrackingMode::GlobalMarker, ObjectType::General));
        assert!(!m.can_scan(LightSource::FullFieldHd, TrackingMode::Marker, ObjectType::Dark));
        // Lasers are never combined with plain geometry tracking on the POP 4.
        assert!(!m.is_valid_combination(LightSource::CrossLines, TrackingMode::Feature, ObjectType::General));
        assert!(m.is_valid_combination(LightSource::CrossLines, TrackingMode::Marker, ObjectType::Reflective));
        assert_eq!(m.light(LightSource::ParallelLines), Availability::Unsupported);
        // VCSEL Rapid: General/Dark/Face/Outdoor (Outdoor only with VCSEL), host stereo on the
        // 640x1024 slot pair (vcsel.md).
        for object in [ObjectType::General, ObjectType::Dark, ObjectType::Face, ObjectType::Outdoor] {
            assert!(m.is_valid_combination(LightSource::VcselRapid, TrackingMode::Feature, object));
            assert!(m.can_scan(LightSource::VcselRapid, TrackingMode::Feature, object));
        }
        assert!(!m.can_scan(LightSource::FullFieldHd, TrackingMode::Feature, ObjectType::Outdoor));
    }
}
