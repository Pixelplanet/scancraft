//! Scan and tracking modes, following Revopoint's own model (see `docs/protocol/modes.md`).
//! Discriminants mirror Revo Scan's numeric values so logs and templates map one to one.
//! Other vendors map their modes onto the same vocabulary and mark the rest unsupported.

use serde::{Deserialize, Serialize};

/// What lights the scene and how depth is obtained (Revo `ScanSurfaceType`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LightSource {
    /// Fine coded structured light; depth computed on the device.
    FullFieldHd = 0,
    /// One blue laser line (deep holes, grooves); needs marker tracking.
    SingleLine = 1,
    /// Blue laser cross lines (shiny/dark objects); needs marker or colour tracking.
    CrossLines = 2,
    /// Invisible parallel infrared lines (not on every model).
    ParallelLines = 3,
    /// Dense infrared dot pattern; depth from host stereo matching. Longer range.
    VcselRapid = 4,
    /// Turntable-driven variant of full-field scanning.
    Turntable = 5,
    /// Both projectors combined; host stereo.
    HybridHd = 6,
}

impl LightSource {
    pub const ALL: [LightSource; 7] = [
        Self::FullFieldHd,
        Self::VcselRapid,
        Self::HybridHd,
        Self::CrossLines,
        Self::SingleLine,
        Self::ParallelLines,
        Self::Turntable,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::FullFieldHd => "Full-field HD",
            Self::SingleLine => "Single Line",
            Self::CrossLines => "Cross Lines",
            Self::ParallelLines => "Parallel Lines",
            Self::VcselRapid => "VCSEL Rapid",
            Self::Turntable => "Auto Turntable",
            Self::HybridHd => "Hybrid HD",
        }
    }
}

/// How consecutive frames are aligned (Revo `ScanMode`, "Tracking Mode").
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TrackingMode {
    /// Geometry: works on objects with shape features, not on plain planes or revolving bodies.
    Feature = 0,
    /// Retro-reflective marker dots on or around the object.
    Marker = 1,
    /// Two-step: scan a marker frame first, then the object against it (large objects).
    GlobalMarker = 2,
    /// Image texture/colour features from the RGB camera.
    Color = 3,
}

impl TrackingMode {
    pub const ALL: [TrackingMode; 4] = [Self::Feature, Self::Marker, Self::GlobalMarker, Self::Color];

    pub fn label(self) -> &'static str {
        match self {
            Self::Feature => "Feature",
            Self::Marker => "Marker",
            Self::GlobalMarker => "Global Marker",
            Self::Color => "Color",
        }
    }
}

/// Material / subject preset: exposure, gain, depth range and filtering tuned per object
/// (Revo `ScanObject`). Order after `General` follows Revo's string table (inferred).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ObjectType {
    General = 0,
    Face = 1,
    Head = 2,
    Body = 3,
    Dark = 4,
    Reflective = 5,
    Outdoor = 6,
    Large = 7,
    BodyLarge = 8,
}

impl ObjectType {
    pub const ALL: [ObjectType; 9] = [
        Self::General,
        Self::Dark,
        Self::Reflective,
        Self::Face,
        Self::Head,
        Self::Body,
        Self::BodyLarge,
        Self::Large,
        Self::Outdoor,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::General => "Normal",
            Self::Face => "Face",
            Self::Head => "Head / Hair",
            Self::Body => "Body",
            Self::Dark => "Dark / Black",
            Self::Reflective => "Shiny / Metallic",
            Self::Outdoor => "Outdoor",
            Self::Large => "Large Object",
            Self::BodyLarge => "Body (fast)",
        }
    }
}

/// Whether a device can use a mode in this build of ScanCraft.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Availability {
    Available,
    /// The hardware supports it but ScanCraft does not implement it yet.
    NotImplemented(String),
    /// The hardware does not have it; usually hidden in the UI.
    Unsupported,
}

impl Availability {
    pub fn is_available(&self) -> bool {
        matches!(self, Availability::Available)
    }
}

/// The modes a device offers, each with its availability, plus which combinations are valid.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ModeSupport {
    pub light_sources: Vec<(LightSource, Availability)>,
    pub tracking_modes: Vec<(TrackingMode, Availability)>,
    pub object_types: Vec<(ObjectType, Availability)>,
    /// Valid (light, tracking, object) combinations; empty means "any combination of available
    /// options".
    pub combinations: Vec<(LightSource, TrackingMode, ObjectType)>,
}

impl ModeSupport {
    fn lookup<T: PartialEq + Copy>(list: &[(T, Availability)], v: T) -> Availability {
        list.iter().find(|(x, _)| *x == v).map(|(_, a)| a.clone()).unwrap_or(Availability::Unsupported)
    }

    pub fn light(&self, v: LightSource) -> Availability {
        Self::lookup(&self.light_sources, v)
    }

    pub fn tracking(&self, v: TrackingMode) -> Availability {
        Self::lookup(&self.tracking_modes, v)
    }

    pub fn object(&self, v: ObjectType) -> Availability {
        Self::lookup(&self.object_types, v)
    }

    /// True if the device has this combination (regardless of implementation status).
    pub fn is_valid_combination(&self, light: LightSource, tracking: TrackingMode, object: ObjectType) -> bool {
        self.combinations.is_empty() || self.combinations.contains(&(light, tracking, object))
    }

    /// True if the combination exists and every part of it is implemented.
    pub fn can_scan(&self, light: LightSource, tracking: TrackingMode, object: ObjectType) -> bool {
        self.is_valid_combination(light, tracking, object)
            && self.light(light).is_available()
            && self.tracking(tracking).is_available()
            && self.object(object).is_available()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn availability_and_combinations() {
        let support = ModeSupport {
            light_sources: vec![
                (LightSource::FullFieldHd, Availability::Available),
                (LightSource::CrossLines, Availability::NotImplemented("needs markers".into())),
            ],
            tracking_modes: vec![(TrackingMode::Feature, Availability::Available)],
            object_types: vec![(ObjectType::General, Availability::Available), (ObjectType::Dark, Availability::Available)],
            combinations: vec![
                (LightSource::FullFieldHd, TrackingMode::Feature, ObjectType::General),
                (LightSource::FullFieldHd, TrackingMode::Feature, ObjectType::Dark),
            ],
        };
        assert!(support.can_scan(LightSource::FullFieldHd, TrackingMode::Feature, ObjectType::Dark));
        assert!(!support.can_scan(LightSource::CrossLines, TrackingMode::Feature, ObjectType::General));
        assert_eq!(support.light(LightSource::ParallelLines), Availability::Unsupported);
        assert!(!support.is_valid_combination(LightSource::FullFieldHd, TrackingMode::Feature, ObjectType::Face));
    }
}
