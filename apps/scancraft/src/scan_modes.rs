//! Scan mode, tracking mode and object type as the Scan settings offer them: which options a
//! device shows, which it can use now, and why the others are disabled (`docs/protocol/modes.md`).

use scancraft_driver::{Availability, LightSource, ModeSupport, ObjectType, TrackingMode};

/// One option as the settings show it.
#[derive(Clone, Debug, PartialEq)]
pub struct OptionState<T> {
    pub value: T,
    pub label: &'static str,
    /// `Some(reason)` when it cannot be picked now.
    pub disabled: Option<String>,
}

impl<T> OptionState<T> {
    pub fn enabled(&self) -> bool {
        self.disabled.is_none()
    }
}

/// Light sources in Revo Scan's order.
pub const LIGHTS: [LightSource; 7] = [
    LightSource::FullFieldHd,
    LightSource::VcselRapid,
    LightSource::HybridHd,
    LightSource::CrossLines,
    LightSource::SingleLine,
    LightSource::Turntable,
    LightSource::ParallelLines,
];

/// Tracking modes in Revo Scan's order.
pub const TRACKING: [TrackingMode; 4] = [TrackingMode::Feature, TrackingMode::Marker, TrackingMode::GlobalMarker, TrackingMode::Color];

/// Object types in Revo Scan's order.
pub const OBJECTS: [ObjectType; 9] = [
    ObjectType::General,
    ObjectType::Dark,
    ObjectType::Reflective,
    ObjectType::Face,
    ObjectType::Outdoor,
    ObjectType::Head,
    ObjectType::Body,
    ObjectType::BodyLarge,
    ObjectType::Large,
];

fn join_or(labels: &[&str]) -> String {
    match labels {
        [] => String::new(),
        [one] => one.to_string(),
        [rest @ .., last] => format!("{} or {last}", rest.join(", ")),
    }
}

/// The current choice of the three settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Choice {
    pub light: LightSource,
    pub tracking: TrackingMode,
    pub object: ObjectType,
}

/// The states of every option of the device (`None` when no scanner is connected: every option
/// is shown and none is judged).
pub struct Options {
    pub lights: Vec<OptionState<LightSource>>,
    pub tracking: Vec<OptionState<TrackingMode>>,
    pub objects: Vec<OptionState<ObjectType>>,
}

pub fn options(modes: Option<&ModeSupport>, c: Choice) -> Options {
    let Some(m) = modes else {
        return Options {
            lights: LIGHTS.iter().map(|&value| OptionState { value, label: value.label(), disabled: None }).collect(),
            tracking: TRACKING.iter().map(|&value| OptionState { value, label: value.label(), disabled: None }).collect(),
            objects: OBJECTS.iter().map(|&value| OptionState { value, label: value.label(), disabled: None }).collect(),
        };
    };
    let state = |availability: Availability| -> Option<Option<String>> {
        match availability {
            Availability::Unsupported => None,
            Availability::NotImplemented(why) => Some(Some(format!("Not available yet: {why}"))),
            Availability::Available => Some(None),
        }
    };
    let lights = LIGHTS
        .iter()
        .filter_map(|&value| {
            let disabled = state(m.light(value))?.or_else(|| {
                (!m.is_valid_combination(value, c.tracking, c.object)).then(|| {
                    let tracks: Vec<&str> = TRACKING.iter().filter(|&&t| m.is_valid_combination(value, t, c.object)).map(|t| t.label()).collect();
                    if !tracks.is_empty() {
                        format!("{} needs {} tracking", value.label(), join_or(&tracks))
                    } else {
                        let objects: Vec<&str> = OBJECTS.iter().filter(|&&o| m.is_valid_combination(value, c.tracking, o)).map(|o| o.label()).collect();
                        format!("{} is not offered with {} objects{}", value.label(), c.object.label(), if objects.is_empty() { String::new() } else { format!("; use {}", join_or(&objects)) })
                    }
                })
            });
            Some(OptionState { value, label: value.label(), disabled })
        })
        .collect();
    let tracking = TRACKING
        .iter()
        .filter_map(|&value| {
            let disabled = state(m.tracking(value))?.or_else(|| {
                (!m.is_valid_combination(c.light, value, c.object)).then(|| {
                    let lights: Vec<&str> = LIGHTS.iter().filter(|&&l| m.is_valid_combination(l, value, c.object)).map(|l| l.label()).collect();
                    if lights.is_empty() {
                        format!("{} tracking is not offered for {} objects", value.label(), c.object.label())
                    } else {
                        format!("{} tracking needs {}", value.label(), join_or(&lights))
                    }
                })
            });
            Some(OptionState { value, label: value.label(), disabled })
        })
        .collect();
    let objects = OBJECTS
        .iter()
        .filter_map(|&value| {
            let disabled = state(m.object(value))?.or_else(|| {
                (!m.is_valid_combination(c.light, c.tracking, value)).then(|| {
                    let lights: Vec<&str> = LIGHTS.iter().filter(|&&l| m.is_valid_combination(l, c.tracking, value)).map(|l| l.label()).collect();
                    if lights.is_empty() {
                        format!("Not offered with {} tracking", c.tracking.label())
                    } else {
                        format!("Needs {}", join_or(&lights))
                    }
                })
            });
            Some(OptionState { value, label: value.label(), disabled })
        })
        .collect();
    Options { lights, tracking, objects }
}

/// The first combination the device can scan with, preferring the current choice's parts.
pub fn reconcile(m: &ModeSupport, c: Choice) -> Choice {
    if m.can_scan(c.light, c.tracking, c.object) {
        return c;
    }
    let mut best: Option<(usize, Choice)> = None;
    for &light in &LIGHTS {
        for &tracking in &TRACKING {
            for &object in &OBJECTS {
                if !m.can_scan(light, tracking, object) {
                    continue;
                }
                // Keep as much of the current choice as possible.
                let kept = (light == c.light) as usize + (tracking == c.tracking) as usize + (object == c.object) as usize;
                if best.is_none_or(|(k, _)| kept > k) {
                    best = Some((kept, Choice { light, tracking, object }));
                }
            }
        }
    }
    best.map(|(_, c)| c).unwrap_or(c)
}

/// One line on what a light source does (Revo Scan's help, `modes.md` section 2).
pub fn light_help(l: LightSource) -> &'static str {
    match l {
        LightSource::FullFieldHd => "Fine structured-light pattern; depth is computed on the scanner. Markerless on objects with shape features.",
        LightSource::VcselRapid => "Dense infrared dot pattern with a longer range; depth comes from stereo matching on the computer.",
        LightSource::HybridHd => "Both projectors together for more detail; depth comes from stereo matching on the computer.",
        LightSource::CrossLines => "Blue laser cross lines for shiny metal and dark objects; used with Marker or Color tracking.",
        LightSource::SingleLine => "One blue laser line for deep holes and grooves; used with Marker tracking.",
        LightSource::ParallelLines => "Invisible parallel infrared laser lines.",
        LightSource::Turntable => "Full-field scanning driven by the automatic turntable.",
    }
}

/// One line on what a tracking mode does (`modes.md` section 3).
pub fn tracking_help(t: TrackingMode) -> &'static str {
    match t {
        TrackingMode::Feature => "Aligns frames by their geometry. Not for flat objects or plain revolving bodies.",
        TrackingMode::Marker => "Aligns frames by marker dots placed on or around the object: mechanical parts, planes, cylinders.",
        TrackingMode::GlobalMarker => "For large objects, in two steps: scan the markers into a marker frame, then the object against it.",
        TrackingMode::Color => "Aligns frames by image texture and colour from the RGB camera, for objects with distinct patterns.",
    }
}

/// One line on what an object preset is for (`modes.md` section 5).
pub fn object_help(o: ObjectType) -> &'static str {
    match o {
        ObjectType::General => "Everyday objects.",
        ObjectType::Dark => "Dark or black surfaces: longer exposure, more gain and multi-frame fusion.",
        ObjectType::Reflective => "Shiny or metallic surfaces; used with the blue laser lines.",
        ObjectType::Face => "People's faces, with settings suited to skin.",
        ObjectType::Outdoor => "Scanning in daylight; used with VCSEL Rapid or the laser lines and the outdoor filter.",
        ObjectType::Head => "Heads and hair.",
        ObjectType::Body => "Whole bodies.",
        ObjectType::BodyLarge => "Whole bodies at 18 frames per second.",
        ObjectType::Large => "Large objects.",
    }
}

/// The camera settings a preset starts from: exposure (µs), gain, depth window (mm). Known for
/// the POP 4's Full-field HD presets (Revo's table, mirrored by the driver).
pub fn preset(light: LightSource, object: ObjectType) -> Option<(u32, u8, (f32, f32))> {
    if light != LightSource::FullFieldHd {
        return None;
    }
    scancraft_driver_revopoint::modes::full_field_preset(object).map(|p| (p.exposure_us, p.gain, (p.z_range_mm.0 as f32, p.z_range_mm.1 as f32)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_driver_revopoint::modes::pop4_modes;

    fn find<T: PartialEq + Copy>(list: &[OptionState<T>], v: T) -> Option<&OptionState<T>> {
        list.iter().find(|o| o.value == v)
    }

    #[test]
    fn pop4_hides_unsupported_and_explains_the_rest() {
        let m = pop4_modes();
        let c = Choice { light: LightSource::FullFieldHd, tracking: TrackingMode::Feature, object: ObjectType::General };
        let o = options(Some(&m), c);
        assert!(find(&o.lights, LightSource::ParallelLines).is_none(), "unsupported light hidden");
        assert!(find(&o.objects, ObjectType::Head).is_none(), "unsupported object hidden");
        assert!(find(&o.lights, LightSource::FullFieldHd).unwrap().enabled());
        let hybrid = find(&o.lights, LightSource::HybridHd).unwrap();
        assert!(hybrid.disabled.as_ref().unwrap().starts_with("Not available yet"));
        assert!(find(&o.objects, ObjectType::Dark).unwrap().enabled());
        assert!(find(&o.tracking, TrackingMode::Color).unwrap().enabled());
        assert!(find(&o.tracking, TrackingMode::Marker).unwrap().enabled());
        // Revo's order.
        let labels: Vec<_> = o.lights.iter().map(|l| l.label).collect();
        assert_eq!(labels, ["Full-field HD", "VCSEL Rapid", "Hybrid HD", "Cross Lines", "Single Line", "Auto Turntable"]);
        let objects: Vec<_> = o.objects.iter().map(|l| l.label).collect();
        assert_eq!(objects, ["Normal", "Dark / Black", "Shiny / Metallic", "Face", "Outdoor"]);
    }

    #[test]
    fn invalid_combinations_get_a_hint() {
        // Every option implemented, so only the combination table decides.
        let mut m = pop4_modes();
        for (_, a) in m.light_sources.iter_mut().chain([]) {
            if !matches!(a, Availability::Unsupported) {
                *a = Availability::Available;
            }
        }
        for (_, a) in m.tracking_modes.iter_mut() {
            *a = Availability::Available;
        }
        let c = Choice { light: LightSource::FullFieldHd, tracking: TrackingMode::Feature, object: ObjectType::General };
        let o = options(Some(&m), c);
        let cross = find(&o.lights, LightSource::CrossLines).unwrap();
        assert_eq!(cross.disabled.as_deref(), Some("Cross Lines needs Marker, Global Marker or Color tracking"));
    }

    #[test]
    fn reconcile_falls_back_to_a_scannable_choice() {
        let m = pop4_modes();
        let bad = Choice { light: LightSource::CrossLines, tracking: TrackingMode::Marker, object: ObjectType::Dark };
        let c = reconcile(&m, bad);
        assert!(m.can_scan(c.light, c.tracking, c.object));
        assert_eq!(c.object, ObjectType::Dark, "keeps what it can");
    }

    #[test]
    fn full_field_presets_mirror_revo() {
        assert_eq!(preset(LightSource::FullFieldHd, ObjectType::General), Some((3000, 1, (250.0, 500.0))));
        assert_eq!(preset(LightSource::FullFieldHd, ObjectType::Dark), Some((5400, 4, (250.0, 500.0))));
        assert_eq!(preset(LightSource::VcselRapid, ObjectType::General), None);
    }
}
