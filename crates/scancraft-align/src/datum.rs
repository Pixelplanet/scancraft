//! A coordinate frame from user-picked datums (the 3-2-1 scheme): a plane locks the world
//! plane it is mapped onto (its normal and the height), an axis locks an in-plane direction
//! (or the normal itself) and the remaining translation, an origin point fixes what is left.
//! All transforms are `new <- old`.

use glam::{Quat, Vec3};
use scancraft_core::Transform;

use crate::rigid;

/// A world coordinate plane the picked plane is mapped onto.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum WorldPlane {
    /// z = 0, normal +Z.
    #[default]
    XY,
    /// x = 0, normal +X.
    YZ,
    /// y = 0, normal +Y.
    ZX,
}

impl WorldPlane {
    /// The plane's unit normal.
    pub fn normal(&self) -> Vec3 {
        match self {
            WorldPlane::XY => Vec3::Z,
            WorldPlane::YZ => Vec3::X,
            WorldPlane::ZX => Vec3::Y,
        }
    }
}

/// A world axis the picked direction is mapped onto.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum WorldAxis {
    /// +X.
    #[default]
    X,
    /// +Y.
    Y,
    /// +Z.
    Z,
}

impl WorldAxis {
    /// The axis's unit direction.
    pub fn dir(&self) -> Vec3 {
        match self {
            WorldAxis::X => Vec3::X,
            WorldAxis::Y => Vec3::Y,
            WorldAxis::Z => Vec3::Z,
        }
    }
}

/// A picked plane (e.g. a fitted face) and the world plane it should become.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct PlaneDatum {
    /// A point on the plane.
    pub point: Vec3,
    /// The plane's unit normal.
    pub normal: Vec3,
    /// The world plane it maps onto.
    pub onto: WorldPlane,
    /// false: `normal` maps to the world plane's +normal; true: flipped.
    pub flip: bool,
}

/// A picked axis (edge direction, cylinder axis) and the world axis it should become.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct AxisDatum {
    /// A point on the axis line.
    pub point: Vec3,
    /// The axis's unit direction.
    pub dir: Vec3,
    /// The world axis it maps onto.
    pub onto: WorldAxis,
    /// false: `dir` maps to the world axis's +direction; true: flipped.
    pub flip: bool,
}

/// The picked datums: any subset of plane, axis and origin point.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Datums {
    /// Primary datum: fixes rotation fully and the coordinate along its normal.
    pub plane: Option<PlaneDatum>,
    /// Secondary datum: fixes the remaining rotation (and two more coordinates).
    pub axis: Option<AxisDatum>,
    /// Tertiary datum: a point that ends up at the remaining zero coordinates.
    pub origin: Option<Vec3>,
}

/// The frame built from the datums.
#[derive(Debug, Clone, Copy)]
pub struct DatumFrame {
    /// new <- old.
    pub transform: Transform,
    /// Plane + axis given: how far the picked axis was from the relation the datums demand
    /// (off square to the plane's normal, or off parallel when the axis maps to that normal);
    /// how much it was bent. None when no plane+axis pair exists.
    pub axis_off_deg: Option<f32>,
}

/// Why a datum frame could not be built.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum DatumError {
    /// No datum at all.
    #[error("pick at least one datum")]
    Empty,
    /// The datums contradict each other.
    #[error("the axis cannot go onto that world axis given the plane (they differ by less than 20 degrees from the demanded relation)")]
    Conflict,
}

/// Smallest rotation taking `from` to `to` (unit vectors).
fn shortest_arc(from: Vec3, to: Vec3) -> Quat {
    Quat::from_rotation_arc(from.normalize(), to.normalize())
}

/// Builds the `new <- old` transform the datums demand.
///
/// Rotation: the plane's (possibly flipped) normal goes exactly to the world plane's normal
/// (smallest rotation). With an axis as well, the axis direction — projected into the world
/// plane when the world axis lies in it — goes exactly to the world axis (rotation about the
/// plane normal); mapping the axis onto the plane's normal instead is only allowed when the
/// axis already lies within 20 degrees of the normal (it then fixes nothing rotationally),
/// and a projected axis shorter than sin(20 deg) is a Conflict. With only an axis it maps
/// straight onto the world axis.
///
/// Translation fixes only the coordinates the rotation leaves free, in this order: the plane
/// puts its coordinate along the world normal to 0; the axis puts the coordinates
/// perpendicular to the world axis to 0 (except one already fixed by the plane); the origin
/// point puts every still-free coordinate to 0. Coordinates no datum fixes keep whatever the
/// rotation produced (equivalently: the point `origin`, else the axis point, else the plane
/// point, keeps those of its rotated coordinates that remain free).
pub fn datum_frame(d: &Datums) -> Result<DatumFrame, DatumError> {
    if d.plane.is_none() && d.axis.is_none() && d.origin.is_none() {
        return Err(DatumError::Empty);
    }
    let sin20 = 20.0f32.to_radians().sin();
    // Rotation and the axis's residual error.
    let mut rot = Quat::IDENTITY;
    let mut axis_off_deg = None;
    if let Some(p) = &d.plane {
        let wn = p.onto.normal();
        let n = (if p.flip { -p.normal } else { p.normal }).normalize();
        rot = shortest_arc(n, wn);
        if let Some(a) = &d.axis {
            let wa = a.onto.dir();
            let ad = (if a.flip { -a.dir } else { a.dir }).normalize();
            let v = rot * ad;
            if wa == wn {
                // Parallel case: the axis only positions; it must lie near the normal.
                let off = v.angle_between(wa);
                if off > 20.0f32.to_radians() {
                    return Err(DatumError::Conflict);
                }
                axis_off_deg = Some(off.to_degrees());
            } else {
                // In-plane case: the projected direction goes to the world axis. The
                // projection must be long enough, and the residual error is how far the
                // axis is bent out of the plane (off square to the normal).
                let pv = v - wn * v.dot(wn);
                if pv.length() < sin20 {
                    return Err(DatumError::Conflict);
                }
                axis_off_deg = Some(v.dot(wn).abs().min(1.0).asin().to_degrees());
                let proj = pv.normalize();
                let turn = shortest_arc(proj, wa);
                // `turn` is about the normal (both vectors lie in the plane).
                rot = turn * rot;
            }
        }
    } else if let Some(a) = &d.axis {
        let ad = (if a.flip { -a.dir } else { a.dir }).normalize();
        rot = shortest_arc(ad, a.onto.dir());
    }
    // Translation, freeing coordinates as the datums take them.
    let mut free = [true; 3];
    let mut t = Vec3::ZERO;
    if let Some(p) = &d.plane {
        let wn = p.onto.normal();
        t += wn * -(rot * p.point).dot(wn);
        for i in 0..3 {
            if wn[i].abs() > 0.5 {
                free[i] = false;
            }
        }
    }
    if let Some(a) = &d.axis {
        let wa = a.onto.dir();
        let rp = rot * a.point;
        for i in 0..3 {
            if wa[i].abs() < 0.5 && free[i] {
                let mut e = Vec3::ZERO;
                e[i] = 1.0;
                t += e * -rp[i];
                free[i] = false;
            }
        }
    }
    if let Some(o) = &d.origin {
        let rp = rot * *o;
        for i in 0..3 {
            if free[i] {
                let mut e = Vec3::ZERO;
                e[i] = 1.0;
                t += e * -rp[i];
                free[i] = false;
            }
        }
    }
    Ok(DatumFrame { transform: rigid(rot, t), axis_off_deg })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cloud::Rng;
    use crate::rotation_angle;
    use scancraft_core::Transform;

    /// A box (top plane +Z at z = 40, a drilled hole along +Z) placed by a known rigid
    /// transform: the picked plane and axis in the placed frame.
    fn placed() -> (Transform, PlaneDatum, AxisDatum) {
        let mut rng = Rng::new(31);
        let q = Quat::from_rotation_y(0.4 + rng.unit() * 0.6) * Quat::from_rotation_z(-1.0 + rng.unit() * 2.0) * Quat::from_rotation_x(0.3);
        let place = rigid(q, Vec3::new(50.0 + rng.unit() * 40.0, -80.0 + rng.unit() * 60.0, 120.0 + rng.unit() * 30.0));
        let plane = PlaneDatum { point: place.transform_point3(Vec3::new(10.0, -15.0, 40.0)), normal: (place.rotation * Vec3::Z).normalize(), onto: WorldPlane::XY, flip: false };
        let axis = AxisDatum { point: place.transform_point3(Vec3::new(5.0, 8.0, 10.0)), dir: (place.rotation * Vec3::Z).normalize(), onto: WorldAxis::Z, flip: false };
        (place, plane, axis)
    }

    #[test]
    fn plane_axis_and_origin_rebuild_the_frame() {
        let (place, mut plane, mut axis) = placed();
        let origin = place.transform_point3(Vec3::new(-12.0, 20.0, 0.0));
        let frame = datum_frame(&Datums { plane: Some(plane), axis: Some(axis), origin: Some(origin) }).expect("frame");
        // The plane's points land on z = 0.
        for dp in [Vec3::ZERO, Vec3::X * 30.0, Vec3::Y * 20.0] {
            let p = frame.transform.transform_point3(plane.point + place.rotation * dp);
            assert!(p.z.abs() < 1e-3, "z {}", p.z);
        }
        // The axis line sits on x = y = 0.
        for s in [-10.0, 0.0, 25.0] {
            let p = frame.transform.transform_point3(axis.point + axis.dir * s);
            assert!(p.x.abs() < 1e-3 && p.y.abs() < 1e-3, "axis at {p}");
        }
        // Composed with the placement: a pure spin about Z (the axis onto Z fixes no
        // heading), no translation left.
        let both = crate::compose(&frame.transform, &place);
        assert!((both.rotation * Vec3::Z - Vec3::Z).length() < 1e-3, "composed keeps +Z");
        // The object origin is 40 mm under the top plane and keeps that (the axis already
        // fixed x,y and the plane z, so the origin datum pins nothing here).
        assert!((both.translation.z - (-40.0)).abs() < 1e-3, "composed translation {}", both.translation);
        // A hole 30 deg off the normal conflicts; 15 deg is accepted and reported.
        axis.dir = (place.rotation * (Vec3::Z + Vec3::X * 0.7)).normalize();
        assert_eq!(datum_frame(&Datums { plane: Some(plane), axis: Some(axis), origin: None }).unwrap_err(), DatumError::Conflict);
        axis.dir = (place.rotation * (Vec3::Z - Vec3::X * 0.28)).normalize();
        let bent = datum_frame(&Datums { plane: Some(plane), axis: Some(axis), origin: None }).expect("15 deg is fine");
        let off = bent.axis_off_deg.expect("off");
        eprintln!("axis off normal: {off:.2} deg (built ~{:.1})", 0.28f32.atan().to_degrees());
        assert!(off > 10.0 && off < 20.0);
        // Flipping the plane turns the part over.
        plane.flip = true;
        let flipped = datum_frame(&Datums { plane: Some(plane), axis: None, origin: None }).expect("frame");
        assert!((flipped.transform.rotation * plane.normal - (-Vec3::Z)).length() < 1e-4);
    }

    #[test]
    fn in_plane_axis_sets_the_heading() {
        let (place, plane, _) = placed();
        // A box edge lying in the top plane but bent 3 deg out of it (off square to the
        // normal), mapped onto +X.
        let dir = place.rotation * (Vec3::X + Vec3::Z * 3.0f32.to_radians().tan()).normalize();
        let axis = AxisDatum { point: place.transform_point3(Vec3::new(0.0, 0.0, 40.0)), dir, onto: WorldAxis::X, flip: false };
        let frame = datum_frame(&Datums { plane: Some(plane), axis: Some(axis), origin: None }).expect("frame");
        let off = frame.axis_off_deg.expect("off");
        eprintln!("edge off square: {off:.2} deg (3 expected)");
        assert!(off > 2.5 && off < 3.5, "off {off}");
        assert!((frame.transform.rotation * plane.normal - Vec3::Z).length() < 1e-4);
        let adir = frame.transform.rotation * dir;
        assert!(adir.y.abs() < 1e-4 && adir.x > 0.9, "axis dir {adir}");
        // Same on other variants: top -> ZX (normal +Y), edge -> X.
        let mut p2 = plane;
        p2.onto = WorldPlane::ZX;
        let frame2 = datum_frame(&Datums { plane: Some(p2), axis: Some(axis), origin: None }).expect("frame");
        assert!((frame2.transform.rotation * plane.normal - Vec3::Y).length() < 1e-4);
        let adir2 = frame2.transform.rotation * dir;
        assert!(adir2.z.abs() < 1e-4 && adir2.x > 0.9, "axis dir {adir2}");
        // An axis parallel to the plane's normal cannot go onto an in-plane axis.
        let bad = AxisDatum { point: Vec3::ZERO, dir: plane.normal, onto: WorldAxis::X, flip: false };
        assert_eq!(datum_frame(&Datums { plane: Some(plane), axis: Some(bad), origin: None }).unwrap_err(), DatumError::Conflict);
        let _ = rotation_angle(frame.transform.rotation, Quat::IDENTITY);
    }

    #[test]
    fn single_datums_and_every_variant() {
        let (place, plane, axis) = placed();
        // Plane only -> smallest rotation to each world normal + the coordinate fixed.
        for (onto, wn) in [(WorldPlane::XY, Vec3::Z), (WorldPlane::YZ, Vec3::X), (WorldPlane::ZX, Vec3::Y)] {
            let mut p = plane;
            p.onto = onto;
            let f = datum_frame(&Datums { plane: Some(p), axis: None, origin: None }).expect("frame");
            assert!((f.transform.rotation * plane.normal - wn).length() < 1e-4, "normal for {onto:?}");
            assert!(f.transform.transform_point3(plane.point).dot(wn).abs() < 1e-3);
            assert!(f.axis_off_deg.is_none());
            let extra = rotation_angle(f.transform.rotation, Quat::from_rotation_arc(plane.normal, wn)).to_degrees();
            assert!(extra < 0.01, "not the smallest rotation ({extra} deg)");
        }
        // Axis only -> smallest rotation onto each world axis, line passing through it.
        for (onto, wd) in [(WorldAxis::X, Vec3::X), (WorldAxis::Y, Vec3::Y), (WorldAxis::Z, Vec3::Z)] {
            let mut a = axis;
            a.onto = onto;
            let f = datum_frame(&Datums { plane: None, axis: Some(a), origin: None }).expect("frame");
            assert!((f.transform.rotation * axis.dir - wd).length() < 1e-4, "dir for {onto:?}");
            let rp = f.transform.transform_point3(axis.point);
            for i in 0..3 {
                if wd[i].abs() < 0.5 {
                    assert!(rp[i].abs() < 1e-3, "axis line on {onto:?}: {rp}");
                }
            }
        }
        // Flipped axis maps to -dir.
        let mut a = axis;
        a.onto = WorldAxis::Y;
        a.flip = true;
        let f = datum_frame(&Datums { plane: None, axis: Some(a), origin: None }).expect("frame");
        assert!((f.transform.rotation * axis.dir - (-Vec3::Y)).length() < 1e-4);
        // Origin only -> pure translation taking the point to 0.
        let o = place.transform_point3(Vec3::new(3.0, -7.0, 15.0));
        let f = datum_frame(&Datums { plane: None, axis: None, origin: Some(o) }).expect("frame");
        assert_eq!(f.transform.rotation, Quat::IDENTITY);
        assert!(f.transform.transform_point3(o).length() < 1e-3);
        assert_eq!(datum_frame(&Datums::default()).unwrap_err(), DatumError::Empty);
    }
}
