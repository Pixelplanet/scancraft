//! Normals for sparse line points: local PCA over neighbouring lines within a radius, oriented
//! towards the sensor (origin). A point without a neighbouring line gets the direction towards
//! the sensor made perpendicular to its own line.

use crate::planes::eigen_sym3;
use glam::Vec3;
use std::collections::HashMap;

/// `points`: (position in a sensor-centred frame, id of the line it belongs to).
pub fn estimate(points: &[(Vec3, usize)], radius: f32) -> Vec<Vec3> {
    let cell = radius.max(1e-3);
    let key = |p: Vec3| ((p.x / cell).floor() as i32, (p.y / cell).floor() as i32, (p.z / cell).floor() as i32);
    let mut grid: HashMap<(i32, i32, i32), Vec<u32>> = HashMap::new();
    for (i, (p, _)) in points.iter().enumerate() {
        grid.entry(key(*p)).or_default().push(i as u32);
    }
    let r2 = radius * radius;
    let mut out = Vec::with_capacity(points.len());
    let mut neigh: Vec<Vec3> = Vec::new();
    for (i, &(p, line)) in points.iter().enumerate() {
        neigh.clear();
        let mut other_line = false;
        let (kx, ky, kz) = key(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(ids) = grid.get(&(kx + dx, ky + dy, kz + dz)) {
                        for &j in ids {
                            let (q, ql) = points[j as usize];
                            if (q - p).length_squared() <= r2 {
                                neigh.push(q);
                                other_line |= ql != line;
                            }
                        }
                    }
                }
            }
        }
        let towards_sensor = -p.normalize_or_zero();
        let mut n = Vec3::ZERO;
        if other_line && neigh.len() >= 6 {
            let c = neigh.iter().copied().sum::<Vec3>() / neigh.len() as f32;
            let mut m = [[0f32; 3]; 3];
            for q in &neigh {
                let a = (*q - c).to_array();
                for r in 0..3 {
                    for s in 0..3 {
                        m[r][s] += a[r] * a[s];
                    }
                }
            }
            let (vals, vecs) = eigen_sym3(m);
            // Require a real 2-D spread (neighbouring lines), not a single straight line.
            if vals[1] > 4.0 * vals[0].max(1e-9) {
                n = vecs[0].normalize_or_zero();
            }
        }
        if n == Vec3::ZERO {
            // Tangent of the own line from the neighbours on it.
            let tangent = points
                .iter()
                .enumerate()
                .filter(|(j, (q, ql))| *j != i && *ql == line && (*q - p).length_squared() <= r2)
                .map(|(_, (q, _))| *q - p)
                .fold(Vec3::ZERO, |acc, d| acc + if d.dot(acc) < 0.0 { -d } else { d })
                .normalize_or_zero();
            n = (towards_sensor - tangent * towards_sensor.dot(tangent)).normalize_or_zero();
            if n == Vec3::ZERO {
                n = towards_sensor;
            }
        }
        if n.dot(towards_sensor) < 0.0 {
            n = -n;
        }
        out.push(n);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normals_of_lines_on_a_tilted_plane_face_the_sensor() {
        // Plane z = 250 + 0.3 x, lines every 8 mm along x.
        let mut pts = Vec::new();
        for line in 0..6 {
            let x0 = -20.0 + 8.0 * line as f32;
            for k in 0..80 {
                let y = -20.0 + 0.5 * k as f32;
                let x = x0 + 0.4 * y;
                pts.push((Vec3::new(x, y, 250.0 + 0.3 * x), line));
            }
        }
        let normals = estimate(&pts, 12.0);
        let expected = Vec3::new(0.3, 0.0, -1.0).normalize();
        for (n, (p, _)) in normals.iter().zip(&pts) {
            assert!(n.dot(-*p) > 0.0, "faces the sensor");
            assert!(n.dot(expected) > 0.99, "{n:?}");
        }
        // A single line: perpendicular to the line, towards the sensor.
        let single: Vec<(Vec3, usize)> = (0..40).map(|k| (Vec3::new(0.5 * k as f32, 0.0, 300.0), 0)).collect();
        for (n, (p, _)) in estimate(&single, 12.0).iter().zip(&single) {
            assert!(n.dot(-*p) > 0.0);
            assert!(n.x.abs() < 1e-3);
        }
    }
}
