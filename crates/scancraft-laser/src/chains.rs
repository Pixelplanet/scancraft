//! Links line-centre samples into chains (one per visible stretch of a laser line) and maps them
//! into the rectified frame, where a chain is a function `u(v)` (the POP 4 lines run at 35-55°
//! from vertical, never horizontal, so every rectified row crosses a chain at most once).

use crate::steger::LinePoint;
use scancraft_core::RawCamera;

/// A linked line in rectified coordinates, sorted by `v`.
#[derive(Debug, Clone)]
pub struct Chain {
    /// Rectified (u, v) samples, strictly increasing `v`.
    pub points: Vec<(f32, f32)>,
    /// Mean `du/dv` in the rectified frame; its sign tells the two cross-line families apart.
    pub slope: f32,
}

impl Chain {
    pub fn v_range(&self) -> (f32, f32) {
        (self.points[0].1, self.points[self.points.len() - 1].1)
    }

    /// `u` at rectified row `v` by linear interpolation; `None` outside the chain or across a gap
    /// wider than `max_gap` rows.
    pub fn u_at(&self, v: f32, max_gap: f32) -> Option<f32> {
        let (v0, v1) = self.v_range();
        if v < v0 || v > v1 {
            return None;
        }
        let i = self.points.partition_point(|p| p.1 < v);
        if i == 0 {
            return Some(self.points[0].0);
        }
        let (a, b) = (self.points[i - 1], self.points[i.min(self.points.len() - 1)]);
        if b.1 - a.1 > max_gap {
            return None;
        }
        if b.1 <= a.1 {
            return Some(a.0);
        }
        Some(a.0 + (b.0 - a.0) * (v - a.1) / (b.1 - a.1))
    }
}

/// Union-find over sample indices.
struct Dsu(Vec<u32>);

impl Dsu {
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[self.0[x as usize] as usize];
            self.0[x as usize] = p;
            x = p;
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.0[ra as usize] = rb;
        }
    }
}

/// Links samples whose centres are within `link_px` of each other and whose tangents agree,
/// keeps chains with at least `min_points` samples, and rectifies them with `camera`
/// (`None`: the image is already rectified).
pub fn link(points: &[LinePoint], width: usize, height: usize, camera: Option<&RawCamera>, link_px: f32, min_points: usize) -> Vec<Chain> {
    if points.is_empty() {
        return Vec::new();
    }
    // Samples sit on integer pixels (one per pixel at most), so a pixel grid is a perfect hash.
    let mut grid = vec![u32::MAX; width * height];
    for (i, p) in points.iter().enumerate() {
        let (x, y) = (p.u.round() as usize, p.v.round() as usize);
        if x < width && y < height {
            grid[y * width + x] = i as u32;
        }
    }
    let mut dsu = Dsu((0..points.len() as u32).collect());
    let reach = link_px.ceil() as isize;
    for (i, p) in points.iter().enumerate() {
        let (x, y) = (p.u.round() as isize, p.v.round() as isize);
        for dy in 0..=reach {
            for dx in -reach..=reach {
                if dy == 0 && dx <= 0 {
                    continue;
                }
                let (nx, ny) = (x + dx, y + dy);
                if nx < 0 || ny < 0 || nx >= width as isize || ny >= height as isize {
                    continue;
                }
                let j = grid[ny as usize * width + nx as usize];
                if j == u32::MAX {
                    continue;
                }
                let q = &points[j as usize];
                let (du, dv) = (q.u - p.u, q.v - p.v);
                if du * du + dv * dv > link_px * link_px || (p.tu * q.tu + p.tv * q.tv).abs() < 0.9 {
                    continue;
                }
                dsu.union(i as u32, j);
            }
        }
    }
    let mut groups: std::collections::HashMap<u32, Vec<usize>> = std::collections::HashMap::new();
    for i in 0..points.len() {
        let r = dsu.find(i as u32);
        groups.entry(r).or_default().push(i);
    }
    let mut chains = Vec::new();
    for idx in groups.into_values() {
        if idx.len() < min_points {
            continue;
        }
        let mut pts: Vec<(f32, f32)> = idx
            .iter()
            .map(|&i| {
                let p = &points[i];
                match camera {
                    Some(c) => {
                        let (u, v) = c.rectify(p.u as f64, p.v as f64);
                        (u as f32, v as f32)
                    }
                    None => (p.u, p.v),
                }
            })
            .collect();
        pts.sort_by(|a, b| a.1.total_cmp(&b.1));
        // One sample per 0.5 rows: Steger may report two neighbouring pixels of a thick line.
        let mut merged: Vec<(f32, f32)> = Vec::with_capacity(pts.len());
        for p in pts {
            match merged.last_mut() {
                Some(last) if p.1 - last.1 < 0.5 => {
                    last.0 = 0.5 * (last.0 + p.0);
                    last.1 = 0.5 * (last.1 + p.1);
                }
                _ => merged.push(p),
            }
        }
        if merged.len() < min_points / 2 + 2 {
            continue;
        }
        let (first, last) = (merged[0], merged[merged.len() - 1]);
        let slope = (last.0 - first.0) / (last.1 - first.1).max(1e-3);
        chains.push(Chain { points: merged, slope });
    }
    chains.sort_by(|a, b| a.points[0].1.total_cmp(&b.points[0].1));
    chains
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(u0: f32, slope: f32, v0: usize, v1: usize) -> Vec<LinePoint> {
        let n = (1.0 + slope * slope).sqrt();
        (v0..v1)
            .map(|v| LinePoint { u: (u0 + slope * v as f32).round(), v: v as f32, tu: slope / n, tv: 1.0 / n, strength: 10.0, peak: 100.0 })
            .collect()
    }

    #[test]
    fn links_parallel_lines_separately() {
        let mut pts = line(20.0, 0.8, 0, 60);
        pts.extend(line(40.0, 0.8, 0, 60));
        pts.extend(line(120.0, -0.7, 10, 50));
        let chains = link(&pts, 128, 64, None, 1.6, 10);
        assert_eq!(chains.len(), 3);
        let neg = chains.iter().filter(|c| c.slope < 0.0).count();
        assert_eq!(neg, 1);
        let c = chains.iter().find(|c| c.slope > 0.0 && c.points[0].0 < 30.0).unwrap();
        assert!((c.u_at(30.0, 3.0).unwrap() - 44.0).abs() < 1.0);
        assert!(c.u_at(80.0, 3.0).is_none());
    }
}
