//! Fast Point Feature Histograms (Rusu et al., 2009): a 33-bin description of the local surface
//! shape around a point that does not depend on the point set's pose.

use glam::Vec3;
use rayon::prelude::*;

use crate::cloud::Index;
use crate::Oriented;

pub const BINS: usize = 11;
pub const DIM: usize = 3 * BINS;
pub type Feature = [f32; DIM];

/// Pair features of (source, target) in the Darboux frame: (alpha, phi, theta), PCL's convention.
fn pair_features(ps: Vec3, ns: Vec3, pt: Vec3, nt: Vec3) -> Option<(f32, f32, f32)> {
    let mut dp = pt - ps;
    let d = dp.length();
    if d < 1e-6 {
        return None;
    }
    dp /= d;
    let (mut n1, mut n2) = (ns, nt);
    let a1 = n1.dot(dp);
    let a2 = n2.dot(dp);
    let f3;
    if a1.abs().acos() > a2.abs().acos() {
        // Use the point whose normal makes the smaller angle with the line as source.
        std::mem::swap(&mut n1, &mut n2);
        dp = -dp;
        f3 = -a2;
    } else {
        f3 = a1;
    }
    let v = dp.cross(n1);
    let vl = v.length();
    if vl < 1e-6 {
        return None;
    }
    let v = v / vl;
    let w = n1.cross(v);
    let f2 = v.dot(n2);
    // atan2(0, 0) (perpendicular normals with v along n2) is undefined; call it 0.
    let (y, x) = (w.dot(n2), n1.dot(n2));
    let f1 = if y.abs() < 1e-3 && x.abs() < 1e-3 { 0.0 } else { y.atan2(x) };
    Some((f1, f2, f3))
}

fn bin(v: f32, lo: f32, hi: f32) -> usize {
    (((v - lo) / (hi - lo) * BINS as f32).floor() as isize).clamp(0, BINS as isize - 1) as usize
}

/// FPFH of every point, from neighbours within `radius`. Points with too few neighbours get an
/// all-zero feature (they match nothing well).
pub fn fpfh(points: &[Oriented], index: &Index, radius: f32) -> Vec<Feature> {
    let neighbours: Vec<Vec<(usize, f32)>> = points
        .par_iter()
        .enumerate()
        .map(|(i, (p, _))| index.within(*p, radius).into_iter().filter(|&j| j != i).map(|j| (j, (index.point(j) - *p).length())).collect())
        .collect();
    let spfh: Vec<Feature> = points
        .par_iter()
        .enumerate()
        .map(|(i, &(p, n))| {
            let mut h = [0.0f32; DIM];
            let mut count = 0usize;
            for &(j, _) in &neighbours[i] {
                let (q, m) = points[j];
                if let Some((f1, f2, f3)) = pair_features(p, n, q, m) {
                    h[bin(f1, -std::f32::consts::PI, std::f32::consts::PI)] += 1.0;
                    h[BINS + bin(f2, -1.0, 1.0)] += 1.0;
                    h[2 * BINS + bin(f3, -1.0, 1.0)] += 1.0;
                    count += 1;
                }
            }
            if count > 0 {
                let s = 100.0 / count as f32;
                h.iter_mut().for_each(|v| *v *= s);
            }
            h
        })
        .collect();
    (0..points.len())
        .into_par_iter()
        .map(|i| {
            let nb = &neighbours[i];
            if nb.len() < 5 {
                return [0.0; DIM];
            }
            let mut f = spfh[i];
            let k = nb.len() as f32;
            for &(j, d) in nb {
                let w = 1.0 / (k * d.max(1e-3));
                for (a, b) in f.iter_mut().zip(spfh[j].iter()) {
                    *a += w * b;
                }
            }
            // Each of the three histograms sums to 100.
            for part in f.chunks_mut(BINS) {
                let s: f32 = part.iter().sum();
                if s > 0.0 {
                    part.iter_mut().for_each(|v| *v *= 100.0 / s);
                }
            }
            f
        })
        .collect()
}

fn dist_sq(a: &Feature, b: &Feature) -> f32 {
    a.iter().zip(b.iter()).map(|(x, y)| (x - y) * (x - y)).sum()
}

/// For every feature in `a`, the index of the nearest feature in `b` (brute force; the clouds
/// are subsampled to a few thousand points).
pub fn nearest_features(a: &[Feature], b: &[Feature]) -> Vec<Option<usize>> {
    a.par_iter()
        .map(|fa| {
            if fa.iter().all(|v| *v == 0.0) {
                return None;
            }
            let mut best = (f32::MAX, None);
            for (j, fb) in b.iter().enumerate() {
                let d = dist_sq(fa, fb);
                if d < best.0 {
                    best = (d, Some(j));
                }
            }
            best.1
        })
        .collect()
}

/// Correspondences (source index, target index) by feature similarity: the mutual nearest
/// neighbours, or all source -> target nearest neighbours when too few are mutual.
pub fn correspondences(src: &[Feature], tgt: &[Feature], min_mutual: usize) -> Vec<(usize, usize)> {
    let forward = nearest_features(src, tgt);
    let backward = nearest_features(tgt, src);
    let all: Vec<(usize, usize)> = forward.iter().enumerate().filter_map(|(i, j)| j.map(|j| (i, j))).collect();
    let mutual: Vec<(usize, usize)> = all.iter().copied().filter(|&(i, j)| backward[j] == Some(i)).collect();
    if mutual.len() >= min_mutual {
        mutual
    } else {
        all
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    #[test]
    fn features_do_not_depend_on_pose() {
        let obj = crate::synthetic::object(1.5);
        let pts: Vec<Oriented> = obj.to_vec();
        let r = Quat::from_euler(glam::EulerRot::XYZ, 0.7, -1.1, 2.0);
        let t = Vec3::new(30.0, -100.0, 400.0);
        let moved: Vec<Oriented> = pts.iter().map(|&(p, n)| (r * p + t, r * n)).collect();
        let fa = fpfh(&pts, &Index::new(pts.iter().map(|p| p.0).collect()), 9.3);
        let fb = fpfh(&moved, &Index::new(moved.iter().map(|p| p.0).collect()), 9.3);
        // Neighbourhoods at the radius and rounding move a few points across bins; the bulk
        // must agree.
        let mut diffs: Vec<f32> = fa.iter().zip(fb.iter()).map(|(a, b)| dist_sq(a, b).sqrt()).collect();
        diffs.sort_by(f32::total_cmp);
        let median = diffs[diffs.len() / 2];
        let p90 = diffs[diffs.len() * 9 / 10];
        assert!(median < 1.0 && p90 < 10.0, "feature difference median {median}, 90th percentile {p90}");
        // And they are distinctive: features of different points differ by much more.
        let spread = (0..200).map(|i| dist_sq(&fa[i * 37 % fa.len()], &fa[(i * 91 + 13) % fa.len()]).sqrt()).sum::<f32>() / 200.0;
        assert!(spread > 10.0 * median.max(0.1), "spread {spread}");
    }
}
