//! Geometric object isolation: splits a scan into the object, the support it stands on (table,
//! turntable disc) and everything else (background, walls, clutter, marker blocks, detached
//! fragments).
//!
//! 1. Support plane: RANSAC on a voxel subsample. With an `up` hint only planes facing up count;
//!    without one the largest plane. The normal points to the object side.
//! 2. Points within `plane_band_mm` of the plane are support, points further below are other.
//! 3. The points above are split into connected components on a voxel grid.
//! 4. The object is the largest component that is not plane-like, or the components holding
//!    positive seeds. Components holding negative seeds are never object. Components within about
//!    `join_gap_mm` of the object join it; anything further away is a detached fragment.
//! 5. Band points where the object reaches down to the plane go back to the object (the lowest
//!    rows of its walls), and so do band points facing down under it (its base, seen in a merged
//!    or flipped scan). Table seen under an overhang stays support.
//! 6. A support plane extends beyond the object and has (almost) nothing below it. If hardly any
//!    band points lie outside the object's silhouette, or many points lie below the plane, the
//!    plane is a face of the object (for example an already isolated scan): steps 3 to 5 run
//!    again without a plane.

use crate::cluster::{cell_components, principal_spread, usable, voxel_key};
use crate::plane::{ransac_plane, refine_plane, NormalOrientation, Plane, PlaneSearch};
use glam::Vec3;
use scancraft_core::PointCloud;
use std::collections::{HashMap, HashSet};

/// What a point belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SegmentLabel {
    Object,
    /// On the support plane (table, turntable).
    Support,
    /// Background, clutter, below the support, detached fragments.
    Other,
}

/// The plane the object stands on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupportPlane {
    /// A point on the plane (centre of its inliers).
    pub point: Vec3,
    /// Unit normal, pointing to the object side.
    pub normal: Vec3,
}

impl SupportPlane {
    /// Signed height above the plane.
    pub fn height_of(&self, p: Vec3) -> f32 {
        self.normal.dot(p - self.point)
    }
}

/// Points the user marked: on the object (positive) or not on it (negative).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Seeds {
    pub positive: Vec<Vec3>,
    pub negative: Vec<Vec3>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SegmentConfig {
    /// Subsample grid of the plane search.
    pub plane_sample_mm: f32,
    pub ransac_iterations: usize,
    /// Inlier distance of the plane search.
    pub ransac_inlier_mm: f32,
    /// With normals: plane inliers need at least this |cos| between their normal and the plane's.
    pub normal_cos: f32,
    /// With an up hint: the support normal needs at least this component along it.
    pub min_up: f32,
    /// Points closer than this to the plane are support.
    pub plane_band_mm: f32,
    /// Grid of the connected-component search.
    pub cluster_voxel_mm: f32,
    /// Grid of the object footprint on the plane (step 5). Fine, so little table comes back.
    pub footprint_cell_mm: f32,
    /// Band points come back to the object where it reaches within this height of the plane.
    pub base_reach_mm: f32,
    /// Grid of the object silhouette on the plane (step 6).
    pub silhouette_cell_mm: f32,
    /// Smallest object component chosen without a seed.
    pub min_object_points: usize,
    /// A component is plane-like (a marker board, a wall patch) when its smallest spread is below
    /// max(`plane_like_ratio` * largest spread, `plane_like_mm`).
    pub plane_like_ratio: f32,
    pub plane_like_mm: f32,
    /// Components (not plane-like) within about this distance join the object; 0 disables.
    pub join_gap_mm: f32,
    /// A plane is a support only with at least this many band points outside the object's
    /// silhouette...
    pub min_support_points: usize,
    /// ...at least this share of all band points...
    pub min_support_share: f32,
    /// ...and at most this many points below the band per such band point.
    pub max_below_share: f32,
}

impl Default for SegmentConfig {
    fn default() -> Self {
        Self {
            plane_sample_mm: 3.0,
            ransac_iterations: 1500,
            ransac_inlier_mm: 1.0,
            normal_cos: 0.8,
            min_up: 0.5,
            plane_band_mm: 1.5,
            cluster_voxel_mm: 2.0,
            footprint_cell_mm: 1.5,
            base_reach_mm: 5.0,
            silhouette_cell_mm: 4.0,
            min_object_points: 200,
            plane_like_ratio: 0.02,
            plane_like_mm: 1.0,
            join_gap_mm: 6.0,
            min_support_points: 100,
            min_support_share: 0.2,
            max_below_share: 0.25,
        }
    }
}

/// Result of [`segment_object`].
#[derive(Debug, Clone, PartialEq)]
pub struct Segmentation {
    /// One label per input point.
    pub labels: Vec<SegmentLabel>,
    /// The support plane, if the scan has one.
    pub support: Option<SupportPlane>,
}

impl Segmentation {
    pub fn count(&self, label: SegmentLabel) -> usize {
        self.labels.iter().filter(|l| **l == label).count()
    }

    /// The items (points, normals, colours, ...) whose point has `label`.
    pub fn select<T: Clone>(&self, items: &[T], label: SegmentLabel) -> Vec<T> {
        items.iter().zip(&self.labels).filter(|(_, l)| **l == label).map(|(t, _)| t.clone()).collect()
    }
}

/// Labels every point as object, support or other (see the module docs). `normals` (unit or
/// zero, any sign) improve the plane search and keep a downward-facing base; `up_hint` is a
/// rough "up" direction in the points' frame. Without an object found no point is labelled
/// object.
pub fn segment_object(points: &[Vec3], normals: Option<&[Vec3]>, cfg: &SegmentConfig, seeds: Option<&Seeds>, up_hint: Option<Vec3>) -> Segmentation {
    let none = Seeds::default();
    let seeds = seeds.unwrap_or(&none);
    let normals = normals.filter(|n| n.len() == points.len());
    let up = up_hint.and_then(|u| u.try_normalize());
    if let Some(plane) = find_support(points, normals, cfg, up, seeds) {
        let split = split(points, normals, cfg, seeds, Some(plane));
        let outside = split.band_outside as f32;
        if split.band_outside >= cfg.min_support_points
            && outside >= cfg.min_support_share * split.band_points as f32
            && split.below_points as f32 <= cfg.max_below_share * outside
        {
            return split.segmentation;
        }
        log::debug!(
            "segment: plane is not a support ({} band points, {} outside the object, {} below)",
            split.band_points,
            split.band_outside,
            split.below_points
        );
    }
    split(points, normals, cfg, seeds, None).segmentation
}

/// [`segment_object`] on a point cloud, using its normals when they are set.
pub fn segment_cloud(cloud: &PointCloud, cfg: &SegmentConfig, seeds: Option<&Seeds>, up_hint: Option<Vec3>) -> Segmentation {
    let points: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from(p.position)).collect();
    let normals: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from(p.normal)).collect();
    let has_normals = normals.iter().any(|n| n.length_squared() > 0.25);
    segment_object(&points, has_normals.then_some(normals.as_slice()), cfg, seeds, up_hint)
}

/// Just the object points of `cloud` (see [`segment_object`]).
pub fn isolate_object(cloud: &PointCloud, cfg: &SegmentConfig, seeds: Option<&Seeds>, up_hint: Option<Vec3>) -> PointCloud {
    let seg = segment_cloud(cloud, cfg, seeds, up_hint);
    PointCloud::from_points(seg.select(&cloud.points, SegmentLabel::Object))
}

/// Indices of the first point in every `cell` mm voxel, in input order.
fn subsample(points: &[Vec3], cell: f32) -> Vec<usize> {
    let mut seen = HashSet::new();
    (0..points.len()).filter(|&i| usable(points[i]) && seen.insert(voxel_key(points[i], cell))).collect()
}

/// Step 1: the support plane candidate, oriented to the object side.
fn find_support(points: &[Vec3], normals: Option<&[Vec3]>, cfg: &SegmentConfig, up: Option<Vec3>, seeds: &Seeds) -> Option<SupportPlane> {
    let picked = subsample(points, cfg.plane_sample_mm);
    if picked.len() < 50 {
        return None;
    }
    let sample: Vec<Vec3> = picked.iter().map(|&i| points[i]).collect();
    let sample_normals: Option<Vec<Vec3>> = normals.map(|n| picked.iter().map(|&i| n[i]).collect());
    let search = PlaneSearch {
        iterations: cfg.ransac_iterations,
        inlier_mm: cfg.ransac_inlier_mm,
        normal_cos: cfg.normal_cos,
        orientation: up.map_or(NormalOrientation::Free, NormalOrientation::Along),
        prefer: up.map(|u| (u, cfg.min_up)),
        ..PlaneSearch::default()
    };
    let found = ransac_plane(&sample, sample_normals.as_deref(), &search)?;
    if up.is_some() && !found.preferred {
        return None;
    }
    let mut plane = refine_plane(&sample, found.plane, &[2.0, 1.5, 1.2]).unwrap_or(found.plane);
    let flip = if let Some(u) = up {
        plane.normal.dot(u) < 0.0
    } else if !seeds.positive.is_empty() {
        seeds.positive.iter().map(|p| plane.distance(*p)).sum::<f32>() < 0.0
    } else {
        // The object side holds more points well off the plane.
        let above = points.iter().filter(|p| usable(**p) && plane.distance(**p) > 5.0).count();
        let below = points.iter().filter(|p| usable(**p) && plane.distance(**p) < -5.0).count();
        below > above
    };
    if flip {
        plane = Plane { normal: -plane.normal, d: -plane.d };
    }
    let inliers: Vec<Vec3> = sample.iter().copied().filter(|p| plane.abs_distance(*p) <= cfg.plane_band_mm).collect();
    let centre = if inliers.is_empty() { -plane.normal * plane.d } else { inliers.iter().sum::<Vec3>() / inliers.len() as f32 };
    Some(SupportPlane { point: centre - plane.normal * plane.distance(centre), normal: plane.normal })
}

struct Split {
    segmentation: Segmentation,
    /// Points within the plane band.
    band_points: usize,
    /// Band points outside the object's silhouette.
    band_outside: usize,
    /// Points below the band.
    below_points: usize,
}

/// Steps 2 to 5 for one plane (or none).
fn split(points: &[Vec3], normals: Option<&[Vec3]>, cfg: &SegmentConfig, seeds: &Seeds, plane: Option<SupportPlane>) -> Split {
    let mut labels = vec![SegmentLabel::Other; points.len()];
    let (mut above, mut band, mut below_points) = (Vec::new(), Vec::new(), 0);
    for (i, p) in points.iter().enumerate().filter(|(_, p)| usable(**p)) {
        let h = plane.map_or(f32::INFINITY, |pl| pl.height_of(*p));
        if h > cfg.plane_band_mm {
            above.push(i);
        } else if h >= -cfg.plane_band_mm {
            band.push(i);
        } else {
            below_points += 1;
        }
    }

    let object = choose_object(points, &above, cfg, seeds);
    for &i in &object {
        labels[i] = SegmentLabel::Object;
    }

    let mut band_outside = 0;
    if let Some(pl) = plane {
        let u = pl.normal.any_orthonormal_vector();
        let v = pl.normal.cross(u);
        let key = |p: Vec3, cell: f32| [(u.dot(p) / cell).floor() as i32, (v.dot(p) / cell).floor() as i32];
        let fc = cfg.footprint_cell_mm;
        let footprint = fill_pinholes(object.iter().map(|&i| key(points[i], fc)).collect());
        let low = object.iter().filter(|&&i| pl.height_of(points[i]) <= cfg.base_reach_mm);
        let grounded = fill_pinholes(low.map(|&i| key(points[i], fc)).collect());
        let silhouette = filled_silhouette(object.iter().map(|&i| key(points[i], cfg.silhouette_cell_mm)).collect());
        for &i in &band {
            let k = key(points[i], fc);
            let facing_down = normals.is_some_and(|n| n[i].dot(pl.normal) <= -0.5);
            labels[i] = if grounded.contains(&k) || (facing_down && footprint.contains(&k)) {
                SegmentLabel::Object
            } else {
                SegmentLabel::Support
            };
            if !silhouette.contains(&key(points[i], cfg.silhouette_cell_mm)) {
                band_outside += 1;
            }
        }
    }
    Split { segmentation: Segmentation { labels, support: plane }, band_points: band.len(), band_outside, below_points }
}

/// The occupied cells grown by one cell, with every region they enclose filled.
fn filled_silhouette(cells: HashSet<[i32; 2]>) -> HashSet<[i32; 2]> {
    if cells.is_empty() {
        return cells;
    }
    let grown: HashSet<[i32; 2]> =
        cells.iter().flat_map(|c| (-1..=1).flat_map(move |dx| (-1..=1).map(move |dy| [c[0] + dx, c[1] + dy]))).collect();
    let (mut lo, mut hi) = ([i32::MAX; 2], [i32::MIN; 2]);
    for c in &grown {
        for k in 0..2 {
            lo[k] = lo[k].min(c[k] - 1);
            hi[k] = hi[k].max(c[k] + 1);
        }
    }
    // Flood the free cells from a corner of the padded bounding box; the rest is inside.
    let mut outside: HashSet<[i32; 2]> = HashSet::from([lo]);
    let mut stack = vec![lo];
    while let Some(c) = stack.pop() {
        for n in [[c[0] + 1, c[1]], [c[0] - 1, c[1]], [c[0], c[1] + 1], [c[0], c[1] - 1]] {
            let in_box = (lo[0]..=hi[0]).contains(&n[0]) && (lo[1]..=hi[1]).contains(&n[1]);
            if in_box && !grown.contains(&n) && outside.insert(n) {
                stack.push(n);
            }
        }
    }
    (lo[0]..=hi[0]).flat_map(|x| (lo[1]..=hi[1]).map(move |y| [x, y])).filter(|c| !outside.contains(c)).collect()
}

/// Adds the empty cells with at least 5 of their 8 neighbours occupied (holes between sparse
/// samples), without growing straight edges.
fn fill_pinholes(cells: HashSet<[i32; 2]>) -> HashSet<[i32; 2]> {
    let ring = |c: [i32; 2]| (-1..=1).flat_map(move |dx| (-1..=1).map(move |dy| [c[0] + dx, c[1] + dy])).filter(move |n| *n != c);
    let candidates: HashSet<[i32; 2]> = cells.iter().flat_map(|c| ring(*c)).filter(|n| !cells.contains(n)).collect();
    let fill: Vec<[i32; 2]> = candidates.into_iter().filter(|c| ring(*c).filter(|n| cells.contains(n)).count() >= 5).collect();
    let mut out = cells;
    out.extend(fill);
    out
}

/// Steps 3 and 4: indices (into `points`) of the object among the `above` points.
fn choose_object(points: &[Vec3], above: &[usize], cfg: &SegmentConfig, seeds: &Seeds) -> Vec<usize> {
    let cell = cfg.cluster_voxel_mm;
    let mut occupancy: HashMap<[i32; 3], usize> = HashMap::new();
    for &i in above {
        *occupancy.entry(voxel_key(points[i], cell)).or_default() += 1;
    }
    let comps = cell_components(&occupancy, 1, 1);
    let mut members: Vec<Vec<usize>> = vec![Vec::new(); comps.sizes.len()];
    for &i in above {
        members[comps.label[&voxel_key(points[i], cell)]].push(i);
    }
    let mut cells_of: Vec<Vec<[i32; 3]>> = vec![Vec::new(); comps.sizes.len()];
    for (c, &id) in &comps.label {
        cells_of[id].push(*c);
    }

    let seed_component = |s: &Vec3| nearest_component(&comps.label, *s, cell);
    let negative: HashSet<usize> = seeds.negative.iter().filter_map(seed_component).collect();
    let mut chosen: Vec<usize> = seeds.positive.iter().filter_map(seed_component).filter(|c| !negative.contains(c)).collect();
    chosen.sort_unstable();
    chosen.dedup();
    let plane_like = |c: usize| {
        let sd = principal_spread(members[c].iter().map(|&i| points[i]));
        sd[2] < (cfg.plane_like_ratio * sd[0]).max(cfg.plane_like_mm)
    };
    if chosen.is_empty() {
        let mut order: Vec<usize> = (0..comps.sizes.len()).collect();
        order.sort_by_key(|&c| (std::cmp::Reverse(comps.sizes[c]), c));
        chosen.extend(
            order
                .into_iter()
                .take_while(|&c| comps.sizes[c] >= cfg.min_object_points)
                .filter(|c| !negative.contains(c))
                .find(|&c| !plane_like(c)),
        );
    }
    if chosen.is_empty() {
        return Vec::new();
    }

    // Join nearby components: first on a coarse grid of the gap size, then cell by cell.
    if cfg.join_gap_mm > 0.0 {
        let reach = (cfg.join_gap_mm / cell).ceil() as i32;
        let coarse = cell * (2 * reach + 1) as f32;
        let to_coarse = |c: [i32; 3]| voxel_key((Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32) + 0.5) * cell, coarse);
        let object_cells: HashSet<[i32; 3]> = chosen.iter().flat_map(|&c| cells_of[c].iter().copied()).collect();
        let object_coarse: HashSet<[i32; 3]> = object_cells.iter().map(|c| to_coarse(*c)).collect();
        let near = |c: [i32; 3], set: &HashSet<[i32; 3]>, r: i32| {
            (-r..=r).any(|dx| (-r..=r).any(|dy| (-r..=r).any(|dz| set.contains(&[c[0] + dx, c[1] + dy, c[2] + dz]))))
        };
        let joined: Vec<usize> = (0..comps.sizes.len())
            .filter(|c| !chosen.contains(c) && !negative.contains(c))
            .filter(|&c| cells_of[c].iter().any(|cc| near(to_coarse(*cc), &object_coarse, 1) && near(*cc, &object_cells, reach)))
            .filter(|&c| !plane_like(c))
            .collect();
        chosen.extend(joined);
    }
    chosen.iter().flat_map(|&c| members[c].iter().copied()).collect()
}

/// Component of the labelled cell nearest to `p`, searching up to two cells away.
fn nearest_component(label: &HashMap<[i32; 3], usize>, p: Vec3, cell: f32) -> Option<usize> {
    let k = voxel_key(p, cell);
    let mut best: Option<(f32, usize)> = None;
    for dx in -2..=2 {
        for dy in -2..=2 {
            for dz in -2..=2 {
                let c = [k[0].saturating_add(dx), k[1].saturating_add(dy), k[2].saturating_add(dz)];
                if let Some(&id) = label.get(&c) {
                    let centre = (Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32) + 0.5) * cell;
                    let d = centre.distance_squared(p);
                    if best.is_none_or(|(d0, _)| d < d0) {
                        best = Some((d, id));
                    }
                }
            }
        }
    }
    best.map(|(_, id)| id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    /// Deterministic noise in [-1, 1).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
        }
    }

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Part {
        Object,
        Base,
        Table,
        Wall,
        Marker,
        Board,
        Clutter,
        Noise,
    }

    #[derive(Default)]
    struct Scene {
        points: Vec<Vec3>,
        normals: Vec<Vec3>,
        parts: Vec<Part>,
    }

    impl Scene {
        fn add(&mut self, part: Part, p: Vec3, n: Vec3) {
            self.points.push(p);
            self.normals.push(n);
            self.parts.push(part);
        }

        /// Rectangle `origin + s * a + t * b` (s, t in [0, 1]) sampled every `step` mm, with
        /// 0.2 mm noise along its normal; `skip` drops points (occlusion).
        #[allow(clippy::too_many_arguments)]
        fn rect(&mut self, part: Part, origin: Vec3, a: Vec3, b: Vec3, step: f32, rng: &mut Lcg, skip: impl Fn(Vec3) -> bool) {
            let n = a.cross(b).normalize();
            let (na, nb) = ((a.length() / step) as usize, (b.length() / step) as usize);
            for i in 0..=na {
                for j in 0..=nb {
                    let p = origin + a * (i as f32 / na as f32) + b * (j as f32 / nb as f32) + n * 0.2 * rng.next();
                    if !skip(p) {
                        self.add(part, p, n);
                    }
                }
            }
        }

        /// Open box (no bottom face) with its lowest corner at `min`.
        fn block(&mut self, part: Part, min: Vec3, size: Vec3, step: f32, rng: &mut Lcg) {
            let (x, y, z) = (Vec3::X * size.x, Vec3::Y * size.y, Vec3::Z * size.z);
            self.rect(part, min + z, x, y, step, rng, |_| false);
            self.rect(part, min, x, z, step, rng, |_| false);
            self.rect(part, min + y, x, z, step, rng, |_| false);
            self.rect(part, min, y, z, step, rng, |_| false);
            self.rect(part, min + x, y, z, step, rng, |_| false);
        }

        fn kept(&self, seg: &Segmentation, part: Part) -> f32 {
            let total = self.parts.iter().filter(|p| **p == part).count();
            let kept = self.parts.iter().zip(&seg.labels).filter(|(p, l)| **p == part && **l == SegmentLabel::Object).count();
            kept as f32 / total.max(1) as f32
        }

        fn transformed(mut self, r: Quat, t: Vec3) -> Self {
            for (p, n) in self.points.iter_mut().zip(self.normals.iter_mut()) {
                *p = r * *p + t;
                *n = r * *n;
            }
            self
        }
    }

    /// Dome object (half ellipsoid 80 x 60 x 50 mm) on a table (z = 0) in front of a wall, with a
    /// second small object, marker blocks, a marker board and floating noise around it.
    fn scene() -> Scene {
        let mut rng = Lcg(7);
        let mut s = Scene::default();
        let (rx, ry, rz) = (80.0f32, 60.0f32, 50.0f32);
        let under_dome = |p: Vec3| (p.x / rx).powi(2) + (p.y / ry).powi(2) < 1.0;
        s.rect(Part::Table, Vec3::new(-250.0, -250.0, 0.0), Vec3::X * 500.0, Vec3::Y * 450.0, 2.0, &mut rng, under_dome);
        s.rect(Part::Wall, Vec3::new(-250.0, 200.0, 0.0), Vec3::X * 500.0, Vec3::Z * 200.0, 2.0, &mut rng, |_| false);
        for i in 0..200 {
            let theta = (i as f32 + 0.5) / 200.0 * std::f32::consts::FRAC_PI_2; // from the top to the table
            for j in 0..400 {
                let phi = j as f32 / 400.0 * std::f32::consts::TAU;
                let dir = Vec3::new(theta.sin() * phi.cos(), theta.sin() * phi.sin(), theta.cos());
                let p = Vec3::new(rx * dir.x, ry * dir.y, rz * dir.z);
                let n = Vec3::new(dir.x / rx, dir.y / ry, dir.z / rz).normalize();
                if (i * 400 + j) % 3 == 0 {
                    s.add(Part::Object, p + n * 0.2 * rng.next(), n);
                }
            }
        }
        // Clutter object: a 30 x 30 x 40 mm block, and four 20 mm marker blocks.
        s.block(Part::Clutter, Vec3::new(140.0, -170.0, 0.0), Vec3::new(30.0, 30.0, 40.0), 1.0, &mut rng);
        for c in [Vec3::new(-130.0, 90.0, 0.0), Vec3::new(110.0, 90.0, 0.0), Vec3::new(-140.0, -120.0, 0.0), Vec3::new(30.0, -100.0, 0.0)] {
            s.block(Part::Marker, c, Vec3::splat(20.0), 1.0, &mut rng);
        }
        // A marker board lying on the table, 4 mm thick.
        s.rect(Part::Board, Vec3::new(-200.0, -40.0, 4.0), Vec3::X * 60.0, Vec3::Y * 80.0, 1.0, &mut rng, |_| false);
        for _ in 0..400 {
            let p = Vec3::new(rng.next() * 240.0, rng.next() * 190.0, 60.0 + rng.next().abs() * 100.0);
            s.add(Part::Noise, p, Vec3::Z);
        }
        s
    }

    fn assert_object_only(s: &Scene, seg: &Segmentation) {
        assert!(s.kept(seg, Part::Object) >= 0.98, "object kept {}", s.kept(seg, Part::Object));
        assert!(s.kept(seg, Part::Table) <= 0.01, "table kept {}", s.kept(seg, Part::Table));
        for part in [Part::Wall, Part::Marker, Part::Board, Part::Clutter] {
            assert_eq!(s.kept(seg, part), 0.0, "{part:?} kept");
        }
    }

    #[test]
    fn isolates_the_object_with_an_up_hint() {
        let s = scene();
        let seg = segment_object(&s.points, Some(&s.normals), &SegmentConfig::default(), None, Some(Vec3::Z));
        assert_object_only(&s, &seg);
        let support = seg.support.expect("table found");
        assert!(support.normal.dot(Vec3::Z) > 0.999, "{support:?}");
        assert!(support.height_of(Vec3::ZERO).abs() < 0.2, "{support:?}");
        // The table is support, the wall is other.
        let table_support = s.parts.iter().zip(&seg.labels).filter(|(p, l)| **p == Part::Table && **l == SegmentLabel::Support).count();
        assert!(table_support as f32 > 0.98 * s.parts.iter().filter(|p| **p == Part::Table).count() as f32);
    }

    #[test]
    fn isolates_the_object_in_any_frame_without_hints() {
        // Scanner coordinates: arbitrary rotation and offset, no normals, no up hint.
        let r = Quat::from_axis_angle(Vec3::new(0.3, -0.8, 0.5).normalize(), 2.1);
        let s = scene().transformed(r, Vec3::new(120.0, -40.0, 450.0));
        let seg = segment_object(&s.points, None, &SegmentConfig::default(), None, None);
        assert_object_only(&s, &seg);
        assert!(seg.support.unwrap().normal.dot(r * Vec3::Z) > 0.999);
    }

    #[test]
    fn invalid_point_markers_are_ignored() {
        // Some files mark invalid samples with FLT_MAX coordinates.
        let mut s = scene();
        let marker = s.points.len();
        for k in 0..50 {
            s.add(Part::Noise, Vec3::new(f32::MAX, if k % 2 == 0 { f32::MAX } else { -f32::MAX }, f32::MAX), Vec3::Z);
        }
        let seeds = Seeds { positive: vec![Vec3::splat(f32::MAX)], negative: vec![Vec3::new(-f32::MAX, 0.0, 0.0)] };
        let seg = segment_object(&s.points, Some(&s.normals), &SegmentConfig::default(), Some(&seeds), Some(Vec3::Z));
        assert!(seg.labels[marker..].iter().all(|l| *l == SegmentLabel::Other));
        assert_object_only(&s, &seg);
    }

    #[test]
    fn seeds_select_another_object() {
        let s = scene();
        let cfg = SegmentConfig::default();
        let seeds = Seeds { positive: vec![Vec3::new(155.0, -155.0, 40.0)], negative: Vec::new() };
        let seg = segment_object(&s.points, Some(&s.normals), &cfg, Some(&seeds), Some(Vec3::Z));
        assert!(s.kept(&seg, Part::Clutter) >= 0.98, "clutter kept {}", s.kept(&seg, Part::Clutter));
        assert_eq!(s.kept(&seg, Part::Object), 0.0);
        assert_eq!(s.kept(&seg, Part::Marker), 0.0);

        // A negative seed on the dome: the next largest object is taken instead.
        let seeds = Seeds { positive: Vec::new(), negative: vec![Vec3::new(0.0, 0.0, 50.0)] };
        let seg = segment_object(&s.points, Some(&s.normals), &cfg, Some(&seeds), Some(Vec3::Z));
        assert_eq!(s.kept(&seg, Part::Object), 0.0);
        assert!(s.kept(&seg, Part::Clutter) >= 0.98, "clutter kept {}", s.kept(&seg, Part::Clutter));
    }

    #[test]
    fn keeps_a_flat_base_on_the_table() {
        // A 60 x 40 x 30 mm box with its bottom face scanned (for example from a flipped pass)
        // lying flat on the table; the table is hidden under it.
        let mut rng = Lcg(3);
        let mut s = Scene::default();
        let inside = |p: Vec3| (0.0..=60.0).contains(&p.x) && (0.0..=40.0).contains(&p.y);
        s.rect(Part::Table, Vec3::new(-150.0, -150.0, 0.0), Vec3::X * 360.0, Vec3::Y * 340.0, 2.0, &mut rng, inside);
        s.block(Part::Object, Vec3::ZERO, Vec3::new(60.0, 40.0, 30.0), 1.0, &mut rng);
        s.rect(Part::Base, Vec3::new(0.0, 40.0, 0.0), Vec3::X * 60.0, -Vec3::Y * 40.0, 1.0, &mut rng, |_| false);
        let seg = segment_object(&s.points, Some(&s.normals), &SegmentConfig::default(), None, None);
        assert!(s.kept(&seg, Part::Base) >= 0.98, "base kept {}", s.kept(&seg, Part::Base));
        assert!(s.kept(&seg, Part::Object) >= 0.99, "box kept {}", s.kept(&seg, Part::Object));
        // Including the lowest rows of the walls, inside the plane band.
        let low = s.parts.iter().zip(&s.points).zip(&seg.labels).filter(|((p, q), _)| **p == Part::Object && q.z < 1.5);
        let (n, kept) = low.fold((0, 0), |(n, k), (_, l)| (n + 1, k + (*l == SegmentLabel::Object) as usize));
        assert!(n > 100 && kept as f32 >= 0.98 * n as f32, "{kept} of {n} wall-foot points kept");
        assert!(s.kept(&seg, Part::Table) <= 0.01, "table kept {}", s.kept(&seg, Part::Table));
    }

    #[test]
    fn table_under_an_overhang_stays_support() {
        // An L-shaped object: a post with an arm 40 mm above the table; the table shows under it.
        let mut rng = Lcg(9);
        let mut s = Scene::default();
        let under_post = |p: Vec3| (0.0..=20.0).contains(&p.x) && (0.0..=20.0).contains(&p.y);
        s.rect(Part::Table, Vec3::new(-150.0, -150.0, 0.0), Vec3::X * 360.0, Vec3::Y * 320.0, 2.0, &mut rng, under_post);
        s.block(Part::Object, Vec3::ZERO, Vec3::new(20.0, 20.0, 60.0), 1.0, &mut rng);
        s.block(Part::Object, Vec3::new(20.0, 0.0, 40.0), Vec3::new(80.0, 20.0, 10.0), 1.0, &mut rng);
        let seg = segment_object(&s.points, Some(&s.normals), &SegmentConfig::default(), None, None);
        assert!(s.kept(&seg, Part::Object) >= 0.98, "object kept {}", s.kept(&seg, Part::Object));
        assert!(s.kept(&seg, Part::Table) <= 0.01, "table kept {}", s.kept(&seg, Part::Table));
        let under_arm = s.points.iter().zip(&s.parts).zip(&seg.labels).filter(|((p, part), _)| {
            **part == Part::Table && (25.0..100.0).contains(&p.x) && (0.0..20.0).contains(&p.y)
        });
        let (n, support) = under_arm.fold((0, 0), |(n, k), (_, l)| (n + 1, k + (*l == SegmentLabel::Support) as usize));
        assert!(n > 300 && support == n, "{support} of {n} table points under the arm are support");
    }

    #[test]
    fn an_isolated_object_is_not_cut_by_its_own_face() {
        // An already cleaned scan: a box without any table. Its largest face is a plane, but not a
        // support.
        let mut rng = Lcg(5);
        let mut s = Scene::default();
        s.block(Part::Object, Vec3::ZERO, Vec3::new(80.0, 60.0, 30.0), 1.0, &mut rng);
        s.rect(Part::Object, Vec3::new(0.0, 60.0, 0.0), Vec3::X * 80.0, -Vec3::Y * 60.0, 1.0, &mut rng, |_| false);
        let seg = segment_object(&s.points, None, &SegmentConfig::default(), None, None);
        assert!(seg.support.is_none(), "{:?}", seg.support);
        assert_eq!(s.kept(&seg, Part::Object), 1.0);
    }

    #[test]
    fn detached_fragments_are_dropped_and_close_ones_joined() {
        let mut s = scene();
        let mut rng = Lcg(11);
        // A 10 mm piece 5 mm above the dome top (split off by a scan gap) and one 40 mm above
        // it (a ghost fragment).
        s.block(Part::Object, Vec3::new(-5.0, -5.0, 55.0), Vec3::splat(10.0), 0.5, &mut rng);
        s.block(Part::Noise, Vec3::new(-5.0, -5.0, 90.0), Vec3::splat(10.0), 0.5, &mut rng);
        let seg = segment_object(&s.points, None, &SegmentConfig::default(), None, Some(Vec3::Z));
        let piece = |z: f32| s.points.iter().zip(&seg.labels).filter(|(p, l)| p.z >= z && p.z < z + 11.0 && p.x.abs() < 6.0 && **l == SegmentLabel::Object).count();
        assert!(piece(54.0) > 400, "close piece joined: {}", piece(54.0));
        assert_eq!(piece(89.0), 0, "ghost fragment dropped");
        assert!(s.kept(&seg, Part::Object) >= 0.98);
        // Without joining, only the dome is kept.
        let cfg = SegmentConfig { join_gap_mm: 0.0, ..SegmentConfig::default() };
        let seg = segment_object(&s.points, None, &cfg, None, Some(Vec3::Z));
        let close = s.points.iter().zip(&seg.labels).filter(|(p, l)| p.z >= 54.0 && **l == SegmentLabel::Object).count();
        assert_eq!(close, 0);
    }

    #[test]
    fn select_and_isolate_return_object_points() {
        let s = scene();
        let cloud = PointCloud::from_points(
            s.points.iter().zip(&s.normals).map(|(p, n)| scancraft_core::Point3D::new(p.to_array()).with_normal(n.to_array())).collect(),
        );
        let object = isolate_object(&cloud, &SegmentConfig::default(), None, Some(Vec3::Z));
        let expected = segment_object(&s.points, Some(&s.normals), &SegmentConfig::default(), None, Some(Vec3::Z));
        assert_eq!(object.len(), expected.count(SegmentLabel::Object));
        assert_eq!(expected.select(&s.parts, SegmentLabel::Object).len(), object.len());
    }
}
