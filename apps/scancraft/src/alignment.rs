//! The work behind the Align stage's jobs, without any app state: Align All (every pass against
//! every other, `scancraft_align::align_all`, then a shell-aware joint refinement), Refine All as
//! a before/after comparison, and the per-pass quality read-outs.
//!
//! Passes come in as [`PassInput`]s in their own frames with their current project <- pass
//! transforms; results come back keyed by pass id.

use scancraft_align::quality::{moved, QualityConfig};
use scancraft_align::{compose, evaluate, Agreement, EdgeState, GraphConfig, PassCloud, Quality, RefineConfig, RefineInput, Rejection, Verdict};
use scancraft_core::Transform;

use crate::model::AlignCheck;

/// Grid the before/after and per-pass qualities are measured on, mm.
const MEASURE_SAMPLE_MM: f32 = 1.5;

/// Huber threshold of the joint refinement's pose graph, mm.
const REFINE_HUBER_MM: f32 = 1.0;

/// The verdict shown for an automatic alignment: Poor when the fit seen from the other pass
/// contradicts the data, at most Uncertain when aligning the other way round finds a different
/// fit.
pub fn checked_verdict(quality: &Quality, check: Option<&AlignCheck>) -> Verdict {
    let verdict = quality.verdict();
    match check {
        Some(c) if c.reverse.contradicted() => Verdict::Poor,
        Some(c) if c.agreement.is_some() && !c.consistent && verdict == Verdict::Good => Verdict::Fair,
        _ => verdict,
    }
}

/// One pass handed to a job.
pub struct PassInput {
    pub id: u32,
    /// project <- pass.
    pub pose: Transform,
    /// In the project frame already (reference or aligned).
    pub placed: bool,
    pub reference: bool,
    /// The pass in its own frame (its object only, unless the whole scene is used).
    pub cloud: PassCloud,
}

/// The settings both jobs use for the joint refinement: robust, and pairing the two faces of a
/// wall so inside and outside scans of a shell part hold each other.
pub fn refine_config() -> RefineConfig {
    RefineConfig { huber_mm: Some(REFINE_HUBER_MM), shell: true, ..RefineConfig::default() }
}

/// One number for how well a pair fits, mm: the same-side RMS where the two overlap, else the
/// spread of the wall between an inside and an outside scan (IQR / 1.349 = sigma for a normal
/// distribution). Infinite when neither can be measured.
pub fn fit_error(q: &Quality) -> f32 {
    if q.inliers >= 100 && q.rmse_mm.is_finite() {
        return q.rmse_mm;
    }
    match q.wall() {
        Some(w) => w.iqr_mm / 1.349,
        None => q.rmse_mm,
    }
}

/// Median of the finite values (`None` without any).
pub fn median(values: impl Iterator<Item = f32>) -> Option<f32> {
    let mut v: Vec<f32> = values.filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return None;
    }
    v.sort_by(f32::total_cmp);
    let n = v.len();
    Some(if n % 2 == 1 { v[n / 2] } else { 0.5 * (v[n / 2 - 1] + v[n / 2]) })
}

/// A linked pair before and after refinement.
#[derive(Debug, Clone)]
pub struct EdgeChange {
    pub a: u32,
    pub b: u32,
    /// `a` moved onto `b`, at the old and at the new poses.
    pub before: Quality,
    pub after: Quality,
    /// Wall thickness between the two, when they are an inside and an outside scan, mm.
    pub thickness_mm: Option<f32>,
}

impl EdgeChange {
    pub fn error_before(&self) -> f32 {
        fit_error(&self.before)
    }

    pub fn error_after(&self) -> f32 {
        fit_error(&self.after)
    }

    /// The pair's verdict dropped, or it newly contradicts the data.
    pub fn worse(&self) -> bool {
        let rank = |q: &Quality| match q.verdict() {
            Verdict::Good => 0,
            Verdict::Fair => 1,
            Verdict::Poor => 2,
        };
        rank(&self.after) > rank(&self.before) || (self.after.contradicted() && !self.before.contradicted())
    }
}

/// A joint refinement, to preview before it is accepted.
#[derive(Debug, Clone)]
pub struct RefineOutcome {
    /// New project <- pass transforms of the passes that moved.
    pub poses: Vec<(u32, Transform)>,
    pub edges: Vec<EdgeChange>,
    /// Each moved pass against the others at the new poses.
    pub qualities: Vec<(u32, Quality)>,
    pub seconds: f32,
}

impl RefineOutcome {
    pub fn median_before(&self) -> Option<f32> {
        median(self.edges.iter().map(EdgeChange::error_before))
    }

    pub fn median_after(&self) -> Option<f32> {
        median(self.edges.iter().map(EdgeChange::error_after))
    }

    /// The refinement made things worse and is rejected: the median pair error rose, or one pair
    /// got a worse verdict or now contradicts the data (pierces the other pass, lies below its
    /// table). The median alone lets a wall pair that got worse hide behind same-side pairs that
    /// got slightly better.
    pub fn worse(&self) -> bool {
        let median_rose = match (self.median_before(), self.median_after()) {
            (Some(b), Some(a)) => a > b * 1.01 + 0.005,
            (None, Some(_)) => false,
            (_, None) => true,
        };
        median_rose || self.edges.iter().any(EdgeChange::worse)
    }
}

/// Refines the placed passes jointly (the reference held) and measures every linked pair before
/// and after. `None` when no two passes overlap enough to be linked.
pub fn refine_passes(inputs: &[PassInput]) -> Option<RefineOutcome> {
    let t0 = std::time::Instant::now();
    let placed: Vec<&PassInput> = inputs.iter().filter(|p| p.placed).collect();
    // Placed passes are all in the project frame: without the reference, holding the first one
    // fixes the frame (otherwise the pose graph could drift as a whole).
    let has_reference = placed.iter().any(|p| p.reference);
    let refine: Vec<RefineInput> = placed.iter().enumerate().map(|(k, p)| RefineInput { cloud: &p.cloud, pose: p.pose, fixed: p.reference || (!has_reference && k == 0) }).collect();
    let result = scancraft_align::refine_global(&refine, &refine_config());
    if result.edges.is_empty() {
        return None;
    }
    let old: Vec<Transform> = placed.iter().map(|p| p.pose).collect();
    let edges = result
        .edges
        .iter()
        .map(|e| EdgeChange {
            a: placed[e.a].id,
            b: placed[e.b].id,
            before: pair_quality(&placed[e.a].cloud, &placed[e.b].cloud, &old[e.a], &old[e.b]),
            after: pair_quality(&placed[e.a].cloud, &placed[e.b].cloud, &result.poses[e.a], &result.poses[e.b]),
            thickness_mm: e.thickness_mm,
        })
        .collect();
    let poses: Vec<(u32, Transform)> = placed.iter().zip(&refine).zip(&result.poses).filter(|((_, r), _)| !r.fixed).map(|((p, _), t)| (p.id, *t)).collect();
    let clouds: Vec<(u32, &PassCloud, Option<Transform>)> = placed.iter().zip(&result.poses).map(|(p, t)| (p.id, &p.cloud, Some(*t))).collect();
    let qualities = pass_qualities(&clouds).into_iter().filter(|(id, _)| poses.iter().any(|(p, _)| p == id)).collect();
    Some(RefineOutcome { poses, edges, qualities, seconds: t0.elapsed().as_secs_f32() })
}

/// `a` moved onto `b`, both placed in the project frame by their poses.
fn pair_quality(a: &PassCloud, b: &PassCloud, pose_a: &Transform, pose_b: &Transform) -> Quality {
    let relative = compose(&pose_b.inverse(), pose_a);
    evaluate(a, b, &relative, QualityConfig::for_sample(MEASURE_SAMPLE_MM))
}

/// Each placed pass (pose `Some`) against the union of the other placed ones, in the project
/// frame. The union carries a table plane only when it is a single pass.
///
/// Every pass is moved into the project frame once, already on the measuring grid, and the
/// passes are scored one after the other (each evaluation is parallel inside), so only one union
/// exists at a time instead of a full copy of every other pass per pass.
pub fn pass_qualities(passes: &[(u32, &PassCloud, Option<Transform>)]) -> Vec<(u32, Quality)> {
    use rayon::prelude::*;
    let placed: Vec<(u32, &PassCloud, Transform)> = passes.iter().filter_map(|(id, c, t)| t.map(|t| (*id, *c, t))).collect();
    if placed.len() < 2 {
        return Vec::new();
    }
    let config = QualityConfig::for_sample(MEASURE_SAMPLE_MM);
    let sampled: Vec<PassCloud> = placed
        .par_iter()
        .map(|(_, c, t)| {
            let points = scancraft_align::cloud::downsample(&c.points, config.sample_mm);
            moved(&PassCloud { points, views: c.views.clone(), support: c.support }, t)
        })
        .collect();
    placed
        .iter()
        .enumerate()
        .map(|(i, (id, cloud, pose))| {
            let others: Vec<&PassCloud> = sampled.iter().enumerate().filter(|(j, _)| *j != i).map(|(_, c)| c).collect();
            let mut model = PassCloud { support: if others.len() == 1 { others[0].support } else { None }, ..PassCloud::default() };
            for o in &others {
                model.points.extend_from_slice(&o.points);
                model.views.extend(o.views.iter().cloned());
            }
            (*id, evaluate(cloud, &model, pose, config))
        })
        .collect()
}

/// What became of one pair in Align All.
#[derive(Debug, Clone, PartialEq)]
pub enum EdgeKind {
    /// Measured directly and used to place a pass.
    Placed,
    /// Measured directly; kept, and it agrees with the placement.
    Confirmed,
    /// Measured, but not used.
    Rejected(String),
    /// Not measured usefully; follows from the placements through `via`.
    Composed { via: Vec<u32> },
}

/// One pair in the Align All report.
#[derive(Debug, Clone)]
pub struct EdgeReport {
    pub from: u32,
    pub to: u32,
    pub kind: EdgeKind,
    /// `from` moved onto `to`; `None` when nothing was found or a composed pair was not scored.
    pub quality: Option<Quality>,
    pub verdict: Option<Verdict>,
    /// Aligning the pair the other way round, against the result.
    pub agreement: Option<Agreement>,
}

/// Where Align All puts one pass.
#[derive(Debug, Clone)]
pub struct Placement {
    pub id: u32,
    /// project <- pass; `None` when it could not be connected to the reference.
    pub pose: Option<Transform>,
    /// Against the other placed passes at the final poses.
    pub quality: Option<Quality>,
    /// The pass it was placed from.
    pub parent: Option<u32>,
    pub reference: bool,
}

/// The result of Align All, to preview before it is accepted.
#[derive(Debug, Clone)]
pub struct AlignAllOutcome {
    pub placements: Vec<Placement>,
    pub edges: Vec<EdgeReport>,
    /// The joint refinement after the graph: kept, or dropped because it made things worse.
    pub refine: Option<RefineSummary>,
    pub seconds: f32,
}

/// How the joint refinement inside Align All went.
#[derive(Debug, Clone, Copy)]
pub struct RefineSummary {
    pub before_mm: Option<f32>,
    pub after_mm: Option<f32>,
    pub kept: bool,
}

impl AlignAllOutcome {
    /// Passes that get a (new) place: connected and not the reference.
    pub fn moved(&self) -> impl Iterator<Item = &Placement> {
        self.placements.iter().filter(|p| p.pose.is_some() && !p.reference)
    }

    pub fn unconnected(&self) -> impl Iterator<Item = &Placement> {
        self.placements.iter().filter(|p| p.pose.is_none())
    }
}

/// Aligns all passes at once: every pair measured, a spanning tree from the reference, missing
/// pairs composed and cycles checked, then a joint shell-aware refinement that is kept only when
/// it lowers the median pair error. Passes need not be placed beforehand; the reference's current
/// pose is kept.
///
/// Without the reference among `inputs` nothing is placed: placing the passes in the frame of
/// another one would mark them aligned in a frame that is not the project's.
pub fn align_all_passes(inputs: &[PassInput], config: &GraphConfig) -> AlignAllOutcome {
    let t0 = std::time::Instant::now();
    let Some(reference) = inputs.iter().position(|p| p.reference) else {
        log::warn!("align all: the reference pass is not among the {} inputs", inputs.len());
        let placements = inputs.iter().map(|p| Placement { id: p.id, pose: None, quality: None, parent: None, reference: false }).collect();
        return AlignAllOutcome { placements, edges: Vec::new(), refine: None, seconds: t0.elapsed().as_secs_f32() };
    };
    let clouds: Vec<PassCloud> = inputs.iter().map(|p| p.cloud.clone()).collect();
    let graph = scancraft_align::align_all(&clouds, reference, config);
    let base = inputs.get(reference).map(|p| p.pose).unwrap_or_default();
    let mut poses: Vec<Option<Transform>> = graph.poses.iter().map(|p| p.map(|t| compose(&base, &t))).collect();
    let id = |k: usize| inputs[k].id;

    // Joint refinement of the connected passes; kept when it helps.
    let refine = {
        let connected: Vec<PassInput> = inputs
            .iter()
            .zip(&poses)
            .filter_map(|(p, t)| t.map(|t| PassInput { id: p.id, pose: t, placed: true, reference: p.reference, cloud: p.cloud.clone() }))
            .collect();
        refine_passes(&connected).map(|r| {
            let kept = !r.worse();
            if kept {
                for (pid, t) in &r.poses {
                    if let Some(k) = inputs.iter().position(|p| p.id == *pid) {
                        poses[k] = Some(*t);
                    }
                }
            }
            RefineSummary { before_mm: r.median_before(), after_mm: r.median_after(), kept }
        })
    };

    let scored: Vec<(u32, &PassCloud, Option<Transform>)> = inputs.iter().zip(&poses).map(|(p, t)| (p.id, &p.cloud, *t)).collect();
    let qualities = pass_qualities(&scored);
    let placements = inputs
        .iter()
        .enumerate()
        .map(|(k, p)| Placement {
            id: p.id,
            pose: poses[k],
            quality: qualities.iter().find(|(q, _)| *q == p.id).map(|(_, q)| *q),
            parent: graph.parents[k].map(id),
            reference: k == reference,
        })
        .collect();
    let mut edges: Vec<EdgeReport> = graph
        .edges
        .iter()
        .map(|e| {
            let kind = match &e.state {
                EdgeState::Tree => EdgeKind::Placed,
                EdgeState::Kept => EdgeKind::Confirmed,
                EdgeState::Unreached => EdgeKind::Rejected("not connected to the reference".into()),
                EdgeState::Rejected(r) => EdgeKind::Rejected(rejection_label(r)),
            };
            let found = e.transform.is_some();
            EdgeReport { from: id(e.from), to: id(e.to), kind, quality: found.then_some(e.quality), verdict: found.then_some(e.verdict), agreement: e.agreement }
        })
        .collect();
    for c in &graph.composed {
        let via = c.path.iter().map(|&k| id(k)).collect();
        let report = EdgeReport { from: id(c.from), to: id(c.to), kind: EdgeKind::Composed { via }, quality: c.quality, verdict: c.quality.map(|q| q.verdict()), agreement: None };
        // A composed pair replaces the report of its rejected direct measurement.
        match edges.iter_mut().find(|e| (e.from, e.to) == (report.from, report.to) || (e.from, e.to) == (report.to, report.from)) {
            Some(e) if matches!(e.kind, EdgeKind::Rejected(_)) => {
                let reason = match &e.kind {
                    EdgeKind::Rejected(r) => r.clone(),
                    _ => String::new(),
                };
                log::info!("align all: pair {}-{} {reason}; composed instead", e.from, e.to);
                *e = report;
            }
            Some(_) => {}
            None => edges.push(report),
        }
    }
    AlignAllOutcome { placements, edges, refine, seconds: t0.elapsed().as_secs_f32() }
}

/// Why an edge was not used, for the report.
pub fn rejection_label(r: &Rejection) -> String {
    match r {
        Rejection::Failed(e) => format!("no fit found ({e})"),
        Rejection::Poor => "poor fit".into(),
        Rejection::Inconsistent => "the two directions disagree".into(),
        Rejection::Cycle => "breaks a loop of passes".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_align::synthetic;

    #[test]
    fn median_and_fit_error() {
        assert_eq!(median([3.0, 1.0, f32::INFINITY, 2.0].into_iter()), Some(2.0));
        assert_eq!(median([4.0, 1.0].into_iter()), Some(2.5));
        assert_eq!(median(std::iter::empty()), None);
        let q = Quality { overlap: 0.4, rmse_mm: 0.3, visibility_conflict: 0.0, inliers: 500, ..Quality::NONE };
        assert_eq!(fit_error(&q), 0.3);
        assert!(fit_error(&Quality::NONE).is_infinite());
    }

    #[test]
    fn the_reverse_view_and_disagreement_lower_the_verdict() {
        let good = Quality { overlap: 0.4, rmse_mm: 0.3, visibility_conflict: 0.01, inliers: 5000, slide_mm: 2.0, ..Quality::NONE };
        assert_eq!(good.verdict(), Verdict::Good);
        let check = |reverse: Quality, consistent: bool| AlignCheck {
            reverse,
            agreement: Some(Agreement { rotation_deg: if consistent { 0.2 } else { 40.0 }, offset_mm: 1.0 }),
            consistent,
            runs: 6,
            agreeing: 3,
            object_only: true,
        };
        assert_eq!(checked_verdict(&good, None), Verdict::Good);
        assert_eq!(checked_verdict(&good, Some(&check(good, true))), Verdict::Good);
        assert_eq!(checked_verdict(&good, Some(&check(good, false))), Verdict::Fair);
        let under_table = Quality { below_support: 0.3, ..good };
        assert_eq!(checked_verdict(&good, Some(&check(under_table, true))), Verdict::Poor);
    }

    #[test]
    fn a_worse_refinement_is_flagged() {
        let q = |rmse: f32| Quality { overlap: 0.4, rmse_mm: rmse, visibility_conflict: 0.0, inliers: 500, ..Quality::NONE };
        let edge = |before, after| EdgeChange { a: 1, b: 2, before: q(before), after: q(after), thickness_mm: None };
        let outcome = |edges| RefineOutcome { poses: Vec::new(), edges, qualities: Vec::new(), seconds: 0.0 };
        assert!(!outcome(vec![edge(0.5, 0.3), edge(0.4, 0.35), edge(0.2, 0.25)]).worse());
        assert!(outcome(vec![edge(0.3, 0.5), edge(0.3, 0.4), edge(0.3, 0.2)]).worse());
    }

    #[test]
    fn a_refinement_that_breaks_one_wall_pair_is_flagged() {
        let same = |rmse: f32| Quality { overlap: 0.4, rmse_mm: rmse, visibility_conflict: 0.0, inliers: 5000, slide_mm: 2.0, ..Quality::NONE };
        let edge = |before: Quality, after: Quality| EdgeChange { a: 1, b: 2, before, after, thickness_mm: None };
        let outcome = |edges| RefineOutcome { poses: Vec::new(), edges, qualities: Vec::new(), seconds: 0.0 };
        // An inside / outside pair meeting only at the lip (at least 100 inliers, so its error
        // is the lip's RMS): after the refinement 4 % of it pierces the other pass.
        let wall = |piercing: f32| Quality {
            overlap: 0.02,
            rmse_mm: 0.1,
            inliers: 150,
            shell: Some(scancraft_align::Shell { pairs: 20_000, fraction: 0.8, offset_mm: -3.0, iqr_mm: 0.3, piercing }),
            ..same(0.1)
        };
        let (before, after) = (wall(0.0), wall(0.04));
        assert_ne!(before.verdict(), Verdict::Poor, "{before:?}");
        assert!(after.contradicted(), "{after:?}");
        let refined = outcome(vec![edge(same(0.4), same(0.3)), edge(same(0.4), same(0.3)), edge(before, after)]);
        assert!(refined.median_after() < refined.median_before(), "the median alone looks better");
        assert!(refined.worse());
        assert!(!outcome(vec![edge(same(0.4), same(0.3)), edge(before, before)]).worse());
    }

    #[test]
    fn align_all_without_the_reference_places_nothing() {
        let (passes, _) = synthetic::cover_passes(3.0);
        let inputs: Vec<PassInput> =
            passes.into_iter().enumerate().map(|(k, cloud)| PassInput { id: k as u32 + 1, pose: Transform::default(), placed: false, reference: false, cloud }).collect();
        let outcome = align_all_passes(&inputs, &GraphConfig::default());
        assert_eq!(outcome.moved().count(), 0);
        assert_eq!(outcome.unconnected().count(), inputs.len());
    }

    #[test]
    fn pass_qualities_score_each_pass_against_the_others() {
        let (passes, object_poses) = synthetic::cover_passes(3.0);
        // project (= pass 0's frame) <- pass k.
        let poses: Vec<Transform> = object_poses.iter().map(|p| compose(&object_poses[0], &p.inverse())).collect();
        let scored: Vec<(u32, &PassCloud, Option<Transform>)> = passes.iter().zip(&poses).enumerate().map(|(k, (c, t))| (k as u32 + 1, c, Some(*t))).collect();
        let q = pass_qualities(&scored);
        assert_eq!(q.len(), 3);
        assert!(q.iter().all(|(_, q)| q.verdict() != Verdict::Poor), "{q:?}");
    }

    /// The synthetic shell cover: an outside scan and two inside scans, none placed yet.
    #[test]
    fn align_all_places_inside_and_outside_scans_of_a_shell() {
        let (passes, object_poses) = synthetic::cover_passes(3.0);
        let inputs: Vec<PassInput> = passes
            .into_iter()
            .enumerate()
            .map(|(k, cloud)| PassInput { id: k as u32 + 1, pose: Transform::default(), placed: k == 0, reference: k == 0, cloud })
            .collect();
        let outcome = align_all_passes(&inputs, &GraphConfig::default());
        assert_eq!(outcome.moved().count(), 2, "{:?}", outcome.edges);
        assert_eq!(outcome.unconnected().count(), 0);
        for p in outcome.moved() {
            // project <- pass k = (pass 0 <- part) (pass k <- part)^-1; pass 0's frame is the part's.
            let truth = object_poses[p.id as usize - 1].inverse();
            let pose = p.pose.unwrap();
            let probe = scancraft_align::probe(&inputs[p.id as usize - 1].cloud.points, 500);
            let a = Agreement::between(&pose, &truth, &probe);
            assert!(a.within(0.5, 1.0), "pass {}: {a:?}", p.id);
            let q = p.quality.expect("quality");
            assert_ne!(q.verdict(), Verdict::Poor, "pass {}: {q:?}", p.id);
        }
        // Every pair is reported once.
        assert_eq!(outcome.edges.len(), 3, "{:?}", outcome.edges);
        assert!(outcome.edges.iter().any(|e| matches!(e.kind, EdgeKind::Placed)));
    }
}
