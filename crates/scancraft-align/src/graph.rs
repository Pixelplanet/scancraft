//! Aligning all passes of a project at once.
//!
//! Every pair of passes is measured with [`align_robust`]. Edges that are plausible (not Poor)
//! and found in both directions are kept. A maximum-quality spanning tree from the reference pass
//! gives every pass its transform, pairs without a kept edge are composed through the tree, and
//! every kept edge outside the tree closes a cycle whose residual is checked: while some cycle
//! does not close, the edge that breaks the most cycles is rejected and the tree rebuilt.
//!
//! This finds passes that share no surface with the reference directly, e.g. an inside scan of a
//! shell part that only meets the outside scan through a second inside scan.

use rayon::prelude::*;
use scancraft_core::Transform;

use crate::global::{align_robust, RobustAlignment, RobustConfig};
use crate::quality::QualityConfig;
use crate::{compose, evaluate, probe, Agreement, AlignError, PassCloud, Quality, Verdict};

/// Points per pass used to measure cycle residuals.
const PROBE_POINTS: usize = 2000;

/// Settings for [`align_all`].
#[derive(Debug, Clone)]
pub struct GraphConfig {
    /// How each pair is measured.
    pub robust: RobustConfig,
    /// Use Uncertain edges too (after the Good ones); otherwise only Good edges are kept.
    pub accept_fair: bool,
    /// Keep only edges found the same way in both directions ([`RobustAlignment::consistent`]).
    /// Has no effect when [`RobustConfig::both_directions`] is off: nothing was checked then.
    pub require_consistent: bool,
    /// A cycle closes when going round it turns by at most this...
    pub max_cycle_rotation_deg: f32,
    /// ...and moves the first pass's points by at most this (RMS), mm.
    pub max_cycle_offset_mm: f32,
    /// Sample grid for scoring the composed pairs, mm; 0: not scored.
    pub composed_sample_mm: f32,
}

impl Default for GraphConfig {
    fn default() -> Self {
        GraphConfig { robust: RobustConfig::default(), accept_fair: true, require_consistent: true, max_cycle_rotation_deg: 3.0, max_cycle_offset_mm: 5.0, composed_sample_mm: 1.5 }
    }
}

/// Why an edge is not used.
#[derive(Debug, Clone, PartialEq)]
pub enum Rejection {
    /// No alignment was found.
    Failed(AlignError),
    /// The best result is Poor (or Uncertain, when only Good edges are kept).
    Poor,
    /// Aligning the other way round gives a different result.
    Inconsistent,
    /// It does not close the cycles it is part of.
    Cycle,
}

/// What became of a measured edge.
#[derive(Debug, Clone, PartialEq)]
pub enum EdgeState {
    /// Part of the spanning tree: it places a pass.
    Tree,
    /// Kept outside the tree; the cycle it closes was checked and closes.
    Kept,
    /// Kept, but neither pass is connected to the reference.
    Unreached,
    Rejected(Rejection),
}

/// One measured pair of passes.
#[derive(Debug, Clone)]
pub struct GraphEdge {
    pub from: usize,
    pub to: usize,
    /// `to <- from`; `None` when nothing was found.
    pub transform: Option<Transform>,
    /// `from` moved onto `to`, and the reverse.
    pub quality: Quality,
    pub reverse: Quality,
    pub verdict: Verdict,
    /// Ranking score ([`crate::Candidate::score`]).
    pub score: f32,
    /// Best result against the closest one found the other way round.
    pub agreement: Option<Agreement>,
    pub consistent: bool,
    pub state: EdgeState,
}

impl GraphEdge {
    /// The edge for one [`align_robust`] result (`from` moved onto `to`), kept or rejected as
    /// `config` says.
    pub fn new(from: usize, to: usize, result: Result<RobustAlignment, AlignError>, config: &GraphConfig) -> Self {
        match result {
            Err(e) => GraphEdge {
                from,
                to,
                transform: None,
                quality: Quality::NONE,
                reverse: Quality::NONE,
                verdict: Verdict::Poor,
                score: 0.0,
                agreement: None,
                consistent: false,
                state: EdgeState::Rejected(Rejection::Failed(e)),
            },
            Ok(r) => {
                let verdict = r.verdict();
                let state = if verdict == Verdict::Poor || (verdict == Verdict::Fair && !config.accept_fair) {
                    EdgeState::Rejected(Rejection::Poor)
                } else if config.require_consistent && config.robust.both_directions && !r.consistent {
                    EdgeState::Rejected(Rejection::Inconsistent)
                } else {
                    EdgeState::Kept
                };
                GraphEdge {
                    from,
                    to,
                    transform: Some(r.transform()),
                    quality: r.best.alignment.quality,
                    reverse: r.best.reverse,
                    verdict,
                    score: r.best.score(),
                    agreement: r.agreement,
                    consistent: r.consistent,
                    state,
                }
            }
        }
    }

    /// Kept (in the tree or not).
    pub fn accepted(&self) -> bool {
        matches!(self.state, EdgeState::Tree | EdgeState::Kept | EdgeState::Unreached)
    }

    /// Links passes `a` and `b` (either way round).
    pub fn joins(&self, a: usize, b: usize) -> bool {
        (self.from == a && self.to == b) || (self.from == b && self.to == a)
    }

    /// Tree order: Good before Uncertain, then by score.
    fn priority(&self) -> (u8, f32) {
        (u8::from(self.verdict == Verdict::Good), self.score)
    }

    /// `other <- pass` along this edge, `pass` being one of its ends.
    fn towards_other(&self, pass: usize) -> Transform {
        let t = self.transform.unwrap_or_default();
        if pass == self.from { t } else { t.inverse() }
    }
}

/// A pair of passes placed through the tree instead of by a kept edge of their own.
#[derive(Debug, Clone)]
pub struct ComposedEdge {
    pub from: usize,
    pub to: usize,
    /// `to <- from`.
    pub transform: Transform,
    /// The passes from `from` to `to` through the tree.
    pub path: Vec<usize>,
    /// How the composed fit looks (`from` moved onto `to`); `None` when not scored.
    pub quality: Option<Quality>,
}

/// One cycle check: a kept edge outside the tree against the tree's path between its passes.
#[derive(Debug, Clone)]
pub struct CycleCheck {
    /// The closing edge (index into [`GraphResult::edges`]).
    pub edge: usize,
    /// The passes round the cycle: the edge's `from`, through the tree, to its `to`.
    pub passes: Vec<usize>,
    /// Measured against composed `to <- from`, on `from`'s points.
    pub residual: Agreement,
    pub closed: bool,
}

/// The result of [`align_all`].
#[derive(Debug, Clone)]
pub struct GraphResult {
    pub reference: usize,
    /// `reference <- pass` for every pass, `None` for passes not connected to the reference.
    pub poses: Vec<Option<Transform>>,
    /// Each pass's parent in the spanning tree (`None` for the reference and unconnected passes).
    pub parents: Vec<Option<usize>>,
    /// Every measured pair, with what became of it.
    pub edges: Vec<GraphEdge>,
    /// Connected pairs without a kept edge, composed through the tree.
    pub composed: Vec<ComposedEdge>,
    /// Every cycle check made, in order: those that failed (and got an edge rejected), then the
    /// final ones.
    pub cycles: Vec<CycleCheck>,
    /// Passes that could not be connected to the reference.
    pub unconnected: Vec<usize>,
}

impl GraphResult {
    /// Every pass has a transform to the reference.
    pub fn connected(&self) -> bool {
        self.unconnected.is_empty()
    }

    /// The edge between `a` and `b`, if that pair was measured.
    pub fn edge(&self, a: usize, b: usize) -> Option<&GraphEdge> {
        self.edges.iter().find(|e| e.joins(a, b))
    }
}

/// Aligns all `passes` to `passes[reference]`: measures every pair ([`measure_edges`]), then
/// builds and checks the graph ([`solve_graph`]).
pub fn align_all(passes: &[PassCloud], reference: usize, config: &GraphConfig) -> GraphResult {
    let edges = measure_edges(passes, config);
    solve_graph(passes, reference, edges, config)
}

/// Every pair `i < j` aligned with [`align_robust`] (`i` moved onto `j`), in parallel.
pub fn measure_edges(passes: &[PassCloud], config: &GraphConfig) -> Vec<GraphEdge> {
    let n = passes.len();
    let pairs: Vec<(usize, usize)> = (0..n).flat_map(|i| (i + 1..n).map(move |j| (i, j))).collect();
    pairs.par_iter().map(|&(i, j)| GraphEdge::new(i, j, align_robust(&passes[i], &passes[j], &config.robust), config)).collect()
}

/// Spanning tree over the accepted edges: per pass its (parent, edge) and `reference <- pass`.
struct Tree {
    links: Vec<Option<(usize, usize)>>,
    poses: Vec<Option<Transform>>,
}

impl Tree {
    /// Prim's algorithm from `reference`, always adding the best accepted edge that reaches a new
    /// pass.
    fn build(n: usize, reference: usize, edges: &[GraphEdge]) -> Tree {
        let mut tree = Tree { links: vec![None; n], poses: vec![None; n] };
        tree.poses[reference] = Some(Transform::default());
        loop {
            let next = edges
                .iter()
                .enumerate()
                .filter(|(_, e)| e.accepted() && tree.poses[e.from].is_some() != tree.poses[e.to].is_some())
                .max_by(|a, b| a.1.priority().partial_cmp(&b.1.priority()).unwrap_or(std::cmp::Ordering::Equal));
            let Some((k, e)) = next else { break };
            let (parent, child) = if tree.poses[e.from].is_some() { (e.from, e.to) } else { (e.to, e.from) };
            let parent_pose = tree.poses[parent].unwrap_or_default();
            tree.poses[child] = Some(compose(&parent_pose, &e.towards_other(child)));
            tree.links[child] = Some((parent, k));
        }
        tree
    }

    /// `b <- a` through the tree (both connected).
    fn relative(&self, a: usize, b: usize) -> Transform {
        compose(&self.poses[b].unwrap_or_default().inverse(), &self.poses[a].unwrap_or_default())
    }

    /// The passes from `a` to `b` through the tree and the edges on the way.
    fn path(&self, a: usize, b: usize) -> (Vec<usize>, Vec<usize>) {
        let up = |mut p: usize| {
            let mut chain = vec![(p, None)];
            while let Some((parent, edge)) = self.links[p] {
                chain.push((parent, Some(edge)));
                p = parent;
            }
            chain
        };
        let (ua, ub) = (up(a), up(b));
        // Lowest common ancestor: the first pass of a's chain that is also on b's.
        let (ia, ib) = ua.iter().enumerate().find_map(|(i, (p, _))| ub.iter().position(|(q, _)| q == p).map(|j| (i, j))).unwrap_or((ua.len() - 1, ub.len() - 1));
        let mut passes: Vec<usize> = ua[..=ia].iter().map(|x| x.0).collect();
        passes.extend(ub[..ib].iter().rev().map(|x| x.0));
        // The edge leading up from a chain entry is stored with its parent (the next entry).
        let edges = ua[1..=ia].iter().chain(ub[1..=ib].iter()).filter_map(|x| x.1).collect();
        (passes, edges)
    }
}

/// Builds the graph from measured `edges` (any [`EdgeState`]; rejected ones stay rejected):
/// spanning tree, cycle checks with rejection of the edges that break cycles, composed pairs.
pub fn solve_graph(passes: &[PassCloud], reference: usize, mut edges: Vec<GraphEdge>, config: &GraphConfig) -> GraphResult {
    let n = passes.len();
    let probes: Vec<Vec<glam::Vec3>> = passes.iter().map(|p| probe(&p.points, PROBE_POINTS)).collect();
    let mut cycles = Vec::new();
    let tree = loop {
        for e in edges.iter_mut().filter(|e| e.accepted()) {
            e.state = EdgeState::Kept;
        }
        let tree = Tree::build(n, reference, &edges);
        for &(_, k) in tree.links.iter().flatten() {
            edges[k].state = EdgeState::Tree;
        }
        let mut checks = Vec::new();
        let mut bad_edges: Vec<Vec<usize>> = Vec::new();
        for (k, e) in edges.iter_mut().enumerate().filter(|(_, e)| e.state == EdgeState::Kept) {
            if tree.poses[e.from].is_none() || tree.poses[e.to].is_none() {
                e.state = EdgeState::Unreached;
                continue;
            }
            let predicted = tree.relative(e.from, e.to);
            let residual = Agreement::between(&e.transform.unwrap_or_default(), &predicted, &probes[e.from]);
            let closed = residual.within(config.max_cycle_rotation_deg, config.max_cycle_offset_mm);
            let (path, on_path) = tree.path(e.from, e.to);
            if !closed {
                bad_edges.push(on_path.into_iter().chain([k]).collect());
            }
            checks.push(CycleCheck { edge: k, passes: path, residual, closed });
        }
        if bad_edges.is_empty() {
            cycles.extend(checks);
            break tree;
        }
        // Reject the edge in the most open cycles; among those, the weakest.
        let mut count = vec![0usize; edges.len()];
        for k in bad_edges.iter().flatten() {
            count[*k] += 1;
        }
        let worst = (0..edges.len())
            .filter(|&k| count[k] > 0)
            .max_by(|&a, &b| count[a].cmp(&count[b]).then(edges[b].priority().partial_cmp(&edges[a].priority()).unwrap_or(std::cmp::Ordering::Equal)))
            .unwrap_or(bad_edges[0][0]);
        log::debug!("align_all: edge {}-{} breaks {} cycle(s), rejected", edges[worst].from, edges[worst].to, count[worst]);
        edges[worst].state = EdgeState::Rejected(Rejection::Cycle);
        cycles.extend(checks.into_iter().filter(|c| !c.closed));
    };

    // Connected pairs without a kept edge, composed through the tree.
    let pairs: Vec<(usize, usize)> = (0..n)
        .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
        .filter(|&(i, j)| tree.poses[i].is_some() && tree.poses[j].is_some() && !edges.iter().any(|e| e.joins(i, j) && e.accepted()))
        .collect();
    let composed = pairs
        .par_iter()
        .map(|&(i, j)| {
            let transform = tree.relative(i, j);
            let quality = (config.composed_sample_mm > 0.0).then(|| evaluate(&passes[i], &passes[j], &transform, QualityConfig::for_sample(config.composed_sample_mm)));
            ComposedEdge { from: i, to: j, transform, path: tree.path(i, j).0, quality }
        })
        .collect();
    let unconnected = (0..n).filter(|&k| tree.poses[k].is_none()).collect();
    let parents = tree.links.iter().map(|l| l.map(|(p, _)| p)).collect();
    GraphResult { reference, poses: tree.poses, parents, edges, composed, cycles, unconnected }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic;
    use crate::{rigid, rotation_angle};
    use glam::{Quat, Vec3};

    /// A measured edge with the given transform and score, kept.
    fn edge(from: usize, to: usize, transform: Transform, score: f32) -> GraphEdge {
        GraphEdge {
            from,
            to,
            transform: Some(transform),
            quality: Quality::NONE,
            reverse: Quality::NONE,
            verdict: Verdict::Good,
            score,
            agreement: None,
            consistent: true,
            state: EdgeState::Kept,
        }
    }

    #[test]
    fn one_direction_only_is_not_counted_as_inconsistent() {
        let good = Quality { overlap: 0.4, rmse_mm: 0.3, visibility_conflict: 0.01, inliers: 5000, slide_mm: 2.0, ..Quality::NONE };
        assert_eq!(good.verdict(), Verdict::Good);
        let best = crate::global::Candidate { alignment: crate::Alignment { transform: Transform::default(), quality: good }, reverse: good, voxel_mm: 4.0, reversed: false };
        // What align_robust returns when only one direction was run.
        let found = || Ok(RobustAlignment { best, candidates: vec![best], agreement: None, consistent: false, agreeing: 1 });
        let one_way = GraphConfig { robust: RobustConfig { both_directions: false, ..Default::default() }, ..Default::default() };
        assert_eq!(GraphEdge::new(0, 1, found(), &one_way).state, EdgeState::Kept);
        // Both directions run and the other one found nothing: not confirmed.
        assert_eq!(GraphEdge::new(0, 1, found(), &GraphConfig::default()).state, EdgeState::Rejected(Rejection::Inconsistent));
    }

    #[test]
    fn a_wrong_edge_that_breaks_cycles_is_rejected_even_when_it_scores_best() {
        let cloud = PassCloud { points: synthetic::object(4.0), ..Default::default() };
        let passes = vec![cloud; 5];
        // Truth: reference <- pass k.
        let truth: Vec<Transform> = (0..4).map(|k| rigid(Quat::from_rotation_z(0.7 * k as f32), Vec3::new(20.0 * k as f32, -5.0, 3.0 * k as f32))).collect();
        let rel = |a: usize, b: usize| compose(&truth[b].inverse(), &truth[a]);
        let mut edges = Vec::new();
        for a in 0..4 {
            for b in a + 1..4 {
                let t = if (a, b) == (1, 3) { compose(&rigid(Quat::from_rotation_x(0.35), Vec3::new(0.0, 15.0, 0.0)), &rel(a, b)) } else { rel(a, b) };
                edges.push(edge(a, b, t, if (a, b) == (1, 3) { 0.9 } else { 0.5 }));
            }
        }
        // Pass 4 is measured against nothing that holds.
        let mut failed = edge(0, 4, Transform::default(), 0.0);
        failed.state = EdgeState::Rejected(Rejection::Poor);
        edges.push(failed);
        let result = solve_graph(&passes, 0, edges, &GraphConfig::default());
        let wrong = result.edge(1, 3).expect("measured");
        assert_eq!(wrong.state, EdgeState::Rejected(Rejection::Cycle), "{:?}", result.cycles);
        for c in &result.cycles {
            eprintln!("cycle {:?} closed by {}-{}: {:?}, closed {}", c.passes, result.edges[c.edge].from, result.edges[c.edge].to, c.residual, c.closed);
        }
        assert!(result.cycles.iter().any(|c| !c.closed));
        // The final checks (the last one per closing edge) all close: 6 edges, 3 in the tree,
        // one rejected, two closing cycles.
        let kept: Vec<usize> = (0..result.edges.len()).filter(|&k| result.edges[k].state == EdgeState::Kept).collect();
        assert_eq!(kept.len(), 2);
        for k in kept {
            assert!(result.cycles.iter().rev().find(|c| c.edge == k).is_some_and(|c| c.closed), "edge {k}");
        }
        for k in 1..4 {
            let (pose, expected) = (result.poses[k].expect("connected"), rel(k, 0));
            assert!(rotation_angle(pose.rotation, expected.rotation) < 1e-3 && (pose.translation - expected.translation).length() < 1e-2, "pass {k}");
        }
        assert_eq!(result.unconnected, vec![4]);
        assert!(result.composed.iter().all(|c| c.from != 4 && c.to != 4));
        // 1-3 is the only connected pair without a kept edge.
        assert_eq!(result.composed.len(), 1);
        assert_eq!((result.composed[0].from, result.composed[0].to), (1, 3));
        assert_eq!(result.composed[0].path.first(), Some(&1));
        assert_eq!(result.composed[0].path.last(), Some(&3));
    }

    /// Three passes of a thick shell: the outside, the inside, and the inside again with some of
    /// the outside near the rim. All three are placed, the outside-inside pair either measured
    /// consistently or composed through the second inside pass, and the composed or measured
    /// nesting shows the wall.
    #[test]
    fn connects_inside_and_outside_scans_of_a_shell() {
        const WALL: f32 = 3.0;
        let (passes, object_poses) = synthetic::cover_passes(WALL);
        let t0 = std::time::Instant::now();
        let result = align_all(&passes, 0, &GraphConfig::default());
        eprintln!("align_all: {:?}", t0.elapsed());
        for e in &result.edges {
            eprintln!(
                "edge {}->{}: {:?}, {:?}, score {:.3}, agreement {:?}, wall {:?}, forward {:?}",
                e.from,
                e.to,
                e.state,
                e.verdict,
                e.score,
                e.agreement,
                e.quality.wall().map(|s| s.thickness_mm()),
                e.quality
            );
        }
        for c in &result.composed {
            eprintln!("composed {}->{} via {:?}: {:?}", c.from, c.to, c.path, c.quality);
        }
        for c in &result.cycles {
            eprintln!("cycle {:?}: {:?}, closed {}", c.passes, c.residual, c.closed);
        }
        assert!(result.connected(), "unconnected {:?}", result.unconnected);
        for k in 1..3 {
            let truth = object_poses[k].inverse();
            let found = result.poses[k].expect("placed");
            let points: Vec<Vec3> = passes[k].points.iter().map(|p| p.0).collect();
            let rot = rotation_angle(found.rotation, truth.rotation).to_degrees();
            let disp = crate::displacement_on(&found, &truth, &points);
            eprintln!("pass {k}: rotation error {rot:.3} deg, max displacement {disp:.3} mm");
            assert!(rot < 1.0 && disp < 1.0, "pass {k}: {rot:.3} deg, {disp:.3} mm");
        }
        // Outside (0) and the first inside pass (1): measured both ways alike, or composed.
        let nesting = match result.edge(0, 1) {
            Some(e) if e.accepted() => {
                assert!(e.consistent, "{e:?}");
                e.quality
            }
            _ => {
                let c = result.composed.iter().find(|c| (c.from, c.to) == (0, 1)).expect("composed 0-1");
                assert_eq!(c.path, vec![0, 2, 1]);
                c.quality.expect("scored")
            }
        };
        let wall = nesting.shell.expect("wall pairs");
        assert!((wall.thickness_mm() - WALL).abs() < 0.5 && wall.piercing < 0.01, "{nesting:?}");
    }
}
