# Open issues and requested features (2026-11-24)

Collected after merging branch `feature/clean-per-pass-meshing` into `main` (commit 3151b3d:
per-pass cleaning, meshing rework, Fill Holes, file dialogs). **Fixed on 2026-11-25: 1.1, 1.2,
1.3, 1.6 and 1.7, and the crease around filled holes from 2.2** (marked below); the rest is
open. Sections 1 and 2 come from a review and a measurement run on the original PC. Section 3
is what the user found by hand in the app. Line numbers refer to commit 3151b3d.

Suggested order for the next session: bugs 1.1 and 1.2 first (small, and they corrupt
results), then 3.1 (data loss), then the mesher detail problem (2.2), then the rest.

## 1. Bugs found by code review

Three reviewers read the merged commit, and each finding was then checked by a second agent
that tried to refute it. Only the findings that survived are listed.

### 1.1 Cleaning or resetting a pass leaves the merged model and the mesh stale (high)

**Fixed 2026-11-25.** `apply_cleaned` and `reset_cleaning_of` call `invalidate_after_cleaning`:
the merged model goes when it contains a changed pass, the mesh goes in any case, one toast
names both and Undo restores them; cleaning only the merged model keeps it and drops the mesh.
Reset Cleaning also keeps a hidden pass hidden now. Tests:
`cleaning_a_pass_drops_the_merged_model_and_mesh_made_from_it`,
`cleaning_only_the_merged_model_keeps_it_and_drops_the_mesh`,
`a_cleaned_pass_is_not_dropped_from_a_merge_it_is_not_in`.

`apply_cleaned` (`apps/scancraft/src/app.rs:1646`) and `reset_cleaning_of` (`app.rs:1720`)
replace a pass's surface. Neither one sets `self.merged = None`, and neither calls
`invalidate_derived` (`app.rs:499`).

- Every other operation that changes what a pass contributes does both: `place_pass`
  and its callers, and `scale_pass` (`app_project.rs` 234, 258, 435, 544, 1022).
- `apply_cleaned` is used by the clean-job result (`app.rs:2090`) and by `commit_object`
  (`app.rs:1849`).
- **Scenario:** Align, Merge, Reconstruct. Then clean Pass 1, for example with Remove Plane.
  - The merged model and the mesh still contain the table.
  - `working_is_merged` (`app.rs:1100`) keeps using them.
  - Save writes them to `outputs/merged.ply` and `outputs/mesh.ply`.
  - Reopening shows the stale mesh (`app_project.rs:846`).
- Reset Cleaning goes wrong the same way.
- **Fix:** both functions should drop the merged model and call `invalidate_derived`, but
  not when the target is the merged model itself, and keep Undo working. Optionally add a
  pass generation to `MergedLayer` (`model.rs:326`) so a stale merge can be detected.

### 1.2 A cleaned pass is ignored when the next pass starts at "Same Position" (medium)

**Fixed 2026-11-25.** `seed_tracker` has an `Edited` arm: the cleaned surface as points, and for
a captured pass its kept keyframes as views (`project::seed_views`; cleaning removes points but
does not move the pass, so the poses hold). If seeding fails, a toast now says the pass starts
unaligned. Test: `a_cleaned_pass_seeds_a_same_position_pass`.

`seed_tracker` (`app_project.rs:162-179`) builds the seed model only from
`PassSource::Tracked` (line 164) and `PassSource::Fixed` (line 174). The commit added
`PassSource::Edited`, which `ScanPass::source()` (`model.rs:296`) returns for every cleaned
pass, and `seed_tracker` was not updated.

- **Scenario:** capture Pass 1, clean it (the new stage order invites that), then capture
  Pass 2 with Same Position.
  - The seed is empty, so `Tracker::seed_model` returns false (slam `lib.rs:402`).
  - Pass 2 is created `Unaligned` instead of `Locating` (`app.rs:913-916`), with no message.
- With several passes, the cleaned ones are simply missing from the seed.
- **Fix:** add an `Edited` arm. It uses the cleaned surface as points and the tracker's
  keyframes for views, if those are still valid for the cleaned surface.

### 1.3 Stray-point filter in the new mesher keeps points far off the surface (medium)

**Fixed 2026-11-25.** The filter evaluates leave-one-out (`Field::eval_excluding`). Test
`implicit_surface_drops_stray_points_up_to_the_support_radius` (strays 0.3–1.4 mm above a noisy
plane): 54 of 54 removed, 20 of 54 before the fix. On the two-pass shell project at app defaults
3 518 points are removed instead of 819 and boundary loops drop 395 → 381; the crumpled spots at
the pen marks remain, so 2.2 is still open.

`implicit.rs:407` measures each point's offset with `probe.eval(p, false)`, which includes
the point itself in its own neighbourhood.

- The point gets full kernel weight with offset 0, and its normal decides the side test.
- The robust reweighting then down-weights the real surface points.
- So the measured offset falls back towards 0 as the true offset approaches the support
  radius h.
- A numerical copy of `Field::eval` showed this with defaults (h = 1.5 mm, plane, noise
  0.05 mm):
  - strays at 0.3–0.9 mm are removed;
  - strays at 1.0–1.4 mm measure 0.07–0.00 mm and are kept.
- **Fix:** evaluate leave-one-out (skip the point itself).
- This may also be one cause of the pits in 2.2.

### 1.4 Open Project choice is lost if the app became busy while the dialog was open (low)

**Fixed 2026-11-25:** the picked project is kept (`pending_open`) and opened as soon as the app
is free, with a message; tabs do not switch meanwhile. Test
`a_project_picked_while_busy_opens_once_the_app_is_free`.

The `rfd` dialog runs on its own thread without a parent window (`file_picker.rs:78-104`), so
the main window stays usable.

- `poll_picker` (`app.rs:2041`) calls `open_project`. That returns silently when
  `!can_edit()` (`app_project.rs:768-771`).
- So if Align All or a capture was started meanwhile, the picked project is never opened
  and no message explains why.
- **Fix:** queue the request like imports, or at least show a message. Alternatively make the
  dialog modal to the window (set the parent).

### 1.5 New mesher's memory grows with point count × up to 216 (low, per user)

`implicit.rs:432-442` emits (2·band)³ grid corners per input point (64 at band 2, 216 at
band 3) into one `Vec` and only deduplicates afterwards.

| Points | Triangle size | Band | Corner list |
| --- | --- | --- | --- |
| 3 M (default budget) | 0.7 mm (default) | 2 | 2.3 GB |
| 3 M | 0.3 mm (slider minimum) | 3 | 7.8 GB |

The sort and the lookup map come on top of that. **The user considers this acceptable for
now** (8 GB is a normal amount for a 3D-scanning machine). If it is fixed later, deduplicate
per block or per cell instead of collecting everything.

### 1.6 `mesh_project` example does not use the app's defaults (low)

**Fixed 2026-11-25:** defaults are now cell 0.7, smooth 4, sigma 0.5144 (the app's).

The header of `crates/scancraft-mesh/examples/mesh_project.rs` says cell 1.0 and smooth 2 are
"the app's default". The app now uses:

- 0.7 mm cells (`app.rs:139`);
- 4 smoothing iterations (`app.rs:141`);
- smoothing sigma 0.514, from `smooth_keep_edges` 0.7 (`app.rs:143`, `168-170`).

The example uses sigma 0.35. So the numbers in merge-and-ai.md §7 were not measured at the
app's settings. **Fix:** change the example's defaults (use `--cell 0.7 --smooth 4 --sigma
0.514` until then).

### 1.7 Filling a very large hole takes minutes (low)

**Fixed 2026-11-25:** `FAIR_DIRECT_MAX` raised to 1 500 000, so holes up to the slider maximum
use the direct solve. Check on a 0.5 mm grid: a 1 193 mm hole (199 000 new vertices, above the
old cap) fills in 28 s; a 1 029 mm hole (144 000) in 14 s. The 1.3 M-triangle bench is unchanged
(0.57 s, watertight).

Above `FAIR_DIRECT_MAX` = 150 000 unknowns (`holes.rs:58`, `1074`), fairing switches from
Cholesky to PCG.

- PCG runs all 5000 iterations, and it is repeated after each 25 % growth of the patch.
- Measured on a hole of about 990 mm perimeter on a 0.5 mm grid, which is within the slider
  range (max 1000 mm):

  | | Total time | Distance to the true surface |
  | --- | --- | --- |
  | PCG (current) | 177 s | mean 0.14 mm, max 0.46 mm |
  | Cholesky forced | 27 s | mean 0.015 mm, max 0.06 mm |

- **Fix:** raise the cap (memory permitting), or solve once at the end, or make the job
  cancellable.

Two further claims were checked and rejected (Fill Holes on inconsistently oriented meshes,
which no app tool produces; and `keep_largest` sparing one loop per mesh, which is the
documented behaviour).

## 2. New mesher compared with the old one

### 2.1 Setup

- **Data:** the three Revo Scan scans of the thin shell part, as a saved project with the
  current alignment.
- **Tool:** `examples/mesh_project.rs`.
- **Outputs** (on the original PC, not in the repository): `E:\Claude\eval\mesh-rework\`.
  The side-by-side close-up is `compare_app_defaults.png`.

| App defaults (0.7 mm, 4 smoothing passes) | Old (`--method legacy`) | New (RIMLS) |
| --- | --- | --- |
| Roughness, mean / p90 (lower = smoother) | 2.53° / 6.11° | **1.52° / 2.46°** |
| Vertex to scan points, median / p95 | 0.31 / 0.68 mm | **0.30 / 0.54 mm** |
| Boundary loops (holes) | **1 302** | 2 103 |
| Time (reconstruct + smooth) | 0.5 s | 4.6 s |

At 1.0 mm cells and 2 smoothing passes (the example's old defaults), the new mesher left
74 184 boundary loops, nearly all single-cell pinholes, against 625 for the old one. At
detail 4 that drops to 1 212.

### 2.2 What it looks like

**Fixed 2026-11-25 (second PC), as far as it can be checked there.** The three Revo scans and
`E:\Claude\eval` are on the original PC; the second PC has the two-pass shell project (the one
with pen marks) and a synthetic relief. **Checked on the original PC 2026-11-26: there is no
lettering.** The user, who owns the part, says its outer top is a perfectly smooth rounded shape
with nothing on it; every feature on the smooth surfaces is a marker sticker site or an artefact
of Revo's cross-line laser mode. What was called "moulded lettering" is a lattice of laser-line
strays in one inside scan (S3). See 2.3.

Causes, each measured (synthetic relief = four ribs 0.3–1 mm high, 1–1.5 mm wide, and a ring on
a noisy plane, 0.5 mm pitch; test `implicit_surface_keeps_small_relief_without_pits`):

1. **Holes and pinholes: corners beyond the support radius.** A cell the surface crosses has
   corners up to √3 cells from it; next to a step (the flank of a letter has no points on it)
   they are further than `h` from every point, stayed unknown, and the cell was skipped. The
   synthetic relief had 356 open edges inside, all along the flanks of the 0.8 and 1 mm ribs.
   The tangential cut-off is *not* the cause (switching it off changed 356 to 354). Fix:
   `Field::eval_corner` evaluates such corners with a kernel 1.5× and 2.25× as wide. → 0.
2. **Pits and spikes: radii in units of a spacing that was too small.** "Spacing" was the median
   nearest-neighbour distance: 0.39 mm for a 0.5 mm pitch with ±0.15 mm jitter, 0.51 mm on the
   real merge whose density is one point per 0.65 mm square. The normal refit (2.5 × spacing)
   then fits a plane to about 9–12 points, tilts at every step, and the stray filter and the
   robust weights throw the relief out (pits 0.73 mm, spikes 0.38 mm; with either the robust
   weights or the stray filter alone the pits stay, with a refit radius of 3.3 spacings they go).
   Fix: `point_spacing` measures the pitch from the point density. Pits 0.06 mm, spikes 0.10 mm,
   rib tops at 101 % of their height (76 % after the app's 4 smoothing passes; old mesher 87 %
   with a plain area 1.4× as rough). Since 2026-11-27 it takes every point and the mean of the
   middle fifth of their values, not the median of a sample (see 2.4).
3. **Flaps and torn patches on the real scan: a second, sparse layer.** At the pen marks the
   merge has scatters of points 0.7–1.6 mm *behind* the continuous surface, facing either way
   (in a 50 mm window: 7 907 surface points, 540 behind it facing the same way, 1 343 facing
   away). They are not relief; the old mesher blends them into raised "ropes". Here they sat
   inside the support radius and the field switched layer from corner to corner. Fix:
   `dense_layers` drops points whose own sheet has at most 0.6 times as many points as the
   layer right above or below them (both faces of a thin wall and relief stay; test
   `implicit_surface_drops_a_sparse_layer_behind_the_surface`). A few small spots remain in the
   pen-mark area, where the second layer is as dense as the surface.
4. **Pinholes: the side test.** The side of a thin wall was the nearest point's normal; one
   tilted normal shut out the surface around it. Without the side test 87 → 51 pinholes. Fix:
   the points about as near as the nearest decide together when they agree (52).
5. **Stair-stepped open edges:** marching cubes only meshes fully known cells, and nothing
   moved the rim afterwards (the old mesher's Laplacian smoothing did). Fix: `round_open_edges`
   (rim vertices towards the middle of their rim neighbours, at most 0.6 cell). Rim waviness
   on a patch at 20° to the grid 0.24 → below 0.1 mm (`implicit_surface_has_round_open_edges`).

Not causes: the projection clamp (no effect on holes or pits), the robust weights' scales.

Two-pass shell project, app defaults (0.7 mm, 4 smoothing passes), `mesh_project`:

| | Old (`--method legacy`) | New before | New after |
| --- | --- | --- | --- |
| Boundary loops (of up to 8 edges) | 110 (52) | 381 | **123 (58)** |
| Roughness, mean / p90 | 2.65° / 5.78° | 2.14° / 3.98° | **1.64° / 3.40°** |
| Vertex to scan points, median / p95 | 0.277 / 0.573 mm | 0.271 / 0.489 mm | 0.273 / 0.523 mm |
| Points dropped before meshing | – | 3 518 | 5 403 |
| Time (reconstruct + smooth) | 0.1 s | 0.9 s | 1.3 s |

At 1.0 mm cells and 2 smoothing passes: 179 loops (119 pinholes) against the old mesher's 85
(45). The support radius on this project is now 1.94 mm instead of 1.52 mm (same `detail` 3, the
spacing is measured differently); `--detail 2.35` gives the old radius, with about 150 loops and 1.8°.
The "moulded pad" in this scan (8 mm across, 0.25–0.3 mm high, flat) is very probably a marker
sticker site (stickers are flat discs ~10–11 mm across, ~0.4 mm proud), not part geometry; the
ring-with-bars the old mesher shows there is its grain, not the part.

The list below is the state before these fixes.

- **Better:** large plain areas. The old mesher's woven grain and scaly patches are gone,
  and the fit to the points is closer.
- **Worse: small raised detail.** What looked like moulded lettering inside the shell (in fact
  cross-line laser strays, see 2.3) is "readable" with the old mesher and becomes a field of pits
  and spikes with the new one. This is the main
  problem before RIMLS can replace the old mesher. Check 1.3 first. Also check the normal
  refit and the robust weights on small relief.
- **Worse: open edges** are stair-stepped, aligned to the voxel grid; the old ones are
  ragged but rounded.
- **Pass overlap:** where two passes meet near the rim, the new mesher shows a 1–2 mm step.
  The old one blends it away. It is probably a real alignment offset the old mesher was
  averaging, but it shows as a crease.
- **Thin wall:** both meshers keep the 3 mm wall as two separate surfaces.
- **Fill Holes** (perimeter ≤ 60 mm) closed 82 868 of 82 926 holes in 1 s. *Partly fixed
  2026-11-25:* every filled hole is now blended into its surroundings
  (`HoleFiller::blend_rings`, default 2: Taubin steps over the patch, its rim and two rings,
  weighted down away from the hole), because the crease came from the stair-stepped rim the patch
  hangs from. Test `blending_removes_the_crease_at_a_jagged_rim`; on the two-pass project
  roughness after Fill Holes 2.76° → 2.63°. Filling still does not fix the pinholes' cause (see
  above): it should not be used to hide them. Every filled
  pinhole becomes a flat patch with a crease around it, so the result looks crumpled
  around detail, and roughness rises from 2.00° to 2.52° (in the 1.0 mm run). It should not
  be used to hide the pinholes.
- The roughness number does not see holes or small pits. Always look at boundary loops and
  at the renders as well.

### 2.3 Laser artefacts and marker sites on the Revo shell scans (2026-11-26, original PC)

Ground truth from the user: the part's top is smooth. Everything on it in our meshes is either a
marker sticker site or an artefact of the cross-line laser mode the three scans were made in.
Real geometry: rim, edges, the hinge brackets with their screw holes, screw bosses, ribs.

**What the artefacts are (measured, harness in `E:\Claude\eval\mesh-fixes\scripts\harness.py`):**

1. **Laser-line lattice in S3 (inside scan), the former "lettering".** Two families of curved
   lines, 9–12 mm apart, about 2 mm wide, near the upper hinge. They are stray points
   0.6–1.0 mm *behind* the inner surface, inside the wall, with an empty gap in between; 83 % face
   the wrong way. They are already in Revo's `raw_preview.ply`. S2 covers the same area cleanly.
   The new mesher turned them into pits up to 1 mm deep (relief std 0.31 mm).
2. **S3's grazing fringe on the outer face near the lower rim:** streaks of ±0.5 mm along the laser
   lines, 4 × as rough as S1 there.
3. **Fine straight hatching** 0.02–0.05 mm over S1 and S2 (also in Revo's fused cloud).
4. **S3 sits 0.17 mm above S2** (inner face) and 0.10 mm above S1 (outer face): steps where the
   coverage changes from one scan to the other.

Marker stickers are flat plateaus 0.37–0.46 mm high, flat to r ≈ 4–4.5 mm, back on the surface at
r ≈ 5.5–6 mm. The mkf radius of 3 mm is only the white dot. Some S2 stickers have an empty ring at
r 6–9 mm round them.

**What was changed** (branch `agent/mesh-fixes`):

- **Reconstruct** prepares the merged points first (`scancraft_mesh::prepare_points`, settings
  "Suppress scan artefacts" and "Remove marker stickers", both on by default). It drops points off
  their sheet and ghost layers (the Make Solid clean-up), then cuts every listed sticker and
  refills it from the ring. After meshing, the sites are faired.
- **Marker sites (both tools):**
  - The cut is at least 6.5 mm. It grows over an empty ring round the sticker (up to 5 mm more,
    measured by occupancy). Sites that are only partly covered (near the rim) are patched only
    where the scan has surface.
  - Where a quadric leaves a rough annulus, the ring fit is a cubic.
  - Mesh fairing now puts the patch on the fit plus the residual of the mesh in a 2 mm band
    beyond the cut, interpolated inwards. Before, it blended a 2 mm band onto the fit, which left
    a visible ring.
  - Revo entries with a radius below 2 mm are not stickers and are ignored.
- **Make Solid:**
  - drops laser strays in each scan's own points (`prep::drop_behind_strays`: a denser sheet of
    the same scan 0.4–1.6 mm in front of them with an empty gap);
  - removes S3's offset (`equalise_scan_offsets`: only the scan that is offset the same way
    against every scan it overlaps is moved, and a moved scan is a reference from then on);
  - drops the rougher scan where another scan covers the same sheet much more smoothly
    (`drop_rough_layers`);
  - smooths with the feature-preserving smoother (4 steps) before the marker fairing.
- **Screw holes (Make Solid):**
  - A detected round "hole" whose rim goes on down as a bore wall is no marker. Before, the
    bracket's screw hole was detected as one and patched shut.
  - A detected hole needs plain sheet all round it.
  - The hole filler leaves round openings open when their axis is clear of points on both sides.

**Results** (harness; Reconstruct = app defaults, 0.7 mm, 4 smoothing passes; before = main
11c54c4, after = `agent/mesh-fixes`; outputs in `E:\Claude\eval\mesh-fixes\v6`):

| | Reconstruct before | after | Make Solid before | after |
| --- | --- | --- | --- | --- |
| Inner lattice window, relief std / p99 (mm) | 0.307 / 1.004 | **0.004 / 0.010** (app path 0.013 / 0.038) | 0.015 / 0.059 | **0.005 / 0.020** |
| Outer S3 band | 0.005 / 0.013 | 0.005 / 0.013 | 0.015 / 0.055 | 0.011 / 0.039 |
| Outer plain top | 0.004 / 0.010 | 0.004 / 0.010 | 0.006 / 0.016 | 0.004 / 0.011 |
| Marker disc vs ring fit, median outer / inner (mm) | +0.40 / +0.36 | **+0.010 / +0.002** | +0.004 / +0.005 | +0.014 / +0.007 |
| Bracket screw hole, free radius (raw S3 2.62 mm) | 0.54 (app path: 2.20 after) | 0.54 | 0.17 (closed) | **1.44 (open)** |
| Boss recess depth A / B / C / D (raw 1.9–2.6 mm) | 0.86 / −0.15 / −0.27 / −0.20 | 1.24 / −0.13 / −0.26 / −0.23 | 2.14 / 1.82 / 1.77 / 1.73 | 2.20 / 1.94 / 1.54 / 1.69 |

For scale: one good scan's own relief is 0.018–0.038 mm. Make Solid on the three scans now takes
33 s (was 30 s), with a peak working set of 1.0 GB. The result is watertight and has genus 11
(was 24).

Comparison images (grazing light): `E:\Claude\eval\mesh-fixes\compare_top.png` (outside top with
five markers, the inside lattice, two marker close-ups) and `compare_bracket_hole.png`. In them:

- The raised sticker rings are gone in Reconstruct, and the 1 mm pits of the lattice are gone.
- The disc outlines are gone in Make Solid; the screw hole is open again.

**Update 2026-11-27, from the user:** the round "depressions" are flat fills. Every spot where a
sticker was is a hole in the scan, and the fill is a flat spot. This shows in both of our solids
and also in Revo Scan's own result. On the curved dome a flat spot looks like a dent under light.
The 0.06–0.20 mm offsets the harness found at seven outer sites match this: a flat 11 mm disc on a
surface of radius R sinks by r²/2R, 0.15 mm at R = 100 mm. The fills now continue the curvature
round them. What changed, the per-site numbers and what remains are in 2.4.

**Still open:**

- **Marker sites:** see 2.4. (Before 2026-11-27: seven outer sites more than 0.03 mm off a quadric
  of their own 7–11 mm ring, S1-M1, M4, M6, M20, M21, M36 and M43, up to +0.20 mm. On curving areas
  and at rims that quadric is the wrong reference.)
- **Faint dots of the S3 lattice in the solid** are still visible under grazing light (relief
  p99 0.020 mm).
- **Make Solid narrows the screw hole** with a small tongue (free radius 1.44 of 3.1 mm): the
  hole filler patches the shadowed gap beside the bore.
- **The new mesher fills the boss pilot recesses** (unchanged, a mesher property; the old mesher
  and Make Solid keep them).
- **Reconstruct's laser-stray filter is off.** The same direct filter on the *merged* points
  removed only part of the strays, and the wall test and the mesher's layer test then missed the
  rest (lattice window 0.004 → 0.064 mm). `PrepareConfig::laser_strays` stays off; Make Solid
  runs the filter per scan.
- **Isolate Object cut a sticker out of S2.** It dropped all S2 points within 12 mm of S2-M13, so
  Reconstruct has a 20 mm opening there and the solid a ring of beads round its hole fill. This
  is a data gap from the isolation mask, not from the marker cut.
- **The 0.1 mm "rounded-rectangle" line on the outer dome is real** (user, 2026-11-27). The part
  is painted in two colours, and a white stripe covers the boundary between them. It is a real
  feature and must be kept, not smoothed away.

**Trade-off** (test `prepare_keeps_small_relief`): the preparation keeps relief on a plain sheet,
such as the mesher test's synthetic ribs, with "Suppress scan artefacts" on or off. It does drop
narrow relief that stands 0.6–2.5 mm in front of or behind a thin wall when its thickness
disagrees with the wall map. Turn the setting off for parts with fine engraving.
`implicit_surface_keeps_small_relief_without_pits` is unchanged.

### 2.4 Marker-site fills that continue the curvature (2026-11-27, with the review fixes)

**What the user saw:** round depressions at the marker sites, in our solids and in Revo Scan's own
mesh: the sticker hides the surface, the hole is filled with a flat spot, and on a dome a flat
disc looks like a dent. The rounded-rectangle line on the outside is real (two paint colours under
a white stripe) and is kept.

**What was measured first** (`E:\Claude\eval\mesh-curv`, scripts in `scripts\`): Revo's mesh has
flat fills (flatness ratio 0.54–0.76 at the clean sites). Ours did not at the clean sites (ratio
0.9–1.2, centre within 0.02 mm of the surface's own continuation). The sites that showed were the
rim sites (M1, M4, M36, M43, S2-M4), the curvature-band and flange sites (M6, M16, M20, M21, the
rc-12 site S2-M12/S3-M22), and every fill was smoother than the scanned surface round it.

**What changed** (`crates/scancraft-mesh/src/solid/sitefit.rs`, `markers.rs`; first pass 41e7b72,
review fixes after it):
- **Ring fit:** samples must face like the fit (normal within 20°) and lie within 3 σ of it, σ from
  the MAD. The quadric is a conic: its quadratic part enters as the sag 2Q / (1 + sqrt(1 − 2κ|Q|)),
  κ its largest principal curvature. That is exact on a sphere or a cylinder of any radius. A plain
  quadric missed the centre of a small ball by r⁴/8R³: +0.11 mm at R 15 mm, +0.06 at R 20.
- **Two ring fits:** one trimmed against the quadric plus the band's low orders (a curvature band is
  not taken for noise), and a plain one trimmed against the quadric alone. The plain fit drops a
  feature beside the cut and is the fill wherever the band is not continued.
- **Curvature bands:** where the 4.5 mm band next to the cut is plain sheet all round, the band's
  low angular orders (m ≤ 3) are continued into the cut with value and slope (A r^m + B r^(m+2)).
  Two extra tests since the review:
  - the orders must explain clearly more than they would explain of plain noise (σ sqrt(k / n)).
    A fixed 0.01 mm let noise of 0.1–0.2 mm switch them on, and the fill got 3–6 × worse;
  - inside the cut the continuation must stay within the range it has on the cut against the plain
    fit. The slope of a narrow ridge beside the cut, carried over 6.5 mm, dug a dent of 0.1–0.2 mm at
    the centre. Where more than half of the slope has to go, the band is not continued.
- **Partial sites (rims):**
  - The patch stays within 0.4 mm of the convex hull of the sheet. Cut points count as sheet only in
    the outer 0.4 mm of the cut. Further in they may be the sticker's edge ramp or a thin sticker,
    which over a rim widened the hull out to the sticker's edge (422 fill points past the rim).
  - The patch keeps behind surface going down in the cut (a rim's side face) only where at least six
    such points lie in directions without annulus surface. One stray below the sheet used to leave a
    wedge of the sheet empty.
- **Fairing** (`fair_sites`):
  - The mesh inside the cut goes onto the site's fill surface from the scan points, met by the mesh
    round the cut (`SiteSurface::met_by`). A site that was kept or left to the solve (no fill surface)
    is faired onto a fit of the mesh ring, as before 41e7b72. A kept sticker stayed 0.33 mm proud.
  - Quality check again for every site: the mesh ring's RMS against a robust quadric must be
    ≤ 0.1 mm with 60 % kept. M4 (0.120) and M36 (0.166) are left as meshed again. Their folded rim
    edges are gone. `FairOutcome::ring_rms_mm` reports that RMS (it was always 0).
  - `met_by` carries in, over the whole cut, only the band samples within 3 σ of the band's median.
    The rest (a rim's rolled edge) is carried in only near the cut, decaying over 1 mm. Carried over
    the whole cut it sank M1 by 0.1 mm.
  - Weights fade smoothly (offset 0.6–0.8 mm, normal 18–25°). Vertices toward directions where the
    mesh round the cut is not on the target (a rolled edge) are left alone near the cut. A vertex
    more than 0.3 mm below the target at a rim site fades out by 0.5 mm. Hard limits had moved one
    vertex of a rolled edge fully and left its neighbour.
- **Reconstruct:** band slopes stay off (`PrepareConfig`); the merged model has no pass ids.
- **Reconstruct's spacing:** `implicit::point_spacing` takes every point (up to 400 000) and the mean
  of the middle fifth of their values. The per-point value is a count within a radius, so it comes in
  steps of about 1 %. The median of a step sample jumped by one step when 64 fill points went
  (0.65 → 0.64 mm, support radius 2.0 → 1.9 mm), and that flipped the laser-lattice suppression.

**Per site, final** (`E:\Claude\eval\mesh-curv\e2e-final`). Two references, because each fails
somewhere:
- *self*: patch centre minus the biharmonic fill of the output's own band (mm), then the flatness
  ratio (1 = continues the curvature). Biased where the band holds a rolled rim (M1, M4, M36).
- *raw*: median height within 1.5 mm of the axis above a robust quadric of the raw scan's ring
  7.5–14 mm, normal within 37° (mm). Biased on curvature bands (M16, M20, M21: about +0.1–0.24 in
  every output) and at rims with little ring (M36).

| site | self, solid v7 → final | self, Reconstruct v7 → final | raw, solid v7 / 41e7b72 / final | raw, Reconstruct v7 / 41e7b72 / final |
| --- | --- | --- | --- | --- |
| M1 (rim) | −0.029 → −0.148 | +0.048 → +0.006 | −0.006 / −0.134 / **−0.007** | −0.020 / −0.070 / **−0.009** |
| M2 | +0.019 → +0.021 | +0.009 → +0.009 | −0.005 / −0.008 / −0.010 | +0.031 / +0.036 / +0.036 |
| M3 | +0.008 → +0.016 | +0.016 → +0.025 | +0.015 / +0.029 / +0.020 | +0.006 / +0.017 / +0.014 |
| M4 (rim) | −0.195 → −0.083 | −0.025 → +0.045 | −0.327 / −0.175 / **−0.035** | −0.184 / −0.087 / **+0.013** |
| M5 | +0.013 → +0.005 | +0.004 → −0.000 | −0.008 / −0.011 / −0.012 | −0.001 / −0.006 / −0.007 |
| M6 (flange) | +0.115 → +0.070 | +0.008 → −0.030 | +0.146 / +0.075 / +0.075 | +0.234 / +0.199 / +0.196 |
| M7 | +0.006 → −0.001 | −0.009 → −0.013 | +0.004 / −0.009 / −0.006 | −0.002 / −0.006 / −0.006 |
| M14 | −0.000 → −0.008 | −0.023 → −0.025 | −0.007 / −0.012 / −0.012 | −0.012 / −0.015 / −0.013 |
| M16 (band) | −0.013 → −0.014 | +0.014 → +0.017 | (+0.10 all) | (+0.10 all) |
| M20 (band) | +0.027 → +0.016 | +0.036 → +0.016 | (+0.22 all) | (+0.20–0.22 all) |
| M21 (band) | −0.001 → +0.003 | +0.049 → +0.039 | (+0.17–0.24 all) | (+0.19–0.24 all) |
| M22 | +0.014 → +0.020 | −0.012 → −0.007 | −0.004 / −0.015 / −0.000 | −0.006 / −0.001 / +0.000 |
| M36 (rim) | −0.455 → −0.339 | −0.375 → −0.405 | (−1.0 all) | (−0.8 to −1.0 all) |
| M43 | −0.135 → −0.088 | −0.002 → −0.015 | +0.128 / +0.091 / +0.091 | +0.241 / +0.348 / +0.219 |
| S2-M4 | −0.088 → −0.054 | −0.007 → +0.034 | −0.304 / −0.219 / −0.220 | −0.055 / −0.006 / −0.013 |
| S2-M7 | +0.007 → +0.008 | −0.033 → −0.028 | −0.006 / +0.012 / +0.011 | +0.151 / +0.149 / +0.151 |
| S2-M12 (rc 12) | +0.077 → +0.091 | +0.328 → +0.262 | −0.350 / −0.359 / −0.360 | −0.295 / −0.284 / −0.313 |
| S2-M20 | −0.008 → −0.007 | −0.004 → −0.019 | +0.009 / +0.003 / +0.003 | +0.185 / +0.165 / +0.166 |
| S2-M44 | −0.005 → +0.007 | +0.110 → +0.125 | +0.031 / +0.058 / +0.058 | +0.165 / +0.177 / +0.182 |
| S2-M61 | −0.059 → −0.025 | −0.019 → −0.002 | – | – |

(The raw column of the inner sites and of M43 also carries the offset between the output and the
raw scan round the cut, e.g. S2-M4 −0.25 mm at 7.5–8.5 mm in the solid, where S3 is equalised.)
Under grazing light (`compare_sites.png`): M1 and M4 show no disc and no dent any more in either
output (v7: a bump with an edge line at M1, a dark disc at M4).

**Real features** (harness, v7 / 41e7b72 / final):

| | Make Solid | Reconstruct |
| --- | --- | --- |
| Inner lattice window, relief std / p99 (mm) | 0.005 / 0.020, all three | 0.013 / 0.038 · 0.071 / 0.418 · **0.022 / 0.049** |
| Bracket screw hole, free radius (mm) | 1.44, all three | 2.20 · 2.42 · **2.20** |
| Boss recess A / B / C / D (mm) | 2.20 / 1.94 / 1.54 / 1.69, all three | C: 0.90 · −0.21 · **0.34**; A 0.25, B 0.79, D −0.08 |
| Bottom crease, peak to peak (mm) | 0.066, all three | 0.093, all three |
| Openings (Reconstruct) | – | 119 · 144 · **134** |

The solid stays watertight, one component, genus 11. Make Solid takes 30 s.

**Tests:** 13 new tests, each fails on 41e7b72: a narrow ridge beside the cut (fill and solid);
noise 0.12–0.2 mm on a sphere; balls R 15–30 and cylinders R 10–15; one stray below the sheet at a
rim site; a sticker over a rim without a side face (thick and thin); a sticker on a rough ring, kept
or filled; a site left to the solve; the fairing's ring check and report; a rim's rolled edge carried
into the cut; a rolled edge inside the cut not folded; Reconstruct on a ball R 20 with its default
settings; the spacing estimate with a few points fewer. `cargo test --release -p scancraft-mesh -p
scancraft`: 89 and 88 (3 ignored) pass; the end-to-end test passes.

**Still not right:**
- **Reconstruct's stray suppression still depends on the support radius.** The spacing is stable
  now (0.647 mm on the Revo merge, the same with or without the marker fills), but a sweep shows the
  laser-lattice result changes with it: one pit at the upper hinge (−0.6 mm, 8 samples) at
  0.645–0.648 mm, −0.3 mm at 0.650–0.660 (as in v7), four pits at 0.640. Boss C's recess flips between −0.27 and +0.90 mm over
  0.64–0.68 mm. v7's good values sat at 0.652 by chance. The cause is a gap in the inner face with
  laser strays behind it: `dense_layers` compares counts within 1.5 spacings, where the face has few
  points. A wider radius (2.5 spacings) removed them in a Python model; not tried in the mesher.
- **M36:** the rim bulge stays (no side face under the overhanging sticker; the patch now ends at the
  rim, but the bulge is in the data round it).
- **M43, S2-M44 in Reconstruct:** faint domes (+0.1 mm against the band), as in v7.
- **S2-M12 (rc 12):** still a large smooth disc, +0.26 mm in Reconstruct.
- **The 1 mm rib beside a cut** (synthetic, h 1, sigma 2, 3 mm outside): the ring's cubic follows the
  rib and lowers the centre by 0.08 mm (old code −0.03, 41e7b72 −0.13).
- **M16 fold count:** 36 triangles above 60° within 8 mm, v7 20. They are needle triangles of
  0.0001 mm² from the marching cubes, not visible.
- **Fills are smoother than the scanned texture round them** (left as decided, no noise added).

### 2.5 What the synthetic scenes show (2026-11-28, branch `agent/synth-scenes`)

30 synthetic scenes with an exact truth (`crates/scancraft-synth`, README there): plates, vents,
a perforated grille, plates with screw holes / bosses / ribs, domes, balls, cylinders, a saddle,
a cup, a block with edges, fine relief, fins, the shell like the Revo part (also standing on its
rim). The simulated scanner was checked against S1-S3's `raw_preview.ply` with
`examples/synth_realism.rs` (4 points per mm², normals 5° median, noise 0.025 mm correlated over
0.55 mm, the stickers' dot measured and their black ring thin, empty or with an empty halo to
9-12 mm, tables with clutter under most parts, passes 0.05-0.15 mm deep or tilted 0.22-0.25°
against each other). The `*_marker_holes` scenes model Revo's Fused export (sites cut to 5.5 mm,
no table). Every scene went through the app's pipeline with its defaults (`synth_eval`, passes at
their true poses) twice: with the app's isolation, and with the table and clutter dropped by the
generator's labels (`--truth-isolation`, to measure the meshers alone).

Results: `E:\Claude\eval\synth\results.txt` (app isolation), `E:\Claude\eval\synth\truth_iso\results.txt`
(label isolation), failures by kind in `failures.txt` beside each, one-page overviews
`E:\Claude\eval\synth\overview.png` and `overview_truth_isolation.png`, renders in `img\` and
`truth_iso\img\`. Make Solid stays watertight on every scene.

**Before the meshers: object isolation on a table with clutter (app isolation, `results.txt`).**

- A plate lying on a table is "plane-like" and is never taken as the object, so a marker block or
  the can is: `perforated_grille` keeps 5 648 of 93 626 points (27 446 part points dropped in each
  pass; both meshes empty), `relief_fine` keeps the can (70 036 part points dropped),
  `plate_screws_bosses_ribs` pass 1 keeps 5 719 (31 137 part points dropped). `flat_markers` and
  `flat_laser_strays` keep everything (table and clutter in the mesh), `flat_vents` falls back to
  everything (Make Solid genus 75 for 5: the table closes the slots).
- It depends on where the clutter stands: moving the marker block 4 mm turned `perforated_grille`
  from "plate kept" into "marker block kept".
- Where the part is found, table pieces at the contact come back with it: Make Solid with 5
  components and genus 11 on `box_edges` (truth 1 and 0), 5 / genus 3 on `cylinder_r15_markers`,
  5 / genus 7 on `thin_fins`, 7 on `shell_two_pass`.
- Without a table (`ball_r15_marker_holes`) the plate under the ball is dropped (37 724 points), as
  in the first run.
- Images: `img\perforated_grille__overview.png`, `img\relief_fine__overview.png`,
  `img\box_edges__overview_tilted.png`, `img\flat_vents__slot-6.png` (table inside the slot).

**Mesher failures to work on, ranked (label isolation, `truth_iso\`).**

1. **Sticker sites with an empty halo (S2's pattern) are left partly open by Reconstruct.** It
   cuts and refills the disc but leaves the empty ring round it: 17 of 21 halo sites on curved
   surfaces fail, 29 % of the site open in the median, up to 54 % (`dome_r60_markers` M4-r18 30 %,
   M8-r33 41 %, `dome_density_steps` M8-r33 54 %, `grazing_cylinder_r30` M4-65 45 %,
   `cylinder_r15_markers` M4-80 20 %). Make Solid closes them but 13 of 21 are off, median
   |centre| 0.17 mm, worst 0.26 (`saddle_markers` M4 +0.20 with a 0.22 mm ring). Sites whose ring
   is only thin (S1) mostly pass in Reconstruct (6 of 58 curved fail). Images:
   `truth_iso\img\dome_r60_markers__M4-r18.png`, `__M8-r33.png`, `truth_iso\img\saddle_markers__M4.png`.
2. **Pass misregistration is kept.** Reconstruct keeps a constant offset (`flat_two_pass_offset`:
   pass-2-only surface -0.12 mm, sites -0.07 to -0.14; shell inside -0.147) and a tilt
   (`flat_two_pass_tilt`: pass-2-only +0.235, sites up to +0.29, top relief std 0.094 mm). Make
   Solid removes the constant offset (handled) but not the tilt (pass-2-only +0.137, sites +0.11 to
   +0.19), and on the shell it moves the outside pass by -0.075 while the inside stays at -0.142:
   0 of 18 features. Images: `truth_iso\img\flat_two_pass_tilt__overview.png`, `__M2.png`,
   `truth_iso\img\shell_two_pass__overview_tilted.png`.
3. **Holes and slots close or come out narrow** (now checked with `through` probes along the axis).
   Reconstruct closes 93 of 99 holes of 1.5 mm (0.52 mm where open) and 6 of 36 of 3 mm (2.80),
   5 of 18 louvre stations (1.76 for 2), slot 1.5 at 3 of 5 stations (1.10 wide), slot 3 2.79, 4
   3.77; 3 of the 4 screw holes in the 2 mm plate have no walls at mid-thickness. Make Solid keeps every hole
   open but narrow at mid-thickness: 1.5 -> 1.00, 3 -> 2.72, slot 1.5 -> 1.32, screw holes 3.2 ->
   2.96-2.97 and 4.5 -> 4.26-4.29 (the shell's 3.2 holes 3.42-3.89, wide). Images:
   `truth_iso\img\perforated_grille__holes-1.5.png`, `truth_iso\img\flat_vents__slot-1.5.png`.
4. **Fine relief is flattened** (`relief_fine`, 0.25 mm spacing): Reconstruct keeps 41 % of the
   0.3 mm emboss, 20 % of the 0.4 mm engraving, 19 % of the 0.2 mm bumps, 74 % of the 0.1 mm
   stripe; Make Solid 74 / 63 / 38 / 84 %. The shell's stripe fails all 8 probes in Make Solid.
   Images: `truth_iso\img\relief_fine__emboss.png`, `__bumps.png`.
5. **Make Solid is thrown by correlated noise on curved marker sites**: thin-ring sites on curved
   surfaces 25 of 58 fail (median |centre| 0.047, at the 0.05 limit), the noise series falls to
   41 of 93 probes at 0.05 mm (Reconstruct 83), and every dome rim fails 7-8 of 8 probes (0.26 mm
   from the edge line; Reconstruct 0.01). Images: `truth_iso\img\dome_r60_noise_005__overview.png`,
   `truth_iso\img\dome_r60_markers__rim.png`.
6. **Laser strays**: Reconstruct's preparation keeps 1 896 of the 1 966 strays the merge leaves
   on the plate (96 %; its stray filter drops none) and 1 646 of 1 930 on the shell, where a stray
   layer is left under 25 % of the window with pits to -0.48 mm. Make Solid's plate in the window
   is wavy (relief std 0.062 mm, pits -0.25) and its made-up bottom lies at the stray depth (inner
   sheet under 100 % of the window). Images: `truth_iso\img\flat_laser_strays__strays.png`,
   `truth_iso\img\shell_two_pass__wall.png`.
7. **Screw bosses**: Reconstruct caps the through pilot (2 extra surfaces, 2.16 mm for 2.5) and
   loses the shell's pilot walls; Make Solid caps the shell's pilots (3 extra surfaces, 1.98-2.07
   mm) and the plate's through pilot. Images: `truth_iso\img\shell_two_pass__boss-1.png`,
   `truth_iso\img\plate_screws_bosses_ribs__boss-2.png`.
8. **Cut-out sites of a Fused export**: Reconstruct leaves all 45 open (`markers_on_surface` keeps a
   marker only with a point within 3 mm; a 5.5 mm cut has none); Make Solid's hole filling leaves
   20 of 45 off (R 15 ball -0.07 to -0.08, flat rings 0.05-0.07, the saddle's band sites). This
   happens only with the Fused export: every listed marker in a raw preview has dot points 0.05-0.9
   mm from its centre. Images: `truth_iso\img\flat_marker_holes__M7.png`,
   `truth_iso\img\ball_r15_marker_holes__M3-45.png`.
9. **Edges**: the 0.5 mm fillet comes out 1.02 (Reconstruct) / 0.86 mm (Make Solid), the box step
   has a 0.30 / 0.17 mm bead at its concave edge, plate edges are rounded 0.25-0.4 mm with beads of
   0.13-0.16 mm in Make Solid (the corrected bead measure; the first run's 0.3-0.6 mm were slot
   walls). Images: `truth_iso\img\box_edges__step.png`, `__edge-fillet-0.5.png`.
10. **Thin fins**: the 0.6 mm fin fails 3 (Reconstruct) / 2 (Make Solid) of 5 probes, edges 0.3-0.4
    mm off. Image: `truth_iso\img\thin_fins__fin-0.6.png`.

**Alignment check:** Auto Align (geometry only) still puts the second pass of every symmetric part
180° round (7 scenes).

**What passes:** kept sites whose ring is only thin, on flat and large-radius surfaces (R 65 barrel
7 of 8 features in both meshes); the dome surfaces and the density steps in Reconstruct; Make Solid's
removal of a constant offset between overlapping passes; Make Solid's topology wherever isolation
is clean (genus exact for the grille's 141 holes, the vents' 5 slots, the plate's 5 holes).

**Corrections to the first run's numbers** (metric fixes): the 0.28-0.6 mm "beads beside slot
edges" were slot walls measured against the far edge's rounding; the knurl's "169 % kept" was a
sign error (it is raised relief); relief is now measured against the base 1.6 mm clear of the
feature (the first run halved the emboss); "preparation keeps 2 781 of 6 753 strays" credited the
merge's resampling to the preparation (it removes 3-15 % of what it receives); the blind pilots
were never measured; the hole probes could not see caps; "pass offset handled" ignored a shared
bias.

## 3. Found by the user in the app

### 3.1 New Project throws away the open project without asking

**Done 2026-11-25** (`apps/scancraft/src/app_tabs.rs`). Several projects open as tabs in the app
bar (name, a dot for unsaved changes, ×). The active project stays in the app's fields and the
others are parked in `ProjectTab`s; switching swaps them and is only allowed while nothing runs
(no job, capture, file dialog or queued import), so a job's result always lands in its own tab.
History is per tab. New Project opens a new tab; Open Project opens into a new tab unless the
current one is empty. Closing a tab with unsaved changes asks Save / Discard changes / Cancel
(Save without a folder opens Save Project and closes after it worked; a failed save keeps the
tab); quitting with unsaved tabs asks Quit without saving / Cancel. Previews, highlights and
outlines no longer mark a project as changed (`preview_generation`). Add Project from another open tab:
done 2026-11-25 (right-click the other tab → Add Passes to …; its passes are rebuilt from their
keyframes, so the two projects never share a tracker). Save All on quit: done 2026-11-25 (the quit question saves every changed project into its
folder, then quits; disabled, with the reason, while a changed project has never been saved). Tests: `new_project_opens_a_tab_and_keeps_the_open_one`,
`closing_a_tab_with_unsaved_changes_asks_first`, `save_and_close_closes_only_after_saving`,
`tabs_cannot_switch_while_a_job_runs`, `previews_do_not_mark_the_project_changed`.

**Now:** clicking *New project* (`ui_shell.rs:42` → `new_project`, `app_project.rs:712`)
clears everything at once, with no confirmation. The work can be got back only with Undo
(it pushes a snapshot), which the user does not expect. The current ribbon tab stays
selected.

**Wanted:** New Project opens a **new tab in the application** with an empty project. The
open project stays in its own tab, untouched.

What this means:

- The app state today is one project. Passes, merged model, mesh, history and settings live
  directly on the app struct. Several projects need a per-project state object, and a tab
  strip that switches between them.
- The scanner connection stays global. Only one tab can capture at a time (scanner rule:
  one client).
- Closing a tab, or anything else that would discard unsaved work, must ask first: Save /
  Discard / Cancel. `ProjectInfo.saved_generation` already exists to detect unsaved changes.
- Undo history should be per tab.
- Consider whether *Add Project* should also be able to pull passes from another open tab.

### 3.2 Confirm / Apply buttons are not in a consistent place

**Done 2026-11-25** (`crates/scancraft-ui/src/kit`). One button-placement rule, shared widgets:
`Button::Danger` for destructive commands, `Actions` + `action_row`/`dialog_footer` put the
primary at the right edge, Cancel to its left, destructive apart at the left (`form.rs`);
`confirm_modal` (the close/quit dialogs), `menu_item` (context menus, every disabled entry says
why), `close_button` (the "close" icon instead of the missing "✕" glyph), `note_with_tip`.
Wording is a written page: `docs/architecture/ui-wording.md` (glossary, Title Case commands, the
three fixed disabled phrases). Rough edges fixed: long notes shortened to one or two lines with
the detail on a tooltip, the Fill Holes form, every disabled ribbon and app-bar button now shows
its reason (`rb` helper, `app.edit_blocked`), and the tab strip ellipsises names at 96–220 pt
with a chevron overflow menu when they do not fit. New `--auto` script steps (`dev.rs`): `tool:`,
`closetool`, `size:`, `rename:`, `askquit`, `selectnode:`, `rclick:`, `hover:` for UI screenshots.
Not done: the camera-controls Reset link stays inline; Add Project from another open tab.
Tests: `action_row_*`, `dialog_footer_insets_the_buttons_by_space_12`,
`a_narrow_row_shrinks_the_destructive_button_without_overlap` (`scancraft-ui`).

When a tool or dialog opens, its confirm button is sometimes on the left of the window and
sometimes on the right.

**Wanted:** one rule for all panels, dialogs and tool popups. Proposal:

- Primary action (Apply / Confirm / OK) at the bottom right, Cancel to its left.
- The same order and spacing everywhere.
- Destructive actions are styled differently.

Implement it as shared widgets in the UI kit rather than per panel, then go through every
panel (`ui_panels.rs`, `ui_shell.rs`, the Clean, Align, Mesh and Export tools, Import and
Save dialogs) and the menus. While doing that, check that menu entries and button labels use
the same words for the same action.

### 3.3 History timeline for Undo / Redo

**Done 2026-11-25.** Layers share their points, mesh and GPU copies (`Arc` in `CloudLayer`,
`MergedLayer`, `MeshLayer`), so a snapshot only costs what the action replaced and undo restores
the very same data (no re-upload). The history is bounded by memory (`HISTORY_BUDGET_BYTES`
2 GiB of geometry the live project no longer uses, counted once however many steps share it)
and `MAX_HISTORY` = 200 steps, instead of 8. The History button next to Undo / Redo opens a
timeline (`ui_history.rs`): every step with its age, the current state highlighted, undone steps
greyed; a click jumps there in one go (`History::jump`, one restore, one frame-restore job). Each
step keeps the action that leads out of it, so the timeline stays complete in both directions.
Alignment and transform changes are listed (they change the passes); view changes are not.
Tests: `a_jump_over_several_steps_keeps_the_timeline_in_both_directions`,
`the_history_keeps_shared_geometry_once_and_trims_by_memory`,
`snapshots_share_the_passes_that_did_not_change`,
`jumping_to_a_timeline_step_undoes_or_redoes_several_actions_at_once`.

**Wanted:** a timeline of the actions that modified the point clouds and mesh. The user can
jump to any earlier or later step directly, not only one step back or forward.

Today:

- `History` (`model.rs:537`) is a two-stack undo/redo with labelled `Snapshot`s, limited to
  `MAX_HISTORY = 8` steps.
- `PassSnapshot.cloud` (`model.rs:524`) is a full `PointCloud` clone, which is why the depth
  is kept small.

What this needs:

- Make snapshots cheap: share unchanged clouds through `Arc` (the surfaces and edits already
  are `Arc`), so that a step only stores what changed.
- Then allow a much deeper history, bounded by memory rather than by a step count.
- A timeline panel listing each step with its label (and time). Click a step to jump there.
  The steps after it stay available as redo until the next new action.
- Show the current position.
- Decide whether alignment / transform changes and view changes appear in the timeline. The
  user's words: every action that modified the point cloud.

### 3.4 Live AI masking in Scan stage: pre-capture gating, lost-track hold, and controls

Deferred follow-up to `agent/sam-fast` (after the multi-object test scene in `agent/ai-test-scene`). Live SAM 2.1 depth masking already exists under **Scan -> AI Mask (Experimental)** (`worker.rs`, `ui_ai.rs`, `ui_viewport.rs`), but three gaps remain before it guarantees zero unwanted background points during live scanning:

- **Pre-capture gating / Live Preview prompting:** `preview_worker` does not accept SAM 2.1 clicks yet, and hitting **Start Capture** starts fusing immediately while `latest_ai` is still `None`, so background points are recorded until the user clicks the object in the Colour inset. Gate recording or allow clicking/locking the object mask in Live Preview first.
- **Hold on lost mask:** in `worker.rs`, if SAM 2.1 loses tracking or dips below `min_iou`, the worker falls back to passing the unmasked depth frame through instead of holding/skipping fusion for that frame, letting background points leak in.
- **Discoverability and live controls:** surface the live AI toggle more prominently when enabled, add a **Reset mask** button, and support Shift/right-click **Exclude** points in the live Colour view.
