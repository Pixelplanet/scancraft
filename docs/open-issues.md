# Open issues and requested features (2026-11-24)

Collected after merging branch `feature/clean-per-pass-meshing` into `main` (commit 3151b3d:
per-pass cleaning, meshing rework, Fill Holes, file dialogs). **Fixed on 2026-11-25: 1.1, 1.2,
1.3 and 1.6** (marked below); the rest is open. Sections 1 and 2 come from a review and a measurement run on the original PC. Section 3
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

- **Better:** large plain areas. The old mesher's woven grain and scaly patches are gone,
  and the fit to the points is closer.
- **Worse: small raised detail.** The moulded lettering inside the shell is readable with
  the old mesher and becomes a field of pits and spikes with the new one. This is the main
  problem before RIMLS can replace the old mesher. Check 1.3 first. Also check the normal
  refit and the robust weights on small relief.
- **Worse: open edges** are stair-stepped, aligned to the voxel grid; the old ones are
  ragged but rounded.
- **Pass overlap:** where two passes meet near the rim, the new mesher shows a 1–2 mm step.
  The old one blends it away. It is probably a real alignment offset the old mesher was
  averaging, but it shows as a crease.
- **Thin wall:** both meshers keep the 3 mm wall as two separate surfaces.
- **Fill Holes** (perimeter ≤ 60 mm) closed 82 868 of 82 926 holes in 1 s. Every filled
  pinhole becomes a flat patch with a crease around it, so the result looks crumpled
  around detail, and roughness rises from 2.00° to 2.52° (in the 1.0 mm run). It should not
  be used to hide the pinholes.
- The roughness number does not see holes or small pits. Always look at boundary loops and
  at the renders as well.

## 3. Found by the user in the app

### 3.1 New Project throws away the open project without asking

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
