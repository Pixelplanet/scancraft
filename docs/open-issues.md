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
with pen marks) and a synthetic relief. **Still to do on the original PC:** run `mesh_project`
on the three-scan project and look at the moulded lettering.

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
   with a plain area 1.4× as rough).
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
The moulded pad in this scan (8 mm across, 0.25–0.3 mm high, flat) comes out as a flat pad; the
ring-with-bars the old mesher shows there is its grain, not the part.

The list below is the state before these fixes.

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

## 4. Planned for later

### 4.1 A real project file that opens with a double-click (user, 2026-11-26)

**Wanted:** the user clicks a project file in Explorer and it opens in ScanCraft. Today a project
is a folder (`project.json`, `pass-<id>/…`, `outputs/`, `crates/scancraft-io/src/project.rs`),
and Open Project asks for that folder. Keep the format simple: plain, readable files, no
encryption, nothing proprietary unless a requirement forces it. The user suggests two forms: a
simple project file that lives in the project folder next to the scan data, and a full bundle
that contains all the scan data in one file. Not started; needs a decision on the points at the
end.

**Form A: project file in the project folder** (for example `Part.scancraft` next to `pass-1/`,
`outputs/`). The file is today's `project.json` under a registered extension, with relative paths
only.

| Pros | Cons |
| --- | --- |
| Saving stays incremental: only changed passes are rewritten (a project is easily 1–10 GB) | Moving or sharing means taking the whole folder |
| No copy or extraction on open; large files stay memory-mappable | The file can be separated from its data: Open must say clearly what is missing |
| Readable and diffable, easy to repair by hand or by an agent | Two files with the same name in one folder would share the data: one project per folder |
| Backups and cloud sync only move what changed | |
| Old projects migrate trivially (`project.json` is still read; save writes the new name) | |

**Form B: bundle** (one file with everything). Proposal: a plain ZIP64 of the same folder layout
(like `.3mf` or `.docx`), entries *stored* (point data barely compresses and stored entries can be
read in place), the project JSON as the first entry, an optional SHA-256 per entry in it for
integrity.

| Pros | Cons |
| --- | --- |
| One file to send, archive or attach; nothing can go missing | Every save rewrites the whole file (minutes for GBs), needs twice the disk space and a temp-file + rename to be crash-safe |
| Standard tools open it (any ZIP program); no custom container | Cloud sync and backups re-upload the whole file after each save |
| Self-describing: the file says what it contains and checks itself | Editing in place is impractical: it has to be unpacked to a working folder anyway |
| | Antivirus scans of large archives slow opening |

**Recommendation (to discuss):** A is the working format and the default. B is for exchange:
*Save as Bundle…* writes it from a project, and opening a bundle unpacks it into a project folder
the user picks (or a cache folder, with a prompt to choose a place on the first save). So nobody
ever edits a multi-GB archive in place. Raw stream recordings (several MB/s) stay out of bundles
unless the user ticks them in.

**Needed for either form:**
- **File association:** the installer registers the extension per user (HKCU `Software\Classes`:
  ProgID, icon, `scancraft.exe "%1"`). The app takes a project path as its argument.
- **Single instance:** a second double-click must open the project as a tab in the running app
  (local pipe or socket), not start a second app. Two apps could both try to talk to the scanner,
  which breaks the one-client rule (HANDOFF §3).
- **Project lock:** a lock file in the project folder so two apps (or PCs, via a share) don't save
  over each other.
- **Robustness:** atomic saves (write temp, then rename), a format version in the file, unknown
  fields kept on save, and old folder projects keep opening.
- **Later:** an Explorer thumbnail (a preview image in the folder or bundle).

Precedent: Revo Scan 6 also uses a JSON project file (`.revox`) next to its data folders.

**Decided by the user (2026-11-26): build it as recommended.**
- Extensions: `.scancraft` for the project file (form A) and `.scancraftz` for the bundle
  (form B).
- **Opening a bundle works on a working folder (a temporary form A):**
  - Open unpacks the bundle into a working folder, and all work goes there.
  - *Save* writes everything back into the bundle.
  - Closing the tab removes the working folder.
- **Bundle contents are chosen in a settings page** (future Settings menu). Each item says what
  it is needed for (see below).

**Design notes for the working folder:**
- **Where:** a working-folder root set in Settings. It defaults to the user's local app data,
  but scans are large, so it must be easy to move to a big drive. Show the space needed (the
  bundle's size) and refuse with a clear message if it does not fit.
- **Unpack lazily:** bundle entries are stored, not compressed, so large pass files can be read
  straight from the bundle. Only files that change are written to the working folder
  (copy-on-write).
- **Save:** write a new bundle next to the old one (`*.scancraftz.tmp`), then rename it over
  the old one.
  - Changed files come from the working folder.
  - Unchanged entries are copied raw from the old bundle, without decompressing anything.
  - That keeps saves at disk speed and crash-safe.
  - The working folder stays until the tab closes.
- **Crash recovery:** a working folder records which bundle it belongs to and when that bundle
  was last saved. On the next start the app offers to recover unsaved work (Open / Discard)
  rather than deleting it.
- **Save As Project Folder:** turns an opened bundle into a normal form-A project
  (and *Save as Bundle…* goes the other way).
- **The bundle is locked while it is open** (the same lock as for project folders), so a second
  app or PC cannot open it for writing.

**Bundle contents (settings, with the explanation shown to the user):**

| Item | Needed for | Default |
| --- | --- | --- |
| Project file, pass surfaces and alignment, cleaned surfaces, object labels | Opening the project at all, merging, meshing | always |
| Keyframes (depth frames and poses) | Continuing a pass, discarding frames, re-tracking, Same Position start, adding passes from it | on |
| Colour keyframes | Colour, AI object segmentation | on |
| Merged model and mesh | Opening with results ready; otherwise Merge / Reconstruct again (minutes) | on |
| Marker maps | Make Solid marker removal, marker display | on |
| Raw stream recordings | Replaying a scan for analysis or bug reports only; several MB per second | off |
