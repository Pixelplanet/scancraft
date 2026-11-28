# scancraft-synth: synthetic scan scenes with ground truth

Written 2026-11-28. A development crate (not used by the app). It builds test parts whose true
shape is known exactly, scans them with a simulated scanner and writes the result so that the app
imports it like a Revo Scan project. A `truth.json` says what a mesher must do with each feature,
and `synth_score` checks a mesh against it.

The faults come from what we measured on the real Revo scans (`docs/open-issues.md` §2.3, §2.4):

- marker stickers: 11 mm discs, 0.4 mm proud, flat to r 4.3 mm, back on the surface at r 5.6 mm;
  Revo lists only the 3 mm dot;
- sticker cut-outs: Revo removes the points to r 5.5 mm, sometimes with an empty ring out to 6–9 mm;
- cross-line laser strays: lines of points 0.6–1.0 mm behind the surface, two families 70° apart,
  2 mm wide, 9 and 11 mm apart, 83 % with normals facing the wrong way;
- passes that sit 0.1–0.2 mm too deep;
- grazing fringes that are noisier;
- real features that must be kept: screw holes through thin plates, bosses with pilot recesses,
  ribs, rims, and a painted stripe 0.1 mm high.

## Running it

```powershell
# All scenes (about 45 s, 375 MB). Filter by part of the name; --scale 2 doubles the point
# spacing (4x fewer points); --no-mesh skips reference_mesh.ply.
cargo run --release -p scancraft-synth --example synth_scenes -- E:\Claude\testdata\synthetic
cargo run --release -p scancraft-synth --example synth_scenes -- E:\Claude\testdata\synthetic dome --scale 2
cargo run --release -p scancraft-synth --example synth_scenes -- --list

# Score a mesh. The mesh must be in the part frame: export it from a project with pass 1 as the
# reference.
cargo run --release -p scancraft-synth --example synth_score -- <scene dir> <mesh.ply> [report.json]

# Reconstruct with the app's defaults, then score (passes placed at their true poses).
cargo run --release -p scancraft-synth --example synth_baseline -- <scene dir> [out dir]

cargo test -p scancraft-synth   # 19 tests, about 4 s
```

Keep the generated data out of the repository (`E:\Claude\testdata\synthetic` on the original
PC). Scenes are deterministic: the same code always writes the same bytes.

**In the app:** Import Scan on `<scene>/pass-N/<scene>-pass-N.ply` for each pass. The
`marker_framework.mkf` next to it is read as the pass's marker list, as for a Revo project. Keep
pass 1 as the reference. Passes other than pass 1 are in their own frame (the part turned over or
moved), so Align All has to place them.

## Files per scene

```text
<scene>/
  truth.json                    everything known about the scene (below)
  reference_mesh.ply            true part as a mesh (surface nets, vertices on the exact surface;
                                sharp edges bevelled by up to one cell, 0.15-0.4 mm)
  pass-1/<scene>-pass-1.ply     points: x y z, normals, colour, quality = rays fused into the point
  pass-1/<scene>-pass-1.labels  one byte per point in PLY order: 0 part, 1 sticker, 2 table, 3 laser stray
  pass-1/marker_framework.mkf   Revo's layout (version 3, 104-byte records), pass frame
  pass-2/...
index.json (in the out dir)     every scene: passes, point and marker counts, features, sizes
```

Colours: part grey, sticker ring black, sticker dot white, table brown. Strays are grey like the
part.

## truth.json

- `part`: the exact true part as a tree of signed distance shapes (`src/sdf.rs`; boxes, spheres,
  cylinders, 2D profiles extruded, height fields, CSG, inset, smooth intersection, painted
  coats). `Truth::load(dir)?.part.sdf(p)` gives the signed distance in mm (negative inside).
  Stickers and tables are not part of it.
- `passes[]`: file names, `pass_to_part` (rigid; quaternion xyzw and a 4 x 4 matrix), the full
  pass spec (views, spacing, noise, grazing exponent, maximum incidence, depth bias, offset,
  table, clipping, dropout, density regions, strays), scan statistics, and the listed markers as
  written.
- `stickers[]`: centre on the true surface, normal, radii, height, cut radius, and which passes
  list it.
- `features[]`: `id`, `kind`, `expect` (in words) and `probes`. Each probe has one of three checks:
  - `on {tol}`: the mesh passes within `tol` of the point (edges, corners);
  - `clear {radius}`: no mesh within `radius` (open holes and slots, the inside of a wall, where
    the strays are);
  - `height {normal, expect, tol, range}`: along the line through the point, the mesh crosses at
    `expect` (0 = on the truth) within `tol` (surfaces, marker sites, relief tops, floors).

  Every probe is checked against the truth itself in
  `tests::every_scene_is_consistent_with_its_truth`.

Frames: everything in `truth.json` is in the part frame. Pass 1 is written in the part frame, so
a project with pass 1 as the reference is in the part frame. The faults `depth_bias_mm` and
`offset_mm` are errors and are not part of `pass_to_part`.

## The simulated scanner (`src/scan.rs`)

1. Rays from every view of a pass: a pinhole whose field covers the scene, ray pitch 0.7 × the
   point spacing at the target distance (about 225 mm). Each ray is sphere-traced against the
   part plus the pass's table and stickers, so occlusion is exact.
2. Points seen past the maximum incidence (78° by default) are lost, and coverage thins out over
   the last 6° before it.
3. Noise: Gaussian along the ray, σ = noise / cos(incidence)^grazing, cos clamped at 0.15. The
   depth bias is added along the ray.
4. Fusion: one point per cell of the point spacing and per sheet (the faces of a thin wall do not
   merge); the most frontal ray wins, and `quality` counts the rays in the cell. Density regions
   use their own spacing.
5. Clipping and patchy dropout (value noise).
6. Normals: a plane fit of the noisy neighbours within 2.5 × the spacing on the same sheet,
   facing the scanner. Like a scanner's normals, they are rounded at edges and noisy (median
   error under 4° on the dome). `NormalMode::True` gives the true normals.
7. Markers listed: the stickers with at least 6 points on their dot. Centre 0.16 mm above the
   surface (± 0.03), normal within about 1.5°, radius 3.0 or 3.05, as Revo's are.
8. Sticker cut-outs, then laser strays behind the part's points on the lines, then the pass
   offset.

## Scenes

| Scene | What it is | Passes | Checks |
| --- | --- | --- | --- |
| `flat_plain` | plate 100 x 80 x 4 | top | flat sheet, sharp edges |
| `flat_markers` | the plate with 8 stickers, one 0.4 mm from the edge | top | sites flush |
| `flat_marker_holes` | the 8 stickers cut out (5.5 mm, rings to 6.2–9 mm) | top | holes filled flush |
| `flat_vents` | plate 120 x 80 x 3: through slots 1.5 / 2 / 3 / 4 / 6 mm (30 mm, round ends), grooves 2 x 1 and 1 x 0.5, pocket 16 x 10 x 1.5, stickers 1.5 mm from slots | top, turned over (+0.1 mm deep) | slots open at width, floors kept, edges sharp |
| `perforated_grille` | plate 80 x 60 x 2: 99 holes of 1.5 mm at 3 mm pitch, 36 of 3 mm, six 2 mm louvres | top, turned over | every hole open |
| `flat_two_pass_offset` | plate with stickers, two top passes overlapping 40 mm, pass 2 0.15 mm deep and turned 70° | 2 | one sheet after aligning |
| `flat_laser_strays` | plate with stickers and a 30 mm window of laser strays | top | no pits, nothing inside the plate |
| `dome_r60_markers` | sphere cap R 60 (base 90, height 20), 9 stickers, one reaching the rim | outside | sites flush on the curve, rim |
| `dome_r60_marker_holes` | the same with the stickers cut out | outside | fills follow the curvature |
| `dome_r60_noise_002/005/010` | dome with stickers, noise 0.02 / 0.05 / 0.10 mm | outside | as above |
| `dome_density_steps` | dome, spacing 0.15 / 0.3 / 0.6 mm in three bands | outside | no seams at the steps |
| `ball_r15_markers`, `_marker_holes` | half ball R 15 on a plate; an 11 mm disc spans 42° | outside | small-radius sites, concave corner |
| `cylinder_r15_markers`, `_marker_holes` | cylinder R 15 lying down, stickers at 0 / 45 / 80° | outside | sites on a tight curve, end edges |
| `cylinder_r65_markers`, `_marker_holes` | barrel panel R 65 | outside | large-radius sites |
| `saddle_markers`, `_marker_holes` | saddle x²/300 − y²/220 with a curvature band (0.8 mm ridge), two stickers on its flank | outside | band kept, sites on the band |
| `grazing_cylinder_r30` | cylinder R 30 seen only from above, noise / cos^1.5, coverage to 82° | above | grazing flanks, open boundary |
| `shell_two_pass` | shell like the Revo part: 120 x 80 x 35, wall 2.5, domed top, stripe 0.1 mm, screw holes, 2 bosses with pilot recesses, rib | outside; inside (turned over, 0.15 mm deep, strays, noisier) | wall, stripe, holes, bosses, rib, rim |
| `cup_rim_partial` | cup R 40, wall 2.5, open rim, stickers just below the rim (3 cut out) | all round | rim kept and open, partial sites |
| `box_edges` | block 70 x 50 x 30: sharp / fillet 1.5 / fillet 0.5 / chamfer 1 edges, chamfer 2, step; on tables with table stickers | upright, turned over | edges, table removed |
| `relief_fine` | plate with embossed strokes 0.8 x 0.3, engraved 0.6 x 0.4, knurl 1.2 / 0.25, bumps 0.2, stripe 0.1 | top (0.25 mm, noise 0.02) | relief kept |
| `plate_screws_bosses_ribs` | plate 100 x 70 x 2, screw holes 3.2 and 4.5, 3 bosses with pilot recesses (one through), 2 ribs | top, turned over (+0.05 mm) | holes open, recesses, ribs |
| `thin_fins` | fins 0.6 / 1.0 / 1.5 / 2.5 mm thick, 12 mm high | all round | both faces of each fin |

## Baseline (2026-11-28, `agent/mesh-fixes` aafe808, `synth_baseline`)

The app's Reconstruct with its default settings, passes at their true poses (no alignment, no
isolation), scored against the truth. Results are in `E:\Claude\eval\synth\baseline\<scene>`.
Probe failures that are expected because of how the baseline runs:

- `flat_two_pass_offset` and `shell_two_pass`: the 0.15 mm offset is not removed, because the
  baseline does not align or equalise the passes;
- `box_edges`: the table is still in the result, because the baseline does not isolate the part.

Probe failures that show what the mesher does:

- **Sharp edges**: rounded by 0.2–0.45 mm at the 0.7 mm cell (plate edges, slot and hole edges,
  rims, fin tops).
- **Blind grooves** of 2 x 1 and 1 x 0.5 mm: filled (floor 0.15–0.2 mm too high, groove not
  open).
- **Fine relief**:
  - embossed strokes keep only half their 0.3 mm;
  - engraved strokes keep about 0.15–0.25 of their 0.4 mm;
  - bumps keep 0.05 of their 0.2 mm;
  - knurl peaks lose 0.18 mm.
- **1.5 mm holes** at 3 mm pitch: half of them closed.
- **Marker sites on R 15**: 0.05–0.09 mm low; on the saddle's band 0.08–0.15 mm high.
- **Some cut-out sites with a ring**: left open (no surface).
- **0.6 mm fin**: its top is 0.48 mm low.

The dome, the R 65 barrel and the noise series pass every probe, including all marker sites.

## Evaluation as the app runs it (`synth_eval`, 2026-11-28)

`synth_eval` runs every scene through the app's pipeline with the app's default settings and
measures Reconstruct and Make Solid against the truth. The app is a binary crate with a GPU
interface, so the pipeline is copied on library calls in `src/app_pipeline.rs`; the table at the
top of that file names the app function each step copies (keep them in step when the app
changes):

1. Import Scan of each pass with the `marker_framework.mkf` beside it;
2. object isolation on import (the app's default, "whole scene" off; `--whole-scene` turns it
   off as the app's setting does);
3. passes placed at their **true poses** (`pass_to_part`), so alignment is out of the mesh
   evaluation; the pass faults (offsets, depth bias, strays) stay;
4. an alignment check: the app's Auto Align (`align_robust`) of every further pass against pass
   1, compared with the true pose (`--no-align` skips it);
5. Merge (pitch 0.5 mm, outliers 12 / 3.0), Reconstruct (listed markers on the surface, prepare,
   cell 0.7, detail 3, smoother 4 / 0.7, marker fairing), Make Solid (cell 0.4, markers removed).

```powershell
cargo run --release -p scancraft-synth --example synth_eval -- E:\Claude\testdata\synthetic E:\Claude\eval\synth [filter] [--no-align] [--no-solid] [--whole-scene]
python scripts\render_features.py E:\Claude\testdata\synthetic E:\Claude\eval\synth [filter] [--only-failed]
python scripts\compact_table.py E:\Claude\eval\synth\results.json
```

Each scene runs in a child process (its own peak memory). Output: `results.txt` (table,
isolation losses, every failing feature with its reasons and image), `results.json`, and per
scene `eval.json`, `reconstruct.ply` / `solid.ply` (vertex property `dev` = signed distance to the
true part) and `merged.ply`. The renders (`img/<scene>__<feature>.png`, plus `__overview` and
`__overview_tilted`) show truth / Reconstruct / Make Solid in grazing light (12° above the
surface) and the merged points / Reconstruct / Make Solid coloured by signed distance (±0.1 mm).

What is measured (`src/eval.rs`), always against the exact true shape:

- **global**: output -> truth (area weighted, over vertices within 0.75 mm of a measured point;
  the rest is reported as made-up surface), truth -> output over the true surface the passes saw
  (mean, rms, p50/p95/p99, max, share with no mesh within 1 mm), area outside the part (tables)
  and 0.3–2 mm inside the material (stray layers), topology (watertight, manifold, components,
  genus against the truth's genus, boundary loops), run time and peak working set per stage;
- **marker sites / cut-outs**: centre deviation, rms and max over the sticker disc, share left
  open, radial profile, the largest ring against the surrounding surface and the largest step
  between 0.5 mm rings, and how much of the true curvature the fill follows (1 = as the surface,
  0 = flat; on curved surfaces only);
- **through slots and holes**: open at each station (no surface on the axis through the plate),
  width at mid-thickness and just under the top; **blind grooves and pockets**: share of the
  depth kept, width; **boss pilots**: true vs meshed surfaces along the pilot axis, pilot width;
- **relief**: share of the height (depth) kept at each top probe, against the base round it;
- **edges**: distance of the mesh from the edge line, the rounding radius that distance means
  (for the edge's angle; for fillets the radius found), and the bead beside the edge (highest
  bulge 0.3–3 mm from the edge against the face's level further away);
- **laser strays**: stray points left after Merge and after Reconstruct's preparation, share of
  the window with a layer 0.3–1.4 mm inside the material, pits;
- **pass offset**: mean signed distance on surface seen by pass 1 only, by other passes only and
  by both; the step between them.

Tests (`src/eval_tests.rs`): three small scenes of their own run in about 2 s in a debug build
(`cargo test -p scancraft-synth`): a plate with a kept and a cut-out sticker and a 3 mm slot, a
dome with a kept and a cut-out sticker, a plate with laser strays. `known_failure_*` tests
(ignored) assert the ideal where the pipeline falls short now. `synth_catalogue` (ignored, about
4 min in release) runs the whole catalogue from `SCANCRAFT_SYNTH_DATA` and fails when a scene
fails more features or passes fewer probes than on 2026-11-28, or Make Solid is not watertight:

```powershell
$env:SCANCRAFT_SYNTH_DATA = "E:\Claude\testdata\synthetic"
cargo test --release -p scancraft-synth --lib -- --ignored synth_catalogue
cargo test -p scancraft-synth --lib -- --ignored known_failure
```

### Findings of the first run (2026-11-28, `agent/synth-scenes` on `agent/mesh-fixes` aafe808)

Results in `E:\Claude\eval\synth` (isolation on, as the app's default) and
`E:\Claude\eval\synth\whole_scene` (isolation off). Not fixed here.

- **Cut-out marker sites are never used.** Revo lists the stickers it cut out, but
  `markers_on_surface` keeps a marker only with a point within 3 mm of its centre; a cut-out has
  none within 5.5 mm. Reconstruct leaves 45 of the 48 cut-outs open (6 scenes; the 3 on the cup
  count only because the inside of its 2.5 mm wall is within 3 mm); Make
  Solid closes them with its hole filling, without the marker fairing (dents of 0.05–0.15 mm on
  the R 15 ball, a 0.2 mm bump on the saddle).
- **Isolation removes real part** when a part has no table under it: the plate under the R 15
  ball (39 500 points), the plate under the fins (96 000 of 109 000), half of the grazing
  cylinder, and the whole plate of the turned-over pass of `plate_screws_bosses_ribs`.
- **Make Solid of a plate seen from one side** guesses the wall: 3.0 mm normally, but the laser
  strays (`flat_laser_strays`, 0.97 mm) or the relief (`relief_fine`, 0.82 mm, genus 36 instead
  of 0) set it, so the made-up bottom lies where the strays were or cuts through the relief.
- **Laser strays**: Reconstruct's preparation keeps 2 781 of 6 753 (a layer under 29 % of the
  window, pits to −0.2 mm).
- **Pass offsets**: Make Solid estimates and removes the 0.15 mm offset of an overlapping pass
  (`flat_two_pass_offset`, shift 0.143 mm); Reconstruct keeps it (sites −0.15 mm). On the shell
  (inside pass 0.15 mm too deep, no overlap) Make Solid shifts the *outside* pass by −0.076 mm.
- **Holes and slots**: Reconstruct caps 72 of 99 1.5 mm holes and narrows slots (1.5 -> 1.0 mm,
  3 -> 2.4 mm, walls missing on the 4 mm slot); Make Solid keeps them open but 0.3–0.4 mm narrow
  at mid-thickness and wide at the top (edges rounded).
- **Edges**: rounded (fillet 0.5 comes out 1.05–1.1 mm), and Make Solid leaves a bead 0.05–0.3 mm
  proud within 1–2 mm of edges and chamfers.
- **Thin plate print-through**: on the 2 mm `plate_screws_bosses_ribs` Make Solid shows the ribs
  and bosses underneath as 0.05–0.1 mm dents on the top.
- **Auto Align** (geometry only) puts the second pass of every symmetric part (plates, block,
  shell) 180° round: the parts here are symmetric and their marker lists were not used.
- **What passes**: domes (all noise levels, density steps) and R 65 barrels — every kept marker
  site flush and following the curvature in both meshes, Make Solid watertight with the right
  genus everywhere except `box_edges` (genus 1 for 0) and `relief_fine`; the grille's genus 141
  is exact.

## Limits

- No speckle or stripe structure in the noise. The noise is white along the ray, with no
  correlation between neighbours, and the fine hatching of the real scans is not modelled.
- Each pass is one fused cloud. There is no per-frame tracking jitter, and no misregistration
  inside a pass beyond `offset_mm` and `depth_bias_mm`.
- Strays are placed only along the view ray behind part points. There are no strays in front of
  the surface and no ghost layers.
- Normals come from the fused cloud (sheet-aware), not from per-view depth images.
- Stickers follow the surface they are on. The edge ramp is a smoothstep, and there is no paper
  edge.
