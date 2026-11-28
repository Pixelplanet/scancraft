# scancraft-synth: synthetic scan scenes with ground truth

Written 2026-11-28. A development crate (not used by the app). It builds test parts whose true
shape is known exactly, scans them with a simulated scanner and writes the result so that the app
imports it like a Revo Scan project. A `truth.json` says what a mesher must do with each feature;
`synth_eval` runs every scene through the app's pipeline and measures Reconstruct and Make Solid
against the truth.

The simulated scanner is checked against Revo's own output with the same numbers
(`synth_realism`, below). What it reproduces, all measured on the real scans (S1 outside, S2 and
S3 inside, `raw_preview.ply`; `docs/open-issues.md` §2.3, §2.4):

- **density**: about 4 points per mm² (S1-S3: 4.8 / 2.9 / 3.5), blue-noise spacing with some
  near duplicates, independent of how the surface is turned;
- **normals** from single depth images, not from the true surface: median 5°, p90 10° off a
  2 mm plane fit (S1-S3: 4.7-5.3°, 10-18°), blended at edges;
- **noise** 0.025 mm with 80 % of it correlated over 0.55 mm (neighbour correlation 0.35 at
  0.2-0.5 mm; S1-S3 0.25-0.42), straight hatching of ±0.012 mm, and streaks of up to ±0.5 mm
  along the laser lines in grazing fringes;
- **marker stickers**: 11 mm discs, 0.4 mm proud, flat to r 4.3 mm, back on the surface at
  r 5.6 mm; Revo lists only the 3 mm dot. In a raw scan the dot is measured and the black ring
  round it is thin (S1) or empty (S3), some with an empty halo out to 9-12 mm (S2);
- **Revo's Fused export** (the `*_marker_holes` scenes): every sticker site cut out to exactly
  5.5 mm, 0.23 mm spacing (10.8 points per mm²), smoother, no table;
- **tables and clutter**: most parts lie on a table with a marker block, a can and a box
  (65-70 % of the points; S2: 71 %); the table shows through every hole and slot; parts stand on
  their rim, their bosses or their face as they would;
- **cross-line laser strays**: lines of points 0.6-1.0 mm behind the surface, two families 70°
  apart, 2 mm wide, 9 and 11 mm apart, 83 % with normals facing the wrong way;
- **pass errors**: passes that sit 0.05-0.15 mm too deep, and an alignment's residual tilt
  (0.22-0.25°; Align All leaves 0.19-0.27°), so the gap between passes changes across the part;
- real features that must be kept: screw holes through thin plates, bosses with pilot recesses,
  ribs, rims, a painted stripe 0.1 mm high, fine relief.

## Running it

```powershell
# Generate all scenes (about 2 min, 376 MB). Filter by part of the name; --scale 2 doubles the
# point spacing (4x fewer points); --no-mesh skips reference_mesh.ply.
cargo run --release -p scancraft-synth --example synth_scenes -- E:\Claude\testdata\synthetic
cargo run --release -p scancraft-synth --example synth_scenes -- --list

# How real a cloud looks: a synthetic scene (every pass) or any PLY, e.g. Revo's raw_preview.ply
# (the marker_framework.mkf beside it gives the sticker-ring densities with --rings).
cargo run --release -p scancraft-synth --example synth_realism -- E:\Claude\testdata\synthetic\dome_r60_markers --rings
cargo run --release -p scancraft-synth --example synth_realism -- "<Revo project>\raw_preview.ply" --rings

# Evaluate every scene the way the app runs it (about 5 min), then render.
cargo run --release -p scancraft-synth --example synth_eval -- E:\Claude\testdata\synthetic E:\Claude\eval\synth [filter] [--no-align] [--no-solid] [--whole-scene] [--truth-isolation]
python scripts\overview.py E:\Claude\testdata\synthetic E:\Claude\eval\synth            # overview.png
python scripts\render_features.py E:\Claude\testdata\synthetic E:\Claude\eval\synth [filter] [--only-failed]
python scripts\failure_summary.py E:\Claude\eval\synth\results.json                    # failures by kind
python scripts\compact_table.py E:\Claude\eval\synth\results.json

# Score one mesh (in the part frame: export from a project with pass 1 as the reference).
cargo run --release -p scancraft-synth --example synth_score -- <scene dir> <mesh.ply> [report.json]

cargo test --release -p scancraft-synth   # 26 tests, about 15 s
$env:SCANCRAFT_SYNTH_DATA = "E:\Claude\testdata\synthetic"
cargo test --release -p scancraft-synth --lib -- --ignored synth_catalogue   # about 5 min
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
  pass-1/<scene>-pass-1.labels  one byte per point in PLY order: 0 part, 1 sticker on the part,
                                2 table, clutter and the stickers on them, 3 laser stray
  pass-1/marker_framework.mkf   Revo's layout (version 3, 104-byte records), pass frame; lists the
                                part's stickers and those on the pass's table and marker block
  pass-2/...
index.json (in the out dir)     every scene: passes, point and marker counts, features, sizes
```

Colours: part grey, sticker ring black, sticker dot white, table brown. Strays are grey like the
part.

## truth.json

- `part`: the exact true part as a tree of signed distance shapes (`src/sdf.rs`; boxes, spheres,
  cylinders, 2D profiles extruded, height fields, CSG, inset, smooth intersection, painted
  coats). `Truth::load(dir)?.part.sdf(p)` gives the signed distance in mm (negative inside).
  Stickers, tables and clutter are not part of it.
- `passes[]`: file names, `pass_to_part` (rigid; quaternion xyzw and a 4 x 4 matrix), the full
  pass spec (views, spacing, noise model, grazing exponent, maximum incidence, depth bias,
  offset, pose error, table and clutter, clipping, dropout, density regions, strays, streaks,
  fusion, normal window), scan statistics, and the listed markers as written.
- `stickers[]`: centre on the true surface, normal, radii, height, how the dot, ring and halo
  scan (`dot_keep`, `ring_keep`, `ring_radius`, `halo_radius`, `halo_keep`), cut radius, and
  which passes list it.
- `features[]`: `id`, `kind`, `expect` (in words) and `probes`. Each probe has one of four checks:
  - `on {tol}`: the mesh passes within `tol` of the point (edges, corners);
  - `clear {radius}`: no mesh within `radius` (open holes and slots, the inside of a wall, where
    the strays are);
  - `through {axis, half_length}`: the line along the axis crosses no mesh (a hole or slot open
    all the way through; a cap or a table left at its bottom fails, which a `clear` sphere at
    mid-depth does not see);
  - `height {normal, expect, tol, range}`: along the line through the point, the mesh crosses at
    `expect` (0 = on the truth) within `tol` (surfaces, marker sites, relief tops, floors).

  Every probe is checked against the truth itself in
  `tests::every_scene_is_consistent_with_its_truth`.

Frames: everything in `truth.json` is in the part frame. Pass 1 is written in the part frame, so
a project with pass 1 as the reference is in the part frame. The faults `depth_bias_mm`,
`offset_mm` and `pose_error` are errors and are not part of `pass_to_part`.

## The simulated scanner (`src/scan.rs`)

1. Rays from every view of a pass: a pinhole whose field covers the scene (and 28 mm round it
   when the pass has a table), ray pitch 0.7 × the point spacing at the target distance (about
   225 mm). Each ray is sphere-traced against the part plus the pass's table, clutter and
   stickers, so occlusion is exact.
2. Points seen past the maximum incidence (78° by default) are lost, and coverage thins out over
   the last 6° before it.
3. Depth error along the ray (`NoiseModel`): σ = noise / cos(incidence)^grazing (cos clamped at
   0.15), 80 % of the variance a stationary Gaussian field with correlation exp(-r² / 0.55²)
   (sum of 48 random plane waves), 20 % white; plus hatching (a sine of ±0.012 mm, period
   1.5 mm, one direction per pass), streaks along the laser lines past 60° incidence (`Streaks`)
   and the depth bias.
4. Normals (`NormalMode::DepthImage`): a plane fit over the 3 x 3 neighbours in the view's depth
   image (5 x 5 for the Fused export), not across depth jumps, from a single frame's depth
   (0.04 mm of extra frame noise), facing the scanner. Nothing of the true surface is used, so
   they blend at edges and carry the noise. `NormalMode::True` gives the true normals.
5. Fusion (`Fusion`): blue-noise thinning, the most frontal hits first (random order within 0.05
   of cosine): no two points of a sheet (surfaces facing the same way) closer than the spacing
   (0.4 mm); 80 % of the points kept at random; 12 % keep a near duplicate from another view.
   `quality` counts the rays fused into a point. Density regions use their own spacing.
6. Clipping and patchy dropout (value noise).
7. Markers listed: the stickers with at least 6 points on their dot, before any loss on them.
   Centre 0.16 mm above the surface (± 0.03), normal within about 1.5°, radius 3.0 or 3.05, as
   Revo's are.
8. Losses on the stickers (`Sticker::keep_share`: dot, black ring, halo; `RingStyle` Thin /
   Empty / Halo as S1 / S3 / S2), sticker cut-outs (Fused export), laser strays behind the part's
   points on the lines, then the pass offset and pose error.

## Scenes

Raw-scan scenes (`*_markers` and the rest) use the raw-preview model and, where the part lies on
something, a table with clutter; the `*_marker_holes` scenes are the same parts as Revo's Fused
export gives them.

| Scene | What it is | Passes | Checks |
| --- | --- | --- | --- |
| `flat_plain` | plate 100 x 80 x 4, no table (as S1) | top | flat sheet, sharp edges |
| `flat_markers` | the plate on a table with clutter, 8 stickers (rings thin / empty / halo), one 0.4 mm from the edge | top | sites flush |
| `flat_marker_holes` | the plate as a Fused export: 8 sites cut to 5.5 mm | top | holes filled flush |
| `flat_vents` | plate 120 x 80 x 3: through slots 1.5 / 2 / 3 / 4 / 6 mm (30 mm, round ends), grooves 2 x 1 and 1 x 0.5, pocket 16 x 10 x 1.5, stickers 1.5 mm from slots; on a table both ways | top, turned over (+0.1 mm deep) | slots open at width, floors kept, edges sharp |
| `perforated_grille` | plate 80 x 60 x 2: 99 holes of 1.5 mm at 3 mm pitch, 36 of 3 mm, six 2 mm louvres; on a table both ways | top, turned over | every hole open |
| `flat_two_pass_offset` | plate with stickers, two top passes overlapping 40 mm, pass 2 0.15 mm deep and turned 70° | 2 | one sheet |
| `flat_two_pass_tilt` | the same, pass 2 tilted 0.25° and raised 0.1 mm (gap 0.01-0.19 mm across the overlap) | 2 | one sheet, no crease |
| `flat_laser_strays` | plate on a table with stickers and a 30 mm window of laser strays | top | no pits, nothing inside the plate |
| `dome_r60_markers` | sphere cap R 60 (base 90, height 20) standing on its rim on a table, 9 stickers, one reaching the rim | outside | sites flush on the curve, rim |
| `dome_r60_marker_holes` | the dome as a Fused export | outside | fills follow the curvature |
| `dome_r60_noise_002/005/010` | dome on a table, noise 0.02 / 0.05 / 0.10 mm | outside | as above |
| `dome_density_steps` | dome on a table, spacing 0.25 / 0.4 / 0.8 mm in three bands | outside | no seams at the steps |
| `ball_r15_markers`, `_marker_holes` | half ball R 15 on a plate (raw: on a table); an 11 mm disc spans 42° | outside | small-radius sites, concave corner |
| `cylinder_r15_markers`, `_marker_holes` | cylinder R 15 lying down (raw: on a table, line contact), stickers at 0 / 45 / 80° | outside | sites on a tight curve, end edges |
| `cylinder_r65_markers`, `_marker_holes` | barrel panel R 65 | outside | large-radius sites |
| `saddle_markers`, `_marker_holes` | saddle x²/300 − y²/220 with a curvature band (0.8 mm ridge), two stickers on its flank | outside | band kept, sites on the band |
| `grazing_cylinder_r30` | cylinder R 30 on a table seen only from above, noise / cos^1.5, streaks past 60°, coverage to 82° | above | grazing flanks, open boundary |
| `shell_two_pass` | shell like the Revo part: 120 x 80 x 35, wall 2.5, domed top, stripe 0.1 mm, screw holes, 2 bosses with pilot recesses, rib | outside (no table, as S1); inside (turned over onto a table with clutter, 0.15 mm deep, tilted 0.22°, strays, streaks) | wall, stripe, holes, bosses, rib, rim |
| `shell_on_rim` | the shell standing on its rim on a table with clutter | outside | isolation keeps the rim; holes, stripe |
| `cup_rim_partial` | cup R 40, wall 2.5, open rim, on a table, stickers just below the rim (one with a halo) | all round | rim kept and open, partial sites |
| `box_edges` | block 70 x 50 x 30: sharp / fillet 1.5 / fillet 0.5 / chamfer 1 edges, chamfer 2, step; on tables with clutter | upright, turned over | edges, table removed |
| `relief_fine` | plate on a table: embossed strokes 0.8 x 0.3, engraved 0.6 x 0.4, knurl 1.2 / 0.25, bumps 0.2, stripe 0.1; 0.25 mm spacing (ScanCraft's own density) | top | relief kept |
| `plate_screws_bosses_ribs` | plate 100 x 70 x 2, screw holes 3.2 and 4.5, 3 bosses with pilot recesses (one through), 2 ribs; standing on its bosses, then on its face | top, turned over (+0.05 mm) | holes open, recesses, ribs |
| `thin_fins` | plate on a table, fins 0.6 / 1.0 / 1.5 / 2.5 mm thick, 12 mm high | all round | both faces of each fin |

## Evaluation as the app runs it (`synth_eval`)

`synth_eval` runs every scene through the app's pipeline with the app's default settings. The
app is a binary crate with a GPU interface, so the pipeline is copied on library calls in
`src/app_pipeline.rs`; the table at the top of that file names the app function each step copies
(keep them in step when the app changes):

1. Import Scan of each pass with the `marker_framework.mkf` beside it;
2. object isolation on import (the app's default; `--whole-scene` turns it off as the app's
   setting does; `--truth-isolation` is not the app: it drops the table and clutter by the
   generator's labels, keeping the app's support plane, so the meshers are measured on a perfect
   isolation);
3. passes placed at their **true poses** (`pass_to_part`); the pass faults (offsets, depth bias,
   pose error, strays) stay;
4. an alignment check: the app's Auto Align (`align_robust`) of every further pass against pass
   1, compared with the true pose (`--no-align` skips it);
5. Merge (pitch 0.5 mm, outliers 12 / 3.0), Reconstruct (listed markers on the surface, prepare,
   cell 0.7, detail 3, smoother 4 / 0.7, marker fairing), Make Solid (cell 0.4, markers removed).

Each scene runs in a child process (its own peak memory). Output: `results.txt` (table,
isolation losses, every failing feature with its reasons and image), `results.json`, and per
scene `eval.json`, `reconstruct.ply` / `solid.ply` (vertex property `dev` = signed distance to the
true part) and `merged.ply`. `overview.png` has one row per scene (truth / Reconstruct / Make
Solid in grazing light and the key numbers); the feature renders
(`img/<scene>__<feature>.png`, plus `__overview` and `__overview_tilted`) show truth /
Reconstruct / Make Solid in grazing light and the merged points / Reconstruct / Make Solid
coloured by signed distance (±0.1 mm).

What is measured (`src/eval.rs`), always against the exact true shape:

- **global**: output -> truth (area weighted, over vertices within 0.75 mm of a measured point;
  the rest is reported as made-up surface), truth -> output over the true surface the passes saw
  (mean, rms, p50/p95/p99, max, share with no mesh within 1 mm), area outside the part (tables)
  and 0.3–2 mm inside the material (stray layers), topology (watertight, manifold, components,
  genus against the truth's genus, boundary loops), run time and peak working set per stage;
- **marker sites / cut-outs**: over the whole site (disc, cut-out or halo): centre deviation, rms
  and max, share left open, radial profile, the largest ring against the surrounding surface and
  the largest step between 0.5 mm rings, and how much of the true curvature the fill follows
  (1 = as the surface, 0 = flat; curved surfaces only);
- **through slots and holes**: open at each station (no surface on the axis through the plate),
  width at mid-thickness and just under the top; louvre rows station by station; **blind grooves
  and pockets**: share of the depth kept, width; **boss pilots**: true vs meshed surfaces along
  the pilot axis, and the pilot's width halfway down the recess (from the boss end along
  `pilot_axis` by half of `pilot_depth_mm`; a recess with no walls there fails);
- **relief**: share of the height (depth) kept at each top probe against the true base round
  it, sampled from the feature's half width + 1.6 mm out to + 2.9 mm (clear of the mesher's
  skirt); a knurl as peak minus valley; fails below 80 % (grooves 70 %) and above 125 %; a top
  with no mesh is counted as missing (perforated or dropped), not as 0 %;
- **edges**: distance of the mesh from the edge line, the rounding radius that distance means
  (for the edge's angle, taken from the faces' most frequent normals; for fillets the radius
  found), and the bead beside the edge: the highest bulge from 0.3 mm on against the face's level
  over the last 0.5 mm of a walk that stops 1 mm before the face ends (so a slot wall's far edge
  is not its level), with the search kept to 0.45 x the free room in front of the face;
- **laser strays**: stray points in the passes, after Merge and after Reconstruct's preparation;
  the preparation is judged against what the merge left it (the merge's resampling thins the
  strays with everything else); share of the window with a layer 0.3–1.4 mm inside the material,
  pits;
- **pass offset**: mean signed distance on smooth surface (1.5 mm from edges) seen by pass 1
  only, by other passes only and by both; `bias_mm` is the worst group against the truth,
  `step_mm` the largest difference between groups; handled = both within 0.05 mm.

Tests (`src/tests.rs`, `src/eval_tests.rs`): the generator's own tests (catalogue consistent with
its truth, sticker heights and rings, Fused-export cuts, strays, offsets and tilt, tables under
holes, realism against the S1-S3 numbers, files read back as the app reads them); three small
eval scenes (a plate with a kept and a cut-out sticker and a 3 mm slot, a dome with a kept and a
cut-out sticker, a plate with laser strays); `known_failure_*` tests (ignored) assert the ideal
where the pipeline falls short now; `synth_catalogue` (ignored) runs the whole catalogue from
`SCANCRAFT_SYNTH_DATA` and fails when a scene fails more features or passes fewer probes than on
2026-11-28, or Make Solid is not watertight.

## Findings (2026-11-28, second run)

The ranked list, with numbers and the renders to look at, is in `docs/open-issues.md` §2.5. In
short (label isolation, meshers alone):

1. Reconstruct leaves the empty halo round S2-style sticker sites open (17 of 21 on curved
   surfaces, 29 % of the site in the median); Make Solid closes them 0.17 mm off.
2. Misregistered passes stay: Reconstruct keeps offsets (-0.12 mm) and a 0.25° tilt (+0.24);
   Make Solid removes a constant offset but not a tilt, and shifts the wrong pass on the shell.
3. Holes and slots: Reconstruct closes 93 of 99 holes of 1.5 mm; Make Solid keeps them open but
   1.00 mm wide.
4. Fine relief: Reconstruct keeps 19-41 % of emboss, engraving and bumps; Make Solid 38-74 %.
5. Make Solid on curved sites with correlated noise and at rims (every dome rim 0.26 mm off).
6. Laser strays: the preparation keeps 96 % of the strays the merge leaves.
7. Screw-boss pilots capped; 8. Fused-export cut-outs (Reconstruct leaves them open);
9. edges (0.5 mm fillet -> 0.86-1.02 mm, beads 0.13-0.30 mm); 10. the 0.6 mm fin.

Before any of that, the app's object isolation fails on plates lying on a table with clutter: it
takes the marker block or the can as the object (`perforated_grille`, `relief_fine`, the top pass
of `plate_screws_bosses_ribs`), keeps the whole table (`flat_markers`, `flat_laser_strays`,
`flat_vents`) or brings table pieces back at the contact (`box_edges`: Make Solid 5 components,
genus 11). The first run's findings (`E:\Claude\eval\synth\results_before_2026-11-28.*`) are
superseded; several of its numbers came from the metric errors fixed here (§2.5, last paragraph).

Per-scene numbers: `E:\Claude\eval\synth\compact.txt` (app isolation) and
`E:\Claude\eval\synth\truth_iso\compact.txt` (label isolation); one page:
`E:\Claude\eval\synth\overview.png`, `overview_truth_isolation.png`.


## Limits

- Each pass is one fused cloud: no per-frame tracking jitter; misregistration is one rigid error
  per pass (`offset_mm`, `pose_error`, `depth_bias_mm`), not a drift along the pass.
- Strays are placed only along the view ray behind part points; there are no strays in front of
  the surface and no ghost layers of whole frames.
- The hatching is a sine along one direction per pass; its real period was not measured.
- Stickers follow the surface they are on; the edge ramp is a smoothstep, there is no paper edge.
- Clutter is three simple shapes; there is no background wall and no turntable.
- The real scans' p99 normal errors (22-64°) come from edges and grazing areas the synthetic
  scenes have less of (synthetic p99 15°).
