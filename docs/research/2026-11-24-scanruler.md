# ScanRuler (CNC Kitchen): what it is and what ScanCraft can take from it

Quick study, 2026-11-24, read online only (GitHub API: README, manual, package.json, file headers
and comments). Nothing was cloned or run; algorithms were not traced line by line.

## What it is

- TypeScript / React / three.js browser app (scanruler.com, or self-hosted), v0.11.2, active.
  Scan metrology: primitive fits (plane, cylinder, cone, sphere, torus), dimensions, GD&T,
  sections, wall thickness, deviation maps. Plugins: Mesh Tools (plane cut with capping, Liepa
  hole fill with fairing, ball-pivot wrap, booleans, offset) and Reverse engineer.
- **Licence: AGPL-3.0-only** with a CLA; commercial exceptions are sold. Its code cannot be
  copied into ScanCraft (MIT OR Apache-2.0). Ideas and published algorithms can be
  re-implemented in our own code.
- **"AI" is not in the app.** An external coding agent (Claude Code, Codex, …) drives the open
  tab through an MCP bridge (loopback WebSocket with a pairing token), following guides the app
  serves. The agent sketches on sections, extrudes or revolves, calls a review tool (deviation
  map, volume error, worst places numbered on renders, trend, target) and repeats until about
  75–90 % of the scan is within tolerance. Their published runs reach 69–96 %; one run took
  106 min and 284 tool calls.
- Solids and STEP: OpenCascade compiled to WASM (LGPL-2.1 + exception, loaded as a separate,
  replaceable file); measured primitives are written as analytic AP214 by their own writer.
- **No multi-scan merging.** Its "alignment" is (1) a 3-2-1 datum alignment to world axes,
  (2) auto-orientation of a part to its own faces, (3) scan-to-CAD ICP with 24 start attitudes.

## Ideas worth re-implementing in ScanCraft (own code)

| Idea | Where | Effort |
| --- | --- | --- |
| Auto-orient a merged part: area-weighted histogram of normal directions, peaks = face directions, great-circle poles = turned/extruded axis, open rim votes as the base, vector area tells which side is open | after merging, before meshing/export | S–M |
| "Ambiguous" flag: a clearly different pose scores almost as well | `scancraft-align` quality, next to the slide check | S |
| Score candidate poses by a capped mean over all samples (dropping points cannot improve the score); facing-aware closest-point search | refine / quality | S |
| Datum alignment UI: plane or 3 points, axis or 2 points, origin, live preview; primitive fits grown from a click | app, Clean/Align stage | M |
| Picked point pairs solved with Horn's quaternion method, warning on collinear picks, as the manual fallback when Align All rejects an edge | `align_point_pairs` | S |
| Scale from several measured lengths (for imported photogrammetry scans) | import | S |
| Volume-error check (excess/missing material regions via winding number) | solid-model benchmark in `scancraft-mesh` | M |
| Review tool for an agent: ranked worst places on renders, trend, target; guides served by the app | AI assistant plan, `merge-and-ai.md` §6 | M |

## Scan to CAD / STEP

- Short term, no code reuse: export the merged mesh (STL/PLY, mm) and open it in ScanRuler.
  Exchanging files does not touch our licence; bundling or modifying ScanRuler would.
- ScanCraft alone: write fitted primitives (planes, cylinders, spheres) as analytic AP214 STEP
  with our own Part 21 writer or `truck-stepio` (Apache-2.0): datums for CAD without B-rep
  modelling (M).
- Real scan-to-CAD belongs in SolidCraft (same licence; `truck` kernel and a STEP writer
  already there): ScanCraft hands over the watertight mesh plus fitted primitives; SolidCraft
  adds section/sketch fitting and an agent loop on deviation reviews (L). SolidCraft's rules
  forbid reading GPL/LGPL/AGPL code, so that work must be written from this summary by someone
  who has not read ScanRuler's sources.
- Avoid OpenCascade unless the licence policy changes.
