# Revo Scan's `marker_framework.mkf`

Written 2026-11-26. Revo Scan 6 writes one `marker_framework.mkf` into every project folder, next
to `raw_preview.ply`. It lists the marker stickers Revo tracked in that scan. ScanCraft reads it
with `scancraft_io::read_marker_framework` (`crates/scancraft-io/src/revo.rs`): Import Scan takes
it as the new pass's marker map, and the real-data alignment test
(`crates/scancraft-align/tests/real_data.rs`) checks poses against it. Labels: **[C]** confirmed
on the data, **[I]** inference.

## 1. Layout

All little endian. [C] for every field marked "used"; the meaning of the unused ones is not
known.

| Offset | Type | Field | Evidence |
| --- | --- | --- | --- |
| 0 | u32 | version, 3 | 3 in all three files |
| 4 | u32 | marker count | 69, 67, 72; file sizes 7184, 6976, 7496 = 8 + count x 104 exactly |
| 8 | 104-byte records | one per marker | |

Record:

| Offset | Type | Field | Evidence |
| --- | --- | --- | --- |
| 0 | f64 x3 | centre, mm, frame of `raw_preview.ply` (used) | see section 2 |
| 24 | f64 x3 | normal, unit length, out of the surface (used) | length 1 within 1e-15 in every record; section 2 |
| 48 | i32 | -1 in every record (not used) | [I] a code or group id, -1 = uncoded sticker |
| 52 | f32 | 12.5 .. 2442, no unit known (not used) | not correlated with radius or the field at 96 (r < 0.35) |
| 56 | 36 bytes | always zero | |
| 92 | f32 | sticker radius, mm (used) | median 3.05 / 3.05 / 3.00 (S2 / S3 / S1); the stickers are about 6 mm across. S1 holds rounded values (18 distinct, 1.5 .. 3.0) |
| 96 | f32 | 0.0003 .. 1.4 (not used) | [I] a fit residual |
| 100 | 4 bytes | padding | random-looking, includes NaN patterns: uninitialised struct padding (the record is 8-byte aligned) |

The reader rejects a file whose length is not exactly 8 + count x 104, an unknown version, a
non-finite centre, a normal that is not of unit length (±1e-3), or a negative radius, with an
error (never a panic; tested on random bytes of every length up to three records).

## 2. Check against the scans

Three Revo Scan 6 projects of one thin-walled shell part: S1 outside (preview without the table),
S2 and S3 inside on a table with marker blocks (preview with the table). For each marker, the
nearest point of the same folder's `raw_preview.ply`, and, for the markers on the preview, the
marker normal against the normal of a plane fitted to the preview points within 6 mm (sign from
the preview's own normals):

| Scan | Markers | Within 1 mm of the preview | Distance (median / max of those) | Normal vs plane (median / p90 / max) | Opposite to the preview's normals |
| --- | --- | --- | --- | --- | --- |
| S1 | 69 | 14 | 0.28 / 0.71 mm | 1.4° / 3.9° / 14° | 0 |
| S2 | 67 | 60 | 0.27 / 0.89 mm | 2.1° / 4.3° / 55° | 0 |
| S3 | 72 | 50 | 0.24 / 0.92 mm | 1.4° / 3.5° / 13° | 2 |

The large maxima are markers at block edges, where the 6 mm plane spans two faces.

- The markers that are not on the preview are 21–174 mm (S1) and 1–84 mm (S2, S3) away and do
  not lie on one plane: they are on marker blocks and table parts Revo cut from the preview
  (S1's preview has no table at all, and Revo keeps a `cache/preview.mask` for it). [I]
- The centre sits on top of the sticker: median 0.08–0.23 mm above the plane of the surface in
  a 4–6 mm ring around it, about a sticker's thickness. [I]
- The fused clouds in `pointclouds/` are in the same frame as the preview (fused to preview
  median 0.20 mm for S1, 0.23 mm for S2), so the markers fit them too; Revo cuts a hole of about
  5.5 mm radius into the fused cloud at each marker.
- Matching markers between scans under the alignment confirms the frame (section 3).

## 3. Alignment check

`real_data.rs` matches the markers of two placed scans under the computed pose (mutual nearest
within 3 mm, normals within 30°), fits the matched markers with Horn's method, matches again
under that fit and fits once more, then compares the computed pose with the fit: rotation
difference and RMS displacement of the moving scan's points. Pass limit: 0.5° and 1.0 mm.

Results on 2026-11-26 (`align_all` poses as the app computes them, release build, the three
projects of section 2; every test reference S1, S2 or S3 gives the same poses):

| Pair | Matched markers | RMS under the pose | RMS under the fit | Pose vs fit |
| --- | --- | --- | --- | --- |
| S1 → S3 | 6 | 0.56 mm | 0.11 mm | 0.27° / 0.63 mm |
| S2 → S3 | 7 | 0.39 mm | 0.06 mm | 0.19° / 0.36 mm |
| S1 fused → S3 | 6 | 0.77 mm | 0.11 mm | 0.22° / 0.81 mm |
| S2 fused → S3 | 7 | 0.15 mm | 0.06 mm | 0.07° / 0.15 mm |

S1 and S2 share no markers (outside and inside). The older "verified" transforms are
1.07° / 2.56 mm (S2 → S1) off the same poses. Limits: 0.5° and 1.2 mm, about 1.5 to 2 times the
worst pair, still well below the older transforms' error. Whole-scene alignment places nothing,
so it has no pair to check.
