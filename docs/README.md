# Documentation map

Which document to read for what, and how far to trust it. The notes were written over several
weeks by different people and agents while the understanding changed; older ones keep their
value for structure and evidence, but some of their conclusions were overturned. Where two
documents disagree, the one higher in this list wins.

## Current (read these)

| Document | Content |
| --- | --- |
| [HANDOFF.md](HANDOFF.md) | Where the project stands, first steps on a new machine, rules for the scanner, next tests, known gaps |
| [open-issues.md](open-issues.md) | Open bugs (review of the per-pass cleaning / meshing work), old vs new mesher measurements, user-requested UI features (project tabs, button placement, history timeline) |
| [architecture/scan-modes.md](architecture/scan-modes.md) | Every scan mode and tracking mode, which combine, measured status of each, open items |
| [protocol/device-internals.md](protocol/device-internals.md) | Ground truth about the POP 4: control interface as a shell and its encoding rules, the DSP log, DSP reload on format change, the start order, the DSP's own name for every message, firmware v5.3.99 changes |
| [architecture/scan-sessions.md](architecture/scan-sessions.md) | Projects, passes, alignment: the application's data model |
| [architecture/splatting.md](architecture/splatting.md) | Gaussian-splat output |
| [architecture/merge-and-ai.md](architecture/merge-and-ai.md) | Merging scans and AI object segmentation: decisions, Revo's approach, alignment test on real scans, plan |
| [protocol/revo-marker-framework.md](protocol/revo-marker-framework.md) | Revo Scan's per-project marker list `marker_framework.mkf`: byte layout with evidence, how it is used (import, alignment test) |
| [architecture/ui-wording.md](architecture/ui-wording.md) | UI wording and button placement: the glossary, capitals and disabled-reason rules every text and button in the app follows |

## Per-mode notebooks (current at the end, history above)

These are lab notebooks: dated sections in the order things were found, including hypotheses
that were later rejected. Each starts with a status line; the **last dated sections are the
current state**.

| Document | Mode | Ends with |
| --- | --- | --- |
| [protocol/vcsel.md](protocol/vcsel.md) | VCSEL Rapid | §6: the 640x1024 slot pair (solved) |
| [protocol/hybrid.md](protocol/hybrid.md) | Hybrid HD | the format-order fix (solved) |
| [protocol/laser.md](protocol/laser.md) | Cross Lines, Single Line | 2026-11-10 sections: Revo's sequence, marker tracking, Dark preset, lines + dots stream |
| [protocol/markers.md](protocol/markers.md) | Marker / Global Marker tracking | written for Full-field; laser frames added in laser.md |
| [protocol/color.md](protocol/color.md) | Colour camera and Color tracking | 2026-11-01; frame pairing fixed later (scan-modes.md) |

## Reference (valid, with caveats)

| Document | Content | Caveat |
| --- | --- | --- |
| [protocol/modes.md](protocol/modes.md) | Revo Scan's full catalogue of modes, presets and parameters, with sources | §8 "implementation assessment" is from 2026-11-01 and superseded by scan-modes.md |
| [protocol/prior-art.md](protocol/prior-art.md) | What other projects published about Revopoint scanners | its inferences about exposure and Hybrid were made before the register fix |
| [protocol/captures/](protocol/captures/) | Notes from traffic captures and stream sweeps | 2026-11-01 firmware v5.3.94; the "no effect" results for exposure are explained in device-internals.md |
| [protocol/reconstruction.md](protocol/reconstruction.md) | Coordinate systems and calibration maths | written early; the driver's `calibration.rs` is authoritative |
| [protocol/next-live-tests.md](protocol/next-live-tests.md) | A test plan from 2026-11-02 | superseded; kept for history |

## Historical (do not act on these without checking)

Written during the first protocol analysis, partly from static analysis alone. They describe
transports and packet framing that the working driver does not use in that form, and contain
statements that later measurements contradicted (for example that `camera_id=21` always carries
depth, or the meaning of `0xb04`).

| Document | What it was |
| --- | --- |
| [protocol/overview.md](protocol/overview.md), [framing.md](protocol/framing.md), [commands.md](protocol/commands.md) | first protocol write-up |
| [protocol/developer_handoff.md](protocol/developer_handoff.md) | first implementation blueprint (carries its own correction note) |
| [protocol/lasers_and_markers.md](protocol/lasers_and_markers.md) | emitter control from static analysis; register meanings are now in device-internals.md §4 |
| [app_architecture_and_revo_analysis.md](app_architecture_and_revo_analysis.md) | early application design study |
| `../SCANCRAFT_PLAN.md`, `../reverse-engineering-plan.md` | the original plans |
