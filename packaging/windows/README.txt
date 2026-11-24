ScanCraft @VERSION@ (build of @DATE@) - Windows x64
https://github.com/Pixelplanet/scancraft

INSTALLING
  ScanCraft-Setup-@VERSION@.exe   Installs for the current user (no administrator rights) to
                                  %LOCALAPPDATA%\Programs\ScanCraft, with Start menu entries
                                  and an uninstaller under Settings > Apps.
  Portable zip                    Unpack anywhere and start scancraft.exe; nothing is installed.
  Windows may warn that the publisher is unknown (the program is not code-signed):
  "More info" > "Run anyway".

STARTING
  scancraft.exe                     Start, then Connect > Find Scanners.
                                    Scanner and PC must be on the same network.
  scancraft.exe --connect <ip>      Connect to a scanner at a known address.
  scancraft.exe --mock              Built-in simulated scanner (no hardware needed).
  "ScanCraft (record).bat"          As scancraft.exe, and the raw stream of every scan
                                    is saved to the "recordings" folder next to it, for
                                    later analysis. Several MB per second: delete
                                    recordings you no longer need.

BEFORE SCANNING WITH A REVOPOINT POP 4
  - Close Revo Scan. Only one program can use the scanner at a time.
  - Always end a scan with Stop in the app. Do not close the window or kill the
    program in the middle of a scan: the scanner can stop sending images until it
    is restarted.
  - Laser modes (Cross Lines, Single Line): the first time on a new PC, do one short
    scan with the "Normal" object preset on a matte, light scene. The laser geometry
    is measured once (a few seconds, no points appear meanwhile) and then kept.

MERGING SCANS (new in 0.2)
  - Scan the object several times from different sides (for example flipped) as
    passes of one project, then Align > Align All. The table is removed
    automatically ("Use whole scene" turns that off). The report shows which pairs
    were measured, which were connected through another pass, and the wall
    thickness when an inside and an outside scan meet.
  - Import Scan (PLY, OBJ, STL) adds someone else's scan as a pass; Add Project
    appends the passes of another ScanCraft project. Type or drop the file path.
  - Refine shows a preview first; Merge builds one cloud from all aligned passes.

STATUS
  Every scan mode of the POP 4 works on a scene that does not move (firmware
  v5.3.99). Scanning with a moving scanner is only partly tested. Details:
  docs/architecture/scan-modes.md, docs/architecture/merge-and-ai.md and
  docs/HANDOFF.md in the repository.

TOOLS (command line, folder "tools")
  mode_check.exe <ip> <light> <tracking> <object> [frames]
      Runs one mode through the same code as the app and reports frames, geometry and
      tracking. light: fullfield | vcsel | hybrid | cross | single
      tracking: feature | marker | global | color     object: general | dark | ...
      Example: mode_check.exe 192.168.1.50 vcsel feature general 60
      Set MODE_CHECK_VERBOSE=1 for one line per frame.
  live_scan.exe <ip> [frames]
      Minimal Full-field capture with a flatness check.
  stream_probe.exe
      Opens any stream format and reports what arrives (for protocol work; run it
      without arguments for usage).
  merge_ply.exe <out_dir> <scan.ply> <scan.ply> ... [--whole-scene] [--reference i]
      Aligns and merges scan files without the app and writes the merged cloud,
      per-scan colours and an alignment report.
  Let the scanner tools finish by themselves; do not press Ctrl-C while they stream.

Licence: MIT or Apache-2.0 (LICENSE-MIT, LICENSE-APACHE). Third-party notices: ATTRIBUTION.md.
