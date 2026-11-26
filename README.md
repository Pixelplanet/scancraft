# ScanCraft

Open-source 3D scanning software in Rust, built to be vendor-neutral: scanner support lives in
driver crates, the application (capture, tracking, alignment, cleanup, meshing, export) does not
know which scanner it talks to. The first driver is for the **Revopoint POP 4** over Wi-Fi,
written from our own protocol analysis of hardware we own, for interoperability.

Status: work in progress. Every scan mode of the POP 4 starts, streams and tracks a **static**
scene; scanning with a moving scanner is only partly tested. See
[docs/architecture/scan-modes.md](docs/architecture/scan-modes.md).

## Run it

**Prebuilt (Windows x64):** from the
[Releases page](https://github.com/Pixelplanet/scancraft/releases), either

- `ScanCraft-Setup-<version>.exe` — installs for the current user (no administrator rights) to
  `%LOCALAPPDATA%\Programs\ScanCraft`, with Start menu entries and an uninstaller under
  *Settings → Apps*. The program is not code-signed, so Windows may ask for confirmation
  (*More info → Run anyway*); or
- the portable zip — unpack it anywhere and start `scancraft.exe`.

- `scancraft.exe` — start, then *Connect → Find Scanners* (the scanner must be on the same
  network as the PC).
- Double-click a project file (`<name>.scancraft`, in the project folder) or a bundle
  (`<name>.scancraftz`, the whole project in one file) — the installer registers both. While
  ScanCraft runs, it opens as a new tab there; a second app is never started (only one program may
  use the scanner). `scancraft.exe <project file, bundle or folder>` does the same;
  `--new-instance` starts a second app anyway.
- `scancraft.exe --connect <scanner ip>` — connect to a known address at start.
- `scancraft.exe --mock` — a built-in simulated scanner, to try the app without hardware.
- `ScanCraft (record).bat` — as the first, and every scan's raw stream is written to a
  `recordings` folder next to the program for later analysis (large: up to several MB per second).

The zip also contains the command-line check tools (`mode_check.exe`, `stream_probe.exe`,
`live_scan.exe`); see [docs/HANDOFF.md](docs/HANDOFF.md).

**From source:** install [Rust](https://rustup.rs) (stable, MSVC toolchain with the Visual Studio
C++ build tools), then

```
cargo run --release -p scancraft
```

The first build takes a while (the Gaussian-splat trainer pulls in a GPU compute stack).
`cargo test --workspace` runs the tests; none of them needs a scanner. To build the installer and
the portable zip yourself (Windows, uses the built-in IExpress):

```
powershell -ExecutionPolicy Bypass -File packaging\windows\build_release.ps1 -Out <folder>
```

## What works on the POP 4 (firmware v5.3.99)

| Light source | Tracking modes | Depth computed |
| --- | --- | --- |
| Full-field HD | Feature, Marker, Global Marker, Color | on the scanner, ~13 fps |
| VCSEL Rapid | Feature, Color | on the PC (stereo), ~10 fps |
| Hybrid HD | Feature, Color | on the PC (dots + stripes), ~8 fps |
| Cross Lines | Marker, Global Marker, Color | on the PC (laser lines) |
| Single Line | Marker, Global Marker | on the PC (laser line) |

Measured on a static scene. Open items and numbers:
[docs/architecture/scan-modes.md](docs/architecture/scan-modes.md).

## Repository layout

| Path | What |
| --- | --- |
| `apps/scancraft` | the desktop application (egui / wgpu) |
| `crates/scancraft-driver` | scanner driver interface, mode model, mock scanner |
| `crates/scancraft-driver-revopoint` | POP 4 driver, and `examples/` with the device test tools |
| `crates/scancraft-slam`, `-pipeline`, `-markers`, `-color` | tracking and fusion |
| `crates/scancraft-stereo`, `-hybrid`, `-laser` | depth computed on the PC |
| `crates/scancraft-align`, `-mesh`, `-io`, `-splat`, … | alignment, meshing, export, splats |
| `docs/` | protocol notes and architecture; start at [docs/README.md](docs/README.md) |
| `tools/scanner` | shell helpers for the scanner (remote shell, DSP log, restart) |

Contributors and agents: read [AGENTS.md](AGENTS.md) and [docs/HANDOFF.md](docs/HANDOFF.md) first.

## Licence

MIT or Apache-2.0, at your option ([LICENSE-MIT](LICENSE-MIT), [LICENSE-APACHE](LICENSE-APACHE)).
Third-party notices: `apps/scancraft/ATTRIBUTION.md`.
