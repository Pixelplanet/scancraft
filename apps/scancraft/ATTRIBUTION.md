# Attribution

ScanCraft's desktop interface is built in the look of **SolidCraft**
(<https://github.com/bherbruck/solidcraft>, © SolidCraft contributors, MIT OR Apache-2.0 — the
same licence as ScanCraft). Code and assets were ported from the checkout in
`scratch/solidcraft`; ported files say so in their header comments.

## Ported code

| ScanCraft | SolidCraft source |
| --- | --- |
| `crates/scancraft-ui/src/theme.rs` | `assets/styles/ui/palettes.rs` (`Tokens::light` dimensions, `Tokens::dark` colours); `crates/ui-egui/src/theme.rs` (`visuals`, `apply`, `shell_shadow`) |
| `crates/scancraft-ui/src/fonts.rs` | `crates/ui-egui/src/fonts.rs` (`base_font_definitions`, weight helpers) |
| `crates/scancraft-ui/src/icons.rs` | `crates/ui-egui/src/icon_svg.rs` (palette-role resolution), `assets/icons/ui/design/palette.toml` (dark role values) |
| `crates/scancraft-ui/src/kit/chrome.rs` | `ui_kit/container.rs` (`Container::frame`), `ui_kit/panel.rs` (`panel_header_hover`, `header_actions`), `ui_kit/disclosure.rs`, `ui_kit/actions.rs`, `browser.rs` (`browser_header`), `status_bar.rs`, `shell.rs` (`SHELL_GAP`) |
| `crates/scancraft-ui/src/kit/tree.rs` | `ui_kit/tree.rs` (`tree_row`), `ui_kit/history.rs` (`TREE_ROW_HEIGHT`) |
| `crates/scancraft-ui/src/kit/form.rs` | `ui_kit/layout.rs` (`scope`, `sized_row_body`, `section_heading`, `section_with_badge`), `ui_kit/choice.rs` (`segmented`), `ui_kit/checks.rs`, `ui_kit/fields.rs` (`slider`, `dropdown`), `ui_kit/actions.rs` (`button_widget`) |
| `crates/scancraft-ui/src/kit/ribbon.rs` | `toolbar.rs` (ribbon constants, `ribbon_tabs`, `ribbon_tab_style`, button and section painting, `ribbon_button_width`, `ribbon_label_lines`), `shell.rs` (`Cards::new`) |
| `crates/scancraft-ui/src/kit/nav.rs` | `ui_kit/navigation.rs` (`NAV_*` metrics, `navigation_button`, `navigation_fill`, `navigation_ink`, `navigation_separator`, `navigation_surface`), `navigation_bar_view.rs` (`measure_row`) |

## Libraries used for Gaussian splatting (`crates/scancraft-splat`)

| Dependency | Licence | Use |
| --- | --- | --- |
| **Brush** — `brush-train`, `brush-render`, `brush-dataset`, `brush-serde`, `brush-vfs` (tests) and their internal crates (<https://github.com/ArthurBrussee/brush>, © Arthur Brussee and contributors) | Apache-2.0 | Gaussian-splat training, rendering and 3DGS PLY export, used as a library at a pinned git revision. No Brush code is copied into this repository. |
| **Burn** (<https://github.com/tracel-ai/burn>) and **CubeCL** / **CubeK** (<https://github.com/tracel-ai/cubecl>), pulled in by Brush | MIT OR Apache-2.0 | Tensor/autodiff framework and wgpu compute backend Brush runs on. |
| `kiddo` | MIT OR Apache-2.0 | k-d tree for the initial splat scales (already used by `scancraft-slam`). |

Brush's licence requires keeping its copyright/licence notice with redistributed binaries:
ship `LICENSE` from the Brush repository (Apache-2.0) alongside ScanCraft builds that include
`scancraft-splat`. Licences of everything compiled into `scancraft-splat` at the pinned
revision (`cargo tree -p scancraft-splat -e normal,build --format "{l}|{p}"`): MIT and/or
Apache-2.0 for almost all crates, plus Unicode-3.0, Zlib, BSD-2/3-Clause, ISC, BSL-1.0,
Unlicense, Apache-2.0 WITH LLVM-exception — and two crates under **MPL-2.0** (`buildid`,
`colored`, used by Burn/CubeCL). MPL-2.0 is file-level weak copyleft: it only requires that the
source of those (unmodified, crates.io) files stays available, which it does; it places no
obligation on ScanCraft's own code. No GPL/LGPL/AGPL dependency. Brush itself enforces a
permissive allow-list with `cargo deny`.

## Assets

- `crates/scancraft-ui/assets/icons/` — SolidCraft UI icon masters from
  `assets/icons/ui/design/svg/{model,flat,workspace}` (MIT OR Apache-2.0). `model/pause.svg` and
  `model/stop_capture.svg` were drawn for ScanCraft after SolidCraft's `play.svg`, using the same
  palette roles.
- `crates/scancraft-ui/assets/fonts/` — IBM Plex Sans (variable) and IBM Plex Mono, SIL Open Font
  Licence 1.1 (`OFL-IBMPlexSans.txt`, `OFL-IBMPlexMono.txt`), as bundled by SolidCraft.
