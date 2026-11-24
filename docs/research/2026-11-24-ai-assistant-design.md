# ScanCraft AI assistant: design study

Status 2026-11-24. Design only, nothing built. Written for `docs/architecture/merge-and-ai.md`
section 6 (decision 9: "an AI API interface like SolidCraft's agent chat, later"). Source studied:
SolidCraft 0.27.1 (`scratch/solidcraft`, git-ignored copy, MIT OR
Apache-2.0) and ScanCraft on `feature/align-merge` (commit 8167673).

Evidence is cited as `path:line` (SolidCraft paths are relative to the SolidCraft copy, ScanCraft
paths to the repo root). [H] marks a hypothesis or a proposal that was not checked.

---

## 1. How SolidCraft's interface works

### 1.1 The big picture

SolidCraft has two separate AI entry points that share one tool server:

```
                       (A) external agents: Claude Code, Codex, Cursor, Claude Desktop ...
                              │  MCP over stdio
                              ▼
 SolidCraft window ◄──loopback control channel (token)──  solidcraft-cli mcp [--connect | --in file]
      │   ▲                                                    ▲
      │   │ chat's own endpoint (fresh port + token, gate)     │ starts it per session/new
      │   └────────────────────────────────────────────────────┘
      │ child process, ACP over stdio                    (B) in-app Assistant panel
      └──────────────────────────────────────────────►  agent (claude-agent-acp, codex-acp, opencode acp, grok …)
                                                         or ApiBackend (SolidCraft runs the tool loop itself
                                                         against an OpenAI- or Anthropic-compatible URL)
```

- **The MCP server** (`crates/mcp`, binary `solidcraft-cli mcp`) is the only way any model touches a
  design. It runs either *headless* (an in-process engine session, `--in part.sc`) or *bridged* to
  the running app over its loopback control channel (`docs/mcp.md:18-64`).
- **The in-app chat** (`crates/ui-egui/src/agent_chat/`) is a client of that same server: it starts
  an agent, hands it the MCP server, shows the conversation, and gates every tool request the
  server sends back to the app (`docs/design/in-app-agent-chat.md:80-103`).
- **Everything is an engine command.** Tools are thin: `execute`, `batch`, `list_commands`,
  `describe_command`, plus a few session tools (open, save, screenshot, measure). The contributor
  rule is "add a command, not a new tool" (`skills/solidcraft-agent-tools/SKILL.md:20`). The same
  commands drive the toolbar, scripts and the control channel, so an agent models exactly like a
  person (`docs/mcp.md:5-6`).

### 1.2 Providers ("backends")

The panel talks only to a `ChatBackend` trait (`crates/ui-egui/src/agent_chat/backend.rs:94-121`:
`poll`, `prompt`, `cancel`, `new_session`, `answer_permission`, `authenticate`, `set_model`,
`set_mode`, `shutdown`) and receives `Event`s (text, tool, plan, permission, auth, modes, models,
turn ended, exited; `backend.rs:49-79`). Two implementations:

| Backend | What it is | Who runs the tool loop |
| --- | --- | --- |
| `AcpBackend` (`acp.rs`, `acp_client.rs`) | A coding agent the user already has, started as a child process, speaking the Agent Client Protocol (JSON-RPC 2.0 over stdio). Claude Code (via `@agentclientprotocol/claude-agent-acp`), Codex, Antigravity, OpenCode, Grok Build, Copilot CLI, Qwen, goose, or any custom command. | The agent. SolidCraft passes the MCP server in `session/new {mcpServers}` (`acp.rs:120`). |
| `ApiBackend` (`api_client.rs`, 131 KB) | An OpenAI-compatible (`/chat/completions`, Ollama, LM Studio, OpenAI, Mammouth, OpenCode Go) or Anthropic-compatible (`/v1/messages`) endpoint. | SolidCraft. It starts the same `solidcraft-cli mcp --connect` and is its MCP client; `tools/list` becomes the endpoint's tools (`crash_reports` hidden, `api_client.rs:50`). Text per tool result capped at 20 000 chars, history at 240 messages, at most 2 view pictures kept (`api_client.rs:24-32`). |

Discovery (`discover.rs`) looks for ACP agents on `PATH` and the usual install folders
(`%APPDATA%\npm`, nvm, Volta, `~/.local/bin` …), runs only `--version` (3 s), and reports
"login found" by checking that the agent's own login file or key variable exists, never reading it
(`docs/agent-chat.md:153-169`).

### 1.3 Auth and key storage

- **Coding agents:** SolidCraft never sees a credential. The agent keeps its own login
  (`~/.claude`, `~/.codex`, …). If it answers `-32000 Authentication required`, the panel lists
  the agent's own `authMethods`: a terminal command to copy, a button that calls `authenticate`
  (the agent opens the browser), or an environment variable to set. There is deliberately no key
  field for coding agents (`docs/design/in-app-agent-chat.md:61-78`).
- **Endpoints:** the key goes into the OS keychain through a `Secrets` trait
  (`backend.rs:137-142`), implemented by the host with the `keyring` crate, service name
  "SolidCraft agent chat" (`apps/solidcraft/src/main.rs:551-570`; Windows Credential Manager).
  It is never in preferences, is read for the connection only, and is refused over plain HTTP to
  another computer (`docs/design/in-app-agent-chat.md:114-126`). The HTTP client is the host's
  (`Services::agent_http`, ureq, no redirects, no cookies; `apps/solidcraft/src/main.rs:495`).
- **Tool channel:** the chat's endpoint is a fresh loopback port with a random per-launch token in
  a user-only file; the token never passes through the agent and `SOLIDCRAFT_CONTROL_TOKEN` is
  removed from the agent's environment (`docs/design/in-app-agent-chat.md:131-136`,
  `apps/solidcraft/src/control_server.rs:92`).

### 1.4 The MCP server

- JSON-RPC 2.0, one message per line, MCP revisions 2025-06-18 / 2025-03-26 / 2024-11-05
  (`crates/mcp/src/server.rs:15-16`). `Server { backend: Box<dyn Backend>, scope }` with backends
  `Headless`, `Remote`, `Auto` (`crates/mcp/src/lib.rs:17`).
- **Server instructions** tell the model what the app is and how to work
  (`server.rs:28-38`); the bundled skills are readable through a `skill_reference` tool, so an
  agent without file access can follow them (`docs/mcp.md:281`).
- **Tools** (`docs/mcp.md:283-304`): `list_commands`, `describe_command` (JSON Schemas of
  parameters and result), `execute`, `batch` (stops at the first error and rolls back to an engine
  checkpoint), `inspect_design`, `measure`, `body_topology`, `set_parameter`, `screenshot`
  (headless GPU/CPU render or the app window, with named view or orbit camera), `export`,
  `undo`/`redo`, `pending`, `running`, `crash_reports`, `skill_reference`, `new_design`, `open`,
  `save`. Resources `solidcraft://design`, `solidcraft://commands`.
- **Tool annotations** are honest: read-only tools say so; anything that may write files is
  `destructiveHint`/`openWorldHint` (`docs/mcp.md:151-158`).
- **File scope:** tools reach only the design's folder (if inside home and not hidden), folders the
  agent opened or saved a design in, the recovery folder, a private scratch folder, and
  `--allow-path` folders. Paths are resolved before comparing; on Windows UNC, device, 8.3,
  drive-relative and ADS paths are refused (`docs/mcp.md:86-149`, `crates/mcp/src/lib.rs:25-70`).
- **Long requests:** the app answers "still running" after 60 s; the server polls for 240 s more;
  the `pending` tool collects the result later; nothing is re-sent blindly
  (`docs/mcp.md:342-373`).
- **Compact replies** (`verbose: false`) keep large designs from flooding the context
  (`docs/mcp.md:389-404`).
- **Client setup:** `solidcraft-cli mcp setup` edits Claude Desktop, Claude Code, Cursor, Codex,
  Gemini, Copilot, Windsurf, Zed, VS Code configs in place (backup first) and the app does it by
  itself at start unless turned off (`docs/mcp.md:160-200`).

### 1.5 The UI

Side panel (dockable left/right, own window, minimised strip with state and a count of open
questions), Ctrl+Shift+A, an Assistant button in the app bar, Preferences → AI as the single page
with a master switch "Show AI features" (`docs/agent-chat.md:13-82`). Per design tab: its own
conversation, draft, attachments and queue. Messages can carry pictures, text files and CAD files
(imported at once); "Attach view" sends a picture of the viewport; "Attach selection" and "Pick
context" send what is selected as chips (`docs/agent-chat.md:84-335`). Agent text is rendered as
bounded Markdown (`markdown.rs`, `markdown_view.rs`). Consecutive tool calls fold into an activity
group; each tool call is a card with its output.

### 1.6 Safety and approval

- **Gate** (`crates/ui-egui/src/agent_chat/gate.rs`): every request from the chat's endpoint gets a
  `Verdict::{Run{change}, Ask(text), Refuse(reason)}` (`gate.rs:20-27`).
  - Only seven control methods are reachable (`gate.rs:31`); `app.*`, `prefs.*`, `diagnostics.*`
    and crash reports are refused (`gate.rs:118-123`).
  - Ask: `file.*`, `doc.*`, render export, any parameter that names a file (heuristics on key
    names and values, `gate.rs:33-112`), anything that uploads the model off the machine
    (`gate.rs:133-136`), and `edit.undo` past the turn's first change (`gate.rs:143-146`).
  - Batches are judged step by step; any Refuse refuses the batch, Asks are merged into one
    question (`gate.rs:160-186`).
- **Permission modes** (global with per-agent override): *Ask every time*, *Auto-allow model edits*
  (default), *Allow everything* (warned; still never runs what is refused). "Always allow" remembers
  a command per agent with Reset (`docs/agent-chat.md:337-363`).
- **Limits:** optional per-turn change limit (24 when turned on), 400 requests of any kind;
  unanswered questions declined after 4 minutes; requests outside a turn refused
  (`docs/design/in-app-agent-chat.md:156-159`, `docs/agent-chat.md:365-381`).
- **One undo per message:** all snapshots of a turn are merged into one undo step named after the
  message, with an "Undo these changes" button (`docs/design/in-app-agent-chat.md:164-170`).
- **The agent's own tools off:** no fs/terminal capabilities declared, an empty working folder of
  SolidCraft's own, and Claude Code's shell/file/web tools disabled via
  `_meta.claudeCode.options.disallowedTools` (`acp.rs:112-126`).
- **Processes:** the agent runs in its own process group (Windows: `taskkill /T`); stop = close
  stdin, then TERM, then KILL; quitting waits for it (`process.rs`,
  `docs/design/in-app-agent-chat.md:210-218`).
- **Turn validity flag:** a request already past the gate cannot apply to another tab after the
  turn ends (`docs/design/in-app-agent-chat.md:192-199`).

### 1.7 Benchmark

`solidcraft-cli bench-assistant` (`apps/solidcraft-cli/src/bench_assistant.rs`) runs a fixed set of
11 tasks against a running app over the control channel: per task it resets the design, runs the
prescribed setup, adds the endpoint as an API agent with All permissions, sends one prompt, waits,
and **checks the result on the design, never on what the model says** (`bench_assistant.rs:1-17`,
task table `:26-38`). Failures are classified ("wrote the call as text", "wrong arguments", "only
talked", …). This is the pattern for measuring which local models are good enough.

---

## 2. Reuse: as-is vs adapted

### 2.1 Licences

Both projects are `MIT OR Apache-2.0` (SolidCraft `Cargo.toml:10`, ScanCraft `Cargo.toml`
workspace.package). Copying code is allowed; we must keep the copyright notice and licence. Do it
the way SolidCraft itself does for CADCraft/GridCraft (`NOTICE`): an `ATTRIBUTION.md` in ScanCraft
listing every adapted file with "adapted from SolidCraft <version>, Copyright (c) 2026 SolidCraft
contributors, MIT OR Apache-2.0", plus a one-line header comment in each file. Record the
upstream version (0.27.1) so later fixes can be pulled.

Third-party pieces: `keyring` 3.x and `ureq` are MIT/Apache. The agents are installed by the user
and only started as programs, never bundled or linked (several are proprietary;
`claude-agent-acp` depends on Anthropic's SDK under Anthropic's own terms,
`docs/design/in-app-agent-chat.md:49`). Do not commit `scratch/solidcraft` (already in
`.gitignore`).

### 2.2 File by file

| SolidCraft source | Coupling found | ScanCraft use |
| --- | --- | --- |
| `agent_chat/backend.rs` (trait, events, `Secrets`, `HttpFn`, base64, content blocks) | `crate::i18n::tr`, `crate::ControlRequest` | **As-is** after replacing `tr` with plain strings and `ControlRequest` with ours. |
| `agent_chat/acp.rs` (ACP messages, `session/new` params, Claude tool switch-off, WSL path mapping) | `BRIDGE_NAME = "solidcraft-app"`, instructions text | **As-is** with names changed (`scancraft-app`) and ScanCraft's instructions. |
| `agent_chat/acp_client.rs` (`AcpBackend`) | 3× `tr` | **As-is** with strings. |
| `agent_chat/process.rs` (process group, polite stop, tests with fake agents) | none found | **As-is.** Important on Windows: never leaves helpers behind. |
| `agent_chat/discover.rs` (agent table, PATH search, login-file checks) | none found | **As-is**, agent table trimmed to what we document. |
| `agent_chat/api_client.rs` (OpenAI/Anthropic tool loop, OpenCode Go routing) | `tr`, `Protocol` from `agents.rs` | **Adapt.** Keep the OpenAI and Anthropic paths; the OpenCode Go multi-protocol routing (~600 lines) can be dropped at first. |
| `agent_chat/markdown.rs`, `markdown_view.rs` | `crate::` widgets in the view (5) | Parser **as-is**; view adapted to `scancraft-ui` theme tokens. |
| `agent_chat/gate.rs` | `solidcraft_engine::command_specs()` in a test | **Rewrite the rules, keep the shape** (`Verdict`, method allow-list, batch folding, file heuristics). Our commands carry explicit flags, so the heuristics shrink. |
| `agent_chat/agents.rs`, `settings.rs` | `SolveApp`, `Prefs` (38 refs) | **Adapt** heavily: agent list, permission modes, "always allow", Add agent dialog, keychain. |
| `agent_chat/mod.rs` (95 KB), `panel.rs` (110 KB), `attach.rs`, `context.rs`, `paste.rs`, `summary.rs` | `SolveApp`, SolidCraft document/undo model (100+ refs in `panel.rs`) | **Re-implement** for ScanCraft's app and theme, using these as the reference. Conversation, queue, approvals and tool cards carry over in behaviour, not in code. |
| `agent_chat/tests.rs` (147 KB) | `SolveApp` | Port the backend, gate, process and API tests; drop the UI ones. |
| `crates/mcp/src/server.rs` (framing, lifecycle, errors, batches) | `solidcraft_engine::MAX_LINE`, `vfs::PathScope` | **Adapt**: framing and lifecycle as-is; tool dispatch ours. |
| `crates/mcp/src/tools.rs`, `backend.rs` | the CAD engine | **Rewrite** for scans (section 3). Keep conventions: schemas, annotations, `isError` results, `still running` / `pending`. |
| `solidcraft_engine::io::vfs` (`PathScope`, Windows path checks) | engine crate | **Copy** the module; this security code is worth not rewriting. |
| `crates/mcp/src/skill_reference.rs` | skill list | **As-is** pattern with our skills. |
| `apps/solidcraft/src/control_server.rs` (`listen_agent`, token file) | app | **Adapt.** |
| `apps/solidcraft/src/main.rs:495-570` (`agent_http`, keyring `Secrets`) | none | **As-is.** |
| `apps/solidcraft-cli/src/bench_assistant.rs` | control methods | **Pattern**: same structure, our tasks (section 4, WP10). |
| `skills/` (Agent Skills format, `skill_reference`, tests that skills name only real tools) | — | **Pattern** for `scancraft-aligning`, `-cleaning`, `-merging` skills. |

### 2.3 Proposed crate boundaries in ScanCraft

| Crate / module | Content | Depends on |
| --- | --- | --- |
| `crates/scancraft-agent` (new, no egui) | ACP and API backends, process, discover, `Secrets`/`HttpFn` traits, gate rules, Markdown parser. Pure and testable. | serde_json |
| `crates/scancraft-project` (new, moved from `apps/scancraft/src/project.rs` and the data half of `model.rs`) | Project load/save, `PassMeta`, `AlignState`, pass rebuild without a window. Needed for a headless MCP server. | core, slam, align, io |
| `crates/scancraft-inspect` (new) or `scancraft-align::inspect` | Signed-offset comparison, shell/wall metrics, sections, CPU orthographic renderer (lifted from `align_ply.rs:544` `render`), offset maps. | align, image |
| `crates/scancraft-mcp` (new) | MCP server, tool definitions, `PathScope`, `Headless` (project folder) and `Remote` (app control port) backends. | project, inspect, align, mesh |
| `apps/scancraft/src/commands.rs` (new) | Command registry and dispatcher over the existing `ScanCraftApp` methods. | app |
| `apps/scancraft/src/control.rs` (new) | Loopback control endpoint, token file, request queue drained each frame. | commands |
| `apps/scancraft/src/assistant/` (new) | Panel, conversation, approvals, Preferences → AI, keychain host. | scancraft-agent, scancraft-ui |
| `apps/scancraft-cli` (new) or a `scancraft mcp` subcommand | Starts the MCP server. [H] A separate small console binary is simpler on Windows than reusing the GUI binary's stdio. | scancraft-mcp |

---

## 3. A ScanCraft tool surface

### 3.1 First the missing layer: commands

SolidCraft's whole design rests on "every action is a named command with JSON parameters". ScanCraft
has no such layer today: actions are methods on `ScanCraftApp` (`align_auto`, `accept_preview`,
`refine_all`, `merge_passes`, `run_tool(Tool)`; `apps/scancraft/src/app_project.rs:184-488`,
`apps/scancraft/src/app.rs:1336-1462`) whose parameters live in `Settings`
(`app.rs:86-108`), plus a development step list for `--auto` (`apps/scancraft/src/dev.rs:6-10`).
Long work goes through `spawn_job`, which runs one job at a time and **silently returns** when busy
(`app.rs:1323-1333`); results arrive in `poll_jobs` (`app.rs:1464-1531`) with no request id.

The prerequisite is therefore a command registry:

- `CommandSpec { id, label, stage, params: JSON Schema, result schema, kind }` with
  `kind ∈ {Read, Preview, Change, File, Destructive, Scanner}`, declared next to each command, not
  guessed from parameter names as SolidCraft's gate must (`gate.rs:33-112`).
- `dispatch(app, id, params) -> Result<Value | Pending(job_id), Error>`: sets the matching
  `Settings` fields, calls the existing method, and returns the `*_blocked()` reason as an error
  instead of doing nothing (`align_blocked`, `refine_blocked`, `merge_blocked`, `tool_blocked`
  already give the reasons: `app_project.rs:87`, `:398`, `:424`, `app.rs:1258`).
- Jobs get an id; `JobResult` carries it, so a waiting tool call gets *its* result. A busy app
  returns "busy: <label>" so the agent can wait (`running`/`pending` tools).
- The UI buttons and `--auto` call the same commands, so the agent never has abilities the user
  lacks.

### 3.2 Read-only inspection (run without asking, never counted as changes)

| Command | Returns | Built on |
| --- | --- | --- |
| `project.inspect` | name, path, dirty, reference pass, passes (id, name, frames, kept keyframes, points, visible, included, weight, scan config: light/tracking/accuracy, start: moved/same, alignment state + method + `Quality`), merged/cleaned/mesh summaries, undo labels, busy job. No device serial, no paths outside the project. | `ScanCraftApp` fields `app.rs:172-260`, `PassMeta` `project.rs:98-112` |
| `pass.inspect {pass, keyframes?}` | bbox, centroid, principal axes, normal-orientation summary (share pointing away from centroid: tells outside vs inside scans), keyframe list with `rmse_mm`, `visibility_conflict`, discarded. | `KeyframeInfo` `model.rs:196-224` |
| `align.quality {source, target?, transform?}` | `Quality {overlap, rmse_mm, visibility_conflict, inliers}`, `verdict`, `score`, and once plan item 1 lands the shell terms (wall thickness, piercing, below-support-plane share, sliding warning). `transform` = `current`, `preview`, `candidate:<n>` or a 4×4. | `scancraft_align::quality::evaluate` (`crates/scancraft-align/src/quality.rs`) |
| `align.compare {source, target, transform?, lateral_mm=2, max_mm=15, bins}` | **Signed offsets** of source points along the target's normals, split into same-facing and opposite-facing groups: share, median, IQR, p10/p90, histogram; coverage within 0.5/1/3/15 mm; "poke-through" share (> 0.7 mm on the wrong side); crossing indicator (mass on both sides); share below the target's table plane. | new (`scancraft-inspect`); today's Python checks `E:\Claude\eval\align-revo\verify\offsets_3_2.py`, `offset_field.py` |
| `align.diff_transforms {pass, a, b}` | rotation angle (deg), translation (mm), largest displacement on the pass's points. | `rotation_angle`, `displacement_on` (`crates/scancraft-align/src/lib.rs:98-107`) |
| `align.candidates {pass}` | the candidates of the last auto alignment with their quality (needs `align_global` to keep its top-N; [H]). | `GlobalConfig::candidates` (`align_ply.rs:11`) |
| `geometry.stats {layer}` | for `passes`, `merged`, `cleaned`, `mesh`: counts, bbox, density (mean NN spacing), dominant plane. Mesh: watertight, holes, triangle count. | `MeshAnalysis`, `Mesher::analyze` |
| `view.render {layers, colour_by, view, camera?, width?, height?, section?}` | PNG image block + JSON (camera, scale mm/px). `layers`: passes (with `transform` overrides, e.g. a preview), merged, cleaned, mesh. `colour_by`: `pass`, `normal`, `offset` (needs a `target`), `quality`, `colour` (texture, see 5.1). `view`: `top/front/…`, `principal:1..3` (along the part's principal axes, as `align_ply` does), `home`, or an orbit camera. | CPU renderer from `align_ply.rs:544`; later an offscreen wgpu path of `scancraft-render` |
| `view.section {plane | axis+at, width_mm=1, layers}` | PNG of the slab, each layer in its colour, plus per-section statistics: gap between layers along the section (wall thickness profile), crossing points. | new |
| `view.offset_map {source, target, transform?}` | PNG of the source coloured by signed offset (diverging scale, legend), as `offset_map_default.png` today. | new |
| `ui.screenshot` (bridged only) | the app window as the user sees it. | egui viewport screenshot (`dev.rs` `shot:`) |
| `device.status` | connected or not, model, firmware, mode, capturing, frame rates, tracking state. **No serial number, IP or Wi-Fi data.** | `Connection`, `DeviceInfo` |

All replies are compact by default: statistics and images, never raw point arrays. The one way
to get points out is `export`, a file action.

### 3.3 Actions

| Command | Kind | Default rule (*Auto-allow edits* mode) |
| --- | --- | --- |
| `align.auto {pass, options?}` (both directions, voxel list, seed: `align_robust` from the plan) | Preview | Run. Only creates the preview; the project does not change (`app_project.rs:227-253`). |
| `align.pairs {pass, pairs:[[model xyz],[pass xyz]]…}` | Preview | Run (the agent can pick pairs from `pass.inspect`/renders; [H] useful only for strong models). |
| `align.accept` / `align.discard` | Change / Read | Run, undoable, shown in the turn summary. |
| `align.all` (pose graph, plan item 1) | Preview → Change | Run; produces previews per pass. |
| `align.refine` (Refine as a preview, plan item 1) | Preview → Change | Run. |
| `align.reset {pass}`, `align.set_reference {pass}` | Change | Run. |
| `pass.set_included`, `pass.set_weight`, `pass.discard_frames {pass, from|ids}` | Change | Run. |
| `object.isolate {target, options}` (`segment_object`, plan item 1), `clean.remove_plane`, `clean.outliers`, `clean.downsample`, `clean.crop` | Change | Run. |
| `merge.run {outliers?, sigma?}`, `merge.discard` | Change | Run. |
| `mesh.reconstruct {cell_mm}`, `mesh.smooth`, `mesh.decimate`, `mesh.discard` | Change | Run. |
| `undo` / `redo` | Change | Run within the turn; **Ask** past the turn's first change (as SolidCraft). |
| `export {format, path}` | File | **Ask**, path inside the allowed folders. |
| `project.save {path?}`, `project.open`, `project.new`, `project.add`, `import {path}` | File / Destructive | **Ask**. |
| `pass.delete`, `session.clear` | Destructive | **Ask** (removes captured data; undo keeps only 8 snapshots). |
| `settings.*` (app preferences), `assistant.*`, record dir, AI pack download | — | **Refuse.** |
| `device.connect`, `device.start_preview`, `device.start_scan`, `device.continue_scan`, `device.set_parameter`, `device.set_mode` | Scanner | **Ask, always**, even in *Allow everything*; no "Always allow" button. Off entirely unless Preferences → AI → "Let the assistant operate the scanner" is on (default off). See 5.2. |
| `device.stop_scan`, `device.stop_preview`, `device.pause` | Scanner (safe) | Run: stopping cleanly is always the safe direction. |
| Anything reaching the driver's root shell (`system_cmd`, `download`), register writes, reboot, firmware, laser calibration writes | — | **Refuse, always.** Not exposed as commands at all. |

Permission modes, per-agent overrides, "Always allow", per-turn limit, 4-minute timeout, refusal
outside a turn: as SolidCraft (section 1.6). Scanner commands never come under "Always allow".

### 3.4 One undo step per message

ScanCraft's `History` keeps at most 8 full snapshots (`model.rs:445-450`), each holding pass
clouds, merged, cleaned and mesh. A turn that runs ten commands would evict all of the user's own
undo steps. So: at turn start push one snapshot "Assistant: <message>", and while the turn runs
**suppress** further pushes (a `history.group` flag in `ScanCraftApp::snapshot` callers). Undo then
takes the whole turn back. Memory: one snapshot per turn instead of one per command.

### 3.5 What today's verification agents did, as tool calls

The verification on the three Revo scans (`E:\Claude\eval\align_result.txt`,
`E:\Claude\eval\align-revo\verify\`) used ad-hoc Python on PLY dumps: renders along principal
axes, signed offsets along the target's normals split by normal direction, wall-thickness
statistics, poke-through share, below-table share, sections, offset maps, and comparing transforms
between runs. With the tools above a model does the same inside ScanCraft:

```text
1  project.inspect                                    -> S1 reference, S3 previewed onto S1, quality Good
2  pass.inspect {pass: 1}; pass.inspect {pass: 3}     -> S1 normals 99.7 % outward (outside scan),
                                                         S3 concave (inside scan)
3  view.render {layers: [{pass:3, transform:"preview"}, {pass:1}], colour_by:"pass",
                view:"principal:1"}  (and principal:2, principal:3)   -> look for nesting vs crossing
4  align.compare {source:3, target:1, transform:"preview"}
       -> opposite-facing 52 %: median -3.08 mm, IQR -3.19..-2.94; poke-through 0.06 %;
          same-facing 17.7 % at 0.00 mm (rim); below table 0 %
5  view.section {axis:"principal:2", at:"centre", width_mm:1, layers:[3 preview, 1]}
       -> constant gap ~3 mm along the profile, no crossings
6  view.offset_map {source:3, target:1, transform:"preview"}   -> uniform colour band
7  align.diff_transforms {pass:3, a:"preview", b:"candidate:2"} -> 0.3 deg / 3.2 mm: runs agree
8  verdict: correct (constant positive wall, never pierces) -> align.accept
   or: refuted (offsets on both sides = crossing; 83 % within 1 mm between an outside and an
       inside scan = "nested bowls") -> align.discard, align.auto {pass:3, options:{both_directions:true}}
```

The numbers in step 4 are the ones recorded in `E:\Claude\eval\merged\stats.txt` and
`align_result.txt`; they are the regression targets for `align.compare` (WP3). A bundled skill
(`scancraft-aligning`) teaches the procedure, including the "try to refute it" habit that caught
the nested-bowls and tilted-wall failures: check normal orientation first, never trust overlap
alone between an outside and an inside scan, require a constant positive wall and no piercing,
check the table, and compare both directions and several runs.

---

## 4. Work packages

Sizes: S ≈ 1 day, M ≈ 2–4 days, L ≈ 1–2 weeks of focused work. The alignment-and-merge core
(plan item 1 in `merge-and-ai.md`: shell quality, `align_robust`, `align_all`, `segment_object`,
Refine as preview) comes first; several tools below wrap it.

| # | Package | Size | Depends on | Done when |
| --- | --- | --- | --- | --- |
| WP1 | **Command layer** (`commands.rs`): registry with kinds and JSON Schemas, dispatcher over existing methods, job ids in `JobResult`, busy → error, `--auto` re-based on commands, tests per command on the demo project. | L | — | Every Align/Clean/Mesh/Export/Project button calls a command; `--auto` uses them. |
| WP2 | **`scancraft-project` crate**: move `project.rs` and the pass data model out of the app; headless load + rebuild. | M | — | A test loads a saved project and lists passes without egui. |
| WP3 | **Inspection kit** (`scancraft-inspect`): `align.compare` (signed offsets, shell stats), sections, offset maps, CPU renderer from `align_ply.rs`, principal-axis views; Rust ports of today's Python checks. | L | plan item 1 for shell quality | On the three Revo scans it reproduces wall −3.08 mm (IQR ±0.15) and poke-through < 0.1 % for S3→S1 (ignored test with `SCANCRAFT_ALIGN_DATA`). |
| WP4 | **MCP server, headless** (`scancraft-mcp`, `scancraft-cli mcp --project <dir>`): JSON-RPC from SolidCraft, `PathScope` copy, tools of 3.2–3.3 except Scanner, annotations, `skill_reference`, tests like `crates/mcp/src/tests.rs`. | M | WP1 (specs), WP2, WP3 | Claude Code with `.mcp.json` judges an alignment in a saved project. **First user-visible value**: external agents, no UI work. |
| WP5 | **Skills and instructions**: `skills/scancraft-aligning`, `-cleaning`, `-merging`; server instructions; test that skills name only real tools. | S–M | WP4 | Skills bundled and readable via `skill_reference`. |
| WP6 | **Control endpoint** (`control.rs`): loopback, per-launch token in `%LOCALAPPDATA%\ScanCraft\control`, request queue drained each frame, `still running`/`pending`; bridged MCP (`--connect`). Off unless the assistant or a user setting turns it on. | M | WP1 | `scancraft-cli mcp --connect` drives the open window. |
| WP7 | **`scancraft-agent` crate**: port backend, ACP, API (OpenAI + Anthropic), process, discover, Markdown; port their tests. | M | — | Fake ACP agent and a mock HTTP endpoint pass the ported tests on Windows. |
| WP8 | **Gate and permissions**: verdict rules from command kinds, modes, always-allow, per-turn limits, scanner category, turn validity. | S–M | WP1, WP7 | Table-driven tests for every command kind and mode. |
| WP9 | **Assistant panel** (`apps/scancraft/src/assistant/`): panel beside the properties card, conversation, tool cards, approval cards, queue, Stop/New chat, Attach view, Attach selection (`Node` chips: pass, keyframe, merged, mesh), per-turn undo grouping, Preferences → AI with master switch, Add agent dialog, keychain `Secrets`, data-destination label (Local/Cloud). | L | WP6–WP8 | A local Ollama model and Claude Code each align the demo project from the panel. |
| WP10 | **Benchmark** (`scancraft-cli bench-assistant`): tasks on synthetic projects with ground truth (`dev.rs::write_demo_project`: second pass flipped and moved), e.g. "align all passes", "is pass 2 correctly aligned?" (with a deliberately wrong preview), "clean, merge and mesh", "export an STL"; checks on project state (pose error vs ground truth, accepted/rejected), failure categories. | M | WP6, WP9 | Results table for at least one local and one cloud model. |
| WP11 | **Scanner tools** (optional, last): `device.*` commands behind their own preference, always-ask, clean stop on turn end, rate limits; tests with the mock driver and replay only. | M | WP9 | Mock-driver tests; one supervised real-scanner session following HANDOFF §3. |
| WP12 | **Docs**: `docs/assistant.md` (user guide), `docs/mcp.md`, update `merge-and-ai.md` §6–7, `ATTRIBUTION.md`, README. | S | each WP | — |

Order: WP1 and WP2 in parallel → WP3 → WP4 + WP5 (milestone 1: headless MCP for Claude Code
and other external agents; enough for "judge my alignment" today) → WP6, WP7, WP8 → WP9
(milestone 2: in-app chat) → WP10 → WP11 → WP12 throughout. Rough total: 7–10 weeks of focused
work, of which WP1, WP3 and WP9 are the big ones.

---

## 5. Risks and rules

### 5.1 Privacy

- **Local by default.** Nothing leaves the PC unless the user adds a cloud endpoint or runs a
  coding agent that talks to its own service. The first preset offered is a local one (Ollama /
  LM Studio on `localhost`). The panel header shows where data goes ("Local model" vs "Sends
  renders and numbers to <host>").
- **What a model receives:** statistics and rendered images of geometry. Raw clouds, meshes and
  project files are never sent through tools; `export` is a file action that asks.
- **Camera images and colour.** IR and colour frames (and colour keyframes, which the plan stores
  by default) can show the room and people. Tools never return camera frames; `colour_by: colour`
  renders the object's texture only and, for cloud providers, asks first [H, proposed].
- **Keys:** endpoint keys only in Windows Credential Manager via `keyring` (as SolidCraft), never
  in settings, project files, logs, recordings or crash data; sent only over HTTPS or to loopback.
  Coding agents keep their own logins; ScanCraft never reads them.
- **Device identity:** `device.status` and `project.inspect` never return the serial number, IP or
  Wi-Fi configuration (public repo rule in `AGENTS.md`; also keeps them out of cloud prompts).
- **Prompt injection:** project and pass names, PLY headers and imported metadata are data. The
  agent's own shell/file/web tools are switched off where possible, its working folder is empty,
  and the file scope is enforced by the server, not by the model.

### 5.2 Scanner control and the safety rules (`docs/HANDOFF.md` §3)

- **No scanner control without explicit approval.** Scanner commands are off unless the user turns
  them on; when on, every start, connect, mode or parameter change asks, in every mode, and is
  never remembered. Stopping is always allowed.
- **One client at a time.** The agent never opens its own connection: it acts only through the
  app's existing `Connection` and the driver's lock file (`%TEMP%\scancraft-scanner.lock`). The
  headless MCP server has no driver at all and refuses `device.*`.
- **Never kill a streaming process.** The scanner belongs to the app process, not to the agent, so
  stopping or killing the agent cannot cut a stream. When a turn ends, is cancelled, or the agent
  exits while a capture it started is running, the app calls its own clean `stop_scan`
  (emitters off, streams closed) instead of leaving it or dropping the connection.
- **No raw device access.** The root-shell control CGI (`system_cmd`, `download`), register
  writes, reboot, firmware and laser-plane calibration writes are not commands and cannot be
  reached. A hung media server is recovered by the user (`tools/scanner/scanner_reboot.sh`), not
  by the agent.
- **Start order and DSP reloads.** Mode starts go through `device.rs::start_stream` only, never
  reordered; rate-limit start/stop and mode changes (each format change reloads the DSP) [H: one
  start per 10 s].
- **Disk.** Streams are several MB/s and filled a drive on 2026-11-01. The agent cannot enable
  `SCANCRAFT_RECORD_DIR` or raw recording; [H] a capture started by the agent gets a time limit
  and a free-space check.

### 5.3 Other risks

| Risk | Mitigation |
| --- | --- |
| Wrong verdicts: metrics fooled (nested bowls scored Good, correct inside/outside nesting scored Poor, `merge-and-ai.md` §3). A model that trusts `verdict` repeats the error. | Shell-aware quality first (plan item 1); skills require evidence (offset groups, section, render) before accepting; accept is undoable and listed in the turn summary; benchmark tasks with deliberately wrong previews. |
| Small local models can't drive tools (SolidCraft saw "wrote the call as text", "only talked"). | Few, well-described tools; compact replies; the benchmark (WP10) tells users which models work; capability warning for Ollama models without `tools` (SolidCraft does this). |
| Long jobs (global alignment, merge, meshing: seconds to minutes) and a single job slot. | Job ids, `busy` errors, `running`/`pending` tools, MCP client timeout above 300 s (SolidCraft's numbers). |
| Memory: snapshots of full clouds; renders of millions of points. | One snapshot per turn; renders on a sampled cloud; image size cap (2048 px, as SolidCraft). |
| Code size of the port (~400 KB of SolidCraft agent code) and drift from upstream. | Port the protocol layers nearly verbatim (easy to diff and pull fixes), re-implement only the UI; record the upstream version in `ATTRIBUTION.md`. |
| Windows process trees (agents spawn node helpers). | SolidCraft's `process.rs` with its leak tests; job objects later (an open item there too). |
| Cost and rate limits of cloud models with images. | Show image count per turn; keep at most 2 old pictures in history (as SolidCraft). |
| Licence mistakes when copying. | `ATTRIBUTION.md` per file; no vendoring of agents or their SDKs. |
