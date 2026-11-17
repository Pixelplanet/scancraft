# Solidcraft Architectural Analysis & Revo Scan 6 Feature Decomposition

> **Historical document.** Written during the first protocol analysis; several statements here were later contradicted by measurement. Current state: [the documentation map](README.md).
**Target:** Architecture, UX/UI Design, and Algorithmic Roadmap for a Next-Generation Rust 3D Scanner Application.

---

## Executive Summary

To build an open, modern, and high-performance 3D scanning application for the **Revopoint POP 4** (and future scanners), we conducted an in-depth technical analysis of two systems:
1. **Solidcraft** (`tangency-labs/solidcraft`): A high-performance, open-source parametric 3D CAD application written in pure Rust with an immediate-mode UI (`egui`), a modern WebGPU rendering pipeline (`wgpu`), an off-thread computation engine, and built-in Model Context Protocol (MCP) agent support.
2. **Revo Scan 6** (`E:\Revopoint\Revo Scan 6`): Revopoint's official Windows C++/Qt6 software suite. Through deep PE symbol inspection of `3DCamera.dll`, `zxPointCloudProcessing64.dll` (GPP engine), `cscalibration.dll`, and decomposition of its UI resource catalogs (`RevoScan_en_US.qm`), we extracted the complete functional and algorithmic architecture of the proprietary platform.

This document synthesizes both analyses into a concrete design specification for our custom Rust 3D scanner suite.

---

## Part 1: Deep Analysis of Solidcraft

### 1.1 High-Level Architecture & Layering

Solidcraft follows a strict layered architectural contract designed to guarantee modularity, compilation efficiency, and zero circular dependencies:

```
┌────────────────────────────────────────────────────────┐
│                   Apps Layer                           │
│  apps/solidcraft (Desktop)    apps/solidcraft-cli      │
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                   L5: Presentation                     │
│  solidcraft-ui-egui   solidcraft-gpu   solidcraft-mcp  │
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                   L4: Session & Commands               │
│  solidcraft-engine (Command Registry, Undo/Redo)       │
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                   L3: File I/O                         │
│  solidcraft-io (STEP, STL, OBJ, 3MF, glTF)             │
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                   L2: Document Model                   │
│  solidcraft-doc (Timeline, Parameters, Graph)          │
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                   L1: Domain Kernels                   │
│  solidcraft-kernel    solidcraft-sketch   solidcraft-render
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                   L0: Foundation                       │
│  solidcraft-geom (Vectors, Matrices, Bounds, Rays)     │
└────────────────────────────────────────────────────────┘
```

### 1.2 The UI Design System (`crates/ui-egui`)

Solidcraft’s visual polish is driven by a bespoke immediate-mode design system built atop `egui` and `eframe`:

- **Design Token Palette (`Tokens`)**:
  - Color hierarchy: `file_background`, `file_surface`, `file_border`, `nav_viewport`, `nav_hover`, `nav_press`, `nav_dim`.
  - Contrast styling: Subtle edge bevels (`Bevel` struct with highlight, light, shadow, dark_shadow) providing depth without visual clutter.
  - Border roles: Explicit `Primary`, `Secondary`, `Tertiary`, `Container`, `Control` border roles for visual stability.
- **Componentized `ui_kit`**:
  - Modular widgets: `Container`, `Surface`, `Disclosure`, `Fields` (numeric drag fields with expression evaluators and units), `Gallery`, `History`, `Tree`, `Breadcrumbs`, `Radial` (radial tool gestures).
  - Shortcut Command Box (`sbox`): Quick-action HUD triggered by pressing `S` at the mouse cursor.
- **Off-Thread Concurrency Architecture**:
  - The UI runs locked to monitor refresh rate (60–144 Hz).
  - Expensive geometric operations (tessellation, booleans, sweeps) never run on the UI thread. Instead, they are dispatched as asynchronous background worker jobs via `Applying` and `Deferred` queues with progress channels.
  - While computing, the UI shows non-blocking progress spinners and allows continuous viewport camera interaction.

### 1.3 Viewport & GPU Rendering (`crates/gpu` and `crates/render`)

- **Rendering Backend**:
  - Written in pure WebGPU via `wgpu` (version 30) and WGSL shaders.
  - Integrated into egui via `egui_wgpu::CallbackTrait` (`ViewportCallback`), rendering directly into the viewport frame rectangle.
- **Point Cloud Support (`GpuPoints`)**:
  - Native, highly optimized point cloud rendering primitive already exists in `crates/gpu/src/view.rs`:
    ```rust
    pub struct GpuPoints {
        pub id: u64,
        pub version: u64,
        pub bytes: Arc<Vec<u8>>, // Packed 16 bytes per point: [f32; 3] xyz + [u8; 3] rgb + u8 size
    }
    pub const CLOUD_POINT_SIZE: usize = 16;
    ```
  - Instanced vertex layout with depth testing and configurable point size.
- **Visual Features**:
  - Screen-Space Ambient Occlusion (SSAO) with dynamic sample scaling (32 samples when idle, 8 samples during camera rotation for fluid 120 FPS).
  - Order-Independent Transparency (OIT) using weighted blended OIT or depth peeling.
  - Smooth camera interpolation (`CameraAnim`) with animated axis cube navigation (`navigation_cube.rs`).
  - Interactive on-canvas gizmos (3D translation arrows, rotation tori, and scale handles) with screen-space snapping.

### 1.4 AI Integration via Model Context Protocol (`crates/mcp`)

- Built-in MCP server (`solidcraft-cli mcp`) implementing standard tools:
  - Query scene hierarchy, inspect bodies, measure distances, execute CAD commands, capture viewport screenshots.
  - Supports `--connect` to drive a running desktop GUI session via localhost JSON-lines RPC.

---

## Part 2: Exhaustive Functional Analysis of Revo Scan 6

Through our extraction of 4,536 localization strings and reverse engineering of `zxPointCloudProcessing64.dll`, `3DCamera.dll`, and `cscalibration.dll`, we categorized Revo Scan 6's complete feature pipeline:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        REVO SCAN 6 PIPELINE                            │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
    ┌───────────────────────────────┼───────────────────────────────┐
    ▼                               ▼                               ▼
[1. Hardware & Setup]     [2. Live Acquisition]          [3. Post-Processing]
- Dual HID + UVC (USB)    - Tracking Modes:              - Point Cloud Fusion
- UDP Discovery + CGI     · Feature (geometry)           - Isolation Removal
- Exposure & Laser Mode   · Marker (retroreflective)     - Overlap De-duplication
- Turntable BT Control    · Global Marker Library        - Screened Poisson Mesh
- Guided 9-Pose Calib     · Dark/Reflective Laser Lines  - QEM Simplification
- Distance Indicator      · Body / Face Non-Rigid        - Hole Filling (Curved/Flat)
- IMU Telemetry (138Hz)   - Real-time TSDF Volume        - Multi-View Texture UV
                                                         - Point-Pair / ICP Merge
                                                         - Dimensional Deviation
```

### 2.1 Hardware Interfacing & Device Control
- **Dual Transports**:
  - **USB**: Dual HID interfaces (HID control for commands/registers, secondary HID/bulk endpoint for raw frames) + standard UVC for RGB preview.
  - **Network (WiFi/Ethernet)**: UDP broadcast discovery (port 9999), HTTP CGI command interface (`/cgi-bin/zx_cmd.cgi` on port 80), and high-throughput TCP streaming (port 8871).
- **Sensor Configurations (POP 4 & Line Models)**:
  - Dual infrared depth projectors & stereo infrared cameras.
  - Structured light patterns: Single laser line, parallel multi-line, 30-cross blue laser lines, and diffuse speckle projection.
  - Gain & Brightness calibration tables per scanning mode (`brightnessGainMap_POP4.json`).
  - Real-time IMU (gyroscope + accelerometer) streaming at 138 Hz for motion compensation.
  - Real-time distance indicator bar (`Too Near`, `Near`, `Excellent`, `Good`, `Far`, `Too Far`) derived from depth search range parameters.
- **Accessories**:
  - Bluetooth BLE integration (`simpleble.dll`, `btaccessories.dll`) for:
    - Dual-axis turntable (tilt angle $-30^\circ$ to $+30^\circ$, rotation speed, step intervals, scan sync).
    - Large heavy-duty turntable.
    - Handheld stabilizer gimbal.

### 2.2 Live Scanning & Tracking Modes
1. **Feature Tracking**: Geometric gradient-based alignment for objects with rich contours.
2. **Marker Tracking**: High-precision centroid tracking of retroreflective dot targets.
3. **Global Marker Library**: Loads pre-surveyed marker coordinate frameworks to prevent drift over meter-scale objects.
4. **Dark / High Reflectivity Mode**: Alternating exposure and tuned multi-line laser illumination for specular metals and matte black plastics.
5. **Body / Face Scanning**: Non-rigid tracking algorithms (`DumpNonRigidICP`, `DumpNonRigidGlobalRegistrate`) robust to minor breathing and eye movement at up to 18 FPS.
6. **Relocalization & Loop Closure**: Uses a 100K-word Faiss visual bag-of-words vocabulary tree (`vocab_tree_faiss_flickr100K_words256K.bin`) to instantly re-localize tracking when the scanner loses lock.

### 2.3 Point Cloud Processing (`zxPointCloudProcessing64.dll` GPP Engine)
- **Point Cloud Fusion (`DumpFusePointCloudExtract`, `DumpFusePointCloudUpdate`)**:
  - Integrates accumulated depth frames into an optimized voxel grid.
  - Adjustable point distance resolution (slider: $0.05\text{ mm}$ to $2.00\text{ mm}$).
  - Base filtering ratio to prune low-confidence grazing-angle points.
- **Isolation Removal (`DumpPointCloudConsolidationCalculateIsolation`)**:
  - Spatial cluster analysis detecting and pruning disconnected airborne point clusters.
- **Overlap Removal (`DumpPointCloudConsolidationDetectOverlap`)**:
  - Pruning redundant multi-pass points to prevent artificial surface thickening.
- **Point Cloud Smoothing (`DumpPointCloudConsolidationGeometrySmoothing`)**:
  - Bilateral normal-guided spatial smoothing.

### 2.4 Mesh Processing & Surface Reconstruction
- **Meshing (`DumpPointCloudReconstructMesh`)**:
  - High-quality Screened Poisson Surface Reconstruction and Delaunay triangulation.
  - Watertight implicit wrapping (`DumpComputeTriMeshImplicitWrap`).
- **Mesh Simplification (`DumpMeshSimplificationQuadric`)**:
  - Quadric Error Metric (QEM) decimation down to target face count or ratio (1%–99%).
  - Boundary and texture coordinate preservation (`DumpMeshSimplificationTextureCoord`).
- **Mesh Smoothing & Subdivision**:
  - Curvature-preserving Laplacian smoothing (`DumpGeometryRemeshSubMesh`, `DumpLoopSubdivideSubMesh`).
- **Hole Filling (`DumpFillMeshHole`, `DumpTriangulateLargeMeshHolesApp`)**:
  - Automatic boundary cycle detection.
  - Modes: Flat fill, Curved/Tangent-continuous fill, Bridge tool for dividing complex topological holes.
- **Mesh Healing**:
  - Non-manifold edge repair, degenerate triangle culling, self-intersection resolution (`DumpRemoveSelfIntersection`).

### 2.5 Texture Mapping & Color Processing (`rvtexture.dll`)
- **Keyframe Selection (`ChooseKeyFrames4TextureMap`)**:
  - Automatically scores RGB frames based on sharpness, orientation perpendicularity to surface normals, and lighting uniformity.
- **UV Parameterization (`DumpMeshUnfoldGenerateUVAtlas`, `DumpMeshUnfoldPackUVAtlas`)**:
  - Conformal and isometric UV unfolding and packing into square texture sheets.
- **Multi-View Blending (`DumpCreateTextureImageByRefImages`, `DumpOptimiseMappingFuseImageColorIds`)**:
  - Seam leveling, Poisson color blending, and exposure equalization.

### 2.6 Alignment, Measurement & Quality Inspection
- **Scan Registration & Alignment (`rvalign.dll`, `rvmerge.dll`)**:
  - Manual 1-point and 3-point corresponding landmark alignment.
  - Automatic feature-based alignment.
  - Fine ICP (Iterative Closest Point) global registration.
- **Inspection & Metrology (`Revo Measure`)**:
  - Datum alignment (3-2-1 reference plane alignment, bounding box center).
  - Wall thickness analysis (`DumpMeshMeasureThickness`).
  - Nominal-Actual CAD comparison with colored deviation heatmap.

### 2.7 Next-Gen 3D Gaussian Splatting (3DGS)
- Revo Scan 6 includes `3DGSScene.exe` and CUDA-based neural reconstruction, allowing users to train a 3D Gaussian Splat model directly from dense scan frames and supplemented RGB imagery.

---

## Part 3: Architecture for the New Rust 3D Scanner Application

Borrowing Solidcraft's modularity, token-driven aesthetics, and WebGPU graphics, we design our application: **`revocraft`** (working name).

### 3.1 Proposed Crate Structure

```
revocraft/
├── apps/
│   ├── revocraft-desktop/       # Main egui + wgpu desktop application
│   └── revocraft-cli/           # Headless scanning, batch processing, and MCP server
├── crates/
│   ├── revocraft-driver/        # Scanner hardware abstraction (USB HID/UVC & Network TCP/UDP/HTTP)
│   ├── revocraft-protocol/      # POP 4 packet framing (0xAA55AA55, XOR checksum, QuickLZ)
│   ├── revocraft-slam/          # Real-time tracking (ICP, IMU fusion, keyframe pose estimation)
│   ├── revocraft-fusion/        # TSDF volumetric fusion & Point Cloud generation (wgpu compute)
│   ├── revocraft-mesh/          # Meshing (Poisson recon, QEM decimation, hole filling, repair)
│   ├── revocraft-texture/       # Keyframe selection, xatlas UV packing, GPU texture baking
│   ├── revocraft-measure/       # 3-2-1 alignment, caliper measurements, deviation heatmap
│   ├── revocraft-io/            # PLY, OBJ, STL, 3MF, PCD file import/export
│   ├── revocraft-render/        # wgpu point cloud & mesh renderer with Solidcraft shaders
│   ├── revocraft-ui/            # egui UI Kit, themes, panels, and on-canvas scanner HUD
│   └── revocraft-mcp/           # Model Context Protocol server for AI-driven scanning
```

### 3.2 Visual & Workflow Comparison

| Area | Revo Scan 6 (Qt6 / C++) | Solidcraft Approach (Our Target in Rust) |
|---|---|---|
| **Architecture** | Monolithic Qt, heavy CUDA DLLs (>1GB), external tools | Modular pure Rust crates, thin WGPU backend, sub-second startup |
| **UI Paradigm** | Modal dialogs, separate windows for Revo Measure, clunky Qt widgets | Fluid immediate-mode `egui`, unified workspace, dockable panels |
| **Theme & Style** | Standard Qt dark palette | Solidcraft `Tokens`: sleek dark slate `#2e3440`, subtle bevels, crisp strokes |
| **Viewport** | OpenGL/Vulkan with fixed point styling | Pure `wgpu` with instanced `GpuPoints`, SSAO, view cube, interactive gizmos |
| **Computation** | UI freezes during heavy Poisson meshing; opaque progress | Off-thread worker threadpool (`rayon` + `tokio`), live progressive rendering |
| **Pipeline** | Destructive step-by-step wizard (lose raw frames after fusion) | Non-destructive history timeline: adjust voxel size or decimation retrospectively |
| **Automation & AI** | None (closed proprietary black box) | Built-in MCP server: AI agents can scan, crop, align, mesh, and inspect |

### 3.3 The Core User Experience (Workspaces)

Instead of Revo Scan's rigid tabs, our application uses four integrated workspaces:

```
[ 1. SCAN ] ──────► [ 2. FUSION & CLEANUP ] ──────► [ 3. MESH & TEXTURE ] ──────► [ 4. METROLOGY & EXPORT ]
```

1. **Scan Workspace**:
   - Split viewport: Center shows accumulating real-time 3D point cloud; corner PIP displays live Depth IR stream with heat exposure map and RGB camera preview.
   - On-canvas HUD: Real-time Distance Meter bar (Green = optimal $150\text{--}400\text{ mm}$), tracking quality score, frame count, and turntable controls.
2. **Fusion & Cleanup Workspace**:
   - Voxel-based TSDF consolidation with real-time preview.
   - Interactive clipping box and turntable base plane auto-removal tool.
   - One-click isolation purge and lasso/box selection.
3. **Mesh & Texture Workspace**:
   - High-fidelity Poisson reconstruction and Marching Cubes options.
   - Interactive hole-filling with boundary bridge tool.
   - Fast UV parameterization and multi-view texture projection.
4. **Metrology & Export Workspace**:
   - 3-2-1 datum alignment to establish CAD coordinate systems.
   - Caliper tools: Point-to-point, circle center, angle, and wall thickness.
   - Multi-format export: PLY (colored), OBJ + MTL/PNG, STL (3D printing ready), 3MF, and Gaussian Splat PLY.

---

## Part 4: Implementation Roadmap

- **Phase 1: Driver & Transport Integration**
  - Port Python client protocol parsing (`client/framing.py`, `client/scanner.py`) to `revocraft-protocol` and `revocraft-driver` in Rust.
  - Implement async streaming over TCP port 8871 and USB HID/UVC using `tokio` and `nusb` / `windows-rs`.
- **Phase 2: Viewport & UI Foundation**
  - Scaffold `revocraft-ui` and `revocraft-render` reusing Solidcraft's `Tokens`, `ui_kit`, and `wgpu` `GpuPoints` rendering pipeline.
  - Add camera view cube, distance indicator HUD, and dual IR/RGB video overlays.
- **Phase 3: Point Cloud Processing & Slam**
  - Implement real-time ICP point cloud tracking and IMU pose correction.
  - Implement TSDF voxel grid fusion on GPU compute shaders.
- **Phase 4: Meshing, Texture & Tooling**
  - Integrate Rust Poisson surface reconstruction (`poisson-reconstruction` or `truck-meshalgo`).
  - Add QEM mesh decimation, hole filling, and xatlas UV packing.
- **Phase 5: Metrology & MCP Agent Server**
  - Add coordinate datum alignment and dimensioning calipers.
  - Implement `revocraft-mcp` so AI agents can autonomously operate the scanner.
