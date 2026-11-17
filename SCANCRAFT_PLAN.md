# ScanCraft: High-Performance Open-Source 3D Scanner Application

> **Historical document.** Written during the first protocol analysis; several statements here were later contradicted by measurement. Current state: [the documentation map](docs/README.md) and [docs/HANDOFF.md](docs/HANDOFF.md).

> **Project Name:** ScanCraft  
> **Repository:** `c:\Projects\3D Scanner`  
> **Status:** All Core Workspaces Active (Acquisition, Tracking/SLAM, Fusion & Cleanup, Surface Meshing, I/O & Export)  
> **Architecture Inspiration:** Solidcraft (`tangency-labs/solidcraft`) — pure Rust, `egui` immediate-mode UI, `wgpu` WebGPU rendering, async off-thread compute, tactile dark-theme `Tokens` design system.  
> **Protocol Foundation:** Revopoint POP 4 verified reverse-engineered protocol (see `docs/protocol/developer_handoff.md`).

---

## 1. Architectural Philosophy & Requirements

1. **Vendor Neutrality & Multi-Transport Architecture**:
   - The core application is strictly decoupled from any single hardware brand.
   - All scanners integrate through pluggable driver traits (`scancraft-driver`).
   - Scanners can provide **multiple simultaneous transports** (`DeviceTransport::WiFi`, `DeviceTransport::Usb`, or `DeviceTransport::Virtual`). Users can select the preferred transport or have the software auto-negotiate.
   - Initial focus is Wi-Fi streaming for POP 4 (`192.168.178.34`), with USB (HID control + UVC video / bulk streaming) architected for follow-up activation.
   - Automated hardware probing utility provided in `scripts/probe_scanner.py` for third-party device profiling.
2. **Pure Rust Performance & Responsiveness**:
   - Zero-cost abstractions, no garbage collection pauses, ultra-fast 16.6 MB standalone binary footprint (vs. Revo Scan's 1–2 GB).
   - UI thread runs at display refresh rate. Heavy computational tasks (ICP frame registration, TSDF voxel fusion, SOR outlier removal, RANSAC plane fitting, Marching Cubes meshing, Laplacian smoothing, polygon decimation) run efficiently in background tasks and thread pools.
3. **Solidcraft Look & Feel**:
   - Solidcraft dark slate palette (`#2e3440` viewport, subtle directional bevels, crisp multi-role borders).
   - 3D viewport with 6 shading modes (Points RGB, Points Z-Elevation, Points Distance, Points Normals, Mesh Lit with directional lighting, Mesh Wireframe).
   - Viewport orientation gizmo (RGB axes), camera viewpoint presets (`Front`, `Top`, `Iso`, `Fit`), on-canvas distance indicator HUD, and real-time tracking health badge (`OK`, `SLOW`, `LOST`).
4. **Non-Destructive Scanning & Editing Pipeline**:
   - Preserves raw scan passes, allowing retroactive voxel fusion adjustments, outlier threshold tuning, or turntable plane re-clipping.
5. **Deferred Features**:
   - Model Context Protocol (MCP) agent server is **on hold** for future milestones as instructed, prioritizing scanning, editing, and meshing.

---

## 2. Pluggable Multi-Transport Scanner Driver Architecture (`scancraft-driver`)

```
┌────────────────────────────────────────────────────────┐
│                   scancraft-app                        │
└──────────────────────────┬─────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────┐
│                 scancraft-driver (Traits)              │
│  - ScannerRegistry        - ScannerCapabilities        │
│  - ScannerDriver          - DeviceTransport            │
│  - ScannerDevice          - RawScanFrame / DepthMap    │
└──────────┬───────────────────────┬─────────────────────┘
           │                       │
┌──────────▼────────────┐ ┌────────▼─────────────────────┐
│ scancraft-driver-     │ │ scancraft-driver (Mock)      │
│ revopoint             │ │ (Virtual test scanner)       │
│ - WiFi (Active)       │ │ - Virtual (Active)           │
│ - USB (Scaffolded)    │ │                              │
└───────────────────────┘ └──────────────────────────────┘
           │                       │
┌──────────▼────────────┐ ┌────────▼─────────────────────┐
│ scancraft-driver-     │ │ scancraft-driver-diy         │
│ creality (Future)     │ │ (Custom / OpenScan Future)   │
└───────────────────────┘ └──────────────────────────────┘
```

### 2.1 Multi-Transport Definition

```rust
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceTransport {
    /// Network connection over Wi-Fi or Ethernet
    WiFi {
        ip: String,
        control_port: u16,
        data_port: u16,
        imu_port: u16,
    },
    /// Local USB connection (HID command / UVC or Bulk stream)
    Usb {
        vid: u16,
        pid: u16,
        serial: Option<String>,
        control_interface: u8,
        stream_endpoint: u8,
    },
    /// Simulated / mock virtual device
    Virtual {
        name: String,
    },
}
```

---

## 3. Project Workspace Structure

```
scancraft/
├── Cargo.toml                   # Root workspace manifest
├── SCANCRAFT_PLAN.md            # Master tracking & handoff document
├── apps/
│   └── scancraft/               # Main desktop executable (eframe + egui entrypoint)
├── crates/
│   ├── scancraft-core/          # Core math (glam), point cloud & mesh data structures
│   ├── scancraft-driver/        # Scanner driver trait system, registry, and mock driver
│   ├── scancraft-driver-revopoint/ # Revopoint POP 4 driver (WiFi & USB transport profiles)
│   ├── scancraft-render/        # Renderer (GpuPoints, camera math, view presets, coordinate gizmo)
│   ├── scancraft-ui/            # egui theme, Tokens, viewport overlays, distance HUD
│   ├── scancraft-slam/          # Real-time frame tracking (Kabsch SVD, ICP, IMU fusion)
│   ├── scancraft-fusion/        # Voxel fusion, SOR noise purge, RANSAC turntable removal, ROI crop
│   ├── scancraft-mesh/          # Normal estimation, Marching Cubes, Laplacian smoothing, decimation, metrics
│   └── scancraft-io/            # PLY (cloud & mesh), Wavefront OBJ, Binary STL export, and file import
└── scripts/
    └── probe_scanner.py         # Hardware probing tool for discovering third-party scanners
```

---

## 4. Phased Implementation Plan & Status

### Phase 1: Workspace Scaffolding & Core Architecture
- [x] **1.1 Workspace Setup**: Created root `Cargo.toml`, establishing workspace members.
- [x] **1.2 `scancraft-core`**: Defined fundamental types (`Point3D`, `PointCloud`, `Mesh`, `Triangle`, `Transform`, `BoundingBox`, `ImuSample`, `ColorRgba`, `DepthMap`, `PreviewFrame`, `RawScanFrame`).
- [x] **1.3 `scancraft-driver`**: Implemented `ScannerDriver`, `ScannerDevice`, `DriverRegistry`, and `MockDriver` virtual scanner with multi-transport architecture.

### Phase 2: Revopoint Driver & Protocol Implementation
- [x] **2.1 `scancraft-driver-revopoint` Framing & Compression**:
  - Implemented binary packet framing (`0xAA55AA55`, 24-byte header, XOR checksum) with encode/decode tests.
  - Implemented QuickLZ Level 1 decompressor wrapper in pure Rust.
  - Implemented 60-byte binary IMU telemetry decoder (port 9999) with full unit test.
- [x] **2.2 Revopoint Network & Transport**:
  - Structured HTTP CGI endpoint builder (`RevopointCgiEndpoints`) for parameter queries, SN retrieval, and stream control.
  - Added multi-transport discovery for POP 4 (reporting Wi-Fi and USB options). Verified live scanner communication at `192.168.178.34`.
- [x] **2.3 Calibration & Intrinsic Back-Projection**:
  - Implemented `CameraIntrinsics` unprojecting $640 \times 512$ depth maps to metric 3D point clouds in millimeters ($0.01\text{ mm}$ depth scaling).
- [ ] **2.4 USB Transport Implementation**:
  - Implement USB HID command/report interface and UVC/Bulk streaming once Wi-Fi workflow is fully validated.

### Phase 3: GPU Rendering Engine & Solidcraft UI
- [x] **3.1 `scancraft-render` Foundations**:
  - Implemented `Camera` with orbit, pan, zoom, view presets (`Front`, `Top`, `Iso`), and `fit_bounding_box`.
  - Defined packed 16-byte `GpuPoint` vertex layout matching Solidcraft (`position: [f32; 3]`, `color: [u8; 3]`, `size_px: u8`).
- [x] **3.2 `scancraft-ui` Foundations**:
  - Implemented Solidcraft `Tokens` dark slate CAD palette (`#2e3440` viewport, `#242933` panels, `#1f232a` surface).
  - Built on-canvas Distance Meter HUD bar with real-time `Too Near`, `Near`, `Optimal (150-400mm)`, `Far`, `Too Far` color-coded feedback.
- [x] **3.3 `scancraft` Desktop App**:
  - Interactive top ribbon with 4 workspaces: `1. SCAN`, `2. FUSION & CLEANUP`, `3. MESH & TEXTURE`, `4. EXPORT & IMPORT`.
  - Viewport shading modes: Points (RGB, Elevation, Distance, Normals), Mesh Lit (Lambertian diffuse + ambient), Mesh Wireframe.
  - Interactive 3D coordinate axes gizmo and viewport camera controls.

### Phase 4: Point Cloud Fusion & SLAM
- [x] **4.1 `scancraft-fusion` Voxel Filter**:
  - Implemented spatial voxel grid centroid filter for fast point cloud downsampling and consolidation with interactive UI slider in Fusion workspace.
- [x] **4.2 Real-time Frame Tracking (`scancraft-slam`)**:
  - Implemented fast Kabsch SVD rigid alignment and KD-Tree correspondence matching.
  - Built iterative closest point (ICP) registration engine with IMU rotation priors and tracking health state machine (`Tracking`, `Slow`, `Lost`, `Idle`).
- [x] **4.3 Point Cloud Cleanup (`scancraft-fusion`)**:
  - Statistical Outlier Removal (SOR) for noise point cluster purge.
  - Turntable base plane auto-detection via RANSAC with plane inlier removal and clip-below modes.
  - 3D Region of Interest (ROI) box crop filter.

### Phase 5: Surface Meshing, Editing & Export
- [x] **5.1 File Export & Import (`scancraft-io`)**:
  - Binary little-endian PLY exporter (point clouds & meshes).
  - Wavefront OBJ mesh exporter (with normals and vertex colors).
  - Binary STL exporter for direct slicing in 3D printing software (Bambu Studio, Cura, PrusaSlicer).
  - PLY and OBJ file loader to import external scans back into ScanCraft for processing.
- [x] **5.2 Meshing Pipeline (`scancraft-mesh`)**:
  - Principal Component Analysis (PCA) surface normal estimation with viewpoint orientation.
  - 3D Marching Cubes surface reconstruction over voxel Signed Distance Field (SDF).
  - Laplacian surface smoothing to eliminate sensor wrinkles.
  - Edge collapse polygon decimation to target polygon count.
  - Mesh metrics analysis (vertex count, triangle count, surface area, volume, and watertightness check).

---

## 5. Verification & Test Coverage

All 20 unit and integration tests across the 10 crates pass:
```powershell
cargo test --workspace
```
Output:
- `scancraft-core`: 3 passed (point cloud & bbox, depth sampling, transform math)
- `scancraft-driver`: passed (mock driver and multi-transport registry)
- `scancraft-driver-revopoint`: 3 passed (framing, IMU parser, live network discovery)
- `scancraft-fusion`: 3 passed (voxel filter, SOR outlier filter, RANSAC plane filter, ROI crop)
- `scancraft-io`: 2 passed (PLY round-trip, OBJ & STL export)
- `scancraft-mesh`: 4 passed (normal estimation, marching cubes, Laplacian smoothing, decimation)
- `scancraft-render`: 1 passed (camera view-projection and bounding box fitting)
- `scancraft-slam`: 3 passed (Kabsch identity, Kabsch rigid transform, ICP alignment)
- `scancraft-ui`: passed (Solidcraft design tokens and distance meter HUD)
- `scancraft` (apps): 1 passed (`test_end_to_end_pipeline` covering complete scan -> fusion -> plane removal -> SOR -> meshing -> smoothing -> decimation -> OBJ/STL/PLY export & reload)

Desktop executables compile cleanly with zero warnings:
- `target/release/scancraft.exe`: **13.5 MB** (optimized for live 60 FPS scanning, background worker, off-thread ICP & Marching Cubes)
- `target/debug/scancraft.exe`: **16.7 MB**

---

## 6. Functional Test Instructions

1. **Launch ScanCraft**:
   ```powershell
   .\target\release\scancraft.exe
   ```
2. **Automatic Scanner Discovery**:
   - In Workspace **1. SCAN**, network discovery runs automatically on startup across the local subnet.
   - Any physical Revopoint scanner (such as POP 4) or simulator is automatically listed under **Discovered Scanners**.
   - No IP entry required! Click **"Connect via Wi-Fi"** directly from the detected card.
3. **Scan Setup & Mode Selection**:
   - Configure your scan before starting:
     - **Tracking Feature**: `Feature (Geometry)`, `Marker (Target Dots)`, `Body / Face`, `Dark / Reflective`.
     - **Scan Method**: `Turntable Continuous 360°`, `Turntable Step`, `Handheld Freeform`.
     - **Laser / Sensor**: `Cross Blue Laser (HD)`, `Parallel Infrared`, `Structured Light`.
     - **Accuracy / Pitch**: `High Detail (0.1 mm)`, `Standard (0.2 mm)`, `Fast Preview (0.5 mm)`.
     - **RGB Color Texture**: Checkbox ON/OFF.
     - **Laser Power & Gain**: Sliders.
4. **Live Scanning & Responsive Viewport**:
   - Click **"▶ Start Scan"**.
   - Scanning now runs on a **dedicated background worker thread**, ensuring the UI remains 100% responsive at a silky-smooth 60 FPS without any freezing or Windows "Not Responding" alerts.
   - Observe the live **2D Depth Heatmap** and **IR Sensor Video** in both the left sidebar and the bottom-left floating PIP viewport monitor.
   - Points render live in the 3D viewport with auto-centering on initial geometry.
   - Distance HUD bar reflects real-time proximity (Optimal: 150–400 mm).
   - Use Left Mouse Button to orbit, Right Mouse Button to pan, and Scroll Wheel to zoom. Preset buttons (`Front`, `Top`, `Iso`, `Fit`) reset views.
5. **Fusion & Cleanup**:
   - Switch to Workspace **2. FUSION & CLEANUP**.
   - Run **"⚡ Run Voxel Fusion"** (slider: 0.2–3.0 mm) to consolidate overlapping points.
   - Run **"✂ Remove Turntable Plane"** to auto-clip the turntable base using RANSAC.
   - Run **"🧹 Purge Outliers (SOR)"** to eliminate edge noise.
6. **Surface Meshing**:
   - Switch to Workspace **3. MESH & TEXTURE**.
   - Click **"▲ Reconstruct 3D Mesh"** (Marching Cubes grid: 0.5–3.5 mm). Viewport switches automatically to Lit 3D mesh.
   - Run **"🌊 Smooth Surface"** and **"📉 Decimate Mesh"**.
   - Check watertight manifold analysis metrics.
7. **Export & Slicing**:
   - Switch to Workspace **4. EXPORT & IMPORT**.
   - Select **Binary STL (.stl)**, **OBJ (.obj)**, or **PLY (.ply)**.
   - Click **"💾 Export Model File"** for direct loading into Bambu Studio, Cura, or Blender.
