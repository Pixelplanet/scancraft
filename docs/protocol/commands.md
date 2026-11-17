# Revopoint POP 4 Protocol Commands & Endpoints

> **Historical document.** Written during the first protocol analysis; several statements here were later contradicted by measurement. Current state: [the documentation map](../README.md) and [device-internals.md](device-internals.md).

> Verified against live hardware (Revopoint POP 4, S/N `D00000000B0S00E1B` at `192.168.178.34`) and static analysis of `3DCamera.dll`.

---

## 1. Network Topology & Ports

The POP 4 runs an embedded Linux system (web server `lighttpd/1.4.61`) and listens on the following ports:

| Port | Protocol | Purpose | Status / Verification |
| :--- | :--- | :--- | :--- |
| **80** | TCP (HTTP) | CGI Command Plane (`/cgi-bin/zx_cmd.cgi`) & File Transfers | Confirmed live |
| **8871** | TCP | Device Data Socket (`DeviceDataComTcp`) with `0xAA55AA55` binary framing | Confirmed live |
| **9999** | TCP | Real-time IMU Telemetry (Quaternion, Accel, Gyro) at ~125 Hz | Confirmed live |
| **22** | TCP (SSH) | Embedded Linux root/maintenance shell | Confirmed open |

---

## 2. HTTP CGI Control Plane (`/cgi-bin/zx_cmd.cgi`)

All commands are issued via HTTP GET/POST queries to `/cgi-bin/zx_cmd.cgi`.

### A. Device Identification & Diagnostics

| Query | Live Return Example | Description |
| :--- | :--- | :--- |
| `?get_sn` | `D00000000B0S00E1B` | Returns device serial number string |
| `?get_thermal` | `{"thermal": 24130}` | Main sensor temperature ($24.13^\circ\text{C}$) |
| `?get_ntcTemperature` | `{"ntcCnt":4,"ntc":[24640,19965,19718,20170]}` | Temperatures of 4 onboard NTC thermistors ($^\circ\text{C} \times 1000$) |
| `?get_eth_info` | `{eth_en=x0;eth_type=\x00 eth_static_default_ip=}` | Ethernet adapter configuration status |

### B. Sensor Stream Resolutions & Parameters

| Query | Live Return Example | Description |
| :--- | :--- | :--- |
| `?cam_type=mipi&get_depth_reso` | `{"curr-resolution":"1280x1024x3"}`<br>`{"depth-resolution":"640x512x2"}`<br>`{"cali-resolution":"1280x1024x2-640x512x2-640x1024x2-640x3072x2"}`<br>`{"depth+ir-resolution":"640x512x4"}`<br>`{"mjpg-resolution":"..."}` | Returns supported sensor pipeline modes and pixel formats |
| `?cam_type=mipi&get_exposureRange` | `{"min":4000, "max":42000, "step":1, "default":5000}` | MIPI stereo IR sensor exposure time bounds ($\mu\text{s}$) |
| `?cam_type=mipi&get_trigger_mode` | `{"result": 0}` | Sensor trigger mode (`0` = internal continuous) |
| `?cam_type=usb&&get_resolution=1` | `{"curr-resolution":"1280x800"}{"h264-resolution":"1280x1024"}` | RGB sensor formats |
| `?close_stream_all` | `{result: 0}` | Stops and tears down active camera streams |

### C. Direct Calibration File Download

Calibration tables stored on the scanner's flash `/data/camparam/` are downloadable directly via HTTP:

| Endpoint Query | Size | Content Description | Binary Layout |
| :--- | :--- | :--- | :--- |
| `?download=/data/camparam/camparam.yaml` | 2,058 B | Full OpenCV YAML stereo calibration (intrinsics, baseline $Tx = -116.8\text{ mm}$, $Q$ disparity matrix) | Plaintext YAML (`%YAML:1.0`) |
| `?download=/data/camparam/Prgb.bin` | 40 B | RGB camera image size and $3 \times 3$ intrinsic matrix | `uint16_t w (1280), h (1024)` + `float32[9] K_rgb` |
| `?download=/data/camparam/Distort.bin` | 20 B | RGB camera 5-coefficient lens distortion array | `float32[5]` ($k_1, k_2, p_1, p_2, k_3$) |
| `?download=/data/camparam/LC_RT.bin` | 48 B | Left IR to RGB camera extrinsics | `float32[9] R (3x3)` + `float32[3] T (3x1)` ($Tx = -24.10, Ty = 0.08, Tz = -2.71\text{ mm}$) |
| `?download=/data/camparam/mapparamL.bin` | 148 B | Left IR sensor rectification parameters | `uint32_t[3] (1024, 1280, 5)` + `float32[34]` ($R_1$, $K_L$, $D_L$) |
| `?download=/data/camparam/mapparamR.bin` | 148 B | Right IR sensor rectification parameters | `uint32_t[3] (1024, 1280, 5)` + `float32[34]` ($R_2$, $K_R$, $D_R$) |

---

## 3. Sensor Modes & Pixel Formats (`get_depth_reso`)

Returned by `GET /cgi-bin/zx_cmd.cgi?cam_type=mipi&get_depth_reso`:

- `curr-resolution`: `1280x1024x3` — Raw RGB image sensor ($1280 \times 1024$, 3 channels = 24-bit BGR/RGB).
- `depth-resolution`: `640x512x2` — Standard rectified depth stream ($640 \times 512$, 2 bytes/pixel = 16-bit uint16 depth in units of $0.01\text{ mm}$).
- `depth+ir-resolution`: `640x512x4` — Interleaved Depth (2B) + IR Brightness (2B) ($640 \times 512$, 4 bytes/pixel).
- `depth+ir+p-resolution`: `640x512x6` — Depth (2B) + IR (2B) + Phase/Pattern (2B) ($640 \times 512$, 6 bytes/pixel).
- `depth+st+sp-resolution`: `640x512x10` — Depth (2B) + Stereo Texture (4B) + Speckle (4B) ($640 \times 512$, 10 bytes/pixel).
- `mjpg-resolution`: `1280x1024x2-640x512x2-640x1024x2-640x3072x2` — Supported MJPEG preview resolutions.

---

## 4. Real-time IMU Telemetry Stream (Port 9999)

Port 9999 streams continuous 60-byte telemetry packets over a raw TCP socket at ~125 Hz:

```
[ 0..7  ] uint64_t timestamp_ms  (e.g., 197897, 197904, 197912...)
[ 8..23 ] float[4]  Quaternion   (qw, qx, qy, qz - normalized 3D orientation)
[ 24..35] float[3]  Gyroscope    (gx, gy, gz in rad/s or deg/s)
[ 36..47] float[3]  Accelerometer(ax, ay, az in Gs, norm = 1.0G)
[ 48..59] float[3]  Magnetometer / Euler attitude
```

---

## 4. Binary Data Channel (Port 8871)

Managed by `DeviceDataComTcp::start(host, 8871)` and parsed by `cs::DeviceDataPacketParser`. Employs the `0xAA55AA55` framing layer documented in [framing.md](file:///c:/Projects/3D%20Scanner/docs/protocol/framing.md).
