# 3D Point Cloud Reconstruction & Calibration Math

This document details the coordinate systems, optical calibration parameters, depth decoding math, and RGB texturing pipeline for the **Revopoint POP 4**.

---

## 1. Coordinate Frames & Units

- **World / IR Camera Frame**:
  - Origin $(0, 0, 0)$: Optical center of the **Left IR Camera**.
  - $+X$: Right (towards the Right IR camera).
  - $+Y$: Downwards (following standard computer vision conventions).
  - $+Z$: Forward along the optical axis (depth into the scene).
  - **Distance Unit**: Millimeters ($\text{mm}$).
- **Depth Map Units**:
  - The rectified depth frame delivered by the scanner is a 2D array of $640 \times 512$ unsigned 16-bit integers (`uint16`).
  - **Depth Scale Factor**: `0.01 mm` ($10\ \mu\text{m}$ per LSB), verified from `3DCamera.dll` constant `0x3c23d70a` for `CameraType = 32 (POP_4)`:
    $$\text{Depth in mm:}\quad Z(u, v) = d_{raw}(u, v) \times 0.01$$
  - A raw value of `0` denotes an invalid/unmeasured pixel (shadow, out of range, or absorbed).
  - Valid measurement range for POP 4: $150\text{ mm} \le Z \le 400\text{ mm}$ (raw values $15000 \le d_{raw} \le 40000$).

---

## 2. Optical Calibration Parameters

The scanner hosts its factory calibration files in `/data/camparam/`, retrievable via HTTP CGI:
`GET http://<scanner-ip>/cgi-bin/zx_cmd.cgi?download=/data/camparam/<filename>`

### 2.1 Calibration Files Summary

| File | Size | Format | Description |
|---|---|---|---|
| `camparam.yaml` | ~2.1 KB | OpenCV YAML | Complete stereo calibration ($M_1, D_1, M_2, D_2, R, T, R_1, R_2, P_1, P_2, Q$) |
| `Prgb.bin` | 40 B | Binary | RGB camera image size ($1280 \times 1024$) + $3 \times 3$ intrinsic matrix $K_{RGB}$ |
| `Distort.bin` | 20 B | Binary | RGB camera 5-parameter Brown-Conrady distortion ($k_1, k_2, p_1, p_2, k_3$) |
| `LC_RT.bin` | 48 B | Binary | Left IR to RGB camera extrinsics: $3 \times 3$ rotation matrix $R_{RGB}$ and $3 \times 1$ translation $T_{RGB}$ |
| `mapparamL.bin` | 148 B | Binary | Left IR rectification lookup table configuration ($1024 \times 1280$, $K_{L}, D_{L}$) |
| `mapparamR.bin` | 148 B | Binary | Right IR rectification lookup table configuration ($1024 \times 1280$, $K_{R}, D_{R}$) |

---

## 3. Mathematical Formulas

### 3.1 Depth-to-Point Back-Projection (Unprojecting Depth to 3D)

For a rectified depth frame at resolution $W \times H$ ($640 \times 512$), given the rectified camera principal point $(c_x, c_y)$ and focal length $(f_x, f_y)$:

$$X = \frac{(u - c_x) \cdot Z}{f_x}$$

$$Y = \frac{(v - c_y) \cdot Z}{f_y}$$

$$Z = d_{raw}(u, v) \times 0.01\text{ mm}$$

Where:
- $u \in [0, W-1]$ (horizontal pixel index).
- $v \in [0, H-1]$ (vertical pixel index).
- $f_x, f_y$: Rectified focal length in pixels (from matrix $P_1$ or $Q$).
- $c_x, c_y$: Rectified principal point in pixels.

#### Reprojection using Disparity-to-Depth Matrix $Q$

The OpenCV $4 \times 4$ reprojection matrix $Q$ provided in `camparam.yaml`:

$$\begin{bmatrix} X \\ Y \\ Z \\ W \end{bmatrix} = Q \cdot \begin{bmatrix} u \\ v \\ d \\ 1 \end{bmatrix}$$

$$P_{3D} = \begin{bmatrix} X/W \\ Y/W \\ Z/W \end{bmatrix}$$

Where $d$ is disparity ($d = \frac{f \cdot |Tx|}{Z}$).

---

### 3.2 RGB Color Alignment & Texture Mapping

To project the 3D point $P_{IR} = [X, Y, Z]^T$ onto the RGB camera sensor:

#### Step 1: Transform 3D point to RGB camera coordinate frame

Using the extrinsic parameters from `LC_RT.bin`:

$$P_{RGB} = R_{RGB} \cdot P_{IR} + T_{RGB}$$

Where:
- $R_{RGB}$ is the $3 \times 3$ rotation matrix (row-major `float32`).
- $T_{RGB}$ is the $3 \times 1$ translation vector ($[-24.10, 0.08, -2.71]^T\text{ mm}$).

#### Step 2: Normalized Coordinates

$$x' = \frac{X_{RGB}}{Z_{RGB}}, \quad y' = \frac{Y_{RGB}}{Z_{RGB}}$$

$$r^2 = (x')^2 + (y')^2$$

#### Step 3: Lens Distortion Correction (Brown-Conrady)

Using coefficients $(k_1, k_2, p_1, p_2, k_3)$ from `Distort.bin`:

$$x_{distorted} = x' \cdot (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2 p_1 x' y' + p_2 (r^2 + 2(x')^2)$$

$$y_{distorted} = y' \cdot (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2(y')^2) + 2 p_2 x' y'$$

#### Step 4: Project to RGB Pixel Coordinates

Using intrinsic matrix $K_{RGB}$ from `Prgb.bin`:

$$u_{RGB} = f_{x, RGB} \cdot x_{distorted} + c_{x, RGB}$$

$$v_{RGB} = f_{y, RGB} \cdot y_{distorted} + c_{y, RGB}$$

If $0 \le u_{RGB} < 1280$ and $0 \le v_{RGB} < 1024$, sample the RGB pixel color at $(u_{RGB}, v_{RGB})$ via bilinear interpolation.

---

## 4. Reference Python Implementation

```python
import numpy as np

def depth_to_point_cloud(depth_u16: np.ndarray, fx: float, fy: float, cx: float, cy: float, depth_scale: float = 0.01):
    """
    Convert a 2D uint16 depth image into a 3D point cloud array (N, 3).
    
    Args:
        depth_u16: (H, W) uint16 array
        fx, fy: focal length in pixels
        cx, cy: optical center in pixels
        depth_scale: depth conversion factor (0.01 for POP 4 -> millimeters)
    Returns:
        points_3d: (N, 3) float32 array of [X, Y, Z] in millimeters
        valid_mask: (H, W) boolean mask of valid points
    """
    h, w = depth_u16.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    
    # Filter valid depth readings (non-zero)
    valid_mask = depth_u16 > 0
    z = depth_u16.astype(np.float32) * depth_scale
    
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    # Stack valid 3D points
    points_3d = np.stack((x[valid_mask], y[valid_mask], z[valid_mask]), axis=-1)
    return points_3d, valid_mask
```
