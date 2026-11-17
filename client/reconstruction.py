"""
3D Point Cloud Reconstruction & RGB Color Alignment for Revopoint POP 4.
"""

from typing import Tuple, Optional
import numpy as np


class PointCloudGenerator:
    """Generates 3D point clouds from Revopoint POP 4 depth maps and calibration data."""

    def __init__(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        depth_scale: float = 0.01,
        rgb_k: Optional[np.ndarray] = None,
        rgb_distortion: Optional[np.ndarray] = None,
        r_ir_to_rgb: Optional[np.ndarray] = None,
        t_ir_to_rgb: Optional[np.ndarray] = None,
    ):
        """
        Initialize the generator.

        Args:
            fx, fy: Focal lengths in pixels (rectified stereo / depth camera).
            cx, cy: Optical center coordinates in pixels.
            depth_scale: Depth scaling factor (0.01 for POP 4, mapping uint16 to millimeters).
            rgb_k: (3, 3) Intrinsic matrix of RGB camera (from Prgb.bin).
            rgb_distortion: (5,) Distortion coefficients [k1, k2, p1, p2, k3] (from Distort.bin).
            r_ir_to_rgb: (3, 3) Rotation matrix from Left IR to RGB camera (from LC_RT.bin).
            t_ir_to_rgb: (3,) Translation vector in mm from Left IR to RGB camera (from LC_RT.bin).
        """
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.depth_scale = float(depth_scale)

        self.rgb_k = rgb_k
        self.rgb_dist = rgb_distortion
        self.r_ir_to_rgb = r_ir_to_rgb
        self.t_ir_to_rgb = t_ir_to_rgb

    @classmethod
    def from_camparam(cls, camparam: dict) -> "PointCloudGenerator":
        """Instantiate PointCloudGenerator from parsed camparam.yaml."""
        p1 = camparam.get("P1")
        if p1 is not None:
            # P1 is 3x4 projection matrix: [fx 0 cx Tx; 0 fy cy 0; 0 0 1 0]
            data = p1.get("data", [])
            fx = data[0]
            cx = data[2]
            fy = data[5]
            cy = data[6]
        else:
            m1 = camparam.get("M1", {}).get("data", [])
            fx = m1[0]
            fy = m1[4]
            cx = m1[2]
            cy = m1[5]

        return cls(fx=fx, fy=fy, cx=cx, cy=cy, depth_scale=0.01)

    def depth_to_points(
        self, depth_u16: np.ndarray, min_depth_mm: float = 100.0, max_depth_mm: float = 500.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Unproject a 2D depth map into a 3D point cloud.

        Args:
            depth_u16: (H, W) uint16 array of raw depth values.
            min_depth_mm: Minimum distance cutoff in mm (default 100 mm).
            max_depth_mm: Maximum distance cutoff in mm (default 500 mm).

        Returns:
            points_3d: (N, 3) float32 array of [X, Y, Z] coordinates in millimeters.
            valid_mask: (H, W) boolean mask indicating valid points.
        """
        h, w = depth_u16.shape
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

        z = depth_u16.astype(np.float32) * self.depth_scale
        valid_mask = (z >= min_depth_mm) & (z <= max_depth_mm)

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        points_3d = np.stack((x[valid_mask], y[valid_mask], z[valid_mask]), axis=-1)
        return points_3d, valid_mask

    @staticmethod
    def save_ply(filename: str, points: np.ndarray, colors: Optional[np.ndarray] = None) -> None:
        """
        Save 3D point cloud to ASCII PLY format.

        Args:
            filename: Destination .ply path.
            points: (N, 3) array of float XYZ coordinates.
            colors: Optional (N, 3) array of uint8 RGB values (0..255).
        """
        n = len(points)
        has_colors = colors is not None and len(colors) == n

        with open(filename, "w", encoding="ascii") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {n}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            if has_colors:
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")
            f.write("end_header\n")

            if has_colors:
                for pt, clr in zip(points, colors):
                    f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f} {int(clr[0])} {int(clr[1])} {int(clr[2])}\n")
            else:
                for pt in points:
                    f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")
