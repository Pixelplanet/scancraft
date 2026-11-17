import os
import tempfile
import numpy as np
import pytest
from client.reconstruction import PointCloudGenerator


def test_depth_to_points_calculation():
    # Setup camera with known intrinsics
    # fx = fy = 1000.0, cx = 320.0, cy = 256.0
    # Image size: 512 x 640
    gen = PointCloudGenerator(fx=1000.0, fy=1000.0, cx=320.0, cy=256.0, depth_scale=0.01)

    # Synthetic depth image
    depth = np.zeros((512, 640), dtype=np.uint16)

    # Pixel at center (320, 256) with depth 20000 -> 20000 * 0.01 = 200.0 mm
    depth[256, 320] = 20000

    # Pixel at (420, 256): u - cx = 100 px.
    # X = (100 * 200.0) / 1000.0 = 20.0 mm
    depth[256, 420] = 20000

    # Pixel at (320, 356): v - cy = 100 px.
    # Y = (100 * 200.0) / 1000.0 = 20.0 mm
    depth[356, 320] = 20000

    # Invalid / out-of-range pixel (depth 0, and depth 100000)
    depth[0, 0] = 0
    depth[1, 1] = 65000  # 650 mm > max_depth_mm (500)

    points, mask = gen.depth_to_points(depth, min_depth_mm=100.0, max_depth_mm=500.0)

    assert len(points) == 3
    assert mask.sum() == 3

    # Check center point: (0, 0, 200)
    np.testing.assert_allclose(points[0], [0.0, 0.0, 200.0], atol=1e-3)

    # Check shifted X point: (20, 0, 200)
    np.testing.assert_allclose(points[1], [20.0, 0.0, 200.0], atol=1e-3)

    # Check shifted Y point: (0, 20, 200)
    np.testing.assert_allclose(points[2], [0.0, 20.0, 200.0], atol=1e-3)


def test_save_ply_ascii():
    gen = PointCloudGenerator(fx=1000.0, fy=1000.0, cx=320.0, cy=256.0)
    points = np.array([[0.0, 0.0, 200.0], [10.0, 20.0, 250.0]], dtype=np.float32)
    colors = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)

    with tempfile.TemporaryDirectory() as tmpdir:
        ply_path = os.path.join(tmpdir, "test.ply")
        gen.save_ply(ply_path, points, colors)

        assert os.path.exists(ply_path)
        with open(ply_path, "r", encoding="ascii") as f:
            lines = f.readlines()

        assert lines[0].strip() == "ply"
        assert "element vertex 2\n" in lines
        assert "property uchar red\n" in lines
        # Check last vertex line
        assert "0.0000 0.0000 200.0000 255 0 0\n" in lines
