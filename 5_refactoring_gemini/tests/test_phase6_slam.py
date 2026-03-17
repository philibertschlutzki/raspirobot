import numpy as np
import pytest
import math
import sys
import os
import json

# Ensure correct import path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from slam_engine import SLAMEngine
from data_models import RobotPose

def test_spatial_downsampling():
    """
    TC 6.1: Verify spatial downsampling (Voxel Grid Filter) replaces random thinning.
    Points falling into the same voxel should be reduced.
    """
    slam = SLAMEngine(resolution_m=0.05)

    # Generate many points within the same 5cm voxel (e.g., around 1.0, 1.0)
    # Voxel at 1.0/0.05 = 20.
    # Points: (1.01, 1.01), (1.02, 1.02), ...
    points = []
    for i in range(50):
        points.append((1.0 + i*0.0001, 1.0 + i*0.0001)) # All within 0.005m spread

    # Create a dummy scan format: List[Tuple[float, float]] -> (angle, dist_mm)
    # We test via the public API `update_map` by constructing valid scan_data
    # (reverse engineering scan_to_xy) to ensure realistic system behavior.
    # Let's create a scenario where robot is at 0,0 and points are at specific angles/distances.

    scan_data = []
    for p in points:
        x, y = p
        dist_m = math.sqrt(x*x + y*y)
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)
        scan_data.append((angle_deg, dist_m * 1000.0))

    pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)

    # First update
    slam.update_map(pose, scan_data)

    # All these points fall in the same voxel (approx).
    # The resolution is 0.05.
    # 1.0 / 0.05 = 20.
    # 1.005 / 0.05 = 20.1 -> 20.
    # So they should all map to the same voxel.
    # The number of reference points should be 1 (or very few if boundary crossing).

    assert len(slam.reference_points) < 5, f"Expected few points, got {len(slam.reference_points)}"

    # Determinism check: Run again with same data
    initial_len = len(slam.reference_points)
    slam.update_map(pose, scan_data)
    assert len(slam.reference_points) == initial_len, "Determinism failed: point count changed"

def test_raycasting_occupancy():
    """
    Verify Raycasting: Cells between robot and hit are free (low prob), hit is occupied (high prob).
    """
    slam = SLAMEngine(map_size_pixels=100, resolution_m=0.1, origin_pixels=(50, 50))
    # Origin at 50,50.
    # Robot at 0,0 -> pixel 50,50.
    # Hit at x=2.0 -> pixel 50 + 2.0/0.1 = 70. (70, 50).

    pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)
    scan_data = [(0.0, 2000.0)] # 0 degrees, 2 meters

    slam.update_map(pose, scan_data)

    # Check Hit Point (approx 70, 50)
    # Note: scan_to_xy -> x=2.0, y=0.0.
    # Global: x=2.0, y=0.0.
    # Grid: 2.0/0.1 + 50 = 70.

    hit_val = slam.grid[50, 70] # rows=y, cols=x. y=0->50, x=2->70.
    # Wait, indices in update_map: rows = valid_indices[:, 1], cols = valid_indices[:, 0]
    # x corresponds to col, y to row.

    assert hit_val > 0, "Hit point should have positive occupancy value"

    # Check Free Space (e.g., x=1.0 -> pixel 60, 50)
    free_val = slam.grid[50, 60]
    assert free_val < 0, "Space between robot and hit should have negative occupancy value (free)"

    # Check Unknown Space (e.g., x=3.0 -> pixel 80, 50)
    unknown_val = slam.grid[50, 80]
    assert unknown_val == 0, "Space beyond hit should be unknown (0)"

def test_save_map(tmp_path):
    """
    Verify map export to .npy and .json.
    """
    # Use tmp_path fixture from pytest
    base_filename = str(tmp_path / "test_map")

    slam = SLAMEngine(map_size_pixels=50, resolution_m=0.1)
    # Modify grid slightly
    slam.grid[25, 25] = 10.0

    # Execute save
    if hasattr(slam, 'save_map'):
        slam.save_map(base_filename)

        # Check files
        npy_path = base_filename + ".npy"
        json_path = base_filename + ".json"

        assert os.path.exists(npy_path), "NPY file not created"
        assert os.path.exists(json_path), "JSON file not created"

        # Verify NPY content
        loaded_grid = np.load(npy_path)
        assert loaded_grid.shape == (50, 50)
        assert loaded_grid[25, 25] == 10.0

        # Verify JSON content
        with open(json_path, 'r') as f:
            meta = json.load(f)

        assert meta["map_size_pixels"] == 50
        assert meta["resolution_m"] == 0.1
        assert "origin_pixels" in meta
    else:
        pytest.fail("save_map method not implemented yet")
