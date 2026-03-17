import numpy as np
from scipy.spatial import KDTree
import math
from typing import List, Tuple, Optional
import json
import os

# Log-Odds constants
LO_OCC = 0.85
LO_FREE = -0.4
LO_MAX = 5.0
LO_MIN = -5.0

try:
    from .data_models import RobotPose
except ImportError:
    from data_models import RobotPose

class SLAMEngine:
    def __init__(self, map_size_pixels=400, resolution_m=0.05, origin_pixels=(200, 200), max_correspondence_dist=1.0):
        self.map_size = map_size_pixels
        self.resolution = resolution_m
        self.origin = np.array(origin_pixels)
        self.max_correspondence_dist = max_correspondence_dist

        self.grid = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)

        # Pre-allocated buffer for reference points to avoid np.vstack reallocations
        self._max_ref_points = 10000
        self._ref_buffer = np.empty((self._max_ref_points * 2, 2), dtype=np.float32)
        self._num_ref_points = 0

    @property
    def reference_points(self) -> np.ndarray:
        return self._ref_buffer[:self._num_ref_points]

    @reference_points.setter
    def reference_points(self, val: np.ndarray):
        n = len(val)
        if n > self._max_ref_points:
            val = val[-self._max_ref_points:]
            n = self._max_ref_points
        self._ref_buffer[:n] = val
        self._num_ref_points = n

    def _bresenham_line(self, start: np.ndarray, end: np.ndarray) -> np.ndarray:
        """
        Generates grid coordinates for a line from start (col, row) to end (col, row).
        Returns a (N, 2) numpy array of integer coordinates [col, row].
        Uses a standard integer-based Bresenham line algorithm to avoid float arrays and allocations.
        """
        x0, y0 = int(start[0]), int(start[1])
        x1, y1 = int(end[0]), int(end[1])

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x, y = x0, y0

        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1

        points = []

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append([x, y])
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            points.append([x, y])
        else:
            err = dy / 2.0
            while y != y1:
                points.append([x, y])
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            points.append([x, y])

        return np.array(points, dtype=int)

    def scan_to_xy(self, scan_data: List[Tuple[float, float]]) -> np.ndarray:
        """Converts scan (angle_deg, dist_mm) to local XY (meters)."""
        points = []
        for angle, dist_mm in scan_data:
            if dist_mm > 0:
                dist_m = dist_mm / 1000.0
                angle_rad = math.radians(angle)
                x = dist_m * math.cos(angle_rad)
                y = dist_m * math.sin(angle_rad)
                points.append([x, y])
        if not points:
            return np.empty((0, 2))
        return np.array(points)

    def transform_points(self, points: np.ndarray, pose: RobotPose) -> np.ndarray:
        """Transforms local points to global frame given pose."""
        if len(points) == 0:
            return points

        c = math.cos(pose.theta)
        s = math.sin(pose.theta)
        rotation = np.array([[c, -s], [s, c]])
        translation = np.array([pose.x, pose.y])

        return points @ rotation.T + translation

    def icp(self, current_points: np.ndarray, initial_pose: RobotPose, max_iterations=20, tolerance=1e-4) -> RobotPose:
        """
        Performs ICP to align current_points to self.reference_points starting from initial_pose.
        Returns optimized pose.
        """
        # Relaxed checks for sparse scans
        if len(self.reference_points) < 5 or len(current_points) < 5:
            return initial_pose

        pose = initial_pose

        tree = KDTree(self.reference_points)

        for i in range(max_iterations):
            transformed = self.transform_points(current_points, pose)

            distances, indices = tree.query(transformed)

            # Dynamic threshold? Or fixed from init?
            valid = distances < self.max_correspondence_dist

            if np.sum(valid) < 5:
                # Not enough matches
                break

            source = transformed[valid]
            target = self.reference_points[indices[valid]]

            mean_s = np.mean(source, axis=0)
            mean_t = np.mean(target, axis=0)

            s_centered = source - mean_s
            t_centered = target - mean_t

            H = s_centered.T @ t_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            if np.linalg.det(R) < 0:
                 Vt[-1, :] *= -1
                 R = Vt.T @ U.T

            delta_theta = math.atan2(R[1, 0], R[0, 0])
            # delta_translation is translation to apply AFTER rotation around centroids
            # t = mean_t - R @ mean_s
            # Here we work with delta relative to current pose

            # Correct update of Robot Pose:
            c = math.cos(pose.theta)
            s = math.sin(pose.theta)
            R_old = np.array([[c, -s], [s, c]])
            t_old = np.array([pose.x, pose.y])

            # New orientation
            R_new = R @ R_old
            new_theta = math.atan2(R_new[1, 0], R_new[0, 0])

            # New position
            # The cloud 'source' (current global) moves to 'target'.
            # Transformation is T_delta(x) = R(x - mean_s) + mean_t
            # = R*x - R*mean_s + mean_t
            # = R*x + t_delta
            # where t_delta = mean_t - R*mean_s

            # So New Robot Position P_new should result in points:
            # P_new_points = R_new * local + t_new
            # And we want P_new_points = R * P_old_points + t_delta
            # = R * (R_old * local + t_old) + t_delta
            # = R * R_old * local + R * t_old + t_delta
            # So t_new = R * t_old + t_delta

            t_delta = mean_t - (source @ R.T).mean(axis=0) # Or mean_t - R @ mean_s
            # Note: s_centered = source - mean_s. R aligns s_centered to t_centered.
            # R * s_centered ~ t_centered
            # R * (source - mean_s) ~ target - mean_t
            # R * source - R * mean_s ~ target - mean_t
            # R * source + (mean_t - R * mean_s) ~ target

            # Wait, `t_delta` calculation using matrix mult:
            # mean_t (1x2) - mean_s (1x2) @ R.T (2x2)
            # = mean_t - (R @ mean_s.T).T
            # Correct.

            t_delta = mean_t - mean_s @ R.T

            t_new = t_old @ R.T + t_delta

            pose = RobotPose(x=t_new[0], y=t_new[1], theta=new_theta, timestamp=pose.timestamp)

            if abs(delta_theta) < tolerance and np.linalg.norm(t_delta) < tolerance:
                break

        return pose

    def update_map(self, pose: RobotPose, scan_data: List[Tuple[float, float]]):
        """Updates grid and reference points."""
        local_points = self.scan_to_xy(scan_data)
        if len(local_points) == 0:
            return

        global_points = self.transform_points(local_points, pose)

        # --- Update Reference Points (for ICP) with Voxel Grid Filter ---
        n_new = len(global_points)
        if n_new > 0:
            buffer_capacity = len(self._ref_buffer)

            # If adding new points exceeds capacity, compact the buffer
            if self._num_ref_points + n_new > buffer_capacity:
                keep = self._max_ref_points - n_new
                if keep > 0:
                    # Keep the newest `keep` points
                    self._ref_buffer[:keep] = self._ref_buffer[self._num_ref_points - keep:self._num_ref_points]
                    self._num_ref_points = keep
                else:
                    # New points exceed max_ref_points, only keep the newest max_ref_points of them
                    self._num_ref_points = 0
                    global_points = global_points[-self._max_ref_points:]
                    n_new = len(global_points)

            # Append new points to the buffer
            self._ref_buffer[self._num_ref_points:self._num_ref_points + n_new] = global_points
            self._num_ref_points += n_new


        # Spatial Downsampling: Voxel Grid Filter
        # Convert to integer grid coordinates based on resolution
        grid_coords = (self.reference_points / self.resolution).astype(int)

        # Keep unique voxels (deterministically)
        _, unique_indices = np.unique(grid_coords, axis=0, return_index=True)
        self.reference_points = self.reference_points[unique_indices]

        # --- Update Occupancy Grid with Raycasting (Bresenham + Log-Odds) ---

        # Robot position in grid
        start_col = int(pose.x / self.resolution + self.origin[0])
        start_row = int(pose.y / self.resolution + self.origin[1])
        start_pixel = np.array([start_col, start_row])

        # End positions in grid
        end_pixels = (global_points / self.resolution + self.origin).astype(int)

        # Process each ray
        for end_pixel in end_pixels:
            # Get all points on the line
            line_points = self._bresenham_line(start_pixel, end_pixel)

            # Filter points within map bounds
            mask = (line_points[:, 0] >= 0) & (line_points[:, 0] < self.map_size) & \
                   (line_points[:, 1] >= 0) & (line_points[:, 1] < self.map_size)
            valid_points = line_points[mask]

            if len(valid_points) == 0:
                continue

            cols = valid_points[:, 0]
            rows = valid_points[:, 1]

            # Mark all cells on the ray as free initially
            self.grid[rows, cols] += LO_FREE

            # Mark the specific hit point as occupied
            # Only if the last point in the ray corresponds to the target (hit)
            # (It might be clipped if outside map, but if inside, last point is hit)
            last_p = valid_points[-1]
            if np.array_equal(last_p, end_pixel):
                # Revert free update for the last cell and apply occupied update
                self.grid[rows[-1], cols[-1]] -= LO_FREE
                self.grid[rows[-1], cols[-1]] += LO_OCC

        # Clamp values
        np.clip(self.grid, LO_MIN, LO_MAX, out=self.grid)

    def save_map(self, base_filename: str):
        """
        Exports the current grid to .npy and metadata to .json.
        """
        # Save Grid
        np.save(f"{base_filename}.npy", self.grid)

        # Save Metadata
        metadata = {
            "map_size_pixels": self.map_size,
            "resolution_m": self.resolution,
            "origin_pixels": self.origin.tolist()
        }
        with open(f"{base_filename}.json", 'w') as f:
            json.dump(metadata, f, indent=4)
