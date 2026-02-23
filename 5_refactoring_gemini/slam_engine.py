import numpy as np
from scipy.spatial import KDTree
import math
from typing import List, Tuple, Optional

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
        self.reference_points = np.empty((0, 2))

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

        if len(self.reference_points) == 0:
            self.reference_points = global_points
        else:
            self.reference_points = np.vstack((self.reference_points, global_points))
            if len(self.reference_points) > 10000:
                # Deterministic FIFO downsampling (keep newest 10000)
                self.reference_points = self.reference_points[-10000:]

        indices = (global_points / self.resolution + self.origin).astype(int)

        mask = (indices[:, 0] >= 0) & (indices[:, 0] < self.map_size) & \
               (indices[:, 1] >= 0) & (indices[:, 1] < self.map_size)

        valid_indices = indices[mask]

        rows = valid_indices[:, 1]
        cols = valid_indices[:, 0]

        np.add.at(self.grid, (rows, cols), 1.0)
