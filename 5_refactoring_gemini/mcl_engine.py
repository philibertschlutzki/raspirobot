import numpy as np
from scipy.ndimage import distance_transform_edt
from pydantic import BaseModel, ConfigDict
import math
from typing import List, Tuple, Optional

try:
    from .data_models import RobotPose
    from .map_loader import load_and_validate_map
except ImportError:
    from data_models import RobotPose
    from map_loader import load_and_validate_map

class LocalizationResult(BaseModel):
    pose: RobotPose
    confidence: float
    unexpected_obstacle_detected: bool
    model_config = ConfigDict(frozen=True)

class MCLEngine:
    def __init__(self, map_path: str, initial_pose: RobotPose, num_particles: int = 3000):
        self.map_grid = load_and_validate_map(map_path)
        self.resolution = 0.05
        self.origin = np.array([200, 200]) # Pixels
        self.map_size = 400

        # Precompute Likelihood Field
        # Obstacles (> 0.5) are 0, Free are 1
        binary_map = (self.map_grid > 0.5)
        # edt computes distance to nearest zero. We want distance to nearest obstacle.
        # So we want obstacles to be 0, and free space to be 1.
        input_map = np.ones_like(self.map_grid)
        input_map[binary_map] = 0

        self.dist_map_pixels = distance_transform_edt(input_map)
        self.dist_map_meters = self.dist_map_pixels * self.resolution

        # Particles
        self.num_particles = num_particles
        self.min_particles = 500
        self.particles = np.zeros((self.num_particles, 3)) # x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Initialize particles around initial pose
        self._initialize_particles(initial_pose)

        # Parameters
        self.odom_noise = [0.05, 0.05, 0.05] # x, y, theta noise std dev
        self.sensor_sigma = 0.2 # meters (likelihood sigma)
        self.resample_threshold = 0.5 # N_eff threshold factor

        # Dynamic Obstacle
        self.dynamic_obstacle_dist_threshold = 0.5 # If map distance > 0.5m at hit point, it's dynamic

    def _initialize_particles(self, pose: RobotPose, position_std=0.2, theta_std=0.1):
        self.particles[:, 0] = np.random.normal(pose.x, position_std, self.num_particles)
        self.particles[:, 1] = np.random.normal(pose.y, position_std, self.num_particles)
        self.particles[:, 2] = np.random.normal(pose.theta, theta_std, self.num_particles)
        self.weights.fill(1.0 / self.num_particles)

    def predict(self, odom_delta: Tuple[float, float, float]):
        """
        Updates particles based on local odometry delta (dx_local, dy_local, dtheta).
        Adds Gaussian noise to the control input, then applies to particles based on their orientation.
        """
        dx_l, dy_l, dtheta = odom_delta

        # Add noise to local control input
        n_x = np.random.normal(0, self.odom_noise[0], self.num_particles)
        n_y = np.random.normal(0, self.odom_noise[1], self.num_particles)
        n_theta = np.random.normal(0, self.odom_noise[2], self.num_particles)

        dx_noisy = dx_l + n_x
        dy_noisy = dy_l + n_y
        dtheta_noisy = dtheta + n_theta

        # Rotate to global frame for each particle
        p_theta = self.particles[:, 2]
        c = np.cos(p_theta)
        s = np.sin(p_theta)

        # Global displacement: x_g = x_l * c - y_l * s, y_g = x_l * s + y_l * c
        dx_g = dx_noisy * c - dy_noisy * s
        dy_g = dx_noisy * s + dy_noisy * c

        self.particles[:, 0] += dx_g
        self.particles[:, 1] += dy_g
        self.particles[:, 2] += dtheta_noisy

        # Normalize angles
        self.particles[:, 2] = (self.particles[:, 2] + np.pi) % (2 * np.pi) - np.pi

    def update(self, scan_data: List[Tuple[float, float]], current_odom_pose: RobotPose) -> LocalizationResult:
        """
        Updates weights based on Lidar scan.
        scan_data: list of (angle, distance)
        """
        if not scan_data:
            return self._get_result()

        # 1. Downsample Scan (~72 rays)
        # Scan usually 360 or more.
        step = max(1, len(scan_data) // 72)
        downsampled_scan = scan_data[::step]

        # Arrays for broadcasting
        # scan_angles: (K,)
        # scan_dists: (K,)
        downsampled_arr = np.array(downsampled_scan, dtype=float)
        scan_angles = np.radians(downsampled_arr[:, 0])
        scan_dists = downsampled_arr[:, 1] / 1000.0 # mm to m

        # Filter valid rays
        valid_mask = (scan_dists > 0.05) & (scan_dists < 10.0)
        scan_angles = scan_angles[valid_mask]
        scan_dists = scan_dists[valid_mask]

        if len(scan_dists) == 0:
            return self._get_result()

        # 2. Vectorized Likelihood Calculation
        num_particles = len(self.particles)
        num_rays = len(scan_dists)

        # Particle poses: (N, 1)
        p_x = self.particles[:, 0].reshape(-1, 1)
        p_y = self.particles[:, 1].reshape(-1, 1)
        p_theta = self.particles[:, 2].reshape(-1, 1)

        # Ray angles in global frame: theta + scan_angle
        # (N, 1) + (1, K) -> (N, K)
        global_angles = p_theta + scan_angles.reshape(1, -1)

        # Endpoints
        # (N, K)
        end_x = p_x + scan_dists.reshape(1, -1) * np.cos(global_angles)
        end_y = p_y + scan_dists.reshape(1, -1) * np.sin(global_angles)

        # Convert to Grid Coordinates
        # (N, K)
        grid_col = ((end_x / self.resolution) + self.origin[0]).astype(int)
        grid_row = ((end_y / self.resolution) + self.origin[1]).astype(int)

        # Clip to map bounds
        np.clip(grid_col, 0, self.map_size - 1, out=grid_col)
        np.clip(grid_row, 0, self.map_size - 1, out=grid_row)

        # Lookup Distances
        # (N, K)
        # Note: map is accessed as [row, col] usually (y, x)
        distances = self.dist_map_meters[grid_row, grid_col]

        # Calculate Likelihoods
        # P(z|x) ~ exp(-d^2 / 2sigma^2)
        # We can sum log-likelihoods or product of probs.
        # Log-likelihood is numerically more stable, but for weights we need exp.
        # likelihood = exp( - sum(d^2) / (2*sigma^2) ) ?
        # No, product of independent measurements:
        # P(Z|X) = prod( P(z_k | X) )
        # log P = sum ( log P )
        # log P ~ sum ( -d^2 / 2s^2 )

        # Robustness: "Cutoff tolerieren"
        # Truncate distance to avoid killing particles due to dynamic obstacles
        max_dist = 0.5 # 50cm cutoff
        distances = np.minimum(distances, max_dist)

        # Sum of squared distances for each particle
        # (N,)
        sum_sq_dist = np.sum(distances**2, axis=1)

        # Weights
        log_weights = -sum_sq_dist / (2 * self.sensor_sigma**2)

        # Shift to avoid underflow
        log_weights -= np.max(log_weights)
        weights = np.exp(log_weights)

        # Update weights
        self.weights *= weights
        self.weights += 1.e-300 # Avoid zero
        self.weights /= np.sum(self.weights)

        # 3. Dynamic Obstacle Detection
        # Check if the BEST particle sees something unexpected
        best_idx = np.argmax(self.weights)
        # Look at distances for best particle (before cutoff)
        best_distances = self.dist_map_meters[grid_row[best_idx], grid_col[best_idx]]

        # Count rays that hit "free space" (large distance in map)
        # If map distance is large, it means we are far from an obstacle.
        # But we hit something (the ray end).
        # So there is an object where map says free.
        unexpected_count = np.sum(best_distances > self.dynamic_obstacle_dist_threshold)
        unexpected_ratio = unexpected_count / num_rays

        unexpected_obstacle = unexpected_ratio > 0.1 # 10% of rays hit unexpected things

        # 4. Adaptive Resampling
        n_eff = 1.0 / np.sum(self.weights**2)
        if n_eff < self.num_particles * self.resample_threshold:
             self._resample()

        return self._get_result(unexpected_obstacle)

    def _resample(self):
        indices = np.random.choice(self.num_particles, size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

        # Reduce particle count if converged?
        # "Start mit 3000, adaptives Resampling auf 500 bei Konvergenz."
        # Simple heuristic: if weights are concentrated, reduce count.
        # Actually usually we just set N based on N_eff or fixed schedule.
        # Let's just clamp to min_particles if we want to reduce.
        # But resampling step usually maintains size.
        # To reduce size, we just sample fewer particles.

        # Let's implement simple reduction logic
        # If we are resampling, it means we had low N_eff.
        # If we want to reduce to 500 eventually...
        # Maybe reduce by 10% each resample until 500?
        if self.num_particles > self.min_particles:
             new_count = max(self.min_particles, int(self.num_particles * 0.9))
             # We need to sample 'new_count' particles
             indices = np.random.choice(self.num_particles, size=new_count, p=self.weights)
             self.particles = self.particles[indices] # Now shape is (new_count, 3)
             self.num_particles = new_count
             self.weights = np.ones(self.num_particles) / self.num_particles

    def _get_result(self, unexpected=False) -> LocalizationResult:
        # Mean or Best?
        # Weighted mean
        mean_x = np.sum(self.particles[:, 0] * self.weights)
        mean_y = np.sum(self.particles[:, 1] * self.weights)

        # Angular mean (careful with wrapping)
        sin_sum = np.sum(np.sin(self.particles[:, 2]) * self.weights)
        cos_sum = np.sum(np.cos(self.particles[:, 2]) * self.weights)
        mean_theta = np.arctan2(sin_sum, cos_sum)

        confidence = np.max(self.weights) # Simple proxy

        pose = RobotPose(x=mean_x, y=mean_y, theta=mean_theta, timestamp=0.0)

        return LocalizationResult(
            pose=pose,
            confidence=float(confidence),
            unexpected_obstacle_detected=unexpected
        )
