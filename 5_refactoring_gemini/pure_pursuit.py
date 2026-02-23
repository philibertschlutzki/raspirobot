import math
import numpy as np
from typing import List, Tuple

try:
    from .data_models import RobotPose
    from .config import WHEEL_BASE_M
except ImportError:
    from data_models import RobotPose
    from config import WHEEL_BASE_M

class PurePursuitController:
    def __init__(self, lookahead_dist: float = 0.4, max_speed: float = 0.5):
        self.lookahead_dist = lookahead_dist
        self.max_speed = max_speed
        self.wheel_base = WHEEL_BASE_M

    def get_command(self, pose: RobotPose, path: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calculates motor commands (normalized -1.0 to 1.0) to follow the path.
        Returns (left_throttle, right_throttle).
        """
        if not path:
            return 0.0, 0.0

        # 1. Find target point
        target_point = self._get_target_point(pose, path)

        # 2. Transform to vehicle frame
        dx = target_point[0] - pose.x
        dy = target_point[1] - pose.y

        # Rotate into robot frame (X forward, Y left)
        # sin/cos of negative robot theta
        c = math.cos(pose.theta)
        s = math.sin(pose.theta)

        # x_local = dot(dp, forward_vec)
        # y_local = dot(dp, left_vec)
        # forward_vec = (c, s)
        # left_vec = (-s, c)

        x_local = dx * c + dy * s
        y_local = -dx * s + dy * c

        # 3. Calculate curvature
        # L^2 = x^2 + y^2
        lookahead_sq = x_local**2 + y_local**2

        if lookahead_sq < 0.001:
            return 0.0, 0.0

        # Gamma = 2 * y / L^2
        curvature = 2.0 * y_local / lookahead_sq

        # 4. Calculate Wheel Velocities
        # Target linear speed
        # Slow down if sharp turn? Or fixed speed?
        # Requirement says "Pure Pursuit Controller ... fährt die geglättete Kurve ab".
        # Let's use constant speed for now, or slow down based on curvature.
        target_v = self.max_speed

        # v = (vl + vr)/2
        # omega = (vr - vl)/W
        # v = w * R = w / gamma
        # omega = v * gamma

        omega = target_v * curvature

        # vr = v + w*W/2
        # vl = v - w*W/2

        vr = target_v + omega * self.wheel_base / 2.0
        vl = target_v - omega * self.wheel_base / 2.0

        # Normalize to max_speed (keep ratio)
        max_v = max(abs(vr), abs(vl))
        if max_v > self.max_speed:
            scale = self.max_speed / max_v
            vr *= scale
            vl *= scale

        # Convert to throttle (-1.0 to 1.0)
        # Assuming max_speed corresponds to throttle 1.0?
        # Or MotorController takes -1 to 1 and maps to duty cycle.
        # OdometryEngine uses max_speed=0.5m/s at throttle 1.0.
        # So divide by max_speed_param (which should match OdometryEngine's max_speed).
        # We assume self.max_speed IS the physical max speed at throttle 1.0.

        throttle_l = vl / self.max_speed
        throttle_r = vr / self.max_speed

        # Clip just in case
        throttle_l = max(-1.0, min(1.0, throttle_l))
        throttle_r = max(-1.0, min(1.0, throttle_r))

        return throttle_l, throttle_r

    def _get_target_point(self, pose: RobotPose, path: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Finds the lookahead point on the path.
        """
        # Find closest point index
        min_dist_sq = float('inf')
        closest_idx = 0

        for i, (px, py) in enumerate(path):
            d2 = (px - pose.x)**2 + (py - pose.y)**2
            if d2 < min_dist_sq:
                min_dist_sq = d2
                closest_idx = i

        # Look forward for first point > lookahead_dist
        # Note: we search from closest_idx to end
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            d2 = (px - pose.x)**2 + (py - pose.y)**2
            if d2 > self.lookahead_dist**2:
                return (px, py)

        # If not found (end of path), return last point
        return path[-1]
