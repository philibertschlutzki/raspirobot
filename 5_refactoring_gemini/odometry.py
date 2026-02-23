import math
import time
from typing import Tuple

try:
    from .data_models import RobotPose
except ImportError:
    from data_models import RobotPose

class OdometryEngine:
    def __init__(self, wheel_base: float = 0.20, max_speed: float = 0.5):
        """
        :param wheel_base: Distance between wheels in meters.
        :param max_speed: Max linear speed in m/s at full throttle (1.0).
        """
        self.wheel_base = wheel_base
        self.max_speed = max_speed

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = time.time()

    def set_pose(self, pose: RobotPose):
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta
        self.last_update_time = pose.timestamp

    def update(self, left_motor_val: float, right_motor_val: float, current_time: float = None) -> RobotPose:
        """
        Updates pose based on motor values (-1.0 to 1.0) and elapsed time.
        :param left_motor_val: Normalized speed (-1.0 to 1.0)
        :param right_motor_val: Normalized speed (-1.0 to 1.0)
        :param current_time: Optional timestamp, defaults to time.time()
        :return: New RobotPose
        """
        if current_time is None:
            current_time = time.time()

        dt = current_time - self.last_update_time
        if dt <= 0:
            return RobotPose(x=self.x, y=self.y, theta=self.theta, timestamp=self.last_update_time)

        # Calculate wheel velocities
        vl = left_motor_val * self.max_speed
        vr = right_motor_val * self.max_speed

        # Differential drive kinematics
        v = (vl + vr) / 2.0
        omega = (vr - vl) / self.wheel_base

        # Update pose
        # Simple Euler integration
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Normalize theta? Usually good practice but requirements don't specify.
        # Let's keep it simple or wrap to -pi, pi.
        # Wraparound is useful for SLAM.
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        self.last_update_time = current_time

        return RobotPose(x=self.x, y=self.y, theta=self.theta, timestamp=self.last_update_time)

    def get_pose(self) -> RobotPose:
        return RobotPose(x=self.x, y=self.y, theta=self.theta, timestamp=self.last_update_time)
