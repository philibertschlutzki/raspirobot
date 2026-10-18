"""Pose-Schätzung aus den gesetzten PWM-Werten (Koppelnavigation ohne Encoder)."""

import math
import threading
import time
from dataclasses import dataclass

from . import config


@dataclass
class RobotPose:
    x: float      # m, ab Start bzw. letztem Reset
    y: float      # m
    theta: float  # rad, 0 = Startausrichtung, normalisiert auf [-pi, pi) nach Kurvenfahrt
    timestamp: float


def integrate_pose(x, y, theta, left_duty, right_duty, is_moving_forward, dt):
    """Ein Integrationsschritt eines Differentialantriebs.

    Geschwindigkeit je Rad = Duty/100 * ODOMETRY_MAX_SPEED_M_S, im
    Rückwärtsmodus negativ. Die Richtungspins (z. B. gegenläufige Räder beim
    Tank-Turn) gehen nicht ein, nur ``is_moving_forward``.
    """
    v_left = (left_duty / 100.0) * config.ODOMETRY_MAX_SPEED_M_S
    v_right = (right_duty / 100.0) * config.ODOMETRY_MAX_SPEED_M_S
    if not is_moving_forward:
        v_left = -v_left
        v_right = -v_right

    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / config.WHEEL_BASE_M

    if abs(omega) < 0.001:
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
    else:
        radius = v / omega
        x += radius * (math.sin(theta + omega * dt) - math.sin(theta))
        y += radius * (-math.cos(theta + omega * dt) + math.cos(theta))
        theta += omega * dt
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
    return x, y, theta


class PoseTracker:
    """Integriert die Pose mit MAX_POSE_UPDATE_HZ aus den zuletzt gesetzten Duty Cycles."""

    def __init__(self, is_moving_forward):
        """``is_moving_forward``: Funktion, die die aktuelle Fahrtrichtung liefert."""
        self._is_moving_forward = is_moving_forward
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._timestamp = 0.0
        self.left_duty = 0.0
        self.right_duty = 0.0

    def set_duty(self, left_duty: float, right_duty: float) -> None:
        self.left_duty = left_duty
        self.right_duty = right_duty

    def step(self, dt: float) -> None:
        with self._lock:
            self._x, self._y, self._theta = integrate_pose(
                self._x, self._y, self._theta, self.left_duty, self.right_duty, self._is_moving_forward(), dt
            )
            self._timestamp = time.time()

    def reset(self) -> None:
        with self._lock:
            self._x = 0.0
            self._y = 0.0
            self._theta = 0.0

    def get_pose(self) -> RobotPose:
        with self._lock:
            return RobotPose(self._x, self._y, self._theta, self._timestamp)

    def run(self, should_run) -> None:
        print(f"[THREAD] Pose Thread gestartet ({config.MAX_POSE_UPDATE_HZ} Hz)")
        sleep_time = 1.0 / config.MAX_POSE_UPDATE_HZ
        last_update = time.time()

        while should_run():
            loop_start = time.time()
            self.step(loop_start - last_update)
            last_update = loop_start

            sleep_duration = sleep_time - (time.time() - loop_start)
            if sleep_duration > 0:
                time.sleep(sleep_duration)

        print("[THREAD] Pose Thread beendet")
