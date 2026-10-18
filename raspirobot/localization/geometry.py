"""Posen (x, y, theta) und Umrechnung von LIDAR-Scans in den Roboter-Rahmen.

Koordinatensystem: x nach vorn, y nach links, theta gegen den Uhrzeigersinn
(wie in ``odometry.py``).
"""

import math
from typing import Iterable, Tuple

import numpy as np

from .. import config

Pose = Tuple[float, float, float]


def wrap_angle(angle):
    """Normalisiert auf [-pi, pi). Funktioniert für Skalare und Arrays."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compose(a: Pose, b: Pose) -> Pose:
    """Pose ``b`` (relativ zu ``a``) im Rahmen von ``a``."""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (
        a[0] + ca * b[0] - sa * b[1],
        a[1] + sa * b[0] + ca * b[1],
        float(wrap_angle(a[2] + b[2])),
    )


def inverse(a: Pose) -> Pose:
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (
        -(ca * a[0] + sa * a[1]),
        -(-sa * a[0] + ca * a[1]),
        float(wrap_angle(-a[2])),
    )


def relative(frm: Pose, to: Pose) -> Pose:
    """Bewegung von ``frm`` nach ``to``, ausgedrückt im Rahmen von ``frm``."""
    return compose(inverse(frm), to)


def scan_to_points(
    scan: Iterable[Tuple[float, float, float]],
    min_quality: float = config.LIDAR_MIN_QUALITY,
    min_range_m: float = config.LIDAR_MIN_RANGE_M,
    max_range_m: float = config.LIDAR_MAX_RANGE_M,
) -> np.ndarray:
    """Wandelt ``(quality, angle_deg, distance_mm)`` in (N, 2)-Punkte im Roboter-Rahmen.

    Punkte mit Distanz 0 (ungültig), zu geringer Qualität, außerhalb des
    Messbereichs oder mit nicht endlichen Werten fallen weg. Winkel werden mit
    ``LIDAR_ANGLE_SIGN`` in die Drehrichtung des Roboters gebracht und mit
    Einbauwinkel und -position des LIDAR in den Roboter-Rahmen gelegt.
    """
    arr = np.asarray(list(scan), dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] == 0 or arr.shape[1] != 3:
        return np.empty((0, 2))

    quality, angle_deg, dist_m = arr[:, 0], arr[:, 1], arr[:, 2] / 1000.0
    valid = (
        np.isfinite(quality) & np.isfinite(angle_deg) & np.isfinite(dist_m)
        & (quality >= min_quality) & (dist_m >= min_range_m) & (dist_m <= max_range_m)
    )
    angle = config.LIDAR_ANGLE_SIGN * np.radians(angle_deg[valid]) + math.radians(config.LIDAR_ANGLE_OFFSET)
    dist = dist_m[valid]
    return np.column_stack((
        dist * np.cos(angle) + config.LIDAR_OFFSET_X,
        dist * np.sin(angle) + config.LIDAR_OFFSET_Y,
    ))


def transform_points(points: np.ndarray, pose: Pose) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([pose[0], pose[1]])
