"""Kollisionsbewertung aus Ultraschall und LIDAR."""

from dataclasses import dataclass
from typing import Iterable, Tuple

from . import config


@dataclass
class CollisionStatus:
    ultrasonic_left: float        # cm
    ultrasonic_right: float       # cm
    lidar_min_distance: float     # m; 100.0, wenn im Sektor kein Punkt liegt
    lidar_obstacles_front: int    # Punkte im Sektor näher als LIDAR_WARNING_DISTANCE_M
    collision_detected: bool
    collision_critical: bool


def analyze_collision(
    distance_left_cm: float,
    distance_right_cm: float,
    scan: Iterable[Tuple[float, float, float]],
) -> CollisionStatus:
    """Bewertet die aktuelle Lage.

    LIDAR: Es werden nur Punkte mit ``LIDAR_FRONT_SECTOR_MIN_DEG <= angle <=
    LIDAR_FRONT_SECTOR_MAX_DEG`` betrachtet, Winkel roh wie vom Sensor (ohne
    LIDAR_ANGLE_OFFSET). Weil der Sensor 0-360 Grad liefert, deckt das nur
    0-45 Grad ab.

    - ``collision_detected``: ein Ultraschallwert < COLLISION_DISTANCE_CM oder
      LIDAR-Minimum < LIDAR_COLLISION_DISTANCE_M.
    - ``collision_critical``: dieselben Bedingungen mit halber Schwelle.
    """
    lidar_min_dist = float("inf")
    obstacles_front = 0

    for _quality, angle_deg, distance_mm in scan:
        distance_m = distance_mm / 1000.0
        if config.LIDAR_FRONT_SECTOR_MIN_DEG <= angle_deg <= config.LIDAR_FRONT_SECTOR_MAX_DEG:
            if distance_m < lidar_min_dist:
                lidar_min_dist = distance_m
            if distance_m < config.LIDAR_WARNING_DISTANCE_M:
                obstacles_front += 1

    if lidar_min_dist == float("inf"):
        lidar_min_dist = 100.0

    collision_detected = (
        distance_left_cm < config.COLLISION_DISTANCE_CM
        or distance_right_cm < config.COLLISION_DISTANCE_CM
        or lidar_min_dist < config.LIDAR_COLLISION_DISTANCE_M
    )
    collision_critical = (
        distance_left_cm < config.COLLISION_DISTANCE_CM * 0.5
        or distance_right_cm < config.COLLISION_DISTANCE_CM * 0.5
        or lidar_min_dist < config.LIDAR_COLLISION_DISTANCE_M * 0.5
    )

    return CollisionStatus(
        ultrasonic_left=distance_left_cm,
        ultrasonic_right=distance_right_cm,
        lidar_min_distance=lidar_min_dist,
        lidar_obstacles_front=obstacles_front,
        collision_detected=collision_detected,
        collision_critical=collision_critical,
    )
