"""Prüft den LIDAR, ohne Motoren zu bewegen.

    <venv>/bin/python -m raspirobot.lidar_check [Anzahl Scans]

Zeigt Scanrate, Punktzahl und die nächste Distanz je Richtung im Roboter-Rahmen.
Zum Prüfen der Drehrichtung (``LIDAR_ANGLE_SIGN``, ``LIDAR_ANGLE_OFFSET``) die
Hand ca. 30 cm vor den Sensor halten: Steht sie links vom Roboter, muss
"links" die kleinste Distanz zeigen.
"""

import sys
import time

import numpy as np

from . import config
from .hardware.lidar import Lidar
from .localization.geometry import scan_to_points

SECTORS = {"vorn": 0.0, "links": 90.0, "hinten": 180.0, "rechts": -90.0}
HALF_WIDTH_DEG = 30.0


def sector_minimum(points: np.ndarray, center_deg: float) -> float:
    """Kleinste Distanz (m) zum Roboter-Mittelpunkt im Sektor, ``nan`` ohne Punkte."""
    if points.shape[0] == 0:
        return float("nan")
    angle = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    diff = (angle - center_deg + 180.0) % 360.0 - 180.0
    inside = np.abs(diff) <= HALF_WIDTH_DEG
    if not inside.any():
        return float("nan")
    return float(np.hypot(points[inside, 0], points[inside, 1]).min())


def main(scans_wanted: int = 20) -> int:
    lidar = Lidar()
    if not lidar.connect():
        return 1
    count = 0
    start = time.time()

    def on_scan(timestamp, scan, index):
        nonlocal count
        count += 1
        points = scan_to_points(scan)
        mins = "  ".join(f"{name}={sector_minimum(points, deg):5.2f}m" for name, deg in SECTORS.items())
        print(f"Scan {index:3d}: {len(scan):3d} Punkte ({points.shape[0]} gültig)  {mins}")

    try:
        lidar.run(lambda: count < scans_wanted, on_scan)
    finally:
        elapsed = max(time.time() - start, 1e-6)
        print(f"{count} Scans in {elapsed:.1f}s ({count / elapsed:.1f} Hz), "
              f"Port {config.LIDAR_PORT}, ANGLE_SIGN={config.LIDAR_ANGLE_SIGN}")
        lidar.shutdown()
    return 0 if count else 1


if __name__ == "__main__":
    sys.exit(main(int(sys.argv[1]) if len(sys.argv) > 1 else 20))
