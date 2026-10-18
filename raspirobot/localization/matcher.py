"""Scan-Matching: sucht die Pose, bei der ein Scan am besten zur Karte passt."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .. import config
from .geometry import Pose, wrap_angle
from .grid_map import OccupancyMap

COARSE_MAX_POINTS = 150
REFINE_STAGES = 3
TRIM_FRACTION = 0.25     # schlechteste Punkte, die bei der Bewertung ignoriert werden
CHUNK = 4000             # Kandidatenposen pro Auswertungsblock (begrenzt den Speicher)


@dataclass
class MatchResult:
    pose: Pose
    score: float          # getrimmter Mittelwert des Feldes an den Scanpunkten, 0..1
    at_edge: bool         # bestes Ergebnis lag am Rand des Suchfensters
    ambiguous: bool       # gleich guter Treffer in >= 2 Schritten Entfernung


def trimmed_mean(values: np.ndarray, trim: float = TRIM_FRACTION) -> float:
    if values.size == 0:
        return 0.0
    keep = max(1, int(round(values.size * (1.0 - trim))))
    return float(np.sort(values)[-keep:].mean())


class ScanMatcher:
    def __init__(self, grid: OccupancyMap):
        self.grid = grid

    def score(self, points: np.ndarray, pose: Pose) -> float:
        """Getrimmte Bewertung eines Scans (Roboter-Rahmen) an einer Pose."""
        if points.shape[0] == 0:
            return 0.0
        world = _transform(points, np.array([pose]))[0]
        return trimmed_mean(self.grid.sample(world))

    def _score_many(self, points: np.ndarray, poses: np.ndarray) -> np.ndarray:
        out = np.empty(poses.shape[0])
        points = points.astype(np.float32)
        poses = poses.astype(np.float32)
        for start in range(0, poses.shape[0], CHUNK):
            chunk = poses[start:start + CHUNK]
            out[start:start + CHUNK] = self.grid.sample(_transform(points, chunk)).mean(axis=1)
        return out

    def _search(
        self,
        points: np.ndarray,
        center: Pose,
        prior: Pose,
        half_xy: float,
        half_theta: float,
        step_xy: float,
        step_theta: float,
    ) -> Tuple[Pose, float, bool, bool]:
        """Rastersuche um ``center``. Rückgabe: Pose, Rohscore, am Rand, mehrdeutig."""
        nx = int(round(half_xy / step_xy))
        nt = int(round(half_theta / step_theta))
        ox = np.arange(-nx, nx + 1) * step_xy
        ot = np.arange(-nt, nt + 1) * step_theta
        gx, gy, gt = np.meshgrid(ox, ox, ot, indexing="ij")
        gx, gy, gt = gx.ravel(), gy.ravel(), gt.ravel()
        poses = np.column_stack((center[0] + gx, center[1] + gy, center[2] + gt))

        raw = self._score_many(points, poses)
        penalty = (
            config.SLAM_PRIOR_PENALTY_PER_M * np.hypot(poses[:, 0] - prior[0], poses[:, 1] - prior[1])
            + config.SLAM_PRIOR_PENALTY_PER_RAD * np.abs(wrap_angle(poses[:, 2] - prior[2]))
        )
        best = int(np.argmax(raw - penalty))

        at_edge = (
            (nx > 0 and (abs(gx[best]) >= half_xy - 1e-9 or abs(gy[best]) >= half_xy - 1e-9))
            or (nt > 0 and abs(gt[best]) >= half_theta - 1e-9)
        )
        far = np.hypot(gx - gx[best], gy - gy[best]) >= 2 * step_xy - 1e-9
        ambiguous = bool(far.any() and raw[far].max() >= raw[best] - config.SLAM_AMBIGUITY_MARGIN)
        return (float(poses[best, 0]), float(poses[best, 1]), float(wrap_angle(poses[best, 2]))), float(raw[best]), at_edge, ambiguous

    def match(
        self,
        points: np.ndarray,
        prediction: Pose,
        half_xy: float,
        half_theta: float,
        step_xy: Optional[float] = None,
        step_theta: Optional[float] = None,
    ) -> MatchResult:
        """Grobe Suche im Fenster, danach drei Verfeinerungen mit halbem Schritt."""
        step_xy = step_xy or config.SLAM_COARSE_STEP_M
        if step_theta is None:
            reach = float(np.percentile(np.hypot(points[:, 0], points[:, 1]), 75)) if points.shape[0] else 1.0
            step_theta = float(np.clip(step_xy / max(reach, 1.0), 0.005, 0.05))

        stride = max(1, points.shape[0] // COARSE_MAX_POINTS)
        coarse_pts = points[::stride]
        pose, _raw, at_edge, ambiguous = self._search(
            coarse_pts, prediction, prediction, half_xy, half_theta, step_xy, step_theta
        )

        for _ in range(REFINE_STAGES):
            pose, _raw, _e, _a = self._search(
                points, pose, prediction, step_xy, step_theta, step_xy / 2, step_theta / 2
            )
            step_xy, step_theta = step_xy / 2, step_theta / 2

        return MatchResult(pose=pose, score=self.score(points, pose), at_edge=at_edge, ambiguous=ambiguous)


def _transform(points: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """Punkte (N, 2) an K Posen (K, 3) -> (K, N, 2)."""
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    x = c * points[None, :, 0] - s * points[None, :, 1] + poses[:, 0:1]
    y = s * points[None, :, 0] + c * points[None, :, 1] + poses[:, 1:2]
    return np.stack((x, y), axis=-1)
