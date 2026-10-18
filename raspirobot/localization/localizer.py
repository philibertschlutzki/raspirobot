"""Positionsbestimmung: Odometrie als Vorhersage, Scan-Matching als Korrektur.

Ablauf je Scan:
    1. Vorhersage = letzte Pose + Odometrie-Bewegung seit dem letzten Scan.
    2. Scan gegen die Karte abgleichen (enges Fenster, bei Bedarf weites).
    3. Nur ein Match mit ausreichender Wertung, das nicht am Fensterrand liegt
       und nicht mehrdeutig ist (Flur), korrigiert die Pose und baut die Karte aus.
       Sonst bleibt die Odometrie-Vorhersage; der Status zeigt das an.
    4. Nach ``SLAM_LOST_AFTER`` Fehlmatches in Folge gilt der Roboter als
       verloren und sucht jeden Scan global (±``SLAM_RELOCALIZE_XY_M``, alle Winkel)
       nach seiner Position, mit strengerer Schwelle.

Die Karte beginnt beim ersten gültigen Scan an Pose (0, 0, 0). Wegen
der Odometrie ohne Encoder gilt: Je länger der Roboter ohne gute Matches fährt,
desto weniger ist der Pose zu trauen; ``status`` und ``score`` sagen, wie gut.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Tuple

from .. import config
from ..log import logger
from .geometry import Pose, compose, relative, scan_to_points, transform_points
from .grid_map import OccupancyMap
from .matcher import ScanMatcher

STATUS_INIT = "INIT"
STATUS_OK = "OK"
STATUS_DEGRADED = "DEGRADED"
STATUS_LOST = "LOST"


@dataclass(frozen=True)
class LocalizationResult:
    x: float
    y: float
    theta: float
    timestamp: float
    status: str = STATUS_INIT
    score: float = 0.0        # Wertung des letzten Matches (0..1)
    matched: bool = False     # Pose stammt aus dem letzten Scan-Match, nicht nur aus der Odometrie
    n_points: int = 0
    scans_processed: int = 0


class Localizer:
    def __init__(self, odom_getter: Callable[[], object], grid: Optional[OccupancyMap] = None):
        """``odom_getter`` liefert ein Objekt mit x, y, theta und optional epoch (siehe ``RobotPose``)."""
        self._odom_getter = odom_getter
        self.grid = grid or OccupancyMap()
        self.matcher = ScanMatcher(self.grid)

        self._pose: Pose = (0.0, 0.0, 0.0)
        self._last_odom: Optional[Pose] = None
        self._last_epoch: Optional[int] = None
        self._initialized = False
        self._status = STATUS_INIT
        self._fail_count = 0
        self._scans = 0

        self._result = LocalizationResult(0.0, 0.0, 0.0, time.time())
        self._result_lock = threading.Lock()
        self._cond = threading.Condition()
        self._pending: Optional[Tuple[float, list, object]] = None

    # ------------------------------------------------------------ Schnittstelle

    def get_pose(self) -> LocalizationResult:
        with self._result_lock:
            return self._result

    def offer_scan(self, timestamp: float, scan: Iterable, scan_index: int = 0) -> None:
        """Vom LIDAR-Thread aufgerufen: merkt sich nur den neuesten Scan samt Odometrie, rechnet nicht."""
        odom = self._odom_getter()
        with self._cond:
            self._pending = (timestamp, list(scan), odom)
            self._cond.notify()

    def run(self, should_run: Callable[[], bool]) -> None:
        """Worker: verarbeitet immer den neuesten Scan, ältere werden übersprungen."""
        print("[THREAD] Localization Thread gestartet")
        while should_run():
            with self._cond:
                if self._pending is None:
                    self._cond.wait(timeout=0.2)
                job, self._pending = self._pending, None
            if job is None:
                continue
            try:
                self.process(*job)
            except Exception as e:
                logger.error(f"Positionsbestimmung Fehler: {e}")
        print("[THREAD] Localization Thread beendet")

    def save_map(self, base_path: str):
        return self.grid.save(base_path)

    # ------------------------------------------------------------------ Kern

    def process(self, timestamp: float, scan: Iterable, odom) -> LocalizationResult:
        """Verarbeitet einen Scan synchron und veröffentlicht die neue Pose."""
        odom_pose: Pose = (float(odom.x), float(odom.y), float(odom.theta))
        epoch = getattr(odom, "epoch", 0)
        if self._last_odom is None or epoch != self._last_epoch:
            rel: Pose = (0.0, 0.0, 0.0)  # erster Aufruf oder Odometrie wurde zurückgesetzt
        else:
            rel = relative(self._last_odom, odom_pose)
        self._last_odom, self._last_epoch = odom_pose, epoch
        prediction = compose(self._pose, rel)

        points = scan_to_points(scan)
        self._scans += 1
        score, matched = 0.0, False

        if points.shape[0] < config.SLAM_MIN_POINTS:
            self._pose = prediction
            if self._initialized and self._status == STATUS_OK:
                self._status = STATUS_DEGRADED
        elif not self._initialized:
            self._pose = prediction
            self._integrate(points)
            self._initialized = True
            self._status = STATUS_OK
            matched = True
        else:
            score, matched = self._localize(points, prediction)

        return self._publish(timestamp, score, matched, points.shape[0])

    def _localize(self, points, prediction: Pose) -> Tuple[float, bool]:
        if self._status == STATUS_LOST:
            return self._relocalize(points, prediction)

        result = self.matcher.match(points, prediction, config.SLAM_NARROW_XY_M, config.SLAM_NARROW_THETA_RAD)
        if result.score < config.SLAM_ACCEPT_SCORE or result.at_edge:
            result = self.matcher.match(points, prediction, config.SLAM_WIDE_XY_M, config.SLAM_WIDE_THETA_RAD)

        if result.score >= config.SLAM_ACCEPT_SCORE and not result.at_edge:
            if result.ambiguous:
                # Die Karte legt die Verschiebung nicht eindeutig fest (z. B. Flur ohne
                # Merkmale): Odometrie behalten, die Karte wächst mit dieser Pose weiter.
                self._pose = prediction
                self._fail_count = 0
                self._status = STATUS_DEGRADED
                self._integrate(points)
                return result.score, False
            self._pose = result.pose
            self._fail_count = 0
            self._status = STATUS_OK
            self._integrate(points)
            return result.score, True

        self._pose = prediction
        self._fail_count += 1
        self._status = STATUS_LOST if self._fail_count >= config.SLAM_LOST_AFTER else STATUS_DEGRADED
        if self._status == STATUS_LOST:
            logger.warning("Positionsbestimmung: verloren, suche global")
        return result.score, False

    def _relocalize(self, points, prediction: Pose) -> Tuple[float, bool]:
        result = self.matcher.match(
            points, prediction, config.SLAM_RELOCALIZE_XY_M, math.pi,
            step_xy=config.SLAM_RELOCALIZE_STEP_M, step_theta=0.05,
        )
        if result.score >= config.SLAM_RELOCALIZE_SCORE and not result.ambiguous:
            logger.info(f"Positionsbestimmung: wiedergefunden (Wertung {result.score:.2f})")
            self._pose = result.pose
            self._fail_count = 0
            self._status = STATUS_OK
            self._integrate(points)
            return result.score, True
        self._pose = prediction
        return result.score, False

    def _integrate(self, points) -> None:
        origin = compose(self._pose, (config.LIDAR_OFFSET_X, config.LIDAR_OFFSET_Y, 0.0))
        self.grid.integrate((origin[0], origin[1]), transform_points(points, self._pose))

    def _publish(self, timestamp: float, score: float, matched: bool, n_points: int) -> LocalizationResult:
        result = LocalizationResult(
            x=self._pose[0],
            y=self._pose[1],
            theta=self._pose[2],
            timestamp=timestamp,
            status=self._status,
            score=score,
            matched=matched,
            n_points=n_points,
            scans_processed=self._scans,
        )
        with self._result_lock:
            self._result = result
        return result
