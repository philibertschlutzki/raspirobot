"""Manuelle Fahrt wie ``RobotApp``, zusätzlich mit Wegpunkten je Aufnahme.

Neben der bisherigen Aufnahme ``<session>.json.gz`` entsteht
``<session>_path.json.gz`` mit den Posen relativ zum Startpunkt der Aufnahme.
Die Odometrie wertet hier die Richtung je Rad aus, damit ein Tank-Turn als
Drehung im Weg landet.
"""

import time

from .. import config
from ..app import RobotApp
from ..log import logger
from ..recorder import Recorder
from .waypoints import PoseSource, RecordedPath, WaypointSampler, save_path


class PathRecorder(Recorder):
    """``Recorder`` plus Wegpunkte aus einer ``PoseSource``."""

    def __init__(self, pose_source: PoseSource, **kwargs):
        super().__init__(**kwargs)
        self._pose_source = pose_source
        self.sampler = WaypointSampler()
        self.path_file = None
        self._base_dir = config.RECORDING_BASE_DIR

    def setup(self, base_dir: str = config.RECORDING_BASE_DIR) -> None:
        super().setup(base_dir)
        self._base_dir = base_dir

    def start(self) -> bool:
        if not super().start():
            return False
        # Nach dem Odometrie-Reset in ``super().start()``: Ursprung des Wegs festlegen.
        # Frische Karte ab Aufnahmestart; ohne LIDAR bzw. SLAM bleibt es bei der Odometrie.
        source = self._pose_source.restart_slam() or self._pose_source.set_origin()
        self.sampler = WaypointSampler()
        self.path_file = None
        sample = self._pose_source.sample()
        self.sampler.add(0.0, sample.pose, backward=False, force=True, sample=sample)
        print(f"[PATH] 📍 Wegaufzeichnung gestartet (Pose-Quelle: {source.upper()})")
        return True

    def add_sample(self, controller) -> None:
        super().add_sample(controller)
        if not self.active:
            return
        sample = self._pose_source.sample()
        backward = not controller.is_moving_forward and not controller.a_pressed
        self.sampler.add(time.time() - self.start_time, sample.pose, backward, sample=sample)

    def stop(self, force_save: bool = False) -> bool:
        was_active = self.active
        if was_active:
            # Endpose immer übernehmen, auch wenn sie unter dem Mindestabstand liegt.
            sample = self._pose_source.sample()
            self.sampler.add(time.time() - self.start_time, sample.pose,
                             backward=self.sampler.waypoints[-1].backward if self.sampler.waypoints else False,
                             force=True, sample=sample)
        saved = super().stop(force_save=force_save)
        if was_active and saved:
            self._save_path()
        return saved

    def _save_path(self) -> None:
        waypoints = self.sampler.waypoints
        if len(waypoints) < 2:
            print("[PATH] ⚠️ Zu wenige Wegpunkte - kein Weg gespeichert")
            return
        path = RecordedPath(
            session_id=self.session_id,
            pose_source=self._pose_source.source,
            waypoints=waypoints,
            created=time.time(),
            recording_file=f"{self.session_id}.json.gz",
        )
        try:
            self.path_file = save_path(path, self._base_dir)
            print(f"[PATH] ✅ Weg gespeichert: {self.path_file} "
                  f"({len(waypoints)} Wegpunkte, {path.length_m():.2f} m)")
            logger.info(f"Weg gespeichert: {self.path_file}")
        except Exception as e:
            print(f"[PATH] ❌ Weg nicht gespeichert: {e}")
            logger.error(f"Weg nicht gespeichert: {e}")


class TeachApp(RobotApp):
    USE_WHEEL_DIRECTIONS = True

    def _create_recorder(self) -> Recorder:
        self.pose_source = PoseSource(
            self.pose.get_pose, self.localizer, slam_available=lambda: self.lidar.connected
        )
        return PathRecorder(
            self.pose_source,
            status_indicator=self.status,
            reset_pose=self.pose.reset,
            lidar_connected=lambda: self.lidar.connected,
        )

    def _print_banner(self) -> None:
        super()._print_banner()
        print("📍 WEG: Jede Aufnahme (X + Y) speichert zusätzlich <session>_path.json.gz.")
        print("   Startpunkt und Ausrichtung merken: dort muss der Roboter zum Abfahren wieder stehen.\n")

    def _print_status(self, left_duty: float, right_duty: float) -> None:
        super()._print_status(left_duty, right_duty)
        if self.recorder.active:
            (x, y, _), status, _ = self.pose_source.get()
            print(f"[PATH] {len(self.recorder.sampler.waypoints)} Wegpunkte | "
                  f"{self.pose_source.source.upper()}[{status}] X={x:.2f}m Y={y:.2f}m")
