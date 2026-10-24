"""Wegpunkte einer Fahrt: Erfassung, Pose-Quelle und Dateiformat ``*_path.json.gz``.

Alle Posen sind relativ zur Pose beim Start der Aufnahme (bzw. beim Start des
Abfahrens). Der Roboter muss zum Abfahren deshalb wieder an derselben Stelle
und in derselben Ausrichtung stehen wie beim Start der Aufnahme.
"""

import gzip
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from .. import config
from ..localization import STATUS_INIT, STATUS_LOST
from ..localization.geometry import Pose, relative, wrap_angle

PATH_FORMAT = "raspirobot-path"
PATH_FORMAT_VERSION = 1
PATH_FILE_SUFFIX = "_path.json.gz"

SOURCE_SLAM = "slam"
SOURCE_ODOM = "odom"


@dataclass
class Waypoint:
    t: float          # s seit Start der Aufnahme
    x: float          # m
    y: float          # m
    theta: float      # rad
    backward: bool = False  # Roboter fuhr hier rückwärts (B-Modus, kein Tank-Turn)


@dataclass
class RecordedPath:
    session_id: str
    pose_source: str
    waypoints: List[Waypoint]
    created: float = 0.0
    recording_file: str = ""

    def length_m(self) -> float:
        return sum(
            math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(self.waypoints, self.waypoints[1:])
        )


def path_filename(session_id: str) -> str:
    return f"{session_id}{PATH_FILE_SUFFIX}"


def save_path(path: RecordedPath, directory: str) -> Path:
    data = {
        "format": PATH_FORMAT,
        "version": PATH_FORMAT_VERSION,
        "session_id": path.session_id,
        "pose_source": path.pose_source,
        "created": path.created or time.time(),
        "recording_file": path.recording_file,
        "waypoints": [asdict(w) for w in path.waypoints],
    }
    target = Path(directory) / path_filename(path.session_id)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(".tmp")
    tmp.write_bytes(gzip.compress(json.dumps(data, indent=1).encode("utf-8")))
    tmp.replace(target)
    return target


def load_path(filepath) -> RecordedPath:
    """Raises: ValueError bei fremdem Format oder ohne Wegpunkte."""
    data = json.loads(gzip.decompress(Path(filepath).read_bytes()).decode("utf-8"))
    if data.get("format") != PATH_FORMAT:
        raise ValueError(f"Keine Wegdatei: {filepath}")
    if data.get("version") != PATH_FORMAT_VERSION:
        raise ValueError(f"Unbekannte Version {data.get('version')} in {filepath}")
    waypoints = [Waypoint(**w) for w in data["waypoints"]]
    if len(waypoints) < 2:
        raise ValueError(f"Weg hat weniger als zwei Wegpunkte: {filepath}")
    return RecordedPath(
        session_id=data["session_id"],
        pose_source=data["pose_source"],
        waypoints=waypoints,
        created=data.get("created", 0.0),
        recording_file=data.get("recording_file", ""),
    )


def find_latest_path(directory: str) -> Optional[Path]:
    files = sorted(Path(directory).glob(f"*{PATH_FILE_SUFFIX}"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


class WaypointSampler:
    """Legt einen Wegpunkt an, wenn sich der Roboter genug bewegt oder gedreht hat."""

    def __init__(
        self,
        spacing_m: float = config.PATH_WAYPOINT_SPACING_M,
        angle_rad: float = config.PATH_WAYPOINT_ANGLE_RAD,
    ):
        self.spacing_m = spacing_m
        self.angle_rad = angle_rad
        self.waypoints: List[Waypoint] = []

    def add(self, t: float, pose: Pose, backward: bool, force: bool = False) -> bool:
        x, y, theta = pose
        if self.waypoints and not force:
            last = self.waypoints[-1]
            moved = math.hypot(x - last.x, y - last.y)
            turned = abs(wrap_angle(theta - last.theta))
            if moved < self.spacing_m and turned < self.angle_rad:
                return False
        self.waypoints.append(Waypoint(t, x, y, theta, bool(backward)))
        return True


class PoseSource:
    """Liefert die Pose relativ zu einem Ursprung, aus SLAM wenn verfügbar, sonst aus der Odometrie.

    Die Quelle wird bei ``set_origin`` festgelegt und bleibt bis zum nächsten
    ``set_origin`` gleich, damit der Weg nicht zwischen zwei Rahmen springt.
    """

    def __init__(self, odom_getter: Callable, localizer=None, slam_available: Callable[[], bool] = lambda: False):
        self._odom_getter = odom_getter
        self._localizer = localizer
        self._slam_available = slam_available
        self.source = SOURCE_ODOM
        self._origin: Pose = (0.0, 0.0, 0.0)

    def slam_ready(self) -> bool:
        return (
            self._localizer is not None
            and self._slam_available()
            and self._localizer.get_pose().status != STATUS_INIT
        )

    def _absolute(self) -> Tuple[Pose, str, float]:
        """(Pose, Status, Zeitstempel) der gewählten Quelle."""
        if self.source == SOURCE_SLAM:
            loc = self._localizer.get_pose()
            return (loc.x, loc.y, loc.theta), loc.status, loc.timestamp
        odom = self._odom_getter()
        return (odom.x, odom.y, odom.theta), "ODOM", time.time()

    def set_origin(self) -> str:
        self.source = SOURCE_SLAM if self.slam_ready() else SOURCE_ODOM
        self._origin = self._absolute()[0]
        return self.source

    def get(self) -> Tuple[Pose, str, float]:
        """(Pose relativ zum Ursprung, Status, Alter in s)."""
        pose, status, timestamp = self._absolute()
        return relative(self._origin, pose), status, max(0.0, time.time() - timestamp)

    def is_reliable(self, max_age: float = config.FOLLOW_POSE_MAX_AGE_SEC) -> bool:
        if self.source == SOURCE_ODOM:
            return True
        _, status, age = self.get()
        return status != STATUS_LOST and age <= max_age
