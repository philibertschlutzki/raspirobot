"""Wegpunkte einer Fahrt: Erfassung, Pose-Quelle und Dateiformat ``*_path.json.gz``.

Alle Posen sind relativ zur Pose beim Start der Aufnahme (bzw. beim Start des
Abfahrens). Der Roboter muss zum Abfahren deshalb wieder an derselben Stelle
und in derselben Ausrichtung stehen wie beim Start der Aufnahme.

Ab Formatversion 2 steht an jedem Wegpunkt zusätzlich der SLAM-Status, die
Match-Wertung und die Odometrie-Pose, damit sich eine Fahrt nachträglich
beurteilen lässt (``python -m raspirobot.path.analyze``). Version 1 lässt sich
weiter laden.
"""

import gzip
import json
import math
import time
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from .. import config
from ..localization import STATUS_INIT, STATUS_LOST
from ..localization.geometry import Pose, compose, relative, wrap_angle

PATH_FORMAT = "raspirobot-path"
PATH_FORMAT_VERSION = 2
SUPPORTED_PATH_VERSIONS = (1, 2)
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
    # Ab Version 2, nur zur Auswertung; der Regler nutzt sie nicht.
    status: str = ""                    # SLAM-Status (OK/DEGRADED/LOST) bzw. ODOM
    score: Optional[float] = None       # Wertung des letzten Scan-Matches (0..1)
    odom_x: Optional[float] = None      # Odometrie-Pose relativ zum Start der Aufnahme
    odom_y: Optional[float] = None
    odom_theta: Optional[float] = None


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
    if data.get("version") not in SUPPORTED_PATH_VERSIONS:
        raise ValueError(f"Unbekannte Version {data.get('version')} in {filepath}")
    known = {f.name for f in fields(Waypoint)}
    waypoints = [Waypoint(**{k: v for k, v in w.items() if k in known}) for w in data["waypoints"]]
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


def densify(waypoints: List[Waypoint], spacing_m: float = config.PATH_WAYPOINT_SPACING_M) -> List[Waypoint]:
    """Füllt Lücken über ``spacing_m`` mit linear interpolierten Wegpunkten auf.

    Es fällt kein Wegpunkt weg; Drehungen auf der Stelle (gleiche Position,
    andere Richtung) bleiben deshalb erhalten. Eingefügte Punkte übernehmen die
    Fahrtrichtung des folgenden Wegpunkts und tragen keine Diagnosewerte.
    """
    if len(waypoints) < 2 or spacing_m <= 0:
        return list(waypoints)
    out = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        n = int(math.ceil(math.hypot(b.x - a.x, b.y - a.y) / spacing_m - 1e-9))
        dtheta = float(wrap_angle(b.theta - a.theta))
        for i in range(1, n):
            f = i / n
            out.append(replace(
                b,
                t=a.t + (b.t - a.t) * f,
                x=a.x + (b.x - a.x) * f,
                y=a.y + (b.y - a.y) * f,
                theta=float(wrap_angle(a.theta + dtheta * f)),
                status="", score=None, odom_x=None, odom_y=None, odom_theta=None,
            ))
        out.append(b)
    return out


@dataclass(frozen=True)
class PoseSample:
    """Pose relativ zum Ursprung plus Angaben zur Beurteilung."""

    pose: Pose
    status: str
    age: float                    # s seit dem zugrunde liegenden Scan (Odometrie: 0)
    score: Optional[float]        # Match-Wertung, nur bei SLAM
    odom: Pose                    # Odometrie relativ zu ihrem Stand beim Ursprung


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

    def add(self, t: float, pose: Pose, backward: bool, force: bool = False,
            sample: Optional[PoseSample] = None) -> bool:
        """``sample`` liefert die Diagnosewerte des Wegpunkts (Status, Wertung, Odometrie)."""
        x, y, theta = pose
        if self.waypoints and not force:
            last = self.waypoints[-1]
            moved = math.hypot(x - last.x, y - last.y)
            turned = abs(wrap_angle(theta - last.theta))
            if moved < self.spacing_m and turned < self.angle_rad:
                return False
        wp = Waypoint(t, x, y, theta, bool(backward))
        if sample is not None:
            wp.status = sample.status
            wp.score = None if sample.score is None else round(float(sample.score), 3)
            wp.odom_x, wp.odom_y, wp.odom_theta = (float(v) for v in sample.odom)
        self.waypoints.append(wp)
        return True


class PoseSource:
    """Liefert die Pose relativ zu einem Ursprung, aus SLAM wenn verfügbar, sonst aus der Odometrie.

    Die Quelle wird bei ``set_origin`` festgelegt und bleibt bis zum nächsten
    ``set_origin`` gleich, damit der Weg nicht zwischen zwei Rahmen springt.

    SLAM liefert nur je verarbeitetem Scan eine neue Pose (etwa 8-10 Hz, bei
    langsamem Scan-Matching seltener). Dazwischen wird die letzte SLAM-Pose mit
    der Odometrie-Bewegung seit diesem Scan fortgeschrieben
    (``extrapolate``); sonst stünde die Pose zwischen zwei Scans still und
    spränge danach.
    """

    def __init__(self, odom_getter: Callable, localizer=None, slam_available: Callable[[], bool] = lambda: False,
                 extrapolate: bool = True):
        self._odom_getter = odom_getter
        self._localizer = localizer
        self._slam_available = slam_available
        self.extrapolate = extrapolate
        self.source = SOURCE_ODOM
        self._origin: Pose = (0.0, 0.0, 0.0)
        self._odom_origin: Pose = (0.0, 0.0, 0.0)
        self._odom_origin_epoch = 0
        self._pending_since: Optional[float] = None   # wartet nach ``restart_slam`` auf den ersten Scan

    def slam_ready(self) -> bool:
        return (
            self._localizer is not None
            and self._slam_available()
            and self._localizer.get_pose().status != STATUS_INIT
        )

    def _absolute(self, odom=None) -> Tuple[Pose, str, float, Optional[float]]:
        """(Pose, Status, Zeitstempel, Wertung) der gewählten Quelle."""
        odom = odom if odom is not None else self._odom_getter()
        odom_pose = (odom.x, odom.y, odom.theta)
        if self.source == SOURCE_SLAM:
            loc = self._localizer.get_pose()
            pose = (loc.x, loc.y, loc.theta)
            scan_odom = getattr(loc, "odom", None)
            if (self.extrapolate and scan_odom is not None
                    and getattr(loc, "odom_epoch", -1) == getattr(odom, "epoch", 0)):
                pose = compose(pose, relative(scan_odom, odom_pose))
            return pose, loc.status, loc.timestamp, getattr(loc, "score", None)
        return odom_pose, "ODOM", time.time(), None

    def set_origin(self) -> str:
        self._pending_since = None
        self.source = SOURCE_SLAM if self.slam_ready() else SOURCE_ODOM
        odom = self._odom_getter()
        self._origin = self._absolute(odom)[0]
        self._odom_origin = (odom.x, odom.y, odom.theta)
        self._odom_origin_epoch = getattr(odom, "epoch", 0)
        return self.source

    def restart_slam(self, timeout: float = config.SLAM_RESTART_TIMEOUT_SEC) -> Optional[str]:
        """Beginnt eine neue SLAM-Karte; der Ursprung liegt an der Pose des ersten Scans.

        So startet ein Weg nicht in einer Karte, die vorher beim Herumfahren
        entstanden (und womöglich verloren) ist. Blockiert nicht: Bis der erste
        Scan verarbeitet ist, meldet ``sample`` die Pose (0, 0, 0) mit Status
        ``INIT``. Kommt nach ``timeout`` Sekunden kein Scan, wird auf die
        Odometrie ausgewichen. Gibt None zurück, wenn SLAM nicht verfügbar ist
        (dann ``set_origin`` nutzen).
        """
        if self._localizer is None or not hasattr(self._localizer, "reset") or not self._slam_available():
            return None
        self._localizer.reset()
        self.source = SOURCE_SLAM
        self._pending_since = time.time()
        self._pending_timeout = timeout
        odom = self._odom_getter()
        self._odom_origin = (odom.x, odom.y, odom.theta)
        self._odom_origin_epoch = getattr(odom, "epoch", 0)
        self._origin = (0.0, 0.0, 0.0)
        return self.source

    def _resolve_pending(self, odom) -> bool:
        """True, solange noch auf den ersten Scan gewartet wird."""
        if self._pending_since is None:
            return False
        if self._localizer.get_pose().status != STATUS_INIT:
            self._pending_since = None
            self._origin = self._absolute(odom)[0]
            return False
        if time.time() - self._pending_since > self._pending_timeout:
            self._pending_since = None
            self.source = SOURCE_ODOM
            self._origin = (odom.x, odom.y, odom.theta)
            return False
        return True

    def sample(self) -> PoseSample:
        odom = self._odom_getter()
        if self._resolve_pending(odom):
            return PoseSample((0.0, 0.0, 0.0), STATUS_INIT, 0.0, None,
                              relative(self._odom_origin, (odom.x, odom.y, odom.theta)))
        pose, status, timestamp, score = self._absolute(odom)
        odom_pose = (odom.x, odom.y, odom.theta)
        if getattr(odom, "epoch", 0) != self._odom_origin_epoch:
            # Odometrie wurde nach dem Ursprung zurückgesetzt: sie zählt dann ab ihrem Nullpunkt.
            self._odom_origin, self._odom_origin_epoch = (0.0, 0.0, 0.0), getattr(odom, "epoch", 0)
        return PoseSample(
            pose=relative(self._origin, pose),
            status=status,
            age=max(0.0, time.time() - timestamp),
            score=score,
            odom=relative(self._odom_origin, odom_pose),
        )

    def get(self) -> Tuple[Pose, str, float]:
        """(Pose relativ zum Ursprung, Status, Alter in s)."""
        s = self.sample()
        return s.pose, s.status, s.age

    def is_reliable(self, max_age: float = config.FOLLOW_POSE_MAX_AGE_SEC) -> bool:
        if self.source == SOURCE_ODOM:
            return True
        _, status, age = self.get()
        return status != STATUS_LOST and age <= max_age
