"""Auswertung einer Wegaufzeichnung ohne Hardware.

    python -m raspirobot.path.analyze path_recordings/<session>_path.json.gz [--aufnahme <session>.json.gz]

Liest die Wegdatei und, falls vorhanden, die zugehörige Aufnahme mit
Controller-Samples und LIDAR-Frames. Aus der Aufnahme wird nachgerechnet:

- Odometrie aus den Controller-Eingaben (wie ``PoseTracker`` mit Richtung je
  Rad). Die Kollisionsbegrenzung durch den Ultraschall ist nicht aufgezeichnet
  und fehlt deshalb; die LIDAR-Begrenzung wird aus den Frames nachgebildet.
- SLAM: Jeder LIDAR-Frame läuft durch einen frischen ``Localizer``. Mit
  ``--cpu-faktor`` lässt sich ein langsamerer Rechner nachbilden: Frames, die
  eintreffen, während der vorige noch gerechnet wird, fallen weg wie live.
- LIDAR-Takt: Abstände der Frames und Punkte je Umlauf. Der RPLIDAR C1 misst
  rund 5000 Punkte/s; aus den Punkten je Umlauf folgt die Drehzahl des Motors.
  Liegt sie deutlich über der gemessenen Frame-Rate, gehen Scans in der
  Software verloren oder stauen sich.

Die Aufnahme wird nur gelesen; Motoren oder Sensoren werden nicht angesprochen.
"""

import argparse
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, Sequence, Tuple

from .. import config
from ..collision import analyze_collision
from ..drive import compute_drive_command, limit_for_collision, trigger_to_duty_cycle
from ..localization import STATUS_LOST, Localizer
from ..localization.geometry import Pose, compose, relative, wrap_angle
from ..odometry import integrate_pose_signed
from .waypoints import RecordedPath, WaypointSampler, load_path

C1_SAMPLE_RATE_HZ = 5000.0   # Messpunkte pro Sekunde laut Datenblatt RPLIDAR C1


def _quantile(values: Sequence[float], q: float) -> float:
    values = sorted(values)
    if not values:
        return float("nan")
    pos = (len(values) - 1) * q
    lo, hi = int(math.floor(pos)), int(math.ceil(pos))
    return values[lo] + (values[hi] - values[lo]) * (pos - lo)


# --------------------------------------------------------------------- Weg

@dataclass
class Gap:
    index: int          # Wegpunkt nach der Lücke
    t: float            # s seit Start
    dt: float
    distance_m: float
    status: str


@dataclass
class PathStats:
    n: int
    length_m: float
    duration_s: float
    source: str
    version_with_diagnostics: bool
    spacing_median_m: float
    spacing_p90_m: float
    spacing_max_m: float
    gaps: List[Gap]                         # Abstand > 2 * PATH_WAYPOINT_SPACING_M, größte zuerst
    status_counts: dict
    score_median: Optional[float]
    score_min: Optional[float]
    odom_divergence_end_m: Optional[float]
    odom_divergence_max_m: Optional[float]


def path_stats(path: RecordedPath, max_gaps: int = 10) -> PathStats:
    wps = path.waypoints
    spacing = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(wps, wps[1:])]
    gaps = [
        Gap(i + 1, wps[i + 1].t, wps[i + 1].t - wps[i].t, d, wps[i + 1].status)
        for i, d in enumerate(spacing) if d > 2 * config.PATH_WAYPOINT_SPACING_M
    ]
    gaps.sort(key=lambda g: g.distance_m, reverse=True)

    with_diag = any(w.odom_x is not None for w in wps)
    counts: dict = {}
    for w in wps:
        if w.status:
            counts[w.status] = counts.get(w.status, 0) + 1
    scores = [w.score for w in wps if w.score is not None and w.status != "ODOM"]
    div = [math.hypot(w.x - w.odom_x, w.y - w.odom_y) for w in wps if w.odom_x is not None]
    return PathStats(
        n=len(wps),
        length_m=path.length_m(),
        duration_s=wps[-1].t - wps[0].t,
        source=path.pose_source,
        version_with_diagnostics=with_diag,
        spacing_median_m=_quantile(spacing, 0.5),
        spacing_p90_m=_quantile(spacing, 0.9),
        spacing_max_m=max(spacing) if spacing else 0.0,
        gaps=gaps[:max_gaps],
        status_counts=counts,
        score_median=_quantile(scores, 0.5) if scores else None,
        score_min=min(scores) if scores else None,
        odom_divergence_end_m=div[-1] if div else None,
        odom_divergence_max_m=max(div) if div else None,
    )


# --------------------------------------------------------------- LIDAR-Takt

@dataclass
class LidarStats:
    frames: int
    duration_s: float
    rate_hz: float
    interval_median_s: float
    interval_p90_s: float
    interval_max_s: float
    long_intervals: int        # Abstand > 1,5 x Median: Scan verloren oder verzögert
    points_median: float
    valid_points_median: float
    motor_hz_from_points: float
    controller_rate_hz: float


def lidar_stats(recording) -> Optional[LidarStats]:
    frames = recording.lidar_frames
    if len(frames) < 2:
        return None
    ts = [f.timestamp for f in frames]
    intervals = [b - a for a, b in zip(ts, ts[1:])]
    med = _quantile(intervals, 0.5)
    points = [len(f.scan_data) for f in frames]
    valid = [sum(1 for _q, _a, d in f.scan_data if d > 0) for f in frames]
    cs = recording.controller_samples
    c_dur = cs[-1].timestamp - cs[0].timestamp if len(cs) > 1 else 0.0
    duration = ts[-1] - ts[0]
    points_med = _quantile(points, 0.5)
    return LidarStats(
        frames=len(frames),
        duration_s=duration,
        rate_hz=(len(frames) - 1) / duration if duration > 0 else float("nan"),
        interval_median_s=med,
        interval_p90_s=_quantile(intervals, 0.9),
        interval_max_s=max(intervals),
        long_intervals=sum(1 for i in intervals if i > 1.5 * med),
        points_median=points_med,
        valid_points_median=_quantile(valid, 0.5),
        motor_hz_from_points=C1_SAMPLE_RATE_HZ / points_med if points_med else float("nan"),
        controller_rate_hz=(len(cs) - 1) / c_dur if c_dur > 0 else float("nan"),
    )


# ------------------------------------------------------------ Nachrechnung

@dataclass
class ReplayPoint:
    t: float            # s seit Start der Aufnahme
    slam: Pose
    odom: Pose
    status: str
    score: float
    matched: bool
    processed: bool     # False: Frame wäre live übersprungen worden (``cpu_factor``)
    proc_time_s: float


@dataclass
class Replay:
    points: List[ReplayPoint] = field(default_factory=list)
    lost_intervals: List[Tuple[float, float]] = field(default_factory=list)
    odom_track: List[Tuple[float, Pose]] = field(default_factory=list)

    def processed(self) -> List[ReplayPoint]:
        return [p for p in self.points if p.processed]


def replay_odometry(recording) -> List[Tuple[float, Pose]]:
    """Odometrie (t seit Start, Pose) aus den Controller-Samples, Richtung je Rad wie im Aufnahme-Script."""
    cs = recording.controller_samples
    if not cs:
        return []
    t0 = cs[0].timestamp
    frames = recording.lidar_frames
    fi = 0
    scan: list = []
    pose: Pose = (0.0, 0.0, 0.0)
    track = [(0.0, pose)]
    for a, b in zip(cs, cs[1:]):
        while fi < len(frames) and frames[fi].timestamp <= a.timestamp:
            scan = frames[fi].scan_data
            fi += 1
        forward = a.direction.get("forward", 1.0) > 0
        tank = bool(a.buttons.get("A", False))
        cmd = compute_drive_command(
            trigger_to_duty_cycle(a.left_trigger), trigger_to_duty_cycle(a.right_trigger), tank, forward
        )
        # Ultraschall ist nicht aufgezeichnet: nur die LIDAR-Begrenzung nachbilden.
        left, right = limit_for_collision(cmd.left_duty, cmd.right_duty, analyze_collision(1e9, 1e9, scan))
        left = -left if cmd.left_backward else left
        right = -right if cmd.right_backward else right
        pose = integrate_pose_signed(*pose, left, right, b.timestamp - a.timestamp)
        track.append((b.timestamp - t0, pose))
    return track


def _pose_at(track: List[Tuple[float, Pose]], t: float) -> Pose:
    """Letzte Pose mit Zeit <= t (Track ist zeitlich sortiert)."""
    lo, hi = 0, len(track) - 1
    if t <= track[0][0]:
        return track[0][1]
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if track[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return track[lo][1]


def replay_slam(recording, cpu_factor: float = 1.0) -> Replay:
    """Lässt die LIDAR-Frames durch einen frischen ``Localizer`` laufen.

    ``cpu_factor`` > 1 bildet einen langsameren Rechner nach: Ein Frame wird nur
    verarbeitet, wenn der vorige (Rechenzeit x Faktor) fertig ist, sonst fällt
    er weg, wie im Localization-Thread, der immer nur den neuesten Scan nimmt.
    """
    track = replay_odometry(recording)
    replay = Replay(odom_track=track)
    if not track or not recording.lidar_frames:
        return replay
    t0 = recording.controller_samples[0].timestamp
    odom_now = {"pose": track[0][1]}
    loc = Localizer(lambda: SimpleNamespace(x=odom_now["pose"][0], y=odom_now["pose"][1],
                                            theta=odom_now["pose"][2], epoch=0))
    busy_until = -math.inf
    last = None
    lost_since: Optional[float] = None
    for frame in recording.lidar_frames:
        t = frame.timestamp - t0
        odom = _pose_at(track, t)
        if t < busy_until:
            if last is not None:
                # Live bliebe die alte SLAM-Pose stehen; mit Odometrie fortgeschrieben wie PoseSource.
                slam = compose(last.slam, relative(last.odom, odom))
                replay.points.append(ReplayPoint(t, slam, odom, last.status, last.score, False, False, 0.0))
            continue
        odom_now["pose"] = odom
        start = time.perf_counter()
        odom_obj = SimpleNamespace(x=odom[0], y=odom[1], theta=odom[2], epoch=0)
        res = loc.process(frame.timestamp, frame.scan_data, odom_obj)
        proc = time.perf_counter() - start
        busy_until = t + proc * cpu_factor
        last = ReplayPoint(t, (res.x, res.y, res.theta), odom, res.status, res.score, res.matched, True, proc)
        replay.points.append(last)
        if res.status == STATUS_LOST and lost_since is None:
            lost_since = t
        elif res.status != STATUS_LOST and lost_since is not None:
            replay.lost_intervals.append((lost_since, t))
            lost_since = None
    if lost_since is not None:
        replay.lost_intervals.append((lost_since, replay.points[-1].t))
    return replay


def resample_waypoints(replay: Replay) -> int:
    """Wie viele Wegpunkte die Nachrechnung mit 50 Hz Odometrie-Fortschreibung ergäbe."""
    sampler = WaypointSampler()
    pts = replay.processed()
    if not pts:
        return 0
    j = 0
    for t, odom in replay.odom_track:
        while j + 1 < len(pts) and pts[j + 1].t <= t:
            j += 1
        p = pts[j]
        pose = compose(p.slam, relative(p.odom, odom)) if p.t <= t else p.slam
        sampler.add(t, pose, False, force=not sampler.waypoints)
    return len(sampler.waypoints)


# ------------------------------------------------------------------ Bericht

def _fmt_pose(p: Pose) -> str:
    return f"({p[0]:+.2f}, {p[1]:+.2f}, {math.degrees(p[2]):+.0f}°)"


def report(path: Optional[RecordedPath], recording, cpu_factor: float = 1.0, timeline_step_s: float = 1.0) -> str:
    lines: List[str] = []
    out = lines.append

    if path is not None:
        ps = path_stats(path)
        out("== Weg ==")
        out(f"Pose-Quelle der Wegpunkte: {ps.source.upper()}")
        out(f"{ps.n} Wegpunkte, {ps.length_m:.2f} m in {ps.duration_s:.1f} s "
            f"(Ø {ps.length_m / ps.duration_s if ps.duration_s else 0:.3f} m/s, "
            f"Ø {ps.length_m / max(ps.n - 1, 1) * 100:.1f} cm je Wegpunkt; "
            f"Soll {config.PATH_WAYPOINT_SPACING_M * 100:.0f} cm)")
        out(f"Abstände: Median {ps.spacing_median_m * 100:.1f} cm, 90 % {ps.spacing_p90_m * 100:.1f} cm, "
            f"max {ps.spacing_max_m * 100:.1f} cm")
        if ps.gaps:
            out("Größte Lücken (Wegpunkt, Zeit, Δt, Abstand, Status):")
            for g in ps.gaps:
                out(f"  #{g.index:3d}  t={g.t:6.1f}s  Δt={g.dt:5.2f}s  {g.distance_m * 100:5.1f} cm  {g.status or '-'}")
        if ps.version_with_diagnostics:
            out(f"Status an den Wegpunkten: {ps.status_counts}")
            if ps.score_median is not None:
                out(f"SLAM-Wertung: Median {ps.score_median:.2f}, min {ps.score_min:.2f} "
                    f"(Schwelle {config.SLAM_ACCEPT_SCORE:.2f})")
            out(f"Abstand SLAM ↔ Odometrie: am Ende {ps.odom_divergence_end_m:.2f} m, "
                f"max {ps.odom_divergence_max_m:.2f} m")
        else:
            out("Wegdatei Version 1: keine SLAM-Wertung oder Odometrie je Wegpunkt gespeichert.")
        out("")

    if recording is None:
        out("Keine Aufnahme (*.json.gz) gefunden: LIDAR-Takt und SLAM-Nachrechnung entfallen.")
        return "\n".join(lines)

    ls = lidar_stats(recording)
    out("== LIDAR-Takt ==")
    if ls is None:
        out("Weniger als zwei LIDAR-Frames in der Aufnahme.")
        return "\n".join(lines)
    out(f"{ls.frames} Frames in {ls.duration_s:.1f} s = {ls.rate_hz:.2f} Hz")
    out(f"Frame-Abstand: Median {ls.interval_median_s * 1000:.0f} ms, 90 % {ls.interval_p90_s * 1000:.0f} ms, "
        f"max {ls.interval_max_s * 1000:.0f} ms; {ls.long_intervals} Abstände > 1,5 x Median")
    out(f"Punkte je Umlauf: Median {ls.points_median:.0f} (gültig {ls.valid_points_median:.0f}) "
        f"→ Motor dreht etwa {ls.motor_hz_from_points:.1f} Hz bei {C1_SAMPLE_RATE_HZ:.0f} Punkten/s")
    if ls.motor_hz_from_points > 1.15 * ls.rate_hz:
        out("  Der Sensor dreht schneller, als Frames ankommen: Scans gehen in der Software verloren "
            "oder stauen sich (Log nach 'Too many bytes in the input buffer' durchsuchen).")
    else:
        out("  Frame-Rate passt zur Drehzahl: Der Sensor dreht selbst so langsam, es gehen keine Scans verloren.")
    out(f"Controller-Samples: {ls.controller_rate_hz:.1f} Hz (Soll {config.RECORDING_SAMPLE_RATE_HZ} Hz)")
    out("")

    rp = replay_slam(recording, cpu_factor)
    done = rp.processed()
    out(f"== SLAM nachgerechnet (CPU-Faktor {cpu_factor:g}) ==")
    if not done:
        out("Keine Frames verarbeitet.")
        return "\n".join(lines)
    procs = [p.proc_time_s for p in done]
    out(f"{len(done)} von {len(rp.points)} Frames verarbeitet; Rechenzeit je Scan Median "
        f"{_quantile(procs, 0.5) * 1000:.0f} ms, 90 % {_quantile(procs, 0.9) * 1000:.0f} ms (dieser Rechner)")
    counts: dict = {}
    for p in done:
        counts[p.status] = counts.get(p.status, 0) + 1
    out(f"Status je Scan: {counts}; Matches übernommen: {sum(p.matched for p in done)}")
    scores = [p.score for p in done if p.score > 0]
    if scores:
        out(f"Wertung: Median {_quantile(scores, 0.5):.2f}, 10 % {_quantile(scores, 0.1):.2f}, "
            f"min {min(scores):.2f} (Schwelle {config.SLAM_ACCEPT_SCORE:.2f})")
    if rp.lost_intervals:
        out("LOST: " + ", ".join(f"{a:.1f}–{b:.1f} s" for a, b in rp.lost_intervals))
    else:
        out("LOST: nie")
    div = [math.hypot(p.slam[0] - p.odom[0], p.slam[1] - p.odom[1]) for p in done]
    out(f"Abstand SLAM ↔ Odometrie: am Ende {div[-1]:.2f} m, max {max(div):.2f} m; "
        f"Richtung am Ende {math.degrees(wrap_angle(done[-1].slam[2] - done[-1].odom[2])):+.0f}°")
    slam_len = sum(math.hypot(b.slam[0] - a.slam[0], b.slam[1] - a.slam[1]) for a, b in zip(done, done[1:]))
    odom_len = sum(math.hypot(b.odom[0] - a.odom[0], b.odom[1] - a.odom[1]) for a, b in zip(done, done[1:]))
    out(f"Weglänge SLAM {slam_len:.2f} m, Odometrie {odom_len:.2f} m")
    if odom_len > 0.2 and slam_len > 0.2 and done[-1].status != STATUS_LOST:
        out(f"  Passt die SLAM-Länge, wäre ODOMETRY_MAX_SPEED_M_S etwa "
            f"{config.ODOMETRY_MAX_SPEED_M_S * slam_len / odom_len:.2f} statt {config.ODOMETRY_MAX_SPEED_M_S:.2f} "
            "(grob; Drehungen und Rauschen verfälschen die Länge)")
    out(f"Mit Odometrie-Fortschreibung zwischen Scans ergäben sich {resample_waypoints(rp)} Wegpunkte.")

    if path is not None:
        errs = []
        for w in path.waypoints:
            p = min(done, key=lambda q: abs(q.t - w.t))
            errs.append(math.hypot(w.x - p.slam[0], w.y - p.slam[1]))
        out(f"Wegpunkte ↔ Nachrechnung: Median {_quantile(errs, 0.5):.2f} m, max {max(errs):.2f} m "
            "(die Live-Karte entstand schon vor der Aufnahme; kleine Abweichungen sind normal)")

    out("")
    out("Zeitverlauf (t, Status, Wertung, SLAM-Pose, Odometrie-Pose, Abstand):")
    next_t = 0.0
    for p in done:
        if p.t + 1e-9 < next_t:
            continue
        next_t = p.t + timeline_step_s
        out(f"  {p.t:6.1f}s  {p.status:8s} {p.score:4.2f}  SLAM {_fmt_pose(p.slam)}  ODOM {_fmt_pose(p.odom)}  "
            f"{math.hypot(p.slam[0] - p.odom[0], p.slam[1] - p.odom[1]):.2f} m")
    return "\n".join(lines)


def _load_recording(filepath: Path):
    from ..recording.storage import PathRecordingStorageManager

    storage = PathRecordingStorageManager(str(filepath.parent), backup_enabled=False)
    return storage.load_json(filepath)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Wegaufzeichnung auswerten (nur lesend, ohne Hardware)")
    parser.add_argument("weg", help="*_path.json.gz oder Aufnahme *.json.gz")
    parser.add_argument("--aufnahme", help="zugehörige Aufnahme *.json.gz (Standard: aus der Wegdatei)")
    parser.add_argument("--cpu-faktor", type=float, default=1.0,
                        help="Rechenzeit x Faktor, um den Pi nachzubilden (z. B. 4)")
    parser.add_argument("--takt", type=float, default=1.0, help="Abstand der Zeilen im Zeitverlauf in s")
    args = parser.parse_args(argv)

    target = Path(args.weg)
    path = None
    rec_file = Path(args.aufnahme) if args.aufnahme else None
    if target.name.endswith("_path.json.gz"):
        path = load_path(target)
        if rec_file is None and path.recording_file:
            rec_file = target.parent / path.recording_file
    elif rec_file is None:
        rec_file = target
    recording = _load_recording(rec_file) if rec_file is not None and rec_file.exists() else None
    print(report(path, recording, args.cpu_faktor, args.takt))


if __name__ == "__main__":
    main()
