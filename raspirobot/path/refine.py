"""Wegdatei nachträglich aus der Aufnahme berechnen (ohne Hardware).

    python -m raspirobot.path.refine path_recordings/<session>.json.gz

Live bekommt der Localizer nur so viele Scans, wie der Pi schafft, und verliert
bei LIDAR-Aussetzern (Neustart des Scans, bis zu 2,5 s) die Position. Hier
laufen alle aufgezeichneten LIDAR-Frames in Ruhe durch einen frischen
Localizer; die Odometrie aus den Controller-Eingaben schreibt die Pose
zwischen den Scans fort. Heraus kommt ``<session>_refined_path.json.gz`` im
selben Verzeichnis, mit Wegpunkten alle 2 cm bzw. 5° und denselben
Diagnosewerten wie die Live-Wegdatei. ``run_path_follow.py`` nimmt ohne
Dateiangabe die neueste ``*_path.json.gz`` und findet sie damit.
"""

import argparse
import time
from pathlib import Path
from typing import List, Optional

from ..localization.geometry import compose, relative
from .analyze import Replay, _load_recording, replay_slam
from .waypoints import SOURCE_ODOM, SOURCE_SLAM, PoseSample, RecordedPath, Waypoint, WaypointSampler, save_path

REFINED_SUFFIX = "_refined"


def refined_waypoints(recording, replay: Optional[Replay] = None) -> List[Waypoint]:
    """Wegpunkte im 50-Hz-Takt der Odometrie, SLAM-Pose zwischen den Scans mit der Odometrie fortgeschrieben."""
    replay = replay or replay_slam(recording)
    scans = replay.processed()
    if not scans or not replay.odom_track:
        return []
    controller = recording.controller_samples
    t0 = controller[0].timestamp
    sampler = WaypointSampler()
    j, k = 0, 0
    for t, odom in replay.odom_track:
        while j + 1 < len(scans) and scans[j + 1].t <= t:
            j += 1
        scan = scans[j]
        pose = compose(scan.slam, relative(scan.odom, odom)) if scan.t <= t else scan.slam
        while k + 1 < len(controller) and controller[k + 1].timestamp - t0 <= t:
            k += 1
        c = controller[k]
        backward = c.direction.get("forward", 1.0) < 0 and not c.buttons.get("A", False)
        sample = PoseSample(pose, scan.status if scan.t <= t else "INIT", t - scan.t, scan.score, odom)
        sampler.add(t, pose, backward, force=not sampler.waypoints, sample=sample)
    # Endpose immer übernehmen
    last_t, last_odom = replay.odom_track[-1]
    scan = scans[j]
    pose = compose(scan.slam, relative(scan.odom, last_odom))
    sampler.add(last_t, pose, sampler.waypoints[-1].backward, force=True,
                sample=PoseSample(pose, scan.status, last_t - scan.t, scan.score, last_odom))
    return sampler.waypoints


def refine_file(recording_file, out_dir: Optional[str] = None) -> Path:
    recording_file = Path(recording_file)
    recording = _load_recording(recording_file)
    waypoints = refined_waypoints(recording)
    if len(waypoints) < 2:
        raise ValueError("Zu wenige Wegpunkte (LIDAR-Frames in der Aufnahme?)")
    session = recording.header.metadata.session_id if recording.header.metadata else recording_file.name.split(".")[0]
    path = RecordedPath(
        session_id=session + REFINED_SUFFIX,
        pose_source=SOURCE_SLAM if recording.lidar_frames else SOURCE_ODOM,
        waypoints=waypoints,
        created=time.time(),
        recording_file=recording_file.name,
    )
    return save_path(path, out_dir or str(recording_file.parent))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Wegdatei nachträglich aus der Aufnahme berechnen (ohne Hardware)")
    parser.add_argument("aufnahme", help="Aufnahme <session>.json.gz")
    parser.add_argument("--ziel", help="Zielverzeichnis (Standard: neben der Aufnahme)")
    args = parser.parse_args(argv)
    target = refine_file(args.aufnahme, args.ziel)
    print(f"Weg gespeichert: {target}")
    print(f"Auswerten: python -m raspirobot.path.analyze {target}")


if __name__ == "__main__":
    main()
