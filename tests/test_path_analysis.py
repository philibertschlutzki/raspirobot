import gzip
import json
import math
import time
from types import SimpleNamespace

import numpy as np

from raspirobot import config
from raspirobot.drive import trigger_to_duty_cycle
from raspirobot.localization import STATUS_OK, Localizer
from raspirobot.path import analyze
from raspirobot.path.waypoints import (
    PATH_FORMAT,
    SOURCE_SLAM,
    PoseSource,
    RecordedPath,
    Waypoint,
    WaypointSampler,
    densify,
    load_path,
    save_path,
)
from raspirobot.recorder import create_recording_data
from raspirobot.recording.data_format import ControllerSample, LidarFrame
from tests.sim import FakeOdom, room, simulate_scan

# ------------------------------------------------------------ Wegpunkte


def test_densify_fills_gaps_and_keeps_turns():
    wps = [Waypoint(0.0, 0.0, 0.0, 0.0), Waypoint(1.0, 0.10, 0.0, 0.0),
           Waypoint(2.0, 0.10, 0.0, math.pi / 2), Waypoint(3.0, 0.10, 0.05, math.pi / 2, backward=True)]
    out = densify(wps, 0.02)
    assert out[0] == wps[0] and out[-1] == wps[-1]
    assert all(w in out for w in wps)                     # nichts fällt weg, auch die Drehung nicht
    gaps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(out, out[1:])]
    assert max(gaps) <= 0.02 + 1e-9
    assert len(out) == 4 + 4 + 2                          # 10 cm -> 4 neue, 5 cm -> 2 neue
    inserted = [w for w in out[6:] if w not in wps]
    assert all(w.backward for w in inserted)              # Richtung des folgenden Wegpunkts
    assert all(w.status == "" and w.odom_x is None for w in inserted)


def test_path_v2_roundtrip_with_diagnostics_and_v1_still_loads(tmp_path):
    wps = [Waypoint(0.0, 0.0, 0.0, 0.0, False, "OK", 0.81, 0.0, 0.0, 0.0),
           Waypoint(1.0, 0.1, 0.0, 0.0, False, "LOST", 0.32, 0.12, 0.01, 0.02)]
    target = save_path(RecordedPath("s", SOURCE_SLAM, wps, 1.0, "s.json.gz"), str(tmp_path))
    assert load_path(target).waypoints == wps

    v1 = tmp_path / "alt_path.json.gz"
    v1.write_bytes(gzip.compress(json.dumps({
        "format": PATH_FORMAT, "version": 1, "session_id": "alt", "pose_source": "odom",
        "waypoints": [{"t": 0, "x": 0, "y": 0, "theta": 0, "backward": False},
                      {"t": 1, "x": 0.1, "y": 0, "theta": 0, "backward": False}],
    }).encode()))
    old = load_path(v1)
    assert len(old.waypoints) == 2 and old.waypoints[1].status == "" and old.waypoints[1].score is None


def _slam_setup(slam_pose, scan_odom, epoch=0):
    loc = SimpleNamespace(x=slam_pose[0], y=slam_pose[1], theta=slam_pose[2], status=STATUS_OK,
                          timestamp=time.time(), score=0.8, odom=scan_odom, odom_epoch=epoch)
    odom = SimpleNamespace(x=scan_odom[0], y=scan_odom[1], theta=scan_odom[2], epoch=epoch)
    return loc, odom


def test_pose_source_extrapolates_slam_pose_with_odometry_between_scans():
    # SLAM sieht den Roboter um 90° gedreht, die Odometrie bei 0°: die Bewegung seit
    # dem Scan muss im SLAM-Rahmen ankommen.
    loc, odom = _slam_setup((1.0, 1.0, math.pi / 2), (0.0, 0.0, 0.0))
    src = PoseSource(lambda: odom, SimpleNamespace(get_pose=lambda: loc), slam_available=lambda: True)
    assert src.set_origin() == SOURCE_SLAM
    odom.x = 0.05                                          # 5 cm vorwärts, noch kein neuer Scan
    s = src.sample()
    assert math.isclose(s.pose[0], 0.05, abs_tol=1e-9) and abs(s.pose[1]) < 1e-9
    assert math.isclose(s.odom[0], 0.05) and s.score == 0.8 and s.status == STATUS_OK

    src.extrapolate = False
    assert abs(src.get()[0][0]) < 1e-9                     # ohne Fortschreibung steht die Pose still

    src.extrapolate = True
    odom.epoch = 1                                         # Odometrie zurückgesetzt: nicht fortschreiben
    assert abs(src.get()[0][0]) < 1e-9


def test_sampler_stores_diagnostics():
    loc, odom = _slam_setup((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    src = PoseSource(lambda: odom, SimpleNamespace(get_pose=lambda: loc), slam_available=lambda: True)
    src.set_origin()
    sampler = WaypointSampler()
    s = src.sample()
    sampler.add(0.0, s.pose, False, force=True, sample=s)
    w = sampler.waypoints[0]
    assert (w.status, w.score, w.odom_x, w.odom_y, w.odom_theta) == (STATUS_OK, 0.8, 0.0, 0.0, 0.0)


def test_localizer_publishes_odometry_of_the_scan():
    segs = room()
    loc = Localizer(lambda: FakeOdom(0.0, 0.0, 0.0))
    res = loc.process(time.time(), simulate_scan(segs, (0.0, 0.0, 0.0)), FakeOdom(0.3, 0.1, 0.2, epoch=4))
    assert res.odom == (0.3, 0.1, 0.2) and res.odom_epoch == 4


# ------------------------------------------------------------ Auswertung


def _synthetic_recording(seconds=6.0, lidar_hz=8.0, beams=360):
    """Geradeausfahrt mit vollem Trigger; LIDAR-Frames aus der Simulation an der Odometrie-Pose."""
    rng = np.random.default_rng(1)
    segs = room()
    data = create_recording_data("s1", True)
    t0 = 1000.0
    speed = trigger_to_duty_cycle(1.0) / 100.0 * config.ODOMETRY_MAX_SPEED_M_S
    n = int(seconds * config.RECORDING_SAMPLE_RATE_HZ)
    for i in range(n + 1):
        t = t0 + i / config.RECORDING_SAMPLE_RATE_HZ
        data.add_controller_sample(ControllerSample(t, 1.0, 1.0, {"A": False, "B": False, "X": False, "Y": False},
                                                    {"forward": 1.0, "tank_turn": 0.0}))
    for k in range(int(seconds * lidar_hz)):
        t = t0 + k / lidar_hz
        scan = simulate_scan(segs, (speed * (t - t0), 0.0, 0.0), rng=rng, beams=beams)
        data.add_lidar_frame(LidarFrame(t, scan))
    return data, speed


def test_lidar_stats_detects_rate_and_motor_speed():
    data, _ = _synthetic_recording(lidar_hz=8.0, beams=625)
    ls = analyze.lidar_stats(data)
    assert math.isclose(ls.rate_hz, 8.0, rel_tol=0.01)
    assert math.isclose(ls.motor_hz_from_points, 8.0, rel_tol=0.05)
    assert ls.long_intervals == 0
    assert math.isclose(ls.controller_rate_hz, config.RECORDING_SAMPLE_RATE_HZ, rel_tol=0.01)

    del data.lidar_frames[10::3]                           # jeder dritte Frame fehlt
    assert analyze.lidar_stats(data).long_intervals > 0


def test_replay_tracks_straight_drive_and_reports():
    data, speed = _synthetic_recording()
    rp = analyze.replay_slam(data)
    done = rp.processed()
    assert len(done) == len(data.lidar_frames)
    assert not rp.lost_intervals
    end = done[-1]
    assert math.isclose(end.odom[0], speed * end.t, rel_tol=0.02)
    assert math.hypot(end.slam[0] - end.odom[0], end.slam[1] - end.odom[1]) < 0.05

    wps = [Waypoint(p.t, *p.slam) for p in done]
    text = analyze.report(RecordedPath("s", SOURCE_SLAM, wps), data)
    assert "LIDAR-Takt" in text and "LOST: nie" in text and "Wegdatei Version 1" in text
    # Mit Fortschreibung zwischen den Scans liegen die Wegpunkte im Soll-Abstand.
    length = speed * done[-1].t
    assert analyze.resample_waypoints(rp) >= 0.8 * length / config.PATH_WAYPOINT_SPACING_M


def test_replay_cpu_factor_skips_frames():
    data, _ = _synthetic_recording(seconds=3.0)
    rp = analyze.replay_slam(data, cpu_factor=1e6)
    assert len(rp.processed()) == 1 and len(rp.points) == len(data.lidar_frames)


def test_main_reads_files(tmp_path, capsys):
    from raspirobot.recording.storage import PathRecordingStorageManager

    data, _ = _synthetic_recording(seconds=2.0)
    storage = PathRecordingStorageManager(str(tmp_path), backup_enabled=False)
    storage.save_json(data, "s1")
    wps = [Waypoint(0.0, 0.0, 0.0, 0.0, False, "OK", 0.9, 0.0, 0.0, 0.0),
           Waypoint(1.0, 0.2, 0.0, 0.0, False, "OK", 0.9, 0.18, 0.0, 0.0)]
    target = save_path(RecordedPath("s1", SOURCE_SLAM, wps, 1.0, "s1.json.gz"), str(tmp_path))
    analyze.main([str(target)])
    out = capsys.readouterr().out
    assert "2 Wegpunkte" in out and "Abstand SLAM ↔ Odometrie: am Ende 0.02 m" in out
    assert "SLAM nachgerechnet" in out


# ------------------------------------------------- Neue Karte ab Aufnahmestart


def test_localizer_reset_starts_a_fresh_map():
    segs = room()
    loc = Localizer(lambda: FakeOdom(0.0, 0.0, 0.0))
    scan = simulate_scan(segs, (0.0, 0.0, 0.0))
    loc.process(time.time(), scan, FakeOdom(0.0, 0.0, 0.0))
    assert loc.get_pose().status == STATUS_OK and loc.grid.scans_integrated == 1

    loc.reset()
    assert loc.get_pose().status == "INIT"
    res = loc.process(time.time(), scan, FakeOdom(5.0, 5.0, 1.0))   # Odometrie lief weiter, Karte beginnt neu
    assert res.status == STATUS_OK and (res.x, res.y, res.theta) == (0.0, 0.0, 0.0)
    assert loc.grid.scans_integrated == 1


def test_pose_source_restart_slam_waits_for_first_scan_then_uses_it():
    segs = room()
    odom = FakeOdom(3.0, 0.0, 0.0)
    loc = Localizer(lambda: odom)
    scan = simulate_scan(segs, (0.0, 0.0, 0.0))
    loc.process(time.time(), scan, odom)                        # alte Karte, Roboter ist inzwischen weitergefahren
    src = PoseSource(lambda: odom, loc, slam_available=lambda: True)

    assert src.restart_slam() == SOURCE_SLAM
    s = src.sample()
    assert s.status == "INIT" and s.pose == (0.0, 0.0, 0.0)     # wartet auf den ersten Scan, blockiert nicht
    loc.process(time.time(), scan, odom)
    s = src.sample()
    assert s.status == STATUS_OK and s.pose == (0.0, 0.0, 0.0)  # Ursprung = Pose des ersten Scans
    odom.x = 3.1
    assert math.isclose(src.sample().pose[0], 0.1, abs_tol=1e-6)   # Odometrie schreibt fort


def test_pose_source_restart_slam_falls_back_to_odometry_without_scan():
    odom = FakeOdom(1.0, 0.0, 0.0)
    loc = Localizer(lambda: odom)
    src = PoseSource(lambda: odom, loc, slam_available=lambda: True)
    assert src.restart_slam(timeout=0.0) == SOURCE_SLAM
    time.sleep(0.01)
    odom.x = 1.2
    s = src.sample()
    assert src.source == "odom" and s.status == "ODOM" and s.pose[0] == 0.0
    odom.x = 1.5
    assert math.isclose(src.sample().pose[0], 0.3)

    no_lidar = PoseSource(lambda: odom, loc, slam_available=lambda: False)
    assert no_lidar.restart_slam() is None
