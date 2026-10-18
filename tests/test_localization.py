import threading
import time

import numpy as np
import pytest

import sim
from raspirobot import config
from raspirobot.localization import STATUS_DEGRADED, STATUS_LOST, STATUS_OK, Localizer
from raspirobot.localization.geometry import relative

LOOP = [(0.4, 0.0)] * 40 + [(0.0, 0.8)] * 20 + [(0.4, 0.0)] * 30 + [(0.0, -0.8)] * 25 + [(0.35, 0.2)] * 40


class Harness:
    """Fährt eine simulierte Strecke und vergleicht die Lokalisierung mit der wahren Pose."""

    def __init__(self, segments, seed=1, **drive_kwargs):
        self.rng = np.random.default_rng(seed)
        self.segments = segments
        self.drive = sim.Drive(self.rng, **drive_kwargs)
        self.epoch = 0
        self.localizer = Localizer(lambda: self.drive.odom_obj(self.epoch))
        self.origin = None   # wahre Pose beim ersten Scan = Ursprung des Kartenrahmens
        self.t = 0.0
        self.last = None

    def step(self, v, omega, scan=None):
        self.drive.step(v, omega)
        if self.origin is None:
            self.origin = self.drive.true
        if scan is None:
            scan = sim.simulate_scan(self.segments, self.drive.true, self.rng)
        self.t += 0.1
        self.last = self.localizer.process(self.t, scan, self.drive.odom_obj(self.epoch))
        return self.last

    def error(self):
        truth = relative(self.origin, self.drive.true)
        return sim.pose_error((self.last.x, self.last.y, self.last.theta), truth)

    def odom_error(self):
        truth = relative(self.origin, self.drive.true)
        odom = relative(self.origin_odom, self.drive.odom)
        return sim.pose_error(odom, truth)

    @property
    def origin_odom(self):
        return (0.0, 0.0, 0.0)


def test_tracks_loop_much_better_than_odometry():
    h = Harness(sim.room())
    worst_xy = worst_th = 0.0
    for v, w in LOOP:
        h.step(v, w)
        exy, eth = h.error()
        worst_xy, worst_th = max(worst_xy, exy), max(worst_th, eth)
    assert worst_xy < 0.05 and worst_th < 0.04
    assert h.last.status == STATUS_OK and h.last.matched and h.last.score > 0.9
    # Die Odometrie allein driftet auf derselben Strecke deutlich stärker.
    assert h.odom_error()[0] > 3 * h.error()[0]


def test_localization_survives_odometry_reset():
    h = Harness(sim.room())
    for v, w in LOOP[:60]:
        h.step(v, w)
    before = h.error()[0]
    # Aufnahme startet: Odometrie wird auf 0 gesetzt, die wahre Pose bleibt.
    h.epoch += 1
    h.drive.odom = (0.0, 0.0, 0.0)
    for v, w in LOOP[60:120]:
        h.step(v, w)
        assert h.error()[0] < 0.10
    assert h.last.status == STATUS_OK and before < 0.10


def test_relocalizes_after_being_moved():
    h = Harness(sim.room())
    for v, w in LOOP[:100]:
        h.step(v, w)
    # Roboter wird hochgehoben und versetzt, die Odometrie merkt nichts.
    h.drive.true = sim.compose(h.drive.true, (0.55, -0.4, 0.5))
    statuses = []
    for _ in range(40):
        statuses.append(h.step(0.0, 0.0).status)
        if statuses[-1] == STATUS_OK and STATUS_LOST in statuses:
            break
    assert STATUS_LOST in statuses, "Verlust muss erkannt werden"
    assert h.last.status == STATUS_OK
    assert h.error()[0] < 0.06 and h.error()[1] < 0.03


def test_wide_window_recovers_moderate_odometry_jump_without_losing():
    h = Harness(sim.room())
    for v, w in LOOP[:100]:
        h.step(v, w)
    h.drive.true = sim.compose(h.drive.true, (0.25, 0.0, 0.2))
    h.step(0.0, 0.0)
    assert h.last.status == STATUS_OK
    assert h.error()[0] < 0.08


def test_featureless_corridor_keeps_odometry_instead_of_inventing_motion():
    h = Harness(sim.corridor(), scale_error=0.0, turn_error=0.0, noise=0.0, noise_theta=0.0)
    h.drive.noise = 0.0
    for _ in range(5):
        h.step(0.0, 0.0)
    for _ in range(60):
        h.step(0.4, 0.0)
    # Entlang des Flurs fehlen Merkmale: kein Wegdriften, Pose folgt der (hier fehlerfreien) Odometrie.
    assert h.error()[0] < 0.02
    assert h.last.status == STATUS_DEGRADED and not h.last.matched


def test_featureless_corridor_is_never_worse_than_odometry():
    h = Harness(sim.corridor(), seed=3)
    for _ in range(5):
        h.step(0.0, 0.0)
    for _ in range(80):
        h.step(0.4, 0.0)
    assert h.error()[0] <= h.odom_error()[0] + 0.02


def test_empty_scan_keeps_prediction_and_recovers():
    h = Harness(sim.room())
    for v, w in LOOP[:30]:
        h.step(v, w)
    h.step(0.4, 0.0, scan=[])
    assert h.last.status == STATUS_DEGRADED and not h.last.matched and h.last.n_points == 0
    for v, w in LOOP[31:50]:
        h.step(v, w)
    assert h.last.status == STATUS_OK and h.error()[0] < 0.08


def test_garbage_scans_do_not_corrupt_map():
    h = Harness(sim.room())
    for v, w in LOOP[:40]:
        h.step(v, w)
    occupied = h.localizer.grid.occupied_count
    scans_integrated = h.localizer.grid.scans_integrated
    rng = np.random.default_rng(5)
    for _ in range(3):
        garbage = [(15.0, float(a), float(d)) for a, d in zip(np.arange(360), rng.uniform(300, 9000, 360))]
        h.step(0.0, 0.0, scan=garbage)
        assert not h.last.matched
        assert h.last.status in (STATUS_DEGRADED, STATUS_LOST)
    assert h.localizer.grid.scans_integrated == scans_integrated
    assert h.localizer.grid.occupied_count == occupied


def test_stationary_robot_does_not_drift():
    h = Harness(sim.room())
    for _ in range(60):
        h.step(0.0, 0.0)
    assert h.error()[0] < 0.02 and h.error()[1] < 0.01


def test_first_scan_defines_origin():
    h = Harness(sim.room())
    result = h.step(0.0, 0.0)
    assert (result.x, result.y, result.theta) == (0.0, 0.0, 0.0)
    assert result.status == STATUS_OK and h.localizer.grid.scans_integrated == 1


def test_worker_thread_processes_latest_scan_only():
    h = Harness(sim.room())
    loc = h.localizer
    stop = threading.Event()
    thread = threading.Thread(target=loc.run, args=(lambda: not stop.is_set(),), daemon=True)
    for i in range(5):  # Worker läuft noch nicht: nur der letzte Scan darf übrig bleiben
        h.drive.step(0.0, 0.0)
        loc.offer_scan(i * 0.1, sim.simulate_scan(h.segments, h.drive.true, h.rng), i)
    thread.start()
    deadline = time.time() + 5
    while loc.get_pose().scans_processed < 1 and time.time() < deadline:
        time.sleep(0.01)
    stop.set()
    thread.join(timeout=2)
    assert not thread.is_alive()
    assert loc.get_pose().scans_processed == 1
    assert loc.get_pose().status == STATUS_OK


def test_worker_survives_exception_in_processing():
    calls = {"n": 0}

    def flaky_odom():
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("Odometrie kaputt")
        return sim.FakeOdom(0.0, 0.0, 0.0)

    loc = Localizer(flaky_odom)
    stop = threading.Event()
    with pytest.raises(RuntimeError):
        loc.offer_scan(0.0, [], 0)   # Fehler beim Abholen der Odometrie erreicht den Aufrufer
    scan = sim.simulate_scan(sim.room(), (0.0, 0.0, 0.0), np.random.default_rng(0))
    thread = threading.Thread(target=loc.run, args=(lambda: not stop.is_set(),), daemon=True)
    thread.start()
    loc.offer_scan(0.1, scan, 1)
    deadline = time.time() + 5
    while loc.get_pose().scans_processed < 1 and time.time() < deadline:
        time.sleep(0.01)
    stop.set()
    thread.join(timeout=2)
    assert loc.get_pose().status == STATUS_OK


def test_save_map_after_run(tmp_path):
    h = Harness(sim.room())
    for v, w in LOOP[:20]:
        h.step(v, w)
    npz, pgm = h.localizer.save_map(str(tmp_path / "map"))
    assert npz.exists() and pgm.stat().st_size > 1000
