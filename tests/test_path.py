import gzip
import json
import math
import time
from types import SimpleNamespace

import pytest

from raspirobot import config
from raspirobot.hardware.xbox_controller import ControllerInput
from raspirobot.localization import STATUS_LOST, STATUS_OK
from raspirobot.odometry import PoseTracker, integrate_pose_signed
from raspirobot.path.follow_app import STATE_ABORTED, STATE_DRIVING, STATE_FINISHED, PathFollowApp
from raspirobot.path.follower import PHASE_DONE, InputReplayer, PathFollower
from raspirobot.path.teach import TeachApp
from raspirobot.path.waypoints import (
    SOURCE_ODOM,
    SOURCE_SLAM,
    PoseSource,
    RecordedPath,
    Waypoint,
    WaypointSampler,
    find_latest_path,
    load_path,
    save_path,
)
from raspirobot.recording.data_format import ControllerSample

DT = 1.0 / config.CONTROL_LOOP_HZ


def signed(cmd):
    left = -cmd.left_duty if cmd.left_backward else cmd.left_duty
    right = -cmd.right_duty if cmd.right_backward else cmd.right_duty
    return left, right


def drive(pose, cmd, dt=DT):
    return integrate_pose_signed(*pose, *signed(cmd), dt)


def line(x0, y0, x1, y1, theta, step=0.02, backward=False):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [Waypoint(0.0, x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, theta, backward) for i in range(n + 1)]


def l_path():
    """1 m geradeaus, Linksdrehung um 90° auf der Stelle, 0,6 m geradeaus."""
    return line(0, 0, 1.0, 0, 0.0) + [Waypoint(0, 1.0, 0, math.pi / 4)] + line(1.0, 0, 1.0, 0.6, math.pi / 2)


def run_follower(follower, pose, max_steps=20000):
    for _ in range(max_steps):
        step = follower.update(pose)
        if step.finished:
            return pose, step
        pose = drive(pose, step.command)
    raise AssertionError(f"Ziel nicht erreicht, Pose {pose}, Phase {follower.phase}")


# ---------------------------------------------------------------- Odometrie


def test_signed_odometry_turns_in_place_on_tank_turn():
    tracker = PoseTracker(is_moving_forward=lambda: True, use_wheel_directions=True)
    tracker.set_wheel_directions(True, False)
    tracker.set_duty(10.0, 10.0)
    tracker.step(1.0)
    pose = tracker.get_pose()
    assert abs(pose.x) < 1e-9 and abs(pose.y) < 1e-9
    assert math.isclose(pose.theta, 0.5)


def test_default_odometry_unchanged_on_tank_turn():
    tracker = PoseTracker(is_moving_forward=lambda: True)
    tracker.set_wheel_directions(True, False)
    tracker.set_duty(10.0, 10.0)
    tracker.step(1.0)
    assert math.isclose(tracker.get_pose().x, 0.05)


# --------------------------------------------------------------- Wegpunkte


def test_sampler_spacing_and_rotation():
    s = WaypointSampler(spacing_m=0.02, angle_rad=0.1)
    assert s.add(0, (0, 0, 0), False)
    assert not s.add(0, (0.01, 0, 0), False)
    assert s.add(0, (0.03, 0, 0), False)
    assert s.add(0, (0.03, 0, 0.2), False)       # Drehung auf der Stelle
    assert s.add(0, (0.03, 0, 0.2), False, force=True)
    assert len(s.waypoints) == 4


def test_path_file_roundtrip(tmp_path):
    path = RecordedPath("20260101_120000_robot01", SOURCE_SLAM, l_path(), created=1.0)
    target = save_path(path, str(tmp_path))
    assert target.name == "20260101_120000_robot01_path.json.gz"
    loaded = load_path(target)
    assert loaded.pose_source == SOURCE_SLAM
    assert loaded.waypoints == path.waypoints
    assert find_latest_path(str(tmp_path)) == target


def test_load_rejects_other_files(tmp_path):
    f = tmp_path / "x_path.json.gz"
    f.write_bytes(gzip.compress(json.dumps({"format": "anders"}).encode()))
    with pytest.raises(ValueError):
        load_path(f)


def test_pose_source_is_relative_to_origin_and_detects_lost():
    odom = SimpleNamespace(x=1.0, y=2.0, theta=math.pi / 2)
    loc = SimpleNamespace(x=1.0, y=2.0, theta=math.pi / 2, status=STATUS_OK, timestamp=time.time())
    localizer = SimpleNamespace(get_pose=lambda: loc)
    src = PoseSource(lambda: odom, localizer, slam_available=lambda: True)
    assert src.set_origin() == SOURCE_SLAM
    loc.y = 2.5                                  # 0,5 m in Blickrichtung
    (x, y, theta), _, _ = src.get()
    assert math.isclose(x, 0.5) and abs(y) < 1e-9 and abs(theta) < 1e-9
    assert src.is_reliable()
    loc.status = STATUS_LOST
    assert not src.is_reliable()
    loc.status, loc.timestamp = STATUS_OK, time.time() - 5
    assert not src.is_reliable()

    no_lidar = PoseSource(lambda: odom, localizer, slam_available=lambda: False)
    assert no_lidar.set_origin() == SOURCE_ODOM


# ------------------------------------------------------------------ Regler


def test_follower_drives_l_path_with_turn_in_place():
    follower = PathFollower(l_path())
    pose, step = run_follower(follower, (0.0, 0.0, 0.0))
    assert math.hypot(pose[0] - 1.0, pose[1] - 0.6) <= config.FOLLOW_GOAL_TOLERANCE_M + 0.01
    assert abs(pose[2] - math.pi / 2) <= config.FOLLOW_FINAL_HEADING_TOLERANCE_RAD + 0.01
    assert step.phase == PHASE_DONE


def test_follower_converges_from_offset_start():
    follower = PathFollower(line(0, 0, 1.5, 0, 0.0))
    pose, _ = run_follower(follower, (0.0, 0.08, 0.15))
    assert math.hypot(pose[0] - 1.5, pose[1]) <= config.FOLLOW_GOAL_TOLERANCE_M + 0.01


def test_follower_drives_arc():
    r = 0.6
    arc = [Waypoint(0, r * math.sin(a), r - r * math.cos(a), a)
           for a in [i * (math.pi / 2) / 60 for i in range(61)]]
    follower = PathFollower(arc)
    pose = (0.0, 0.0, 0.0)
    max_dev = 0.0
    for _ in range(20000):
        step = follower.update(pose)
        max_dev = max(max_dev, step.deviation_m)
        if step.finished:
            break
        pose = drive(pose, step.command)
    assert step.finished
    assert max_dev < 0.05


def test_follower_drives_backward_segment_backward():
    path = line(0, 0, 0.8, 0, 0.0) + line(0.8, 0, 0.2, 0, 0.0, backward=True)
    follower = PathFollower(path)
    pose = (0.0, 0.0, 0.0)
    saw_backward = False
    for _ in range(20000):
        step = follower.update(pose)
        if step.finished:
            break
        if step.command.left_backward and step.command.right_backward:
            saw_backward = True
            assert step.phase != "DREHEN"
        pose = drive(pose, step.command)
    assert step.finished and saw_backward
    assert math.hypot(pose[0] - 0.2, pose[1]) <= config.FOLLOW_GOAL_TOLERANCE_M + 0.01
    assert abs(pose[2]) < 0.2                   # nicht umgedreht


def test_follower_does_not_shortcut_loop_back_to_start():
    # Hin und auf parallelem Weg 0,3 m daneben zurück: Ziel liegt nahe am Start.
    path = line(0, 0, 1.0, 0, 0.0) + [Waypoint(0, 1.0, 0, math.pi / 2)] + line(1.0, 0, 1.0, 0.3, math.pi / 2) \
        + [Waypoint(0, 1.0, 0.3, math.pi)] + line(1.0, 0.3, 0.0, 0.3, math.pi)
    follower = PathFollower(path)
    pose, _ = run_follower(follower, (0.0, 0.0, 0.0))
    assert follower.index == len(path) - 1
    assert math.hypot(pose[0], pose[1] - 0.3) <= config.FOLLOW_GOAL_TOLERANCE_M + 0.01


def test_follower_duty_within_limits():
    follower = PathFollower(l_path(), drive_duty=50.0, turn_duty=50.0)
    step = follower.update((0.0, 0.05, 0.3))
    assert step.command.left_duty <= config.MAX_DUTY_CYCLE and step.command.right_duty <= config.MAX_DUTY_CYCLE


def test_input_replayer_reproduces_controller_drive():
    t0 = 1000.0
    samples = []
    for i in range(150):
        tank = 50 <= i < 100
        samples.append(ControllerSample(
            timestamp=t0 + i * 0.02, left_trigger=1.0 if not tank else -1.0, right_trigger=1.0,
            buttons={"A": tank, "B": False}, direction={"forward": 1.0, "tank_turn": 1.0 if tank else 0.0},
        ))
    rp = InputReplayer(samples)
    assert rp.command_at(0.0).left_duty == config.MAX_DUTY_CYCLE
    turn = rp.command_at(1.5)
    assert turn.left_backward and not turn.right_backward
    assert rp.command_at(rp.duration + 0.1) is None


# ------------------------------------------------------------------- Apps


def test_teach_app_writes_path_with_recording(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    app = TeachApp()
    app.recorder.setup(str(tmp_path))
    assert app.recorder.start()
    state = ControllerInput(trigger_left=1.0, trigger_right=1.0)
    app.pose.set_duty(10.0, 10.0)
    for _ in range(50):
        app.pose.step(0.02)                      # 0,05 m/s
        app.recorder.add_sample(state)
    app.recorder.start_time -= config.MIN_RECORDING_DURATION_SEC
    assert app.recorder.stop()

    assert len(list(tmp_path.glob("*_robot01.json.gz"))) == 1
    path = load_path(app.recorder.path_file)
    assert path.pose_source == SOURCE_ODOM
    assert path.waypoints[0].x == 0.0
    assert math.isclose(path.waypoints[-1].x, 0.05, abs_tol=1e-3)
    assert len(path.waypoints) >= 3


def test_short_teach_recording_writes_no_path(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    app = TeachApp()
    app.recorder.setup(str(tmp_path))
    app.recorder.start()
    app.recorder.stop()
    assert not list(tmp_path.glob("*_path.json.gz"))


def make_follow_app(tmp_path, waypoints):
    target = save_path(RecordedPath("s1", SOURCE_ODOM, waypoints), str(tmp_path))
    app = PathFollowApp(target)
    app.load()
    app.motors.setup_pwm()
    app.controller.connect()
    app.running = True
    return app


def simulate(app, steps):
    for _ in range(steps):
        app.control_step()
        if app.state != STATE_DRIVING and app.state != "WARTEN":
            return
        app.pose.step(DT)


def test_follow_app_waits_for_a_then_reaches_goal(tmp_path):
    app = make_follow_app(tmp_path, l_path())
    simulate(app, 20)
    assert app.state == "WARTEN" and app.motors.pwm_left.duty == 0
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    simulate(app, 20000)
    assert app.state == STATE_FINISHED, app.result
    assert app.motors.pwm_left.duty == 0 and app.motors.pwm_right.duty == 0
    pose = app.pose.get_pose()
    assert math.hypot(pose.x - 1.0, pose.y - 0.6) < 0.07


def test_follow_app_emergency_stop_on_b(tmp_path):
    app = make_follow_app(tmp_path, l_path())
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    simulate(app, 50)
    assert app.motors.pwm_left.duty > 0
    app.controller.joystick.buttons[config.BUTTON_B] = 1
    app.control_step()
    assert app.state == STATE_ABORTED and not app.running
    assert app.motors.pwm_left.duty == 0 and app.motors.pwm_right.duty == 0


def test_follow_app_emergency_stop_on_trigger(tmp_path):
    app = make_follow_app(tmp_path, l_path())
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    simulate(app, 5)
    app.controller.joystick.axes[config.AXIS_RIGHT_TRIGGER] = 0.8
    app.control_step()
    assert app.state == STATE_ABORTED
    assert app.motors.pwm_right.duty == 0


def test_follow_app_stops_for_obstacle_and_aborts_after_timeout(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "FOLLOW_OBSTACLE_TIMEOUT_SEC", 0.05)
    app = make_follow_app(tmp_path, l_path())
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    simulate(app, 5)
    app.ultrasonic.distance_left = 5.0
    app.control_step()
    assert app.motors.pwm_left.duty == 0 and app.state == STATE_DRIVING
    time.sleep(0.1)
    app.control_step()
    assert app.state == STATE_ABORTED and "Hindernis" in app.result


def test_follow_app_aborts_when_far_off_path(tmp_path):
    app = make_follow_app(tmp_path, l_path())
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    app.control_step()
    with app.pose._lock:
        app.pose._y = 1.0                        # Roboter wurde weggetragen
    app.control_step()
    assert app.state == STATE_ABORTED and "Weg" in app.result


def test_follow_app_aborts_on_controller_loss(tmp_path):
    app = make_follow_app(tmp_path, l_path())
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    simulate(app, 5)
    app.controller.joystick.quit()
    app.control_step()
    assert app.state == STATE_ABORTED
    assert app.motors.pwm_left.duty == 0


def test_follow_app_waits_for_slam_when_path_was_recorded_with_slam(tmp_path):
    target = save_path(RecordedPath("s2", SOURCE_SLAM, l_path()), str(tmp_path))
    app = PathFollowApp(target)
    app.load()
    app.motors.setup_pwm()
    app.controller.connect()
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    app.control_step()
    assert app.state == "WARTEN"                 # kein LIDAR: SLAM nicht bereit


def test_input_mode_replays_recording(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    teach = TeachApp()
    teach.recorder.setup(str(tmp_path))
    teach.recorder.start()
    for i in range(10):
        teach.recorder.add_sample(ControllerInput(trigger_left=1.0, trigger_right=1.0))
        teach.recorder.data.controller_samples[-1].timestamp = 1000.0 + i * 0.02
    teach.recorder.start_time -= config.MIN_RECORDING_DURATION_SEC
    teach.recorder.stop()
    recording = next(f for f in tmp_path.glob("*.json.gz") if not f.name.endswith("_path.json.gz"))

    app = PathFollowApp(recording, mode="eingaben")
    app.load()
    app.motors.setup_pwm()
    app.controller.connect()
    app.controller.joystick.buttons[config.BUTTON_A] = 1
    app.control_step()
    app.control_step()
    assert app.motors.pwm_left.duty == config.MAX_DUTY_CYCLE
    time.sleep(0.25)
    app.control_step()
    assert app.state == STATE_FINISHED and app.motors.pwm_left.duty == 0
