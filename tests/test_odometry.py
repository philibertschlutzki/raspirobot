import math

from raspirobot.odometry import PoseTracker, integrate_pose


def test_straight_forward():
    x, y, theta = integrate_pose(0, 0, 0, 10.0, 10.0, True, 1.0)
    assert math.isclose(x, 0.05) and y == 0 and theta == 0


def test_straight_backward():
    x, _, _ = integrate_pose(0, 0, 0, 10.0, 10.0, False, 1.0)
    assert math.isclose(x, -0.05)


def test_turn_left_when_right_faster():
    _, _, theta = integrate_pose(0, 0, 0, 0.0, 10.0, True, 1.0)
    assert math.isclose(theta, 0.25)


def test_tracker_reset():
    tracker = PoseTracker(is_moving_forward=lambda: True)
    tracker.set_duty(10.0, 10.0)
    tracker.step(1.0)
    assert tracker.get_pose().x > 0
    tracker.reset()
    assert tracker.get_pose().x == 0
