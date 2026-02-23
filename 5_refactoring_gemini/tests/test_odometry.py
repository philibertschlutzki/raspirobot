import math
import pytest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from odometry import OdometryEngine, RobotPose

def test_odometry_straight():
    odo = OdometryEngine(wheel_base=0.2, max_speed=1.0)
    # v = 1.0 m/s, dt = 1.0s
    pose = odo.update(1.0, 1.0, current_time=odo.last_update_time + 1.0)

    assert math.isclose(pose.x, 1.0, rel_tol=1e-5)
    assert math.isclose(pose.y, 0.0, rel_tol=1e-5)
    assert math.isclose(pose.theta, 0.0, rel_tol=1e-5)

def test_odometry_turn():
    odo = OdometryEngine(wheel_base=0.2, max_speed=1.0)
    # Rotate in place: Left -1, Right 1
    # v = 0
    # omega = (1 - (-1)) / 0.2 = 2 / 0.2 = 10 rad/s
    # dt = 0.1s => theta change = 1.0 rad

    pose = odo.update(-1.0, 1.0, current_time=odo.last_update_time + 0.1)

    assert math.isclose(pose.x, 0.0, rel_tol=1e-5)
    assert math.isclose(pose.y, 0.0, rel_tol=1e-5)
    assert math.isclose(pose.theta, 1.0, rel_tol=1e-5)

def test_odometry_normalization():
    odo = OdometryEngine(wheel_base=0.2, max_speed=1.0)
    # Set initial theta close to PI
    odo.theta = math.pi - 0.1

    # Add small positive rotation (0.2 rad)
    # New theta should be PI + 0.1, which wraps to -PI + 0.1
    # omega = 10 rad/s
    # dt = 0.02s => 0.2 rad

    pose = odo.update(-1.0, 1.0, current_time=odo.last_update_time + 0.02)

    expected = math.pi - 0.1 + 0.2
    normalized_expected = (expected + math.pi) % (2 * math.pi) - math.pi

    assert math.isclose(pose.theta, normalized_expected, rel_tol=1e-5)
    assert pose.theta < 0 # Should have wrapped
