import math

import numpy as np

from raspirobot import config
from raspirobot.localization.geometry import compose, inverse, relative, scan_to_points, wrap_angle


def test_compose_inverse_roundtrip():
    a = (1.0, -2.0, 2.5)
    x, y, th = compose(a, inverse(a))
    assert math.isclose(x, 0, abs_tol=1e-12) and math.isclose(y, 0, abs_tol=1e-12) and math.isclose(th, 0, abs_tol=1e-12)


def test_relative_is_inverse_of_compose():
    a, delta = (0.5, 1.0, 1.0), (0.3, -0.1, 0.4)
    b = compose(a, delta)
    rel = relative(a, b)
    assert all(math.isclose(r, d, abs_tol=1e-12) for r, d in zip(rel, delta))


def test_wrap_angle_range():
    assert math.isclose(float(wrap_angle(3 * math.pi)), -math.pi) or math.isclose(float(wrap_angle(3 * math.pi)), math.pi)
    assert -math.pi <= float(wrap_angle(7.0)) < math.pi


def test_left_object_has_positive_y():
    # Der Sensor zählt im Uhrzeigersinn: 270 Grad am Sensor = links vom Roboter.
    pts = scan_to_points([(15, 270.0, 1000.0)])
    assert pts.shape == (1, 2)
    assert math.isclose(pts[0, 0], config.LIDAR_OFFSET_X, abs_tol=1e-9)
    assert math.isclose(pts[0, 1], 1.0 + config.LIDAR_OFFSET_Y, abs_tol=1e-9)


def test_front_object_is_ahead_of_lidar_mount():
    pts = scan_to_points([(15, 0.0, 2000.0)])
    assert math.isclose(pts[0, 0], 2.0 + config.LIDAR_OFFSET_X)
    assert math.isclose(pts[0, 1], config.LIDAR_OFFSET_Y, abs_tol=1e-9)


def test_invalid_points_are_dropped():
    scan = [
        (15, 10.0, 0.0),                       # Distanz 0 = ungültig
        (0, 20.0, 1000.0),                     # Qualität 0
        (15, 30.0, 50.0),                      # unter Mindestentfernung
        (15, 40.0, 50000.0),                   # über Reichweite
        (15, float("nan"), 1000.0),
        (15, 50.0, float("inf")),
        (15, 60.0, 1500.0),                    # gültig
    ]
    assert scan_to_points(scan).shape == (1, 2)


def test_empty_and_malformed_scans():
    assert scan_to_points([]).shape == (0, 2)
    assert scan_to_points(iter([])).shape == (0, 2)
