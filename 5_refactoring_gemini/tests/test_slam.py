import numpy as np
import pytest
import math
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from slam_engine import SLAMEngine
from data_models import RobotPose

def test_icp_translation():
    slam = SLAMEngine(max_correspondence_dist=2.0)

    # Create reference map (L-shape to fix translation in both X and Y)
    # L-shape: (0,0) to (5,0) and (0,0) to (0,5)
    points = []
    for i in range(6):
        points.append([float(i), 0.0])
        points.append([0.0, float(i)])
    slam.reference_points = np.array(points, dtype=np.float32)

    # Create local scan shifted by (-1.0, -1.0)
    # If robot is at (1.0, 1.0), it sees the map shifted by (-1.0, -1.0)
    local_scan_points = []
    for i in range(6):
        local_scan_points.append([float(i) - 1.0, -1.0])
        local_scan_points.append([-1.0, float(i) - 1.0])
    local_scan = np.array(local_scan_points, dtype=np.float32)

    initial_pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)

    new_pose = slam.icp(local_scan, initial_pose)

    assert math.isclose(new_pose.x, 1.0, abs_tol=0.1)
    assert math.isclose(new_pose.y, 1.0, abs_tol=0.1)
    assert math.isclose(new_pose.theta, 0.0, abs_tol=0.1)

def test_icp_rotation():
    slam = SLAMEngine(max_correspondence_dist=1.0)

    # L-shape: (0,0) to (5,0) and (0,0) to (0,5)
    points = []
    for i in range(6):
        points.append([float(i), 0.0])
        points.append([0.0, float(i)])
    slam.reference_points = np.array(points, dtype=np.float32)

    # Rotate by small angle e.g. 10 deg (0.17 rad)
    theta = 0.17
    c = math.cos(theta)
    s = math.sin(theta)
    R = np.array([[c, -s], [s, c]])

    # Local scan is rotated relative to robot frame?
    # No, local scan is what robot sees.
    # If robot is rotated by -theta, then local scan (aligned with world) is rotated by +theta in robot frame?
    # No.
    # If Map is at 0. Robot is at -theta.
    # Robot sees Map rotated by +theta.
    # So Local Scan = Map * R(theta).

    local_scan = slam.reference_points @ R.T

    # Initial guess: 0,0,0
    initial_pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)

    # ICP should find that robot is at -theta to align local_scan (rotated +theta) to map (0).
    # Wait.
    # Map (Global) = T_robot * Local.
    # Map = R_robot * Local + t.
    # Local = Map * R(theta).
    # Map = R_robot * (Map * R(theta)) + t.
    # For Map = Map, we need R_robot * R(theta) = I => R_robot = R(theta)^-1 = R(-theta).
    # So Robot Theta should be -theta (-0.17).

    new_pose = slam.icp(local_scan, initial_pose)

    assert math.isclose(new_pose.theta, -theta, abs_tol=0.05)
    assert math.isclose(new_pose.x, 0.0, abs_tol=0.1)
    assert math.isclose(new_pose.y, 0.0, abs_tol=0.1)

def test_update_map():
    slam = SLAMEngine()
    pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0.0)
    scan = [(0.0, 1000.0)]

    slam.update_map(pose, scan)

    assert len(slam.reference_points) == 1
    assert math.isclose(slam.reference_points[0][0], 1.0)
    assert slam.grid[200, 220] > 0
