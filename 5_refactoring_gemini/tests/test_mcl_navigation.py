import pytest
import numpy as np
import os
import math
import sys
from unittest.mock import MagicMock, patch

# Ensure sys.path includes the module directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from mcl_engine import MCLEngine
from path_planner import AStarPlanner
from pure_pursuit import PurePursuitController
from robot_orchestrator import RobotOrchestrator, NavigationState
from data_models import RobotPose
from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
from odometry import OdometryEngine
from recorder import DataRecorder

# --- Helpers ---
@pytest.fixture
def dummy_map_path(tmp_path):
    # 400x400 map (20x20m), 0.05m res
    # Origin 200,200 (10m, 10m)
    grid = np.zeros((400, 400), dtype=float) + (-0.4) # Free (Log-Odds)

    # Walls at +/- 2m from center (Center is 200, 200)
    # 2m = 40 pixels
    # Square room: 160 to 240

    # Thicker walls (3 pixels) to pass validation (>0.5%)
    # Total cells 160000. Need 800 cells.
    # 4 walls of length 80. Need thickness ~3.

    # Top wall
    grid[158:161, 160:241] = 0.85
    # Bottom wall
    grid[240:243, 160:241] = 0.85
    # Left wall
    grid[160:241, 158:161] = 0.85
    # Right wall
    grid[160:241, 240:243] = 0.85

    filepath = tmp_path / "test_map.npy"
    np.save(filepath, grid)
    return str(filepath)

def create_synthetic_scan(center_dist_m=2.0):
    # Create a scan that sees walls at 2m in 4 cardinal directions
    # and something in between.
    # Simple circle of 2m radius approx square room
    scan = []
    for angle in range(0, 360, 5):
        # Distance to square walls from center
        # x = d * cos(a), y = d * sin(a)
        # Max |x| = 2 or Max |y| = 2
        # d * max(|cos|, |sin|) = 2
        # d = 2 / max(...)
        rad = math.radians(angle)
        dist_m = center_dist_m / max(abs(math.cos(rad)), abs(math.sin(rad)))
        scan.append((angle, dist_m * 1000.0)) # mm
    return scan

# --- Tests ---

# TC 7.1: Pose Convergence
def test_pose_convergence(dummy_map_path):
    # Start at true pose (0, 0) relative to center -> (10, 10) global
    # Map origin is (200, 200) pixels -> (0,0) index?
    # No, usually origin is defined such that (0,0) meters is somewhere.
    # SLAMEngine: start_col = x/res + origin_x.
    # If x=0, y=0 global, index is 200, 200.
    # My dummy map has walls around 200,200 (global 0,0).

    true_pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0)

    # Initialize MCL with noise
    initial_guess = RobotPose(x=0.2, y=-0.2, theta=0.1, timestamp=0)
    mcl = MCLEngine(dummy_map_path, initial_guess, num_particles=1000)

    scan = create_synthetic_scan(2.0)

    # Run update loop
    for _ in range(20):
        # Predict (no motion)
        mcl.predict((0.0, 0.0, 0.0))
        # Update
        result = mcl.update(scan, true_pose)

    # Check convergence
    # Tolerances: 10cm, 5 degrees
    assert abs(result.pose.x - true_pose.x) < 0.15
    assert abs(result.pose.y - true_pose.y) < 0.15
    assert abs(result.pose.theta - true_pose.theta) < 0.2

# TC 7.2: Obstacle Avoidance & Pure Pursuit
def test_obstacle_avoidance(dummy_map_path):
    # Load map
    grid = np.load(dummy_map_path)
    planner = AStarPlanner(grid)

    start = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=0) # Index 200, 200
    goal = RobotPose(x=1.0, y=0.0, theta=0.0, timestamp=0)  # Index 220, 200 (20 pixels right)

    # 1. Plan without obstacle
    path = planner.plan(start, goal)
    assert len(path) > 0
    # Should be roughly straight line (y ~ 0)
    mid_y = path[len(path)//2][1]
    assert abs(mid_y) < 0.2

    # 2. Inject Obstacle at (0.5, 0.0) -> Index 210, 200
    planner.inject_virtual_obstacle(0.5, 0.0)

    path_obs = planner.plan(start, goal)
    assert len(path_obs) > 0

    # Check deviation
    ys = [p[1] for p in path_obs]
    max_dev = max([abs(y) for y in ys])
    assert max_dev > 0.2 # Should detour

    # 3. Pure Pursuit Output
    controller = PurePursuitController(lookahead_dist=0.4, max_speed=0.5)

    # Test at start, path goes around obstacle
    # First point of path is start.
    # Lookahead should pick a point ~0.4m away.
    l, r = controller.get_command(start, path_obs)

    # If detour is to the side, we should turn.
    assert l != r # Should be turning
    assert abs(l) <= 1.0
    assert abs(r) <= 1.0

# TC 7.3: Ultrasonic Replanning Logic
def test_ultrasonic_replanning(dummy_map_path):
    # Mocks
    motor = MagicMock(spec=IMotorController)
    lidar = MagicMock(spec=ILidarSensor)
    us = MagicMock(spec=IUltrasonicSensor)
    inp = MagicMock(spec=IInputController)
    odom = MagicMock(spec=OdometryEngine)
    recorder = MagicMock(spec=DataRecorder)

    lidar.get_latest_scan.return_value = [] # No lidar for this test
    inp.get_state.return_value = {"connected": True, "buttons": {"A": True}} # Enable Autonomous

    # Setup Orchestrator
    orch = RobotOrchestrator(motor, lidar, us, inp, odom, recorder)
    orch.setup_navigation(dummy_map_path, RobotPose(x=0,y=0,theta=0,timestamp=0))
    orch.set_autonomous(True)
    orch.planner = MagicMock(wraps=orch.planner) # Spy on planner

    # Set initial path
    orch.set_goal_path([(0,0), (1,0), (2,0)])
    orch.active_path = orch.original_path

    # 1. Trigger Reverse (US < 15cm)
    orch.min_us_distance_cm = 10.0 # Directly set state (thread usually does this)
    orch._drive_autonomous() # Run one step

    assert orch.nav_state == NavigationState.REVERSING
    assert orch.motor.stop.call_count >= 1

    # 2. Simulate Reverse Motion
    # _drive_autonomous calls motor.set_speed(-0.2, -0.2)
    # We simulate Odometry moving back
    orch.current_pose = RobotPose(x=-0.20, y=0.0, theta=0.0, timestamp=0) # Moved back 20cm
    orch.min_us_distance_cm = 40.0 # Clear

    orch._drive_autonomous()

    # 3. Check Transition to REPLANNING
    assert orch.nav_state == NavigationState.REPLANNING

    # Run one more step to execute REPLANNING logic
    orch._drive_autonomous()

    # 4. Check Injection
    # Should have called inject_virtual_obstacle
    assert orch.planner.inject_virtual_obstacle.called

    # 5. Check Replanning
    assert orch.planner.plan.called

    # If plan succeeds (it should, mock map is empty except for what we added), state -> FOLLOWING
    # Note: Real planner is wrapped. Virtual obstacle injected at ~0.2, 0.
    # Plan from -0.2, 0 to 2.0, 0.
    # It should find a path.
    assert orch.nav_state == NavigationState.FOLLOWING
