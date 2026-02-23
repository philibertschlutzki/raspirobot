import sys
import os
import time
import unittest
from unittest.mock import MagicMock, patch

# Adjust path to import from parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot_orchestrator import RobotOrchestrator
from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
from odometry import OdometryEngine
from recorder import DataRecorder
from config import *

class TestRobotOrchestrator(unittest.TestCase):
    def setUp(self):
        self.motor = MagicMock(spec=IMotorController)
        self.lidar = MagicMock(spec=ILidarSensor)
        self.ultrasonic = MagicMock(spec=IUltrasonicSensor)
        self.input = MagicMock(spec=IInputController)
        self.odometry = MagicMock(spec=OdometryEngine)
        self.recorder = MagicMock(spec=DataRecorder)

        # Default mock returns
        self.lidar.get_latest_scan.return_value = []
        self.ultrasonic.get_distances.return_value = {"left": 100.0, "right": 100.0}
        self.input.get_state.return_value = {"connected": True, "buttons": {}, "axes": {}}

        self.orchestrator = RobotOrchestrator(
            self.motor, self.lidar, self.ultrasonic, self.input, self.odometry, self.recorder
        )
        self.orchestrator.is_running = True # Simulate running state for helpers

    def tearDown(self):
        self.orchestrator.stop()

    def test_sensor_timeout_stop(self):
        # Simulate sensor timeout > SENSOR_TIMEOUT_STOP_SEC
        self.orchestrator.us_distances = {"left": -1.0, "right": -1.0}
        # Force last valid time to be old
        old_time = time.time() - (SENSOR_TIMEOUT_STOP_SEC + 1.0)
        self.orchestrator.us_last_valid_time = {"left": old_time, "right": old_time}

        self.orchestrator._drive_manual(1.0, 1.0)

        self.motor.stop.assert_called()

    def test_sensor_timeout_warn(self):
        # Simulate sensor timeout > SENSOR_TIMEOUT_WARN_SEC but < STOP
        self.orchestrator.us_distances = {"left": -1.0, "right": -1.0}
        old_time = time.time() - (SENSOR_TIMEOUT_WARN_SEC + 0.5)
        self.orchestrator.us_last_valid_time = {"left": old_time, "right": old_time}

        self.orchestrator._drive_manual(1.0, 1.0)

        # Check if speed was reduced
        # Input 1.0 -> throttle 1.0 -> set_speed should be FAIL_FACTOR
        expected_speed = 1.0 * SENSOR_FAIL_SPEED_FACTOR
        self.motor.set_speed.assert_called_with(expected_speed, expected_speed)

    def test_collision_avoidance_ultrasonic(self):
        # Distance < 15.0 -> Emergency Stop
        self.orchestrator.min_us_distance_cm = 10.0
        self.orchestrator.us_distances = {"left": 10.0, "right": 10.0} # Valid values
        self.orchestrator.us_last_valid_time = {"left": time.time(), "right": time.time()} # Valid time

        self.orchestrator._drive_manual(1.0, 1.0)

        self.motor.set_speed.assert_called_with(0.0, 0.0)

    def test_collision_avoidance_slowdown(self):
        # Distance < COLLISION_DISTANCE_CM -> Slowdown (0.5 factor)
        self.orchestrator.min_us_distance_cm = COLLISION_DISTANCE_CM - 5.0
        self.orchestrator.us_distances = {"left": 25.0, "right": 25.0}
        self.orchestrator.us_last_valid_time = {"left": time.time(), "right": time.time()}

        self.orchestrator._drive_manual(1.0, 1.0)

        expected_speed = 1.0 * 0.5
        self.motor.set_speed.assert_called_with(expected_speed, expected_speed)

if __name__ == '__main__':
    unittest.main()
