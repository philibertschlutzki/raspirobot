import pytest
import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from robot_orchestrator import RobotOrchestrator
from mocks.mock_hardware import MockMotorController, MockLidarSensor, MockUltrasonicSensor, MockInputController
from odometry import OdometryEngine
from recorder import DataRecorder
from data_models import PathRecordingData, LidarFrame, ControllerSample, RobotPose

class MockRecorder(DataRecorder):
    def __init__(self):
        # Do not call super init to avoid creating directories
        self.saved_session = None
        self.output_dir = "mock_dir"
        self.frames = []
        self.started = False
        self.stopped = False

    def start_session(self, session_id, hardware_info):
        self.started = True
        self.session_id = session_id

    def log_frame(self, frame):
        self.frames.append(frame)

    def stop_session(self):
        self.stopped = True

def test_emergency_stop():
    motor = MockMotorController()
    lidar = MockLidarSensor()
    us = MockUltrasonicSensor()
    inp = MockInputController()
    odo = OdometryEngine()
    rec = MockRecorder()

    orch = RobotOrchestrator(motor, lidar, us, inp, odo, rec)

    # Simulate US < 15cm
    orch.min_us_distance_cm = 10.0
    orch.is_moving_forward = True

    # Try to drive forward (trig = 1.0 => speed 1.0)
    orch._drive_manual(1.0, 1.0)

    # Expect stop (0.0)
    assert motor.left_speed == 0.0
    assert motor.right_speed == 0.0

    # Simulate US > 15cm but < 30cm (Collision Dist) -> Factor 0.5
    orch.min_us_distance_cm = 20.0
    orch._drive_manual(1.0, 1.0)

    # Expect 0.5 speed
    assert motor.left_speed == 0.5
    assert motor.right_speed == 0.5

def test_recording_flow():
    motor = MockMotorController()
    lidar = MockLidarSensor()
    us = MockUltrasonicSensor()
    inp = MockInputController()
    odo = OdometryEngine()
    rec = MockRecorder()

    orch = RobotOrchestrator(motor, lidar, us, inp, odo, rec)

    orch.start_recording()
    assert orch.is_recording
    assert rec.started

    # Create valid frame logic happens in lidar thread, but we can call it manually if we exposed it?
    # Or simulate lidar thread.

    # Let's mock lidar behavior.
    lidar_scan = [(100, 0.0, 1000.0)]
    lidar.scan = lidar_scan

    # We can inject a frame into frames via _lidar_thread logic if we run it.
    # But for unit test, we just want to see if `log_frame` is called.
    # The orchestrator calls log_frame in _lidar_thread.

    # Let's verify start/stop first.
    orch.stop_recording()
    assert not orch.is_recording
    assert rec.stopped
