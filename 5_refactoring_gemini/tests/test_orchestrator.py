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

    def save_session_async(self, session, filename=None):
        self.saved_session = session
        return None

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
    assert orch.session_id is not None

    # Create valid frame
    frame = LidarFrame(
        timestamp=time.time(),
        frame_id=0,
        scan_data=[],
        controller_states=[]
    )
    orch.frames.append(frame)

    orch.stop_recording()
    assert not orch.is_recording
    assert rec.saved_session is not None
    assert len(rec.saved_session.frames) == 1
    assert rec.saved_session.frames[0].frame_id == 0
