import pytest
from pydantic import ValidationError
import sys
import os

# Add parent directory to sys.path to allow importing data_models
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from data_models import RobotPose, ControllerSample, LidarFrame, PathRecordingData, LogEntry, LogLevel

def test_robot_pose_valid():
    pose = RobotPose(x=1.0, y=2.0, theta=3.14, timestamp=100.0)
    assert pose.x == 1.0
    assert pose.y == 2.0
    assert pose.theta == 3.14
    assert pose.timestamp == 100.0
    assert pose.model_dump() == {"x": 1.0, "y": 2.0, "theta": 3.14, "timestamp": 100.0}

def test_controller_sample_valid():
    pose = RobotPose(x=0, y=0, theta=0, timestamp=100)
    sample = ControllerSample(
        timestamp=100.1,
        left_duty=0.5,
        right_duty=0.5,
        is_moving_forward=True,
        pose=pose
    )
    assert sample.left_duty == 0.5
    assert sample.pose.x == 0

    dump = sample.model_dump()
    assert dump['left_duty'] == 0.5
    assert dump['pose']['x'] == 0

def test_lidar_frame_valid():
    pose = RobotPose(x=0, y=0, theta=0, timestamp=100)
    sample = ControllerSample(timestamp=100, left_duty=0, right_duty=0, is_moving_forward=True, pose=pose)
    frame = LidarFrame(
        timestamp=100.0,
        frame_id=1,
        scan_data=[(0.0, 1.0), (90.0, 2.0)],
        controller_states=[sample]
    )
    assert frame.scan_data == [(0.0, 1.0), (90.0, 2.0)]
    assert len(frame.controller_states) == 1

    dump = frame.model_dump()
    assert len(dump['scan_data']) == 2
    assert dump['controller_states'][0]['left_duty'] == 0

def test_path_recording_data_valid():
    data = PathRecordingData(
        session_id="test_session",
        start_timestamp=100.0,
        end_timestamp=200.0,
        hardware_info={"os": "linux"},
        frames=[]
    )
    assert data.session_id == "test_session"
    assert data.frames == []

def test_log_entry_valid():
    entry = LogEntry(
        timestamp=123.45,
        level=LogLevel.INFO,
        event_name="TEST_EVENT",
        data={"key": "value"}
    )
    assert entry.level == "INFO"
    assert entry.data["key"] == "value"

def test_log_entry_invalid_level():
    with pytest.raises(ValidationError):
        LogEntry(
            timestamp=123.45,
            level="INVALID",
            event_name="TEST",
            data={}
        )

def test_missing_fields():
    with pytest.raises(ValidationError):
        RobotPose(x=1.0) # Missing other fields
