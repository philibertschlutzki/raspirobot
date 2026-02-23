import sys
import os
import pytest
from unittest.mock import MagicMock

# Add parent directory to path to import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from robot_orchestrator import RobotOrchestrator
from data_models import PathRecordingData

class TestRecordingMetadata:
    def test_capabilities_in_hardware_info(self):
        # Setup
        mock_recorder = MagicMock()
        orch = RobotOrchestrator(
            MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock(), mock_recorder
        )
        orch.logger = MagicMock()

        # Start recording
        orch.start_recording()

        # Simulate some state
        orch.lidar_error_active = False # Lidar is working

        # Stop recording
        orch.stop_recording()

        # Verify
        mock_recorder.save_session_async.assert_called_once()
        args, _ = mock_recorder.save_session_async.call_args
        session_data: PathRecordingData = args[0]

        assert "capabilities" in session_data.hardware_info
        caps = session_data.hardware_info["capabilities"]
        assert caps["has_lidar"] is True
        assert caps["has_pose"] is True

    def test_capabilities_lidar_error(self):
        # Setup
        mock_recorder = MagicMock()
        orch = RobotOrchestrator(
            MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock(), mock_recorder
        )
        orch.logger = MagicMock()

        orch.start_recording()

        # Simulate lidar error
        orch.lidar_error_active = True

        orch.stop_recording()

        args, _ = mock_recorder.save_session_async.call_args
        session_data: PathRecordingData = args[0]

        caps = session_data.hardware_info["capabilities"]
        # If lidar error is active, has_lidar should be False?
        # Or maybe it just records the state.
        # The requirement says "including sensor failure states".
        # So maybe "has_lidar" means "is equipped with lidar".
        # But "Sensorausfall-Stati" suggests we should record if it was working.
        # Let's assume has_lidar reflects current working status OR we add "lidar_healthy".
        # v3.0.2 had: 'has_lidar': True.
        # I will stick to has_lidar = not lidar_error_active for now, or just True if it is physically there.
        # Let's check what I planned: "has_lidar: not self.lidar_error_active".

        assert caps["has_lidar"] is False
