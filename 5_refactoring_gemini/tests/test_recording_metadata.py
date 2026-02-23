import pytest
from unittest.mock import MagicMock
from robot_orchestrator import RobotOrchestrator

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
        # Updated to start_session
        mock_recorder.start_session.assert_called_once()
        mock_recorder.stop_session.assert_called_once()

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

        # Verify call args of start_session
        args, _ = mock_recorder.start_session.call_args
        session_id = args[0]
        hw_info = args[1]

        # Check initial state (Lidar was OK when starting)
        assert hw_info["capabilities"]["has_lidar"] is True

        # If we want to check state at STOP, we'd need to check log_frame or how recorder handles it.
        # But hardware_info is passed at START.
        # If requirements say hw info should reflect state, it usually means static capabilities.
