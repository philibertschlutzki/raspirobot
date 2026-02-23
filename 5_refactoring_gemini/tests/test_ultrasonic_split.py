import sys
import os
import time
import pytest
from typing import Dict
from unittest.mock import MagicMock, patch

# Add parent directory to path to import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from hardware.ultrasonic import RealUltrasonic
from interfaces import IUltrasonicSensor

class TestUltrasonicSplit:
    def test_returns_dict(self):
        sensor = RealUltrasonic()
        sensor.pc_mode = True

        # Call the new method
        result = sensor.get_distances()

        assert isinstance(result, dict)
        assert "left" in result
        assert "right" in result
        assert result["left"] == 1000.0
        assert result["right"] == 1000.0

    def test_orchestrator_integration(self):
        try:
            from robot_orchestrator import RobotOrchestrator
        except ImportError:
            # Maybe path issue if run from wrong dir
            sys.path.append("5_refactoring_gemini")
            from robot_orchestrator import RobotOrchestrator

        mock_us = MagicMock() # spec=IUltrasonicSensor causes issues if class not updated yet? No, import is fresh.
        # Setup the mock to return the dict
        mock_us.get_distances.return_value = {"left": 10.0, "right": 50.0}

        # Create orchestrator
        orch = RobotOrchestrator(
            MagicMock(), MagicMock(), mock_us, MagicMock(), MagicMock(), MagicMock()
        )
        orch.logger = MagicMock() # Avoid logging issues

        orch.is_running = True

        # Mock time.sleep to stop the loop
        def side_effect(seconds):
            orch.is_running = False

        with patch("time.sleep", side_effect=side_effect):
            orch._ultrasonic_thread()

        # Verify min distance is 10.0 (min of 10 and 50)
        assert orch.min_us_distance_cm == 10.0
