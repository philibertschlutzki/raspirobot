import sys
import os
import time
import pytest
from unittest.mock import MagicMock, patch

# Add parent directory to path to import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from hardware.lidar import RealLidarSensor

class TestLidarFrequency:

    def test_frequency_warning(self):
        # 1. Setup
        sensor = RealLidarSensor()
        sensor.pc_mode = False # Force "Real" mode logic
        sensor.running = True  # <--- CRITICAL FIX: Simulate that it is running
        sensor.lidar = MagicMock()
        sensor.logger = MagicMock() # Mock the logger instance directly

        # Mock iterator to return dummy scans
        dummy_scan = [(15, 90, 1000)]
        sensor.iterator = iter([dummy_scan, dummy_scan, dummy_scan, dummy_scan])

        # 2. Mock time
        with patch("hardware.lidar.time.time") as mock_time:

            # Start time
            start_t = 1000.0
            mock_time.return_value = start_t

            # First scan - initialization
            sensor.get_latest_scan()

            # Second scan - 0.05s later (20Hz) - OK
            mock_time.return_value = start_t + 0.05
            sensor.get_latest_scan()

            # Check no warning
            sensor.logger.warn.assert_not_called()

            # Third scan - 0.2s later (5Hz) - Warning!
            mock_time.return_value = start_t + 0.05 + 0.20
            sensor.get_latest_scan()

            # Verify warning
            sensor.logger.warn.assert_called()
            args, kwargs = sensor.logger.warn.call_args
            assert args[0] == "LIDAR_LOW_FREQ"
            assert "hz" in str(kwargs) or "hz" in str(args)
