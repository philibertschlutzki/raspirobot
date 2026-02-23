import sys
import os
import pytest
from unittest.mock import MagicMock, patch

# Add parent directory to path to import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from robot_orchestrator import RobotOrchestrator

class TestShutdown:
    def test_cleanup_called(self):
        # Mock GPIO
        with patch("robot_orchestrator.GPIO") as mock_gpio: # Depending on how it is imported
            # If robot_orchestrator imports GPIO conditionally, we might need to be careful.
            # In PC_TEST_MODE, it might not import it or use a mock.
            # But the prompt says "Implement Graceful Shutdown & Motor Cleanup".
            # "Wenn der robot_orchestrator.py ... gestoppt wird, müssen ... GPIO.cleanup() ausgeführt werden".

            # If PC_TEST_MODE is on, maybe it skips it?
            # "wrapped in try/except for PC mode/import errors".

            # Let's assume we want to call it if it exists.

            # Setup Orchestrator
            motor = MagicMock()
            lidar = MagicMock()
            orch = RobotOrchestrator(
                motor, lidar, MagicMock(), MagicMock(), MagicMock(), MagicMock()
            )
            orch.logger = MagicMock()

            # Call stop
            orch.stop()

            # Verify
            motor.stop.assert_called_once()
            lidar.stop.assert_called_once()

            # Verify GPIO cleanup
            # If robot_orchestrator.py imports RPi.GPIO as GPIO (global scope), patch works.
            # If it does NOT import it because of try/except, patch might fail or mock nothing.

            # Let's check robot_orchestrator.py imports.
            # It imports from config...

            # Actually, robot_orchestrator.py does NOT import GPIO directly in the provided snippet!
            # It uses `IMotorController`.
            # But the prompt says "ensure ... GPIO.cleanup() is executed".
            # This implies robot_orchestrator SHOULD import GPIO and clean it up, OR delegates it to something.
            # "Passe die stop() Methode im Orchestrator so an, dass sie einen sicheren Teardown aller angebundenen Dependency-Injection-Hardware-Instanzen aufruft."
            # AND "Hardware-PWM-Signale zwingend auf 0 gesetzt und ein GPIO.cleanup() ausgeführt werden".

            # The PWM signals are handled by motor.stop().
            # GPIO.cleanup() is global.

            # If I add `import RPi.GPIO as GPIO` inside `stop()` (lazy import) or at top level with try/except, I can call it.

            # I will assert that IF GPIO is present, it is cleaned up.

            mock_gpio.cleanup.assert_called_once()
