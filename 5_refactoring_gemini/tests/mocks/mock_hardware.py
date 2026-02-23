from typing import List, Tuple, Dict, Any
# Expect interfaces to be available in path
try:
    from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
except ImportError:
    # Try relative import if inside a package structure
    import sys
    import os
    # If this file is run directly or imported in a way that interfaces is not found
    # We can try to find it.
    # Assumes file is at 5_refactoring_gemini/tests/mocks/mock_hardware.py
    # Interfaces at 5_refactoring_gemini/interfaces.py
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
    from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController

class MockMotorController(IMotorController):
    def __init__(self):
        self.left_speed = 0.0
        self.right_speed = 0.0

    def set_speed(self, left: float, right: float) -> None:
        self.left_speed = left
        self.right_speed = right

    def stop(self) -> None:
        self.left_speed = 0.0
        self.right_speed = 0.0

class MockLidarSensor(ILidarSensor):
    def __init__(self):
        self.running = False
        self.scan_data = []

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def get_latest_scan(self) -> List[Tuple[float, float]]:
        return self.scan_data

    def set_mock_scan(self, scan: List[Tuple[float, float]]):
        self.scan_data = scan

class MockUltrasonicSensor(IUltrasonicSensor):
    def __init__(self):
        self.distance = 100.0

    def get_distance(self) -> float:
        return self.distance

    def set_mock_distance(self, distance: float):
        self.distance = distance

class MockInputController(IInputController):
    def __init__(self):
        self.state = {}

    def get_state(self) -> Dict[str, Any]:
        return self.state

    def set_mock_state(self, state: Dict[str, Any]):
        self.state = state
