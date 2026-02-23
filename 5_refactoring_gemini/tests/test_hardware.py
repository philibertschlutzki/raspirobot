import os
import sys
import pytest

# Force PC_TEST_MODE
os.environ["PC_TEST_MODE"] = "1"

# Add 5_refactoring_gemini/ to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import hardware modules
# These should NOT crash even if hardware libs are missing
import hardware.motor
import hardware.lidar
import hardware.ultrasonic
import hardware.xbox

from hardware.motor import RealMotorController
from hardware.lidar import RealLidarSensor
from hardware.ultrasonic import RealUltrasonic
from hardware.xbox import XboxInput

def test_motor_pc_mode():
    motor = RealMotorController()
    motor.set_speed(1.0, 1.0) # Should do nothing
    motor.stop()
    motor.cleanup()

def test_lidar_pc_mode():
    lidar = RealLidarSensor()
    lidar.start()
    scan = lidar.get_latest_scan()
    assert scan == []
    lidar.stop()

def test_ultrasonic_pc_mode():
    us = RealUltrasonic()
    dists = us.get_distances()
    assert dists["left"] == 1000.0
    assert dists["right"] == 1000.0

def test_xbox_pc_mode():
    # XboxInput might try to init pygame
    xbox = XboxInput()
    state = xbox.get_state()
    # Should not crash, returns dict
    assert isinstance(state, dict)
    assert "connected" in state
