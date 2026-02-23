import pytest
import sys
import os

# Add 5_refactoring_gemini/ to sys.path to find interfaces.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# Add tests/ to sys.path to find mocks package
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
from mocks.mock_hardware import MockMotorController, MockLidarSensor, MockUltrasonicSensor, MockInputController

def test_interface_instantiation_fails():
    with pytest.raises(TypeError):
        IMotorController() # Abstract class
    with pytest.raises(TypeError):
        ILidarSensor()
    with pytest.raises(TypeError):
        IUltrasonicSensor()

def test_mock_motor_controller():
    motor = MockMotorController()
    assert isinstance(motor, IMotorController)
    motor.set_speed(0.5, -0.5)
    assert motor.left_speed == 0.5
    assert motor.right_speed == -0.5
    motor.stop()
    assert motor.left_speed == 0.0

def test_mock_lidar():
    lidar = MockLidarSensor()
    assert isinstance(lidar, ILidarSensor)
    lidar.start()
    assert lidar.running
    lidar.set_mock_scan([(0.0, 10.0)])
    assert lidar.get_latest_scan() == [(0.0, 10.0)]
    lidar.stop()
    assert not lidar.running

def test_mock_ultrasonic():
    us = MockUltrasonicSensor()
    assert isinstance(us, IUltrasonicSensor)
    us.set_mock_distance(15.5)
    assert us.get_distance() == 15.5

def test_mock_input():
    inp = MockInputController()
    assert isinstance(inp, IInputController)
    inp.set_mock_state({'btn_a': 1})
    assert inp.get_state()['btn_a'] == 1
