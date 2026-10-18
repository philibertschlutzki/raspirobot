import fakes

from raspirobot import config
from raspirobot.hardware.xbox_controller import XboxController


def make():
    c = XboxController()
    c.joystick = fakes.FakeJoystick()
    c.joystick.init()
    return c


def test_b_toggles_direction_on_edge_only():
    c = make()
    c.joystick.buttons = {config.BUTTON_B: 1}
    assert c.read().is_moving_forward is False
    assert c.read().is_moving_forward is False  # gehalten: kein erneutes Umschalten
    c.joystick.buttons = {}
    c.read()
    c.joystick.buttons = {config.BUTTON_B: 1}
    assert c.read().is_moving_forward is True


def test_x_plus_y_requests_toggle_once():
    c = make()
    c.joystick.buttons = {config.BUTTON_X: 1, config.BUTTON_Y: 1}
    assert c.read().recording_toggle_requested
    assert not c.read().recording_toggle_requested


def test_start_requests_quit():
    c = make()
    c.joystick.buttons = {config.BUTTON_START: 1}
    assert c.read().quit_requested
