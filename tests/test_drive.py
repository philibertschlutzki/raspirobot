import pytest

from raspirobot import config
from raspirobot.collision import analyze_collision
from raspirobot.drive import compute_drive_command, limit_for_collision, trigger_to_duty_cycle


def test_trigger_mapping():
    assert trigger_to_duty_cycle(-1.0) == 0.0
    assert trigger_to_duty_cycle(0.0) == config.MAX_DUTY_CYCLE / 2
    assert trigger_to_duty_cycle(1.0) == config.MAX_DUTY_CYCLE
    assert trigger_to_duty_cycle(5.0) == config.MAX_DUTY_CYCLE


@pytest.mark.parametrize(
    "left,right,forward,expected",
    [
        (2.0, 8.0, True, (True, False, 8.0, 8.0)),
        (8.0, 2.0, True, (False, True, 8.0, 8.0)),
        (2.0, 8.0, False, (False, True, 8.0, 8.0)),
        (8.0, 2.0, False, (True, False, 8.0, 8.0)),
        (5.0, 5.0, True, (False, True, 5.0, 5.0)),
    ],
)
def test_tank_turn(left, right, forward, expected):
    cmd = compute_drive_command(left, right, tank_turn=True, is_moving_forward=forward)
    assert (cmd.left_backward, cmd.right_backward, cmd.left_duty, cmd.right_duty) == expected


def test_normal_drive_backward():
    cmd = compute_drive_command(3.0, 6.0, tank_turn=False, is_moving_forward=False)
    assert (cmd.left_backward, cmd.right_backward, cmd.left_duty, cmd.right_duty) == (True, True, 3.0, 6.0)


def test_collision_limits():
    free = analyze_collision(100.0, 100.0, [])
    near = analyze_collision(25.0, 100.0, [])
    critical = analyze_collision(100.0, 10.0, [])
    assert limit_for_collision(10.0, 8.0, free) == (10.0, 8.0)
    assert limit_for_collision(10.0, 8.0, near) == (5.0, 4.0)
    assert limit_for_collision(10.0, 8.0, critical) == (0.0, 0.0)
