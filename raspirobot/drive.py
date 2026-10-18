"""Umsetzung der Controller-Eingaben in Motor-Richtung und Duty Cycle (reine Funktionen)."""

from dataclasses import dataclass

from . import config
from .collision import CollisionStatus


@dataclass(frozen=True)
class DriveCommand:
    left_backward: bool
    right_backward: bool
    left_duty: float    # Prozent, vor Kollisionsbegrenzung
    right_duty: float


def trigger_to_duty_cycle(trigger_value: float) -> float:
    """Trigger -1.0..1.0 linear auf MIN_DUTY_CYCLE..MAX_DUTY_CYCLE (Prozent)."""
    duty_cycle = (trigger_value + 1.0) / 2.0 * config.MAX_DUTY_CYCLE
    return max(config.MIN_DUTY_CYCLE, min(config.MAX_DUTY_CYCLE, duty_cycle))


def compute_drive_command(left_duty: float, right_duty: float, tank_turn: bool, is_moving_forward: bool) -> DriveCommand:
    """Normalfahrt: linker Trigger steuert linken Motor, rechter den rechten, beide in Fahrtrichtung.

    Tank-Turn (A gehalten): Der stärker gedrückte Trigger gibt die Drehrichtung
    vor, beide Motoren laufen gegenläufig mit dessen Duty Cycle. Im
    Rückwärtsmodus ist die Drehrichtung vertauscht. Bei Gleichstand zählt der
    linke Trigger.
    """
    if tank_turn:
        right_dominant = right_duty > left_duty
        duty = right_duty if right_dominant else left_duty
        left_backward = right_dominant == is_moving_forward
        return DriveCommand(left_backward, not left_backward, duty, duty)

    backward = not is_moving_forward
    return DriveCommand(backward, backward, left_duty, right_duty)


def limit_for_collision(left_duty: float, right_duty: float, collision: CollisionStatus):
    """Kritisch: beide 0. Erkannt: beide halbiert. Danach Begrenzung auf MIN..MAX."""
    if collision.collision_critical:
        left_duty = config.MIN_DUTY_CYCLE
        right_duty = config.MIN_DUTY_CYCLE
    elif collision.collision_detected:
        left_duty *= 0.5
        right_duty *= 0.5

    left_duty = max(config.MIN_DUTY_CYCLE, min(config.MAX_DUTY_CYCLE, left_duty))
    right_duty = max(config.MIN_DUTY_CYCLE, min(config.MAX_DUTY_CYCLE, right_duty))
    return left_duty, right_duty
