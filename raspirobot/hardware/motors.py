"""Zwei DC-Motoren: Richtung über GPIO, Geschwindigkeit über Hardware-PWM."""

import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

from .. import config
from ..log import logger


def _direction_level(backward: bool):
    if config.DIR_HIGH_IS_BACKWARD:
        return GPIO.HIGH if backward else GPIO.LOW
    return GPIO.LOW if backward else GPIO.HIGH


class Motors:
    def __init__(self):
        self.pwm_left = None
        self.pwm_right = None

    @staticmethod
    def setup_direction_pins() -> None:
        """Richtungspins als Ausgang, beide auf vorwärts."""
        GPIO.setup(config.DIR_LEFT_PIN, GPIO.OUT)
        GPIO.setup(config.DIR_RIGHT_PIN, GPIO.OUT)
        GPIO.output(config.DIR_LEFT_PIN, _direction_level(False))
        GPIO.output(config.DIR_RIGHT_PIN, _direction_level(False))

    def setup_pwm(self) -> None:
        print("[SETUP] Initialisiere Hardware PWM...")
        try:
            self.pwm_left = HardwarePWM(
                pwm_channel=config.PWM_CHANNEL_LEFT, hz=config.PWM_FREQUENCY, chip=config.PWM_CHIP
            )
            self.pwm_right = HardwarePWM(
                pwm_channel=config.PWM_CHANNEL_RIGHT, hz=config.PWM_FREQUENCY, chip=config.PWM_CHIP
            )
            self.pwm_left.start(config.MIN_DUTY_CYCLE)
            self.pwm_right.start(config.MIN_DUTY_CYCLE)
            print(f"[SETUP] PWM initialisiert - Frequenz: {config.PWM_FREQUENCY} Hz")
        except Exception as e:
            print(f"[ERROR] PWM Initialisierung fehlgeschlagen: {e}")
            raise

    @staticmethod
    def set_direction(left_backward: bool, right_backward: bool) -> None:
        GPIO.output(config.DIR_LEFT_PIN, _direction_level(left_backward))
        GPIO.output(config.DIR_RIGHT_PIN, _direction_level(right_backward))

    def set_duty(self, left_duty: float, right_duty: float) -> None:
        """Setzt den Duty Cycle in Prozent; Fehler werden geloggt, nicht geworfen."""
        try:
            self.pwm_left.change_duty_cycle(left_duty)
            self.pwm_right.change_duty_cycle(right_duty)
        except Exception as e:
            logger.error(f"PWM Update fehlgeschlagen: {e}")

    def stop(self) -> None:
        """Duty Cycle auf 0 und PWM abschalten."""
        print("[CLEANUP] Stoppe Motoren...")
        for name, pwm in (("Left", self.pwm_left), ("Right", self.pwm_right)):
            if pwm:
                try:
                    pwm.change_duty_cycle(config.MIN_DUTY_CYCLE)
                    pwm.stop()
                except Exception as e:
                    print(f"[CLEANUP] PWM {name} Fehler: {e}")
