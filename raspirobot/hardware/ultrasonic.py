"""HC-SR04 Ultraschallsensoren links und rechts."""

import time

import RPi.GPIO as GPIO

from .. import config


def setup_pins() -> None:
    """Trigger als Ausgang (LOW), Echo als Eingang. Erwartet BCM-Modus."""
    GPIO.setup(config.TRIG_LEFT, GPIO.OUT)
    GPIO.setup(config.ECHO_LEFT, GPIO.IN)
    GPIO.output(config.TRIG_LEFT, False)

    GPIO.setup(config.TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(config.ECHO_RIGHT, GPIO.IN)
    GPIO.output(config.TRIG_RIGHT, False)


def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """Eine Messung in cm. Gibt -1.0 bei Timeout, Fehler oder Wert außerhalb 2-400 cm zurück."""
    try:
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        pulse_start = time.time()
        timeout_start = pulse_start
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1.0

        pulse_end = time.time()
        timeout_end = pulse_end
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_end > timeout:
                return -1.0

        distance = ((pulse_end - pulse_start) * config.SOUND_SPEED) / 2
        if distance < 2 or distance > 400:
            return -1.0
        return distance
    except Exception:
        return -1.0


class UltrasonicSensors:
    """Hält die letzten gültigen Messwerte (cm).

    Ungültige Messungen (-1.0) überschreiben den letzten Wert nicht. Startwert
    ist 100.0 cm für beide Seiten.
    """

    def __init__(self):
        self.distance_left = 100.0
        self.distance_right = 100.0

    def run(self, should_run) -> None:
        """Messschleife mit SENSOR_UPDATE_HZ, solange ``should_run()`` True ist."""
        print(f"[THREAD] Sensor Thread gestartet ({config.SENSOR_UPDATE_HZ} Hz)")
        sleep_time = 1.0 / config.SENSOR_UPDATE_HZ

        while should_run():
            loop_start = time.time()

            dist_left = measure_distance(config.TRIG_LEFT, config.ECHO_LEFT)
            if dist_left > 0:
                self.distance_left = dist_left

            dist_right = measure_distance(config.TRIG_RIGHT, config.ECHO_RIGHT)
            if dist_right > 0:
                self.distance_right = dist_right

            sleep_duration = sleep_time - (time.time() - loop_start)
            if sleep_duration > 0:
                time.sleep(sleep_duration)

        print("[THREAD] Sensor Thread beendet")
