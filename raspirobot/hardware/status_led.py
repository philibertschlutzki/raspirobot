"""Status-LED (Blinkmuster) und optionaler Buzzer."""

import time

import RPi.GPIO as GPIO

from .. import config
from ..log import logger


class StatusIndicator:
    """LED-Zustände: ``off``, ``recording`` (blinkt mit 0,25 s), ``saving`` (dauerhaft an)."""

    def __init__(self):
        self.status = "off"
        self._blink_state = False
        self._last_toggle = 0.0

    @staticmethod
    def setup_pins() -> None:
        if config.LED_ENABLED:
            GPIO.setup(config.STATUS_LED_PIN, GPIO.OUT)
            GPIO.output(config.STATUS_LED_PIN, GPIO.LOW)
            print(f"[SETUP] Status-LED initialisiert (GPIO{config.STATUS_LED_PIN})")
        if config.BUZZER_ENABLED:
            GPIO.setup(config.BUZZER_PIN, GPIO.OUT)
            GPIO.output(config.BUZZER_PIN, GPIO.LOW)
            print(f"[SETUP] Buzzer initialisiert (GPIO{config.BUZZER_PIN})")

    def set_status(self, status: str) -> None:
        self.status = status

    def run(self, should_run) -> None:
        """LED-Schleife (alle 50 ms), solange ``should_run()`` True ist."""
        if not config.LED_ENABLED:
            return

        print(f"[THREAD] LED Thread gestartet (GPIO{config.STATUS_LED_PIN})")
        while should_run():
            now = time.time()
            if self.status == "off":
                GPIO.output(config.STATUS_LED_PIN, GPIO.LOW)
                self._blink_state = False
            elif self.status == "recording":
                if now - self._last_toggle > 0.25:
                    self._blink_state = not self._blink_state
                    GPIO.output(config.STATUS_LED_PIN, GPIO.HIGH if self._blink_state else GPIO.LOW)
                    self._last_toggle = now
            elif self.status == "saving":
                GPIO.output(config.STATUS_LED_PIN, GPIO.HIGH)
                self._blink_state = True
            time.sleep(0.05)

        GPIO.output(config.STATUS_LED_PIN, GPIO.LOW)
        print("[THREAD] LED Thread beendet")

    @staticmethod
    def beep(duration: float = 0.1, count: int = 1) -> None:
        if not config.BUZZER_ENABLED:
            return
        try:
            for i in range(count):
                GPIO.output(config.BUZZER_PIN, GPIO.HIGH)
                time.sleep(duration)
                GPIO.output(config.BUZZER_PIN, GPIO.LOW)
                if i < count - 1:
                    time.sleep(duration * 0.5)
        except Exception as e:
            logger.warning(f"Buzzer Fehler: {e}")

    @staticmethod
    def all_off() -> None:
        if config.LED_ENABLED:
            try:
                GPIO.output(config.STATUS_LED_PIN, GPIO.LOW)
            except Exception:
                pass
        if config.BUZZER_ENABLED:
            try:
                GPIO.output(config.BUZZER_PIN, GPIO.LOW)
            except Exception:
                pass
