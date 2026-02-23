import time
import os
from typing import Dict

try:
    from ..interfaces import IUltrasonicSensor
    from ..config import TRIG_LEFT, ECHO_LEFT, TRIG_RIGHT, ECHO_RIGHT, SOUND_SPEED
except ImportError:
    from interfaces import IUltrasonicSensor
    from config import TRIG_LEFT, ECHO_LEFT, TRIG_RIGHT, ECHO_RIGHT, SOUND_SPEED

is_pc_mode = os.environ.get("PC_TEST_MODE") == "1"
if not is_pc_mode:
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("Warning: RPi.GPIO not found. Simulating PC Mode.")
        is_pc_mode = True

class RealUltrasonic(IUltrasonicSensor):
    def __init__(self):
        self.pc_mode = is_pc_mode
        if not self.pc_mode:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(TRIG_LEFT, GPIO.OUT)
                GPIO.setup(ECHO_LEFT, GPIO.IN)
                GPIO.setup(TRIG_RIGHT, GPIO.OUT)
                GPIO.setup(ECHO_RIGHT, GPIO.IN)
            except Exception as e:
                print(f"Error initializing ultrasonic: {e}")
                self.pc_mode = True

    def _measure(self, trig, echo):
        if self.pc_mode: return 1000.0

        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        start = time.time()
        stop = time.time()
        # 40ms timeout ~ 6.8m range (Sound speed ~343m/s => 34300cm/s)
        timeout = start + 0.04

        while GPIO.input(echo) == 0:
            start = time.time()
            if start > timeout: return 1000.0

        while GPIO.input(echo) == 1:
            stop = time.time()
            if stop > timeout: return 1000.0

        elapsed = stop - start
        return (elapsed * SOUND_SPEED) / 2

    def get_distances(self) -> Dict[str, float]:
        """Returns distance from left/right sensors in cm."""
        if self.pc_mode:
            return {"left": 1000.0, "right": 1000.0}

        d1 = self._measure(TRIG_LEFT, ECHO_LEFT)
        time.sleep(0.01) # Avoid interference
        d2 = self._measure(TRIG_RIGHT, ECHO_RIGHT)
        return {"left": d1, "right": d2}
