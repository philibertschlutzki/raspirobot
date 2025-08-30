# sensors.py
import time
import RPi.GPIO as GPIO

SOUND_SPEED = 34300.0  # cm/s
MIN_CM, MAX_CM = 2.0, 400.0

class HCSR04:
    def __init__(self, trig, echo, name=""):
        self.trig, self.echo, self.name = trig, echo, name
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.05)

    def measure_once(self, timeout_s=0.1):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)
        t0 = time.time()
        while GPIO.input(self.echo) == 0:
            if time.time() - t0 > timeout_s:
                return None
        start = time.time()
        t1 = time.time()
        while GPIO.input(self.echo) == 1:
            if time.time() - t1 > timeout_s:
                return None
        end = time.time()
        d_cm = (end - start) * SOUND_SPEED / 2.0
        return d_cm if MIN_CM <= d_cm <= MAX_CM else None

def setup_sensors(left_pins, right_pins):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    left = HCSR04(*left_pins, name="left")
    right = HCSR04(*right_pins, name="right")
    return left, right

def read_pair(left: HCSR04, right: HCSR04, delay_between=0.06):
    dL = left.measure_once()
    time.sleep(delay_between)
    dR = right.measure_once()
    return dL, dR
