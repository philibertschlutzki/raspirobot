import time
import RPi.GPIO as GPIO

class Ultrasonic:
    def __init__(self, trigger_pin, echo_pin):
        GPIO.setup(trigger_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        self.trigger = trigger_pin
        self.echo = echo_pin

    def distance(self):
        GPIO.output(self.trigger, GPIO.LOW)
        time.sleep(0.0002)
        GPIO.output(self.trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger, GPIO.LOW)

        start = time.time()
        while GPIO.input(self.echo) == 0:
            start = time.time()
        while GPIO.input(self.echo) == 1:
            end = time.time()
        duration = end - start
        # Schallgeschwindigkeit 34300 cm/s
        return (duration * 34300) / 2
