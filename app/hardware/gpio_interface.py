import RPi.GPIO as GPIO
from config.gpio_pins import read_pins

class GPIOInterface:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        pins = read_pins()
        # Motorpins initialisieren
        GPIO.setup(pins["motor_pin_forward"], GPIO.OUT)
        GPIO.setup(pins["motor_pin_stop"], GPIO.OUT)
        self.pins = pins

    def set_motor(self, direction):
        if direction == "forward":
            GPIO.output(self.pins["motor_pin_forward"], GPIO.HIGH)
            GPIO.output(self.pins["motor_pin_stop"], GPIO.LOW)
        elif direction == "stop":
            GPIO.output(self.pins["motor_pin_forward"], GPIO.LOW)
            GPIO.output(self.pins["motor_pin_stop"], GPIO.HIGH)
