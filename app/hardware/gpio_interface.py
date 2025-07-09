#!/usr/bin/env python3
import RPi.GPIO as GPIO
from config.gpio_pins import read_pins

class GPIOInterface:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        pins = read_pins()
        # Motor-Pins initialisieren
        GPIO.setup(pins["motor_pin_forward"], GPIO.OUT)
        GPIO.setup(pins["motor_pin_stop"],    GPIO.OUT)
        GPIO.setup(pins["motor_pin_right"],   GPIO.OUT)
        GPIO.setup(pins["motor_pin_left"],    GPIO.OUT)
        self.pins = pins

    def set_motor(self, direction):
        # Alle Motor-Pins ausschalten
        for pin in (
            self.pins["motor_pin_forward"],
            self.pins["motor_pin_stop"],
            self.pins["motor_pin_right"],
            self.pins["motor_pin_left"]
        ):
            GPIO.output(pin, GPIO.LOW)

        # Gewünschte Richtung aktivieren
        if direction == "forward":
            GPIO.output(self.pins["motor_pin_forward"], GPIO.HIGH)
        elif direction == "stop":
            GPIO.output(self.pins["motor_pin_stop"], GPIO.HIGH)
        elif direction == "right":
            GPIO.output(self.pins["motor_pin_right"], GPIO.HIGH)
        elif direction == "left":
            GPIO.output(self.pins["motor_pin_left"], GPIO.HIGH)

    def read_pin(self, pin):
        GPIO.setup(pin, GPIO.IN)
        return GPIO.input(pin)
