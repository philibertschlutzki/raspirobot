#!/usr/bin/env python3
import time
from threading import Thread, Lock
from hardware.ultrasonic import Ultrasonic
from hardware.microwave import MicrowaveSensor
from config.gpio_pins import read_pins
from hardware.gpio_interface import GPIOInterface

class SensorService(Thread):
    def __init__(self):
        super().__init__(daemon=True)
        pins = read_pins()
        gpio = GPIOInterface()
        # Frontsensor
        self.ultrasonic_front = Ultrasonic(
            trigger_pin=pins["ultrasonic_trigger"],
            echo_pin=pins["ultrasonic_echo"]
        )
        # Rechter Sensor
        self.ultrasonic_right = Ultrasonic(
            trigger_pin=pins["ultrasonic_trigger_right"],
            echo_pin=pins["ultrasonic_echo_right"]
        )
        # Linker Sensor
        self.ultrasonic_left = Ultrasonic(
            trigger_pin=pins["ultrasonic_trigger_left"],
            echo_pin=pins["ultrasonic_echo_left"]
        )
        # Mikrowellensensor
        self.microwave = MicrowaveSensor(gpio, pins["microwave_pin"])

        self.data_lock = Lock()
        self.data = {}

    def run(self):
        while True:
            front = self.ultrasonic_front.read_distance()
            right = self.ultrasonic_right.read_distance()
            left  = self.ultrasonic_left.read_distance()
            motion = self.microwave.detect_motion()
            with self.data_lock:
                self.data = {
                    "front":  front,
                    "right":  right,
                    "left":   left,
                    "person": motion
                }
            time.sleep(0.05)  # 20 Hz

    def get_data(self):
        """Thread-sichere Abfrage aller Sensordaten."""
        with self.data_lock:
            return dict(self.data)
