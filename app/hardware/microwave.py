#!/usr/bin/env python3
class MicrowaveSensor:
    def __init__(self, gpio, pin):
        self.gpio = gpio
        self.pin = pin

    def detect_motion(self) -> bool:
        """Gibt True zurück, wenn Bewegung erkannt wurde."""
        return bool(self.gpio.read_pin(self.pin))
