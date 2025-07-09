#!/usr/bin/env python3
import asyncio
from hardware.gpio_interface import GPIOInterface

class MotorService:
    def __init__(self):
        self.gpio = GPIOInterface()
        self.stopped = True

    async def forward(self):
        if self.stopped:
            self.gpio.set_motor(direction="forward")
            self.stopped = False

    async def stop(self):
        if not self.stopped:
            self.gpio.set_motor(direction="stop")
            self.stopped = True

    async def turn_right(self):
        self.gpio.set_motor(direction="right")
        self.stopped = False

    async def turn_left(self):
        self.gpio.set_motor(direction="left")
        self.stopped = False
