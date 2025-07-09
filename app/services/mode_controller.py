#!/usr/bin/env python3
import asyncio

class ModeController:
    def __init__(self, motor_service, sensor_service, mode="auto", interval=0.1):
        self.motor   = motor_service
        self.sensor  = sensor_service
        self.mode    = mode
        self.interval = interval
        self._stop   = False

    async def run(self):
        while not self._stop:
            data = self.sensor.get_data()
            front = data.get("front",  0.0)
            right = data.get("right",  0.0)
            left  = data.get("left",   0.0)

            if self.mode == "auto":
                # Hindernis vorwärts?
                if front < 30:
                    # Ausweichrichtung wählen
                    if right > left:
                        await self.motor.turn_right()
                    else:
                        await self.motor.turn_left()
                else:
                    await self.motor.forward()
            # Manual-Modus: Web-API steuert direkt MotorService

            await asyncio.sleep(self.interval)

    async def set_mode(self, mode: str):
        if mode in ("auto", "manual"):
            self.mode = mode

    async def stop(self):
        self._stop = True
