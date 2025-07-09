import asyncio

class ModeController:
    def __init__(self, motor_service, sensor_service, mode="auto", interval=0.1):
        self.motor = motor_service
        self.sensor = sensor_service
        self.mode = mode
        self.interval = interval
        self._stop = False

    async def run(self):
        while not self._stop:
            if self.mode == "auto":
                distance = self.sensor.get_distance()
                if distance < 20:
                    await self.motor.stop()
                else:
                    await self.motor.forward()
            # Im Manual-Modus steuert der Web-API
            await asyncio.sleep(self.interval)

    async def set_mode(self, mode):
        if mode in ("auto", "manual"):
            self.mode = mode

    async def stop(self):
        self._stop = True
