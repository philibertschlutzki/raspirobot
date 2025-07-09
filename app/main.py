#!/usr/bin/env python3
import argparse
import asyncio
from services.mode_controller import ModeController
from services.motor_service import MotorService
from services.sensor_service import SensorService
from web.server import run_server

async def main():
    parser = argparse.ArgumentParser(description="RaspiRobot Starter")
    parser.add_argument("--mode", choices=["auto", "manual"], default="auto",
                        help="Betriebsmodus auswählen")
    parser.add_argument("--port", type=int, default=8080,
                        help="Port für HTTP-Server")
    args = parser.parse_args()

    motor = MotorService()
    sensor = SensorService()
    controller = ModeController(motor, sensor, mode=args.mode)

    # Starte Web-Server und Steuerung parallel
    await asyncio.gather(
        run_server(controller, host="0.0.0.0", port=args.port),
        controller.run()
    )

if __name__ == "__main__":
    asyncio.run(main())
# This script initializes the RaspiRobot system, setting up the motor and sensor services,