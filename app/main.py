
import yaml
import threading
from app.hardware.gpio_interface import GPIOInterface
from app.hardware.ultrasonic import UltrasonicSensor
from app.hardware.microwave import MicrowaveSensor
from app.services.sensor_service import SensorService
from app.services.motor_service import MotorService
from app.services.mode_controller import ModeController
from app.web.server import run_server

def main():
# 1. GPIO-Pins laden
with open("config/gpio_pins.yaml") as f:
pins = yaml.safe_load(f)

    # 2. Hardware-Interfaces initialisieren
    gpio = GPIOInterface(pins)
    ultrasonic = UltrasonicSensor(gpio, pins['ultrasonic'])
    microwave = MicrowaveSensor(gpio, pins['microwave'])
    motor_svc = MotorService(gpio)
    
    # 3. Services starten
    sensor_svc = SensorService(ultrasonic, microwave)
    sensor_svc.start()
    mode_ctrl = ModeController(sensor_svc, motor_svc)
    mode_ctrl.start()
    
    # 4. Webserver starten (blockierend)
    run_server(sensor_svc, mode_ctrl)
    if __name__ == "__main__":
main()
