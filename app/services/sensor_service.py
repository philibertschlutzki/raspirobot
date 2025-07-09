from hardware.ultrasonic import Ultrasonic

class SensorService:
    def __init__(self):
        # Pin-Konfiguration aus YAML laden
        from config.gpio_pins import read_pins
        pins = read_pins()
        self.sensor = Ultrasonic(trigger_pin=pins["ultrasonic_trigger"],
                                  echo_pin=pins["ultrasonic_echo"])

    def get_distance(self):
        # Entfernung in cm
        return self.sensor.distance()
