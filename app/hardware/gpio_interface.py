import yaml
import RPi.GPIO as GPIO

FORWARD = 1
BACKWARD = 0

class GpioInterface:
    def __init__(self, config_path: str = "config/gpio_pins.yaml"):
        with open(config_path, 'r') as f:
            self.pins = yaml.safe_load(f)
        GPIO.setmode(GPIO.BCM)
        self._setup_all()

    def _setup_all(self):
        # Motoren
        for side in ("left", "right"):
            self.setup_motor_pins(side)
        # Ultraschall
        for sensor in self.pins['ultrasonic'].values():
            GPIO.setup(sensor['trigger'], GPIO.OUT)
            GPIO.setup(sensor['echo'], GPIO.IN)
        # Mikrowelle/weitere Pin
        GPIO.setup(self.pins['microwave_pin'], GPIO.OUT)

    def setup_motor_pins(self, side: str):
        m = self.pins['motors'][side]
        GPIO.setup(m['pwm_pin'], GPIO.OUT)
        GPIO.setup(m['direction_pin'], GPIO.OUT)
        GPIO.setup(m['enable_pin'], GPIO.OUT)
        # PWM-Initialisierung
        pwm = GPIO.PWM(m['pwm_pin'], 1000)
        pwm.start(0)
        setattr(self, f"pwm_{side}", pwm)

    def set_direction(self, side: str, direction: int):
        pin = self.pins['motors'][side]['direction_pin']
        GPIO.output(pin, GPIO.HIGH if direction == FORWARD else GPIO.LOW)

    def set_pwm(self, side: str, duty_cycle: int):
        pwm = getattr(self, f"pwm_{side}")
        pwm.ChangeDutyCycle(duty_cycle)

    def enable_motor(self, side: str):
        pin = self.pins['motors'][side]['enable_pin']
        GPIO.output(pin, GPIO.HIGH)

    def disable_motor(self, side: str):
        pin = self.pins['motors'][side]['enable_pin']
        GPIO.output(pin, GPIO.LOW)

    def cleanup(self):
        for side in ("left", "right"):
            getattr(self, f"pwm_{side}").stop()
        GPIO.cleanup()
