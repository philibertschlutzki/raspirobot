from gpio_interface import GpioInterface, FORWARD, BACKWARD

class MotorService:
    def __init__(self, gpio_interface: GpioInterface):
        self.gpio = gpio_interface

    def forward(self, speed: int):
        """Fährt beide Motoren vorwärts mit gegebener Geschwindigkeit (0–100)."""
        for side in ("left", "right"):
            self.gpio.set_direction(side, FORWARD)
            self.gpio.set_pwm(side, speed)
            self.gpio.enable_motor(side)

    def backward(self, speed: int):
        """Fährt beide Motoren rückwärts."""
        for side in ("left", "right"):
            self.gpio.set_direction(side, BACKWARD)
            self.gpio.set_pwm(side, speed)
            self.gpio.enable_motor(side)

    def stop(self):
        """Stoppt beide Motoren."""
        for side in ("left", "right"):
            self.gpio.disable_motor(side)
            self.gpio.set_pwm(side, 0)

    def turn_left(self, speed: int):
        """Dreht nach links (rechter Motor vorwärts, linker rückwärts)."""
        self.gpio.set_direction("left", BACKWARD)
        self.gpio.set_pwm("left", speed)
        self.gpio.enable_motor("left")
        self.gpio.set_direction("right", FORWARD)
        self.gpio.set_pwm("right", speed)
        self.gpio.enable_motor("right")

    def turn_right(self, speed: int):
        """Dreht nach rechts (linker Motor vorwärts, rechter rückwärts)."""
        self.gpio.set_direction("left", FORWARD)
        self.gpio.set_pwm("left", speed)
        self.gpio.enable_motor("left")
        self.gpio.set_direction("right", BACKWARD)
        self.gpio.set_pwm("right", speed)
        self.gpio.enable_motor("right")
