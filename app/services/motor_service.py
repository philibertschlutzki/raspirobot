
class MotorService:
def __init__(self, gpio):
self.gpio = gpio

    def set_speed(self, left_percent, right_percent):
        self.gpio.set_pwm('left', left_percent)
        self.gpio.set_pwm('right', right_percent)
    ```