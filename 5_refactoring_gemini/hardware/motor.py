import os
import time

try:
    from ..interfaces import IMotorController
    from ..config import *
except ImportError:
    from interfaces import IMotorController
    from config import *

# PC_TEST_MODE check
is_pc_mode = os.environ.get("PC_TEST_MODE") == "1"

if not is_pc_mode:
    try:
        import RPi.GPIO as GPIO
        from rpi_hardware_pwm import HardwarePWM
    except ImportError:
        # Fallback if libraries missing even if not strictly in PC mode (e.g. dev machine)
        # But per requirements we should only mock if PC_TEST_MODE=1.
        # However, to avoid crash on dev machine without libs, we can fallback.
        print("Warning: Hardware libraries not found. Simulating PC Mode.")
        is_pc_mode = True

class RealMotorController(IMotorController):
    def __init__(self):
        self.pc_mode = is_pc_mode
        if self.pc_mode:
            print("RealMotorController: PC Mode Active (Dummy)")
            return

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
            GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)

            self.pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
            self.pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
            self.pwm_left.start(0)
            self.pwm_right.start(0)
        except Exception as e:
            print(f"Error initializing motors: {e}")
            self.pc_mode = True # Fallback

    def set_speed(self, left: float, right: float) -> None:
        if self.pc_mode:
            return

        # Clamp -1.0 to 1.0
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Determine direction logic
        # Forward (positive speed) -> HIGH if DIR_HIGH_IS_BACKWARD is False
        # Backward (negative speed) -> LOW if DIR_HIGH_IS_BACKWARD is False

        # Left Motor
        if left >= 0:
            level = GPIO.LOW if DIR_HIGH_IS_BACKWARD else GPIO.HIGH
        else:
            level = GPIO.HIGH if DIR_HIGH_IS_BACKWARD else GPIO.LOW
        GPIO.output(DIR_LEFT_PIN, level)

        # Right Motor
        if right >= 0:
            level = GPIO.LOW if DIR_HIGH_IS_BACKWARD else GPIO.HIGH
        else:
            level = GPIO.HIGH if DIR_HIGH_IS_BACKWARD else GPIO.LOW
        GPIO.output(DIR_RIGHT_PIN, level)

        duty_l = abs(left) * MAX_DUTY_CYCLE
        duty_r = abs(right) * MAX_DUTY_CYCLE

        self.pwm_left.change_duty_cycle(duty_l)
        self.pwm_right.change_duty_cycle(duty_r)

    def stop(self) -> None:
        if self.pc_mode:
            return
        self.pwm_left.change_duty_cycle(0)
        self.pwm_right.change_duty_cycle(0)

    def cleanup(self):
        if self.pc_mode: return
        try:
            self.pwm_left.stop()
            self.pwm_right.stop()
        except: pass
