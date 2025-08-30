# safety.py
import time
import RPi.GPIO as GPIO

def setup_buttons(confirm_pin, estop_pin, confirm_bounce=200, estop_bounce=50):
    GPIO.setup(confirm_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(estop_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(confirm_pin, GPIO.FALLING, bouncetime=confirm_bounce)
    GPIO.add_event_detect(estop_pin, GPIO.FALLING, bouncetime=estop_bounce)

def confirm_pressed(confirm_pin):
    return GPIO.input(confirm_pin) == GPIO.LOW

def estop_active(estop_pin):
    return GPIO.input(estop_pin) == GPIO.LOW

def wait_for_confirmation(confirm_pin, estop_pin, timeout_s=5.0):
    start = time.time()
    while time.time() - start < timeout_s:
        if estop_active(estop_pin):
            return False
        if confirm_pressed(confirm_pin):
            while confirm_pressed(confirm_pin):
                time.sleep(0.02)
            return True
        time.sleep(0.02)
    return False
