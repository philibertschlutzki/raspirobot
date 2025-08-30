# safety.py (vereinfacht, nur Polling)
import time
import RPi.GPIO as GPIO

def setup_buttons(confirm_pin, estop_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(confirm_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(estop_pin,   GPIO.IN, pull_up_down=GPIO.PUD_UP)

def confirm_pressed(confirm_pin):
    return GPIO.input(confirm_pin) == GPIO.LOW  # aktiv-LOW

def estop_active(estop_pin):
    return GPIO.input(estop_pin) == GPIO.LOW    # aktiv-LOW

def wait_for_confirmation(confirm_pin, estop_pin, timeout_s=5.0):
    start = time.time()
    while time.time() - start < timeout_s:
        if estop_active(estop_pin):
            return False
        if confirm_pressed(confirm_pin):
            # einfache Entprellung + Loslass-Wartezeit
            time.sleep(0.03)
            while confirm_pressed(confirm_pin):
                time.sleep(0.02)
            return True
        time.sleep(0.02)
    return False
