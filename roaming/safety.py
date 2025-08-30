# safety.py

import time
import RPi.GPIO as GPIO

def setup_buttons(confirm_pin, estop_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(confirm_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(estop_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def confirm_pressed(confirm_pin):
    return GPIO.input(confirm_pin) == GPIO.LOW  # aktiv-LOW

def estop_active(estop_pin):
    return GPIO.input(estop_pin) == GPIO.LOW  # aktiv-LOW

def wait_for_confirmation(confirm_pin, estop_pin, timeout_s=5.0, allow_enter=True):
    """
    Wartet bis zu timeout_s Sekunden auf:
    - Hardware-Bestätigungstaste (aktiv-LOW, mit Entprellung und Loslass-Wartezeit)
    - optional: Enter-Taste im Terminal (stdin), nicht-blockierend
    Bricht sofort ab (False), wenn E-Stop aktiv ist.
    """
    import sys, select

    start = time.time()
    while time.time() - start < timeout_s:
        # E-Stop hat immer Vorrang
        if estop_active(estop_pin):
            return False

        # Optional: Enter im Terminal als Bestätigung
        if allow_enter:
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], 0)
                if sys.stdin in rlist:
                    _ = sys.stdin.readline()  # gesamte Zeile inkl. '\n' konsumieren
                    return True
            except Exception:
                # Falls kein TTY/kein stdin vorhanden ist, still weiterpolling
                pass

        # Hardware-Taster prüfen (mit Entprellung und Loslass-Wartezeit)
        if confirm_pressed(confirm_pin):
            time.sleep(0.03)  # Entprellung
            while confirm_pressed(confirm_pin):
                if estop_active(estop_pin):
                    return False
                time.sleep(0.02)
            return True

        time.sleep(0.02)

    return False
