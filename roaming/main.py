# main.py
#!/usr/bin/env python3
import time, signal, sys
import RPi.GPIO as GPIO
from sensors import setup_sensors, read_pair
from filters import FilteredDistance
from decisions import decide
from safety import setup_buttons, wait_for_confirmation, estop_active
import actuators

# Pins (BCM)
LEFT = (12, 13)    # TRIG, ECHO
RIGHT = (23, 24)   # TRIG, ECHO
CONFIRM_PIN = 5
ESTOP_PIN = 6

def cleanup_all():
    try:
        actuators.run_command("HALT")
    except Exception:
        pass
    actuators.cleanup_motors()
    try:
        GPIO.cleanup()
    except Exception:
        pass

def main():
    dry = "--dry-run" in sys.argv
    actuators.DRY_RUN = dry
    print(f"Dry-Run: {dry}")

    left, right = setup_sensors(LEFT, RIGHT)
    setup_buttons(CONFIRM_PIN, ESTOP_PIN)
    actuators.setup_motors()

    fl, fr = FilteredDistance(5), FilteredDistance(5)

    running = True
    def on_sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_sigint)

    try:
        while running:
            if estop_active(ESTOP_PIN):
                print("[SAFETY] E-STOP -> HALT")
                actuators.run_command("HALT")
                while estop_active(ESTOP_PIN) and running:
                    time.sleep(0.05)
                continue

            dL, dR = read_pair(left, right, delay_between=0.06)
            fl.push(dL); fr.push(dR)
            fL, fR = fl.value(), fr.value()
            print(f"[{time.strftime('%H:%M:%S')}] dL={None if fL is None else round(fL,1)} cm | dR={None if fR is None else round(fR,1)} cm")

            cmd = decide(fL, fR)
            if cmd in ("FORWARD_SHORT", "BACK_SHORT", "TURN_LEFT", "TURN_RIGHT"):
                print(f"[PENDING] {cmd} -> bestätigen (5s)...")
                if wait_for_confirmation(CONFIRM_PIN, ESTOP_PIN, timeout_s=5.0):
                    actuators.run_command(cmd)
                else:
                    print("[PENDING] abgebrochen/Timeout -> HALT")
                    actuators.run_command("HALT")
            elif cmd == "HALT":
                actuators.run_command("HALT")

            time.sleep(0.02)
    finally:
        print("[CLEANUP] Aufräumen...")
        cleanup_all()

if __name__ == "__main__":
    main()
