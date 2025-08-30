#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autonomy_controller.py

Autonomer, sicherer Fahr-Controller auf Basis von zwei HC-SR04 (links/rechts)
und den bestehenden Fahrprimitiven aus back_180_fwd.py mit Human-in-the-Loop-Bestätigung
und Not-Aus.

- Sequentielles Triggern der beiden Ultraschallsensoren zur Reduktion von Übersprechen
- Median-/Mittel-Glättung und Plausibilitätsprüfung (2..400 cm)
- Zustandsautomat: EVAL -> PENDING (Bestätigung) -> EXEC -> IDLE/HALT
- Safety: Not-Aus (E-Stop), Timeout in Pending, Fail-Safe-Stopp bei Sensorfehlern
- Dry-Run: Keine Motoransteuerung, nur Logik/Entscheidungen

Verwendet:
- RPi.GPIO
- back_180_fwd.py (vorhandene PWM-/Rampen-Primitive)

"""

import time
import sys
import signal
from collections import deque

import RPi.GPIO as GPIO

# ------------------------------
# Konfiguration (BCM-Pinbelegung)
# ------------------------------

# HC-SR04 links (gemäß bereitgestelltem left-Script)
TRIG_LEFT = 12
ECHO_LEFT = 13

# HC-SR04 rechts (gemäß bereitgestelltem right-Script)
TRIG_RIGHT = 23
ECHO_RIGHT = 24

# Bestätigungs-Taster (aktiv LOW mit Pull-Up)
CONFIRM_BTN_PIN = 5

# Not-Aus (aktiv LOW mit Pull-Up)
ESTOP_BTN_PIN = 6

# Mess-/Filter-Parameter
SOUND_SPEED = 34300.0  # cm/s
MIN_VALID_CM = 2.0
MAX_VALID_CM = 400.0
MEASURE_DELAY_BETWEEN_SENSORS = 0.06  # s, zur Crosstalk-Reduktion
FILTER_WINDOW = 5  # Medianfenster
LOOP_SLEEP = 0.02  # s, Hauptschleifen-Pause

# Entscheidungs-Schwellen (zu kalibrieren)
DIST_TOO_CLOSE = 25.0  # cm
DIST_CLOSE = 50.0      # cm
SIDE_DIFF = 10.0       # cm, asymmetrische Schwelle

# Pending/Confirmation
CONFIRM_TIMEOUT_S = 5.0

# Dry-Run (True => keine Motoren)
DRY_RUN = False

# ------------------------------
# Motor/Primitive (lazy import)
# ------------------------------

pwm_left = None
pwm_right = None

def motors_setup():
    """Initialisiert Motor-PWM und Richtungs-Pins via back_180_fwd.py."""
    global pwm_left, pwm_right
    if DRY_RUN:
        return
    # Lazy import, um Dry-Run ohne Bibliotheken zu ermöglichen
    import back_180_fwd as drv
    pwm_left, pwm_right = drv.setup()

def motors_cleanup():
    """Stoppt PWM sicher und räumt GPIO für Motorpins auf."""
    if DRY_RUN:
        return
    try:
        if pwm_left:
            pwm_left.stop()
        if pwm_right:
            pwm_right.stop()
    except Exception:
        pass

def run_primitive(cmd: str):
    """Mappt symbolische Kommandos auf die sanften Fahr-Primitive."""
    print(f"[ACT] Ausführen: {cmd}")
    if DRY_RUN:
        time.sleep(0.5)
        return
    import back_180_fwd as drv
    # Parameter aus vorhandenem Script übernehmen
    SPEED_DUTY = drv.SPEED_DUTY
    TURN_DUTY = drv.TURN_DUTY
    # Drehdauer konservativ kürzen für Feinpositionierung
    TURN_SHORT_S = max(0.5, drv.TURN_DURATION_S * 0.6)
    FWD_SHORT_S = 1.0
    BACK_SHORT_S = 0.8

    if cmd == "FORWARD_SHORT":
        drv.soft_run_section(pwm_left, pwm_right, backward=False,
                             duration_s=FWD_SHORT_S, target_duty=SPEED_DUTY)
    elif cmd == "BACK_SHORT":
        drv.soft_run_section(pwm_left, pwm_right, backward=True,
                             duration_s=BACK_SHORT_S, target_duty=SPEED_DUTY)
    elif cmd == "TURN_LEFT":
        drv.soft_turn_section(pwm_left, pwm_right, clockwise=False,
                              duration_s=TURN_SHORT_S, target_duty=TURN_DUTY)
    elif cmd == "TURN_RIGHT":
        drv.soft_turn_section(pwm_left, pwm_right, clockwise=True,
                              duration_s=TURN_SHORT_S, target_duty=TURN_DUTY)
    elif cmd == "HALT":
        # Soft-Stop: beide Duty auf 0% via kurzer Rampe
        drv.soft_run_section(pwm_left, pwm_right, backward=False,
                             duration_s=0.0, target_duty=0)
    else:
        print(f"[WARN] Unbekanntes Kommando: {cmd}")

# ------------------------------
# HC-SR04 Sensor-Klasse
# ------------------------------

class HCSR04:
    def __init__(self, trig_pin: int, echo_pin: int, name: str):
        self.trig = trig_pin
        self.echo = echo_pin
        self.name = name
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.05)

    def measure_once(self, timeout_s: float = 0.1):
        """Einzelmessung mit Timeout; Ergebnis in cm oder None."""
        # Trigger 10 µs HIGH
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        # Auf steigende Flanke warten
        t0 = time.time()
        while GPIO.input(self.echo) == 0:
            if (time.time() - t0) > timeout_s:
                return None
        pulse_start = time.time()

        # Auf fallende Flanke warten
        t1 = time.time()
        while GPIO.input(self.echo) == 1:
            if (time.time() - t1) > timeout_s:
                return None
        pulse_end = time.time()

        pulse_dur = pulse_end - pulse_start  # s
        distance_cm = (pulse_dur * SOUND_SPEED) / 2.0
        return distance_cm

# ------------------------------
# Filter-/Fusion-Helfer
# ------------------------------

def median_of(seq):
    vals = sorted(seq)
    n = len(vals)
    if n == 0:
        return None
    if n % 2 == 1:
        return vals[n // 2]
    return 0.5 * (vals[n // 2 - 1] + vals[n // 2])

class FilteredDistance:
    def __init__(self, window: int):
        self.buf = deque(maxlen=window)

    def push(self, v):
        if v is not None and (MIN_VALID_CM <= v <= MAX_VALID_CM):
            self.buf.append(v)

    def value(self):
        usable = list(self.buf)
        if not usable:
            return None
        # Median gegen Ausreißer
        return median_of(usable)

# ------------------------------
# GPIO Setup / Buttons
# ------------------------------

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # Sensor-Pins in Klassen gesetzt
    # Taster: Pull-Ups, aktiv LOW
    GPIO.setup(CONFIRM_BTN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ESTOP_BTN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Entprellte Events
    GPIO.add_event_detect(CONFIRM_BTN_PIN, GPIO.FALLING, bouncetime=200)
    GPIO.add_event_detect(ESTOP_BTN_PIN, GPIO.FALLING, bouncetime=50)

def cleanup_gpio():
    try:
        GPIO.cleanup()
    except Exception:
        pass

def confirm_pressed():
    # aktiv LOW
    return GPIO.input(CONFIRM_BTN_PIN) == GPIO.LOW

def estop_active():
    # aktiv LOW
    return GPIO.input(ESTOP_BTN_PIN) == GPIO.LOW

def wait_for_confirmation(timeout_s: float):
    """Wartet bis Confirm gedrückt oder Timeout; True bei Freigabe."""
    start = time.time()
    while (time.time() - start) < timeout_s:
        if confirm_pressed():
            # Warten bis losgelassen, um Doppeltrigger zu vermeiden
            while confirm_pressed():
                time.sleep(0.02)
            return True
        # Abbruch, wenn E-Stop
        if estop_active():
            return False
        time.sleep(0.02)
    return False

# ------------------------------
# Entscheidung / Zustandsautomat
# ------------------------------

def decide(d_left, d_right):
    """
    Liefert symbolisches Kommando aus gefilterten Distanzen.
    Gibt eine der Optionen zurück:
      - "FORWARD_SHORT", "BACK_SHORT", "TURN_LEFT", "TURN_RIGHT", "HALT", "NONE"
    """
    if d_left is None or d_right is None:
        return "HALT"

    # Sehr nahe Hindernisse beidseitig -> Rückzug
    if d_left < DIST_TOO_CLOSE and d_right < DIST_TOO_CLOSE:
        return "BACK_SHORT"

    # Asymmetrische Annäherung -> wegdrehen
    if d_left < DIST_CLOSE and (d_right - d_left) >= SIDE_DIFF:
        return "TURN_RIGHT"
    if d_right < DIST_CLOSE and (d_left - d_right) >= SIDE_DIFF:
        return "TURN_LEFT"

    # Frei -> kurz vorwärts
    if d_left > DIST_CLOSE and d_right > DIST_CLOSE:
        return "FORWARD_SHORT"

    # Unscharfer Bereich -> vorsichtshalber nur Halt verlangen
    return "HALT"

def main():
    global DRY_RUN
    # Option für Dry-Run via CLI
    if "--dry-run" in sys.argv:
        DRY_RUN = True

    print("=== Autonomer Fahr-Controller mit Bestätigung & Not-Aus ===")
    print(f"Dry-Run: {DRY_RUN}")

    setup_gpio()
    left = HCSR04(TRIG_LEFT, ECHO_LEFT, "left")
    right = HCSR04(TRIG_RIGHT, ECHO_RIGHT, "right")
    fl = FilteredDistance(FILTER_WINDOW)
    fr = FilteredDistance(FILTER_WINDOW)

    # Motoren erst nach erfolgreichem GPIO-Setup starten
    motors_setup()

    running = True

    def on_sigint(sig, frame):
        nonlocal running
        print("\n[INFO] Beenden angefordert...")
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    try:
        while running:
            # Not-Aus überwachen
            if estop_active():
                print("[SAFETY] NOT-AUS aktiv -> Halt")
                run_primitive("HALT")
                # Warten, bis E-Stop freigegeben
                while estop_active() and running:
                    time.sleep(0.05)
                # Nach Freigabe weiter
                continue

            # Sequentiell messen (Crosstalk minimieren)
            dL = left.measure_once()
            time.sleep(MEASURE_DELAY_BETWEEN_SENSORS)
            dR = right.measure_once()

            # Filter füttern
            fl.push(dL)
            fr.push(dR)
            fL = fl.value()
            fR = fr.value()

            # Status
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] dL={None if fL is None else round(fL,1)} cm | dR={None if fR is None else round(fR,1)} cm")

            # Entscheiden
            cmd = decide(fL, fR)
            if cmd == "NONE":
                time.sleep(LOOP_SLEEP)
                continue

            # „Pending“-Phase: manuelle Freigabe einholen
            if cmd in ("FORWARD_SHORT", "BACK_SHORT", "TURN_LEFT", "TURN_RIGHT"):
                print(f"[PENDING] Geplantes Kommando: {cmd} -> Bitte bestätigen (Timeout {CONFIRM_TIMEOUT_S:.0f}s)")
                granted = wait_for_confirmation(CONFIRM_TIMEOUT_S)
                if not granted:
                    print("[PENDING] Keine Freigabe -> verwerfen, Halt")
                    run_primitive("HALT")
                    time.sleep(0.2)
                    continue
                # Ausführen
                run_primitive(cmd)
            elif cmd == "HALT":
                run_primitive("HALT")

            time.sleep(LOOP_SLEEP)

    finally:
        print("[CLEANUP] Stoppe Motoren und räume GPIO auf...")
        try:
            run_primitive("HALT")
        except Exception:
            pass
        motors_cleanup()
        cleanup_gpio()
        print("[CLEANUP] Fertig.")

if __name__ == "__main__":
    main()
