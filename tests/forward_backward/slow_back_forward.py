#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

# Pinbelegung (aus vorhandenem Skript)
DIR_LEFT_PIN = 20   # Richtung links
DIR_RIGHT_PIN = 21  # Richtung rechts

# Hardware-PWM (Pi 5: Originalskript nutzt chip=0 und Kanäle 0/1 -> GPIO 18/19)
PWM_LEFT_CHANNEL = 0
PWM_RIGHT_CHANNEL = 1
PWM_CHIP = 0

# Konfiguration
FREQUENCY_HZ = 1000
SPEED_DUTY = 10      # % Duty-Cycle für Fahrt
DURATION_S = 2       # Sekunden pro Fahrabschnitt
DIR_HIGH_IS_BACKWARD = True  # HIGH = rückwärts (falls invertiert: False)

# Sanftes Fahren
RAMP_STEP = 1          # Schrittweite in % (z.B. 1%)
RAMP_STEP_DELAY = 0.2  # Verzögerung pro Schritt in s (z.B. 0.2s)
STOP_PAUSE_S = 0.5     # Pause im Stillstand vor Richtungswechsel

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
    GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)

    # Richtung initial auf vorwärts setzen
    if DIR_HIGH_IS_BACKWARD:
        GPIO.output(DIR_LEFT_PIN, GPIO.LOW)
        GPIO.output(DIR_RIGHT_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_LEFT_PIN, GPIO.HIGH)
        GPIO.output(DIR_RIGHT_PIN, GPIO.HIGH)

    pwm_left = HardwarePWM(pwm_channel=PWM_LEFT_CHANNEL, hz=FREQUENCY_HZ, chip=PWM_CHIP)
    pwm_right = HardwarePWM(pwm_channel=PWM_RIGHT_CHANNEL, hz=FREQUENCY_HZ, chip=PWM_CHIP)
    pwm_left.start(0)
    pwm_right.start(0)
    return pwm_left, pwm_right

def set_direction(backward: bool):
    if DIR_HIGH_IS_BACKWARD:
        level = GPIO.HIGH if backward else GPIO.LOW
    else:
        level = GPIO.LOW if backward else GPIO.HIGH
    GPIO.output(DIR_LEFT_PIN, level)
    GPIO.output(DIR_RIGHT_PIN, level)

def soft_ramp(pwm_left, pwm_right, start: int, end: int, step: int = RAMP_STEP, delay_s: float = RAMP_STEP_DELAY):
    # Begrenzung und Richtung
    start = max(0, min(100, int(start)))
    end = max(0, min(100, int(end)))
    step = max(1, int(step))

    if start <= end:
        rng = range(start, end + 1, step)
    else:
        rng = range(start, end - 1, -step)

    for duty in rng:
        pwm_left.change_duty_cycle(duty)
        pwm_right.change_duty_cycle(duty)
        time.sleep(delay_s)

def soft_run_section(pwm_left, pwm_right, backward: bool, duration_s: int, target_duty: int):
    # Richtung setzen
    set_direction(backward)

    # Sanft anfahren: 0% -> target_duty
    soft_ramp(pwm_left, pwm_right, start=0, end=target_duty, step=RAMP_STEP, delay_s=RAMP_STEP_DELAY)

    # Konstante Fahrt
    time.sleep(max(0, duration_s))

    # Sanft stoppen: target_duty -> 0%
    soft_ramp(pwm_left, pwm_right, start=target_duty, end=0, step=RAMP_STEP, delay_s=RAMP_STEP_DELAY)

    # Kurze Pause im Stillstand
    time.sleep(STOP_PAUSE_S)

def main():
    print("=== Vor/Zurück-Fahrt (sanft) ===")
    pwm_left, pwm_right = setup()
    try:
        print("Sanft rückwärts anfahren, 5s fahren, sanft stoppen...")
        soft_run_section(pwm_left, pwm_right, backward=True, duration_s=DURATION_S, target_duty=SPEED_DUTY)

        print("Richtung wechseln, sanft vorwärts anfahren, 5s fahren, sanft stoppen...")
        soft_run_section(pwm_left, pwm_right, backward=False, duration_s=DURATION_S, target_duty=SPEED_DUTY)

        print("Fahrt abgeschlossen.")
    except KeyboardInterrupt:
        print("Abgebrochen.")
    finally:
        try:
            pwm_left.stop()
            pwm_right.stop()
        except Exception:
            pass
        GPIO.cleanup()
        print("Aufgeräumt.")

if __name__ == "__main__":
    main()
