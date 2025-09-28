#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

# Pinbelegung (Annahmen gemäß vorhandener Skripte)
DIR_LEFT_PIN = 20    # Richtung links (aus switch_20_21.py)
DIR_RIGHT_PIN = 21   # Richtung rechts (aus switch_20_21.py)

PWM_LEFT_CHANNEL = 0    # Hardware-PWM Kanal 0 -> GPIO 18 (Pi 5)
PWM_RIGHT_CHANNEL = 1   # Hardware-PWM Kanal 1 -> GPIO 19 (Pi 5)
PWM_CHIP = 0            # Pi 5

# Konfiguration
FREQUENCY_HZ = 1000
SPEED_DUTY = 8         # % Duty-Cycle für Fahrt (anpassbar)
DURATION_S = 5          # Sekunden pro Fahrabschnitt
DIR_HIGH_IS_BACKWARD = True  # Annahme: HIGH = rückwärts (bei Bedarf invertieren)

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

def run_section(pwm_left, pwm_right, backward: bool, duration_s: int, duty: int):
    set_direction(backward)
    pwm_left.change_duty_cycle(duty)
    pwm_right.change_duty_cycle(duty)
    time.sleep(duration_s)
    pwm_left.change_duty_cycle(0)
    pwm_right.change_duty_cycle(0)

def main():
    print("=== Vor/Zurück-Fahrt (5s/5s) ===")
    pwm_left, pwm_right = setup()
    try:
        print("Fahre 5 Sekunden rückwärts...")
        run_section(pwm_left, pwm_right, backward=True, duration_s=DURATION_S, duty=SPEED_DUTY)
        time.sleep(0.5)
        print("Fahre 5 Sekunden vorwärts...")
        run_section(pwm_left, pwm_right, backward=False, duration_s=DURATION_S, duty=SPEED_DUTY)
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
