#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import signal
import sys
from rpi_hardware_pwm import HardwarePWM

# Sensor Pins
TRIG_LEFT = 12
ECHO_LEFT = 13
TRIG_RIGHT = 23
ECHO_RIGHT = 24

# Schallgeschwindigkeit in cm/s
SOUND_SPEED = 34300

# PWM für Motoren: links Kanal 0, rechts Kanal 1, 1000 Hz
pwm_left = HardwarePWM(pwm_channel=0, hz=1000, chip=0)
pwm_right = HardwarePWM(pwm_channel=1, hz=1000, chip=0)

def signal_handler(sig, frame):
    print('\nProgramm wird beendet...')
    cleanup()
    sys.exit(0)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    pwm_left.start(0)
    pwm_right.start(0)
    GPIO.output(TRIG_LEFT, False)
    GPIO.output(TRIG_RIGHT, False)
    signal.signal(signal.SIGINT, signal_handler)
    time.sleep(2)  # Sensoren initialisieren

def measure_distance(trig_pin, echo_pin):
    try:
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)  # 10 Mikrosekunden Triggerimpuls
        GPIO.output(trig_pin, False)

        timeout = time.time() + 0.1
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None

        timeout = time.time() + 0.1
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None

        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * SOUND_SPEED) / 2
        return distance
    except Exception as e:
        print(f"Fehler bei der Messung: {e}")
        return None

def set_motor_speed(left_speed, right_speed):
    # Geschwindigkeit in Prozent (0-100)
    pwm_left.change_duty_cycle(left_speed)
    pwm_right.change_duty_cycle(right_speed)

def cleanup():
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    print("GPIO bereinigt.")

def main():
    setup()
    print("Starte Steuerung basierend auf Sensordaten...")
    try:
        while True:
            dist_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
            dist_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)

            if dist_left is None or dist_right is None:
                print("Fehler bei der Distanzmessung, überspringe Messung.")
                time.sleep(0.5)
                continue

            print(f"Links: {dist_left:.1f} cm, Rechts: {dist_right:.1f} cm")

            # Stoppen, wenn Hindernis sehr nah (<= 10 cm)
            if dist_left <= 10 or dist_right <= 10:
                print("Hindernis zu nah, stoppe Motoren.")
                set_motor_speed(0, 0)

            # Geradeaus, wenn beide größer 80 cm
            elif dist_left > 80 and dist_right > 80:
                print("Fahre geradeaus.")
                set_motor_speed(12, 12)

            # Drehen: Wenn links <= 80cm, drehe nach rechts
            elif dist_left <= 80:
                print("Hindernis links, drehe nach rechts.")
                set_motor_speed(10, 0)

            # Wenn rechts <= 80cm, drehe nach links
            elif dist_right <= 80:
                print("Hindernis rechts, drehe nach links.")
                set_motor_speed(0, 10)

            time.sleep(0.5)

    except KeyboardInterrupt:
        signal_handler(None, None)

if __name__ == "__main__":
    main()
