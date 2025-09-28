#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys
from rpi_hardware_pwm import HardwarePWM
from rplidar import RPLidar, RPLidarException

# Ultraschallsensor-Pins
TRIG_LEFT, ECHO_LEFT = 12, 13
TRIG_RIGHT, ECHO_RIGHT = 23, 24
SOUND_SPEED = 34300

# LIDAR
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 460800
EMERGENCY_MIN_POINTS = 10       # Minimum Punkte pro Sektor
EMERGENCY_MAX_DIST_MM = 12000   # Maximalmessung

# Motorkontrolle wie im Original
pwm_left = HardwarePWM(pwm_channel=0, hz=1000, chip=0)
pwm_right = HardwarePWM(pwm_channel=1, hz=1000, chip=0)

def measure_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
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
    return (pulse_duration * SOUND_SPEED) / 2

def init_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([TRIG_LEFT, TRIG_RIGHT], GPIO.OUT)
    GPIO.setup([ECHO_LEFT, ECHO_RIGHT], GPIO.IN)
    GPIO.output(TRIG_LEFT, False)
    GPIO.output(TRIG_RIGHT, False)
    pwm_left.start(0)
    pwm_right.start(0)

def set_motor_speed(left, right):
    pwm_left.change_duty_cycle(left)
    pwm_right.change_duty_cycle(right)

def lidar_sector_analysis(scan, min_points=EMERGENCY_MIN_POINTS, max_dist=EMERGENCY_MAX_DIST_MM):
    # Winkelbereiche: vorne=340-20, links=60-120, rechts=240-300
    sectors = {'front': [], 'left': [], 'right': []}
    for qual, ang, dist in scan:
        if dist <= 0 or dist > max_dist:
            continue
        if (ang >= 340 or ang <= 20): sectors['front'].append(dist)
        elif (60 <= ang <= 120):      sectors['left'].append(dist)
        elif (240 <= ang <= 300):     sectors['right'].append(dist)
    sector_safe = {}
    for key, points in sectors.items():
        # Check auf Minimum und Plausibilität (z.B. alle Werte ≈ max_dist = abgeschnitten)
        if len(points) < min_points or all(d > 0.8*max_dist for d in points): 
            sector_safe[key] = False
        else:
            sector_safe[key] = min(points) > 250  # „Freie“ Fahrt ab 25 cm Abstand
    return sector_safe

def main():
    init_gpio()
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
    try:
        lidar.start_motor()
        for scan in lidar.iter_scans():
            sectors = lidar_sector_analysis(scan)
            if all(sectors[k] for k in sectors):
                set_motor_speed(12, 12)
            elif not sectors['left']:
                set_motor_speed(10, 0)
            elif not sectors['right']:
                set_motor_speed(0, 10)
            else:
                # Invalid LIDAR: Fallback auf Ultraschall
                d_l = measure_distance(TRIG_LEFT, ECHO_LEFT)
                d_r = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
                if d_l is None or d_r is None or d_l < 25 or d_r < 25:
                    set_motor_speed(0, 0)
                elif d_l < 80:
                    set_motor_speed(10, 0)
                elif d_r < 80:
                    set_motor_speed(0, 10)
                else:
                    set_motor_speed(8, 8)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pwm_left.stop()
        pwm_right.stop()
        lidar.stop()
        lidar.disconnect()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
