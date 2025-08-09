#!/usr/bin/env python3

import time
from rpi_hardware_pwm import HardwarePWM

# PWM für beide Motoren initialisieren
# GPIO 18 (pwm_channel=0) für linkes Rad
# GPIO 19 (pwm_channel=1) für rechtes Rad
# Frequenz: 1000Hz, Chip: 0 (für Raspberry Pi 5)

pwm_left = HardwarePWM(pwm_channel=0, hz=1000, chip=0)   # GPIO 18
pwm_right = HardwarePWM(pwm_channel=1, hz=1000, chip=0)  # GPIO 19

# Beide PWM-Kanäle starten mit 0% Duty Cycle
pwm_left.start(0)
pwm_right.start(0)

try:
    print("Roboter-Test startet...")
    
    # Langsam anfahren: 0% -> 2% in 1%-Schritten
    for duty_cycle in range(0, 6, 1):  # 0 bis 2%
        pwm_left.change_duty_cycle(duty_cycle)
        pwm_right.change_duty_cycle(duty_cycle)
        print(f"Duty Cycle: {duty_cycle}% (beide Räder)")
        time.sleep(0.5)
    
    # Konstante Geschwindigkeit für 5 Sekunden bei 2%
    print("Fahre 60 Sekunden mit 2% Geschwindigkeit...")
    time.sleep(600)
    
    # Sanft stoppen: 2% -> 0% in 1%-Schritten
    for duty_cycle in range(2, -1, -1):
        pwm_left.change_duty_cycle(duty_cycle)
        pwm_right.change_duty_cycle(duty_cycle)
        print(f"Duty Cycle: {duty_cycle}% (beide Räder)")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Test abgebrochen")

finally:
    # Beide PWM-Kanäle stoppen und aufräumen
    pwm_left.stop()
    pwm_right.stop()
    print("Roboter gestoppt")
