#!/usr/bin/env python3
import time
from rpi_hardware_pwm import HardwarePWM

# PWM auf GPIO 19 initialisieren pwm_channel=1
# Frequenz: 1000Hz, Chip: 2 (für Raspberry Pi 5)
pwm = HardwarePWM(pwm_channel=1, hz=1000, chip=0)

# PWM starten mit 0% Duty Cycle
pwm.start(0)

try:
    print("Motor-Test startet...")
    
# Langsam beschleunigen: 0% -> 10% in 1%-Schritten
    for duty_cycle in range(0, 11, 1):  # 0 bis 10%
        pwm.change_duty_cycle(duty_cycle)
        print(f"Duty Cycle: {duty_cycle}%")
        time.sleep(0.5)
    
    # Konstante Geschwindigkeit halten
    time.sleep(3)
    
    # Langsam abbremsen: 10% -> 0% in 1%-Schritten
    for duty_cycle in range(10, -1, -1):
        pwm.change_duty_cycle(duty_cycle)
        print(f"Duty Cycle: {duty_cycle}%")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Test abgebrochen")

finally:
    # PWM stoppen und aufräumen
    pwm.stop()
    print("Motor gestoppt")
