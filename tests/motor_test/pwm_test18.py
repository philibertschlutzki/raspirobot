#!/usr/bin/env python3
import time
from rpi_hardware_pwm import HardwarePWM

# PWM auf GPIO 18 initialisieren
# Frequenz: 1000Hz, Chip: 2 (für Raspberry Pi 5)
pwm = HardwarePWM(pwm_channel=0, hz=1000, chip=0)

# PWM starten mit 0% Duty Cycle
pwm.start(0)

try:
    print("Motor-Test startet...")
    
    # Langsam beschleunigen
    for duty_cycle in range(0, 51, 5):  # 0-50% in 5% Schritten
        pwm.change_duty_cycle(duty_cycle)
        print(f"Duty Cycle: {duty_cycle}%")
        time.sleep(1)
    
    # Konstante Geschwindigkeit halten
    time.sleep(3)
    
    # Langsam abbremsen
    for duty_cycle in range(50, -1, -5):  # 50-0% in 5% Schritten
        pwm.change_duty_cycle(duty_cycle)
        print(f"Duty Cycle: {duty_cycle}%")
        time.sleep(1)

except KeyboardInterrupt:
    print("Test abgebrochen")

finally:
    # PWM stoppen und aufräumen
    pwm.stop()
    print("Motor gestoppt")
