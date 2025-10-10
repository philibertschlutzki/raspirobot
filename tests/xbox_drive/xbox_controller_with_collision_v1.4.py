#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Xbox Controller Roboter-Steuerung mit Kollisionsvermeidung und Tank-Turn
==========================================================================
Dieses Script kombiniert die Xbox Controller Eingabe mit Ultraschall-Kollisionserkennung
für einen Raspberry Pi gesteuerten Roboter mit zwei Motoren.

NEU in Version 1.4 (BUGFIX):
- ✅ FEHLER 1 BEHOBEN: Richtungssteuerung komplett invertiert (FWD/REV korrekt)
- ✅ FEHLER 2 BEHOBEN: Tank-Turn beide Motoren drehen jetzt korrekt

Aus Version 1.3:
- Tank-Turn Funktion mit A-Button implementiert
- B-Button schaltet Fahrtrichtung um (vorwärts/rückwärts)
- DIR-Pins für Richtungssteuerung implementiert

HARDWARE ANFORDERUNGEN:
-----------------------
- Raspberry Pi (getestet mit Pi 5)
- Microsoft Xbox Wireless Controller mit USB Dongle
- 2x Ultraschall-Sensoren HC-SR04 (links/rechts)
- 2x Motoren mit PWM-Ansteuerung über HardwarePWM
- Motorcontroller mit Richtungs-Pins (ZS-X11H oder ähnlich)

SOFTWARE ABHÄNGIGKEITEN:
-----------------------
sudo apt install python3-pygame xboxdrv
sudo pip3 install rpi-hardware-pwm RPi.GPIO

HARDWARE VERKABELUNG:
--------------------
Ultraschall-Sensoren:
  Linker Sensor: TRIG=GPIO12, ECHO=GPIO13
  Rechter Sensor: TRIG=GPIO23, ECHO=GPIO24

Motor PWM (Hardware PWM):
  Linker Motor PWM: PWM0 → GPIO18 (Hardware PWM Kanal 0)
  Rechter Motor PWM: PWM1 → GPIO19 (Hardware PWM Kanal 1)

Motor Richtung (DIR-Pins):
  Linker Motor DIR: GPIO20
  Rechter Motor DIR: GPIO21
  
Richtungslogik: HIGH = vorwärts, LOW = rückwärts (KORRIGIERT!)

STEUERUNGSLOGIK:
---------------
Controller → Motor Mapping:
  - Left Trigger (Axis 2) → Linker Motor
  - Right Trigger (Axis 5) → Rechter Motor
  - A Button (Button 0) → Tank-Turn Modus (gehalten aktiviert)
  - B Button (Button 1) → Richtungsumschaltung (Toggle Forward/Backward)

Tank-Turn Modus (KORRIGIERT):
  - A-Button NICHT gedrückt: Normales Fahren (beide Trigger steuern jeweiligen Motor vorwärts/rückwärts)
  - A-Button GEHALTEN:
    * Rechter Trigger → Rechter Motor vorwärts, Linker Motor RÜCKWÄRTS (Drehung nach links)
    * Linker Trigger → Linker Motor vorwärts, Rechter Motor RÜCKWÄRTS (Drehung nach rechts)
    * Beide Motoren erhalten jetzt korrekt PWM-Signal!

Trigger-Wert zu PWM Konvertierung:
  - Trigger = 1.0 (voll gedrückt) → 12% PWM Duty Cycle
  - Trigger = -1.0 (nicht gedrückt) → 0% PWM Duty Cycle
  - Lineare Interpolation zwischen -1.0 und 1.0

Richtungsumschaltung:
  - Druck auf B-Button schaltet zwischen vorwärts/rückwärts um
  - Beide Motoren ändern gleichzeitig die Richtung
  - Richtungswechsel erfolgt sofort (auch während der Fahrt)

Kollisionsvermeidung (PRIORITÄT):
  - Abstandsschwelle: 30 cm (Standard)
  - Bei Hindernis < 30cm: Motor wird SOFORT gestoppt (0% PWM)
  - Überschreibt Controller-Eingaben für maximale Sicherheit

VERWENDUNG:
----------
sudo python3 xbox_controller_with_collision_v1.4.py

Beenden: Ctrl+C oder Start-Button am Controller

AUTOR: Automatisch generiert und erweitert
DATUM: 2025-10-10
VERSION: 1.4 (Bugfixes: Richtung invertiert, Tank-Turn beide Motoren)
"""

import RPi.GPIO as GPIO
import pygame
import sys
import time
import signal
import threading
from typing import Optional
from rpi_hardware_pwm import HardwarePWM

# ============================================================================
# HARDWARE KONFIGURATION
# ============================================================================

# Ultraschall-Sensor GPIO Pins (BCM Nummerierung)
TRIG_LEFT = 12      # Trigger Pin linker Sensor
ECHO_LEFT = 13      # Echo Pin linker Sensor
TRIG_RIGHT = 23     # Trigger Pin rechter Sensor
ECHO_RIGHT = 24     # Echo Pin rechter Sensor

# Motor Richtungs-Pins (DIR-Pins)
DIR_LEFT_PIN = 20   # Richtung linker Motor (GPIO20)
DIR_RIGHT_PIN = 21  # Richtung rechter Motor (GPIO21)

# Richtungslogik (KORRIGIERT!)
DIR_HIGH_IS_BACKWARD = False  # ✅ GEÄNDERT: False => HIGH = vorwärts, LOW = rückwärts

# Kollisionsparameter
COLLISION_DISTANCE_CM = 30.0  # Minimaler Sicherheitsabstand in cm
SOUND_SPEED = 34300           # Schallgeschwindigkeit in cm/s bei 20°C

# Motor PWM Konfiguration
PWM_FREQUENCY = 1000          # PWM Frequenz in Hz (1 kHz)
PWM_CHIP = 0                  # Hardware PWM Chip
PWM_CHANNEL_LEFT = 0          # Hardware PWM Kanal 0 → GPIO18
PWM_CHANNEL_RIGHT = 1         # Hardware PWM Kanal 1 → GPIO19

# Motor Geschwindigkeitsparameter
MAX_DUTY_CYCLE = 12.0         # Maximum PWM Duty Cycle in %
MIN_DUTY_CYCLE = 0.0          # Minimum PWM Duty Cycle in %

# Controller Konfiguration
CONTROLLER_ID = 0             # Erster angeschlossener Controller
AXIS_LEFT_TRIGGER = 2         # Xbox Controller Axis für Left Trigger
AXIS_RIGHT_TRIGGER = 5        # Xbox Controller Axis für Right Trigger
BUTTON_A = 0                  # A-Button für Tank-Turn Modus
BUTTON_B = 1                  # B-Button für Richtungsumschaltung
BUTTON_START = 7              # Start Button für Programm-Ende

# Update Frequenzen
SENSOR_UPDATE_HZ = 10         # Sensor-Abfrage Frequenz (10 Hz)
CONTROL_LOOP_HZ = 50          # Hauptsteuerungs-Loop Frequenz (50 Hz)

# ============================================================================
# GLOBALE VARIABLEN
# ============================================================================

# Hardware Objekte
pwm_left: Optional[HardwarePWM] = None
pwm_right: Optional[HardwarePWM] = None
joystick: Optional[pygame.joystick.Joystick] = None

# Thread Control
running = False
sensor_thread: Optional[threading.Thread] = None

# Sensor Daten
distance_left = 100.0   # Initialer Wert: weit entfernt
distance_right = 100.0

# Controller Daten
trigger_left = -1.0     # Initial: nicht gedrückt
trigger_right = -1.0
a_button_pressed = False  # Tank-Turn Modus

# Richtungsstatus
is_moving_forward = True        # True = vorwärts, False = rückwärts
b_button_was_pressed = False    # Für Edge-Detection B-Button

# ============================================================================
# SIGNAL HANDLER FÜR GRACEFUL SHUTDOWN
# ============================================================================

def signal_handler(sig, frame):
    """Handler für SIGINT (Ctrl+C) Signal."""
    print('\n[SHUTDOWN] SIGINT empfangen - Programm wird beendet...')
    cleanup()
    sys.exit(0)

# ============================================================================
# HARDWARE SETUP FUNKTIONEN
# ============================================================================

def setup_gpio():
    """Initialisiert GPIO Pins für Ultraschall-Sensoren und Motor-Richtungssteuerung."""
    print('[SETUP] Initialisiere GPIO für Sensoren und Motorrichtung...')
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Linker Sensor
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.output(TRIG_LEFT, False)
    
    # Rechter Sensor
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    GPIO.output(TRIG_RIGHT, False)
    
    # Richtungs-Pins als Ausgänge
    GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
    GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)
    
    # ✅ KORRIGIERT: Anfangsrichtung auf "vorwärts" setzen
    # Bei DIR_HIGH_IS_BACKWARD = False: HIGH = vorwärts
    if DIR_HIGH_IS_BACKWARD:
        GPIO.output(DIR_LEFT_PIN, GPIO.LOW)   # LOW = vorwärts
        GPIO.output(DIR_RIGHT_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_LEFT_PIN, GPIO.HIGH)  # HIGH = vorwärts
        GPIO.output(DIR_RIGHT_PIN, GPIO.HIGH)
    
    # Signal Handler registrieren
    signal.signal(signal.SIGINT, signal_handler)
    
    print('[SETUP] GPIO initialisiert - Sensoren stabilisieren (2s)...')
    time.sleep(2)

def set_motor_direction(left_backward: bool, right_backward: bool):
    """
    Setzt die Richtung für jeden Motor individuell.
    
    Args:
        left_backward: True = linker Motor rückwärts, False = vorwärts
        right_backward: True = rechter Motor rückwärts, False = vorwärts
    
    Note:
        ✅ KORRIGIERT: Richtungslogik komplett invertiert
    """
    # ✅ KORRIGIERT: Logik invertiert
    if DIR_HIGH_IS_BACKWARD:
        # HIGH = rückwärts, LOW = vorwärts
        left_level = GPIO.HIGH if left_backward else GPIO.LOW
        right_level = GPIO.HIGH if right_backward else GPIO.LOW
    else:
        # HIGH = vorwärts, LOW = rückwärts (neue Standardkonfiguration)
        left_level = GPIO.LOW if left_backward else GPIO.HIGH
        right_level = GPIO.LOW if right_backward else GPIO.HIGH
    
    GPIO.output(DIR_LEFT_PIN, left_level)
    GPIO.output(DIR_RIGHT_PIN, right_level)

def setup_pwm():
    """Initialisiert Hardware PWM für beide Motoren."""
    global pwm_left, pwm_right
    
    print('[SETUP] Initialisiere Hardware PWM für Motoren...')
    print(f'  PWM0 (Linker Motor) → GPIO18 (Hardware PWM Kanal {PWM_CHANNEL_LEFT})')
    print(f'  PWM1 (Rechter Motor) → GPIO19 (Hardware PWM Kanal {PWM_CHANNEL_RIGHT})')
    
    try:
        pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        
        pwm_left.start(MIN_DUTY_CYCLE)
        pwm_right.start(MIN_DUTY_CYCLE)
        
        print(f'[SETUP] PWM initialisiert - Frequenz: {PWM_FREQUENCY} Hz, Initial DC: {MIN_DUTY_CYCLE}%')
    except Exception as e:
        print(f'[ERROR] PWM Initialisierung fehlgeschlagen: {e}')
        raise

def setup_controller():
    """Initialisiert Xbox Controller über pygame."""
    global joystick
    
    print('[SETUP] Initialisiere Xbox Controller...')
    
    pygame.init()
    pygame.joystick.init()
    
    controller_count = pygame.joystick.get_count()
    if controller_count == 0:
        raise RuntimeError('[ERROR] Kein Xbox Controller gefunden!')
    
    print(f'[SETUP] {controller_count} Controller gefunden')
    
    joystick = pygame.joystick.Joystick(CONTROLLER_ID)
    joystick.init()
    
    print(f'[SETUP] Controller verbunden: {joystick.get_name()}')
    print(f'  Achsen: {joystick.get_numaxes()}')
    print(f'  Buttons: {joystick.get_numbuttons()}')

# ============================================================================
# SENSOR FUNKTIONEN
# ============================================================================

def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """Misst Entfernung mit HC-SR04 Ultraschall-Sensor."""
    try:
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        
        pulse_start = time.time()
        timeout_start = pulse_start
        
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1.0
        
        pulse_end = time.time()
        timeout_end = pulse_end
        
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_end > timeout:
                return -1.0
        
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * SOUND_SPEED) / 2
        
        if distance < 2 or distance > 400:
            return -1.0
        
        return distance
    except Exception as e:
        print(f'[ERROR] Distanzmessung fehlgeschlagen: {e}')
        return -1.0

def sensor_update_thread():
    """Thread für kontinuierliche Sensor-Abfrage."""
    global distance_left, distance_right, running
    
    print(f'[THREAD] Sensor Update Thread gestartet (Update-Rate: {SENSOR_UPDATE_HZ} Hz)')
    sleep_time = 1.0 / SENSOR_UPDATE_HZ
    
    while running:
        loop_start = time.time()
        
        dist_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
        if dist_left > 0:
            distance_left = dist_left
        
        dist_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
        if dist_right > 0:
            distance_right = dist_right
        
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

# ============================================================================
# MOTOR STEUERUNG
# ============================================================================

def trigger_to_duty_cycle(trigger_value: float) -> float:
    """Konvertiert Xbox Trigger Wert zu PWM Duty Cycle."""
    normalized = (trigger_value + 1.0) / 2.0
    duty_cycle = normalized * MAX_DUTY_CYCLE
    duty_cycle = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, duty_cycle))
    return duty_cycle

def set_motor_speed(left_duty: float, right_duty: float):
    """Setzt PWM Duty Cycle für beide Motoren mit Kollisionsvermeidung."""
    global pwm_left, pwm_right, distance_left, distance_right
    
    # Kollisionsvermeidung - Linker Motor
    if 0 < distance_left < COLLISION_DISTANCE_CM:
        left_duty = MIN_DUTY_CYCLE
        print(f'[COLLISION] ⚠️  Linkes Hindernis bei {distance_left:.1f}cm - Linker Motor gestoppt!')
    
    # Kollisionsvermeidung - Rechter Motor
    if 0 < distance_right < COLLISION_DISTANCE_CM:
        right_duty = MIN_DUTY_CYCLE
        print(f'[COLLISION] ⚠️  Rechtes Hindernis bei {distance_right:.1f}cm - Rechter Motor gestoppt!')
    
    # Clamp Duty Cycles
    left_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, left_duty))
    right_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, right_duty))
    
    try:
        pwm_left.change_duty_cycle(left_duty)
        pwm_right.change_duty_cycle(right_duty)
    except Exception as e:
        print(f'[ERROR] PWM Update fehlgeschlagen: {e}')

# ============================================================================
# CONTROLLER INPUT VERARBEITUNG
# ============================================================================

def read_controller_input():
    """
    Liest aktuelle Controller Eingaben und aktualisiert globale Variablen.
    """
    global trigger_left, trigger_right, running, joystick
    global is_moving_forward, b_button_was_pressed, a_button_pressed
    
    pygame.event.pump()
    
    if not joystick or not joystick.get_init():
        print('[ERROR] ❌ Controller Verbindung verloren!')
        running = False
        return
    
    try:
        # Trigger Werte lesen
        trigger_left = joystick.get_axis(AXIS_LEFT_TRIGGER)
        trigger_right = joystick.get_axis(AXIS_RIGHT_TRIGGER)
        
        # A-Button für Tank-Turn
        a_button_pressed = joystick.get_button(BUTTON_A)
        
        # B-Button für Richtungsumschaltung (mit Edge-Detection)
        b_button_currently_pressed = joystick.get_button(BUTTON_B)
        
        if b_button_currently_pressed and not b_button_was_pressed:
            is_moving_forward = not is_moving_forward
            direction_text = "⬆️ VORWÄRTS" if is_moving_forward else "⬇️ RÜCKWÄRTS"
            print(f'[INPUT] 🔄 B-Button: Richtung umgeschaltet → {direction_text}')
        
        b_button_was_pressed = b_button_currently_pressed
        
        # Start Button für Programm-Ende
        if joystick.get_button(BUTTON_START):
            print('[INPUT] 🎮 Start-Button gedrückt - Beende Programm...')
            running = False
    
    except Exception as e:
        print(f'[ERROR] Controller Input Fehler: {e}')
        running = False

# ============================================================================
# HAUPT STEUERUNGS-LOOP
# ============================================================================

def control_loop():
    """
    Haupt-Steuerungsschleife des Roboters.
    
    ✅ KORRIGIERT in v1.4:
    - Tank-Turn beide Motoren erhalten jetzt PWM Signal
    - Richtungslogik komplett überarbeitet
    """
    global running, trigger_left, trigger_right, distance_left, distance_right
    global is_moving_forward, a_button_pressed
    
    print(f'[CONTROL] Steuerungs-Loop gestartet (Update-Rate: {CONTROL_LOOP_HZ} Hz)')
    print('[CONTROL] 🤖 Roboter betriebsbereit!')
    print('\n' + '='*80)
    print('🎮 STEUERUNG:')
    print('  Left Trigger  → Linker Motor')
    print('  Right Trigger → Rechter Motor')
    print('  A Button (halten) → Tank-Turn Modus (Drehen an Ort)')
    print('  B Button → Richtungsumschaltung (Vorwärts ⇄ Rückwärts)')
    print('  Start Button → Programm beenden')
    print('\n🔄 TANK-TURN MODUS (✅ KORRIGIERT):')
    print('  A gehalten + Right Trigger → Drehung nach LINKS (rechts vor, links zurück)')
    print('  A gehalten + Left Trigger  → Drehung nach RECHTS (links vor, rechts zurück)')
    print('  ✅ Beide Motoren drehen jetzt korrekt!')
    print('\n🛡️  SICHERHEIT:')
    print(f'  Kollisionsabstand: {COLLISION_DISTANCE_CM} cm')
    print('='*80 + '\n')
    
    sleep_time = 1.0 / CONTROL_LOOP_HZ
    status_counter = 0
    status_update_interval = CONTROL_LOOP_HZ
    
    while running:
        loop_start = time.time()
        
        # 1. Lese Controller Eingaben
        read_controller_input()
        
        # 2. Konvertiere Trigger zu Duty Cycle
        left_duty = trigger_to_duty_cycle(trigger_left)
        right_duty = trigger_to_duty_cycle(trigger_right)
        
        # 3. ✅ KORRIGIERTE Tank-Turn Logik
        if a_button_pressed:
            # A-Button gehalten: Tank-Turn Modus aktiviert
            
            if is_moving_forward:
                # Vorwärts-Modus mit Tank-Turn
                # Rechter Trigger → Rechter Motor vorwärts (right_duty), Linker Motor rückwärts (right_duty)
                # Linker Trigger  → Linker Motor vorwärts (left_duty), Rechter Motor rückwärts (left_duty)
                
                # ✅ KORREKTUR: Beide Motoren bekommen den Duty Cycle des aktiven Triggers!
                # Bestimme welcher Trigger stärker gedrückt ist
                if right_duty > left_duty:
                    # Rechter Trigger dominiert → Links-Drehung
                    # Rechts vorwärts, Links rückwärts, beide mit right_duty
                    set_motor_direction(left_backward=True, right_backward=False)
                    set_motor_speed(right_duty, right_duty)  # ✅ Beide Motoren mit right_duty!
                else:
                    # Linker Trigger dominiert → Rechts-Drehung
                    # Links vorwärts, Rechts rückwärts, beide mit left_duty
                    set_motor_direction(left_backward=False, right_backward=True)
                    set_motor_speed(left_duty, left_duty)  # ✅ Beide Motoren mit left_duty!
            
            else:
                # Rückwärts-Modus mit Tank-Turn (Logik invertiert)
                if right_duty > left_duty:
                    # Rechter Trigger dominiert → Links-Drehung (rückwärts)
                    set_motor_direction(left_backward=False, right_backward=True)
                    set_motor_speed(right_duty, right_duty)
                else:
                    # Linker Trigger dominiert → Rechts-Drehung (rückwärts)
                    set_motor_direction(left_backward=True, right_backward=False)
                    set_motor_speed(left_duty, left_duty)
        
        else:
            # A-Button NICHT gedrückt: Normales Fahren
            backward = not is_moving_forward
            set_motor_direction(backward, backward)
            set_motor_speed(left_duty, right_duty)
        
        # 4. Status Ausgabe (jede Sekunde)
        status_counter += 1
        if status_counter >= status_update_interval:
            status_counter = 0
            
            left_status = '🔴' if 0 < distance_left < COLLISION_DISTANCE_CM else '🟢'
            right_status = '🔴' if 0 < distance_right < COLLISION_DISTANCE_CM else '🟢'
            
            direction_emoji = "⬆️ " if is_moving_forward else "⬇️ "
            direction_text = "FWD" if is_moving_forward else "REV"
            
            # Tank-Turn Status
            mode_text = "🔄 TANK" if a_button_pressed else "➡️  NORM"
            
            print(f'[STATUS] {left_status} Mode:{mode_text} Dir:{direction_emoji}{direction_text} | '
                  f'Trigger: L={trigger_left:+.2f} R={trigger_right:+.2f} | '
                  f'PWM: L={left_duty:.1f}% R={right_duty:.1f}% | '
                  f'Dist: L={distance_left:.1f}cm R={distance_right:.1f}cm {right_status}')
        
        # 5. Reguliere Loop-Frequenz
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

# ============================================================================
# CLEANUP FUNKTION
# ============================================================================

def cleanup():
    """Räumt alle Hardware-Ressourcen auf und stoppt Threads."""
    global running, pwm_left, pwm_right, joystick, sensor_thread
    
    print('[CLEANUP] 🧹 Starte Ressourcen-Freigabe...')
    running = False
    
    if sensor_thread and sensor_thread.is_alive():
        print('[CLEANUP] Warte auf Sensor Thread...')
        sensor_thread.join(timeout=2.0)
    
    if pwm_left:
        print('[CLEANUP] Stoppe linken Motor...')
        try:
            pwm_left.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_left.stop()
        except Exception as e:
            print(f'[CLEANUP] Warnung beim Stoppen linker Motor: {e}')
    
    if pwm_right:
        print('[CLEANUP] Stoppe rechten Motor...')
        try:
            pwm_right.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_right.stop()
        except Exception as e:
            print(f'[CLEANUP] Warnung beim Stoppen rechter Motor: {e}')
    
    if joystick:
        print('[CLEANUP] Trenne Controller...')
        try:
            joystick.quit()
        except Exception as e:
            print(f'[CLEANUP] Warnung beim Trennen Controller: {e}')
    
    try:
        pygame.quit()
    except Exception as e:
        print(f'[CLEANUP] Warnung beim Beenden pygame: {e}')
    
    print('[CLEANUP] Räume GPIO auf...')
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f'[CLEANUP] Warnung beim GPIO Cleanup: {e}')
    
    print('[CLEANUP] ✅ Alle Ressourcen freigegeben - Programm beendet.')

# ============================================================================
# MAIN FUNKTION
# ============================================================================

def main():
    """Hauptfunktion - Einstiegspunkt des Programms."""
    global running, sensor_thread
    
    print('\n' + '='*80)
    print('🤖 XBOX CONTROLLER ROBOTER-STEUERUNG MIT TANK-TURN')
    print('Version 1.4 - 2025-10-10 (BUGFIXES: Richtung + Tank-Turn)')
    print('✅ FEHLER 1: Richtungssteuerung invertiert')
    print('✅ FEHLER 2: Tank-Turn beide Motoren drehen')
    print('='*80 + '\n')
    
    try:
        setup_gpio()
        setup_pwm()
        setup_controller()
        
        print('\n[INIT] ✅ Alle Systeme initialisiert - Starte Threads...')
        
        running = True
        sensor_thread = threading.Thread(target=sensor_update_thread, daemon=True, name='SensorThread')
        sensor_thread.start()
        
        time.sleep(0.5)
        control_loop()
    
    except KeyboardInterrupt:
        print('\n[SHUTDOWN] ⌨️  Keyboard Interrupt (Ctrl+C) - Beende Programm...')
    except Exception as e:
        print(f'\n[ERROR] ❌ Unerwarteter Fehler: {e}')
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

# ============================================================================
# PROGRAMM START
# ============================================================================

if __name__ == '__main__':
    import os
    
    if os.geteuid() != 0:
        print('❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!')
        print('📝 Verwendung: sudo python3 xbox_controller_with_collision_v1.4.py')
        sys.exit(1)
    
    main()
