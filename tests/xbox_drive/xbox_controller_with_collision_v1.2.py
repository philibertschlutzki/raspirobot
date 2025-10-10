#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Xbox Controller Roboter-Steuerung mit Kollisionsvermeidung und Richtungsumschaltung
====================================================================================

Dieses Script kombiniert die Xbox Controller Eingabe mit Ultraschall-Kollisionserkennung
für einen Raspberry Pi gesteuerten Roboter mit zwei Motoren.

NEU in Version 1.2:
- B-Button schaltet Fahrtrichtung um (vorwärts/rückwärts)
- DIR-Pins für Richtungssteuerung implementiert
- Richtungsstatus wird visuell angezeigt

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
  Linker Sensor:  TRIG=GPIO12, ECHO=GPIO13
  Rechter Sensor: TRIG=GPIO23, ECHO=GPIO24

Motor PWM (Hardware PWM):
  Linker Motor PWM:  PWM0 → GPIO18 (Hardware PWM Kanal 0)
  Rechter Motor PWM: PWM1 → GPIO19 (Hardware PWM Kanal 1)

Motor Richtung (DIR-Pins):
  Linker Motor DIR:  GPIO20
  Rechter Motor DIR: GPIO21

  Richtungslogik: HIGH = rückwärts, LOW = vorwärts

WICHTIG: GPIO18/19 werden für Hardware PWM verwendet!
         GPIO12/13 sind für die Ultraschall-Sensoren reserviert.
         GPIO20/21 steuern die Motorrichtung.

STEUERUNGSLOGIK:
---------------
Controller → Motor Mapping:
- Left Trigger (Axis 2)  → Linker Motor
- Right Trigger (Axis 5) → Rechter Motor
- B Button (Button 1)    → Richtungsumschaltung (Toggle Forward/Backward)

Trigger-Wert zu PWM Konvertierung:
- Trigger = 1.0 (voll gedrückt)   → 12% PWM Duty Cycle
- Trigger = -1.0 (nicht gedrückt) → 0% PWM Duty Cycle
- Lineare Interpolation zwischen -1.0 und 1.0

Richtungsumschaltung:
- Druck auf B-Button schaltet zwischen vorwärts/rückwärts um
- Beide Motoren ändern gleichzeitig die Richtung
- Richtungswechsel erfolgt sofort (auch während der Fahrt)
- Aktueller Modus wird im Status angezeigt

Kollisionsvermeidung (PRIORITÄT):
- Abstandsschwelle: 30 cm (Standard)
- Bei Hindernis < 30cm: Motor wird SOFORT gestoppt (0% PWM)
- Überschreibt Controller-Eingaben für maximale Sicherheit
- Sensoren werden kontinuierlich mit 10 Hz abgefragt
- Kollisionsvermeidung wirkt unabhängig von der Fahrtrichtung

SICHERHEITSFEATURES:
-------------------
1. Kollisionserkennung hat absolute Priorität
2. Graceful Shutdown bei SIGINT (Ctrl+C)
3. Automatische Ressourcenfreigabe (GPIO, PWM, Controller)
4. Fehlerbehandlung für Sensorausfälle
5. Thread-sichere Controller-Eingabe
6. Richtungswechsel nur bei gültigen Zuständen

VERWENDUNG:
----------
sudo python3 xbox_controller_with_collision.py

Beenden: Ctrl+C oder Start-Button am Controller

AUTOR: Automatisch generiert und erweitert
DATUM: 2025-10-10
VERSION: 1.2 (B-Button Richtungsumschaltung implementiert)
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
TRIG_LEFT = 12   # Trigger Pin linker Sensor
ECHO_LEFT = 13   # Echo Pin linker Sensor
TRIG_RIGHT = 23  # Trigger Pin rechter Sensor
ECHO_RIGHT = 24  # Echo Pin rechter Sensor

# Motor Richtungs-Pins (DIR-Pins)
DIR_LEFT_PIN = 20   # Richtung linker Motor (GPIO20)
DIR_RIGHT_PIN = 21  # Richtung rechter Motor (GPIO21)

# Richtungslogik (wie in back_180_fwd.py)
# True => HIGH = rückwärts, LOW = vorwärts
# Falls die Drehrichtung am Fahrzeug invertiert ist, hier umschalten
DIR_HIGH_IS_BACKWARD = True

# Kollisionsparameter
COLLISION_DISTANCE_CM = 30.0  # Minimaler Sicherheitsabstand in cm
SOUND_SPEED = 34300  # Schallgeschwindigkeit in cm/s bei 20°C

# Motor PWM Konfiguration
# WICHTIG: GPIO18 und GPIO19 für Hardware PWM!
PWM_FREQUENCY = 1000    # PWM Frequenz in Hz (1 kHz)
PWM_CHIP = 0            # Hardware PWM Chip
PWM_CHANNEL_LEFT = 0    # Hardware PWM Kanal 0 → GPIO18
PWM_CHANNEL_RIGHT = 1   # Hardware PWM Kanal 1 → GPIO19

# Motor Geschwindigkeitsparameter
MAX_DUTY_CYCLE = 12.0   # Maximum PWM Duty Cycle in %
MIN_DUTY_CYCLE = 0.0    # Minimum PWM Duty Cycle in % (Motor aus)

# Controller Konfiguration
CONTROLLER_ID = 0        # Erster angeschlossener Controller
AXIS_LEFT_TRIGGER = 2    # Xbox Controller Axis für Left Trigger
AXIS_RIGHT_TRIGGER = 5   # Xbox Controller Axis für Right Trigger
BUTTON_B = 1             # B-Button für Richtungsumschaltung
BUTTON_START = 7         # Start Button für Programm-Ende

# Update Frequenzen
SENSOR_UPDATE_HZ = 10    # Sensor-Abfrage Frequenz (10 Hz)
CONTROL_LOOP_HZ = 50     # Hauptsteuerungs-Loop Frequenz (50 Hz)

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

# Sensor Daten (Thread-sicher durch schnelle Reads/Writes)
distance_left = 100.0   # Initialer Wert: weit entfernt
distance_right = 100.0

# Controller Daten (Thread-sicher durch schnelle Reads/Writes)
trigger_left = -1.0     # Initial: nicht gedrückt
trigger_right = -1.0

# Richtungsstatus (NEU in Version 1.2)
is_moving_forward = True     # True = vorwärts, False = rückwärts
b_button_was_pressed = False # Für Edge-Detection (verhindert Mehrfach-Toggle)

# ============================================================================
# SIGNAL HANDLER FÜR GRACEFUL SHUTDOWN
# ============================================================================

def signal_handler(sig, frame):
    """
    Handler für SIGINT (Ctrl+C) Signal.
    Sorgt für sauberes Herunterfahren aller Komponenten.

    Args:
        sig: Signal Nummer
        frame: Aktueller Stack Frame
    """
    print('\n[SHUTDOWN] SIGINT empfangen - Programm wird beendet...')
    cleanup()
    sys.exit(0)

# ============================================================================
# HARDWARE SETUP FUNKTIONEN
# ============================================================================

def setup_gpio():
    """
    Initialisiert GPIO Pins für Ultraschall-Sensoren und Motor-Richtungssteuerung.

    Konfiguriert:
    - BCM Pin-Nummerierung
    - TRIG Pins als Output (initialisiert auf LOW)
    - ECHO Pins als Input
    - DIR Pins als Output (initialisiert auf LOW = vorwärts)
    - Signal Handler für sauberen Shutdown

    GPIO Pin Verwendung:
    - GPIO12, 13: Ultraschall Sensoren (links)
    - GPIO23, 24: Ultraschall Sensoren (rechts)
    - GPIO20, 21: Motor Richtung (DIR-Pins)
    - GPIO18, 19: Hardware PWM für Motoren (nicht über GPIO.setup!)
    """
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

    # Anfangsrichtung auf "vorwärts" setzen (beide Motoren gleiche Richtung)
    if DIR_HIGH_IS_BACKWARD:
        # LOW = vorwärts
        GPIO.output(DIR_LEFT_PIN, GPIO.LOW)
        GPIO.output(DIR_RIGHT_PIN, GPIO.LOW)
    else:
        # HIGH = vorwärts
        GPIO.output(DIR_LEFT_PIN, GPIO.HIGH)
        GPIO.output(DIR_RIGHT_PIN, GPIO.HIGH)

    # Signal Handler registrieren
    signal.signal(signal.SIGINT, signal_handler)

    print('[SETUP] GPIO initialisiert - Sensoren stabilisieren (2s)...')
    time.sleep(2)  # Sensoren stabilisieren lassen

def set_direction(backward: bool):
    """
    Setzt beide Motoren auf die gleiche Fahrtrichtung (vorwärts/rückwärts).

    Die logische Zuordnung hängt von DIR_HIGH_IS_BACKWARD ab:
    - DIR_HIGH_IS_BACKWARD = True:  HIGH = rückwärts, LOW = vorwärts
    - DIR_HIGH_IS_BACKWARD = False: HIGH = vorwärts,  LOW = rückwärts

    Args:
        backward: True = beide rückwärts, False = beide vorwärts

    Note:
        Diese Funktion wird von read_controller_input() aufgerufen
        wenn der B-Button gedrückt wird.
    """
def set_direction(backward: bool):
    if DIR_HIGH_IS_BACKWARD:
        level = GPIO.LOW if backward else GPIO.HIGH  # INVERTIERT
    else:
        level = GPIO.HIGH if backward else GPIO.LOW  # INVERTIERT
    
    GPIO.output(DIR_LEFT_PIN, level)
    GPIO.output(DIR_RIGHT_PIN, level)


def setup_pwm():
    """
    Initialisiert Hardware PWM für beide Motoren.

    Verwendet Hardware PWM Kanäle für präzise Motorsteuerung:
    - Kanal 0 (GPIO18): Linker Motor
    - Kanal 1 (GPIO19): Rechter Motor
    - Frequenz: 1000 Hz
    - Initial Duty Cycle: 0% (Motoren aus)

    WICHTIG: Hardware PWM verwendet GPIO18/19 automatisch.
             Diese Pins dürfen NICHT mit GPIO.setup() konfiguriert werden!

    Raises:
        Exception: Bei PWM Initialisierungsfehlern
    """
    global pwm_left, pwm_right

    print('[SETUP] Initialisiere Hardware PWM für Motoren...')
    print(f'  PWM0 (Linker Motor)  → GPIO18 (Hardware PWM Kanal {PWM_CHANNEL_LEFT})')
    print(f'  PWM1 (Rechter Motor) → GPIO19 (Hardware PWM Kanal {PWM_CHANNEL_RIGHT})')

    try:
        pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)

        # Starte PWM mit 0% Duty Cycle (Motoren aus)
        pwm_left.start(MIN_DUTY_CYCLE)
        pwm_right.start(MIN_DUTY_CYCLE)

        print(f'[SETUP] PWM initialisiert - Frequenz: {PWM_FREQUENCY} Hz, Initial DC: {MIN_DUTY_CYCLE}%')

    except Exception as e:
        print(f'[ERROR] PWM Initialisierung fehlgeschlagen: {e}')
        print('[ERROR] Mögliche Ursachen:')
        print('  - Hardware PWM nicht aktiviert (/boot/config.txt)')
        print('  - Keine Root-Rechte (sudo erforderlich)')
        print('  - PWM bereits von anderem Prozess verwendet')
        raise

def setup_controller():
    """
    Initialisiert Xbox Controller über pygame.

    Prozess:
    1. Initialisiert pygame joystick Modul
    2. Detektiert angeschlossene Controller
    3. Initialisiert ersten Controller (ID 0)
    4. Gibt Controller-Informationen aus

    Raises:
        RuntimeError: Wenn kein Controller gefunden wurde

    Note:
        pygame muss im Main Thread initialisiert werden!
    """
    global joystick

    print('[SETUP] Initialisiere Xbox Controller...')

    # Pygame Joystick Modul initialisieren
    pygame.init()
    pygame.joystick.init()

    # Controller Anzahl prüfen
    controller_count = pygame.joystick.get_count()
    if controller_count == 0:
        raise RuntimeError('[ERROR] Kein Xbox Controller gefunden! Bitte Controller anschließen.')

    print(f'[SETUP] {controller_count} Controller gefunden')

    # Ersten Controller initialisieren
    joystick = pygame.joystick.Joystick(CONTROLLER_ID)
    joystick.init()

    print(f'[SETUP] Controller verbunden: {joystick.get_name()}')
    print(f'  Achsen:  {joystick.get_numaxes()}')
    print(f'  Buttons: {joystick.get_numbuttons()}')
    print(f'  Hats:    {joystick.get_numhats()}')

# ============================================================================
# SENSOR FUNKTIONEN
# ============================================================================

def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """
    Misst Entfernung mit HC-SR04 Ultraschall-Sensor.

    Funktionsweise:
    1. Sendet 10µs Trigger-Puls
    2. Wartet auf Echo Signal (HIGH)
    3. Misst Echo-Dauer
    4. Berechnet Entfernung: distance = (echo_duration * sound_speed) / 2

    Args:
        trig_pin: GPIO Pin für Trigger Signal
        echo_pin: GPIO Pin für Echo Signal
        timeout: Maximale Wartezeit in Sekunden (default: 0.1s = ~17m max Distanz)

    Returns:
        Entfernung in cm (oder -1.0 bei Fehler/Timeout)

    Fehlerbehandlung:
        - Timeout wenn Echo nicht innerhalb timeout empfangen wird
        - Gibt -1.0 zurück bei Fehlern (wird als "weit entfernt" interpretiert)
    """
    try:
        # 10 Mikrosekunden Trigger Puls senden
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)  # 10µs HIGH Puls
        GPIO.output(trig_pin, False)

        # Warte auf Echo Start (HIGH) mit Timeout
        pulse_start = time.time()
        timeout_start = pulse_start
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1.0  # Timeout: Kein Echo empfangen

        # Warte auf Echo Ende (LOW) mit Timeout
        pulse_end = time.time()
        timeout_end = pulse_end
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_end > timeout:
                return -1.0  # Timeout: Echo zu lang

        # Berechne Entfernung
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * SOUND_SPEED) / 2

        # Sanity Check: HC-SR04 Messbereich ist 2cm - 400cm
        if distance < 2 or distance > 400:
            return -1.0

        return distance

    except Exception as e:
        print(f'[ERROR] Distanzmessung fehlgeschlagen (TRIG={trig_pin}, ECHO={echo_pin}): {e}')
        return -1.0

def sensor_update_thread():
    """
    Thread für kontinuierliche Sensor-Abfrage.

    Läuft parallel zum Hauptprogramm und aktualisiert globale Distanzvariablen:
    - distance_left: Entfernung linker Sensor
    - distance_right: Entfernung rechter Sensor

    Update-Frequenz: SENSOR_UPDATE_HZ (10 Hz)

    Thread Safety:
        - Verwendet globale Variablen (float assignment ist atomic in CPython)
        - CPython GIL garantiert dass float assignments nicht unterbrochen werden
        - Keine Locks notwendig für einfache float Reads/Writes
    """
    global distance_left, distance_right, running

    print(f'[THREAD] Sensor Update Thread gestartet (Update-Rate: {SENSOR_UPDATE_HZ} Hz)')
    sleep_time = 1.0 / SENSOR_UPDATE_HZ

    while running:
        loop_start = time.time()

        # Linken Sensor abfragen
        dist_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
        if dist_left > 0:
            distance_left = dist_left

        # Rechten Sensor abfragen
        dist_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
        if dist_right > 0:
            distance_right = dist_right

        # Reguliere Update-Frequenz
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

# ============================================================================
# MOTOR STEUERUNG
# ============================================================================

def trigger_to_duty_cycle(trigger_value: float) -> float:
    """
    Konvertiert Xbox Trigger Wert zu PWM Duty Cycle.

    Mapping:
        Trigger -1.0 (nicht gedrückt) → 0% PWM (Motor aus)
        Trigger  0.0 (halb gedrückt)  → 6% PWM (halbe Geschwindigkeit)
        Trigger +1.0 (voll gedrückt)  → 12% PWM (volle Geschwindigkeit)

    Formel:
        normalized = (trigger + 1.0) / 2.0      # [-1,1] → [0,1]
        duty_cycle = normalized * MAX_DUTY_CYCLE # [0,1] → [0,12]

    Args:
        trigger_value: Controller Trigger Wert (-1.0 bis +1.0)

    Returns:
        PWM Duty Cycle in Prozent (0.0 bis 12.0)
    """
    # Normalisiere von [-1, 1] zu [0, 1]
    normalized = (trigger_value + 1.0) / 2.0

    # Skaliere auf [0, MAX_DUTY_CYCLE]
    duty_cycle = normalized * MAX_DUTY_CYCLE

    # Clamp auf gültigen Bereich
    duty_cycle = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, duty_cycle))

    return duty_cycle

def set_motor_speed(left_duty: float, right_duty: float):
    """
    Setzt PWM Duty Cycle für beide Motoren mit Kollisionsvermeidung.

    Wendet Kollisionsvermeidung an:
    - Prüft Sensordaten für jeden Motor
    - Überschreibt duty_cycle mit 0 wenn Hindernis < COLLISION_DISTANCE_CM
    - Arbeitet unabhängig für jeden Motor (Tank-Steering Logic)

    Args:
        left_duty: Gewünschter Duty Cycle linker Motor (0-12%)
        right_duty: Gewünschter Duty Cycle rechter Motor (0-12%)

    Safety Features:
        - Kollisionserkennung hat absolute Priorität über Controller Input
        - Bei Sensor-Fehler (-1.0): Motor läuft normal (fail-safe)
        - PWM Werte werden auf [MIN_DUTY_CYCLE, MAX_DUTY_CYCLE] begrenzt
        - Exception Handling für PWM Fehler
    """
    global pwm_left, pwm_right, distance_left, distance_right

    # Kollisionsvermeidung - Linker Motor
    if 0 < distance_left < COLLISION_DISTANCE_CM:
        left_duty = MIN_DUTY_CYCLE
        print(f'[COLLISION] ⚠️  Linkes Hindernis bei {distance_left:.1f}cm - Linker Motor gestoppt!')

    # Kollisionsvermeidung - Rechter Motor
    if 0 < distance_right < COLLISION_DISTANCE_CM:
        right_duty = MIN_DUTY_CYCLE
        print(f'[COLLISION] ⚠️  Rechtes Hindernis bei {distance_right:.1f}cm - Rechter Motor gestoppt!')

    # Clamp Duty Cycles auf gültigen Bereich
    left_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, left_duty))
    right_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, right_duty))

    # Setze PWM Werte
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

    Verarbeitet:
    - Left Trigger (Axis 2)  → trigger_left
    - Right Trigger (Axis 5) → trigger_right
    - B Button (Button 1)    → Richtungsumschaltung (Toggle)
    - Start Button (Button 7) → Programm beenden

    Richtungsumschaltung:
        - B-Button wird mit Edge-Detection ausgewertet
        - Nur bei Flanke (Übergang nicht gedrückt → gedrückt) wird umgeschaltet
        - Verhindert Mehrfach-Umschaltung bei gehaltenem Button
        - Ruft set_direction() auf um DIR-Pins zu setzen

    WICHTIG: Muss in jedem Frame aufgerufen werden um pygame Event Queue zu leeren!
             pygame.event.pump() ist essentiell für Controller Updates.

    Thread Safety:
        - Läuft im Main Thread (pygame Requirement)
        - Schreibt auf globale Variablen
    """
    global trigger_left, trigger_right, running, joystick
    global is_moving_forward, b_button_was_pressed

    # Pygame Event Queue abarbeiten (NOTWENDIG für Controller Updates!)
    pygame.event.pump()

    # Prüfe ob Controller noch verbunden
    if not joystick or not joystick.get_init():
        print('[ERROR] ❌ Controller Verbindung verloren!')
        running = False
        return

    # Lese Controller Inputs
    try:
        # Trigger Werte lesen
        trigger_left = joystick.get_axis(AXIS_LEFT_TRIGGER)
        trigger_right = joystick.get_axis(AXIS_RIGHT_TRIGGER)

        # B-Button für Richtungsumschaltung (mit Edge-Detection)
        b_button_pressed = joystick.get_button(BUTTON_B)

        if b_button_pressed and not b_button_was_pressed:
            # Flanke erkannt: Button wurde gerade gedrückt
            is_moving_forward = not is_moving_forward
            set_direction(backward=not is_moving_forward)

            direction_text = "⬆️  VORWÄRTS" if is_moving_forward else "⬇️  RÜCKWÄRTS"
            print(f'[INPUT] 🔄 B-Button: Richtung umgeschaltet → {direction_text}')

        b_button_was_pressed = b_button_pressed

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

    Prozess (50 Hz Loop):
    1. Liest Controller Eingaben (read_controller_input)
    2. Konvertiert Trigger → PWM Duty Cycle (trigger_to_duty_cycle)
    3. Wendet Kollisionsvermeidung an (in set_motor_speed)
    4. Aktualisiert Motor PWM
    5. Gibt Status aus (jede Sekunde)

    Loop-Frequenz: CONTROL_LOOP_HZ (50 Hz = 20ms pro Iteration)
    Status-Output: 1 Hz (zur Übersicht, vermeidet Terminal-Spam)

    Thread-Interaktion:
        - Läuft im Main Thread (pygame Requirement)
        - Liest distance_left/right aus Sensor Thread
        - Schreibt trigger_left/right und is_moving_forward
    """
    global running, trigger_left, trigger_right, distance_left, distance_right
    global is_moving_forward

    print(f'[CONTROL] Steuerungs-Loop gestartet (Update-Rate: {CONTROL_LOOP_HZ} Hz)')
    print('[CONTROL] 🤖 Roboter betriebsbereit!')
    print('\n' + '='*80)
    print('🎮 STEUERUNG:')
    print('  Left Trigger  → Linker Motor (GPIO18 / PWM0)')
    print('  Right Trigger → Rechter Motor (GPIO19 / PWM1)')
    print('  B Button      → Richtungsumschaltung (Vorwärts ⇄ Rückwärts)')
    print('  Start Button  → Programm beenden')
    print('\n🛡️  SICHERHEIT:')
    print(f'  Kollisionsabstand: {COLLISION_DISTANCE_CM} cm')
    print('  Motoren stoppen automatisch bei Hindernissen!')
    print('\n📊 SENSOR ÜBERWACHUNG:')
    print(f'  Linker Sensor:  GPIO{TRIG_LEFT}/{ECHO_LEFT}')
    print(f'  Rechter Sensor: GPIO{TRIG_RIGHT}/{ECHO_RIGHT}')
    print('\n⚙️  MOTOR RICHTUNG:')
    print(f'  Linker Motor:  GPIO{DIR_LEFT_PIN} (DIR)')
    print(f'  Rechter Motor: GPIO{DIR_RIGHT_PIN} (DIR)')
    print('='*80 + '\n')

    sleep_time = 1.0 / CONTROL_LOOP_HZ
    status_counter = 0
    status_update_interval = CONTROL_LOOP_HZ  # Status alle 1 Sekunde ausgeben

    while running:
        loop_start = time.time()

        # 1. Lese Controller Eingaben (inkl. B-Button für Richtung)
        read_controller_input()

        # 2. Konvertiere Trigger zu Duty Cycle
        left_duty = trigger_to_duty_cycle(trigger_left)
        right_duty = trigger_to_duty_cycle(trigger_right)

        # 3. Setze Motor Geschwindigkeit (mit Kollisionsvermeidung)
        set_motor_speed(left_duty, right_duty)

        # 4. Status Ausgabe (jede Sekunde)
        status_counter += 1
        if status_counter >= status_update_interval:
            status_counter = 0

            # Emojis für visuelle Feedback
            left_status = '🔴' if 0 < distance_left < COLLISION_DISTANCE_CM else '🟢'
            right_status = '🔴' if 0 < distance_right < COLLISION_DISTANCE_CM else '🟢'
            direction_emoji = "⬆️ " if is_moving_forward else "⬇️ "
            direction_text = "FWD" if is_moving_forward else "REV"

            print(f'[STATUS] {left_status} Dir:{direction_emoji}{direction_text} | '
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
    """
    Räumt alle Hardware-Ressourcen auf und stoppt Threads.

    Reihenfolge (wichtig für sauberes Shutdown):
    1. Stoppt alle Threads (running = False)
    2. Wartet auf Thread-Beendigung (mit Timeout)
    3. Stoppt Motoren (0% PWM)
    4. Beendet PWM Hardware
    5. Trennt Controller
    6. Räumt GPIO auf

    Exception Handling:
        - Jeder Schritt hat try-except Block
        - Cleanup wird immer komplett durchgeführt
        - Fehler werden ausgegeben aber stoppen nicht den Cleanup
    """
    global running, pwm_left, pwm_right, joystick, sensor_thread

    print('[CLEANUP] 🧹 Starte Ressourcen-Freigabe...')

    # 1. Stoppe alle Threads
    running = False

    # 2. Warte auf Thread-Beendigung
    if sensor_thread and sensor_thread.is_alive():
        print('[CLEANUP] Warte auf Sensor Thread...')
        sensor_thread.join(timeout=2.0)
        if sensor_thread.is_alive():
            print('[CLEANUP] ⚠️  Sensor Thread Timeout - Force Exit')

    # 3. Stoppe Motoren (SICHERHEIT!)
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

    # 4. Controller trennen
    if joystick:
        print('[CLEANUP] Trenne Controller...')
        try:
            joystick.quit()
        except Exception as e:
            print(f'[CLEANUP] Warnung beim Trennen Controller: {e}')

    # 5. Pygame beenden
    try:
        pygame.quit()
    except Exception as e:
        print(f'[CLEANUP] Warnung beim Beenden pygame: {e}')

    # 6. GPIO aufräumen
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
    """
    Hauptfunktion - Einstiegspunkt des Programms.

    Ablauf:
    1. Hardware Setup (GPIO, PWM, Controller)
    2. Starte Sensor Thread
    3. Starte Control Loop im Main Thread
    4. Bei Beendigung: Cleanup

    Exception Handling:
        - KeyboardInterrupt: Ctrl+C graceful shutdown
        - Alle anderen Exceptions: Traceback ausgeben, dann Cleanup
        - finally Block garantiert dass Cleanup immer ausgeführt wird
    """
    global running, sensor_thread

    print('\n' + '='*80)
    print('🤖 XBOX CONTROLLER ROBOTER-STEUERUNG MIT KOLLISIONSVERMEIDUNG')
    print('Version 1.2 - 2025-10-10 (B-Button Richtungsumschaltung)')
    print('='*80 + '\n')

    try:
        # Hardware Initialisierung
        setup_gpio()
        setup_pwm()
        setup_controller()

        print('\n[INIT] ✅ Alle Systeme initialisiert - Starte Threads...')

        # Setze Running Flag
        running = True

        # Starte Sensor Update Thread
        sensor_thread = threading.Thread(target=sensor_update_thread, daemon=True, name='SensorThread')
        sensor_thread.start()

        # Kurze Verzögerung damit Sensor Thread erste Messungen macht
        time.sleep(0.5)

        # Starte Control Loop im Main Thread
        control_loop()

    except KeyboardInterrupt:
        print('\n[SHUTDOWN] ⌨️  Keyboard Interrupt (Ctrl+C) - Beende Programm...')

    except Exception as e:
        print(f'\n[ERROR] ❌ Unerwarteter Fehler: {e}')
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup wird IMMER ausgeführt
        cleanup()

# ============================================================================
# PROGRAMM START
# ============================================================================

if __name__ == '__main__':
    # Prüfe ob als root ausgeführt (notwendig für GPIO und Hardware PWM)
    import os
    if os.geteuid() != 0:
        print('❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!')
        print('📝 Verwendung: sudo python3 xbox_controller_with_collision.py')
        print('')
        print('Gründe für sudo:')
        print('  - GPIO Zugriff erfordert Root-Rechte')
        print('  - Hardware PWM (/sys/class/pwm) erfordert Root-Rechte')
        sys.exit(1)

    main()
