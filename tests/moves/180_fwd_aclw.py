#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Zweck:
- Sanftes Fahren mit zwei DC-Motoren (links/rechts) über Hardware-PWM am Raspberry Pi
- Ablauf: vorwärts fahren -> 360° Drehung (gegenläufig) -> rückwärts fahren
- Sanfte Anfahr- und Abbremsrampen zum mechanisch/elektrisch schonenden Betrieb

Voraussetzungen:
- RPi.GPIO für Richtungs-Pins (Digitalausgänge)
- rpi_hardware_pwm.HardwarePWM für Hardware-PWM (präzise Duty-Cycle-Steuerung)
- Aktiviertes PWM-Overlay (z. B. dtoverlay=pwm-2chan), Zuordnung i. d. R. PWM0->GPIO18, PWM1->GPIO19
  - Auf dem Raspberry Pi stehen PWM-Kanäle/GPIOs wie folgt zur Verfügung (modellabhängig):
    - Klassisch: PWM0 = GPIO12/18, PWM1 = GPIO13/19
    - Pi 5: PWM0=GPIO12, PWM1=GPIO13, PWM2=GPIO18, PWM3=GPIO19
  - Siehe Bibliotheks-/Plattformhinweise zur Kanal/GPIO-Mapping und Overlay-Konfiguration.
"""

import time
import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

# ---------------------------------------
# Pinbelegung (BCM) für Richtungssteuerung
# Je ein DIR-Pin pro Motor, um Drehrichtung getrennt zu schalten
# ---------------------------------------
DIR_LEFT_PIN = 20   # Richtung links (GPIO20)
DIR_RIGHT_PIN = 21  # Richtung rechts (GPIO21)

# ---------------------------------------
# Hardware-PWM-Konfiguration
# - PWM_CHIP und pwm_channel hängen von Overlay/Kernel ab
# - Im üblichen Setup (dtoverlay=pwm-2chan) entsprechen:
#   - channel=0 -> PWM0 (typisch GPIO18)
#   - channel=1 -> PWM1 (typisch GPIO19)
# Prüfen/Anpassen je nach Board/Overlay (siehe Doku der PWM-Bibliothek/Plattform)
# ---------------------------------------
PWM_LEFT_CHANNEL = 0
PWM_RIGHT_CHANNEL = 1
PWM_CHIP = 0  # häufig 0

# ---------------------------------------
# Allgemeine Konfiguration
# ---------------------------------------
FREQUENCY_HZ = 1000     # PWM-Frequenz; 1 kHz ist oft gut für DC-Motoren
SPEED_DUTY = 10         # % Duty-Cycle für lineares Fahren (vorwärts/rückwärts)
DURATION_S = 2          # Sekunden pro geraden Fahrabschnitt

# Richtungslogik:
# - True  => HIGH = rückwärts, LOW = vorwärts
# - False => HIGH = vorwärts, LOW = rückwärts
# Falls die Drehrichtung am Fahrzeug invertiert ist, hier umschalten.
DIR_HIGH_IS_BACKWARD = True

# ---------------------------------------
# Sanftes Fahren (Rampen)
# ---------------------------------------
RAMP_STEP = 1           # Schrittweite in Duty-% pro Schritt
RAMP_STEP_DELAY = 0.2   # Wartezeit je Rampenschritt (s)
STOP_PAUSE_S = 0.5      # Pause im Stillstand vor Richtungswechsel (s)

# ---------------------------------------
# Dreh-Konfiguration (360°)
# ---------------------------------------
# Die Drehung wird zeitbasiert angenähert (ohne Encoder); Wert kalibrieren:
# - TURN_DURATION_S so anpassen, bis ~360° erreicht werden
# - TURN_DUTY moderat wählen, um kontrollierte Drehung zu erhalten
TURN_DUTY = 12          # % Duty-Cycle für die Drehung (ggf. leicht höher als SPEED_DUTY)
TURN_DURATION_S = 0.8   # Sekunden für ~360° (kalibrierpflichtig)
TURN_CLOCKWISE = False   # True = im Uhrzeigersinn, False = gegen den Uhrzeigersinn

def setup():
    """
    Initialisiert GPIO-Modus/Einstellungen und startet die Hardware-PWM mit 0% Duty.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

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

    # Hardware-PWM-Objekte erstellen und mit 0% Duty starten
    pwm_left = HardwarePWM(pwm_channel=PWM_LEFT_CHANNEL, hz=FREQUENCY_HZ, chip=PWM_CHIP)
    pwm_right = HardwarePWM(pwm_channel=PWM_RIGHT_CHANNEL, hz=FREQUENCY_HZ, chip=PWM_CHIP)
    pwm_left.start(0)
    pwm_right.start(0)
    return pwm_left, pwm_right

def set_direction(backward: bool):
    """
    Setzt beide Motoren auf die gleiche Fahrtrichtung (vorwärts/rückwärts).
    - backward=True  -> beide rückwärts
    - backward=False -> beide vorwärts
    Die logische Zuordnung hängt von DIR_HIGH_IS_BACKWARD ab.
    """
    if DIR_HIGH_IS_BACKWARD:
        level = GPIO.HIGH if backward else GPIO.LOW
    else:
        level = GPIO.LOW if backward else GPIO.HIGH

    GPIO.output(DIR_LEFT_PIN, level)
    GPIO.output(DIR_RIGHT_PIN, level)

def set_turn_direction(clockwise: bool):
    """
    Setzt gegenläufige Richtungen für eine Drehung auf der Stelle.
    - clockwise=True  -> linker Motor rückwärts, rechter Motor vorwärts (typ. Drehung im Uhrzeigersinn)
    - clockwise=False -> linker Motor vorwärts, rechter Motor rückwärts (typ. gegen den Uhrzeigersinn)
    Hinweis: Je nach Motorverdrahtung/Fahrgestell kann der tatsächliche Drehsinn invertiert sein.
    """
    if DIR_HIGH_IS_BACKWARD:
        # HIGH = rückwärts, LOW = vorwärts
        if clockwise:
            left_level = GPIO.HIGH   # links rückwärts
            right_level = GPIO.LOW   # rechts vorwärts
        else:
            left_level = GPIO.LOW    # links vorwärts
            right_level = GPIO.HIGH  # rechts rückwärts
    else:
        # HIGH = vorwärts, LOW = rückwärts
        if clockwise:
            left_level = GPIO.LOW    # links rückwärts
            right_level = GPIO.HIGH  # rechts vorwärts
        else:
            left_level = GPIO.HIGH   # links vorwärts
            right_level = GPIO.LOW   # rechts rückwärts

    GPIO.output(DIR_LEFT_PIN, left_level)
    GPIO.output(DIR_RIGHT_PIN, right_level)

def soft_ramp(pwm_left, pwm_right, start: int, end: int,
              step: int = RAMP_STEP, delay_s: float = RAMP_STEP_DELAY):
    """
    Führt eine lineare Rampe des Duty-Cycles von start -> end in 'step'-Schritten aus.
    Clamping auf [0..100] und automatische Richtung (auf/absteigend).
    """
    # Sicherstellen, dass Werte im gültigen Bereich liegen
    start = max(0, min(100, int(start)))
    end = max(0, min(100, int(end)))
    step = max(1, int(step))

    # Auf- oder Abwärtsbereich erzeugen
    if start <= end:
        rng = range(start, end + 1, step)
    else:
        rng = range(start, end - 1, -step)

    # Schrittweise Duty-Cycle anpassen
    for duty in rng:
        pwm_left.change_duty_cycle(duty)
        pwm_right.change_duty_cycle(duty)
        time.sleep(delay_s)

def soft_run_section(pwm_left, pwm_right, backward: bool, duration_s: float, target_duty: int):
    """
    Fährt eine lineare Strecke sanft an, hält konstante Geschwindigkeit und stoppt sanft.
    - backward: True=rückwärts, False=vorwärts
    - duration_s: Dauer der Konstantfahrt
    - target_duty: Duty-Cycle-Ziel in %
    """
    # Richtung für beide Motoren gleich setzen
    set_direction(backward)

    # Sanft anfahren: 0% -> target_duty
    soft_ramp(pwm_left, pwm_right, start=0, end=target_duty,
              step=RAMP_STEP, delay_s=RAMP_STEP_DELAY)

    # Konstante Fahrt
    time.sleep(max(0.0, float(duration_s)))

    # Sanft stoppen: target_duty -> 0%
    soft_ramp(pwm_left, pwm_right, start=target_duty, end=0,
              step=RAMP_STEP, delay_s=RAMP_STEP_DELAY)

    # Kurze Pause im Stillstand vor Richtungswechsel
    time.sleep(STOP_PAUSE_S)

def soft_turn_section(pwm_left, pwm_right, clockwise: bool, duration_s: float, target_duty: int):
    """
    Führt eine sanfte Drehung auf der Stelle aus:
    - setzt gegenläufige Richtungen gemäß 'clockwise'
    - ramped 0% -> target_duty, hält, ramped zurück auf 0%
    - beinhaltet eine kurze Stillstandspause am Ende
    Hinweis: 'duration_s' ist kalibrierpflichtig, um ~360° zu erreichen (ohne Encoder).
    """
    # Gegenläufige Richtungen setzen (links/rechts)
    set_turn_direction(clockwise)

    # Sanft anfahren: 0% -> target_duty (beide PWM-Kanäle identisch, Richtungen sind gegensätzlich)
    soft_ramp(pwm_left, pwm_right, start=0, end=target_duty,
              step=RAMP_STEP, delay_s=RAMP_STEP_DELAY)

    # Haltephase (Drehung läuft)
    time.sleep(max(0.0, float(duration_s)))

    # Sanft stoppen: target_duty -> 0%
    soft_ramp(pwm_left, pwm_right, start=target_duty, end=0,
              step=RAMP_STEP, delay_s=RAMP_STEP_DELAY)

    # Kurze Pause im Stillstand vor weiterem Abschnitt
    time.sleep(STOP_PAUSE_S)

def main():
    print("=== Vorwärts -> 360° Drehung -> Rückwärts (sanft) ===")

    # Initialisierung
    pwm_left, pwm_right = setup()

    try:

        # 2) 360°-Drehung (zeitbasiert; ggf. TURN_DURATION_S kalibrieren)
        print("360°-Drehung ausführen (gegenläufige Motoren)...")
        soft_turn_section(pwm_left, pwm_right, clockwise=TURN_CLOCKWISE,
                          duration_s=TURN_DURATION_S, target_duty=TURN_DUTY)
        
        # 1) Vorwärts
        print("Sanft vorwärts anfahren, fahren, sanft stoppen...")
        soft_run_section(pwm_left, pwm_right, backward=True,
                         duration_s=DURATION_S, target_duty=SPEED_DUTY)

        print("Ablauf abgeschlossen.")
    except KeyboardInterrupt:
        print("Abgebrochen (Strg+C).")
    finally:
        # PWM und GPIO sicher stoppen/aufräumen
        try:
            pwm_left.stop()
            pwm_right.stop()
        except Exception:
            pass
        GPIO.cleanup()
        print("Aufgeräumt.")

if __name__ == "__main__":
    main()
