#!/usr/bin/env python3
"""
HC-SR04 Ultraschallsensor Test Script für Raspberry Pi
Autor: [Ihr Name]
Datum: 2025

Dieses Script testet den HC-SR04 Ultraschallsensor und misst kontinuierlich Entfernungen.
Der Sensor muss korrekt verdrahtet sein (siehe Anleitung).

Verdrahtung:
- VCC: Pin 2 (5V)
- GND: Pin 6 (Ground)  
- TRIG: Pin 16 (GPIO 23)
- ECHO: Pin 18 (GPIO 24) über Spannungsteiler (1kΩ + 2kΩ)
"""

import RPi.GPIO as GPIO
import time
import signal
import sys

# GPIO Pin Definitionen
TRIG_PIN = 12  # GPIO 12 (Pin 16)
ECHO_PIN = 13 # GPIO 13 (Pin 18)

# Schallgeschwindigkeit in cm/s bei 20°C
SOUND_SPEED = 34300

def signal_handler(sig, frame):
    """Handler für Ctrl+C zum sauberen Beenden"""
    print('\n\nProgramm wird beendet...')
    cleanup()
    sys.exit(0)

def setup():
    """GPIO Setup und Initialisierung"""
    print("HC-SR04 Ultraschallsensor Test")
    print("=" * 40)
    print(f"TRIG Pin: GPIO {TRIG_PIN}")
    print(f"ECHO Pin: GPIO {ECHO_PIN}")
    print("Drücken Sie Ctrl+C zum Beenden\n")

    # GPIO Modus setzen
    GPIO.setmode(GPIO.BCM)

    # Pins konfigurieren
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # TRIG Pin auf LOW setzen
    GPIO.output(TRIG_PIN, False)

    # Signal Handler für Ctrl+C registrieren
    signal.signal(signal.SIGINT, signal_handler)

    print("Sensor wird initialisiert...")
    time.sleep(2)

def measure_distance():
    """
    Misst die Entfernung mit dem HC-SR04 Sensor

    Returns:
        float: Entfernung in cm, oder None bei Timeout
    """
    try:
        # TRIG Pin für 10µs auf HIGH setzen
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)  # 10 Mikrosekunden
        GPIO.output(TRIG_PIN, False)

        # Warten bis ECHO Pin HIGH wird (Start der Messung)
        timeout = time.time() + 0.1  # 100ms Timeout
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None

        # Warten bis ECHO Pin LOW wird (Ende der Messung)
        timeout = time.time() + 0.1  # 100ms Timeout
        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None

        # Zeitdifferenz berechnen
        pulse_duration = pulse_end - pulse_start

        # Entfernung berechnen (Zeit * Schallgeschwindigkeit / 2)
        # Division durch 2, da der Schall hin und zurück reist
        distance = (pulse_duration * SOUND_SPEED) / 2

        return distance

    except Exception as e:
        print(f"Fehler bei der Messung: {e}")
        return None

def format_distance(distance):
    """
    Formatiert die Entfernung für die Ausgabe

    Args:
        distance (float): Entfernung in cm

    Returns:
        str: Formatierte Entfernung
    """
    if distance is None:
        return "Timeout/Fehler"
    elif distance < 2:
        return "< 2 cm (zu nah)"
    elif distance > 400:
        return "> 400 cm (zu weit)"
    else:
        return f"{distance:.1f} cm"

def cleanup():
    """GPIO Cleanup"""
    GPIO.cleanup()
    print("GPIO bereinigt.")

def main():
    """Hauptprogramm"""
    try:
        setup()

        print("Starte kontinuierliche Messung...")
        print("-" * 40)

        measurement_count = 0

        while True:
            distance = measure_distance()
            measurement_count += 1

            # Zeitstempel für Messung
            timestamp = time.strftime("%H:%M:%S")

            # Ergebnis ausgeben
            distance_str = format_distance(distance)
            print(f"[{timestamp}] Messung #{measurement_count:3d}: {distance_str}")

            # Kurze Pause zwischen Messungen
            time.sleep(0.5)

    except KeyboardInterrupt:
        signal_handler(None, None)
    except Exception as e:
        print(f"Unerwarteter Fehler: {e}")
        cleanup()

if __name__ == "__main__":
    main()
