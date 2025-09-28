#!/usr/bin/env python3

"""
Robustes autonomes Navigations-Script für RPLIDAR C1 mit Ultraschallsensoren.
Löst den "Descriptor length mismatch" Fehler durch robuste Verbindungslogik.
Basiert auf bewährten Patterns aus lidar_health.py für zuverlässige LIDAR-Kommunikation.
"""

import time
import math
import signal
import sys

# Für Ultraschall und Motorsteuerung (nur auf Raspberry Pi)
import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

# Für RPLIDAR Kommunikation
from rplidar import RPLidar, RPLidarException

# ==== Konfiguration ====
# GPIO-Pins für Ultraschallsensoren
TRIG_LEFT = 12
ECHO_LEFT = 13
TRIG_RIGHT = 23
ECHO_RIGHT = 24
SOUND_SPEED = 34300  # Schallgeschwindigkeit in cm/s

# Motor PWM-Konfiguration
pwm_left = HardwarePWM(pwm_channel=0, hz=1000, chip=0)
pwm_right = HardwarePWM(pwm_channel=1, hz=1000, chip=0)

# Sicherheits- und Navigationsparameter
EMERGENCY_DISTANCE = 15.0  # Notabschaltung bei < 15cm
SAFETY_DISTANCE = 50.0     # Langsamere Fahrt bei < 50cm
BASE_SPEED = 12            # Grundgeschwindigkeit (% PWM)
MAX_SPEED = 25             # Maximalgeschwindigkeit (% PWM)
TURN_SPEED = 8             # Geschwindigkeit für Drehungen

def setup_gpio():
    """
    Initialisiert alle GPIO-Pins für Ultraschallsensoren und startet PWM für Motoren.
    Wartet 2 Sekunden für Sensor-Stabilisierung.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    
    # PWM starten mit 0% Duty Cycle (Motoren stoppen)
    pwm_left.start(0)
    pwm_right.start(0)
    
    # Trigger-Pins auf LOW setzen
    GPIO.output(TRIG_LEFT, False)
    GPIO.output(TRIG_RIGHT, False)
    
    print("GPIO initialisiert, Sensoren stabilisieren sich...")
    time.sleep(2)

def connect_lidar(port='/dev/ttyUSB0', baud=460800, timeout=1.0, retries=2, pause=1.0):
    """
    Robuste LIDAR-Verbindung mit automatischem Reconnect bei Descriptor-Fehlern.
    Basiert auf bewährter Logik aus lidar_health.py.
    
    Args:
        port: Serieller Port (Standard: /dev/ttyUSB0)
        baud: Baudrate für C1 (Standard: 460800)
        timeout: Timeout in Sekunden (Standard: 1.0)
        retries: Anzahl Wiederholungsversuche bei Fehlern
        pause: Wartezeit zwischen Wiederholungsversuchen
    
    Returns:
        RPLidar-Objekt bei Erfolg
    
    Raises:
        RPLidarException: Bei dauerhaften Verbindungsfehlern
    """
    attempt = 0
    last_exc = None
    
    while attempt <= retries:
        try:
            print(f"LIDAR-Verbindungsversuch {attempt + 1}/{retries + 1} auf {port}")
            
            # Neue LIDAR-Instanz erstellen
            lidar = RPLidar(port, baudrate=baud, timeout=timeout)
            
            # get_info() triggert Descriptor-Handshake und validiert Session
            info = lidar.get_info()
            print(f"LIDAR Info: {info}")
            
            # Health-Check durchführen
            health = lidar.get_health()
            print(f"LIDAR Health: {health}")
            
            # Health-Status prüfen
            status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')
            if status == 'Error':
                raise RuntimeError('LIDAR meldet Fehlerzustand - Hardware prüfen!')
            
            print("LIDAR erfolgreich verbunden und validiert!")
            return lidar
            
        except RPLidarException as e:
            last_exc = e
            print(f"LIDAR-Verbindungsfehler: {e}")
            
            # Sauberes Cleanup bei Fehlern
            try:
                if 'lidar' in locals() and lidar:
                    try:
                        lidar.stop()
                    except Exception:
                        pass
                    try:
                        lidar.stop_motor()
                    except Exception:
                        pass
                    lidar.disconnect()
            except Exception:
                pass
            
            # Pause vor nächstem Versuch
            if attempt < retries:
                print(f"Warte {pause} Sekunden vor nächstem Versuch...")
                time.sleep(pause)
            
            attempt += 1
    
    # Alle Versuche fehlgeschlagen
    raise last_exc if last_exc else RuntimeError('LIDAR-Verbindung fehlgeschlagen')

def measure_ultrasound(trig_pin, echo_pin):
    """
    Misst Entfernung mit HC-SR04 Ultraschallsensor.
    
    Args:
        trig_pin: GPIO-Pin für Trigger-Signal
        echo_pin: GPIO-Pin für Echo-Empfang
    
    Returns:
        Entfernung in cm oder None bei Messfehlern
    """
    try:
        # 10µs Trigger-Puls senden
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        
        # Warten auf Echo-Start mit Timeout
        timeout = time.time() + 0.1
        while GPIO.input(echo_pin) == 0:
            if time.time() > timeout:
                return None
        pulse_start = time.time()
        
        # Warten auf Echo-Ende mit Timeout
        timeout = time.time() + 0.1
        while GPIO.input(echo_pin) == 1:
            if time.time() > timeout:
                return None
        pulse_end = time.time()
        
        # Entfernungsberechnung
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * SOUND_SPEED) / 2
        
        return distance
        
    except Exception as e:
        print(f"Ultraschall-Messfehler: {e}")
        return None

def set_motor_speed(left_speed, right_speed):
    """
    Setzt Motorgeschwindigkeiten über PWM.
    
    Args:
        left_speed: PWM Duty Cycle für linken Motor (0-100%)
        right_speed: PWM Duty Cycle für rechten Motor (0-100%)
    """
    # Sicherheitsclipping auf gültigen Bereich
    left_speed = max(0, min(100, left_speed))
    right_speed = max(0, min(100, right_speed))
    
    pwm_left.change_duty_cycle(left_speed)
    pwm_right.change_duty_cycle(right_speed)

def get_lidar_scan_robust(lidar, max_retries=3):
    """
    Robuste LIDAR-Scan-Funktion mit Fehlerbehandlung.
    
    Args:
        lidar: Verbundenes RPLidar-Objekt
        max_retries: Maximale Wiederholungsversuche bei Fehlern
    
    Returns:
        Liste von (quality, angle, distance) Tupeln oder None bei Fehler
    """
    for attempt in range(max_retries):
        try:
            # Einen einzelnen Scan abrufen
            iterator = lidar.iter_scans(max_buf_meas=5000, min_len=5)
            scan = next(iterator)
            return scan
            
        except RPLidarException as e:
            print(f"LIDAR-Scan-Fehler (Versuch {attempt + 1}/{max_retries}): {e}")
            
            if attempt < max_retries - 1:
                # Kurze Pause vor Wiederholung
                time.sleep(0.1)
            else:
                print("LIDAR-Scan dauerhaft fehlgeschlagen!")
                return None
        
        except StopIteration:
            print("LIDAR-Iterator beendet - möglicherweise Verbindungsproblem")
            return None
    
    return None

def lidar_scan_to_sectors(scan, min_quality=10, min_dist=0.15, max_dist=12.0, num_sectors=8):
    """
    Wandelt LIDAR-Scan in Sektoren für einfache Navigation um.
    
    Args:
        scan: LIDAR-Scan-Daten als Liste von (quality, angle, distance) Tupeln
        min_quality: Mindest-Qualitätswert für gültige Messpunkte
        min_dist: Minimale gültige Entfernung (m)
        max_dist: Maximale gültige Entfernung (m)
        num_sectors: Anzahl der Richtungssektoren (Standard: 8 für 45°-Sektoren)
    
    Returns:
        Liste mit minimaler Entfernung pro Sektor (None wenn kein gültiger Punkt)
    """
    if not scan:
        return [None] * num_sectors
    
    # Initialize sectors with infinite distance
    sectors = [float('inf')] * num_sectors
    sector_angle = 360.0 / num_sectors
    
    for quality, angle, distance_mm in scan:
        # Filterung nach Qualität und Entfernung
        distance_m = distance_mm / 1000.0
        if quality < min_quality or distance_m < min_dist or distance_m > max_dist:
            continue
        
        # Sektor bestimmen (0° = vorne, im Uhrzeigersinn)
        sector_idx = int((angle % 360.0) / sector_angle) % num_sectors
        
        # Minimale Entfernung pro Sektor speichern
        sectors[sector_idx] = min(sectors[sector_idx], distance_m)
    
    # Unendliche Werte durch None ersetzen
    return [d if d != float('inf') else None for d in sectors]

def emergency_brake(ultra_left, ultra_right, threshold=EMERGENCY_DISTANCE):
    """
    Prüft, ob Notabschaltung wegen zu nahem Hindernis erforderlich ist.
    
    Args:
        ultra_left: Entfernung linker Ultraschallsensor (cm)
        ultra_right: Entfernung rechter Ultraschallsensor (cm)  
        threshold: Schwellwert für Notabschaltung (cm)
    
    Returns:
        True wenn Notabschaltung erforderlich, sonst False
    """
    if ultra_left is not None and ultra_left < threshold:
        return True
    if ultra_right is not None and ultra_right < threshold:
        return True
    return False

def navigate_with_sectors(sectors):
    """
    Bestimmt Fahrbefehle basierend auf LIDAR-Sektordaten.
    
    Args:
        sectors: Liste mit Entfernungen pro Sektor (8 Sektoren, 45° je Sektor)
                Sektor 0 = vorne, Sektor 2 = rechts, Sektor 4 = hinten, Sektor 6 = links
    
    Returns:
        Tupel (left_speed, right_speed) für Motorsteuerung
    """
    if not sectors or len(sectors) < 8:
        # Fehlerfall: Stoppen
        return (0, 0)
    
    # Sektor-Indizes (0=vorne, 1=vorne-rechts, 2=rechts, usw.)
    front = sectors[0]           # 0°
    front_right = sectors[1]     # 45°
    right = sectors[2]           # 90°
    back_right = sectors[3]      # 135°
    back = sectors[4]            # 180°
    back_left = sectors[5]       # 225°
    left = sectors[6]           # 270°
    front_left = sectors[7]      # 315°
    
    # Entscheidungslogik für Navigation
    # 1. Prüfung: Ist der Weg nach vorne frei?
    if front is None or front > 1.0:  # > 1m frei
        # Geradeausfahrt möglich
        return (BASE_SPEED, BASE_SPEED)
    
    # 2. Prüfung: Hindernisse in Frontsektoren
    if front is not None and front < 0.8:  # < 80cm
        # Hindernis vorne - Drehung erforderlich
        
        # Links oder rechts freier?
        left_free = (front_left is None or front_left > 0.6) and (left is None or left > 0.6)
        right_free = (front_right is None or front_right > 0.6) and (right is None or right > 0.6)
        
        if left_free and not right_free:
            # Links drehen (rechter Motor schneller)
            return (TURN_SPEED//2, TURN_SPEED)
        elif right_free and not left_free:
            # Rechts drehen (linker Motor schneller)  
            return (TURN_SPEED, TURN_SPEED//2)
        elif left_free and right_free:
            # Beide Seiten frei - bevorzuge rechts
            return (TURN_SPEED, TURN_SPEED//2)
        else:
            # Beide Seiten blockiert - langsam rückwärts
            return (-TURN_SPEED//2, -TURN_SPEED//2)
    
    # 3. Standard: Langsame Vorwärtsfahrt
    return (BASE_SPEED//2, BASE_SPEED//2)

def cleanup(lidar=None):
    """
    Sauberes Herunterfahren aller Systeme.
    
    Args:
        lidar: LIDAR-Objekt zum sauberen Trennen (optional)
    """
    print("System wird heruntergefahren...")
    
    # Motoren stoppen
    set_motor_speed(0, 0)
    
    # PWM beenden
    try:
        pwm_left.stop()
        pwm_right.stop()
    except Exception:
        pass
    
    # GPIO freigeben
    try:
        GPIO.cleanup()
    except Exception:
        pass
    
    # LIDAR sauber trennen
    if lidar:
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass
    
    print("System sicher heruntergefahren.")

def signal_handler(sig, frame):
    """
    Signal-Handler für sauberes Beenden bei Ctrl+C.
    """
    print('\nProgramm wird durch Benutzer beendet...')
    cleanup()
    sys.exit(0)

def main():
    """
    Hauptschleife des autonomen Navigationssystems.
    Kombiniert LIDAR-Navigation mit Ultraschall-Notabschaltung.
    """
    # Signal-Handler registrieren
    signal.signal(signal.SIGINT, signal_handler)
    
    lidar = None
    
    try:
        # System initialisieren
        print("=== Autonomes Navigationssystem gestartet ===")
        setup_gpio()
        
        # LIDAR verbinden mit robuster Methode
        print("Verbinde LIDAR...")
        lidar = connect_lidar()
        
        # Optional: Motor starten (bei C1 meist automatisch)
        try:
            lidar.start_motor()
            time.sleep(0.5)  # Kurz warten für Motoranlauf
        except Exception as e:
            print(f"Motor-Start nicht erforderlich/fehlgeschlagen: {e}")
        
        print("Navigation aktiv - Beenden mit Ctrl+C")
        
        # Hauptnavigationsschleife
        while True:
            start_cycle = time.time()
            
            # 1. Ultraschall-Sicherheitsprüfung
            dist_left = measure_ultrasound(TRIG_LEFT, ECHO_LEFT)
            dist_right = measure_ultrasound(TRIG_RIGHT, ECHO_RIGHT)
            
            if emergency_brake(dist_left, dist_right):
                print(f"NOTABSCHALTUNG! Links: {dist_left}cm, Rechts: {dist_right}cm")
                set_motor_speed(0, 0)
                time.sleep(0.5)  # Länger warten bei Notfall
                continue
            
            # 2. LIDAR-Scan abrufen
            scan = get_lidar_scan_robust(lidar)
            if scan is None:
                print("LIDAR-Scan fehlgeschlagen - Stoppe Motoren")
                set_motor_speed(0, 0)
                time.sleep(0.2)
                continue
            
            # 3. Sektordaten berechnen
            sectors = lidar_scan_to_sectors(scan)
            
            # 4. Navigation bestimmen
            left_speed, right_speed = navigate_with_sectors(sectors)
            
            # 5. Motoren ansteuern
            set_motor_speed(left_speed, right_speed)
            
            # 6. Status ausgeben
            cycle_time = (time.time() - start_cycle) * 1000
            front_dist = sectors[0] if sectors[0] is not None else "∞"
            print(f"Scan: {len(scan)} Punkte, Vorne: {front_dist}m, "
                  f"Motor: L{left_speed}% R{right_speed}%, Zeit: {cycle_time:.1f}ms")
            
            # Zykluszeit regulieren (ca. 5-10 Hz)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("Programmende durch Benutzer")
    
    except Exception as e:
        print(f"Unerwarteter Fehler: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        cleanup(lidar)

if __name__ == "__main__":
    main()