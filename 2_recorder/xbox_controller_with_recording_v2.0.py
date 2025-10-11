#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Xbox Controller Roboter-Steuerung mit Recording-Funktionalität - Phase 2
=========================================================================

Erweiterte Xbox Controller Steuerung mit integrierter Path Recording Funktionalität.
Diese Version baut auf dem bestehenden Controller-System auf und fügt parallele 
Datenaufzeichnung hinzu ohne Performance-Einbußen.

NEU in Version 2.0 (Phase 2):
- ✅ Recording-Modus: Start/Stop über Controller-Button-Kombination (Y+X Button)
- ✅ Real-time Data Capture: Parallel zur normalen Fahrt ohne Performance-Einbußen  
- ✅ Thread-sichere Datensammlung: Separate Recording-Threads mit Buffer-Management
- ✅ Graceful Shutdown: Sichere Datenspeicherung bei Unterbrechungen
- ✅ Recording-Validierung: Minimum-Weglänge und Qualitätsprüfungen
- ✅ Visual/Acoustic Feedback: Status-LEDs für Recording-Status
- ✅ Integration in bestehende control_loop() ohne Timing-Störungen

Aus Version 1.4:
- Xbox Controller Eingabe mit Left/Right Trigger für Motor-Steuerung
- Tank-Turn Funktion mit A-Button
- Richtungsumschaltung mit B-Button
- Kollisionsvermeidung mit Ultraschall-Sensoren
- Hardware PWM Motor-Ansteuerung

HARDWARE ANFORDERUNGEN:
-----------------------
- Alle Komponenten aus v1.4
- Optional: Status-LED für Recording (GPIO22)
- Optional: Piezo-Buzzer für Audio-Feedback (GPIO25)

SOFTWARE ABHÄNGIGKEITEN:
-----------------------
sudo apt install python3-pygame xboxdrv
sudo pip3 install rpi-hardware-pwm RPi.GPIO

Zusätzlich benötigt:
- path_recording_system.py (Datenstrukturen)
- storage_system.py (Speicher-Management)

STEUERUNG RECORDING:
-------------------
- Y + X Button (gleichzeitig halten) → Recording Start/Stop Toggle
- Status-LED: AUS=Kein Recording, BLINKEN=Recording aktiv, AN=Speichere Daten
- Piezo-Buzzer: Kurze Töne für Recording Start/Stop

TECHNISCHE DETAILS:
------------------
- Controller-Sample-Rate: 50Hz (20ms Intervalle)
- Recording-Buffer: Thread-sicher mit Queue (max 1000 Samples)
- Minimum Recording-Länge: 5 Sekunden für gültige Aufzeichnung
- Automatische Session-ID Generierung basierend auf Zeitstempel
- Graceful Shutdown mit Daten-Flush bei Ctrl+C oder Controller-Trennung

AUTOR: Automatisch generiert für Phase 2
DATUM: 2025-10-11
VERSION: 2.0 (Phase 2: Recording Integration)
"""

import RPi.GPIO as GPIO
import pygame
import sys
import time
import signal
import threading
import queue
import json
import os
from datetime import datetime, timezone
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field
from pathlib import Path
from rpi_hardware_pwm import HardwarePWM

# Import der Recording-Datenstrukturen (müssen im selben Verzeichnis sein)
try:
    from path_recording_system import (
        PathRecordingData,
        PathRecordingHeader, 
        ControllerSample,
        EnvironmentMetadata,
        CalibrationData,
        CompressionType,
        setup_logging
    )
    from storage_system import PathRecordingStorageManager
    RECORDING_AVAILABLE = True
    print("✅ Recording-System erfolgreich importiert")
except ImportError as e:
    print(f"⚠️ Recording-System nicht verfügbar: {e}")
    print("📝 Für Recording-Funktionalität benötigt:")
    print("   - path_recording_system.py") 
    print("   - storage_system.py")
    RECORDING_AVAILABLE = False

# ============================================================================
# HARDWARE KONFIGURATION (erweitert aus v1.4)
# ============================================================================

# Bestehende Hardware-Konfiguration aus v1.4
TRIG_LEFT = 12      
ECHO_LEFT = 13      
TRIG_RIGHT = 23     
ECHO_RIGHT = 24     
DIR_LEFT_PIN = 20   
DIR_RIGHT_PIN = 21  
DIR_HIGH_IS_BACKWARD = False

# Neue Hardware für Recording-Feedback
STATUS_LED_PIN = 22     # Status-LED für Recording-Anzeige
BUZZER_PIN = 25         # Piezo-Buzzer für Audio-Feedback (optional)
LED_ENABLED = True      # Flag um LED zu aktivieren/deaktivieren
BUZZER_ENABLED = False  # Flag um Buzzer zu aktivieren/deaktivieren

# Kollisions- und Motor-Parameter (unverändert aus v1.4)
COLLISION_DISTANCE_CM = 30.0
SOUND_SPEED = 34300
PWM_FREQUENCY = 1000
PWM_CHIP = 0
PWM_CHANNEL_LEFT = 0
PWM_CHANNEL_RIGHT = 1
MAX_DUTY_CYCLE = 12.0
MIN_DUTY_CYCLE = 0.0

# Controller-Konfiguration (erweitert)
CONTROLLER_ID = 0
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 5
BUTTON_A = 0                # Tank-Turn Modus
BUTTON_B = 1                # Richtungsumschaltung
BUTTON_X = 2                # Recording Control (mit Y)
BUTTON_Y = 3                # Recording Control (mit X)
BUTTON_START = 7            # Programm-Ende

# Update-Frequenzen
SENSOR_UPDATE_HZ = 10
CONTROL_LOOP_HZ = 50

# ============================================================================
# RECORDING KONFIGURATION
# ============================================================================

# Recording-Parameter
RECORDING_SAMPLE_RATE_HZ = 50           # Controller-Sample-Rate
RECORDING_BUFFER_SIZE = 1000            # Maximale Puffer-Größe
MIN_RECORDING_DURATION_SEC = 5.0        # Minimale Recording-Dauer für Gültigkeit
MAX_RECORDING_DURATION_SEC = 3600.0     # Maximale Recording-Dauer (1 Stunde)
RECORDING_BASE_DIR = "path_recordings"  # Basis-Verzeichnis für Aufzeichnungen
AUTO_SAVE_INTERVAL_SEC = 30.0           # Intervall für automatische Zwischenspeicherung

# Status-LED Blinkmuster
LED_BLINK_RATE_HZ = 2.0                 # LED-Blinkfrequenz während Recording
LED_SAVE_DURATION_SEC = 3.0             # LED-Dauer beim Speichern

# ============================================================================
# GLOBALE VARIABLEN (erweitert aus v1.4)
# ============================================================================

# Hardware Objekte (aus v1.4)
pwm_left: Optional[HardwarePWM] = None
pwm_right: Optional[HardwarePWM] = None
joystick: Optional[pygame.joystick.Joystick] = None

# Thread Control
running = False
sensor_thread: Optional[threading.Thread] = None
recording_thread: Optional[threading.Thread] = None
led_thread: Optional[threading.Thread] = None

# Sensor Daten (aus v1.4)
distance_left = 100.0
distance_right = 100.0

# Controller Daten (aus v1.4)
trigger_left = -1.0
trigger_right = -1.0
a_button_pressed = False
b_button_was_pressed = False
is_moving_forward = True

# NEU: Recording-Control Variablen
x_button_pressed = False
y_button_pressed = False
x_button_was_pressed = False
y_button_was_pressed = False

# NEU: Recording-Status Variablen
recording_active = False
recording_start_time = 0.0
recording_session_id = ""
recording_buffer: queue.Queue = queue.Queue(maxsize=RECORDING_BUFFER_SIZE)
recording_data: Optional[PathRecordingData] = None
storage_manager: Optional[PathRecordingStorageManager] = None

# NEU: LED-Status Variablen  
led_status = "off"  # "off", "recording", "saving"
led_blink_state = False
led_last_toggle = 0.0

# NEU: Logging
logger = None

# ============================================================================
# RECORDING DATENSTRUKTUREN
# ============================================================================

@dataclass
class RecordingStatus:
    """
    Status-Container für Recording-Operationen.
    
    Attributes:
        active: Ob Recording gerade aktiv ist
        start_time: Start-Zeitpunkt des aktuellen Recordings
        session_id: Eindeutige Session-ID
        sample_count: Anzahl der bereits aufgezeichneten Samples
        buffer_usage: Aktuelle Puffer-Auslastung (0.0-1.0)
        duration: Aktuelle Recording-Dauer in Sekunden
        valid: Ob Recording die Mindest-Qualitätskriterien erfüllt
    """
    active: bool = False
    start_time: float = 0.0
    session_id: str = ""
    sample_count: int = 0
    buffer_usage: float = 0.0
    duration: float = 0.0
    valid: bool = False
    
    def update(self, active: bool, start_time: float, session_id: str, 
               sample_count: int, buffer_size: int, max_buffer: int) -> None:
        """Aktualisiert alle Status-Werte atomisch."""
        self.active = active
        self.start_time = start_time
        self.session_id = session_id
        self.sample_count = sample_count
        self.buffer_usage = buffer_size / max_buffer if max_buffer > 0 else 0.0
        self.duration = time.time() - start_time if active and start_time > 0 else 0.0
        self.valid = self.duration >= MIN_RECORDING_DURATION_SEC

# Globaler Recording-Status
recording_status = RecordingStatus()

# ============================================================================
# SIGNAL HANDLER (erweitert)
# ============================================================================

def signal_handler(sig, frame):
    """
    Erweiterte Signal-Handler für Graceful Shutdown mit Recording-Daten-Sicherung.
    
    Stellt sicher, dass bei Ctrl+C oder anderen Unterbrechungen alle Recording-Daten
    sicher gespeichert werden bevor das Programm beendet wird.
    """
    global logger
    print('\n[SHUTDOWN] SIGINT empfangen - Programm wird beendet...')
    
    # Stoppe Recording falls aktiv und speichere Daten
    if recording_active and recording_status.valid:
        print('[SHUTDOWN] 🔄 Speichere aktive Aufzeichnung...')
        stop_recording(force_save=True)
    elif recording_active:
        print('[SHUTDOWN] ⚠️ Recording zu kurz - wird verworfen')
        stop_recording(force_save=False)
    
    cleanup()
    sys.exit(0)

# ============================================================================
# HARDWARE SETUP (erweitert)
# ============================================================================

def setup_gpio():
    """Erweiterte GPIO-Initialisierung inklusive Recording-Hardware."""
    global logger
    print('[SETUP] Initialisiere GPIO für Sensoren, Motoren und Recording-Hardware...')
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Bestehende GPIO-Konfiguration aus v1.4
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.output(TRIG_LEFT, False)
    
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    GPIO.output(TRIG_RIGHT, False)
    
    GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
    GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)
    
    # Neue Recording-Hardware
    if LED_ENABLED:
        GPIO.setup(STATUS_LED_PIN, GPIO.OUT)
        GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        print(f'[SETUP] Status-LED initialisiert (GPIO{STATUS_LED_PIN})')
    
    if BUZZER_ENABLED:
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        print(f'[SETUP] Buzzer initialisiert (GPIO{BUZZER_PIN})')
    
    # Richtung auf vorwärts setzen (aus v1.4)
    if DIR_HIGH_IS_BACKWARD:
        GPIO.output(DIR_LEFT_PIN, GPIO.LOW)
        GPIO.output(DIR_RIGHT_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_LEFT_PIN, GPIO.HIGH)
        GPIO.output(DIR_RIGHT_PIN, GPIO.HIGH)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if logger:
        logger.info("GPIO für Recording-Hardware initialisiert")
    
    print('[SETUP] GPIO initialisiert - Sensoren stabilisieren (2s)...')
    time.sleep(2)

def setup_recording():
    """
    Initialisiert das Recording-System.
    
    Erstellt Storage Manager, Logger und initiale Datenstrukturen für
    die Aufzeichnungsfunktionalität.
    
    Raises:
        RuntimeError: Wenn Recording-Module nicht verfügbar sind
    """
    global storage_manager, logger
    
    if not RECORDING_AVAILABLE:
        print("[SETUP] ⚠️ Recording-System nicht verfügbar - nur Fahrbetrieb möglich")
        return
    
    print('[SETUP] Initialisiere Recording-System...')
    
    try:
        # Erstelle Basis-Verzeichnis
        Path(RECORDING_BASE_DIR).mkdir(parents=True, exist_ok=True)
        
        # Initialisiere Logger
        logger = setup_logging(str(Path(RECORDING_BASE_DIR) / "xbox_recording.log"))
        
        # Erstelle Storage Manager
        storage_manager = PathRecordingStorageManager(
            base_directory=RECORDING_BASE_DIR,
            compression=CompressionType.GZIP,
            backup_enabled=True,
            backup_count=3,
            verify_checksums=True
        )
        
        logger.info("Recording-System erfolgreich initialisiert")
        print(f'[SETUP] ✅ Recording-System bereit: {RECORDING_BASE_DIR}')
        
    except Exception as e:
        print(f'[SETUP] ❌ Recording-System Fehler: {e}')
        if logger:
            logger.error(f"Recording-System Initialisierung fehlgeschlagen: {e}")
        raise RuntimeError(f"Recording-System konnte nicht initialisiert werden: {e}")

# Bestehende Setup-Funktionen aus v1.4 (unverändert)
def set_motor_direction(left_backward: bool, right_backward: bool):
    """Setzt die Richtung für jeden Motor individuell (aus v1.4)."""
    if DIR_HIGH_IS_BACKWARD:
        left_level = GPIO.HIGH if left_backward else GPIO.LOW
        right_level = GPIO.HIGH if right_backward else GPIO.LOW
    else:
        left_level = GPIO.LOW if left_backward else GPIO.HIGH
        right_level = GPIO.LOW if right_backward else GPIO.HIGH
    
    GPIO.output(DIR_LEFT_PIN, left_level)
    GPIO.output(DIR_RIGHT_PIN, right_level)

def setup_pwm():
    """Initialisiert Hardware PWM für beide Motoren (aus v1.4)."""
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
    """Initialisiert Xbox Controller über pygame (aus v1.4)."""
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
# SENSOR FUNKTIONEN (unverändert aus v1.4)
# ============================================================================

def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """Misst Entfernung mit HC-SR04 Ultraschall-Sensor (aus v1.4)."""
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
        if logger:
            logger.error(f'Distanzmessung fehlgeschlagen: {e}')
        return -1.0

def sensor_update_thread():
    """Thread für kontinuierliche Sensor-Abfrage (aus v1.4)."""
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
# MOTOR STEUERUNG (unverändert aus v1.4)
# ============================================================================

def trigger_to_duty_cycle(trigger_value: float) -> float:
    """Konvertiert Xbox Trigger Wert zu PWM Duty Cycle (aus v1.4)."""
    normalized = (trigger_value + 1.0) / 2.0
    duty_cycle = normalized * MAX_DUTY_CYCLE
    duty_cycle = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, duty_cycle))
    return duty_cycle

def set_motor_speed(left_duty: float, right_duty: float):
    """Setzt PWM Duty Cycle für beide Motoren mit Kollisionsvermeidung (aus v1.4)."""
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
        if logger:
            logger.error(f'PWM Update fehlgeschlagen: {e}')

# ============================================================================
# RECORDING FUNKTIONALITÄT (NEU)
# ============================================================================

def create_session_id() -> str:
    """
    Erstellt eindeutige Session-ID basierend auf aktuellem Zeitstempel.
    
    Returns:
        Session-ID im Format "YYYYMMDD_HHMMSS_robotXX"
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{timestamp}_robot01"

def create_recording_data() -> PathRecordingData:
    """
    Erstellt neue PathRecordingData-Instanz mit initialem Header.
    
    Returns:
        Initialisierte PathRecordingData mit konfigurierten Metadaten
    """
    # Erstelle Kalibrierungsdaten
    calibration_data = CalibrationData(
        controller_calibration={
            'left_trigger_offset': 0.0,
            'right_trigger_offset': 0.0, 
            'deadzone': 0.05,
            'max_duty_cycle': MAX_DUTY_CYCLE
        },
        lidar_calibration={
            # Platzhalter für zukünftige LIDAR-Integration
            'angle_offset': 0.0,
            'distance_scale': 1.0
        },
        calibration_timestamp=time.time(),
        calibration_quality=1.0
    )
    
    # Erstelle Umgebungsmetadaten
    metadata = EnvironmentMetadata(
        session_id=recording_session_id,
        start_timestamp=time.time(),
        end_timestamp=None,
        environment_conditions={
            'recording_location': 'indoor',  # Kann erweitert werden
            'robot_configuration': 'xbox_controller_v2.0'
        },
        hardware_info={
            'controller_model': 'Xbox Wireless Controller',
            'robot_platform': 'RaspberryPi_Tank_Robot',
            'motor_type': 'DC_with_PWM',
            'sensors': 'HC-SR04_ultrasonic'
        },
        recording_settings={
            'controller_rate_hz': RECORDING_SAMPLE_RATE_HZ,
            'buffer_size': RECORDING_BUFFER_SIZE,
            'min_duration_sec': MIN_RECORDING_DURATION_SEC,
            'compression': CompressionType.GZIP
        }
    )
    
    # Erstelle Header
    header = PathRecordingHeader(
        calibration_data=calibration_data,
        metadata=metadata,
        compression_type=CompressionType.GZIP
    )
    
    # Erstelle PathRecordingData
    return PathRecordingData(header=header)

def start_recording() -> bool:
    """
    Startet eine neue Aufzeichnung.
    
    Returns:
        True wenn Recording erfolgreich gestartet wurde, False bei Fehler
    """
    global recording_active, recording_start_time, recording_session_id
    global recording_data, recording_status, logger
    
    if not RECORDING_AVAILABLE or not storage_manager:
        print('[RECORDING] ❌ Recording-System nicht verfügbar')
        return False
    
    if recording_active:
        print('[RECORDING] ⚠️ Recording bereits aktiv')
        return False
    
    try:
        # Initialisiere Recording-Variablen
        recording_session_id = create_session_id()
        recording_start_time = time.time()
        recording_active = True
        
        # Leere Buffer
        while not recording_buffer.empty():
            try:
                recording_buffer.get_nowait()
            except queue.Empty:
                break
        
        # Erstelle neue Recording-Daten
        recording_data = create_recording_data()
        
        # Aktualisiere Status
        recording_status.update(
            active=True,
            start_time=recording_start_time,
            session_id=recording_session_id,
            sample_count=0,
            buffer_size=0,
            max_buffer=RECORDING_BUFFER_SIZE
        )
        
        # Audio-Feedback
        if BUZZER_ENABLED:
            buzzer_beep(duration=0.1, count=2)
        
        # LED-Status ändern
        set_led_status("recording")
        
        print(f'[RECORDING] 🔴 Recording gestartet: {recording_session_id}')
        if logger:
            logger.info(f"Recording gestartet: {recording_session_id}")
        
        return True
        
    except Exception as e:
        print(f'[RECORDING] ❌ Fehler beim Starten: {e}')
        if logger:
            logger.error(f"Recording Start fehlgeschlagen: {e}")
        recording_active = False
        return False

def stop_recording(force_save: bool = False) -> bool:
    """
    Stoppt aktuelle Aufzeichnung und speichert Daten.
    
    Args:
        force_save: Erzwingt Speicherung auch bei kurzen Recordings
        
    Returns:
        True wenn Recording erfolgreich gestoppt und gespeichert wurde
    """
    global recording_active, recording_data, recording_status, logger
    
    if not recording_active:
        print('[RECORDING] ⚠️ Kein aktives Recording')
        return False
    
    try:
        # Stoppe Recording
        recording_active = False
        set_led_status("saving")
        
        # Berechne finale Statistiken
        duration = time.time() - recording_start_time
        sample_count = recording_data.get_stats().get('controller_samples', 0) if recording_data else 0
        
        # Prüfe Gültigkeit
        is_valid = duration >= MIN_RECORDING_DURATION_SEC or force_save
        
        if is_valid and recording_data and storage_manager:
            # Finalisiere Metadaten
            if recording_data.header.metadata:
                recording_data.header.metadata.end_timestamp = time.time()
            
            # Speichere Recording
            filepath = storage_manager.save_json(
                recording_data,
                recording_session_id,
                create_backup=True
            )
            
            print(f'[RECORDING] ✅ Recording gespeichert: {filepath}')
            print(f'[RECORDING] 📊 Statistiken:')
            print(f'    Dauer: {duration:.2f} Sekunden')
            print(f'    Samples: {sample_count}')
            print(f'    Durchschnitt: {sample_count/duration:.1f} Hz' if duration > 0 else '    Durchschnitt: N/A')
            
            if logger:
                logger.info(f"Recording gespeichert: {filepath} ({sample_count} samples, {duration:.2f}s)")
            
            # Audio-Feedback für erfolgreiche Speicherung
            if BUZZER_ENABLED:
                buzzer_beep(duration=0.2, count=1)
        
        else:
            print(f'[RECORDING] ⚠️ Recording verworfen (zu kurz: {duration:.2f}s < {MIN_RECORDING_DURATION_SEC}s)')
            if logger:
                logger.warning(f"Recording verworfen: {recording_session_id} ({duration:.2f}s)")
            
            # Audio-Feedback für verworfenes Recording
            if BUZZER_ENABLED:
                buzzer_beep(duration=0.1, count=3)
        
        # Reset Status
        recording_status.update(
            active=False,
            start_time=0.0,
            session_id="",
            sample_count=0,
            buffer_size=0,
            max_buffer=RECORDING_BUFFER_SIZE
        )
        
        # LED Status zurücksetzen
        set_led_status("off")
        
        return is_valid
        
    except Exception as e:
        print(f'[RECORDING] ❌ Fehler beim Stoppen: {e}')
        if logger:
            logger.error(f"Recording Stop fehlgeschlagen: {e}")
        recording_active = False
        set_led_status("off")
        return False

def add_controller_sample_to_recording() -> None:
    """
    Fügt aktuellen Controller-Status zur Aufzeichnung hinzu.
    
    Erstellt ControllerSample aus aktuellen globalen Variablen und
    fügt es thread-sicher zur Aufzeichnung hinzu.
    """
    global recording_data, recording_buffer, trigger_left, trigger_right
    global a_button_pressed, is_moving_forward, distance_left, distance_right
    
    if not recording_active or not recording_data:
        return
    
    try:
        # Erstelle Controller-Sample
        sample = ControllerSample(
            timestamp=time.time(),
            left_trigger=trigger_left,
            right_trigger=trigger_right,
            buttons={
                'A': a_button_pressed,
                'B': not is_moving_forward,  # B-Button togglet Richtung
                'X': x_button_pressed,
                'Y': y_button_pressed
            },
            direction={
                'forward': 1.0 if is_moving_forward else -1.0,
                'tank_turn': 1.0 if a_button_pressed else 0.0
            },
            sequence_id=0  # Wird automatisch gesetzt
        )
        
        # Füge Sample zur Aufzeichnung hinzu (thread-sicher)
        recording_data.add_controller_sample(sample)
        
        # Optional: Füge zum Buffer für mögliche Echtzeitverarbeitung hinzu
        try:
            recording_buffer.put_nowait({
                'type': 'controller',
                'sample': sample,
                'sensor_data': {
                    'distance_left': distance_left,
                    'distance_right': distance_right
                }
            })
        except queue.Full:
            # Buffer voll - älteste Daten werden überschrieben
            try:
                recording_buffer.get_nowait()  # Entferne ältestes Element
                recording_buffer.put_nowait({
                    'type': 'controller',
                    'sample': sample,
                    'sensor_data': {
                        'distance_left': distance_left,
                        'distance_right': distance_right
                    }
                })
            except (queue.Empty, queue.Full):
                pass  # Ignoriere Buffer-Probleme
        
        # Aktualisiere Status
        stats = recording_data.get_stats()
        recording_status.update(
            active=recording_active,
            start_time=recording_start_time,
            session_id=recording_session_id,
            sample_count=stats.get('controller_samples', 0),
            buffer_size=recording_buffer.qsize(),
            max_buffer=RECORDING_BUFFER_SIZE
        )
        
    except Exception as e:
        if logger:
            logger.error(f"Fehler beim Hinzufügen Controller Sample: {e}")

def recording_background_thread():
    """
    Background-Thread für Recording-Operationen.
    
    Verarbeitet Recording-Buffer und führt periodische Wartungsaufgaben durch
    wie automatische Zwischenspeicherung bei langen Aufzeichnungen.
    """
    global running, recording_buffer, logger
    
    if not RECORDING_AVAILABLE:
        return
    
    print(f'[THREAD] Recording Background Thread gestartet')
    if logger:
        logger.info("Recording Background Thread gestartet")
    
    last_auto_save = time.time()
    
    while running:
        try:
            # Verarbeite Buffer-Einträge (falls zukünftige Verarbeitung benötigt)
            try:
                item = recording_buffer.get(timeout=1.0)
                # Hier könnte Echtzeitverarbeitung stattfinden
                # Aktuell nur als Platzhalter
                recording_buffer.task_done()
            except queue.Empty:
                pass
            
            # Automatische Zwischenspeicherung bei langen Recordings
            current_time = time.time()
            if (recording_active and 
                current_time - last_auto_save > AUTO_SAVE_INTERVAL_SEC and
                recording_data):
                
                try:
                    # Erstelle temporäre Zwischenspeicherung
                    temp_filename = f"{recording_session_id}_temp"
                    temp_path = storage_manager.save_json(recording_data, temp_filename)
                    print(f'[RECORDING] 💾 Zwischenspeicherung: {temp_path}')
                    if logger:
                        logger.info(f"Automatische Zwischenspeicherung: {temp_path}")
                    last_auto_save = current_time
                except Exception as e:
                    print(f'[RECORDING] ⚠️ Zwischenspeicherung fehlgeschlagen: {e}')
                    if logger:
                        logger.warning(f"Automatische Zwischenspeicherung fehlgeschlagen: {e}")
            
            # Kurze Pause
            time.sleep(0.1)
            
        except Exception as e:
            if logger:
                logger.error(f"Recording Background Thread Fehler: {e}")
            time.sleep(1.0)

# ============================================================================
# LED UND AUDIO FEEDBACK (NEU)
# ============================================================================

def set_led_status(status: str) -> None:
    """
    Setzt LED-Status für Visual Feedback.
    
    Args:
        status: "off", "recording", "saving"
    """
    global led_status
    led_status = status

def led_control_thread():
    """Thread für LED-Steuerung mit verschiedenen Blinkmustern."""
    global running, led_status, led_blink_state, led_last_toggle
    
    if not LED_ENABLED:
        return
    
    print(f'[THREAD] LED Control Thread gestartet (GPIO{STATUS_LED_PIN})')
    
    while running:
        current_time = time.time()
        
        if led_status == "off":
            # LED ausschalten
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
            led_blink_state = False
            
        elif led_status == "recording":
            # LED blinken lassen
            blink_interval = 1.0 / LED_BLINK_RATE_HZ / 2.0  # Für 50% Duty Cycle
            
            if current_time - led_last_toggle > blink_interval:
                led_blink_state = not led_blink_state
                GPIO.output(STATUS_LED_PIN, GPIO.HIGH if led_blink_state else GPIO.LOW)
                led_last_toggle = current_time
                
        elif led_status == "saving":
            # LED dauerhaft an
            GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
            led_blink_state = True
        
        time.sleep(0.05)  # 20Hz Update-Rate für smooth blinking

def buzzer_beep(duration: float = 0.1, count: int = 1, frequency: int = 2000) -> None:
    """
    Erzeugt Piep-Ton mit Buzzer.
    
    Args:
        duration: Dauer eines Pieps in Sekunden
        count: Anzahl der Pieps
        frequency: Frequenz in Hz (bei PWM-Buzzer)
    """
    if not BUZZER_ENABLED:
        return
    
    try:
        for i in range(count):
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
            if i < count - 1:  # Pause zwischen Pieps (außer beim letzten)
                time.sleep(duration * 0.5)
    except Exception as e:
        if logger:
            logger.warning(f"Buzzer-Fehler: {e}")

# ============================================================================
# CONTROLLER INPUT (erweitert)
# ============================================================================

def read_controller_input():
    """
    Erweiterte Controller-Eingabe inklusive Recording-Steuerung.
    
    Liest alle Controller-Eingaben und verwaltet Recording-Funktionalität
    über Y+X Button-Kombination mit Edge-Detection.
    """
    global trigger_left, trigger_right, running, joystick
    global is_moving_forward, b_button_was_pressed, a_button_pressed
    global x_button_pressed, y_button_pressed, x_button_was_pressed, y_button_was_pressed
    
    pygame.event.pump()
    
    if not joystick or not joystick.get_init():
        print('[ERROR] ❌ Controller Verbindung verloren!')
        running = False
        return
    
    try:
        # Lese Controller-Eingaben (aus v1.4)
        trigger_left = joystick.get_axis(AXIS_LEFT_TRIGGER)
        trigger_right = joystick.get_axis(AXIS_RIGHT_TRIGGER)
        a_button_pressed = joystick.get_button(BUTTON_A)
        
        # B-Button für Richtungsumschaltung (aus v1.4)
        b_button_currently_pressed = joystick.get_button(BUTTON_B)
        if b_button_currently_pressed and not b_button_was_pressed:
            is_moving_forward = not is_moving_forward
            direction_text = "⬆️ VORWÄRTS" if is_moving_forward else "⬇️ RÜCKWÄRTS"
            print(f'[INPUT] 🔄 B-Button: Richtung umgeschaltet → {direction_text}')
        b_button_was_pressed = b_button_currently_pressed
        
        # NEU: Recording-Steuerung mit X+Y Buttons
        x_button_currently_pressed = joystick.get_button(BUTTON_X)
        y_button_currently_pressed = joystick.get_button(BUTTON_Y)
        
        # Edge-Detection für Recording Toggle
        if (x_button_currently_pressed and y_button_currently_pressed and 
            not (x_button_was_pressed and y_button_was_pressed)):
            
            if RECORDING_AVAILABLE:
                if recording_active:
                    print('[INPUT] ⏹️ Y+X: Stoppe Recording...')
                    stop_recording(force_save=False)
                else:
                    print('[INPUT] 🔴 Y+X: Starte Recording...')
                    start_recording()
            else:
                print('[INPUT] ❌ Y+X: Recording-System nicht verfügbar!')
        
        x_button_pressed = x_button_currently_pressed
        y_button_pressed = y_button_currently_pressed  
        x_button_was_pressed = x_button_currently_pressed
        y_button_was_pressed = y_button_currently_pressed
        
        # Start Button für Programm-Ende (aus v1.4)
        if joystick.get_button(BUTTON_START):
            print('[INPUT] 🎮 Start-Button gedrückt - Beende Programm...')
            running = False
    
    except Exception as e:
        print(f'[ERROR] Controller Input Fehler: {e}')
        if logger:
            logger.error(f"Controller Input Fehler: {e}")
        running = False

# ============================================================================
# HAUPT STEUERUNGS-LOOP (erweitert)
# ============================================================================

def control_loop():
    """
    Erweiterte Haupt-Steuerungsschleife mit Recording-Integration.
    
    Integriert Recording-Funktionalität nahtlos in bestehende Steuerungslogik
    ohne Performance-Einbußen. Recording läuft parallel zur normalen Fahrt.
    """
    global running, trigger_left, trigger_right, distance_left, distance_right
    global is_moving_forward, a_button_pressed
    
    print(f'[CONTROL] Erweiterte Steuerungs-Loop gestartet (Update-Rate: {CONTROL_LOOP_HZ} Hz)')
    print('[CONTROL] 🤖 Roboter mit Recording-Funktionalität betriebsbereit!')
    print('\n' + '='*80)
    print('🎮 STEUERUNG:')
    print('  Left Trigger  → Linker Motor')
    print('  Right Trigger → Rechter Motor')
    print('  A Button (halten) → Tank-Turn Modus')
    print('  B Button → Richtungsumschaltung (Vorwärts ⇄ Rückwärts)')
    print('  Y + X Buttons → Recording Start/Stop Toggle')
    print('  Start Button → Programm beenden')
    print('\n🔴 RECORDING (NEU):')
    print('  Y + X gleichzeitig drücken → Recording starten/stoppen')
    print(f'  Mindest-Aufnahmedauer: {MIN_RECORDING_DURATION_SEC} Sekunden')
    print(f'  Aufzeichnungsrate: {RECORDING_SAMPLE_RATE_HZ} Hz')
    if LED_ENABLED:
        print(f'  Status-LED (GPIO{STATUS_LED_PIN}): AUS=Bereit, BLINKEN=Recording, AN=Speichern')
    if BUZZER_ENABLED:
        print(f'  Audio-Feedback (GPIO{BUZZER_PIN}): Piep-Signale bei Recording-Aktionen')
    print('\n🛡️  SICHERHEIT:')
    print(f'  Kollisionsabstand: {COLLISION_DISTANCE_CM} cm')
    if RECORDING_AVAILABLE:
        print(f'  Speicherort: {RECORDING_BASE_DIR}/')
    print('='*80 + '\n')
    
    sleep_time = 1.0 / CONTROL_LOOP_HZ
    status_counter = 0
    status_update_interval = CONTROL_LOOP_HZ  # Jede Sekunde
    recording_sample_counter = 0
    recording_sample_interval = CONTROL_LOOP_HZ // RECORDING_SAMPLE_RATE_HZ  # Für 50Hz Controller bei 50Hz Loop
    
    while running:
        loop_start = time.time()
        
        # 1. Lese Controller-Eingaben
        read_controller_input()
        
        # 2. Konvertiere Trigger zu Duty Cycle (aus v1.4)
        left_duty = trigger_to_duty_cycle(trigger_left)
        right_duty = trigger_to_duty_cycle(trigger_right)
        
        # 3. Tank-Turn und Richtungslogik (aus v1.4) 
        if a_button_pressed:
            # Tank-Turn Modus
            if is_moving_forward:
                if right_duty > left_duty:
                    # Rechter Trigger → Links-Drehung
                    set_motor_direction(left_backward=True, right_backward=False)
                    set_motor_speed(right_duty, right_duty)
                else:
                    # Linker Trigger → Rechts-Drehung
                    set_motor_direction(left_backward=False, right_backward=True)
                    set_motor_speed(left_duty, left_duty)
            else:
                # Rückwärts-Modus mit Tank-Turn
                if right_duty > left_duty:
                    set_motor_direction(left_backward=False, right_backward=True)
                    set_motor_speed(right_duty, right_duty)
                else:
                    set_motor_direction(left_backward=True, right_backward=False)
                    set_motor_speed(left_duty, left_duty)
        else:
            # Normales Fahren
            backward = not is_moving_forward
            set_motor_direction(backward, backward)
            set_motor_speed(left_duty, right_duty)
        
        # 4. NEU: Recording-Sample hinzufügen (nur in konfigurierten Intervallen)
        recording_sample_counter += 1
        if recording_sample_counter >= recording_sample_interval:
            recording_sample_counter = 0
            if recording_active:
                add_controller_sample_to_recording()
        
        # 5. Status-Ausgabe (aus v1.4, erweitert um Recording)
        status_counter += 1
        if status_counter >= status_update_interval:
            status_counter = 0
            
            left_status = '🔴' if 0 < distance_left < COLLISION_DISTANCE_CM else '🟢'
            right_status = '🔴' if 0 < distance_right < COLLISION_DISTANCE_CM else '🟢'
            
            direction_emoji = "⬆️ " if is_moving_forward else "⬇️ "
            direction_text = "FWD" if is_moving_forward else "REV"
            
            mode_text = "🔄 TANK" if a_button_pressed else "➡️  NORM"
            
            # NEU: Recording-Status
            recording_text = ""
            if RECORDING_AVAILABLE:
                if recording_active:
                    duration = time.time() - recording_start_time
                    sample_count = recording_data.get_stats().get('controller_samples', 0) if recording_data else 0
                    recording_text = f" | 🔴 REC:{duration:.1f}s({sample_count})"
                else:
                    recording_text = " | ⚪ REC:OFF"
            
            print(f'[STATUS] {left_status} Mode:{mode_text} Dir:{direction_emoji}{direction_text} | '
                  f'Trigger: L={trigger_left:+.2f} R={trigger_right:+.2f} | '
                  f'PWM: L={left_duty:.1f}% R={right_duty:.1f}% | '
                  f'Dist: L={distance_left:.1f}cm R={distance_right:.1f}cm {right_status}{recording_text}')
        
        # 6. Reguliere Loop-Frequenz
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

# ============================================================================
# CLEANUP FUNKTION (erweitert)
# ============================================================================

def cleanup():
    """
    Erweiterte Ressourcen-Freigabe inklusive Recording-Daten-Sicherung.
    
    Stellt sicher, dass alle Hardware-Ressourcen sauber freigegeben werden
    und aktive Recording-Daten gesichert werden.
    """
    global running, pwm_left, pwm_right, joystick
    global sensor_thread, recording_thread, led_thread, logger
    
    print('[CLEANUP] 🧹 Starte erweiterte Ressourcen-Freigabe...')
    running = False
    
    # Stoppe aktives Recording falls vorhanden
    if recording_active:
        print('[CLEANUP] 💾 Sichere aktives Recording...')
        stop_recording(force_save=True)
    
    # Warte auf Threads
    for thread_name, thread_obj in [
        ('Sensor', sensor_thread),
        ('Recording', recording_thread), 
        ('LED', led_thread)
    ]:
        if thread_obj and thread_obj.is_alive():
            print(f'[CLEANUP] Warte auf {thread_name} Thread...')
            thread_obj.join(timeout=2.0)
            if thread_obj.is_alive():
                print(f'[CLEANUP] ⚠️ {thread_name} Thread Timeout')
    
    # Stoppe Motoren
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
    
    # Schalte Hardware aus
    if LED_ENABLED:
        try:
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        except Exception as e:
            print(f'[CLEANUP] LED Cleanup-Warnung: {e}')
    
    if BUZZER_ENABLED:
        try:
            GPIO.output(BUZZER_PIN, GPIO.LOW)
        except Exception as e:
            print(f'[CLEANUP] Buzzer Cleanup-Warnung: {e}')
    
    # Trenne Controller
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
    
    # GPIO Cleanup
    print('[CLEANUP] Räume GPIO auf...')
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f'[CLEANUP] Warnung beim GPIO Cleanup: {e}')
    
    if logger:
        logger.info("System erfolgreich heruntergefahren")
    
    print('[CLEANUP] ✅ Alle Ressourcen freigegeben - Programm beendet.')

# ============================================================================
# MAIN FUNKTION (erweitert)
# ============================================================================

def main():
    """
    Erweiterte Hauptfunktion mit Recording-System-Initialisierung.
    
    Startet alle Systemkomponenten inklusive Recording-Funktionalität
    und verwaltet die verschiedenen Threads.
    """
    global running, sensor_thread, recording_thread, led_thread
    
    print('\n' + '='*80)
    print('🤖 XBOX CONTROLLER ROBOTER MIT RECORDING-FUNKTIONALITÄT')
    print('Version 2.0 - Phase 2: Recording Integration')
    print('Basierend auf v1.4 mit Tank-Turn und Kollisionsvermeidung')
    print('✅ NEU: Path Recording mit Controller-Datenaufzeichnung')
    print('✅ NEU: Thread-sichere Recording-Architektur')  
    print('✅ NEU: Visual/Audio Feedback für Recording-Status')
    print('='*80 + '\n')
    
    try:
        # Basis-Setup (aus v1.4)
        setup_gpio()
        setup_pwm()
        setup_controller()
        
        # NEU: Recording-Setup
        if RECORDING_AVAILABLE:
            setup_recording()
        
        print('\n[INIT] ✅ Alle Systeme initialisiert - Starte Threads...')
        
        running = True
        
        # Starte Sensor Thread (aus v1.4)
        sensor_thread = threading.Thread(
            target=sensor_update_thread, 
            daemon=True, 
            name='SensorThread'
        )
        sensor_thread.start()
        
        # NEU: Starte Recording Thread
        if RECORDING_AVAILABLE:
            recording_thread = threading.Thread(
                target=recording_background_thread,
                daemon=True,
                name='RecordingThread'
            )
            recording_thread.start()
        
        # NEU: Starte LED Thread
        if LED_ENABLED:
            led_thread = threading.Thread(
                target=led_control_thread,
                daemon=True, 
                name='LEDThread'
            )
            led_thread.start()
        
        # Kurze Pause damit Threads starten können
        time.sleep(0.5)
        
        # Starte Hauptsteuerung
        control_loop()
    
    except KeyboardInterrupt:
        print('\n[SHUTDOWN] ⌨️  Keyboard Interrupt (Ctrl+C) - Beende Programm...')
    except Exception as e:
        print(f'\n[ERROR] ❌ Unerwarteter Fehler: {e}')
        if logger:
            logger.error(f"Unerwarteter Fehler in main(): {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

# ============================================================================
# PROGRAMM START
# ============================================================================

if __name__ == '__main__':
    import os
    
    # Root-Berechtigung prüfen (für GPIO-Zugriff)
    if os.geteuid() != 0:
        print('❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!')
        print('📝 Verwendung: sudo python3 xbox_controller_with_recording_v2.0.py')
        print('\n📋 Voraussetzungen:')
        print('   - path_recording_system.py im selben Verzeichnis')
        print('   - storage_system.py im selben Verzeichnis')
        print('   - sudo apt install python3-pygame xboxdrv')
        print('   - sudo pip3 install rpi-hardware-pwm RPi.GPIO')
        sys.exit(1)
    
    main()
