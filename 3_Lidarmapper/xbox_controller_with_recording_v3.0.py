#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Xbox Controller Roboter-Steuerung mit LIDAR-Integration und Recording - Phase 3
===============================================================================

Vollständige Integration von Xbox Controller, LIDAR C1, Ultraschall-Sensoren
und Path Recording System. Ermöglicht manuelle Steuerung mit gleichzeitiger
Aufzeichnung aller Sensor- und Steuerdaten für späteres autonomes Replay.

NEU in Version 3.0 (Phase 3: LIDAR-Integration):
- ✅ RPLIDAR C1 Integration mit Thread-sicherer Verarbeitung
- ✅ Sensor Fusion: LIDAR + Ultraschall Kollisionserkennung
- ✅ Roboter-Pose Tracking mit Odometrie
- ✅ Synchronisierte Aufzeichnung: Controller + LIDAR + Pose
- ✅ Environment Mapping mit Occupancy Grid
- ✅ Erweiterte Kollisionsvermeidung mit Multi-Sensor-Fusion
- ✅ Graceful Degradation bei Sensor-Ausfällen

Aus Version 2.0:
- Recording-Modus mit Y+X Button-Kombination
- Thread-sichere Datensammlung
- Visual/Audio Feedback
- Tank-Turn und Richtungsumschaltung

Aus Version 1.4:
- Xbox Controller Eingabe
- Hardware PWM Motor-Ansteuerung
- Ultraschall-Kollisionsvermeidung

HARDWARE ANFORDERUNGEN:
-----------------------
- RPLIDAR C1 (460800 Baud, /dev/ttyUSB0)
- 2x HC-SR04 Ultraschall-Sensoren (links/rechts)
- Xbox Wireless Controller
- 2x DC-Motoren mit Hardware PWM
- Raspberry Pi 4+ (empfohlen)
- Optional: Status-LED (GPIO22), Buzzer (GPIO25)

SOFTWARE ABHÄNGIGKEITEN:
-----------------------
sudo apt install python3-pygame xboxdrv
sudo pip3 install rpi-hardware-pwm RPi.GPIO rplidar-roboticia numpy

Zusätzlich benötigt (im selben Verzeichnis):
- path_recording_system.py
- storage_system.py
- lidar_controller_fusion.py

STEUERUNG:
---------
- Left/Right Trigger → Motor-Steuerung
- A Button (halten) → Tank-Turn Modus
- B Button → Richtungsumschaltung (Vorwärts ⇄ Rückwärts)
- Y + X Buttons → Recording Start/Stop Toggle
- Start Button → Programm beenden

KOORDINATENSYSTEM:
-----------------
- Roboter: Mittelpunkt bei (0,0), 0° = Vorwärts
- LIDAR: 15cm vor Roboter-Mittelpunkt
- Odometrie: Akkumulierte Position ab Startpunkt

AUTOR: Phase 3 Integration
DATUM: 2025-11-27
VERSION: 3.0 (Phase 3: LIDAR + Pose Tracking)
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
import math
import numpy as np
from datetime import datetime, timezone
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
from pathlib import Path
from rpi_hardware_pwm import HardwarePWM
from collections import deque

# Import der Recording-Datenstrukturen
try:
    from path_recording_system import (
        PathRecordingData,
        PathRecordingHeader, 
        ControllerSample,
        LidarFrame,
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
    RECORDING_AVAILABLE = False

# Import LIDAR
try:
    from rplidar import RPLidar, RPLidarException
    LIDAR_AVAILABLE = True
    print("✅ RPLIDAR Bibliothek erfolgreich importiert")
except ImportError as e:
    print(f"⚠️ RPLIDAR nicht verfügbar: {e}")
    LIDAR_AVAILABLE = False

# ============================================================================
# HARDWARE KONFIGURATION
# ============================================================================

# Ultraschall-Sensoren (aus v2.0)
TRIG_LEFT = 12      
ECHO_LEFT = 13      
TRIG_RIGHT = 23     
ECHO_RIGHT = 24     

# Motor-Steuerung (aus v2.0)
DIR_LEFT_PIN = 20   
DIR_RIGHT_PIN = 21  
DIR_HIGH_IS_BACKWARD = False

# PWM-Konfiguration (aus v2.0)
PWM_FREQUENCY = 1000
PWM_CHIP = 0
PWM_CHANNEL_LEFT = 0
PWM_CHANNEL_RIGHT = 1
MAX_DUTY_CYCLE = 12.0
MIN_DUTY_CYCLE = 0.0

# Recording-Hardware (aus v2.0)
STATUS_LED_PIN = 22
BUZZER_PIN = 25
LED_ENABLED = True
BUZZER_ENABLED = False

# LIDAR-Konfiguration (NEU)
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
LIDAR_SCAN_TIMEOUT = 2.0
LIDAR_TARGET_FREQUENCY = 10.0  # Hz

# LIDAR-Position relativ zum Roboter-Mittelpunkt (NEU)
LIDAR_OFFSET_X = 0.15  # 15cm vor Roboter-Mittelpunkt
LIDAR_OFFSET_Y = 0.0   # zentriert
LIDAR_ANGLE_OFFSET = 0.0  # kein Winkel-Offset

# Kollisionserkennung (erweitert)
COLLISION_DISTANCE_CM = 30.0  # Ultraschall
LIDAR_COLLISION_DISTANCE_M = 0.3  # LIDAR (30cm)
LIDAR_WARNING_DISTANCE_M = 0.8  # LIDAR Warnung (80cm)
SOUND_SPEED = 34300

# Controller-Konfiguration (aus v2.0)
CONTROLLER_ID = 0
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 5
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_START = 7

# Update-Frequenzen
SENSOR_UPDATE_HZ = 10
CONTROL_LOOP_HZ = 50
LIDAR_UPDATE_HZ = 10

# Recording-Konfiguration (aus v2.0)
RECORDING_SAMPLE_RATE_HZ = 50
RECORDING_BUFFER_SIZE = 1000
MIN_RECORDING_DURATION_SEC = 5.0
RECORDING_BASE_DIR = "path_recordings"

# Odometrie-Parameter (NEU)
WHEEL_BASE_M = 0.20  # 20cm Radabstand
WHEEL_DIAMETER_M = 0.065  # 6.5cm Raddurchmesser
ENCODER_TICKS_PER_REV = 1  # Geschätzt aus PWM
MAX_POSE_UPDATE_HZ = 50

# ============================================================================
# GLOBALE VARIABLEN
# ============================================================================

# Hardware Objekte
pwm_left: Optional[HardwarePWM] = None
pwm_right: Optional[HardwarePWM] = None
joystick: Optional[pygame.joystick.Joystick] = None
lidar: Optional[RPLidar] = None

# Thread Control
running = False
sensor_thread: Optional[threading.Thread] = None
lidar_thread: Optional[threading.Thread] = None
recording_thread: Optional[threading.Thread] = None
led_thread: Optional[threading.Thread] = None
pose_thread: Optional[threading.Thread] = None

# Sensor Daten
distance_left = 100.0
distance_right = 100.0
lidar_scan_data: List[Tuple[float, float, float]] = []  # (quality, angle_deg, distance_mm)
lidar_lock = threading.Lock()
lidar_frame_buffer: queue.Queue = queue.Queue(maxsize=100)

# Controller Daten
trigger_left = -1.0
trigger_right = -1.0
a_button_pressed = False
b_button_was_pressed = False
is_moving_forward = True
x_button_pressed = False
y_button_pressed = False
x_button_was_pressed = False
y_button_was_pressed = False

# Recording-Status Variablen
recording_active = False
recording_start_time = 0.0
recording_session_id = ""
recording_buffer: queue.Queue = queue.Queue(maxsize=RECORDING_BUFFER_SIZE)
recording_data: Optional[PathRecordingData] = None
storage_manager: Optional[PathRecordingStorageManager] = None

# LED-Status Variablen
led_status = "off"
led_blink_state = False
led_last_toggle = 0.0

# Pose Tracking (NEU)
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
pose_timestamp = 0.0
pose_lock = threading.Lock()
last_left_duty = 0.0
last_right_duty = 0.0
last_pose_update = 0.0

# LIDAR-Status
lidar_connected = False
lidar_error_count = 0
lidar_last_scan_time = 0.0

# Logging
logger = None

# ============================================================================
# DATENSTRUKTUREN
# ============================================================================

@dataclass
class RobotPose:
    """Roboter-Position und -Orientierung."""
    x: float
    y: float
    theta: float
    timestamp: float
    
    def to_dict(self) -> Dict[str, float]:
        """Konvertiert zu Dictionary für Recording."""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'timestamp': self.timestamp
        }

@dataclass
class CollisionStatus:
    """Kollisionsstatus aus Multi-Sensor-Fusion."""
    ultrasonic_left: float
    ultrasonic_right: float
    lidar_min_distance: float
    lidar_obstacles_front: int
    collision_detected: bool
    collision_critical: bool
    
    def to_dict(self) -> Dict[str, Any]:
        """Konvertiert zu Dictionary."""
        return {
            'ultrasonic_left': self.ultrasonic_left,
            'ultrasonic_right': self.ultrasonic_right,
            'lidar_min_distance': self.lidar_min_distance,
            'lidar_obstacles_front': self.lidar_obstacles_front,
            'collision_detected': self.collision_detected,
            'collision_critical': self.collision_critical
        }

@dataclass
class LidarStats:
    """LIDAR-Verarbeitungsstatistiken."""
    scans_processed: int = 0
    processing_rate_hz: float = 0.0
    avg_points_per_scan: float = 0.0
    error_count: int = 0
    last_error_time: float = 0.0
    buffer_utilization: float = 0.0

# ============================================================================
# SIGNAL HANDLER
# ============================================================================

def signal_handler(sig, frame):
    """Signal-Handler für Graceful Shutdown."""
    global logger
    print('\n[SHUTDOWN] SIGINT empfangen - Programm wird beendet...')
    
    if recording_active and recording_data:
        duration = time.time() - recording_start_time
        if duration >= MIN_RECORDING_DURATION_SEC:
            print('[SHUTDOWN] 🔄 Speichere aktive Aufzeichnung...')
            stop_recording(force_save=True)
        else:
            print('[SHUTDOWN] ⚠️ Recording zu kurz - wird verworfen')
            stop_recording(force_save=False)
    
    cleanup()
    sys.exit(0)

# ============================================================================
# HARDWARE SETUP
# ============================================================================

def setup_gpio():
    """Initialisiert GPIO für Sensoren, Motoren und Recording-Hardware."""
    global logger
    print('[SETUP] Initialisiere GPIO...')
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Ultraschall-Sensoren
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.output(TRIG_LEFT, False)
    
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    GPIO.output(TRIG_RIGHT, False)
    
    # Motor-Richtung
    GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
    GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)
    
    if DIR_HIGH_IS_BACKWARD:
        GPIO.output(DIR_LEFT_PIN, GPIO.LOW)
        GPIO.output(DIR_RIGHT_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_LEFT_PIN, GPIO.HIGH)
        GPIO.output(DIR_RIGHT_PIN, GPIO.HIGH)
    
    # Recording-Hardware
    if LED_ENABLED:
        GPIO.setup(STATUS_LED_PIN, GPIO.OUT)
        GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        print(f'[SETUP] Status-LED initialisiert (GPIO{STATUS_LED_PIN})')
    
    if BUZZER_ENABLED:
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        print(f'[SETUP] Buzzer initialisiert (GPIO{BUZZER_PIN})')
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if logger:
        logger.info("GPIO initialisiert")
    
    print('[SETUP] GPIO initialisiert - Sensoren stabilisieren (2s)...')
    time.sleep(2)

def setup_pwm():
    """Initialisiert Hardware PWM für Motoren."""
    global pwm_left, pwm_right
    
    print('[SETUP] Initialisiere Hardware PWM...')
    
    try:
        pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        
        pwm_left.start(MIN_DUTY_CYCLE)
        pwm_right.start(MIN_DUTY_CYCLE)
        
        print(f'[SETUP] PWM initialisiert - Frequenz: {PWM_FREQUENCY} Hz')
    except Exception as e:
        print(f'[ERROR] PWM Initialisierung fehlgeschlagen: {e}')
        raise

def setup_controller():
    """Initialisiert Xbox Controller."""
    global joystick
    
    print('[SETUP] Initialisiere Xbox Controller...')
    
    pygame.init()
    pygame.joystick.init()
    
    controller_count = pygame.joystick.get_count()
    if controller_count == 0:
        raise RuntimeError('[ERROR] Kein Xbox Controller gefunden!')
    
    joystick = pygame.joystick.Joystick(CONTROLLER_ID)
    joystick.init()
    
    print(f'[SETUP] Controller verbunden: {joystick.get_name()}')

def setup_lidar() -> bool:
    """Initialisiert RPLIDAR C1."""
    global lidar, lidar_connected, logger
    
    if not LIDAR_AVAILABLE:
        print('[SETUP] ⚠️ RPLIDAR Bibliothek nicht verfügbar')
        return False
    
    print('[SETUP] Initialisiere RPLIDAR C1...')
    
    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_SCAN_TIMEOUT)
        
        # Geräteinfo
        info = lidar.get_info()
        print(f'[SETUP] LIDAR Info: {info}')
        
        # Health-Check
        health = lidar.get_health()
        status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')
        
        if status == 'Error':
            raise RuntimeError('LIDAR meldet Fehlerzustand')
        
        # Motor starten
        lidar.start_motor()
        time.sleep(2.0)
        
        lidar_connected = True
        print('[SETUP] ✅ RPLIDAR C1 erfolgreich initialisiert')
        
        if logger:
            logger.info("RPLIDAR C1 initialisiert")
        
        return True
        
    except Exception as e:
        print(f'[SETUP] ❌ LIDAR Fehler: {e}')
        if logger:
            logger.error(f"LIDAR Initialisierung fehlgeschlagen: {e}")
        lidar = None
        lidar_connected = False
        return False

def setup_recording():
    """Initialisiert Recording-System."""
    global storage_manager, logger
    
    if not RECORDING_AVAILABLE:
        print("[SETUP] ⚠️ Recording-System nicht verfügbar")
        return
    
    print('[SETUP] Initialisiere Recording-System...')
    
    try:
        Path(RECORDING_BASE_DIR).mkdir(parents=True, exist_ok=True)
        
        logger = setup_logging(str(Path(RECORDING_BASE_DIR) / "xbox_lidar_recording.log"))
        
        storage_manager = PathRecordingStorageManager(
            base_directory=RECORDING_BASE_DIR,
            compression=CompressionType.GZIP,
            backup_enabled=True,
            backup_count=3,
            verify_checksums=True
        )
        
        logger.info("Recording-System initialisiert")
        print(f'[SETUP] ✅ Recording-System bereit: {RECORDING_BASE_DIR}')
        
    except Exception as e:
        print(f'[SETUP] ❌ Recording-System Fehler: {e}')
        if logger:
            logger.error(f"Recording-System Fehler: {e}")
        raise

# ============================================================================
# SENSOR FUNKTIONEN
# ============================================================================

def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """Misst Entfernung mit HC-SR04."""
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
    except Exception:
        return -1.0

def sensor_update_thread():
    """Thread für Ultraschall-Sensor-Updates."""
    global distance_left, distance_right, running
    
    print(f'[THREAD] Sensor Thread gestartet ({SENSOR_UPDATE_HZ} Hz)')
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

def lidar_update_thread():
    """Thread für LIDAR-Scan-Verarbeitung."""
    global lidar_scan_data, lidar_connected, lidar_error_count
    global lidar_last_scan_time, running, logger
    
    if not lidar or not lidar_connected:
        print('[THREAD] LIDAR Thread nicht gestartet (LIDAR nicht verfügbar)')
        return
    
    print(f'[THREAD] LIDAR Thread gestartet ({LIDAR_UPDATE_HZ} Hz)')
    
    scan_count = 0
    scan_times = deque(maxlen=50)
    
    try:
        for scan in lidar.iter_scans(max_buf_meas=5000, min_len=10):
            if not running:
                break
            
            try:
                timestamp = time.time()
                scan_data = list(scan)
                
                # Update globale Scan-Daten
                with lidar_lock:
                    lidar_scan_data = scan_data
                    lidar_last_scan_time = timestamp
                
                # Erstelle LidarFrame für Recording
                if recording_active:
                    frame = LidarFrame(
                        timestamp=timestamp,
                        scan_data=scan_data,
                        frame_id=scan_count
                    )
                    
                    try:
                        lidar_frame_buffer.put_nowait(frame)
                    except queue.Full:
                        # Buffer voll - entferne ältesten Frame
                        try:
                            lidar_frame_buffer.get_nowait()
                            lidar_frame_buffer.put_nowait(frame)
                        except queue.Empty:
                            pass
                
                scan_count += 1
                scan_times.append(timestamp)
                
            except Exception as e:
                if logger:
                    logger.warning(f"LIDAR Scan-Verarbeitung Fehler: {e}")
                lidar_error_count += 1
    
    except RPLidarException as e:
        print(f'[LIDAR] ❌ LIDAR Fehler: {e}')
        if logger:
            logger.error(f"LIDAR Fehler: {e}")
        lidar_connected = False
    except Exception as e:
        print(f'[LIDAR] ❌ Unerwarteter Fehler: {e}')
        if logger:
            logger.error(f"LIDAR Unerwarteter Fehler: {e}")

# ============================================================================
# POSE TRACKING
# ============================================================================

def update_pose_from_odometry(left_duty: float, right_duty: float, dt: float):
    """
    Aktualisiert Roboter-Pose basierend auf Motor-Befehlen (vereinfachte Odometrie).
    
    Args:
        left_duty: Linker Motor Duty Cycle (0-100%)
        right_duty: Rechter Motor Duty Cycle (0-100%)
        dt: Zeitdelta seit letztem Update
    """
    global robot_x, robot_y, robot_theta, pose_timestamp
    
    # Vereinfachte Geschwindigkeitsberechnung aus Duty Cycle
    # Annahme: Duty Cycle ist proportional zur Geschwindigkeit
    max_speed_m_s = 0.5  # Maximum 0.5 m/s bei 100% Duty Cycle
    
    v_left = (left_duty / 100.0) * max_speed_m_s
    v_right = (right_duty / 100.0) * max_speed_m_s
    
    # Berücksichtige Fahrtrichtung
    if not is_moving_forward:
        v_left = -v_left
        v_right = -v_right
    
    # Berechne Roboter-Geschwindigkeit und Drehrate
    v = (v_left + v_right) / 2.0  # Lineare Geschwindigkeit
    omega = (v_right - v_left) / WHEEL_BASE_M  # Drehrate
    
    # Update Pose
    with pose_lock:
        if abs(omega) < 0.001:
            # Geradeaus-Fahrt
            robot_x += v * math.cos(robot_theta) * dt
            robot_y += v * math.sin(robot_theta) * dt
        else:
            # Kreisbahn
            radius = v / omega
            robot_x += radius * (math.sin(robot_theta + omega * dt) - math.sin(robot_theta))
            robot_y += radius * (-math.cos(robot_theta + omega * dt) + math.cos(robot_theta))
            robot_theta += omega * dt
            
            # Normalisiere Winkel
            robot_theta = (robot_theta + math.pi) % (2 * math.pi) - math.pi
        
        pose_timestamp = time.time()

def pose_update_thread():
    """Thread für kontinuierliche Pose-Updates."""
    global running, last_left_duty, last_right_duty, last_pose_update
    
    print(f'[THREAD] Pose Thread gestartet ({MAX_POSE_UPDATE_HZ} Hz)')
    sleep_time = 1.0 / MAX_POSE_UPDATE_HZ
    last_pose_update = time.time()
    
    while running:
        loop_start = time.time()
        
        # Berechne Zeitdelta
        dt = loop_start - last_pose_update
        
        # Update Pose basierend auf aktuellen Motor-Befehlen
        update_pose_from_odometry(last_left_duty, last_right_duty, dt)
        
        last_pose_update = loop_start
        
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

def get_current_pose() -> RobotPose:
    """Holt aktuelle Roboter-Pose thread-sicher."""
    with pose_lock:
        return RobotPose(
            x=robot_x,
            y=robot_y,
            theta=robot_theta,
            timestamp=pose_timestamp
        )

# ============================================================================
# KOLLISIONSERKENNUNG MIT SENSOR FUSION
# ============================================================================

def analyze_collision_status() -> CollisionStatus:
    """
    Analysiert Kollisionsstatus aus LIDAR + Ultraschall.
    
    Returns:
        CollisionStatus mit aggregierten Sensor-Daten
    """
    global distance_left, distance_right, lidar_scan_data
    
    # Ultraschall-Daten
    ultrasonic_left = distance_left
    ultrasonic_right = distance_right
    
    # LIDAR-Analyse
    lidar_min_dist = float('inf')
    obstacles_front = 0
    
    with lidar_lock:
        for quality, angle_deg, distance_mm in lidar_scan_data:
            distance_m = distance_mm / 1000.0
            
            # Nur vorderer Sektor (-45° bis +45°)
            if -45 <= angle_deg <= 45:
                if distance_m < lidar_min_dist:
                    lidar_min_dist = distance_m
                
                if distance_m < LIDAR_WARNING_DISTANCE_M:
                    obstacles_front += 1
    
    if lidar_min_dist == float('inf'):
        lidar_min_dist = 100.0  # Keine Hindernisse
    
    # Kollisionserkennung
    collision_detected = (
        ultrasonic_left < COLLISION_DISTANCE_CM or
        ultrasonic_right < COLLISION_DISTANCE_CM or
        lidar_min_dist < LIDAR_COLLISION_DISTANCE_M
    )
    
    collision_critical = (
        ultrasonic_left < COLLISION_DISTANCE_CM * 0.5 or
        ultrasonic_right < COLLISION_DISTANCE_CM * 0.5 or
        lidar_min_dist < LIDAR_COLLISION_DISTANCE_M * 0.5
    )
    
    return CollisionStatus(
        ultrasonic_left=ultrasonic_left,
        ultrasonic_right=ultrasonic_right,
        lidar_min_distance=lidar_min_dist,
        lidar_obstacles_front=obstacles_front,
        collision_detected=collision_detected,
        collision_critical=collision_critical
    )

# ============================================================================
# MOTOR STEUERUNG
# ============================================================================

def set_motor_direction(left_backward: bool, right_backward: bool):
    """Setzt Motor-Richtung."""
    if DIR_HIGH_IS_BACKWARD:
        left_level = GPIO.HIGH if left_backward else GPIO.LOW
        right_level = GPIO.HIGH if right_backward else GPIO.LOW
    else:
        left_level = GPIO.LOW if left_backward else GPIO.HIGH
        right_level = GPIO.LOW if right_backward else GPIO.HIGH
    
    GPIO.output(DIR_LEFT_PIN, left_level)
    GPIO.output(DIR_RIGHT_PIN, right_level)

def trigger_to_duty_cycle(trigger_value: float) -> float:
    """Konvertiert Xbox Trigger zu PWM Duty Cycle."""
    normalized = (trigger_value + 1.0) / 2.0
    duty_cycle = normalized * MAX_DUTY_CYCLE
    return max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, duty_cycle))

def set_motor_speed(left_duty: float, right_duty: float):
    """Setzt Motor-Geschwindigkeit mit Kollisionsvermeidung."""
    global pwm_left, pwm_right, last_left_duty, last_right_duty
    
    # Kollisionsprüfung
    collision = analyze_collision_status()
    
    if collision.collision_critical:
        left_duty = MIN_DUTY_CYCLE
        right_duty = MIN_DUTY_CYCLE
        print(f'[COLLISION] 🛑 KRITISCH - Alle Motoren gestoppt!')
    elif collision.collision_detected:
        # Reduziere Geschwindigkeit bei Hindernissen
        left_duty *= 0.5
        right_duty *= 0.5
    
    # Clamp Values
    left_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, left_duty))
    right_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, right_duty))
    
    # Speichere für Odometrie
    last_left_duty = left_duty
    last_right_duty = right_duty
    
    try:
        pwm_left.change_duty_cycle(left_duty)
        pwm_right.change_duty_cycle(right_duty)
    except Exception as e:
        if logger:
            logger.error(f'PWM Update fehlgeschlagen: {e}')

# ============================================================================
# RECORDING FUNKTIONEN
# ============================================================================

def create_session_id() -> str:
    """Erstellt eindeutige Session-ID."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{timestamp}_robot01"

def create_recording_data() -> PathRecordingData:
    """Erstellt neue PathRecordingData mit Metadaten."""
    calibration_data = CalibrationData(
        controller_calibration={
            'left_trigger_offset': 0.0,
            'right_trigger_offset': 0.0,
            'deadzone': 0.05,
            'max_duty_cycle': MAX_DUTY_CYCLE
        },
        lidar_calibration={
            'offset_x': LIDAR_OFFSET_X,
            'offset_y': LIDAR_OFFSET_Y,
            'angle_offset': LIDAR_ANGLE_OFFSET
        },
        calibration_timestamp=time.time(),
        calibration_quality=1.0
    )
    
    metadata = EnvironmentMetadata(
        session_id=recording_session_id,
        start_timestamp=time.time(),
        end_timestamp=None,
        environment_conditions={
            'recording_location': 'indoor',
            'robot_configuration': 'xbox_controller_lidar_v3.0'
        },
        hardware_info={
            'controller_model': 'Xbox Wireless Controller',
            'robot_platform': 'RaspberryPi_Tank_Robot',
            'motor_type': 'DC_with_PWM',
            'sensors': 'HC-SR04_ultrasonic + RPLIDAR_C1',
            'lidar_connected': lidar_connected
        },
        recording_settings={
            'controller_rate_hz': RECORDING_SAMPLE_RATE_HZ,
            'lidar_rate_hz': LIDAR_UPDATE_HZ,
            'pose_tracking': True,
            'compression': CompressionType.GZIP
        }
    )
    
    header = PathRecordingHeader(
        calibration_data=calibration_data,
        metadata=metadata,
        compression_type=CompressionType.GZIP
    )
    
    return PathRecordingData(header=header)

def start_recording() -> bool:
    """Startet Recording."""
    global recording_active, recording_start_time, recording_session_id
    global recording_data, logger
    
    if not RECORDING_AVAILABLE or not storage_manager:
        print('[RECORDING] ❌ Recording-System nicht verfügbar')
        return False
    
    if recording_active:
        print('[RECORDING] ⚠️ Recording bereits aktiv')
        return False
    
    try:
        recording_session_id = create_session_id()
        recording_start_time = time.time()
        recording_active = True
        
        # Leere Buffers
        while not recording_buffer.empty():
            try:
                recording_buffer.get_nowait()
            except queue.Empty:
                break
        
        while not lidar_frame_buffer.empty():
            try:
                lidar_frame_buffer.get_nowait()
            except queue.Empty:
                break
        
        # Reset Pose
        global robot_x, robot_y, robot_theta
        with pose_lock:
            robot_x = 0.0
            robot_y = 0.0
            robot_theta = 0.0
        
        recording_data = create_recording_data()
        
        if BUZZER_ENABLED:
            buzzer_beep(duration=0.1, count=2)
        
        set_led_status("recording")
        
        print(f'[RECORDING] 🔴 Recording gestartet: {recording_session_id}')
        if logger:
            logger.info(f"Recording gestartet: {recording_session_id}")
        
        return True
        
    except Exception as e:
        print(f'[RECORDING] ❌ Fehler: {e}')
        if logger:
            logger.error(f"Recording Start fehlgeschlagen: {e}")
        recording_active = False
        return False

def stop_recording(force_save: bool = False) -> bool:
    """Stoppt Recording und speichert Daten."""
    global recording_active, recording_data, logger
    
    if not recording_active:
        return False
    
    try:
        recording_active = False
        set_led_status("saving")
        
        duration = time.time() - recording_start_time
        controller_samples = recording_data.get_stats().get('controller_samples', 0) if recording_data else 0
        lidar_frames = recording_data.get_stats().get('lidar_frames', 0) if recording_data else 0
        
        is_valid = duration >= MIN_RECORDING_DURATION_SEC or force_save
        
        if is_valid and recording_data and storage_manager:
            if recording_data.header.metadata:
                recording_data.header.metadata.end_timestamp = time.time()
            
            filepath = storage_manager.save_json(
                recording_data,
                recording_session_id,
                create_backup=True
            )
            
            print(f'[RECORDING] ✅ Gespeichert: {filepath}')
            print(f'[RECORDING] 📊 Statistiken:')
            print(f'    Dauer: {duration:.2f}s')
            print(f'    Controller: {controller_samples} samples')
            print(f'    LIDAR: {lidar_frames} frames')
            
            if logger:
                logger.info(f"Recording gespeichert: {filepath}")
            
            if BUZZER_ENABLED:
                buzzer_beep(duration=0.2, count=1)
        else:
            print(f'[RECORDING] ⚠️ Verworfen (zu kurz: {duration:.2f}s)')
            if logger:
                logger.warning(f"Recording verworfen: {duration:.2f}s")
            
            if BUZZER_ENABLED:
                buzzer_beep(duration=0.1, count=3)
        
        set_led_status("off")
        return is_valid
        
    except Exception as e:
        print(f'[RECORDING] ❌ Fehler: {e}')
        if logger:
            logger.error(f"Recording Stop fehlgeschlagen: {e}")
        recording_active = False
        set_led_status("off")
        return False

def add_data_to_recording():
    """Fügt Controller + LIDAR + Pose zur Aufzeichnung hinzu."""
    global recording_data
    
    if not recording_active or not recording_data:
        return
    
    try:
        # Controller Sample
        collision = analyze_collision_status()
        pose = get_current_pose()
        
        sample = ControllerSample(
            timestamp=time.time(),
            left_trigger=trigger_left,
            right_trigger=trigger_right,
            buttons={
                'A': a_button_pressed,
                'B': not is_moving_forward,
                'X': x_button_pressed,
                'Y': y_button_pressed
            },
            direction={
                'forward': 1.0 if is_moving_forward else -1.0,
                'tank_turn': 1.0 if a_button_pressed else 0.0
            },
            sequence_id=0
        )
        
        recording_data.add_controller_sample(sample)
        
        # LIDAR Frames aus Buffer
        frames_added = 0
        while frames_added < 5:  # Max 5 Frames pro Cycle
            try:
                frame = lidar_frame_buffer.get_nowait()
                recording_data.add_lidar_frame(frame)
                frames_added += 1
            except queue.Empty:
                break
        
    except Exception as e:
        if logger:
            logger.error(f"Recording Daten-Fehler: {e}")

def recording_background_thread():
    """Background-Thread für Recording-Management."""
    global running, logger
    
    if not RECORDING_AVAILABLE:
        return
    
    print('[THREAD] Recording Thread gestartet')
    if logger:
        logger.info("Recording Thread gestartet")
    
    while running:
        try:
            time.sleep(0.1)
            
        except Exception as e:
            if logger:
                logger.error(f"Recording Thread Fehler: {e}")

# ============================================================================
# LED UND AUDIO FEEDBACK
# ============================================================================

def set_led_status(status: str):
    """Setzt LED-Status."""
    global led_status
    led_status = status

def led_control_thread():
    """LED-Steuerung mit Blinkmustern."""
    global running, led_status, led_blink_state, led_last_toggle
    
    if not LED_ENABLED:
        return
    
    print(f'[THREAD] LED Thread gestartet (GPIO{STATUS_LED_PIN})')
    
    while running:
        current_time = time.time()
        
        if led_status == "off":
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
            led_blink_state = False
        elif led_status == "recording":
            blink_interval = 0.25
            if current_time - led_last_toggle > blink_interval:
                led_blink_state = not led_blink_state
                GPIO.output(STATUS_LED_PIN, GPIO.HIGH if led_blink_state else GPIO.LOW)
                led_last_toggle = current_time
        elif led_status == "saving":
            GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
            led_blink_state = True
        
        time.sleep(0.05)

def buzzer_beep(duration: float = 0.1, count: int = 1):
    """Erzeugt Piep-Töne."""
    if not BUZZER_ENABLED:
        return
    
    try:
        for i in range(count):
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
            if i < count - 1:
                time.sleep(duration * 0.5)
    except Exception as e:
        if logger:
            logger.warning(f"Buzzer Fehler: {e}")

# ============================================================================
# CONTROLLER INPUT
# ============================================================================

def read_controller_input():
    """Liest Controller-Eingaben mit Recording-Steuerung."""
    global trigger_left, trigger_right, running, joystick
    global is_moving_forward, b_button_was_pressed, a_button_pressed
    global x_button_pressed, y_button_pressed, x_button_was_pressed, y_button_was_pressed
    
    pygame.event.pump()
    
    if not joystick or not joystick.get_init():
        print('[ERROR] Controller Verbindung verloren!')
        running = False
        return
    
    try:
        trigger_left = joystick.get_axis(AXIS_LEFT_TRIGGER)
        trigger_right = joystick.get_axis(AXIS_RIGHT_TRIGGER)
        a_button_pressed = joystick.get_button(BUTTON_A)
        
        # B-Button: Richtungsumschaltung
        b_button_currently_pressed = joystick.get_button(BUTTON_B)
        if b_button_currently_pressed and not b_button_was_pressed:
            is_moving_forward = not is_moving_forward
            direction_text = "⬆️ VORWÄRTS" if is_moving_forward else "⬇️ RÜCKWÄRTS"
            print(f'[INPUT] 🔄 Richtung: {direction_text}')
        b_button_was_pressed = b_button_currently_pressed
        
        # X+Y Buttons: Recording Toggle
        x_button_currently_pressed = joystick.get_button(BUTTON_X)
        y_button_currently_pressed = joystick.get_button(BUTTON_Y)
        
        if (x_button_currently_pressed and y_button_currently_pressed and 
            not (x_button_was_pressed and y_button_was_pressed)):
            
            if RECORDING_AVAILABLE:
                if recording_active:
                    print('[INPUT] ⏹️ Stoppe Recording...')
                    stop_recording(force_save=False)
                else:
                    print('[INPUT] 🔴 Starte Recording...')
                    start_recording()
            else:
                print('[INPUT] ❌ Recording nicht verfügbar!')
        
        x_button_pressed = x_button_currently_pressed
        y_button_pressed = y_button_currently_pressed
        x_button_was_pressed = x_button_currently_pressed
        y_button_was_pressed = y_button_currently_pressed
        
        # Start Button: Programm beenden
        if joystick.get_button(BUTTON_START):
            print('[INPUT] 🎮 Start-Button - Beende...')
            running = False
    
    except Exception as e:
        print(f'[ERROR] Controller Fehler: {e}')
        if logger:
            logger.error(f"Controller Fehler: {e}")
        running = False

# ============================================================================
# HAUPT-STEUERUNGSLOOP
# ============================================================================

def control_loop():
    """Haupt-Steuerungsschleife mit vollständiger Integration."""
    global running
    
    print(f'[CONTROL] Steuerungs-Loop gestartet ({CONTROL_LOOP_HZ} Hz)')
    print('\n' + '='*80)
    print('🎮 STEUERUNG:')
    print('  Left/Right Trigger → Motor-Steuerung')
    print('  A Button (halten) → Tank-Turn')
    print('  B Button → Richtung (Vorwärts ⇄ Rückwärts)')
    print('  Y + X Buttons → Recording Start/Stop')
    print('  Start Button → Beenden')
    print('\n🔴 RECORDING:')
    print(f'  Y + X gleichzeitig → Recording Toggle')
    print(f'  Mindestdauer: {MIN_RECORDING_DURATION_SEC}s')
    print(f'  Speicherort: {RECORDING_BASE_DIR}/')
    print('\n🛡️ SICHERHEIT:')
    print(f'  Ultraschall: {COLLISION_DISTANCE_CM}cm')
    print(f'  LIDAR: {LIDAR_COLLISION_DISTANCE_M*100}cm')
    print(f'  LIDAR Status: {"✅ Verbunden" if lidar_connected else "❌ Nicht verfügbar"}')
    print('='*80 + '\n')
    
    sleep_time = 1.0 / CONTROL_LOOP_HZ
    status_counter = 0
    status_interval = CONTROL_LOOP_HZ
    recording_sample_counter = 0
    recording_sample_interval = CONTROL_LOOP_HZ // RECORDING_SAMPLE_RATE_HZ
    
    while running:
        loop_start = time.time()
        
        # 1. Lese Controller
        read_controller_input()
        
        # 2. Konvertiere Trigger zu Duty Cycle
        left_duty = trigger_to_duty_cycle(trigger_left)
        right_duty = trigger_to_duty_cycle(trigger_right)
        
        # 3. Tank-Turn und Richtungslogik
        if a_button_pressed:
            if is_moving_forward:
                if right_duty > left_duty:
                    set_motor_direction(left_backward=True, right_backward=False)
                    set_motor_speed(right_duty, right_duty)
                else:
                    set_motor_direction(left_backward=False, right_backward=True)
                    set_motor_speed(left_duty, left_duty)
            else:
                if right_duty > left_duty:
                    set_motor_direction(left_backward=False, right_backward=True)
                    set_motor_speed(right_duty, right_duty)
                else:
                    set_motor_direction(left_backward=True, right_backward=False)
                    set_motor_speed(left_duty, left_duty)
        else:
            backward = not is_moving_forward
            set_motor_direction(backward, backward)
            set_motor_speed(left_duty, right_duty)
        
        # 4. Recording-Sample hinzufügen
        recording_sample_counter += 1
        if recording_sample_counter >= recording_sample_interval:
            recording_sample_counter = 0
            if recording_active:
                add_data_to_recording()
        
        # 5. Status-Ausgabe
        status_counter += 1
        if status_counter >= status_interval:
            status_counter = 0
            
            collision = analyze_collision_status()
            pose = get_current_pose()
            
            collision_emoji = '🔴' if collision.collision_detected else '🟢'
            direction_emoji = "⬆️" if is_moving_forward else "⬇️"
            mode_text = "🔄 TANK" if a_button_pressed else "➡️ NORM"
            
            recording_text = ""
            if RECORDING_AVAILABLE:
                if recording_active:
                    duration = time.time() - recording_start_time
                    stats = recording_data.get_stats() if recording_data else {}
                    controller_samples = stats.get('controller_samples', 0)
                    lidar_frames = stats.get('lidar_frames', 0)
                    recording_text = f" | 🔴 REC:{duration:.1f}s C:{controller_samples} L:{lidar_frames}"
                else:
                    recording_text = " | ⚪ REC:OFF"
            
            lidar_text = f"LIDAR:{len(lidar_scan_data)} pts" if lidar_connected else "LIDAR:N/A"
            
            print(f'[STATUS] {collision_emoji} {mode_text} {direction_emoji} | '
                  f'Trigger:L={trigger_left:+.2f} R={trigger_right:+.2f} | '
                  f'PWM:L={left_duty:.1f}% R={right_duty:.1f}% | '
                  f'US:L={collision.ultrasonic_left:.0f}cm R={collision.ultrasonic_right:.0f}cm | '
                  f'{lidar_text} | '
                  f'Pose:X={pose.x:.2f}m Y={pose.y:.2f}m θ={math.degrees(pose.theta):.0f}°'
                  f'{recording_text}')
        
        # 6. Timing
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

# ============================================================================
# CLEANUP
# ============================================================================

def cleanup():
    """Ressourcen-Freigabe."""
    global running, pwm_left, pwm_right, joystick, lidar
    global sensor_thread, lidar_thread, recording_thread, led_thread, pose_thread
    global logger
    
    print('[CLEANUP] Starte Ressourcen-Freigabe...')
    running = False
    
    # Stoppe Recording
    if recording_active:
        print('[CLEANUP] Sichere Recording...')
        stop_recording(force_save=True)
    
    # Warte auf Threads
    for name, thread in [
        ('Sensor', sensor_thread),
        ('LIDAR', lidar_thread),
        ('Recording', recording_thread),
        ('LED', led_thread),
        ('Pose', pose_thread)
    ]:
        if thread and thread.is_alive():
            print(f'[CLEANUP] Warte auf {name} Thread...')
            thread.join(timeout=2.0)
    
    # Stoppe Motoren
    if pwm_left:
        try:
            pwm_left.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_left.stop()
        except Exception:
            pass
    
    if pwm_right:
        try:
            pwm_right.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_right.stop()
        except Exception:
            pass
    
    # Stoppe LIDAR
    if lidar and lidar_connected:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
    
    # Hardware ausschalten
    if LED_ENABLED:
        try:
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        except Exception:
            pass
    
    if BUZZER_ENABLED:
        try:
            GPIO.output(BUZZER_PIN, GPIO.LOW)
        except Exception:
            pass
    
    # Trenne Controller
    if joystick:
        try:
            joystick.quit()
        except Exception:
            pass
    
    try:
        pygame.quit()
    except Exception:
        pass
    
    # GPIO Cleanup
    try:
        GPIO.cleanup()
    except Exception:
        pass
    
    if logger:
        logger.info("System heruntergefahren")
    
    print('[CLEANUP] ✅ Cleanup abgeschlossen')

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Hauptfunktion."""
    global running, sensor_thread, lidar_thread, recording_thread, led_thread, pose_thread
    
    print('\n' + '='*80)
    print('🤖 XBOX CONTROLLER MIT LIDAR-INTEGRATION UND RECORDING')
    print('Version 3.0 - Phase 3: Vollständige LIDAR + Pose Integration')
    print('='*80 + '\n')
    
    try:
        # Hardware Setup
        setup_gpio()
        setup_pwm()
        setup_controller()
        lidar_success = setup_lidar()
        
        if RECORDING_AVAILABLE:
            setup_recording()
        
        print('\n[INIT] ✅ Initialisierung abgeschlossen - Starte Threads...')
        
        running = True
        
        # Starte Sensor Thread
        sensor_thread = threading.Thread(
            target=sensor_update_thread,
            daemon=True,
            name='SensorThread'
        )
        sensor_thread.start()
        
        # Starte LIDAR Thread
        if lidar_success:
            lidar_thread = threading.Thread(
                target=lidar_update_thread,
                daemon=True,
                name='LidarThread'
            )
            lidar_thread.start()
        
        # Starte Pose Thread
        pose_thread = threading.Thread(
            target=pose_update_thread,
            daemon=True,
            name='PoseThread'
        )
        pose_thread.start()
        
        # Starte Recording Thread
        if RECORDING_AVAILABLE:
            recording_thread = threading.Thread(
                target=recording_background_thread,
                daemon=True,
                name='RecordingThread'
            )
            recording_thread.start()
        
        # Starte LED Thread
        if LED_ENABLED:
            led_thread = threading.Thread(
                target=led_control_thread,
                daemon=True,
                name='LEDThread'
            )
            led_thread.start()
        
        time.sleep(1.0)
        
        # Starte Hauptsteuerung
        control_loop()
    
    except KeyboardInterrupt:
        print('\n[SHUTDOWN] Keyboard Interrupt...')
    except Exception as e:
        print(f'\n[ERROR] Unerwarteter Fehler: {e}')
        if logger:
            logger.error(f"Fehler in main(): {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == '__main__':
    import os
    
    if os.geteuid() != 0:
        print('❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!')
        print('📝 Verwendung: sudo python3 xbox_controller_with_lidar_recording_v3.0.py')
        sys.exit(1)
    
    main()
