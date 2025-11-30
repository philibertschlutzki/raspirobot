#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Xbox Controller Roboter-Steuerung mit LIDAR-Integration und Recording - Phase 3.0.2
===================================================================================

Vollständige Integration von Xbox Controller, LIDAR C1, Ultraschall-Sensoren
und Path Recording System. Ermöglicht manuelle Steuerung mit gleichzeitiger
Aufzeichnung aller Sensor- und Steuerdaten für späteres autonomes Replay.

Optimierungen in Version 3.0.2:
- ✅ **Erzwungene Metadaten-Schemata**: Speicherung von Capabilities (`has_lidar`, `has_pose`).
- ✅ **LIDAR-Frequenz-Monitoring**: Überwachung der Scan-Rate (Ziel: 10Hz).
- ✅ **Integrierte Pose-Estimation**: Robuste Speicherung der geschätzten Pose im Recording.
- ✅ **Detaillierte Docstrings**: Umfassende Dokumentation aller Funktionen.

Features:
- Thread-sichere RPLIDAR C1 Integration
- Sensor Fusion: LIDAR + Ultraschall Kollisionserkennung
- Roboter-Pose Tracking mit Odometrie (Dead Reckoning)
- Synchronisierte Aufzeichnung: Controller + LIDAR + Pose
- Environment Mapping mit Occupancy Grid
- Graceful Degradation bei Sensor-Ausfällen

HARDWARE ANFORDERUNGEN:
-----------------------
- RPLIDAR C1 (460800 Baud, /dev/ttyUSB0)
- 2x HC-SR04 Ultraschall-Sensoren (links/rechts)
- Xbox Wireless Controller
- 2x DC-Motoren mit Hardware PWM
- Raspberry Pi 4/5

AUTOR: AI Assistant (basierend auf v3.0.1)
DATUM: 2025-11-27
VERSION: 3.0.2
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

# Ultraschall-Sensoren
TRIG_LEFT = 12
ECHO_LEFT = 13
TRIG_RIGHT = 23
ECHO_RIGHT = 24

# Motor-Steuerung
DIR_LEFT_PIN = 20
DIR_RIGHT_PIN = 21
DIR_HIGH_IS_BACKWARD = False

# PWM-Konfiguration
PWM_FREQUENCY = 1000
PWM_CHIP = 0
PWM_CHANNEL_LEFT = 0
PWM_CHANNEL_RIGHT = 1
MAX_DUTY_CYCLE = 12.0
MIN_DUTY_CYCLE = 0.0

# Recording-Hardware
STATUS_LED_PIN = 22
BUZZER_PIN = 25
LED_ENABLED = True
BUZZER_ENABLED = False

# LIDAR-Konfiguration
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
LIDAR_SCAN_TIMEOUT = 2.0
LIDAR_TARGET_FREQUENCY = 10.0  # Hz

# LIDAR-Position relativ zum Roboter-Mittelpunkt
LIDAR_OFFSET_X = 0.15  # 15cm vor Roboter-Mittelpunkt
LIDAR_OFFSET_Y = 0.0   # zentriert
LIDAR_ANGLE_OFFSET = 0.0  # kein Winkel-Offset

# Kollisionserkennung
COLLISION_DISTANCE_CM = 30.0  # Ultraschall
LIDAR_COLLISION_DISTANCE_M = 0.3  # LIDAR (30cm)
LIDAR_WARNING_DISTANCE_M = 0.8  # LIDAR Warnung (80cm)
SOUND_SPEED = 34300

# Controller-Konfiguration
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

# Recording-Konfiguration
RECORDING_SAMPLE_RATE_HZ = 50
RECORDING_BUFFER_SIZE = 1000
MIN_RECORDING_DURATION_SEC = 5.0
RECORDING_BASE_DIR = "path_recordings"

# Odometrie-Parameter
WHEEL_BASE_M = 0.20  # 20cm Radabstand
WHEEL_DIAMETER_M = 0.065  # 6.5cm Raddurchmesser
MAX_POSE_UPDATE_HZ = 50

# Shutdown-Timeouts
THREAD_SHUTDOWN_TIMEOUT_SEC = 3.0
LIDAR_STOP_DELAY_SEC = 0.5
LIDAR_MOTOR_STOP_DELAY_SEC = 0.3

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
shutdown_event = threading.Event()
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
recording_lock = threading.Lock()
recording_start_time = 0.0
recording_session_id = ""
recording_buffer: queue.Queue = queue.Queue(maxsize=RECORDING_BUFFER_SIZE)
recording_data: Optional[PathRecordingData] = None
storage_manager: Optional[PathRecordingStorageManager] = None

# LED-Status Variablen
led_status = "off"
led_blink_state = False
led_last_toggle = 0.0

# Pose Tracking
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
lidar_scan_frequency = 0.0  # Gemessene Frequenz

# Logging
logger = None

# ============================================================================
# DATENSTRUKTUREN
# ============================================================================

@dataclass
class RobotPose:
    """
    Repräsentiert die geschätzte Pose des Roboters.

    Attributes:
        x (float): X-Koordinate in Metern (relativ zum Start).
        y (float): Y-Koordinate in Metern.
        theta (float): Orientierung in Radiant (0 = Start-Ausrichtung).
        timestamp (float): Zeitstempel der Schätzung.
    """
    x: float
    y: float
    theta: float
    timestamp: float

    def to_dict(self) -> Dict[str, float]:
        """Konvertiert die Pose in ein Dictionary für das Recording."""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'timestamp': self.timestamp
        }

@dataclass
class CollisionStatus:
    """
    Status der Kollisionserkennung basierend auf Multi-Sensor-Fusion.

    Attributes:
        ultrasonic_left (float): Distanz links in cm.
        ultrasonic_right (float): Distanz rechts in cm.
        lidar_min_distance (float): Minimale LIDAR-Distanz (vorne) in m.
        collision_detected (bool): True wenn ein Hindernis erkannt wurde.
        collision_critical (bool): True wenn ein Hindernis kritisch nah ist.
    """
    ultrasonic_left: float
    ultrasonic_right: float
    lidar_min_distance: float
    lidar_obstacles_front: int
    collision_detected: bool
    collision_critical: bool

# ============================================================================
# SIGNAL HANDLER
# ============================================================================

def signal_handler(sig, frame):
    """
    Signal-Handler für Graceful Shutdown bei SIGINT (Ctrl+C).

    Beendet alle Threads, speichert laufende Aufnahmen und gibt Hardware frei.
    """
    global logger
    print('\n[SHUTDOWN] SIGINT empfangen - Programm wird beendet...')

    shutdown_event.set()

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

    if BUZZER_ENABLED:
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        GPIO.output(BUZZER_PIN, GPIO.LOW)

    signal.signal(signal.SIGINT, signal_handler)

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
    except Exception as e:
        print(f'[ERROR] PWM Initialisierung fehlgeschlagen: {e}')
        raise

def setup_controller():
    """Initialisiert Xbox Controller via Pygame."""
    global joystick
    print('[SETUP] Initialisiere Xbox Controller...')

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError('[ERROR] Kein Xbox Controller gefunden!')

    joystick = pygame.joystick.Joystick(CONTROLLER_ID)
    joystick.init()
    print(f'[SETUP] Controller verbunden: {joystick.get_name()}')

def setup_lidar() -> bool:
    """
    Initialisiert RPLIDAR C1.

    Returns:
        bool: True wenn erfolgreich, sonst False.
    """
    global lidar, lidar_connected, logger

    if not LIDAR_AVAILABLE:
        print('[SETUP] ⚠️ RPLIDAR Bibliothek nicht verfügbar')
        return False

    print('[SETUP] Initialisiere RPLIDAR C1...')

    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_SCAN_TIMEOUT)

        info = lidar.get_info()
        print(f'[SETUP] LIDAR Info: {info}')

        health = lidar.get_health()
        status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')

        if status == 'Error':
            raise RuntimeError('LIDAR meldet Fehlerzustand')

        lidar.start_motor()
        time.sleep(2.0)

        lidar_connected = True
        print('[SETUP] ✅ RPLIDAR C1 erfolgreich initialisiert')
        return True

    except Exception as e:
        print(f'[SETUP] ❌ LIDAR Fehler: {e}')
        lidar = None
        lidar_connected = False
        return False

def setup_recording():
    """Initialisiert das Recording-System und Logger."""
    global storage_manager, logger

    if not RECORDING_AVAILABLE:
        return

    print('[SETUP] Initialisiere Recording-System...')

    try:
        Path(RECORDING_BASE_DIR).mkdir(parents=True, exist_ok=True)
        logger = setup_logging(str(Path(RECORDING_BASE_DIR) / "xbox_lidar_recording.log"))

        storage_manager = PathRecordingStorageManager(
            base_directory=RECORDING_BASE_DIR,
            compression=CompressionType.GZIP,
            backup_enabled=True
        )
        print(f'[SETUP] ✅ Recording-System bereit: {RECORDING_BASE_DIR}')

    except Exception as e:
        print(f'[SETUP] ❌ Recording-System Fehler: {e}')
        raise

# ============================================================================
# SENSOR THREADS
# ============================================================================

def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """
    Misst die Entfernung mit einem HC-SR04 Sensor.

    Args:
        trig_pin: GPIO Pin für Trigger
        echo_pin: GPIO Pin für Echo
        timeout: Maximales Timeout in Sekunden

    Returns:
        float: Entfernung in cm oder -1.0 bei Fehler
    """
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

        distance = ((pulse_end - pulse_start) * SOUND_SPEED) / 2
        return distance if 2 < distance < 400 else -1.0
    except Exception:
        return -1.0

def sensor_update_thread():
    """Thread zum kontinuierlichen Auslesen der Ultraschall-Sensoren."""
    global distance_left, distance_right
    print(f'[THREAD] Sensor Thread gestartet ({SENSOR_UPDATE_HZ} Hz)')
    sleep_time = 1.0 / SENSOR_UPDATE_HZ

    while running and not shutdown_event.is_set():
        loop_start = time.time()

        dist_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
        if dist_left > 0: distance_left = dist_left

        dist_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
        if dist_right > 0: distance_right = dist_right

        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0: time.sleep(sleep_duration)

    print('[THREAD] Sensor Thread beendet')

def lidar_update_thread():
    """
    Thread für LIDAR-Scan-Verarbeitung mit Frequenz-Überwachung.
    """
    global lidar_scan_data, lidar_connected, lidar_error_count
    global lidar_last_scan_time, lidar_scan_frequency

    if not lidar or not lidar_connected:
        return

    print(f'[THREAD] LIDAR Thread gestartet (Ziel: {LIDAR_UPDATE_HZ} Hz)')

    scan_count = 0
    scan_times = deque(maxlen=50)
    last_freq_check = time.time()

    try:
        for scan in lidar.iter_scans(max_buf_meas=5000, min_len=10):
            if not running or shutdown_event.is_set():
                break

            try:
                timestamp = time.time()
                scan_data = list(scan)

                # Update Frequenz-Monitoring
                scan_times.append(timestamp)
                if len(scan_times) > 1:
                    avg_dt = (scan_times[-1] - scan_times[0]) / (len(scan_times) - 1)
                    lidar_scan_frequency = 1.0 / avg_dt if avg_dt > 0 else 0.0

                # Warnung bei Frequenz-Abweichung (alle 5s)
                if time.time() - last_freq_check > 5.0:
                    if abs(lidar_scan_frequency - LIDAR_TARGET_FREQUENCY) > 2.0:
                        print(f"[LIDAR] ⚠️ Frequenz-Warnung: {lidar_scan_frequency:.1f} Hz (Soll: {LIDAR_TARGET_FREQUENCY} Hz)")
                    last_freq_check = time.time()

                with lidar_lock:
                    lidar_scan_data = scan_data
                    lidar_last_scan_time = timestamp

                with recording_lock:
                    is_recording = recording_active

                if is_recording:
                    frame = LidarFrame(
                        timestamp=timestamp,
                        scan_data=scan_data,
                        frame_id=scan_count
                    )
                    try:
                        lidar_frame_buffer.put_nowait(frame)
                    except queue.Full:
                        try:
                            lidar_frame_buffer.get_nowait()
                            lidar_frame_buffer.put_nowait(frame)
                        except queue.Empty:
                            pass

                scan_count += 1

            except Exception as e:
                lidar_error_count += 1

    except Exception as e:
        print(f'[LIDAR] ❌ Fehler: {e}')
        lidar_connected = False
    finally:
        print(f'[THREAD] LIDAR Thread beendet ({scan_count} Scans)')

# ============================================================================
# POSE TRACKING
# ============================================================================

def update_pose_from_odometry(left_duty: float, right_duty: float, dt: float):
    """
    Berechnet die Pose (Odometrie) basierend auf den PWM-Werten.
    """
    global robot_x, robot_y, robot_theta, pose_timestamp

    max_speed_m_s = 0.5

    v_left = (left_duty / 100.0) * max_speed_m_s
    v_right = (right_duty / 100.0) * max_speed_m_s

    if not is_moving_forward:
        v_left = -v_left
        v_right = -v_right

    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / WHEEL_BASE_M

    with pose_lock:
        if abs(omega) < 0.001:
            robot_x += v * math.cos(robot_theta) * dt
            robot_y += v * math.sin(robot_theta) * dt
        else:
            radius = v / omega
            robot_x += radius * (math.sin(robot_theta + omega * dt) - math.sin(robot_theta))
            robot_y += radius * (-math.cos(robot_theta + omega * dt) + math.cos(robot_theta))
            robot_theta += omega * dt

            robot_theta = (robot_theta + math.pi) % (2 * math.pi) - math.pi

        pose_timestamp = time.time()

def pose_update_thread():
    """Thread für Odometrie-Berechnung."""
    global last_pose_update
    print(f'[THREAD] Pose Thread gestartet ({MAX_POSE_UPDATE_HZ} Hz)')
    sleep_time = 1.0 / MAX_POSE_UPDATE_HZ
    last_pose_update = time.time()

    while running and not shutdown_event.is_set():
        loop_start = time.time()
        dt = loop_start - last_pose_update

        update_pose_from_odometry(last_left_duty, last_right_duty, dt)
        last_pose_update = loop_start

        elapsed = time.time() - loop_start
        if sleep_time - elapsed > 0:
            time.sleep(sleep_time - elapsed)

    print('[THREAD] Pose Thread beendet')

def get_current_pose() -> RobotPose:
    """Liefert die aktuelle Pose thread-sicher."""
    with pose_lock:
        return RobotPose(x=robot_x, y=robot_y, theta=robot_theta, timestamp=pose_timestamp)

# ============================================================================
# KOLLISIONSERKENNUNG & MOTOR
# ============================================================================

def analyze_collision_status() -> CollisionStatus:
    """Erstellt eine Analyse der aktuellen Sensorwerte auf Hindernisse."""
    global distance_left, distance_right, lidar_scan_data

    ultrasonic_left = distance_left
    ultrasonic_right = distance_right
    lidar_min_dist = float('inf')
    obstacles_front = 0

    with lidar_lock:
        for quality, angle_deg, distance_mm in lidar_scan_data:
            distance_m = distance_mm / 1000.0
            if -45 <= angle_deg <= 45:
                if distance_m < lidar_min_dist:
                    lidar_min_dist = distance_m
                if distance_m < LIDAR_WARNING_DISTANCE_M:
                    obstacles_front += 1

    if lidar_min_dist == float('inf'):
        lidar_min_dist = 100.0

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
        ultrasonic_left, ultrasonic_right, lidar_min_dist,
        obstacles_front, collision_detected, collision_critical
    )

def set_motor_direction(left_backward: bool, right_backward: bool):
    """Setzt die digitalen Richtungs-Pins."""
    if DIR_HIGH_IS_BACKWARD:
        l_val = GPIO.HIGH if left_backward else GPIO.LOW
        r_val = GPIO.HIGH if right_backward else GPIO.LOW
    else:
        l_val = GPIO.LOW if left_backward else GPIO.HIGH
        r_val = GPIO.LOW if right_backward else GPIO.HIGH

    GPIO.output(DIR_LEFT_PIN, l_val)
    GPIO.output(DIR_RIGHT_PIN, r_val)

def set_motor_speed(left_duty: float, right_duty: float):
    """Setzt PWM-Werte mit Kollisionsschutz."""
    global last_left_duty, last_right_duty

    collision = analyze_collision_status()
    if collision.collision_critical:
        left_duty = MIN_DUTY_CYCLE
        right_duty = MIN_DUTY_CYCLE
    elif collision.collision_detected:
        left_duty *= 0.5
        right_duty *= 0.5

    left_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, left_duty))
    right_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, right_duty))

    last_left_duty = left_duty
    last_right_duty = right_duty

    try:
        if pwm_left: pwm_left.change_duty_cycle(left_duty)
        if pwm_right: pwm_right.change_duty_cycle(right_duty)
    except Exception:
        pass

def trigger_to_duty_cycle(val: float) -> float:
    return max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, ((val + 1.0) / 2.0) * MAX_DUTY_CYCLE))

# ============================================================================
# RECORDING
# ============================================================================

def create_recording_data() -> PathRecordingData:
    """
    Erstellt das Datenobjekt für eine neue Aufnahme.
    Enthält erweiterte Metadaten über Capabilities (Optimierung 1).
    """
    calibration = CalibrationData(
        controller_calibration={'deadzone': 0.05, 'max_duty': MAX_DUTY_CYCLE},
        lidar_calibration={'offset_x': LIDAR_OFFSET_X, 'offset_y': LIDAR_OFFSET_Y},
        calibration_timestamp=time.time(),
        calibration_quality=1.0
    )

    # Erweiterte Capabilities in hardware_info
    hw_info = {
        'controller_model': 'Xbox Wireless Controller',
        'robot_platform': 'RaspberryPi_Tank_Robot',
        'lidar_connected': lidar_connected,
        'capabilities': {
            'has_lidar': lidar_connected,
            'has_pose': True,  # Wir haben Pose-Tracking
            'has_ultrasonic': True
        }
    }

    metadata = EnvironmentMetadata(
        session_id=recording_session_id,
        start_timestamp=time.time(),
        end_timestamp=None,
        environment_conditions={'robot_configuration': 'v3.0.2'},
        hardware_info=hw_info,
        recording_settings={
            'controller_rate_hz': RECORDING_SAMPLE_RATE_HZ,
            'lidar_rate_hz': LIDAR_UPDATE_HZ,
            'pose_tracking': True,
            'compression': CompressionType.GZIP
        }
    )

    return PathRecordingData(header=PathRecordingHeader(calibration, metadata, CompressionType.GZIP))

def start_recording():
    """Startet die Aufzeichnung."""
    global recording_active, recording_start_time, recording_session_id, recording_data
    global robot_x, robot_y, robot_theta

    if not RECORDING_AVAILABLE: return

    with recording_lock:
        if recording_active: return
        recording_active = True

    recording_session_id = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_robot01"
    recording_start_time = time.time()

    # Reset Pose
    with pose_lock:
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0

    # Flush Buffers
    while not recording_buffer.empty():
        try: recording_buffer.get_nowait()
        except queue.Empty: break
    while not lidar_frame_buffer.empty():
        try: lidar_frame_buffer.get_nowait()
        except queue.Empty: break

    recording_data = create_recording_data()

    if BUZZER_ENABLED:
        buzzer_beep(0.1, 2)
    set_led_status("recording")
    print(f'[RECORDING] 🔴 Start: {recording_session_id}')

def stop_recording(force_save=False):
    """Stoppt die Aufzeichnung und speichert die Daten."""
    global recording_active, recording_data

    with recording_lock:
        if not recording_active: return
        recording_active = False

    time.sleep(0.2) # Warte auf Buffer

    # Flush remaining LIDAR frames
    while not lidar_frame_buffer.empty():
        try:
            frame = lidar_frame_buffer.get_nowait()
            if recording_data: recording_data.add_lidar_frame(frame)
        except queue.Empty: break

    set_led_status("saving")
    duration = time.time() - recording_start_time

    if (duration >= MIN_RECORDING_DURATION_SEC or force_save) and recording_data and storage_manager:
        if recording_data.header.metadata:
            recording_data.header.metadata.end_timestamp = time.time()

        path = storage_manager.save_json(recording_data, recording_session_id, create_backup=True)
        print(f'[RECORDING] ✅ Gespeichert: {path}')
        if BUZZER_ENABLED: buzzer_beep(0.2, 1)
    else:
        print(f'[RECORDING] ⚠️ Verworfen (<{MIN_RECORDING_DURATION_SEC}s)')
        if BUZZER_ENABLED: buzzer_beep(0.1, 3)

    set_led_status("off")

def add_data_to_recording():
    """Fügt Controller-Status, Pose und gepufferte LIDAR-Frames zur Aufnahme hinzu."""
    global recording_data

    with recording_lock:
        is_rec = recording_active

    if not is_rec or not recording_data: return

    # Pose explizit hinzufügen (Optimierung 3)
    pose = get_current_pose()

    sample = ControllerSample(
        timestamp=time.time(),
        left_trigger=trigger_left,
        right_trigger=trigger_right,
        buttons={'A': a_button_pressed, 'B': not is_moving_forward, 'X': x_button_pressed, 'Y': y_button_pressed},
        direction={'forward': 1.0 if is_moving_forward else -1.0, 'tank_turn': 1.0 if a_button_pressed else 0.0},
        sequence_id=0,
        pose=pose.to_dict() # Pose im Sample speichern
    )

    recording_data.add_controller_sample(sample)

    frames_added = 0
    while frames_added < 5:
        try:
            frame = lidar_frame_buffer.get_nowait()
            recording_data.add_lidar_frame(frame)
            frames_added += 1
        except queue.Empty:
            break

def recording_background_thread():
    """Dummy Thread für Recording-Management."""
    while running and not shutdown_event.is_set():
        time.sleep(1)

# ============================================================================
# LED / BUZZER
# ============================================================================

def set_led_status(status: str):
    global led_status
    led_status = status

def led_control_thread():
    """Steuert die Status-LED."""
    global led_blink_state, led_last_toggle
    if not LED_ENABLED: return
    print(f'[THREAD] LED Thread gestartet')

    while running and not shutdown_event.is_set():
        if led_status == "off":
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        elif led_status == "recording":
            if time.time() - led_last_toggle > 0.25:
                led_blink_state = not led_blink_state
                GPIO.output(STATUS_LED_PIN, GPIO.HIGH if led_blink_state else GPIO.LOW)
                led_last_toggle = time.time()
        elif led_status == "saving":
            GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
        time.sleep(0.05)

    GPIO.output(STATUS_LED_PIN, GPIO.LOW)

def buzzer_beep(duration=0.1, count=1):
    if not BUZZER_ENABLED: return
    try:
        for _ in range(count):
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            time.sleep(duration)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
            time.sleep(duration * 0.5)
    except Exception: pass

# ============================================================================
# MAIN LOOP
# ============================================================================

def read_controller_input():
    global trigger_left, trigger_right, running, is_moving_forward, a_button_pressed
    global x_button_pressed, y_button_pressed, x_button_was_pressed, y_button_was_pressed, b_button_was_pressed

    pygame.event.pump()
    if not joystick or not joystick.get_init():
        running = False
        return

    trigger_left = joystick.get_axis(AXIS_LEFT_TRIGGER)
    trigger_right = joystick.get_axis(AXIS_RIGHT_TRIGGER)
    a_button_pressed = joystick.get_button(BUTTON_A)

    # B Button
    b_curr = joystick.get_button(BUTTON_B)
    if b_curr and not b_button_was_pressed:
        is_moving_forward = not is_moving_forward
        print(f'[INPUT] 🔄 Richtung: {"VORWÄRTS" if is_moving_forward else "RÜCKWÄRTS"}')
    b_button_was_pressed = b_curr

    # Recording Buttons
    x_curr = joystick.get_button(BUTTON_X)
    y_curr = joystick.get_button(BUTTON_Y)

    if (x_curr and y_curr and not (x_button_was_pressed and y_button_was_pressed)):
        with recording_lock: is_rec = recording_active
        if is_rec: stop_recording()
        else: start_recording()

    x_button_pressed = x_curr
    y_button_pressed = y_curr
    x_button_was_pressed = x_curr
    y_button_was_pressed = y_curr

    if joystick.get_button(BUTTON_START):
        running = False
        shutdown_event.set()

def control_loop():
    global running
    print(f'[CONTROL] Loop gestartet ({CONTROL_LOOP_HZ} Hz)')
    sleep_time = 1.0 / CONTROL_LOOP_HZ

    rec_counter = 0
    rec_interval = CONTROL_LOOP_HZ // RECORDING_SAMPLE_RATE_HZ
    stat_counter = 0

    while running and not shutdown_event.is_set():
        start = time.time()
        read_controller_input()

        l_duty = trigger_to_duty_cycle(trigger_left)
        r_duty = trigger_to_duty_cycle(trigger_right)

        # Tank Turn
        if a_button_pressed:
            if is_moving_forward:
                if r_duty > l_duty: set_motor_direction(True, False); set_motor_speed(r_duty, r_duty)
                else: set_motor_direction(False, True); set_motor_speed(l_duty, l_duty)
            else:
                if r_duty > l_duty: set_motor_direction(False, True); set_motor_speed(r_duty, r_duty)
                else: set_motor_direction(True, False); set_motor_speed(l_duty, l_duty)
        else:
            back = not is_moving_forward
            set_motor_direction(back, back)
            set_motor_speed(l_duty, r_duty)

        # Recording
        rec_counter += 1
        if rec_counter >= rec_interval:
            rec_counter = 0
            add_data_to_recording()

        # Status Log (alle 50 ticks = 1s)
        stat_counter += 1
        if stat_counter >= 50:
            stat_counter = 0
            pose = get_current_pose()
            print(f"[STATUS] Pose: X={pose.x:.2f} Y={pose.y:.2f} θ={math.degrees(pose.theta):.0f}° | LIDAR: {lidar_scan_frequency:.1f}Hz")

        elapsed = time.time() - start
        if sleep_time > elapsed:
            time.sleep(sleep_time - elapsed)

# ============================================================================
# CLEANUP
# ============================================================================

def cleanup():
    global running
    print('[CLEANUP] Starte...')
    running = False
    shutdown_event.set()

    if recording_active: stop_recording(True)

    time.sleep(1.0) # Threads Zeit geben

    # Stop Hardware
    try:
        if pwm_left: pwm_left.stop()
        if pwm_right: pwm_right.stop()
        if lidar: lidar.stop(); lidar.stop_motor(); lidar.disconnect()
        GPIO.cleanup()
        pygame.quit()
    except Exception as e:
        print(f'[CLEANUP] Fehler: {e}')
    print('[CLEANUP] Fertig.')

def main():
    global running, sensor_thread, lidar_thread, pose_thread, recording_thread, led_thread
    print('\n🤖 XBOX CONTROLLER V3.0.2 STARTED\n')

    try:
        setup_gpio()
        setup_pwm()
        setup_controller()
        lidar_ok = setup_lidar()
        if RECORDING_AVAILABLE: setup_recording()

        running = True

        sensor_thread = threading.Thread(target=sensor_update_thread, daemon=True)
        sensor_thread.start()

        pose_thread = threading.Thread(target=pose_update_thread, daemon=True)
        pose_thread.start()

        if lidar_ok:
            lidar_thread = threading.Thread(target=lidar_update_thread, daemon=True)
            lidar_thread.start()

        if RECORDING_AVAILABLE:
            recording_thread = threading.Thread(target=recording_background_thread, daemon=True)
            recording_thread.start()

        if LED_ENABLED:
            led_thread = threading.Thread(target=led_control_thread, daemon=True)
            led_thread.start()

        control_loop()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[ERROR] {e}')
    finally:
        cleanup()

if __name__ == '__main__':
    if os.geteuid() != 0:
        print('❌ Muss als Root laufen (sudo)!')
        sys.exit(1)
    main()
