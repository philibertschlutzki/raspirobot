#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autonome Path Replay mit vollständiger Dateninterpretation - Phase 7
=====================================================================

Erweiterte Pfadverfolgung mit:
- Vollständiger Interpretation aller Recording-Daten (Controller, LIDAR, Pose, Metadata)
- Hybrid-Pfadkorrektur: Pose-Tracking + LIDAR-Scan-Matching
- Automatische Initialisierung via LIDAR-Lokalisierung
- Adaptive Motor-Command-Modifikation zur Pfadkorrektur
- Multi-Sensor-Kollisionsvermeidung mit Priorität

NEUE FEATURES Phase 7:
----------------------
- ✅ Vollständiges Laden aller Recording-Datenstrukturen
- ✅ LIDAR-Scan-Matching zur Positions-Korrektur
- ✅ Pose-basierte Pfadverfolgung (x, y, theta)
- ✅ Automatische Initialisierung anhand erstem LIDAR-Scan
- ✅ Adaptive Motor-Korrektur bei Abweichungen
- ✅ Umfangreiches Debug-System (--debug)
- ✅ Optimiert für Raspberry Pi 5 + RPLIDAR C1

HARDWARE:
---------
- RPLIDAR C1 (460800 Baud, /dev/ttyUSB0)
- 2x HC-SR04 Ultraschall-Sensoren (links/rechts)
- 2x DC-Motoren mit PWM-Ansteuerung
- Raspberry Pi 5

VERWENDUNG:
-----------
sudo python3 path_replay_full_v2.py <path_to_recording.json.gz> [OPTIONS]

Optionen:
--speed-factor X     Globaler Geschwindigkeitsfaktor (0.1-1.0, default: 0.8)
--debug              Aktiviert detaillierte Debug-Ausgaben
--no-lidar-match     Deaktiviert LIDAR-Scan-Matching (nur Pose-Tracking)
--max-deviation X    Maximale Pfadabweichung in cm (default: 50)

Beispiele:
sudo python3 path_replay_full_v2.py path_recordings/20251127_204739_robot01.json.gz
sudo python3 path_replay_full_v2.py path_recordings/20251127_204739_robot01.json.gz --debug --speed-factor 0.5
sudo python3 path_replay_full_v2.py path_recordings/20251127_204739_robot01.json.gz --debug --max-deviation 30

AUTOR: Phase 7 - Vollständige Dateninterpretation
DATUM: 2025-11-27
VERSION: 2.0
"""

import RPi.GPIO as GPIO
import sys
import time
import signal
import threading
import json
import gzip
import math
import argparse
import numpy as np
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple, NamedTuple
from dataclasses import dataclass, field
from collections import deque
from rpi_hardware_pwm import HardwarePWM

# RPLIDAR Import
try:
    from rplidar import RPLidar, RPLidarException
    LIDAR_AVAILABLE = True
except ImportError:
    print("⚠️ RPLIDAR nicht verfügbar - nur Ultraschall aktiv")
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

# LIDAR-Konfiguration (RPLIDAR C1)
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
LIDAR_SCAN_TIMEOUT = 2.0

# Kollisionserkennung
SOUND_SPEED = 34300  # cm/s
CRITICAL_DISTANCE_CM = 20.0
WARNING_DISTANCE_CM = 40.0
SAFE_DISTANCE_CM = 60.0

# Update-Frequenzen
SENSOR_UPDATE_HZ = 20
CONTROL_LOOP_HZ = 50
LIDAR_UPDATE_HZ = 10

# ============================================================================
# PFADKORREKTUR PARAMETER
# ============================================================================

# Pose-Korrektur PID
POSE_KP_LINEAR = 0.8      # Proportional für lineare Abweichung
POSE_KI_LINEAR = 0.05     # Integral für lineare Abweichung
POSE_KD_LINEAR = 0.2      # Derivative für lineare Abweichung

POSE_KP_ANGULAR = 1.2     # Proportional für Winkel-Abweichung
POSE_KI_ANGULAR = 0.1     # Integral für Winkel-Abweichung
POSE_KD_ANGULAR = 0.3     # Derivative für Winkel-Abweichung

# LIDAR-Scan-Matching
SCAN_MATCH_ANGLE_RESOLUTION = 5    # Grad pro Bin
SCAN_MATCH_MAX_DISTANCE_M = 5.0    # Maximale Distanz für Matching
SCAN_MATCH_MIN_QUALITY = 0.5       # Minimum Match-Quality (0-1)
SCAN_MATCH_WEIGHT = 0.3            # Gewichtung LIDAR vs. Odometrie

# Lookahead
LOOKAHEAD_TIME_SEC = 0.5           # Lookahead-Zeit
LOOKAHEAD_MIN_DISTANCE_M = 0.2     # Minimum Lookahead-Distanz

# Limits
MAX_CORRECTION_SPEED = 0.3         # Max Korrektur-Geschwindigkeit (m/s)
MAX_CORRECTION_ANGULAR = 0.8       # Max Korrektur-Drehrate (rad/s)

# Konfigurierbare Optionen (CLI)
GLOBAL_SPEED_FACTOR = 0.8
MAX_PATH_DEVIATION_CM = 50.0
DEBUG_MODE = False
ENABLE_LIDAR_MATCHING = True

# ============================================================================
# GLOBALE VARIABLEN
# ============================================================================

# Hardware
pwm_left: Optional[HardwarePWM] = None
pwm_right: Optional[HardwarePWM] = None
lidar: Optional[RPLidar] = None

# Thread Control
running = False
shutdown_event = threading.Event()
sensor_thread: Optional[threading.Thread] = None
lidar_thread: Optional[threading.Thread] = None

# Sensor-Daten
distance_left = 100.0
distance_right = 100.0
lidar_scan_data: List[Tuple[float, float, float]] = []  # (quality, angle_deg, distance_mm)
lidar_lock = threading.Lock()

# Recording-Daten
recording_data: Optional[Dict[str, Any]] = None
controller_samples: List[Dict] = []
lidar_frames: List[Dict] = []
metadata: Optional[Dict] = None
calibration: Optional[Dict] = None

# Playback State
current_sample_index = 0
playback_start_time = 0.0
playback_active = False
initialized = False

# Robot State
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
robot_pose_lock = threading.Lock()

# Korrektur-State
correction_integral_x = 0.0
correction_integral_y = 0.0
correction_integral_theta = 0.0
last_correction_time = 0.0

# Safety
emergency_stop = False
obstacle_detected = False

# Statistics
stats_total_corrections = 0
stats_max_deviation = 0.0
stats_scan_matches = 0
stats_scan_match_quality_avg = 0.0

# ============================================================================
# DATENSTRUKTUREN
# ============================================================================

@dataclass
class Pose2D:
    """2D Roboter-Pose."""
    x: float
    y: float
    theta: float
    timestamp: float = 0.0

    def distance_to(self, other: 'Pose2D') -> float:
        """Berechnet euklidische Distanz zu anderer Pose."""
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx*dx + dy*dy)

    def angle_to(self, other: 'Pose2D') -> float:
        """Berechnet Winkel zu anderer Pose."""
        dx = other.x - self.x
        dy = other.y - self.y
        return math.atan2(dy, dx)

    def angle_difference(self, other: 'Pose2D') -> float:
        """Berechnet Winkeldifferenz (normalisiert auf [-pi, pi])."""
        diff = other.theta - self.theta
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

@dataclass
class ControlCommand:
    """Motor-Steuerbefehl."""
    left_duty: float
    right_duty: float
    forward: bool
    tank_turn: bool
    timestamp: float

@dataclass
class PathDeviation:
    """Pfadabweichung."""
    lateral_error: float      # Querabweichung in m
    heading_error: float      # Winkel-Fehler in rad
    distance_error: float     # Längsabweichung in m
    total_error: float        # Gesamt-Fehler in m

@dataclass
class ScanMatchResult:
    """Ergebnis des LIDAR-Scan-Matchings."""
    matched: bool
    quality: float            # 0-1
    translation_x: float      # m
    translation_y: float      # m
    rotation: float          # rad
    num_points_matched: int

@dataclass
class CorrectionOutput:
    """Korrektur-Ausgabe."""
    left_correction: float    # Zusätzlicher Duty-Cycle links
    right_correction: float   # Zusätzlicher Duty-Cycle rechts
    reason: str              # Grund für Korrektur

# ============================================================================
# SIGNAL HANDLER
# ============================================================================

def signal_handler(sig, frame):
    """Graceful Shutdown."""
    global running
    print('\n[SHUTDOWN] SIGINT empfangen - Beende...')
    running = False
    shutdown_event.set()
    cleanup()
    sys.exit(0)

# ============================================================================
# HARDWARE SETUP
# ============================================================================

def setup_gpio():
    """Initialisiert GPIO."""
    print('[SETUP] Initialisiere GPIO...')

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Ultraschall
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.output(TRIG_LEFT, False)

    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    GPIO.output(TRIG_RIGHT, False)

    # Motoren
    GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
    GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)

    if DIR_HIGH_IS_BACKWARD:
        GPIO.output(DIR_LEFT_PIN, GPIO.LOW)
        GPIO.output(DIR_RIGHT_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_LEFT_PIN, GPIO.HIGH)
        GPIO.output(DIR_RIGHT_PIN, GPIO.HIGH)

    signal.signal(signal.SIGINT, signal_handler)

    print('[SETUP] GPIO initialisiert - Sensoren stabilisieren (2s)...')
    time.sleep(2)

def setup_pwm():
    """Initialisiert PWM."""
    global pwm_left, pwm_right

    print('[SETUP] Initialisiere Hardware PWM...')

    try:
        pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)

        pwm_left.start(MIN_DUTY_CYCLE)
        pwm_right.start(MIN_DUTY_CYCLE)

        print(f'[SETUP] PWM initialisiert - {PWM_FREQUENCY} Hz')
    except Exception as e:
        print(f'[ERROR] PWM Fehler: {e}')
        raise

def setup_lidar() -> bool:
    """Initialisiert RPLIDAR C1."""
    global lidar

    if not LIDAR_AVAILABLE:
        print('[SETUP] ⚠️ RPLIDAR nicht verfügbar')
        return False

    print('[SETUP] Initialisiere RPLIDAR C1...')

    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_SCAN_TIMEOUT)

        info = lidar.get_info()
        print(f'[SETUP] LIDAR Info: {info}')

        health = lidar.get_health()
        status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')

        if status == 'Error':
            raise RuntimeError('LIDAR meldet Fehler')

        lidar.start_motor()
        time.sleep(2.0)

        print('[SETUP] ✅ RPLIDAR C1 initialisiert')
        return True

    except Exception as e:
        print(f'[SETUP] ❌ LIDAR Fehler: {e}')
        lidar = None
        return False

# ============================================================================
# DATEN LADEN
# ============================================================================

def load_recording_data(filepath: str) -> bool:
    """
    Lädt vollständige Recording-Daten.

    Args:
        filepath: Pfad zur .json.gz Datei

    Returns:
        True bei Erfolg
    """
    global recording_data, controller_samples, lidar_frames, metadata, calibration

    print(f'[LOAD] Lade Recording: {filepath}')

    try:
        with gzip.open(filepath, 'rt', encoding='utf-8') as f:
            recording_data = json.load(f)

        # Extrahiere Komponenten
        controller_samples = recording_data.get('controller_samples', [])
        lidar_frames = recording_data.get('lidar_frames', [])

        # Header-Daten
        header = recording_data.get('header', {})
        metadata = header.get('metadata', {})
        calibration = header.get('calibration_data', {})

        # Statistiken
        print(f'[LOAD] ✅ Recording geladen:')
        print(f'  Controller Samples: {len(controller_samples)}')
        print(f'  LIDAR Frames: {len(lidar_frames)}')

        if metadata:
            session_id = metadata.get('session_id', 'Unknown')
            start_time = metadata.get('start_timestamp', 0)
            end_time = metadata.get('end_timestamp', 0)
            duration = end_time - start_time if end_time else 0

            print(f'  Session ID: {session_id}')
            print(f'  Dauer: {duration:.2f}s')

            env_cond = metadata.get('environment_conditions', {})
            hw_info = metadata.get('hardware_info', {})

            if env_cond:
                print(f'  Umgebung: {env_cond.get("recording_location", "N/A")}')
            if hw_info:
                lidar_status = "✅" if hw_info.get('lidar_connected') else "❌"
                print(f'  LIDAR im Recording: {lidar_status}')

        if calibration:
            lidar_cal = calibration.get('lidar_calibration', {})
            if lidar_cal:
                print(f'  LIDAR Offset: x={lidar_cal.get("offset_x", 0)}m y={lidar_cal.get("offset_y", 0)}m')

        # Validierung
        if not controller_samples:
            raise ValueError("Keine Controller-Samples")

        # Normalisiere Zeitstempel
        start_timestamp = controller_samples[0]['timestamp']
        for sample in controller_samples:
            sample['timestamp'] -= start_timestamp

        for frame in lidar_frames:
            frame['timestamp'] -= start_timestamp

        print(f'[LOAD] Zeitstempel normalisiert')

        return True

    except Exception as e:
        print(f'[ERROR] Laden fehlgeschlagen: {e}')
        return False

# ============================================================================
# SENSOR FUNKTIONEN
# ============================================================================

def measure_distance(trig_pin: int, echo_pin: int, timeout: float = 0.1) -> float:
    """Misst Ultraschall-Distanz."""
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

def sensor_update_thread_func():
    """Ultraschall-Sensor Thread."""
    global distance_left, distance_right, running

    print(f'[THREAD] Sensor Thread gestartet ({SENSOR_UPDATE_HZ} Hz)')
    sleep_time = 1.0 / SENSOR_UPDATE_HZ

    while running and not shutdown_event.is_set():
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

    print('[THREAD] Sensor Thread beendet')

def lidar_update_thread_func():
    """LIDAR-Scan Thread."""
    global lidar_scan_data, running, lidar

    if not lidar:
        return

    print(f'[THREAD] LIDAR Thread gestartet ({LIDAR_UPDATE_HZ} Hz)')

    try:
        for scan in lidar.iter_scans(max_buf_meas=5000, min_len=10):
            if not running or shutdown_event.is_set():
                break

            try:
                scan_data = list(scan)

                with lidar_lock:
                    lidar_scan_data = scan_data

            except Exception as e:
                if DEBUG_MODE:
                    print(f'[LIDAR] Scan-Fehler: {e}')

    except RPLidarException as e:
        print(f'[LIDAR] ❌ Fehler: {e}')
    except Exception as e:
        print(f'[LIDAR] ❌ Unerwarteter Fehler: {e}')
    finally:
        print('[THREAD] LIDAR Thread beendet')

# ============================================================================
# LIDAR SCAN MATCHING
# ============================================================================

def create_polar_histogram(scan_data: List[Tuple[float, float, float]], 
                          resolution_deg: int = SCAN_MATCH_ANGLE_RESOLUTION) -> np.ndarray:
    """
    Erstellt Polar-Histogramm aus LIDAR-Scan.

    Args:
        scan_data: Liste von (quality, angle_deg, distance_mm)
        resolution_deg: Winkel-Auflösung in Grad

    Returns:
        Numpy-Array mit Distanzen pro Winkel-Bin
    """
    num_bins = 360 // resolution_deg
    histogram = np.full(num_bins, SCAN_MATCH_MAX_DISTANCE_M * 1000, dtype=np.float32)

    for quality, angle_deg, distance_mm in scan_data:
        # Normalisiere Winkel auf [0, 360)
        angle_norm = angle_deg % 360
        bin_idx = int(angle_norm / resolution_deg)

        if 0 <= bin_idx < num_bins:
            # Nehme minimale Distanz pro Bin
            histogram[bin_idx] = min(histogram[bin_idx], distance_mm)

    return histogram

def match_scans(current_scan: List[Tuple[float, float, float]],
                reference_scan: List[Tuple[float, float, float]]) -> ScanMatchResult:
    """
    Vergleicht zwei LIDAR-Scans (vereinfachtes Scan-Matching).

    Args:
        current_scan: Aktueller Live-Scan
        reference_scan: Referenz-Scan aus Recording

    Returns:
        ScanMatchResult mit Match-Quality und Transformation
    """
    if not current_scan or not reference_scan:
        return ScanMatchResult(False, 0.0, 0.0, 0.0, 0.0, 0)

    # Erstelle Polar-Histogramme
    hist_current = create_polar_histogram(current_scan)
    hist_reference = create_polar_histogram(reference_scan)

    # Berechne beste Rotation (Brute-Force über diskrete Winkel)
    best_rotation_idx = 0
    best_score = 0.0

    # Suche beste Ausrichtung (alle Bins testen)
    num_bins = len(hist_current)
    for shift in range(num_bins):
        hist_shifted = np.roll(hist_current, shift)

        # Berechne Similarity-Score (invertierte normalisierte Differenz)
        diff = np.abs(hist_shifted - hist_reference)
        max_dist = SCAN_MATCH_MAX_DISTANCE_M * 1000
        normalized_diff = diff / max_dist
        similarity = 1.0 - np.mean(np.minimum(normalized_diff, 1.0))

        if similarity > best_score:
            best_score = similarity
            best_rotation_idx = shift

    # Berechne Rotation in Radiant
    rotation_deg = best_rotation_idx * SCAN_MATCH_ANGLE_RESOLUTION
    rotation_rad = math.radians(rotation_deg)

    # Normalisiere auf [-pi, pi]
    if rotation_rad > math.pi:
        rotation_rad -= 2 * math.pi

    # Schätze Translation (vereinfacht: aus durchschnittlicher Distanz-Differenz)
    hist_aligned = np.roll(hist_current, best_rotation_idx)
    dist_diff = hist_aligned - hist_reference

    # Filtere Outliers
    valid_mask = np.abs(dist_diff) < (SCAN_MATCH_MAX_DISTANCE_M * 1000 * 0.5)
    avg_diff_mm = np.mean(dist_diff[valid_mask]) if np.any(valid_mask) else 0.0

    # Konvertiere zu Translation (grobe Schätzung)
    translation_m = avg_diff_mm / 1000.0

    # Quality-Check
    matched = best_score >= SCAN_MATCH_MIN_QUALITY
    num_matched = np.sum(valid_mask)

    return ScanMatchResult(
        matched=matched,
        quality=best_score,
        translation_x=translation_m * math.cos(rotation_rad),
        translation_y=translation_m * math.sin(rotation_rad),
        rotation=rotation_rad,
        num_points_matched=int(num_matched)
    )

def find_closest_lidar_frame(timestamp: float) -> Optional[Dict]:
    """
    Findet LIDAR-Frame mit nächstem Zeitstempel.

    Args:
        timestamp: Ziel-Zeitstempel

    Returns:
        LIDAR-Frame Dict oder None
    """
    if not lidar_frames:
        return None

    # Binäre Suche für Performance
    closest_frame = None
    min_diff = float('inf')

    for frame in lidar_frames:
        diff = abs(frame['timestamp'] - timestamp)
        if diff < min_diff:
            min_diff = diff
            closest_frame = frame

    return closest_frame

# ============================================================================
# INITIALISIERUNG
# ============================================================================

def initialize_robot_pose() -> bool:
    """
    Initialisiert Roboter-Pose anhand erstem LIDAR-Scan.

    Returns:
        True bei Erfolg
    """
    global robot_x, robot_y, robot_theta, initialized

    print('[INIT] Starte automatische Lokalisierung...')

    if not lidar or not lidar_frames:
        print('[INIT] ⚠️ LIDAR nicht verfügbar - nutze Startpose (0, 0, 0)')
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
        initialized = True
        return True

    # Warte auf ersten stabilen LIDAR-Scan
    print('[INIT] Warte auf stabilen LIDAR-Scan...')
    stable_scans = []
    max_attempts = 50

    for attempt in range(max_attempts):
        time.sleep(0.2)

        with lidar_lock:
            if lidar_scan_data and len(lidar_scan_data) > 50:
                stable_scans.append(list(lidar_scan_data))

                if len(stable_scans) >= 3:
                    break

    if len(stable_scans) < 3:
        print('[INIT] ⚠️ Nicht genug Scans - nutze Startpose (0, 0, 0)')
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
        initialized = True
        return True

    # Matche gegen ersten Recording-Frame
    first_recording_frame = lidar_frames[0]
    reference_scan = [(q, a, d) for q, a, d in first_recording_frame.get('scan_data', [])]

    # Nutze besten Match aus stabilen Scans
    best_match = None
    best_quality = 0.0

    for current_scan in stable_scans:
        match_result = match_scans(current_scan, reference_scan)

        if match_result.quality > best_quality:
            best_quality = match_result.quality
            best_match = match_result

    if best_match and best_match.matched:
        # Setze initiale Pose basierend auf Match
        robot_x = best_match.translation_x
        robot_y = best_match.translation_y
        robot_theta = best_match.rotation

        print(f'[INIT] ✅ Lokalisierung erfolgreich:')
        print(f'  Position: x={robot_x:.3f}m, y={robot_y:.3f}m')
        print(f'  Orientierung: θ={math.degrees(robot_theta):.1f}°')
        print(f'  Match Quality: {best_match.quality:.2f}')
    else:
        print('[INIT] ⚠️ Matching fehlgeschlagen - nutze Startpose (0, 0, 0)')
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0

    initialized = True
    return True

# ============================================================================
# PFADABWEICHUNG
# ============================================================================

def calculate_path_deviation(current_pose: Pose2D, target_pose: Pose2D) -> PathDeviation:
    """
    Berechnet Pfadabweichung.

    Args:
        current_pose: Aktuelle Roboter-Pose
        target_pose: Ziel-Pose aus Recording

    Returns:
        PathDeviation mit allen Fehler-Komponenten
    """
    # Distanz
    distance_error = current_pose.distance_to(target_pose)

    # Winkel zum Ziel
    angle_to_target = current_pose.angle_to(target_pose)

    # Heading-Fehler
    heading_error = current_pose.angle_difference(target_pose)

    # Lateral-Fehler (Querabweichung)
    lateral_error = distance_error * math.sin(angle_to_target - current_pose.theta)

    # Total-Fehler
    total_error = distance_error

    return PathDeviation(
        lateral_error=lateral_error,
        heading_error=heading_error,
        distance_error=distance_error,
        total_error=total_error
    )

# ============================================================================
# PFADKORREKTUR
# ============================================================================

def calculate_correction(current_pose: Pose2D, 
                        target_pose: Pose2D,
                        base_command: ControlCommand,
                        dt: float) -> CorrectionOutput:
    """
    Berechnet Motor-Korrektur basierend auf Pfadabweichung.

    Verwendet PID-Controller für lineare und Winkel-Abweichung.

    Args:
        current_pose: Aktuelle Roboter-Pose
        target_pose: Ziel-Pose
        base_command: Basis-Motorbefehl aus Recording
        dt: Zeitdelta

    Returns:
        CorrectionOutput mit Duty-Cycle-Korrekturen
    """
    global correction_integral_x, correction_integral_y, correction_integral_theta
    global stats_total_corrections

    # Berechne Abweichung
    deviation = calculate_path_deviation(current_pose, target_pose)

    # PID für lineare Abweichung (x, y)
    error_x = target_pose.x - current_pose.x
    error_y = target_pose.y - current_pose.y

    correction_integral_x += error_x * dt
    correction_integral_y += error_y * dt

    # Anti-Windup
    max_integral = 1.0
    correction_integral_x = max(-max_integral, min(max_integral, correction_integral_x))
    correction_integral_y = max(-max_integral, min(max_integral, correction_integral_y))

    # PID Output (linear)
    correction_linear = math.sqrt(
        (POSE_KP_LINEAR * error_x + POSE_KI_LINEAR * correction_integral_x) ** 2 +
        (POSE_KP_LINEAR * error_y + POSE_KI_LINEAR * correction_integral_y) ** 2
    )

    # PID für Winkel-Abweichung
    error_theta = deviation.heading_error
    correction_integral_theta += error_theta * dt
    correction_integral_theta = max(-max_integral, min(max_integral, correction_integral_theta))

    correction_angular = (POSE_KP_ANGULAR * error_theta + 
                         POSE_KI_ANGULAR * correction_integral_theta)

    # Limitiere Korrekturen
    correction_linear = max(-MAX_CORRECTION_SPEED, min(MAX_CORRECTION_SPEED, correction_linear))
    correction_angular = max(-MAX_CORRECTION_ANGULAR, min(MAX_CORRECTION_ANGULAR, correction_angular))

    # Konvertiere zu Duty-Cycle-Anpassung
    # Positiver correction_angular = drehe rechts
    left_correction = correction_linear - correction_angular * 0.5
    right_correction = correction_linear + correction_angular * 0.5

    # Skaliere auf Duty-Cycle-Range
    duty_scale = MAX_DUTY_CYCLE / MAX_CORRECTION_SPEED
    left_correction *= duty_scale * 0.3  # Reduzierter Faktor für sanfte Korrektur
    right_correction *= duty_scale * 0.3

    # Limitiere auf +/- 50% des MAX_DUTY_CYCLE
    max_corr = MAX_DUTY_CYCLE * 0.5
    left_correction = max(-max_corr, min(max_corr, left_correction))
    right_correction = max(-max_corr, min(max_corr, right_correction))

    # Statistik
    stats_total_corrections += 1

    reason = f"Linear:{deviation.distance_error:.2f}m Angular:{math.degrees(deviation.heading_error):.1f}°"

    return CorrectionOutput(
        left_correction=left_correction,
        right_correction=right_correction,
        reason=reason
    )

# ============================================================================
# KOLLISIONSVERMEIDUNG
# ============================================================================

def check_collision() -> Tuple[bool, float]:
    """
    Prüft Kollisions-Status.

    Returns:
        (emergency_stop, speed_factor)
    """
    global distance_left, distance_right, lidar_scan_data

    min_distance = min(distance_left, distance_right)

    # LIDAR-Daten prüfen
    with lidar_lock:
        for quality, angle_deg, distance_mm in lidar_scan_data:
            distance_cm = distance_mm / 10.0

            # Nur vorderer Sektor
            if -45 <= angle_deg <= 45:
                min_distance = min(min_distance, distance_cm)

    # Entscheidung
    if min_distance < CRITICAL_DISTANCE_CM:
        return True, 0.0  # Emergency Stop
    elif min_distance < WARNING_DISTANCE_CM:
        # Lineare Geschwindigkeitsreduktion
        factor = (min_distance - CRITICAL_DISTANCE_CM) / (WARNING_DISTANCE_CM - CRITICAL_DISTANCE_CM)
        return False, max(0.3, factor)
    elif min_distance < SAFE_DISTANCE_CM:
        factor = (min_distance - WARNING_DISTANCE_CM) / (SAFE_DISTANCE_CM - WARNING_DISTANCE_CM)
        return False, 0.7 + 0.3 * factor
    else:
        return False, 1.0

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
    """Konvertiert Trigger zu Duty Cycle."""
    normalized = (trigger_value + 1.0) / 2.0
    duty_cycle = normalized * MAX_DUTY_CYCLE
    return max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, duty_cycle))

def apply_motor_command(command: ControlCommand, 
                       correction: CorrectionOutput,
                       speed_factor: float):
    """
    Wendet Motor-Befehl mit Korrektur an.

    Args:
        command: Basis-Befehl aus Recording
        correction: Korrektur-Output
        speed_factor: Geschwindigkeits-Faktor (Kollision)
    """
    global pwm_left, pwm_right, GLOBAL_SPEED_FACTOR

    # Basis-Duty-Cycles
    left_duty = command.left_duty
    right_duty = command.right_duty

    # Wende Korrektur an
    left_duty += correction.left_correction
    right_duty += correction.right_correction

    # Wende Geschwindigkeits-Faktoren an
    left_duty *= speed_factor * GLOBAL_SPEED_FACTOR
    right_duty *= speed_factor * GLOBAL_SPEED_FACTOR

    # Clamp
    left_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, left_duty))
    right_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, right_duty))

    # Setze Richtung
    backward = not command.forward
    set_motor_direction(backward, backward)

    # Setze PWM
    try:
        pwm_left.change_duty_cycle(left_duty)
        pwm_right.change_duty_cycle(right_duty)
    except Exception as e:
        if DEBUG_MODE:
            print(f'[ERROR] PWM Update: {e}')

# ============================================================================
# ODOMETRIE UPDATE
# ============================================================================

def update_odometry(command: ControlCommand, dt: float):
    """
    Aktualisiert Roboter-Pose basierend auf Motor-Befehlen.

    Vereinfachte Odometrie-Schätzung.

    Args:
        command: Aktueller Motor-Befehl
        dt: Zeitdelta
    """
    global robot_x, robot_y, robot_theta

    # Parameter
    wheel_base = 0.20  # 20cm
    max_speed = 0.5    # 0.5 m/s bei 100%

    # Geschwindigkeiten
    v_left = (command.left_duty / MAX_DUTY_CYCLE) * max_speed
    v_right = (command.right_duty / MAX_DUTY_CYCLE) * max_speed

    if not command.forward:
        v_left = -v_left
        v_right = -v_right

    # Roboter-Geschwindigkeit
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / wheel_base

    # Update Pose
    with robot_pose_lock:
        if abs(omega) < 0.001:
            # Geradeaus
            robot_x += v * math.cos(robot_theta) * dt
            robot_y += v * math.sin(robot_theta) * dt
        else:
            # Kreisbahn
            radius = v / omega
            robot_x += radius * (math.sin(robot_theta + omega * dt) - math.sin(robot_theta))
            robot_y += radius * (-math.cos(robot_theta + omega * dt) + math.cos(robot_theta))
            robot_theta += omega * dt

            # Normalisiere
            robot_theta = (robot_theta + math.pi) % (2 * math.pi) - math.pi

def get_current_pose() -> Pose2D:
    """Holt aktuelle Pose thread-safe."""
    with robot_pose_lock:
        return Pose2D(robot_x, robot_y, robot_theta, time.time())

# ============================================================================
# PLAYBACK
# ============================================================================

def extract_pose_from_sample(sample: Dict) -> Optional[Pose2D]:
    """
    Extrahiert Pose aus Controller-Sample (falls vorhanden).

    Args:
        sample: Controller-Sample Dict

    Returns:
        Pose2D oder None
    """
    # Prüfe ob Pose-Daten vorhanden (neuere Recordings)
    if 'pose' in sample:
        pose_data = sample['pose']
        return Pose2D(
            x=pose_data.get('x', 0.0),
            y=pose_data.get('y', 0.0),
            theta=pose_data.get('theta', 0.0),
            timestamp=sample['timestamp']
        )

    # Fallback: Schätze aus vorherigen Samples (simple Integration)
    return None

def playback_with_correction():
    """
    Hauptfunktion: Playback mit vollständiger Pfadkorrektur.
    """
    global playback_active, playback_start_time, current_sample_index
    global emergency_stop, obstacle_detected, running
    global stats_max_deviation, stats_scan_matches, stats_scan_match_quality_avg

    print('\n[PLAYBACK] Starte Pfadverfolgung mit vollständiger Dateninterpretation...')
    print('='*80)

    playback_start_time = time.time()
    current_sample_index = 0
    playback_active = True

    sleep_time = 1.0 / CONTROL_LOOP_HZ
    status_counter = 0
    status_interval = CONTROL_LOOP_HZ

    last_update_time = playback_start_time

    # Rekonstruiere Soll-Trajektorie (falls nicht direkt in Samples)
    target_trajectory = []
    for sample in controller_samples:
        pose = extract_pose_from_sample(sample)
        if pose:
            target_trajectory.append(pose)

    use_trajectory = len(target_trajectory) > 0

    if use_trajectory:
        print(f'[PLAYBACK] Nutze Pose-Trajektorie ({len(target_trajectory)} Punkte)')
    else:
        print('[PLAYBACK] ⚠️ Keine Pose-Daten - nur Controller-Replay')

    while running and playback_active and not shutdown_event.is_set():
        loop_start = time.time()

        # Berechne Zeit
        elapsed_time = loop_start - playback_start_time
        dt = loop_start - last_update_time
        last_update_time = loop_start

        # Finde aktuelles Sample
        while current_sample_index < len(controller_samples):
            sample = controller_samples[current_sample_index]
            if sample['timestamp'] <= elapsed_time:
                current_sample_index += 1
            else:
                break

        # Prüfe Ende
        if current_sample_index >= len(controller_samples):
            print('[PLAYBACK] ✅ Pfad-Ende erreicht')
            playback_active = False
            break

        current_sample = controller_samples[current_sample_index]

        # Erstelle Basis-Command
        base_command = ControlCommand(
            left_duty=trigger_to_duty_cycle(current_sample['left_trigger']),
            right_duty=trigger_to_duty_cycle(current_sample['right_trigger']),
            forward=current_sample['direction']['forward'] > 0,
            tank_turn=current_sample['direction'].get('tank_turn', 0.0) > 0,
            timestamp=current_sample['timestamp']
        )

        # Update Odometrie
        update_odometry(base_command, dt)

        # Hole aktuelle Pose
        current_pose = get_current_pose()

        # Finde Ziel-Pose
        target_pose = None
        if use_trajectory and current_sample_index < len(target_trajectory):
            target_pose = target_trajectory[current_sample_index]

        # Berechne Korrektur (falls Ziel vorhanden)
        correction = CorrectionOutput(0.0, 0.0, "No Correction")
        deviation = None

        if target_pose:
            deviation = calculate_path_deviation(current_pose, target_pose)

            # Prüfe maximale Abweichung
            if deviation.total_error * 100 > MAX_PATH_DEVIATION_CM:
                print(f'[WARNING] ⚠️ Pfadabweichung zu groß: {deviation.total_error*100:.1f}cm')
                # Optional: Pause oder Abbruch

            stats_max_deviation = max(stats_max_deviation, deviation.total_error * 100)

            # Berechne Korrektur
            correction = calculate_correction(current_pose, target_pose, base_command, dt)

        # LIDAR-Scan-Matching (optional)
        scan_match_correction = None
        if ENABLE_LIDAR_MATCHING and lidar and target_pose:
            reference_frame = find_closest_lidar_frame(current_sample['timestamp'])

            if reference_frame:
                with lidar_lock:
                    current_scan = list(lidar_scan_data)

                if current_scan:
                    reference_scan = [(q, a, d) for q, a, d in reference_frame.get('scan_data', [])]
                    match_result = match_scans(current_scan, reference_scan)

                    if match_result.matched:
                        stats_scan_matches += 1
                        stats_scan_match_quality_avg = (
                            (stats_scan_match_quality_avg * (stats_scan_matches - 1) + match_result.quality) / 
                            stats_scan_matches
                        )

                        # Wende LIDAR-Korrektur an (gewichtet)
                        correction.left_correction += match_result.translation_x * SCAN_MATCH_WEIGHT * 10
                        correction.right_correction += match_result.translation_y * SCAN_MATCH_WEIGHT * 10

                        if DEBUG_MODE:
                            print(f'[SCAN-MATCH] Quality:{match_result.quality:.2f} ' +
                                  f'Trans:({match_result.translation_x:.3f}, {match_result.translation_y:.3f}) ' +
                                  f'Rot:{math.degrees(match_result.rotation):.1f}°')

        # Kollisionsprüfung
        emergency_stop, speed_factor = check_collision()

        if emergency_stop:
            print('[SAFETY] 🛑 EMERGENCY STOP')
            apply_motor_command(
                ControlCommand(0, 0, True, False, elapsed_time),
                CorrectionOutput(0, 0, "Emergency Stop"),
                0.0
            )
            time.sleep(0.5)
            continue

        # Wende Motor-Befehl an
        apply_motor_command(base_command, correction, speed_factor)

        # Status-Ausgabe
        status_counter += 1
        if (status_counter >= status_interval) or DEBUG_MODE:
            status_counter = 0

            progress = (current_sample_index / len(controller_samples)) * 100
            direction = "⬆️" if base_command.forward else "⬇️"

            status_line = f'[STATUS] {progress:5.1f}% | {direction} | '
            status_line += f'Speed:{speed_factor*GLOBAL_SPEED_FACTOR*100:3.0f}% | '
            status_line += f'PWM:L={base_command.left_duty*speed_factor*GLOBAL_SPEED_FACTOR:.1f}% '
            status_line += f'R={base_command.right_duty*speed_factor*GLOBAL_SPEED_FACTOR:.1f}% | '
            status_line += f'US:L={distance_left:.0f}cm R={distance_right:.0f}cm'

            if DEBUG_MODE and deviation:
                status_line += f' | 📊 Abweichung:{deviation.total_error*100:.1f}cm '
                status_line += f'Lateral:{deviation.lateral_error*100:.1f}cm '
                status_line += f'Heading:{math.degrees(deviation.heading_error):.1f}° '
                status_line += f'Korr:L={correction.left_correction:.2f} R={correction.right_correction:.2f}'

            print(status_line)

        # Timing
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)

    # Stoppe Motoren
    apply_motor_command(
        ControlCommand(0, 0, True, False, 0),
        CorrectionOutput(0, 0, "Playback Ende"),
        0.0
    )

    # Finale Statistiken
    print('\n[PLAYBACK] ✅ Abgeschlossen')
    print('='*80)
    print(f'Gesamt-Korrekturen: {stats_total_corrections}')
    print(f'Max. Abweichung: {stats_max_deviation:.1f}cm')
    if stats_scan_matches > 0:
        print(f'LIDAR-Matches: {stats_scan_matches}')
        print(f'Durchschn. Match-Quality: {stats_scan_match_quality_avg:.2f}')
    print('='*80)

# ============================================================================
# CLEANUP
# ============================================================================

def cleanup():
    """Cleanup."""
    global running, pwm_left, pwm_right, lidar
    global sensor_thread, lidar_thread

    print('[CLEANUP] Starte Cleanup...')
    running = False
    shutdown_event.set()

    # Threads
    for name, thread in [('Sensor', sensor_thread), ('LIDAR', lidar_thread)]:
        if thread and thread.is_alive():
            print(f'[CLEANUP] Warte auf {name} Thread...')
            thread.join(timeout=3.0)

    # Motoren
    if pwm_left:
        try:
            pwm_left.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_left.stop()
        except Exception as e:
            print(f'[CLEANUP] PWM Left: {e}')

    if pwm_right:
        try:
            pwm_right.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_right.stop()
        except Exception as e:
            print(f'[CLEANUP] PWM Right: {e}')

    # LIDAR
    if lidar:
        try:
            lidar.stop()
            time.sleep(0.5)
            lidar.stop_motor()
            time.sleep(0.3)
            lidar.disconnect()
        except Exception as e:
            print(f'[CLEANUP] LIDAR: {e}')

    # GPIO
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f'[CLEANUP] GPIO: {e}')

    print('[CLEANUP] ✅ Abgeschlossen')

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Hauptfunktion."""
    global running, sensor_thread, lidar_thread
    global GLOBAL_SPEED_FACTOR, DEBUG_MODE, ENABLE_LIDAR_MATCHING, MAX_PATH_DEVIATION_CM

    # Argument Parser
    parser = argparse.ArgumentParser(
        description='Autonome Pfadverfolgung mit vollständiger Dateninterpretation',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('recording', type=str,
                       help='Pfad zur Recording-Datei (.json.gz)')
    parser.add_argument('--speed-factor', type=float, default=0.8,
                       help='Geschwindigkeitsfaktor (0.1-1.0, default: 0.8)')
    parser.add_argument('--debug', action='store_true',
                       help='Aktiviert Debug-Ausgaben')
    parser.add_argument('--no-lidar-match', action='store_true',
                       help='Deaktiviert LIDAR-Scan-Matching')
    parser.add_argument('--max-deviation', type=float, default=50.0,
                       help='Max. Pfadabweichung in cm (default: 50)')

    args = parser.parse_args()

    # Konfiguration
    GLOBAL_SPEED_FACTOR = max(0.1, min(1.0, args.speed_factor))
    DEBUG_MODE = args.debug
    ENABLE_LIDAR_MATCHING = not args.no_lidar_match
    MAX_PATH_DEVIATION_CM = args.max_deviation
    recording_path = args.recording

    print('\n' + '='*80)
    print('🤖 AUTONOME PFADVERFOLGUNG - PHASE 7')
    print('Vollständige Dateninterpretation mit Hybrid-Korrektur')
    print('='*80)
    print(f'Recording: {recording_path}')
    print(f'Speed Factor: {GLOBAL_SPEED_FACTOR*100:.0f}%')
    print(f'Debug Mode: {"✅" if DEBUG_MODE else "❌"}')
    print(f'LIDAR Matching: {"✅" if ENABLE_LIDAR_MATCHING else "❌"}')
    print(f'Max Deviation: {MAX_PATH_DEVIATION_CM:.0f}cm')
    print('='*80 + '\n')

    if not Path(recording_path).exists():
        print(f'❌ [ERROR] Datei nicht gefunden: {recording_path}')
        sys.exit(1)

    try:
        # Lade Recording
        if not load_recording_data(recording_path):
            sys.exit(1)

        # Hardware Setup
        setup_gpio()
        setup_pwm()
        lidar_ok = setup_lidar()

        if ENABLE_LIDAR_MATCHING and not lidar_ok:
            print('[WARNING] ⚠️ LIDAR nicht verfügbar - Scan-Matching deaktiviert')
            ENABLE_LIDAR_MATCHING = False

        print('\n[INIT] ✅ Hardware initialisiert - Starte Threads...')

        running = True

        # Sensor Thread
        sensor_thread = threading.Thread(
            target=sensor_update_thread_func,
            daemon=True,
            name='SensorThread'
        )
        sensor_thread.start()

        # LIDAR Thread
        if lidar_ok:
            lidar_thread = threading.Thread(
                target=lidar_update_thread_func,
                daemon=True,
                name='LIDARThread'
            )
            lidar_thread.start()

        time.sleep(1.0)

        # Initialisierung
        if not initialize_robot_pose():
            print('[ERROR] Initialisierung fehlgeschlagen')
            sys.exit(1)

        # Start Playback
        playback_with_correction()

    except KeyboardInterrupt:
        print('\n[SHUTDOWN] Keyboard Interrupt')
    except Exception as e:
        print(f'\n[ERROR] Fehler: {e}')
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == '__main__':
    import os

    if os.geteuid() != 0:
        print('❌ [ERROR] Muss mit sudo ausgeführt werden!')
        print('📝 Verwendung: sudo python3 path_replay_full_v2.py <recording.json.gz> [OPTIONS]')
        sys.exit(1)

    main()
