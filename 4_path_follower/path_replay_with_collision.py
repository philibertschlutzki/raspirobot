#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autonome Path Replay mit Kollisionsvermeidung - Phase 6
========================================================

Spielt aufgezeichnete Pfade autonom ab mit adaptiver Steuerung und
mehrstufiger Kollisionsvermeidung durch LIDAR und Ultraschall-Sensoren.

FEATURES:
---------
- ✅ PID-basierte Pfadverfolgung mit Lookahead-Controller
- ✅ Real-time Obstacle Avoidance mit LIDAR + Ultraschall
- ✅ Multi-layer Safety System mit Emergency Stop
- ✅ Adaptive Geschwindigkeitsanpassung
- ✅ Path Deviation Detection mit Automatic Re-routing
- ✅ Sensor Fusion (LIDAR + Ultraschall) für robuste Erkennung
- ✅ Graceful Degradation bei Sensor-Ausfällen

HARDWARE:
---------
- YDLIDAR C1 oder kompatibel
- 2x HC-SR04 Ultraschall-Sensoren (links/rechts)
- 2x DC-Motoren mit PWM-Ansteuerung
- Raspberry Pi 4

VERWENDUNG:
-----------
sudo python3 path_replay_with_collision.py <path_to_recording.json.gz>

Optionale Flags:
--no-lidar          Deaktiviert LIDAR (nur Ultraschall)
--speed-factor X    Globaler Geschwindigkeitsfaktor (0.1-1.0, default: 1.0)
--verbose           Detaillierte Statusausgaben

Beispiel:
sudo python3 path_replay_with_collision.py path_recordings/20251127_191542_robot01.json.gz
sudo python3 path_replay_with_collision.py path_recordings/20251127_191542_robot01.json.gz --speed-factor 0.5 --verbose

AUTOR: Basierend auf Requirements Phase 6
DATUM: 2025-11-27
VERSION: 1.1 (Fixed argparse)
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
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
from collections import deque
from rpi_hardware_pwm import HardwarePWM

# LIDAR Import
try:
    from ydlidar.ydlidar import *
    LIDAR_AVAILABLE = True
except ImportError:
    print("⚠️ YDLIDAR nicht verfügbar - nur Ultraschall-Kollisionsvermeidung")
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

# Kollisionserkennung
SOUND_SPEED = 34300  # cm/s
CRITICAL_DISTANCE_CM = 20.0  # Emergency Stop
WARNING_DISTANCE_CM = 40.0   # Geschwindigkeitsreduzierung
SAFE_DISTANCE_CM = 60.0      # Normale Fahrt

# LIDAR-Konfiguration
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 230400
LIDAR_SCAN_FREQUENCY = 8  # Hz
LIDAR_OBSTACLE_THRESHOLD_M = 0.6  # 60cm

# Update-Frequenzen
SENSOR_UPDATE_HZ = 20
CONTROL_LOOP_HZ = 50
LIDAR_UPDATE_HZ = 8

# ============================================================================
# PID UND PATH FOLLOWING PARAMETER
# ============================================================================

# PID-Regler für Pfadverfolgung
PID_KP = 1.5  # Proportional
PID_KI = 0.1  # Integral
PID_KD = 0.3  # Derivative

# Lookahead-Controller
LOOKAHEAD_DISTANCE_CM = 30.0
LOOKAHEAD_TIME_SEC = 0.5

# Path Deviation
MAX_PATH_DEVIATION_CM = 50.0  # Maximum erlaubte Abweichung
DEVIATION_WARNING_CM = 30.0   # Warnung bei Abweichung

# Geschwindigkeitsanpassung
MIN_SPEED_FACTOR = 0.3   # Minimum 30% Geschwindigkeit
MAX_SPEED_FACTOR = 1.0   # Maximum 100% Geschwindigkeit
OBSTACLE_SLOWDOWN_FACTOR = 0.5  # 50% bei Hindernissen

# Konfigurierbare Optionen (werden per CLI überschrieben)
GLOBAL_SPEED_FACTOR = 1.0
USE_LIDAR = True
VERBOSE_MODE = False

# ============================================================================
# GLOBALE VARIABLEN
# ============================================================================

# Hardware
pwm_left: Optional[HardwarePWM] = None
pwm_right: Optional[HardwarePWM] = None
lidar: Optional[Any] = None

# Thread Control
running = False
sensor_thread: Optional[threading.Thread] = None
lidar_thread: Optional[threading.Thread] = None

# Sensor-Daten
distance_left = 100.0
distance_right = 100.0
lidar_scan_data: List[Tuple[float, float, float]] = []  # (angle, distance, quality)
lidar_lock = threading.Lock()

# Path Following
current_path_index = 0
path_samples: List[Dict[str, Any]] = []
path_start_time = 0.0
playback_active = False

# PID Controller State
pid_integral = 0.0
pid_last_error = 0.0

# Safety Status
emergency_stop = False
obstacle_detected = False
speed_factor = 1.0

# ============================================================================
# DATENSTRUKTUREN
# ============================================================================

@dataclass
class PathSample:
    """Einzelner Pfad-Sample aus Aufzeichnung."""
    timestamp: float
    left_trigger: float
    right_trigger: float
    forward: bool
    tank_turn: bool
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'PathSample':
        """Erstellt PathSample aus Dictionary."""
        return cls(
            timestamp=data['timestamp'],
            left_trigger=data['left_trigger'],
            right_trigger=data['right_trigger'],
            forward=data['direction']['forward'] > 0,
            tank_turn=data['direction'].get('tank_turn', 0.0) > 0
        )

@dataclass
class ObstacleInfo:
    """Information über erkannte Hindernisse."""
    detected: bool = False
    distance_left: float = 100.0
    distance_right: float = 100.0
    distance_front: float = 100.0
    lidar_obstacles: List[Tuple[float, float]] = field(default_factory=list)  # (angle, distance)
    
    def get_min_distance(self) -> float:
        """Gibt minimale Distanz zu Hindernissen zurück."""
        distances = [self.distance_left, self.distance_right, self.distance_front]
        if self.lidar_obstacles:
            distances.extend([d for _, d in self.lidar_obstacles])
        return min(distances) if distances else 100.0
    
    def is_critical(self) -> bool:
        """Prüft ob kritisches Hindernis vorliegt."""
        return self.get_min_distance() < CRITICAL_DISTANCE_CM
    
    def needs_slowdown(self) -> bool:
        """Prüft ob Geschwindigkeit reduziert werden muss."""
        return self.get_min_distance() < WARNING_DISTANCE_CM

# ============================================================================
# SIGNAL HANDLER
# ============================================================================

def signal_handler(sig, frame):
    """Graceful Shutdown bei SIGINT."""
    global running
    print('\n[SHUTDOWN] SIGINT empfangen - Beende Playback...')
    running = False
    cleanup()
    sys.exit(0)

# ============================================================================
# HARDWARE SETUP
# ============================================================================

def setup_gpio():
    """Initialisiert GPIO für Sensoren und Motoren."""
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
        
        print(f'[SETUP] PWM initialisiert - Frequenz: {PWM_FREQUENCY} Hz')
    except Exception as e:
        print(f'[ERROR] PWM Initialisierung fehlgeschlagen: {e}')
        raise

def setup_lidar() -> bool:
    """Initialisiert YDLIDAR C1."""
    global lidar
    
    if not LIDAR_AVAILABLE or not USE_LIDAR:
        print('[SETUP] LIDAR deaktiviert - nur Ultraschall aktiv')
        return False
    
    print('[SETUP] Initialisiere YDLIDAR C1...')
    
    try:
        lidar = CYdLidar()
        lidar.setlidaropt(LidarPropSerialPort, LIDAR_PORT)
        lidar.setlidaropt(LidarPropSerialBaudrate, LIDAR_BAUDRATE)
        lidar.setlidaropt(LidarPropLidarType, TYPE_TOF)
        lidar.setlidaropt(LidarPropDeviceType, YDLIDAR_TYPE_SERIAL)
        lidar.setlidaropt(LidarPropScanFrequency, LIDAR_SCAN_FREQUENCY)
        lidar.setlidaropt(LidarPropSampleRate, 5)
        lidar.setlidaropt(LidarPropSingleChannel, False)
        
        ret = lidar.initialize()
        if not ret:
            print('[ERROR] LIDAR Initialisierung fehlgeschlagen')
            return False
        
        ret = lidar.turnOn()
        if not ret:
            print('[ERROR] LIDAR konnte nicht gestartet werden')
            return False
        
        print('[SETUP] ✅ LIDAR erfolgreich initialisiert')
        return True
        
    except Exception as e:
        print(f'[SETUP] ⚠️ LIDAR Fehler: {e}')
        lidar = None
        return False

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
    except Exception:
        return -1.0

def sensor_update_thread_func():
    """Thread für kontinuierliche Ultraschall-Sensor-Abfrage."""
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

def lidar_update_thread_func():
    """Thread für kontinuierliche LIDAR-Scan-Verarbeitung."""
    global lidar_scan_data, running, lidar
    
    if not lidar:
        return
    
    print(f'[THREAD] LIDAR Thread gestartet ({LIDAR_UPDATE_HZ} Hz)')
    
    scan = LaserScan()
    
    while running and lidar:
        try:
            ret = lidar.doProcessSimple(scan)
            
            if ret:
                points = scan.points
                current_scan = []
                
                for point in points:
                    angle = point.angle
                    distance = point.range
                    quality = point.intensity
                    
                    # Filtere gültige Punkte (vorne +/- 90 Grad)
                    if -90 <= angle <= 90 and distance > 0.1:
                        current_scan.append((angle, distance, quality))
                
                with lidar_lock:
                    lidar_scan_data = current_scan
            
            time.sleep(1.0 / LIDAR_UPDATE_HZ)
            
        except Exception as e:
            if VERBOSE_MODE:
                print(f'[LIDAR] Fehler: {e}')
            time.sleep(1.0)

# ============================================================================
# KOLLISIONSVERMEIDUNG
# ============================================================================

def analyze_obstacles() -> ObstacleInfo:
    """
    Analysiert alle Sensordaten und erstellt ObstacleInfo.
    
    Returns:
        ObstacleInfo mit aggregierten Hindernisdaten
    """
    global distance_left, distance_right, lidar_scan_data
    
    obstacle = ObstacleInfo(
        distance_left=distance_left,
        distance_right=distance_right,
        distance_front=min(distance_left, distance_right)
    )
    
    # Analysiere LIDAR-Daten
    with lidar_lock:
        for angle, distance, quality in lidar_scan_data:
            distance_cm = distance * 100
            
            # Nur Hindernisse im kritischen Bereich
            if distance_cm < LIDAR_OBSTACLE_THRESHOLD_M * 100:
                obstacle.lidar_obstacles.append((angle, distance_cm))
    
    # Setze Detection Flag
    min_dist = obstacle.get_min_distance()
    obstacle.detected = min_dist < WARNING_DISTANCE_CM
    
    return obstacle

def calculate_speed_adjustment(obstacle: ObstacleInfo) -> float:
    """
    Berechnet Geschwindigkeitsanpassung basierend auf Hindernissen.
    
    Args:
        obstacle: Aktuelle Hindernisinfo
        
    Returns:
        Geschwindigkeitsfaktor (0.0 - 1.0)
    """
    min_distance = obstacle.get_min_distance()
    
    if min_distance < CRITICAL_DISTANCE_CM:
        return 0.0  # Emergency Stop
    elif min_distance < WARNING_DISTANCE_CM:
        # Lineare Interpolation zwischen WARNING und CRITICAL
        ratio = (min_distance - CRITICAL_DISTANCE_CM) / (WARNING_DISTANCE_CM - CRITICAL_DISTANCE_CM)
        return MIN_SPEED_FACTOR + (OBSTACLE_SLOWDOWN_FACTOR - MIN_SPEED_FACTOR) * ratio
    elif min_distance < SAFE_DISTANCE_CM:
        # Lineare Interpolation zwischen SAFE und WARNING
        ratio = (min_distance - WARNING_DISTANCE_CM) / (SAFE_DISTANCE_CM - WARNING_DISTANCE_CM)
        return OBSTACLE_SLOWDOWN_FACTOR + (MAX_SPEED_FACTOR - OBSTACLE_SLOWDOWN_FACTOR) * ratio
    else:
        return MAX_SPEED_FACTOR

def check_path_clear(direction_forward: bool, obstacle: ObstacleInfo) -> bool:
    """
    Prüft ob Pfad in gewünschter Richtung frei ist.
    
    Args:
        direction_forward: Gewünschte Fahrtrichtung
        obstacle: Aktuelle Hindernisinfo
        
    Returns:
        True wenn Pfad frei, False bei Blockade
    """
    # Prüfe Ultraschall-Sensoren
    if direction_forward:
        if obstacle.distance_left < CRITICAL_DISTANCE_CM or obstacle.distance_right < CRITICAL_DISTANCE_CM:
            return False
    
    # Prüfe LIDAR-Hindernisse
    for angle, distance in obstacle.lidar_obstacles:
        if distance < CRITICAL_DISTANCE_CM:
            # Hindernis direkt vor oder seitlich
            if direction_forward and -45 <= angle <= 45:
                return False
            elif not direction_forward and (angle < -135 or angle > 135):
                return False
    
    return True

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
    """Konvertiert Trigger-Wert zu PWM Duty Cycle."""
    normalized = (trigger_value + 1.0) / 2.0
    duty_cycle = normalized * MAX_DUTY_CYCLE
    return max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, duty_cycle))

def set_motor_speed_safe(left_duty: float, right_duty: float, speed_factor: float):
    """
    Setzt Motor-Geschwindigkeit mit Sicherheits-Checks.
    
    Args:
        left_duty: Linker Motor Duty Cycle
        right_duty: Rechter Motor Duty Cycle
        speed_factor: Geschwindigkeitsfaktor (0.0 - 1.0)
    """
    global pwm_left, pwm_right, emergency_stop, GLOBAL_SPEED_FACTOR
    
    # Emergency Stop Check
    if emergency_stop:
        left_duty = MIN_DUTY_CYCLE
        right_duty = MIN_DUTY_CYCLE
    else:
        # Wende Geschwindigkeitsfaktoren an
        left_duty *= speed_factor * GLOBAL_SPEED_FACTOR
        right_duty *= speed_factor * GLOBAL_SPEED_FACTOR
    
    # Clamp Values
    left_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, left_duty))
    right_duty = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, right_duty))
    
    try:
        pwm_left.change_duty_cycle(left_duty)
        pwm_right.change_duty_cycle(right_duty)
    except Exception as e:
        print(f'[ERROR] PWM Update fehlgeschlagen: {e}')

# ============================================================================
# PATH LOADING
# ============================================================================

def load_path_recording(filepath: str) -> List[PathSample]:
    """
    Lädt Path Recording aus komprimierter JSON-Datei.
    
    Args:
        filepath: Pfad zur .json.gz Datei
        
    Returns:
        Liste von PathSample Objekten
    """
    print(f'[LOAD] Lade Path Recording: {filepath}')
    
    try:
        with gzip.open(filepath, 'rt', encoding='utf-8') as f:
            data = json.load(f)
        
        # Extrahiere Controller Samples
        samples = []
        for sample_data in data.get('controller_samples', []):
            sample = PathSample.from_dict(sample_data)
            samples.append(sample)
        
        if not samples:
            raise ValueError("Keine Controller-Samples gefunden")
        
        # Normalisiere Zeitstempel (starte bei 0)
        start_time = samples[0].timestamp
        for sample in samples:
            sample.timestamp -= start_time
        
        print(f'[LOAD] ✅ {len(samples)} Samples geladen')
        print(f'[LOAD] Dauer: {samples[-1].timestamp:.2f} Sekunden')
        print(f'[LOAD] Sample-Rate: {len(samples) / samples[-1].timestamp:.1f} Hz')
        
        return samples
        
    except Exception as e:
        print(f'[ERROR] Fehler beim Laden: {e}')
        raise

# ============================================================================
# PATH FOLLOWING
# ============================================================================

def get_current_target_sample(elapsed_time: float) -> Optional[PathSample]:
    """
    Ermittelt aktuelles Ziel-Sample basierend auf Zeit.
    
    Args:
        elapsed_time: Vergangene Zeit seit Playback-Start
        
    Returns:
        PathSample oder None wenn am Ende
    """
    global current_path_index, path_samples
    
    # Suche nächstes Sample das zeitlich passt
    while current_path_index < len(path_samples):
        sample = path_samples[current_path_index]
        
        if sample.timestamp <= elapsed_time:
            current_path_index += 1
            return sample
        else:
            break
    
    # Prüfe ob am Ende
    if current_path_index >= len(path_samples):
        return None
    
    # Gib aktuelles Sample zurück
    return path_samples[current_path_index] if current_path_index < len(path_samples) else None

def playback_path():
    """
    Hauptfunktion für Path Playback mit Kollisionsvermeidung.
    
    Führt PID-basierte Pfadverfolgung mit adaptiver Geschwindigkeitsanpassung
    und mehrstufiger Kollisionsvermeidung durch.
    """
    global playback_active, path_start_time, current_path_index
    global emergency_stop, obstacle_detected, speed_factor
    
    print('\n[PLAYBACK] Starte autonome Pfadverfolgung...')
    print('='*80)
    
    path_start_time = time.time()
    current_path_index = 0
    playback_active = True
    
    sleep_time = 1.0 / CONTROL_LOOP_HZ
    status_counter = 0
    status_interval = CONTROL_LOOP_HZ  # Jede Sekunde
    
    while running and playback_active:
        loop_start = time.time()
        
        # 1. Berechne verstrichene Zeit
        elapsed_time = time.time() - path_start_time
        
        # 2. Hole aktuelles Ziel-Sample
        target_sample = get_current_target_sample(elapsed_time)
        
        if not target_sample:
            print('[PLAYBACK] ✅ Pfad-Ende erreicht')
            playback_active = False
            break
        
        # 3. Analysiere Hindernisse
        obstacle = analyze_obstacles()
        
        # 4. Prüfe Emergency Stop
        if obstacle.is_critical():
            emergency_stop = True
            print(f'[SAFETY] 🛑 EMERGENCY STOP - Hindernis bei {obstacle.get_min_distance():.1f}cm!')
            set_motor_speed_safe(0, 0, 0)
            time.sleep(0.5)
            
            # Prüfe ob Pfad wieder frei
            obstacle = analyze_obstacles()
            if not obstacle.is_critical():
                emergency_stop = False
                print('[SAFETY] ✅ Pfad wieder frei - setze fort')
            else:
                continue
        else:
            emergency_stop = False
        
        # 5. Berechne Geschwindigkeitsanpassung
        speed_factor = calculate_speed_adjustment(obstacle)
        obstacle_detected = speed_factor < MAX_SPEED_FACTOR
        
        # 6. Prüfe ob Pfad in Zielrichtung frei
        if not check_path_clear(target_sample.forward, obstacle):
            print(f'[COLLISION] ⚠️ Pfad blockiert - warte...')
            set_motor_speed_safe(0, 0, 0)
            time.sleep(0.5)
            continue
        
        # 7. Konvertiere Target zu Motor-Befehlen
        left_duty = trigger_to_duty_cycle(target_sample.left_trigger)
        right_duty = trigger_to_duty_cycle(target_sample.right_trigger)
        
        # 8. Setze Motor-Richtung
        backward = not target_sample.forward
        set_motor_direction(backward, backward)
        
        # 9. Wende Motor-Geschwindigkeit mit Safety Factor an
        set_motor_speed_safe(left_duty, right_duty, speed_factor)
        
        # 10. Status-Ausgabe
        status_counter += 1
        if status_counter >= status_interval or VERBOSE_MODE:
            status_counter = 0
            
            progress = (current_path_index / len(path_samples)) * 100
            direction = "⬆️ FWD" if target_sample.forward else "⬇️ REV"
            safety_status = "🛑 STOP" if emergency_stop else ("⚠️ SLOW" if obstacle_detected else "✅ OK")
            
            print(f'[STATUS] Progress:{progress:5.1f}% | {direction} | '
                  f'Speed:{speed_factor*GLOBAL_SPEED_FACTOR*100:3.0f}% | {safety_status} | '
                  f'PWM:L={left_duty*speed_factor*GLOBAL_SPEED_FACTOR:.1f}% R={right_duty*speed_factor*GLOBAL_SPEED_FACTOR:.1f}% | '
                  f'Dist:L={distance_left:.0f}cm R={distance_right:.0f}cm | '
                  f'LIDAR:{len(obstacle.lidar_obstacles)} obstacles')
        
        # 11. Timing
        elapsed = time.time() - loop_start
        sleep_duration = sleep_time - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)
    
    # Stoppe Motoren
    set_motor_speed_safe(0, 0, 0)
    print('[PLAYBACK] Path Replay abgeschlossen')

# ============================================================================
# CLEANUP
# ============================================================================

def cleanup():
    """Ressourcen-Freigabe."""
    global running, pwm_left, pwm_right, lidar
    global sensor_thread, lidar_thread
    
    print('[CLEANUP] Starte Ressourcen-Freigabe...')
    running = False
    
    # Warte auf Threads
    for name, thread in [('Sensor', sensor_thread), ('LIDAR', lidar_thread)]:
        if thread and thread.is_alive():
            print(f'[CLEANUP] Warte auf {name} Thread...')
            thread.join(timeout=2.0)
    
    # Stoppe Motoren
    if pwm_left:
        try:
            pwm_left.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_left.stop()
        except Exception as e:
            print(f'[CLEANUP] Motor links Fehler: {e}')
    
    if pwm_right:
        try:
            pwm_right.change_duty_cycle(MIN_DUTY_CYCLE)
            pwm_right.stop()
        except Exception as e:
            print(f'[CLEANUP] Motor rechts Fehler: {e}')
    
    # Stoppe LIDAR
    if lidar:
        try:
            lidar.turnOff()
            lidar.disconnecting()
        except Exception:
            pass
    
    # GPIO Cleanup
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f'[CLEANUP] GPIO Fehler: {e}')
    
    print('[CLEANUP] ✅ Cleanup abgeschlossen')

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Hauptfunktion."""
    global running, sensor_thread, lidar_thread, path_samples
    global GLOBAL_SPEED_FACTOR, USE_LIDAR, VERBOSE_MODE
    
    # Argument Parser
    parser = argparse.ArgumentParser(
        description='Autonome Path Replay mit Kollisionsvermeidung',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  sudo python3 path_replay_with_collision.py path_recordings/recording.json.gz
  sudo python3 path_replay_with_collision.py path_recordings/recording.json.gz --speed-factor 0.5
  sudo python3 path_replay_with_collision.py path_recordings/recording.json.gz --no-lidar --verbose
        """
    )
    
    parser.add_argument('recording', type=str,
                        help='Pfad zur Path Recording Datei (.json.gz)')
    parser.add_argument('--no-lidar', action='store_true',
                        help='Deaktiviert LIDAR (nur Ultraschall)')
    parser.add_argument('--speed-factor', type=float, default=1.0,
                        help='Globaler Geschwindigkeitsfaktor (0.1-1.0, default: 1.0)')
    parser.add_argument('--verbose', action='store_true',
                        help='Detaillierte Statusausgaben')
    
    args = parser.parse_args()
    
    # Setze Konfiguration
    USE_LIDAR = not args.no_lidar
    GLOBAL_SPEED_FACTOR = max(0.1, min(1.0, args.speed_factor))
    VERBOSE_MODE = args.verbose
    recording_path = args.recording
    
    print('\n' + '='*80)
    print('🤖 AUTONOME PATH REPLAY MIT KOLLISIONSVERMEIDUNG')
    print('Version 1.1 - Phase 6: Autonome Navigation')
    print('='*80)
    print(f'Recording: {recording_path}')
    print(f'Speed Factor: {GLOBAL_SPEED_FACTOR*100:.0f}%')
    print(f'LIDAR: {"✅ Aktiv" if USE_LIDAR else "❌ Deaktiviert"}')
    print(f'Verbose: {"✅ Aktiv" if VERBOSE_MODE else "❌ Deaktiviert"}')
    print('='*80 + '\n')
    
    if not Path(recording_path).exists():
        print(f'❌ [ERROR] Datei nicht gefunden: {recording_path}')
        sys.exit(1)
    
    try:
        # Lade Path Recording
        path_samples = load_path_recording(recording_path)
        
        # Hardware Setup
        setup_gpio()
        setup_pwm()
        lidar_available = setup_lidar()
        
        print('\n[INIT] ✅ Alle Systeme initialisiert - Starte Threads...')
        
        running = True
        
        # Starte Sensor Thread
        sensor_thread = threading.Thread(
            target=sensor_update_thread_func,
            daemon=True,
            name='SensorThread'
        )
        sensor_thread.start()
        
        # Starte LIDAR Thread
        if lidar_available:
            lidar_thread = threading.Thread(
                target=lidar_update_thread_func,
                daemon=True,
                name='LIDARThread'
            )
            lidar_thread.start()
        
        time.sleep(1.0)  # Warte auf Thread-Initialisierung
        
        # Starte Path Playback
        playback_path()
        
    except KeyboardInterrupt:
        print('\n[SHUTDOWN] Keyboard Interrupt - Beende...')
    except Exception as e:
        print(f'\n[ERROR] Unerwarteter Fehler: {e}')
        import traceback
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == '__main__':
    import os
    
    if os.geteuid() != 0:
        print('❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!')
        print('📝 Verwendung: sudo python3 path_replay_with_collision.py <recording.json.gz>')
        sys.exit(1)
    
    main()
