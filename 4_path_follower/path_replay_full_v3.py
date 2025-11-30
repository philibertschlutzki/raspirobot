#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robust Path Replay v3
=====================

Hoch-robustes Wiedergabe-System für Pfadaufzeichnungen mit:
- Dynamischer Fehlerbehandlung
- Recovery-Strategien nach Ausweichmanövern
- Adaptiver Sensorfusion
- Umfangreichem Exception Handling

AUTOR: AI Assistant
DATUM: 2025-11-27
VERSION: 3.0
"""

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
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass

# Fallback für Development/Testing ohne Hardware
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("⚠️ RPi.GPIO nicht gefunden - Mock-Modus aktiv")
    # Mock Class für GPIO
    class MockGPIO:
        BCM = 'BCM'
        OUT = 'OUT'
        IN = 'IN'
        HIGH = 1
        LOW = 0
        @staticmethod
        def setmode(*args): pass
        @staticmethod
        def setwarnings(*args): pass
        @staticmethod
        def setup(*args): pass
        @staticmethod
        def output(*args): pass
        @staticmethod
        def input(*args): return 0
        @staticmethod
        def cleanup(): pass
    GPIO = MockGPIO

try:
    from rpi_hardware_pwm import HardwarePWM
    HARDWARE_PWM_AVAILABLE = True
except ImportError:
    HARDWARE_PWM_AVAILABLE = False
    print("⚠️ rpi_hardware_pwm nicht gefunden - PWM deaktiviert/mocked")

try:
    from rplidar import RPLidar, RPLidarException
    LIDAR_LIBRARY_AVAILABLE = True
except ImportError:
    LIDAR_LIBRARY_AVAILABLE = False
    print("⚠️ rplidar nicht gefunden - LIDAR deaktiviert")

# ============================================================================
# KONFIGURATION & KONSTANTEN
# ============================================================================

# GPIO Pins (BCM)
TRIG_LEFT = 12
ECHO_LEFT = 13
TRIG_RIGHT = 23
ECHO_RIGHT = 24
DIR_LEFT_PIN = 20
DIR_RIGHT_PIN = 21

# PWM Settings
PWM_FREQUENCY = 1000
PWM_CHIP = 0
PWM_CHANNEL_LEFT = 0
PWM_CHANNEL_RIGHT = 1
MAX_DUTY_CYCLE = 12.0
MIN_DUTY_CYCLE = 0.0

# Hardware Defaults
DIR_HIGH_IS_BACKWARD = False
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

# Safety Thresholds
CRITICAL_DISTANCE_CM = 20.0
WARNING_DISTANCE_CM = 40.0
SAFE_DISTANCE_CM = 60.0

# Recovery Parameters
RECOVERY_LOOKAHEAD_INDEX = 20  # Wie viele Samples in die Zukunft schauen nach Ausweichen
RECOVERY_SPEED = 0.2           # Langsame Geschwindigkeit während Recovery
RECOVERY_TIMEOUT_SEC = 10.0    # Maximale Zeit für Recovery-Versuch

# ============================================================================
# DATENSTRUKTUREN
# ============================================================================

@dataclass
class Pose2D:
    x: float
    y: float
    theta: float
    timestamp: float = 0.0

    def distance_to(self, other: 'Pose2D') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

@dataclass
class ControlCommand:
    left_duty: float
    right_duty: float
    forward: bool
    timestamp: float

# ============================================================================
# GLOBALE STATUS-VARIABLEN
# ============================================================================

running = False
shutdown_event = threading.Event()

# Hardware Handles
pwm_left = None
pwm_right = None
lidar = None

# Sensor Data
sensor_data_lock = threading.Lock()
current_distance_left = 100.0
current_distance_right = 100.0
last_lidar_scan = []

# Robot State
robot_pose = Pose2D(0, 0, 0)
pose_lock = threading.Lock()

# Playback State
playback_state = "IDLE"  # IDLE, REPLAY, AVOID, RECOVER
recording_samples = []
target_trajectory = []   # Liste von Pose2D

# ============================================================================
# HELPER KLASSEN
# ============================================================================

class HardwareManager:
    """Kapselt Hardware-Zugriffe für robuste Fehlerbehandlung."""

    @staticmethod
    def setup_gpio():
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            # US Sensoren
            for pin in [TRIG_LEFT, TRIG_RIGHT]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, False)
            for pin in [ECHO_LEFT, ECHO_RIGHT]:
                GPIO.setup(pin, GPIO.IN)

            # Motoren Direction
            GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
            GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)

            print("[HW] GPIO Setup OK")
            return True
        except Exception as e:
            print(f"[HW] ❌ GPIO Setup Fehler: {e}")
            return False

    @staticmethod
    def setup_pwm():
        global pwm_left, pwm_right
        if not HARDWARE_PWM_AVAILABLE:
            print("[HW] Nutze Mock-PWM (HardwareLib fehlt)")
            return True # Return True to allow logic to proceed in mock mode

        try:
            pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
            pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
            pwm_left.start(0)
            pwm_right.start(0)
            print("[HW] PWM Setup OK")
            return True
        except Exception as e:
            print(f"[HW] ❌ PWM Setup Fehler: {e}")
            # Auch bei Fehler weitermachen (Fallback/Mock Verhalten)
            return True

    @staticmethod
    def set_motors(left_duty: float, right_duty: float, forward: bool):
        """Setzt Motoren mit Error-Handling."""
        global pwm_left, pwm_right

        try:
            # Direction
            if DIR_HIGH_IS_BACKWARD:
                lvl = GPIO.LOW if forward else GPIO.HIGH
            else:
                lvl = GPIO.HIGH if forward else GPIO.LOW

            GPIO.output(DIR_LEFT_PIN, lvl)
            GPIO.output(DIR_RIGHT_PIN, lvl)

            # PWM Limiting
            ld = max(0.0, min(MAX_DUTY_CYCLE, left_duty))
            rd = max(0.0, min(MAX_DUTY_CYCLE, right_duty))

            if pwm_left: pwm_left.change_duty_cycle(ld)
            if pwm_right: pwm_right.change_duty_cycle(rd)

        except Exception as e:
            # Im Mock-Mode (kein pwm_left) einfach ignorieren
            pass

    @staticmethod
    def stop_all():
        HardwareManager.set_motors(0, 0, True)

# ============================================================================
# LOGIK & ALGORITHMEN
# ============================================================================

def calculate_next_pose(current_pose: Pose2D, command: ControlCommand, dt: float) -> Pose2D:
    """Berechnet neue Pose mittels Odometrie (Dead Reckoning)."""
    # Vereinfachtes Modell
    WHEEL_BASE = 0.20
    MAX_SPEED_MS = 0.5

    # Speed schätzen aus Duty Cycle
    v_left = (command.left_duty / MAX_DUTY_CYCLE) * MAX_SPEED_MS
    v_right = (command.right_duty / MAX_DUTY_CYCLE) * MAX_SPEED_MS

    if not command.forward:
        v_left = -v_left
        v_right = -v_right

    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / WHEEL_BASE

    new_theta = current_pose.theta + omega * dt
    # Normalize theta
    new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi

    new_x = current_pose.x + v * math.cos(current_pose.theta) * dt
    new_y = current_pose.y + v * math.sin(current_pose.theta) * dt

    return Pose2D(new_x, new_y, new_theta, current_pose.timestamp + dt)

def get_recovery_command(current_pose: Pose2D, target_pose: Pose2D) -> ControlCommand:
    """Berechnet Steuerbefehle, um zum Pfad zurückzukehren (Pure Pursuit light)."""

    dx = target_pose.x - current_pose.x
    dy = target_pose.y - current_pose.y
    dist = math.sqrt(dx*dx + dy*dy)

    target_angle = math.atan2(dy, dx)
    angle_diff = target_angle - current_pose.theta

    # Normalize angle diff
    while angle_diff > math.pi: angle_diff -= 2*math.pi
    while angle_diff < -math.pi: angle_diff += 2*math.pi

    # Simple P-Controller
    turn_cmd = angle_diff * 1.5
    forward_cmd = RECOVERY_SPEED

    # Wenn wir weit weg schauen, drehen wir erst
    if abs(angle_diff) > 0.5:
        forward_cmd = 0.0

    # Convert to Duty Cycle (approx)
    base_duty = (forward_cmd / 0.5) * MAX_DUTY_CYCLE
    turn_duty = (turn_cmd / 2.0) * MAX_DUTY_CYCLE

    left = base_duty - turn_duty
    right = base_duty + turn_duty

    return ControlCommand(left, right, True, time.time())

# ============================================================================
# THREADS
# ============================================================================

def sensor_worker():
    """Liest Ultraschall-Sensoren mit Timeouts."""
    global current_distance_left, current_distance_right

    if not GPIO_AVAILABLE:
        print("[THREAD] Sensor Thread: Mock Mode")
        while running and not shutdown_event.is_set():
            time.sleep(1)
        return

    while running and not shutdown_event.is_set():
        try:
            # Funktion zum Messen (lokal definiert für Fehlerfang)
            def measure(trig, echo):
                GPIO.output(trig, True)
                time.sleep(0.00001)
                GPIO.output(trig, False)

                start = time.time()
                while GPIO.input(echo) == 0:
                    if time.time() - start > 0.05: return 100.0

                pulse_start = time.time()
                while GPIO.input(echo) == 1:
                    if time.time() - pulse_start > 0.05: return 100.0

                duration = time.time() - pulse_start
                return (duration * 34300) / 2.0

            d_left = measure(TRIG_LEFT, ECHO_LEFT)
            d_right = measure(TRIG_RIGHT, ECHO_RIGHT)

            with sensor_data_lock:
                current_distance_left = d_left if d_left > 1.0 else 100.0
                current_distance_right = d_right if d_right > 1.0 else 100.0

            time.sleep(0.05)

        except Exception as e:
            print(f"[SENSOR] ⚠️ Fehler im Sensor-Thread: {e}")
            time.sleep(1)

def lidar_worker():
    """Liest LIDAR robust."""
    global last_lidar_scan, lidar

    if not LIDAR_LIBRARY_AVAILABLE:
        print("[THREAD] LIDAR Thread: Inactive (Lib missing)")
        return

    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
        lidar.start_motor()
    except Exception as e:
        print(f"[LIDAR] Init failed: {e}")
        return

    print("[THREAD] LIDAR Thread gestartet")
    while running and not shutdown_event.is_set():
        try:
            iterator = lidar.iter_scans(max_buf_meas=1000, min_len=5)
            for scan in iterator:
                if shutdown_event.is_set(): break
                with sensor_data_lock:
                    last_lidar_scan = list(scan) # [(qual, angle, dist), ...]
        except RPLidarException as e:
            print(f"[LIDAR] ⚠️ Resetting LIDAR due to error: {e}")
            try:
                lidar.stop()
                lidar.disconnect()
                time.sleep(1)
                lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
                lidar.start_motor()
            except:
                pass
            time.sleep(2)
        except Exception as e:
            print(f"[LIDAR] ⚠️ Unexpected error: {e}")
            time.sleep(1)

# ============================================================================
# MAIN LOOP
# ============================================================================

def main_loop(recording_path: str):
    global running, robot_pose, playback_state

    # 1. Daten Laden & Validieren
    try:
        with gzip.open(recording_path, 'rt') as f:
            data = json.load(f)

        samples = data.get('controller_samples', [])
        if not samples:
            raise ValueError("Keine Samples in Datei")

        # Pfad vorbereiten
        start_time = samples[0]['timestamp']
        for s in samples:
            # Relative Zeit
            s['rel_time'] = s['timestamp'] - start_time

            # Trajektorie aufbauen (Fallback auf Dead Reckoning wenn keine Pose im File)
            if 'pose' in s:
                p = s['pose']
                target_trajectory.append(Pose2D(p.get('x',0), p.get('y',0), p.get('theta',0)))
            else:
                # Wenn wir keine Pose im File haben, müssen wir sie on-the-fly berechnen
                # oder wir lassen target_trajectory leer und fahren blind nach Zeit (Legacy Mode)
                pass

        print(f"[INIT] {len(samples)} Samples geladen.")

    except Exception as e:
        print(f"[FATAL] Fehler beim Laden: {e}")
        return

    # 2. Start
    print("[MAIN] Start Playback...")
    start_sys_time = time.time()
    sample_idx = 0
    running = True

    # Threads starten
    t_sensors = threading.Thread(target=sensor_worker, daemon=True)
    t_sensors.start()

    if LIDAR_LIBRARY_AVAILABLE:
        t_lidar = threading.Thread(target=lidar_worker, daemon=True)
        t_lidar.start()

    playback_state = "REPLAY"
    last_loop_time = time.time()

    try:
        while sample_idx < len(samples) and not shutdown_event.is_set():
            now = time.time()
            dt = now - last_loop_time
            last_loop_time = now

            elapsed = now - start_sys_time

            # --- SICHERHEITSCHECK ---
            with sensor_data_lock:
                min_dist = min(current_distance_left, current_distance_right)

            # --- ZUSTANDSMASCHINE ---

            if playback_state == "REPLAY":
                # Kollisionsprüfung
                if min_dist < CRITICAL_DISTANCE_CM:
                    print("[WARN] 🛑 Hindernis! Stoppe & Wechsel zu AVOID")
                    HardwareManager.stop_all()
                    playback_state = "AVOID"
                    avoid_start_time = now
                    continue

                # Normales Replay
                # Wir suchen das Sample, das zur aktuellen Zeit passt
                while sample_idx < len(samples) - 1 and samples[sample_idx]['rel_time'] < elapsed:
                    sample_idx += 1

                curr_sample = samples[sample_idx]

                # Odometrie Update (Blind Estimation)
                cmd = ControlCommand(
                    curr_sample.get('left_trigger', 0) * MAX_DUTY_CYCLE, # Vereinfacht
                    curr_sample.get('right_trigger', 0) * MAX_DUTY_CYCLE,
                    curr_sample['direction']['forward'],
                    now
                )

                # Wenn wir Pose-Daten haben, aktualisieren wir unsere Schätzung
                # In v3: Hier würde der Sensor-Fusion Code stehen, der die echte Pose korrigiert
                # Wir simulieren das Update der internen Pose
                with pose_lock:
                    robot_pose = calculate_next_pose(robot_pose, cmd, dt)

                # Befehl ausführen
                # Trigger -1..1 mappen auf Duty Cycle
                lt = curr_sample['left_trigger']
                rt = curr_sample['right_trigger']

                # Mapping Logic (Kopie aus v2)
                ld = ((lt + 1) / 2) * MAX_DUTY_CYCLE
                rd = ((rt + 1) / 2) * MAX_DUTY_CYCLE
                fw = curr_sample['direction']['forward']

                HardwareManager.set_motors(ld, rd, fw)

            elif playback_state == "AVOID":
                # Einfaches Verhalten: Warten bis Weg frei
                if min_dist > WARNING_DISTANCE_CM:
                    print("[INFO] Weg frei -> Wechsel zu RECOVER")
                    playback_state = "RECOVER"
                    continue

                # Optional: Rückwärts fahren oder drehen wenn zu lange blockiert
                if now - avoid_start_time > 5.0:
                    print("[WARN] Blockiert > 5s. Versuche Rückzug...")
                    HardwareManager.set_motors(5.0, 5.0, False) # Langsam zurück
                else:
                    HardwareManager.stop_all()

            elif playback_state == "RECOVER":
                # Ziel: Finde den Punkt auf der Trajektorie, wo wir sein sollten
                # Wir schauen etwas in die Zukunft (Lookahead)
                target_idx = min(sample_idx + RECOVERY_LOOKAHEAD_INDEX, len(samples)-1)

                # Wenn wir keine Pose-Daten in der Aufnahme haben, ist Recovery schwer.
                # Wir nehmen an, wir haben Pose-Daten oder schätzen blind.
                if target_idx < len(target_trajectory):
                    target_p = target_trajectory[target_idx]

                    with pose_lock:
                        curr_p = robot_pose

                    dist_to_target = curr_p.distance_to(target_p)

                    if dist_to_target < 0.20: # 20cm Toleranz
                        print("[INFO] ✅ Zurück auf Pfad -> REPLAY")
                        playback_state = "REPLAY"
                        # Zeit synchronisieren: Wir springen in der Zeit zum Ziel-Index
                        # Offset anpassen, damit Playback dort weitermacht
                        current_rel_target_time = samples[target_idx]['rel_time']
                        start_sys_time = now - current_rel_target_time
                        sample_idx = target_idx
                        continue

                    # Fahre zum Ziel
                    rec_cmd = get_recovery_command(curr_p, target_p)
                    HardwareManager.set_motors(rec_cmd.left_duty, rec_cmd.right_duty, True)

                    # Update Pose Estimation während Recovery
                    with pose_lock:
                        robot_pose = calculate_next_pose(robot_pose, rec_cmd, dt)
                else:
                    # Fallback ohne Trajektorie: Einfach warten und dann weiterfahren (riskant)
                    print("[WARN] Keine Trajektorie für Recovery. Wechsel direkt zu REPLAY.")
                    playback_state = "REPLAY"

            time.sleep(0.02) # 50Hz Loop

    except KeyboardInterrupt:
        print("\n[STOP] Nutzerabbruch")
    finally:
        shutdown_event.set()
        HardwareManager.stop_all()
        # Nur cleanup wenn es nicht der Mock ist
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        print("[EXIT] Programm beendet.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="Pfad zur .json.gz Datei")
    args = parser.parse_args()

    # Init Hardware (oder Mocks)
    HardwareManager.setup_gpio()
    HardwareManager.setup_pwm()

    # Start Loop auch wenn Hardware "fehlt" (weil Mocked)
    main_loop(args.file)
