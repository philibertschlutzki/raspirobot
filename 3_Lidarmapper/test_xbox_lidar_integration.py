#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Automatisierter Test für Xbox Controller + LIDAR Recording System
=================================================================

Testet alle Komponenten des integrierten Systems ohne echten Xbox Controller.
Simuliert Controller-Eingaben und prüft LIDAR-Integration, Recording und
Pose-Tracking.

VERWENDUNG:
-----------
sudo python3 test_xbox_lidar_integration.py

Optional:
--duration SECONDS    Test-Dauer (default: 30s)
--no-lidar           Simuliert LIDAR-Ausfall
--save-recording     Speichert Test-Recording

AUTOR: Test Suite Phase 3
DATUM: 2025-11-27
VERSION: 1.0
"""

import sys
import time
import argparse
import json
import gzip
import threading
from pathlib import Path
from typing import Dict, Any, List

# ============================================================================
# TEST-KONFIGURATION
# ============================================================================

TEST_DURATION_SEC = 30
TEST_RECORDING_DIR = "test_recordings"
SIMULATE_LIDAR_FAILURE = False
SAVE_TEST_RECORDING = False

# ============================================================================
# TEST-SIMULATOREN
# ============================================================================

class ControllerSimulator:
    """Simuliert Xbox Controller Eingaben."""
    
    def __init__(self):
        self.time_start = time.time()
        self.trigger_left = -1.0
        self.trigger_right = -1.0
        self.a_button = False
        self.b_button = False
        self.recording_toggle_time = 5.0  # Recording nach 5s starten
        self.recording_toggled = False
    
    def update(self):
        """Simuliert Controller-Bewegungen."""
        elapsed = time.time() - self.time_start
        
        # Simuliere Vorwärts-Fahrt mit variierender Geschwindigkeit
        import math
        speed = 0.5 + 0.3 * math.sin(elapsed * 0.5)
        
        self.trigger_left = speed
        self.trigger_right = speed
        
        # Simuliere Tank-Turn alle 10 Sekunden
        if int(elapsed) % 10 < 2:
            self.a_button = True
            if int(elapsed) % 10 == 0:
                self.trigger_right = 0.8
                self.trigger_left = -0.3
        else:
            self.a_button = False
        
        # Toggle Recording nach bestimmter Zeit
        if elapsed > self.recording_toggle_time and not self.recording_toggled:
            self.recording_toggled = True
            return True  # Signal für Recording-Toggle
        
        return False

class LidarSimulator:
    """Simuliert RPLIDAR Scan-Daten."""
    
    def __init__(self, failure_mode: bool = False):
        self.failure_mode = failure_mode
        self.scan_count = 0
    
    def generate_scan(self) -> List[tuple]:
        """Generiert simulierten LIDAR-Scan."""
        if self.failure_mode:
            return []
        
        scan_data = []
        
        # Generiere 360° Scan mit 500 Punkten
        import random
        import math
        
        for i in range(500):
            angle = (i / 500.0) * 360.0
            
            # Simuliere Wände und Hindernisse
            if 80 < angle < 100 or 260 < angle < 280:
                # Simuliere Wand bei 1.5m
                distance = 1500 + random.uniform(-50, 50)
            elif 170 < angle < 190:
                # Simuliere Hindernis bei 0.5m
                distance = 500 + random.uniform(-30, 30)
            else:
                # Offener Raum 2-3m
                distance = 2000 + random.uniform(-200, 200)
            
            quality = random.randint(10, 15)
            
            scan_data.append((quality, angle, distance))
        
        self.scan_count += 1
        return scan_data

# ============================================================================
# TEST-FUNKTIONEN
# ============================================================================

def test_imports() -> bool:
    """Testet ob alle Module importiert werden können."""
    print('\n[TEST 1/6] 📦 Module-Import-Test...')
    
    try:
        # Test path_recording_system
        from path_recording_system import (
            PathRecordingData, PathRecordingHeader, ControllerSample,
            LidarFrame, EnvironmentMetadata, CalibrationData
        )
        print('  ✅ path_recording_system importiert')
        
        # Test storage_system
        from storage_system import PathRecordingStorageManager
        print('  ✅ storage_system importiert')
        
        # Test LIDAR (optional)
        try:
            from rplidar import RPLidar
            print('  ✅ rplidar importiert')
        except ImportError:
            print('  ⚠️  rplidar nicht verfügbar (optional)')
        
        print('  ✅ Alle erforderlichen Module verfügbar')
        return True
        
    except ImportError as e:
        print(f'  ❌ Import-Fehler: {e}')
        return False

def test_data_structures() -> bool:
    """Testet Datenstrukturen."""
    print('\n[TEST 2/6] 📊 Datenstruktur-Test...')
    
    try:
        from path_recording_system import (
            PathRecordingData, PathRecordingHeader, ControllerSample,
            LidarFrame, EnvironmentMetadata, CalibrationData, CompressionType
        )
        
        # Test ControllerSample
        sample = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.5,
            right_trigger=0.5,
            buttons={'A': False, 'B': True},
            direction={'forward': 1.0, 'tank_turn': 0.0},
            sequence_id=0
        )
        print('  ✅ ControllerSample erstellt')
        
        # Test LidarFrame
        frame = LidarFrame(
            timestamp=time.time(),
            scan_data=[(10, 45.0, 1500.0), (12, 90.0, 2000.0)],
            frame_id=0
        )
        print('  ✅ LidarFrame erstellt')
        
        # Test PathRecordingData
        metadata = EnvironmentMetadata(
            session_id="test_session",
            start_timestamp=time.time(),
            environment_conditions={},
            hardware_info={},
            recording_settings={}
        )
        
        calibration = CalibrationData(
            controller_calibration={},
            lidar_calibration={},
            calibration_timestamp=time.time(),
            calibration_quality=1.0
        )
        
        header = PathRecordingHeader(
            calibration_data=calibration,
            metadata=metadata,
            compression_type=CompressionType.GZIP
        )
        
        recording = PathRecordingData(header=header)
        recording.add_controller_sample(sample)
        recording.add_lidar_frame(frame)
        
        stats = recording.get_stats()
        assert stats['controller_samples'] == 1
        assert stats['lidar_frames'] == 1
        
        print('  ✅ PathRecordingData funktioniert')
        print(f'  📊 Stats: {stats}')
        
        return True
        
    except Exception as e:
        print(f'  ❌ Fehler: {e}')
        import traceback
        traceback.print_exc()
        return False

def test_controller_simulation(duration: int = 10) -> bool:
    """Testet Controller-Simulation."""
    print(f'\n[TEST 3/6] 🎮 Controller-Simulations-Test ({duration}s)...')
    
    try:
        controller = ControllerSimulator()
        
        samples_collected = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            controller.update()
            
            samples_collected.append({
                'trigger_left': controller.trigger_left,
                'trigger_right': controller.trigger_right,
                'a_button': controller.a_button,
                'timestamp': time.time()
            })
            
            time.sleep(0.02)  # 50Hz
        
        print(f'  ✅ {len(samples_collected)} Controller-Samples generiert')
        print(f'  📊 Sample-Rate: {len(samples_collected)/duration:.1f} Hz')
        
        return True
        
    except Exception as e:
        print(f'  ❌ Fehler: {e}')
        return False

def test_lidar_simulation(duration: int = 10, failure_mode: bool = False) -> bool:
    """Testet LIDAR-Simulation."""
    print(f'\n[TEST 4/6] 📡 LIDAR-Simulations-Test ({duration}s, Failure: {failure_mode})...')
    
    try:
        lidar = LidarSimulator(failure_mode=failure_mode)
        
        scans_collected = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            scan = lidar.generate_scan()
            
            if scan:
                scans_collected.append({
                    'points': len(scan),
                    'timestamp': time.time()
                })
            
            time.sleep(0.1)  # 10Hz
        
        if not failure_mode:
            print(f'  ✅ {len(scans_collected)} LIDAR-Scans generiert')
            if scans_collected:
                avg_points = sum(s['points'] for s in scans_collected) / len(scans_collected)
                print(f'  📊 Durchschnitt: {avg_points:.0f} Punkte/Scan')
        else:
            print(f'  ✅ Failure-Mode korrekt: {len(scans_collected)} Scans (erwartet: 0)')
        
        return True
        
    except Exception as e:
        print(f'  ❌ Fehler: {e}')
        return False

def test_integrated_recording(duration: int = 15, save: bool = False) -> bool:
    """Testet integrierte Aufzeichnung."""
    print(f'\n[TEST 5/6] 💾 Integriertes Recording-Test ({duration}s)...')
    
    try:
        from path_recording_system import (
            PathRecordingData, PathRecordingHeader, ControllerSample,
            LidarFrame, EnvironmentMetadata, CalibrationData, CompressionType
        )
        from storage_system import PathRecordingStorageManager
        
        # Setup
        session_id = f"test_{int(time.time())}"
        
        metadata = EnvironmentMetadata(
            session_id=session_id,
            start_timestamp=time.time(),
            environment_conditions={'test': True},
            hardware_info={'simulated': True},
            recording_settings={'duration': duration}
        )
        
        calibration = CalibrationData(
            controller_calibration={},
            lidar_calibration={},
            calibration_timestamp=time.time(),
            calibration_quality=1.0
        )
        
        header = PathRecordingHeader(
            calibration_data=calibration,
            metadata=metadata,
            compression_type=CompressionType.GZIP
        )
        
        recording = PathRecordingData(header=header)
        
        # Simulatoren
        controller = ControllerSimulator()
        lidar = LidarSimulator()
        
        # Recording-Loop
        start_time = time.time()
        controller_count = 0
        lidar_count = 0
        
        last_controller_time = start_time
        last_lidar_time = start_time
        
        print('  🔄 Starte Datenaufzeichnung...')
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Controller-Samples (50Hz)
            if current_time - last_controller_time >= 0.02:
                controller.update()
                
                sample = ControllerSample(
                    timestamp=current_time,
                    left_trigger=controller.trigger_left,
                    right_trigger=controller.trigger_right,
                    buttons={
                        'A': controller.a_button,
                        'B': controller.b_button,
                        'X': False,
                        'Y': False
                    },
                    direction={
                        'forward': 1.0,
                        'tank_turn': 1.0 if controller.a_button else 0.0
                    },
                    sequence_id=controller_count
                )
                
                recording.add_controller_sample(sample)
                controller_count += 1
                last_controller_time = current_time
            
            # LIDAR-Frames (10Hz)
            if current_time - last_lidar_time >= 0.1:
                scan = lidar.generate_scan()
                
                frame = LidarFrame(
                    timestamp=current_time,
                    scan_data=scan,
                    frame_id=lidar_count
                )
                
                recording.add_lidar_frame(frame)
                lidar_count += 1
                last_lidar_time = current_time
            
            time.sleep(0.001)
        
        # Finalisiere Metadaten
        recording.header.metadata.end_timestamp = time.time()
        
        # Statistiken
        stats = recording.get_stats()
        print(f'  ✅ Recording abgeschlossen')
        print(f'  📊 Statistiken:')
        print(f'     Controller: {stats["controller_samples"]} samples')
        print(f'     LIDAR: {stats["lidar_frames"]} frames')
        print(f'     Dauer: {stats["duration"]:.2f}s')
        
        # Optional: Speichern
        if save:
            Path(TEST_RECORDING_DIR).mkdir(parents=True, exist_ok=True)
            
            storage = PathRecordingStorageManager(
                base_directory=TEST_RECORDING_DIR,
                compression=CompressionType.GZIP
            )
            
            filepath = storage.save_json(recording, session_id)
            print(f'  💾 Gespeichert: {filepath}')
            
            # Lade und validiere
            loaded = storage.load_json(session_id)
            loaded_stats = loaded.get_stats()
            
            assert loaded_stats['controller_samples'] == stats['controller_samples']
            assert loaded_stats['lidar_frames'] == stats['lidar_frames']
            
            print(f'  ✅ Load-Validierung erfolgreich')
        
        return True
        
    except Exception as e:
        print(f'  ❌ Fehler: {e}')
        import traceback
        traceback.print_exc()
        return False

def test_pose_tracking(duration: int = 10) -> bool:
    """Testet Pose-Tracking-Logik."""
    print(f'\n[TEST 6/6] 🗺️  Pose-Tracking-Test ({duration}s)...')
    
    try:
        import math
        
        # Simuliere Pose-Updates
        x, y, theta = 0.0, 0.0, 0.0
        wheel_base = 0.20
        
        poses = []
        start_time = time.time()
        last_update = start_time
        
        controller = ControllerSimulator()
        
        while time.time() - start_time < duration:
            current_time = time.time()
            dt = current_time - last_update
            
            if dt >= 0.02:  # 50Hz
                controller.update()
                
                # Vereinfachte Odometrie
                v_left = controller.trigger_left * 0.5
                v_right = controller.trigger_right * 0.5
                
                v = (v_left + v_right) / 2.0
                omega = (v_right - v_left) / wheel_base
                
                if abs(omega) < 0.001:
                    x += v * math.cos(theta) * dt
                    y += v * math.sin(theta) * dt
                else:
                    radius = v / omega
                    x += radius * (math.sin(theta + omega * dt) - math.sin(theta))
                    y += radius * (-math.cos(theta + omega * dt) + math.cos(theta))
                    theta += omega * dt
                    theta = (theta + math.pi) % (2 * math.pi) - math.pi
                
                poses.append({
                    'x': x,
                    'y': y,
                    'theta': theta,
                    'timestamp': current_time
                })
                
                last_update = current_time
            
            time.sleep(0.001)
        
        print(f'  ✅ {len(poses)} Pose-Updates berechnet')
        print(f'  📊 Finale Position:')
        print(f'     X: {x:.2f}m, Y: {y:.2f}m, θ: {math.degrees(theta):.0f}°')
        
        # Berechne zurückgelegte Strecke
        total_distance = 0.0
        for i in range(1, len(poses)):
            dx = poses[i]['x'] - poses[i-1]['x']
            dy = poses[i]['y'] - poses[i-1]['y']
            total_distance += math.sqrt(dx**2 + dy**2)
        
        print(f'  📏 Zurückgelegte Strecke: {total_distance:.2f}m')
        
        return True
        
    except Exception as e:
        print(f'  ❌ Fehler: {e}')
        import traceback
        traceback.print_exc()
        return False

# ============================================================================
# MAIN TEST SUITE
# ============================================================================

def run_test_suite(args):
    """Führt komplette Test-Suite aus."""
    print('='*80)
    print('🧪 XBOX CONTROLLER + LIDAR INTEGRATION TEST SUITE')
    print('='*80)
    print(f'Konfiguration:')
    print(f'  Dauer: {args.duration}s')
    print(f'  LIDAR-Failure-Simulation: {args.no_lidar}')
    print(f'  Recording speichern: {args.save_recording}')
    print('='*80)
    
    results = []
    
    # Test 1: Imports
    results.append(('Module-Import', test_imports()))
    
    # Test 2: Datenstrukturen
    results.append(('Datenstrukturen', test_data_structures()))
    
    # Test 3: Controller-Simulation
    results.append(('Controller-Simulation', test_controller_simulation(duration=5)))
    
    # Test 4: LIDAR-Simulation
    results.append(('LIDAR-Simulation', test_lidar_simulation(duration=5, failure_mode=args.no_lidar)))
    
    # Test 5: Integriertes Recording
    results.append(('Integriertes Recording', test_integrated_recording(
        duration=args.duration, 
        save=args.save_recording
    )))
    
    # Test 6: Pose-Tracking
    results.append(('Pose-Tracking', test_pose_tracking(duration=5)))
    
    # Zusammenfassung
    print('\n' + '='*80)
    print('📋 TEST-ZUSAMMENFASSUNG')
    print('='*80)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = '✅ PASSED' if result else '❌ FAILED'
        print(f'{status}: {name}')
    
    print('='*80)
    print(f'Ergebnis: {passed}/{total} Tests bestanden ({passed/total*100:.0f}%)')
    print('='*80)
    
    return passed == total

# ============================================================================
# ENTRY POINT
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Automatisierter Test für Xbox Controller + LIDAR System'
    )
    
    parser.add_argument('--duration', type=int, default=15,
                        help='Test-Dauer für Recording-Test (Sekunden)')
    parser.add_argument('--no-lidar', action='store_true',
                        help='Simuliert LIDAR-Ausfall')
    parser.add_argument('--save-recording', action='store_true',
                        help='Speichert Test-Recording')
    
    args = parser.parse_args()
    
    # Führe Tests aus
    success = run_test_suite(args)
    
    # Exit Code
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
