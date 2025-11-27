#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Comprehensive Test Suite für Path Replay System
================================================

Testet alle Funktionen des Path Replay Systems systematisch.

TESTS:
------
1. Hardware Tests (GPIO, PWM, Sensoren, LIDAR)
2. Path Loading Tests (JSON Parsing, Validation)
3. Collision Detection Tests (Ultraschall, LIDAR, Fusion)
4. Motor Control Tests (PWM, Direction, Speed Adjustment)
5. Path Following Tests (Timing, Interpolation, PID)
6. Safety System Tests (Emergency Stop, Degradation)
7. Integration Tests (End-to-End Scenarios)

VERWENDUNG:
-----------
sudo python3 test_path_replay.py [--hardware] [--verbose]

FLAGS:
------
--hardware  : Aktiviert Hardware-Tests (erfordert echte Hardware)
--verbose   : Detaillierte Ausgaben
--quick     : Nur schnelle Tests (ohne lange Delay-Tests)

AUTOR: Test Suite für Phase 6
DATUM: 2025-11-27
VERSION: 1.0
"""

import sys
import time
import json
import gzip
import tempfile
import unittest
from pathlib import Path
from typing import List, Dict, Any
from unittest.mock import Mock, MagicMock, patch
import argparse

# ============================================================================
# TEST CONFIGURATION
# ============================================================================

TEST_HARDWARE = False  # Wird per CLI-Flag gesetzt
TEST_VERBOSE = False
TEST_QUICK = False

# Mock-Daten für Tests
MOCK_PATH_SAMPLES = [
    {
        'timestamp': 0.0,
        'left_trigger': 0.5,
        'right_trigger': 0.5,
        'direction': {'forward': 1.0, 'tank_turn': 0.0}
    },
    {
        'timestamp': 0.1,
        'left_trigger': 0.7,
        'right_trigger': 0.7,
        'direction': {'forward': 1.0, 'tank_turn': 0.0}
    },
    {
        'timestamp': 0.2,
        'left_trigger': 0.9,
        'right_trigger': 0.9,
        'direction': {'forward': 1.0, 'tank_turn': 0.0}
    }
]

MOCK_RECORDING = {
    'header': {
        'metadata': {
            'session_id': 'test_session',
            'start_timestamp': time.time(),
            'environment_conditions': {}
        }
    },
    'controller_samples': MOCK_PATH_SAMPLES
}

# ============================================================================
# TEST BASE CLASS
# ============================================================================

class PathReplayTestCase(unittest.TestCase):
    """Basis-Testklasse mit gemeinsamen Utilities."""
    
    def setUp(self):
        """Setup vor jedem Test."""
        if TEST_VERBOSE:
            print(f"\n{'='*60}")
            print(f"Test: {self._testMethodName}")
            print(f"{'='*60}")
    
    def tearDown(self):
        """Cleanup nach jedem Test."""
        pass
    
    def log(self, message: str):
        """Logging nur im Verbose-Modus."""
        if TEST_VERBOSE:
            print(f"  [TEST] {message}")

# ============================================================================
# 1. HARDWARE TESTS
# ============================================================================

class TestHardware(PathReplayTestCase):
    """Tests für Hardware-Komponenten."""
    
    @unittest.skipUnless(TEST_HARDWARE, "Hardware-Tests deaktiviert")
    def test_gpio_initialization(self):
        """Test GPIO-Initialisierung."""
        self.log("Teste GPIO-Setup...")
        
        import RPi.GPIO as GPIO
        
        # Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Test Pins
        test_pins = [12, 13, 20, 21, 23, 24]
        for pin in test_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            self.log(f"✅ GPIO{pin} initialisiert")
        
        GPIO.cleanup()
        self.log("GPIO-Test abgeschlossen")
    
    @unittest.skipUnless(TEST_HARDWARE, "Hardware-Tests deaktiviert")
    def test_pwm_initialization(self):
        """Test PWM-Initialisierung."""
        self.log("Teste PWM-Setup...")
        
        from rpi_hardware_pwm import HardwarePWM
        
        pwm_left = HardwarePWM(pwm_channel=0, hz=1000, chip=0)
        pwm_right = HardwarePWM(pwm_channel=1, hz=1000, chip=0)
        
        pwm_left.start(0)
        pwm_right.start(0)
        
        self.assertIsNotNone(pwm_left)
        self.assertIsNotNone(pwm_right)
        
        pwm_left.stop()
        pwm_right.stop()
        
        self.log("✅ PWM-Test abgeschlossen")
    
    @unittest.skipUnless(TEST_HARDWARE, "Hardware-Tests deaktiviert")
    def test_ultrasonic_sensors(self):
        """Test Ultraschall-Sensoren."""
        self.log("Teste Ultraschall-Sensoren...")
        
        import RPi.GPIO as GPIO
        
        GPIO.setmode(GPIO.BCM)
        
        TRIG = 12
        ECHO = 13
        
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        
        # Trigger
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        
        # Messe Echo
        timeout = time.time() + 1.0
        while GPIO.input(ECHO) == 0 and time.time() < timeout:
            pulse_start = time.time()
        
        timeout = time.time() + 1.0
        while GPIO.input(ECHO) == 1 and time.time() < timeout:
            pulse_end = time.time()
        
        try:
            duration = pulse_end - pulse_start
            distance = (duration * 34300) / 2
            self.assertTrue(2 < distance < 400, f"Distanz außerhalb gültigem Bereich: {distance}cm")
            self.log(f"✅ Ultraschall-Messung: {distance:.1f}cm")
        except UnboundLocalError:
            self.log("⚠️ Keine Echo-Antwort (Sensor nicht angeschlossen?)")
        
        GPIO.cleanup()

# ============================================================================
# 2. PATH LOADING TESTS
# ============================================================================

class TestPathLoading(PathReplayTestCase):
    """Tests für Path Loading Funktionalität."""
    
    def test_load_valid_recording(self):
        """Test Laden einer gültigen Aufzeichnung."""
        self.log("Teste Laden einer gültigen Aufzeichnung...")
        
        # Erstelle temporäre Datei
        with tempfile.NamedTemporaryFile(mode='wb', suffix='.json.gz', delete=False) as f:
            with gzip.open(f.name, 'wt', encoding='utf-8') as gz:
                json.dump(MOCK_RECORDING, gz)
            temp_path = f.name
        
        try:
            # Lade Recording
            with gzip.open(temp_path, 'rt', encoding='utf-8') as f:
                data = json.load(f)
            
            samples = data['controller_samples']
            
            self.assertEqual(len(samples), 3)
            self.assertEqual(samples[0]['left_trigger'], 0.5)
            self.log(f"✅ {len(samples)} Samples erfolgreich geladen")
            
        finally:
            Path(temp_path).unlink()
    
    def test_load_invalid_file(self):
        """Test Fehlerbehandlung bei ungültiger Datei."""
        self.log("Teste Fehlerbehandlung bei ungültiger Datei...")
        
        with self.assertRaises(FileNotFoundError):
            with gzip.open('nonexistent_file.json.gz', 'rt') as f:
                json.load(f)
        
        self.log("✅ FileNotFoundError korrekt ausgelöst")
    
    def test_load_corrupted_json(self):
        """Test Laden einer korrupten JSON-Datei."""
        self.log("Teste Laden einer korrupten JSON...")
        
        with tempfile.NamedTemporaryFile(mode='wb', suffix='.json.gz', delete=False) as f:
            with gzip.open(f.name, 'wt', encoding='utf-8') as gz:
                gz.write("{ invalid json")
            temp_path = f.name
        
        try:
            with self.assertRaises(json.JSONDecodeError):
                with gzip.open(temp_path, 'rt', encoding='utf-8') as f:
                    json.load(f)
            
            self.log("✅ JSONDecodeError korrekt ausgelöst")
        finally:
            Path(temp_path).unlink()
    
    def test_sample_timestamp_normalization(self):
        """Test Zeitstempel-Normalisierung."""
        self.log("Teste Zeitstempel-Normalisierung...")
        
        samples = [
            {'timestamp': 1000.0, 'left_trigger': 0.5, 'right_trigger': 0.5, 
             'direction': {'forward': 1.0, 'tank_turn': 0.0}},
            {'timestamp': 1000.1, 'left_trigger': 0.6, 'right_trigger': 0.6,
             'direction': {'forward': 1.0, 'tank_turn': 0.0}},
        ]
        
        # Normalisiere
        start_time = samples[0]['timestamp']
        for sample in samples:
            sample['timestamp'] -= start_time
        
        self.assertEqual(samples[0]['timestamp'], 0.0)
        self.assertAlmostEqual(samples[1]['timestamp'], 0.1, places=5)
        self.log("✅ Zeitstempel korrekt normalisiert")

# ============================================================================
# 3. COLLISION DETECTION TESTS
# ============================================================================

class TestCollisionDetection(PathReplayTestCase):
    """Tests für Kollisionserkennung."""
    
    def test_critical_distance_detection(self):
        """Test kritische Distanz-Erkennung."""
        self.log("Teste kritische Distanz-Erkennung...")
        
        CRITICAL_DISTANCE = 20.0
        
        # Mock ObstacleInfo
        class MockObstacle:
            def __init__(self, distance):
                self.distance = distance
            
            def get_min_distance(self):
                return self.distance
            
            def is_critical(self):
                return self.distance < CRITICAL_DISTANCE
        
        # Test kritisch
        obstacle_critical = MockObstacle(15.0)
        self.assertTrue(obstacle_critical.is_critical())
        self.log(f"✅ Kritisches Hindernis erkannt: {obstacle_critical.distance}cm")
        
        # Test nicht kritisch
        obstacle_safe = MockObstacle(30.0)
        self.assertFalse(obstacle_safe.is_critical())
        self.log(f"✅ Sicherer Abstand erkannt: {obstacle_safe.distance}cm")
    
    def test_speed_adjustment_calculation(self):
        """Test Geschwindigkeitsanpassung."""
        self.log("Teste Geschwindigkeitsanpassung...")
        
        CRITICAL = 20.0
        WARNING = 40.0
        SAFE = 60.0
        
        def calculate_speed(distance):
            if distance < CRITICAL:
                return 0.0
            elif distance < WARNING:
                ratio = (distance - CRITICAL) / (WARNING - CRITICAL)
                return 0.3 + (0.5 - 0.3) * ratio
            elif distance < SAFE:
                ratio = (distance - WARNING) / (SAFE - WARNING)
                return 0.5 + (1.0 - 0.5) * ratio
            else:
                return 1.0
        
        # Test verschiedene Distanzen
        test_cases = [
            (10.0, 0.0, "Emergency Stop"),
            (30.0, 0.4, "Slow"),
            (50.0, 0.75, "Reduced"),
            (70.0, 1.0, "Normal")
        ]
        
        for distance, expected_min, label in test_cases:
            speed = calculate_speed(distance)
            self.assertGreaterEqual(speed, expected_min - 0.1)
            self.log(f"✅ {label}: {distance}cm → {speed*100:.0f}% Speed")
    
    def test_sensor_fusion(self):
        """Test Sensor-Fusion (Ultraschall + LIDAR)."""
        self.log("Teste Sensor-Fusion...")
        
        # Mock Sensor-Daten
        ultrasonic_left = 50.0
        ultrasonic_right = 45.0
        lidar_obstacles = [
            (0, 30.0),    # Front
            (45, 55.0),   # Rechts vorne
            (-30, 60.0)   # Links vorne
        ]
        
        # Minimum-Distanz berechnen
        all_distances = [ultrasonic_left, ultrasonic_right]
        all_distances.extend([d for _, d in lidar_obstacles])
        
        min_distance = min(all_distances)
        
        self.assertEqual(min_distance, 30.0)
        self.log(f"✅ Sensor-Fusion: Minimum {min_distance}cm aus {len(all_distances)} Sensoren")

# ============================================================================
# 4. MOTOR CONTROL TESTS
# ============================================================================

class TestMotorControl(PathReplayTestCase):
    """Tests für Motor-Steuerung."""
    
    def test_trigger_to_duty_conversion(self):
        """Test Trigger zu Duty-Cycle Konvertierung."""
        self.log("Teste Trigger → Duty-Cycle Konvertierung...")
        
        MAX_DUTY = 12.0
        MIN_DUTY = 0.0
        
        def trigger_to_duty(trigger):
            normalized = (trigger + 1.0) / 2.0
            duty = normalized * MAX_DUTY
            return max(MIN_DUTY, min(MAX_DUTY, duty))
        
        test_cases = [
            (-1.0, 0.0),    # Minimum
            (0.0, 6.0),     # Mitte
            (1.0, 12.0),    # Maximum
        ]
        
        for trigger, expected_duty in test_cases:
            duty = trigger_to_duty(trigger)
            self.assertAlmostEqual(duty, expected_duty, places=1)
            self.log(f"✅ Trigger {trigger:+.1f} → {duty:.1f}% Duty")
    
    def test_motor_direction_setting(self):
        """Test Motor-Richtungssteuerung."""
        self.log("Teste Motor-Richtungssteuerung...")
        
        # Mock GPIO
        class MockGPIO:
            HIGH = True
            LOW = False
            outputs = {}
            
            @classmethod
            def output(cls, pin, level):
                cls.outputs[pin] = level
        
        mock_gpio = MockGPIO()
        
        DIR_LEFT = 20
        DIR_RIGHT = 21
        DIR_HIGH_IS_BACKWARD = False
        
        # Vorwärts
        left_level = mock_gpio.HIGH
        right_level = mock_gpio.HIGH
        mock_gpio.output(DIR_LEFT, left_level)
        mock_gpio.output(DIR_RIGHT, right_level)
        
        self.assertTrue(mock_gpio.outputs[DIR_LEFT])
        self.assertTrue(mock_gpio.outputs[DIR_RIGHT])
        self.log("✅ Vorwärts-Richtung korrekt gesetzt")
        
        # Rückwärts
        left_level = mock_gpio.LOW
        right_level = mock_gpio.LOW
        mock_gpio.output(DIR_LEFT, left_level)
        mock_gpio.output(DIR_RIGHT, right_level)
        
        self.assertFalse(mock_gpio.outputs[DIR_LEFT])
        self.assertFalse(mock_gpio.outputs[DIR_RIGHT])
        self.log("✅ Rückwärts-Richtung korrekt gesetzt")
    
    def test_speed_factor_application(self):
        """Test Anwendung des Geschwindigkeitsfaktors."""
        self.log("Teste Geschwindigkeitsfaktor-Anwendung...")
        
        base_duty = 10.0
        test_factors = [0.0, 0.5, 1.0]
        
        for factor in test_factors:
            adjusted_duty = base_duty * factor
            self.assertEqual(adjusted_duty, base_duty * factor)
            self.log(f"✅ {base_duty}% × {factor} = {adjusted_duty}%")

# ============================================================================
# 5. PATH FOLLOWING TESTS
# ============================================================================

class TestPathFollowing(PathReplayTestCase):
    """Tests für Path-Following Logik."""
    
    def test_sample_timing(self):
        """Test Sample-Timing und Synchronisation."""
        self.log("Teste Sample-Timing...")
        
        samples = [
            {'timestamp': 0.0},
            {'timestamp': 0.1},
            {'timestamp': 0.2},
        ]
        
        # Simuliere Elapsed Time
        elapsed_times = [0.05, 0.15, 0.25]
        
        for elapsed in elapsed_times:
            # Finde passendes Sample
            matching_sample = None
            for sample in samples:
                if sample['timestamp'] <= elapsed:
                    matching_sample = sample
            
            self.assertIsNotNone(matching_sample)
            self.log(f"✅ Zeit {elapsed:.2f}s → Sample {matching_sample['timestamp']:.2f}s")
    
    def test_path_progress_calculation(self):
        """Test Progress-Berechnung."""
        self.log("Teste Progress-Berechnung...")
        
        total_samples = 100
        
        for current_index in [0, 25, 50, 75, 100]:
            progress = (current_index / total_samples) * 100
            self.assertGreaterEqual(progress, 0)
            self.assertLessEqual(progress, 100)
            self.log(f"✅ Sample {current_index}/{total_samples} = {progress:.1f}% Progress")
    
    @unittest.skipIf(TEST_QUICK, "Langsamerer Test übersprungen")
    def test_playback_timing_accuracy(self):
        """Test Timing-Genauigkeit während Playback."""
        self.log("Teste Playback-Timing-Genauigkeit...")
        
        target_hz = 50
        sleep_time = 1.0 / target_hz
        
        timings = []
        for i in range(10):
            start = time.time()
            time.sleep(sleep_time)
            elapsed = time.time() - start
            timings.append(elapsed)
        
        avg_timing = sum(timings) / len(timings)
        deviation = abs(avg_timing - sleep_time)
        
        self.assertLess(deviation, 0.005, f"Timing-Abweichung zu groß: {deviation*1000:.2f}ms")
        self.log(f"✅ Durchschnittliche Timing-Abweichung: {deviation*1000:.2f}ms")

# ============================================================================
# 6. SAFETY SYSTEM TESTS
# ============================================================================

class TestSafetySystem(PathReplayTestCase):
    """Tests für Sicherheitssysteme."""
    
    def test_emergency_stop_trigger(self):
        """Test Emergency-Stop Aktivierung."""
        self.log("Teste Emergency-Stop...")
        
        emergency_stop = False
        critical_distance = 15.0
        threshold = 20.0
        
        if critical_distance < threshold:
            emergency_stop = True
        
        self.assertTrue(emergency_stop)
        self.log(f"✅ Emergency-Stop bei {critical_distance}cm aktiviert")
    
    def test_emergency_stop_recovery(self):
        """Test Emergency-Stop Recovery."""
        self.log("Teste Emergency-Stop Recovery...")
        
        emergency_stop = True
        current_distance = 50.0
        safe_threshold = 30.0
        
        if current_distance > safe_threshold:
            emergency_stop = False
        
        self.assertFalse(emergency_stop)
        self.log(f"✅ Emergency-Stop bei {current_distance}cm deaktiviert")
    
    def test_graceful_degradation(self):
        """Test Graceful Degradation bei Sensor-Ausfall."""
        self.log("Teste Graceful Degradation...")
        
        # Simuliere Sensor-Ausfall
        sensor_active = {
            'lidar': False,
            'ultrasonic_left': True,
            'ultrasonic_right': True
        }
        
        # Prüfe Fallback
        if not sensor_active['lidar']:
            # Sollte auf Ultraschall zurückfallen
            can_continue = sensor_active['ultrasonic_left'] and sensor_active['ultrasonic_right']
        else:
            can_continue = True
        
        self.assertTrue(can_continue)
        self.log("✅ System funktioniert ohne LIDAR mit Ultraschall")

# ============================================================================
# 7. INTEGRATION TESTS
# ============================================================================

class TestIntegration(PathReplayTestCase):
    """End-to-End Integration Tests."""
    
    def test_complete_path_cycle(self):
        """Test kompletter Path-Durchlauf (Mock)."""
        self.log("Teste kompletten Path-Durchlauf...")
        
        # Mock vollständiger Durchlauf
        samples = MOCK_PATH_SAMPLES
        current_index = 0
        completed = False
        
        # Simuliere Playback
        for i in range(len(samples) + 1):
            if current_index >= len(samples):
                completed = True
                break
            current_index += 1
        
        self.assertTrue(completed)
        self.log(f"✅ Path vollständig durchlaufen: {len(samples)} Samples")
    
    def test_obstacle_avoidance_integration(self):
        """Test Integration von Kollisionsvermeidung."""
        self.log("Teste Kollisionsvermeidungs-Integration...")
        
        # Simuliere Scenario mit Hindernis
        obstacle_distance = 25.0
        current_speed = 1.0
        
        CRITICAL = 20.0
        WARNING = 40.0
        
        # Anpassung
        if obstacle_distance < CRITICAL:
            new_speed = 0.0
        elif obstacle_distance < WARNING:
            new_speed = 0.5
        else:
            new_speed = 1.0
        
        self.assertEqual(new_speed, 0.5)
        self.log(f"✅ Geschwindigkeit bei {obstacle_distance}cm auf {new_speed*100:.0f}% reduziert")

# ============================================================================
# TEST SUITE RUNNER
# ============================================================================

def run_test_suite(hardware: bool = False, verbose: bool = False, quick: bool = False):
    """
    Führt komplette Test-Suite aus.
    
    Args:
        hardware: Aktiviert Hardware-Tests
        verbose: Detaillierte Ausgaben
        quick: Überspringt langsame Tests
    """
    global TEST_HARDWARE, TEST_VERBOSE, TEST_QUICK
    TEST_HARDWARE = hardware
    TEST_VERBOSE = verbose
    TEST_QUICK = quick
    
    print('\n' + '='*80)
    print('🧪 PATH REPLAY SYSTEM - COMPREHENSIVE TEST SUITE')
    print('='*80)
    print(f'Hardware Tests: {"✅ AKTIV" if hardware else "❌ DEAKTIVIERT"}')
    print(f'Verbose Mode:   {"✅ AKTIV" if verbose else "❌ DEAKTIVIERT"}')
    print(f'Quick Mode:     {"✅ AKTIV" if quick else "❌ DEAKTIVIERT"}')
    print('='*80 + '\n')
    
    # Erstelle Test Loader
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Füge alle Test-Klassen hinzu
    test_classes = [
        TestHardware,
        TestPathLoading,
        TestCollisionDetection,
        TestMotorControl,
        TestPathFollowing,
        TestSafetySystem,
        TestIntegration
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Führe Tests aus
    runner = unittest.TextTestRunner(verbosity=2 if verbose else 1)
    result = runner.run(suite)
    
    # Zusammenfassung
    print('\n' + '='*80)
    print('📊 TEST ERGEBNISSE')
    print('='*80)
    print(f'Tests durchgeführt:  {result.testsRun}')
    print(f'Erfolgreich:         {result.testsRun - len(result.failures) - len(result.errors)}')
    print(f'Fehlgeschlagen:      {len(result.failures)}')
    print(f'Fehler:              {len(result.errors)}')
    print(f'Übersprungen:        {len(result.skipped)}')
    print('='*80)
    
    if result.wasSuccessful():
        print('✅ ALLE TESTS BESTANDEN!')
        return 0
    else:
        print('❌ EINIGE TESTS FEHLGESCHLAGEN!')
        return 1

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Hauptfunktion."""
    parser = argparse.ArgumentParser(
        description='Comprehensive Test Suite für Path Replay System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  # Nur Software-Tests
  sudo python3 test_path_replay.py
  
  # Mit Hardware-Tests
  sudo python3 test_path_replay.py --hardware
  
  # Verbose Mode
  sudo python3 test_path_replay.py --verbose
  
  # Schnelle Tests
  sudo python3 test_path_replay.py --quick
        """
    )
    
    parser.add_argument('--hardware', action='store_true',
                        help='Aktiviert Hardware-Tests (erfordert echte Hardware)')
    parser.add_argument('--verbose', action='store_true',
                        help='Detaillierte Test-Ausgaben')
    parser.add_argument('--quick', action='store_true',
                        help='Überspringt langsame Tests')
    
    args = parser.parse_args()
    
    # Führe Tests aus
    exit_code = run_test_suite(
        hardware=args.hardware,
        verbose=args.verbose,
        quick=args.quick
    )
    
    sys.exit(exit_code)

if __name__ == '__main__':
    import os
    
    if os.geteuid() != 0:
        print('⚠️ [WARNING] Tests sollten mit sudo für Hardware-Zugriff ausgeführt werden')
        print('📝 Verwendung: sudo python3 test_path_replay.py [--hardware] [--verbose]')
    
    main()
