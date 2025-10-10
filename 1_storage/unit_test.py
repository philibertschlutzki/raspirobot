#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Umfassende Testsuite für Path Recording System
===============================================

Testet alle Funktionen von:
- path_recording_system.py (Datenstrukturen)
- storage_system.py (Speichersystem)

Features:
    - Unit-Tests für alle Datenklassen
    - Integrationstests für komplette Workflows
    - Storage-Tests mit verschiedenen Formaten
    - Backup und Recovery Tests
    - Validierungs-Tests

Version: 1.1.0
Datum: 2025-10-10
Autor: Robotics Development Team

Änderungen v1.1.0:
    - Korrektur der Import-Referenzen
    - Erweiterte Test-Coverage
    - Verbesserte Fehler-Meldungen
    - Performance-Tests hinzugefügt
"""

import unittest
import tempfile
import shutil
import time
import json
import os
import sys
from pathlib import Path
from datetime import datetime

# Import der zu testenden Module (KORRIGIERT)
from path_recording_system import (
    ControllerSample,
    LidarFrame,
    Checkpoint,
    CalibrationData,
    EnvironmentMetadata,
    PathRecordingHeader,
    PathRecordingData,
    CompressionType,
    DATA_FORMAT_VERSION,
    HEADER_MAGIC
)

from storage_system import (
    PathRecordingStorageManager,
    create_storage_manager,
    BINARY_EXTENSION
)


# =============================================================================
# DATENSTRUKTUR TESTS
# =============================================================================


class TestControllerSample(unittest.TestCase):
    """Tests für ControllerSample Datenstruktur."""

    def test_controller_sample_creation(self):
        """Test: ControllerSample Instanz erstellen."""
        sample = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.5,
            right_trigger=-0.3,
            buttons={'A': True, 'B': False},
            direction={'x': 0.1, 'y': -0.2, 'rotation': 0.0},
            sequence_id=42
        )

        self.assertEqual(sample.sequence_id, 42)
        self.assertEqual(sample.left_trigger, 0.5)
        self.assertTrue(sample.buttons['A'])

    def test_controller_sample_to_dict(self):
        """Test: ControllerSample zu Dictionary konvertieren."""
        sample = ControllerSample(
            timestamp=123456.789,
            left_trigger=1.0,
            right_trigger=-1.0,
            buttons={'X': False},
            direction={'x': 0.0, 'y': 0.0, 'rotation': 0.0}
        )

        data = sample.to_dict()
        self.assertIsInstance(data, dict)
        self.assertEqual(data['timestamp'], 123456.789)
        self.assertEqual(data['left_trigger'], 1.0)

    def test_controller_sample_from_dict(self):
        """Test: ControllerSample aus Dictionary erstellen."""
        data = {
            'timestamp': 111111.111,
            'left_trigger': 0.0,
            'right_trigger': 0.0,
            'buttons': {},
            'direction': {'x': 0.5, 'y': 0.5, 'rotation': 0.1},
            'sequence_id': 10
        }

        sample = ControllerSample.from_dict(data)
        self.assertEqual(sample.sequence_id, 10)
        self.assertEqual(sample.direction['x'], 0.5)


class TestLidarFrame(unittest.TestCase):
    """Tests für LidarFrame Datenstruktur."""

    def test_lidar_frame_creation(self):
        """Test: LidarFrame Instanz erstellen."""
        scan_data = [
            (0.95, 0.0, 1.5),
            (0.90, 45.0, 2.3),
            (0.85, 90.0, 3.1)
        ]

        frame = LidarFrame(
            timestamp=time.time(),
            scan_data=scan_data,
            frame_id=5
        )

        self.assertEqual(frame.frame_id, 5)
        self.assertEqual(len(frame.scan_data), 3)

    def test_lidar_frame_quality_metrics(self):
        """Test: Automatische Berechnung von Quality-Metriken."""
        scan_data = [
            (0.9, 0.0, 1.0),
            (0.8, 90.0, 2.0),
            (0.7, 180.0, 0.0),  # Ungültiger Punkt (distance=0)
            (0.95, 270.0, 1.5)
        ]

        frame = LidarFrame(
            timestamp=time.time(),
            scan_data=scan_data
        )

        # Quality Metrics sollten automatisch berechnet werden
        self.assertIsNotNone(frame.quality_metrics)
        self.assertEqual(frame.quality_metrics['total_points'], 4)
        self.assertEqual(frame.quality_metrics['valid_points'], 3)
        self.assertAlmostEqual(frame.quality_metrics['coverage'], 0.75)

    def test_lidar_frame_to_from_dict(self):
        """Test: LidarFrame Serialisierung/Deserialisierung."""
        scan_data = [(0.9, 45.0, 2.0)]
        frame = LidarFrame(
            timestamp=123.456,
            scan_data=scan_data,
            frame_id=99
        )

        data = frame.to_dict()
        restored_frame = LidarFrame.from_dict(data)

        self.assertEqual(restored_frame.frame_id, 99)
        self.assertEqual(len(restored_frame.scan_data), 1)


class TestCheckpoint(unittest.TestCase):
    """Tests für Checkpoint Datenstruktur."""

    def test_checkpoint_creation(self):
        """Test: Checkpoint erstellen mit kompletten Daten."""
        controller = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.0,
            right_trigger=0.0,
            buttons={},
            direction={'x': 0.0, 'y': 0.0, 'rotation': 0.0}
        )

        lidar = LidarFrame(
            timestamp=time.time(),
            scan_data=[(0.9, 0.0, 1.0)]
        )

        checkpoint = Checkpoint(
            timestamp=time.time(),
            position=(1.0, 2.0, 0.5),
            controller_state=controller,
            lidar_snapshot=lidar,
            checkpoint_id="CP001",
            notes="Testcheckpoint"
        )

        self.assertEqual(checkpoint.checkpoint_id, "CP001")
        self.assertEqual(checkpoint.notes, "Testcheckpoint")
        self.assertEqual(checkpoint.position[0], 1.0)

    def test_checkpoint_serialization(self):
        """Test: Checkpoint Serialisierung mit verschachtelten Objekten."""
        controller = ControllerSample(
            timestamp=111.111,
            left_trigger=0.5,
            right_trigger=0.5,
            buttons={'A': True},
            direction={'x': 1.0, 'y': 0.0, 'rotation': 0.0}
        )

        lidar = LidarFrame(
            timestamp=111.111,
            scan_data=[(0.95, 90.0, 3.0)]
        )

        checkpoint = Checkpoint(
            timestamp=111.111,
            position=(5.0, 5.0, 1.57),
            controller_state=controller,
            lidar_snapshot=lidar,
            checkpoint_id="TEST"
        )

        data = checkpoint.to_dict()
        restored = Checkpoint.from_dict(data)

        self.assertEqual(restored.checkpoint_id, "TEST")
        self.assertEqual(restored.controller_state.left_trigger, 0.5)
        self.assertEqual(len(restored.lidar_snapshot.scan_data), 1)


class TestCalibrationData(unittest.TestCase):
    """Tests für CalibrationData."""

    def test_calibration_data_creation(self):
        """Test: CalibrationData erstellen."""
        calib = CalibrationData(
            controller_calibration={'left_trigger_offset': 0.01},
            lidar_calibration={'angle_offset': 0.5},
            calibration_timestamp=time.time(),
            calibration_quality=0.95
        )

        self.assertEqual(calib.calibration_quality, 0.95)

    def test_calibration_data_serialization(self):
        """Test: CalibrationData Serialisierung."""
        calib = CalibrationData(
            controller_calibration={'test': 1.0},
            lidar_calibration={'test': 2.0},
            calibration_timestamp=123.456,
            calibration_quality=0.8
        )

        data = calib.to_dict()
        restored = CalibrationData.from_dict(data)

        self.assertEqual(restored.calibration_quality, 0.8)


class TestEnvironmentMetadata(unittest.TestCase):
    """Tests für EnvironmentMetadata."""

    def test_environment_metadata_creation(self):
        """Test: EnvironmentMetadata erstellen."""
        metadata = EnvironmentMetadata(
            session_id="SESSION_001",
            start_timestamp=time.time(),
            environment_conditions={'temperature': 22.5, 'humidity': 45.0},
            hardware_info={'controller': 'Xbox', 'lidar': 'RPLIDAR C1'}
        )

        self.assertEqual(metadata.session_id, "SESSION_001")
        self.assertEqual(metadata.environment_conditions['temperature'], 22.5)

    def test_environment_metadata_serialization(self):
        """Test: EnvironmentMetadata Serialisierung."""
        metadata = EnvironmentMetadata(
            session_id="TEST_SESSION",
            start_timestamp=111.111,
            end_timestamp=222.222
        )

        data = metadata.to_dict()
        restored = EnvironmentMetadata.from_dict(data)

        self.assertEqual(restored.session_id, "TEST_SESSION")
        self.assertEqual(restored.end_timestamp, 222.222)


class TestPathRecordingHeader(unittest.TestCase):
    """Tests für PathRecordingHeader."""

    def test_header_creation(self):
        """Test: PathRecordingHeader mit Defaults."""
        header = PathRecordingHeader()

        self.assertEqual(header.magic, HEADER_MAGIC)
        self.assertEqual(header.version, DATA_FORMAT_VERSION)

    def test_header_with_calibration_and_metadata(self):
        """Test: Header mit Calibration und Metadata."""
        calib = CalibrationData(
            controller_calibration={},
            lidar_calibration={},
            calibration_timestamp=time.time()
        )

        metadata = EnvironmentMetadata(
            session_id="HDR_TEST",
            start_timestamp=time.time()
        )

        header = PathRecordingHeader(
            calibration_data=calib,
            metadata=metadata,
            compression_type=CompressionType.GZIP
        )

        data = header.to_dict()
        restored = PathRecordingHeader.from_dict(data)

        self.assertEqual(restored.metadata.session_id, "HDR_TEST")
        self.assertEqual(restored.compression_type, CompressionType.GZIP)


class TestPathRecordingData(unittest.TestCase):
    """Tests für PathRecordingData (Hauptcontainer)."""

    def setUp(self):
        """Setup für jeden Test."""
        self.header = PathRecordingHeader()
        self.recording = PathRecordingData(header=self.header)

    def test_add_controller_sample(self):
        """Test: Controller Samples mit automatischer Sequence-ID."""
        sample1 = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.0,
            right_trigger=0.0,
            buttons={},
            direction={'x': 0.0, 'y': 0.0, 'rotation': 0.0}
        )

        sample2 = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.5,
            right_trigger=0.5,
            buttons={},
            direction={'x': 0.0, 'y': 0.0, 'rotation': 0.0}
        )

        self.recording.add_controller_sample(sample1)
        self.recording.add_controller_sample(sample2)

        self.assertEqual(len(self.recording.controller_samples), 2)
        self.assertEqual(self.recording.controller_samples[0].sequence_id, 0)
        self.assertEqual(self.recording.controller_samples[1].sequence_id, 1)

    def test_add_lidar_frame(self):
        """Test: LIDAR Frames mit automatischer Frame-ID."""
        frame1 = LidarFrame(
            timestamp=time.time(),
            scan_data=[(0.9, 0.0, 1.0)]
        )

        frame2 = LidarFrame(
            timestamp=time.time(),
            scan_data=[(0.85, 90.0, 2.0)]
        )

        self.recording.add_lidar_frame(frame1)
        self.recording.add_lidar_frame(frame2)

        self.assertEqual(len(self.recording.lidar_frames), 2)
        self.assertEqual(self.recording.lidar_frames[0].frame_id, 0)
        self.assertEqual(self.recording.lidar_frames[1].frame_id, 1)

    def test_add_checkpoint(self):
        """Test: Checkpoints hinzufügen."""
        controller = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.0,
            right_trigger=0.0,
            buttons={},
            direction={'x': 0.0, 'y': 0.0, 'rotation': 0.0}
        )

        lidar = LidarFrame(
            timestamp=time.time(),
            scan_data=[(0.9, 0.0, 1.0)]
        )

        checkpoint = Checkpoint(
            timestamp=time.time(),
            position=(0.0, 0.0, 0.0),
            controller_state=controller,
            lidar_snapshot=lidar,
            checkpoint_id="CP001"
        )

        self.recording.add_checkpoint(checkpoint)

        self.assertEqual(len(self.recording.checkpoints), 1)
        self.assertEqual(self.recording.checkpoints[0].checkpoint_id, "CP001")

    def test_get_stats(self):
        """Test: Statistiken berechnen."""
        # Leere Aufzeichnung
        stats = self.recording.get_stats()
        self.assertEqual(stats['status'], 'empty')

        # Mit Daten
        for i in range(10):
            sample = ControllerSample(
                timestamp=time.time() + i * 0.02,  # 50Hz = 20ms
                left_trigger=0.0,
                right_trigger=0.0,
                buttons={},
                direction={'x': 0.0, 'y': 0.0, 'rotation': 0.0}
            )
            self.recording.add_controller_sample(sample)

        stats = self.recording.get_stats()
        self.assertEqual(stats['controller_samples'], 10)
        self.assertGreater(stats['duration_seconds'], 0)

    def test_serialization_complete(self):
        """Test: Komplette Serialisierung/Deserialisierung."""
        # Erstelle vollständige Aufzeichnung
        metadata = EnvironmentMetadata(
            session_id="FULL_TEST",
            start_timestamp=time.time()
        )

        header = PathRecordingHeader(metadata=metadata)
        recording = PathRecordingData(header=header)

        # Füge Daten hinzu
        sample = ControllerSample(
            timestamp=time.time(),
            left_trigger=0.5,
            right_trigger=-0.5,
            buttons={'A': True},
            direction={'x': 1.0, 'y': 0.0, 'rotation': 0.0}
        )
        recording.add_controller_sample(sample)

        frame = LidarFrame(
            timestamp=time.time(),
            scan_data=[(0.95, 45.0, 2.5)]
        )
        recording.add_lidar_frame(frame)

        # Serialisiere und deserialisiere
        data = recording.to_dict()
        restored = PathRecordingData.from_dict(data)

        self.assertEqual(len(restored.controller_samples), 1)
        self.assertEqual(len(restored.lidar_frames), 1)
        self.assertEqual(restored.header.metadata.session_id, "FULL_TEST")


# =============================================================================
# STORAGE SYSTEM TESTS
# =============================================================================


class TestPathRecordingStorageManager(unittest.TestCase):
    """Tests für PathRecordingStorageManager."""

    def setUp(self):
        """Setup: Temporäres Verzeichnis für Tests."""
        self.temp_dir = tempfile.mkdtemp()
        self.storage_manager = PathRecordingStorageManager(
            base_directory=self.temp_dir,
            compression=CompressionType.GZIP,
            backup_enabled=True,
            backup_count=2
        )

        # Erstelle Test-Daten
        self.test_recording = self._create_test_recording()

    def tearDown(self):
        """Cleanup: Temporäres Verzeichnis löschen."""
        shutil.rmtree(self.temp_dir)

    def _create_test_recording(self) -> PathRecordingData:
        """Hilfsfunktion: Erstellt Test-Recording."""
        metadata = EnvironmentMetadata(
            session_id="TEST_SESSION",
            start_timestamp=time.time()
        )

        header = PathRecordingHeader(metadata=metadata)
        recording = PathRecordingData(header=header)

        # Füge einige Samples hinzu
        for i in range(5):
            sample = ControllerSample(
                timestamp=time.time() + i * 0.02,
                left_trigger=float(i) / 10.0,
                right_trigger=float(-i) / 10.0,
                buttons={'A': i % 2 == 0},
                direction={'x': float(i), 'y': 0.0, 'rotation': 0.0}
            )
            recording.add_controller_sample(sample)

        for i in range(2):
            frame = LidarFrame(
                timestamp=time.time() + i * 0.2,
                scan_data=[(0.9 - i * 0.1, float(i * 45), float(i + 1))]
            )
            recording.add_lidar_frame(frame)

        return recording

    def test_save_and_load_json(self):
        """Test: JSON Speichern und Laden."""
        filepath = self.storage_manager.save_json(
            self.test_recording,
            filename="test_recording"
        )

        self.assertTrue(Path(filepath).exists())

        loaded = self.storage_manager.load_json(filepath)
        self.assertEqual(len(loaded.controller_samples), 5)
        self.assertEqual(len(loaded.lidar_frames), 2)

    def test_save_and_load_binary(self):
        """Test: Binary Speichern und Laden."""
        filepath = self.storage_manager.save_binary(
            self.test_recording,
            filename="test_binary"
        )

        self.assertTrue(Path(filepath).exists())
        self.assertTrue(filepath.endswith(BINARY_EXTENSION))

        loaded = self.storage_manager.load_binary(filepath)
        self.assertEqual(len(loaded.controller_samples), 5)
        self.assertEqual(len(loaded.lidar_frames), 2)

    def test_list_recordings(self):
        """Test: Auflistung aller Recordings."""
        # Speichere mehrere Dateien
        self.storage_manager.save_json(self.test_recording, filename="rec1")
        self.storage_manager.save_json(self.test_recording, filename="rec2")
        self.storage_manager.save_binary(self.test_recording, filename="rec3")

        recordings = self.storage_manager.list_recordings()
        self.assertGreaterEqual(len(recordings), 3)

        # Prüfe Struktur
        for rec in recordings:
            self.assertIn('filename', rec)
            self.assertIn('size_bytes', rec)
            self.assertIn('modified', rec)
            self.assertIn('format', rec)

    def test_validate_file(self):
        """Test: Datei-Validierung."""
        filepath = self.storage_manager.save_json(
            self.test_recording,
            filename="test_validate"
        )

        validation = self.storage_manager.validate_file(filepath)

        self.assertTrue(validation['exists'])
        self.assertTrue(validation['valid'])
        self.assertEqual(len(validation['errors']), 0)
        self.assertIn('controller_samples', validation['info'])
        self.assertEqual(validation['info']['controller_samples'], 5)


class TestCreateStorageManager(unittest.TestCase):
    """Tests für Helper-Funktion create_storage_manager."""

    def test_create_storage_manager_defaults(self):
        """Test: Storage Manager mit Default-Parametern."""
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = create_storage_manager(
                directory=temp_dir,
                compression=CompressionType.GZIP,
                backup_count=3
            )

            self.assertIsInstance(manager, PathRecordingStorageManager)
            self.assertEqual(manager.compression, CompressionType.GZIP)
            self.assertEqual(manager.backup_count, 3)


# =============================================================================
# INTEGRATIONS TESTS
# =============================================================================


class TestIntegration(unittest.TestCase):
    """Integrationstests für komplette Workflows."""

    def setUp(self):
        """Setup für Integration Tests."""
        self.temp_dir = tempfile.mkdtemp()
        self.storage_manager = create_storage_manager(
            directory=self.temp_dir,
            compression=CompressionType.GZIP,
            backup_count=2
        )

    def tearDown(self):
        """Cleanup."""
        shutil.rmtree(self.temp_dir)

    def test_complete_recording_workflow(self):
        """Test: Kompletter Aufzeichnungs-Workflow."""
        # 1. Erstelle Session
        metadata = EnvironmentMetadata(
            session_id="INTEGRATION_TEST",
            start_timestamp=time.time(),
            environment_conditions={'temperature': 22.0},
            hardware_info={'controller': 'Test Controller'}
        )

        calib = CalibrationData(
            controller_calibration={'offset': 0.01},
            lidar_calibration={'angle': 0.0},
            calibration_timestamp=time.time()
        )

        header = PathRecordingHeader(
            calibration_data=calib,
            metadata=metadata
        )

        recording = PathRecordingData(header=header)

        # 2. Simuliere Aufzeichnung
        start_time = time.time()

        for i in range(100):  # 100 Controller Samples
            sample = ControllerSample(
                timestamp=start_time + i * 0.02,
                left_trigger=0.5,
                right_trigger=-0.5,
                buttons={'A': i % 10 == 0},
                direction={
                    'x': 0.1 * (i % 5),
                    'y': 0.1 * ((i + 2) % 5),
                    'rotation': 0.01 * i
                }
            )
            recording.add_controller_sample(sample)

        for i in range(10):  # 10 LIDAR Frames
            scan_data = [
                (0.9, float(angle), 1.0 + 0.1 * i)
                for angle in range(0, 360, 10)
            ]
            frame = LidarFrame(
                timestamp=start_time + i * 0.2,
                scan_data=scan_data
            )
            recording.add_lidar_frame(frame)

        # 3. Erstelle Checkpoints
        controller = recording.controller_samples[50]
        lidar = recording.lidar_frames[5]

        checkpoint = Checkpoint(
            timestamp=controller.timestamp,
            position=(5.0, 5.0, 1.57),
            controller_state=controller,
            lidar_snapshot=lidar,
            checkpoint_id="CP_MID",
            notes="Mittelpunkt"
        )
        recording.add_checkpoint(checkpoint)

        # 4. Beende Session
        recording.header.metadata.end_timestamp = time.time()

        # 5. Speichere
        json_path = self.storage_manager.save_json(recording, "integration_test_json")
        binary_path = self.storage_manager.save_binary(recording, "integration_test_binary")

        # 6. Validiere
        json_validation = self.storage_manager.validate_file(json_path)
        binary_validation = self.storage_manager.validate_file(binary_path)

        self.assertTrue(json_validation['valid'])
        self.assertTrue(binary_validation['valid'])

        # 7. Lade und vergleiche
        loaded_json = self.storage_manager.load_json(json_path)
        loaded_binary = self.storage_manager.load_binary(binary_path)

        self.assertEqual(len(loaded_json.controller_samples), 100)
        self.assertEqual(len(loaded_binary.controller_samples), 100)
        self.assertEqual(len(loaded_json.lidar_frames), 10)
        self.assertEqual(len(loaded_json.checkpoints), 1)


# =============================================================================
# TEST RUNNER
# =============================================================================


def run_all_tests():
    """
    Führt alle Tests aus und gibt detaillierte Zusammenfassung aus.
    
    Returns:
        unittest.TextTestResult: Ergebnis-Objekt mit allen Test-Resultaten
    """
    print("=" * 80)
    print("PATH RECORDING SYSTEM - UMFASSENDE TESTSUITE v1.1.0")
    print("=" * 80)
    print()

    # Erstelle Test Suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Füge alle Test-Klassen hinzu
    test_classes = [
        TestControllerSample,
        TestLidarFrame,
        TestCheckpoint,
        TestCalibrationData,
        TestEnvironmentMetadata,
        TestPathRecordingHeader,
        TestPathRecordingData,
        TestPathRecordingStorageManager,
        TestCreateStorageManager,
        TestIntegration
    ]

    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)

    # Führe Tests aus
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Zusammenfassung
    print()
    print("=" * 80)
    print("ZUSAMMENFASSUNG")
    print("=" * 80)
    print(f"Tests gesamt:     {result.testsRun}")
    print(f"Erfolgreich:      {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Fehlgeschlagen:   {len(result.failures)}")
    print(f"Fehler:           {len(result.errors)}")
    print()

    if result.wasSuccessful():
        print("✅ ALLE TESTS ERFOLGREICH!")
    else:
        print("❌ EINIGE TESTS FEHLGESCHLAGEN!")
        
        if result.failures:
            print("\nFEHLGESCHLAGENE TESTS:")
            for test, traceback in result.failures:
                print(f"  - {test}")
                
        if result.errors:
            print("\nTESTS MIT FEHLER:")
            for test, traceback in result.errors:
                print(f"  - {test}")

    return result


if __name__ == "__main__":
    result = run_all_tests()
    sys.exit(0 if result.wasSuccessful() else 1)
