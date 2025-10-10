#!/usr/bin/env python3
"""Test Script für Path Recording System"""

import time
import random
from path_recording_system import *
from storage_system import *

def test_path_recording_system():
    """Vollständiger Test des Path Recording Systems"""
    
    print("🧪 Starte Path Recording System Test...")
    
    # 1. Storage Manager erstellen
    storage = create_storage_manager(
        directory="test_recordings",
        compression=CompressionType.GZIP,
        backup_count=3
    )
    
    # 2. Kalibrierungsdaten erstellen
    calibration = CalibrationData(
        controller_calibration={
            "left_trigger_offset": 0.02,
            "right_trigger_offset": -0.01,
            "button_delay_ms": 5
        },
        lidar_calibration={
            "angle_offset_deg": 1.5,
            "distance_multiplier": 1.0
        },
        calibration_timestamp=time.time(),
        calibration_quality=0.95
    )
    
    # 3. Environment Metadata
    metadata = EnvironmentMetadata(
        session_id=f"test_session_{int(time.time())}",
        start_timestamp=time.time(),
        environment_conditions={
            "temperature_celsius": 22.5,
            "humidity_percent": 45,
            "lighting": "indoor_artificial"
        },
        hardware_info={
            "controller": "Xbox Wireless Controller",
            "lidar": "RPLIDAR C1",
            "platform": "Raspberry Pi 5"
        },
        recording_settings={
            "controller_sample_rate": 50,
            "lidar_sample_rate": 5
        }
    )
    
    # 4. Header erstellen
    header = PathRecordingHeader(
        calibration_data=calibration,
        metadata=metadata,
        compression_type=CompressionType.GZIP
    )
    
    # 5. PathRecordingData erstellen
    recording = PathRecordingData(header=header)
    
    # 6. Simuliere Controller-Eingaben (5 Sekunden @ 50Hz)
    print("📊 Simuliere Controller-Eingaben...")
    start_time = time.time()
    
    for i in range(250):  # 5 Sekunden * 50Hz
        controller_sample = ControllerSample(
            timestamp=start_time + (i / 50.0),
            left_trigger=random.uniform(-1.0, 1.0),
            right_trigger=random.uniform(-1.0, 1.0),
            buttons={
                "A": random.choice([True, False]),
                "B": random.choice([True, False]),
                "X": False,
                "Y": False
            },
            direction={
                "x": random.uniform(-1.0, 1.0),
                "y": random.uniform(-1.0, 1.0),
                "rotation": random.uniform(-1.0, 1.0)
            }
        )
        recording.add_controller_sample(controller_sample)
    
    # 7. Simuliere LIDAR-Scans (5 Sekunden @ 5Hz)
    print("🔍 Simuliere LIDAR-Scans...")
    
    for i in range(25):  # 5 Sekunden * 5Hz
        # Simuliere 360° LIDAR-Scan
        scan_data = []
        for angle in range(0, 360, 5):  # Alle 5°
            quality = random.uniform(0.7, 1.0)
            distance = random.uniform(100, 3000)  # 10cm bis 3m
            scan_data.append((quality, float(angle), distance))
        
        lidar_frame = LidarFrame(
            timestamp=start_time + (i / 5.0),
            scan_data=scan_data
        )
        recording.add_lidar_frame(lidar_frame)
    
    # 8. Füge Checkpoints hinzu
    print("📍 Erstelle Checkpoints...")
    
    for i in range(3):
        checkpoint_time = start_time + (i + 1) * 1.5
        
        # Controller-State für Checkpoint
        controller_state = ControllerSample(
            timestamp=checkpoint_time,
            left_trigger=0.5,
            right_trigger=0.0,
            buttons={"A": False, "B": True, "X": False, "Y": False},
            direction={"x": 0.0, "y": 1.0, "rotation": 0.0}
        )
        
        # LIDAR-Snapshot
        checkpoint_scan = []
        for angle in range(0, 360, 10):
            checkpoint_scan.append((0.9, float(angle), random.uniform(200, 1000)))
        
        lidar_snapshot = LidarFrame(
            timestamp=checkpoint_time,
            scan_data=checkpoint_scan
        )
        
        checkpoint = Checkpoint(
            timestamp=checkpoint_time,
            position=(float(i * 100), float(i * 50), float(i * 30)),
            controller_state=controller_state,
            lidar_snapshot=lidar_snapshot,
            checkpoint_id=f"checkpoint_{i+1}",
            notes=f"Testpunkt {i+1} - Geradeausfahrt"
        )
        
        recording.add_checkpoint(checkpoint)
    
    # 9. Session beenden
    recording.header.metadata.end_timestamp = time.time()
    
    # 10. Statistiken ausgeben
    stats = recording.get_stats()
    print("\n📈 Aufzeichnungsstatistiken:")
    for key, value in stats.items():
        print(f"   {key}: {value}")
    
    # 11. JSON-Speicherung testen
    print("\n💾 Teste JSON-Speicherung...")
    json_path = storage.save_json(recording)
    print(f"   JSON gespeichert: {json_path}")
    
    # 12. Binary-Speicherung testen
    print("\n💾 Teste Binary-Speicherung...")
    binary_path = storage.save_binary(recording)
    print(f"   Binary gespeichert: {binary_path}")
    
    # 13. Laden testen
    print("\n📂 Teste Laden...")
    loaded_json = storage.load_json(json_path)
    loaded_binary = storage.load_binary(binary_path)
    
    print(f"   JSON geladen: {len(loaded_json.controller_samples)} Controller-Samples")
    print(f"   Binary geladen: {len(loaded_binary.lidar_frames)} LIDAR-Frames")
    
    # 14. Validierung testen
    print("\n✅ Teste Validierung...")
    json_validation = storage.validate_file(json_path)
    binary_validation = storage.validate_file(binary_path)
    
    print(f"   JSON valid: {json_validation['valid']}")
    print(f"   Binary valid: {binary_validation['valid']}")
    
    # 15. Aufzeichnungen auflisten
    print("\n📋 Verfügbare Aufzeichnungen:")
    recordings_list = storage.list_recordings()
    for rec in recordings_list:
        print(f"   {rec['filename']} ({rec['format']}) - {rec['size_bytes']} bytes")
    
    print("\n🎉 Test erfolgreich abgeschlossen!")
    
    return recording, storage

if __name__ == "__main__":
    test_path_recording_system()
