#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Recording System - Phase 1: Datenstruktur und Speichersystem (KORRIGIERT)
==============================================================================

Robuste Datenaufzeichnung für Controller-Eingaben und LIDAR-Scans mit:
- Zeitstempel-basierte Datenstruktur (10-50Hz Controller, 2-5Hz LIDAR)  
- Versionierte JSON/Binary-Speicherung mit Kompression
- Metadaten-Verwaltung und Backup-Mechanismen
- Checksum-Validierung und Recovery-Funktionen

Version: 1.0.1 (BUGFIX)
Datum: 2025-10-10
FIXES: LidarFrame quality_metrics default_factory Problem
"""

import json
import gzip
import pickle
import struct
import hashlib
import os
import time
import threading
import queue
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple, Any, NamedTuple
from dataclasses import dataclass, asdict, field
from pathlib import Path
import logging

# =============================================================================
# KONSTANTEN UND KONFIGURATION
# =============================================================================

# Versionierung
DATA_FORMAT_VERSION = "1.0.1"
HEADER_MAGIC = b'PREC'  # Path RECording magic bytes
CHUNK_MAGIC = b'CHNK'   # Chunk magic bytes

# Sample Rates
CONTROLLER_SAMPLE_RATE_HZ = 50  # 50Hz für Controller (20ms)
LIDAR_SAMPLE_RATE_HZ = 5        # 5Hz für LIDAR (200ms)

# Dateiformate
class CompressionType:
    NONE = "none"
    GZIP = "gzip"
    LZMA = "lzma"

# Datentypen
class DataType:
    CONTROLLER = "controller"
    LIDAR = "lidar" 
    METADATA = "metadata"
    CHECKPOINT = "checkpoint"

# Dateiendungen
FILE_EXTENSIONS = {
    CompressionType.NONE: ".json",
    CompressionType.GZIP: ".json.gz", 
    CompressionType.LZMA: ".json.xz"
}

BINARY_EXTENSION = ".prec"

# =============================================================================
# DATENSTRUKTUREN
# =============================================================================

@dataclass
class ControllerSample:
    """Einzelne Controller-Eingabe mit Zeitstempel"""
    timestamp: float  # Unix timestamp mit Mikrosekunden-Genauigkeit
    left_trigger: float     # -1.0 bis 1.0
    right_trigger: float    # -1.0 bis 1.0
    buttons: Dict[str, bool]  # Button-Name → gedrückt
    direction: Dict[str, float]  # Richtungsachsen (x, y, rotation)
    sequence_id: int = 0  # Fortlaufende Nummer für Lücken-Erkennung

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'ControllerSample':
        return cls(**data)

@dataclass 
class LidarFrame:
    """Einzelner LIDAR-Scan mit Quality-Metriken"""
    timestamp: float  # Unix timestamp
    scan_data: List[Tuple[float, float, float]]  # [(quality, angle, distance), ...]
    quality_metrics: Dict[str, float] = field(default_factory=dict)  # Qualitätsindikatoren - KORRIGIERT!
    frame_id: int = 0  # Fortlaufende Frame-Nummer

    def __post_init__(self):
        """Berechne Quality-Metriken automatisch falls leer - KORRIGIERT!"""
        if not self.quality_metrics and self.scan_data:
            self.quality_metrics = self._calculate_quality_metrics()

    def _calculate_quality_metrics(self) -> Dict[str, float]:
        """Berechnet automatisch Quality-Metriken aus Scan-Daten"""
        if not self.scan_data:
            return {"valid_points": 0, "coverage": 0.0, "avg_quality": 0.0}

        valid_points = sum(1 for q, a, d in self.scan_data if d > 0)
        total_points = len(self.scan_data)
        coverage = valid_points / total_points if total_points > 0 else 0.0
        avg_quality = sum(q for q, a, d in self.scan_data if d > 0) / valid_points if valid_points > 0 else 0.0

        return {
            "valid_points": valid_points,
            "total_points": total_points,
            "coverage": coverage,
            "avg_quality": avg_quality,
            "min_distance": min((d for _, _, d in self.scan_data if d > 0), default=0.0),
            "max_distance": max((d for _, _, d in self.scan_data if d > 0), default=0.0)
        }

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'LidarFrame':
        return cls(**data)

@dataclass
class Checkpoint:
    """Strategischer Wegpunkt für Path-Validierung"""
    timestamp: float
    position: Tuple[float, float, float]  # (x, y, orientation)
    controller_state: ControllerSample
    lidar_snapshot: LidarFrame
    checkpoint_id: str  # Eindeutige ID
    notes: str = ""  # Optionale Beschreibung

    def to_dict(self) -> Dict:
        data = asdict(self)
        data['controller_state'] = self.controller_state.to_dict()
        data['lidar_snapshot'] = self.lidar_snapshot.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict) -> 'Checkpoint':
        controller_data = data.pop('controller_state')
        lidar_data = data.pop('lidar_snapshot')
        return cls(
            controller_state=ControllerSample.from_dict(controller_data),
            lidar_snapshot=LidarFrame.from_dict(lidar_data),
            **data
        )

@dataclass
class CalibrationData:
    """Kalibrierungsdaten für Controller und LIDAR"""
    controller_calibration: Dict[str, float]  # Trigger-Offsets, Button-Delays etc.
    lidar_calibration: Dict[str, float]  # Winkel-Offsets, Distanz-Korrekturen
    calibration_timestamp: float
    calibration_quality: float = 1.0  # 0.0-1.0 Qualitätsbewertung

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod 
    def from_dict(cls, data: Dict) -> 'CalibrationData':
        return cls(**data)

@dataclass
class EnvironmentMetadata:
    """Umgebungsbedingungen und Session-Metadaten"""
    session_id: str  # Eindeutige Session-ID
    start_timestamp: float
    end_timestamp: Optional[float] = None
    environment_conditions: Dict[str, Any] = field(default_factory=dict)  # Temperatur, Feuchtigkeit etc.
    hardware_info: Dict[str, str] = field(default_factory=dict)  # Controller-ID, LIDAR-Model etc.
    software_version: str = DATA_FORMAT_VERSION
    recording_settings: Dict[str, Any] = field(default_factory=dict)  # Sample-Rates etc.

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'EnvironmentMetadata':
        return cls(**data)

@dataclass
class PathRecordingHeader:
    """Header-Information für Path Recording Dateien"""
    magic: bytes = HEADER_MAGIC
    version: str = DATA_FORMAT_VERSION
    timestamp: float = field(default_factory=time.time)
    calibration_data: Optional[CalibrationData] = None
    metadata: Optional[EnvironmentMetadata] = None
    checksum_algorithm: str = "sha256"
    compression_type: str = CompressionType.GZIP

    def to_dict(self) -> Dict:
        data = {
            'magic': self.magic.decode('ascii'),
            'version': self.version,
            'timestamp': self.timestamp,
            'checksum_algorithm': self.checksum_algorithm,
            'compression_type': self.compression_type
        }
        if self.calibration_data:
            data['calibration_data'] = self.calibration_data.to_dict()
        if self.metadata:
            data['metadata'] = self.metadata.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict) -> 'PathRecordingHeader':
        calibration_data = None
        metadata = None

        if 'calibration_data' in data:
            calibration_data = CalibrationData.from_dict(data.pop('calibration_data'))
        if 'metadata' in data:
            metadata = EnvironmentMetadata.from_dict(data.pop('metadata'))

        data['magic'] = data['magic'].encode('ascii')
        return cls(
            calibration_data=calibration_data,
            metadata=metadata,
            **data
        )

@dataclass
class PathRecordingData:
    """Hauptdatencontainer für komplette Aufzeichnung"""
    header: PathRecordingHeader
    controller_samples: List[ControllerSample] = field(default_factory=list)
    lidar_frames: List[LidarFrame] = field(default_factory=list) 
    checkpoints: List[Checkpoint] = field(default_factory=list)

    def add_controller_sample(self, sample: ControllerSample):
        """Fügt Controller-Sample hinzu mit automatischer Sequence-ID"""
        if self.controller_samples:
            sample.sequence_id = self.controller_samples[-1].sequence_id + 1
        else:
            sample.sequence_id = 0
        self.controller_samples.append(sample)

    def add_lidar_frame(self, frame: LidarFrame):
        """Fügt LIDAR-Frame hinzu mit automatischer Frame-ID"""
        if self.lidar_frames:
            frame.frame_id = self.lidar_frames[-1].frame_id + 1
        else:
            frame.frame_id = 0
        self.lidar_frames.append(frame)

    def add_checkpoint(self, checkpoint: Checkpoint):
        """Fügt Checkpoint hinzu"""
        self.checkpoints.append(checkpoint)

    def get_stats(self) -> Dict[str, Any]:
        """Gibt Statistiken über die Aufzeichnung zurück"""
        if not self.controller_samples and not self.lidar_frames:
            return {"status": "empty"}

        stats = {
            "controller_samples": len(self.controller_samples),
            "lidar_frames": len(self.lidar_frames),
            "checkpoints": len(self.checkpoints),
            "duration_seconds": 0.0,
            "controller_sample_rate": 0.0,
            "lidar_sample_rate": 0.0
        }

        if self.controller_samples:
            start_time = self.controller_samples[0].timestamp
            end_time = self.controller_samples[-1].timestamp
            duration = end_time - start_time
            stats["duration_seconds"] = duration
            if duration > 0:
                stats["controller_sample_rate"] = len(self.controller_samples) / duration

        if self.lidar_frames:
            start_time = self.lidar_frames[0].timestamp
            end_time = self.lidar_frames[-1].timestamp  
            duration = end_time - start_time
            if duration > 0:
                stats["lidar_sample_rate"] = len(self.lidar_frames) / duration

        return stats

    def to_dict(self) -> Dict:
        """Konvertiert zu Dictionary für JSON-Serialisierung"""
        return {
            'header': self.header.to_dict(),
            'controller_samples': [sample.to_dict() for sample in self.controller_samples],
            'lidar_frames': [frame.to_dict() for frame in self.lidar_frames],
            'checkpoints': [checkpoint.to_dict() for checkpoint in self.checkpoints]
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'PathRecordingData':
        """Erstellt Instanz aus Dictionary"""
        header = PathRecordingHeader.from_dict(data['header'])
        controller_samples = [ControllerSample.from_dict(s) for s in data.get('controller_samples', [])]
        lidar_frames = [LidarFrame.from_dict(f) for f in data.get('lidar_frames', [])]
        checkpoints = [Checkpoint.from_dict(c) for c in data.get('checkpoints', [])]

        return cls(
            header=header,
            controller_samples=controller_samples,
            lidar_frames=lidar_frames,
            checkpoints=checkpoints
        )

# =============================================================================
# LOGGING KONFIGURATION
# =============================================================================

def setup_logging(log_file: str = "path_recording.log") -> logging.Logger:
    """Konfiguriert Logging für das System"""
    logger = logging.getLogger("PathRecording")
    logger.setLevel(logging.INFO)

    # File Handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)

    # Console Handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    # Formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger

print("✅ KORRIGIERTE Datenstrukturen erfolgreich definiert (v1.0.1)")
