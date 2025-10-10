#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Path Recording System - Datenstruktur und Speichersystem
=========================================================

Robustes System zur Datenaufzeichnung für Controller-Eingaben und LIDAR-Scans.

Features:
    - Zeitstempel-basierte Datenstruktur (10-50Hz Controller, 2-5Hz LIDAR)
    - Versionierte JSON/Binary-Speicherung mit Kompression
    - Metadaten-Verwaltung und Backup-Mechanismen
    - Checksum-Validierung und Recovery-Funktionen
    - Automatische Quality-Metriken für LIDAR-Daten
    - Thread-sichere Operationen

Version: 1.1.0
Datum: 2025-10-10
Autor: Robotics Development Team

Änderungen v1.1.0:
    - Detaillierte Docstrings für alle Klassen und Methoden
    - Verbesserte Typ-Annotationen
    - Optimierte Quality-Metriken Berechnung
    - Erweiterte Fehlerbehandlung
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
DATA_FORMAT_VERSION = "1.1.0"
HEADER_MAGIC = b'PREC'  # Path RECording magic bytes
CHUNK_MAGIC = b'CHNK'   # Chunk magic bytes

# Sample Rates
CONTROLLER_SAMPLE_RATE_HZ = 50  # 50Hz für Controller (20ms)
LIDAR_SAMPLE_RATE_HZ = 5        # 5Hz für LIDAR (200ms)


class CompressionType:
    """
    Unterstützte Kompressionstypen für die Datenspeicherung.
    
    Attributes:
        NONE: Keine Kompression (schnellste Verarbeitung, größte Dateien)
        GZIP: GZIP-Kompression (gute Balance zwischen Geschwindigkeit und Größe)
        LZMA: LZMA-Kompression (beste Kompression, langsamste Verarbeitung)
    """
    NONE = "none"
    GZIP = "gzip"
    LZMA = "lzma"


class DataType:
    """
    Datentypen für verschiedene Recording-Komponenten.
    
    Attributes:
        CONTROLLER: Controller-Eingabedaten
        LIDAR: LIDAR-Scandaten
        METADATA: Session-Metadaten
        CHECKPOINT: Wegpunkt-Daten
    """
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
    """
    Einzelne Controller-Eingabe mit Zeitstempel.
    
    Repräsentiert einen einzelnen Datenpunkt einer Controller-Eingabe mit allen
    relevanten Informationen wie Trigger-Positionen, Button-Zuständen und 
    Richtungseingaben.
    
    Attributes:
        timestamp: Unix-Zeitstempel mit Mikrosekunden-Genauigkeit
        left_trigger: Linker Trigger-Wert von -1.0 bis 1.0
        right_trigger: Rechter Trigger-Wert von -1.0 bis 1.0
        buttons: Dictionary mit Button-Namen als Keys und bool-Status als Values
        direction: Dictionary mit Richtungsachsen (x, y, rotation)
        sequence_id: Fortlaufende Nummer zur Lücken-Erkennung (default: 0)
    
    Example:
        >>> sample = ControllerSample(
        ...     timestamp=time.time(),
        ...     left_trigger=0.5,
        ...     right_trigger=-0.3,
        ...     buttons={'A': True, 'B': False},
        ...     direction={'x': 0.1, 'y': -0.2, 'rotation': 0.0}
        ... )
    """
    timestamp: float
    left_trigger: float
    right_trigger: float
    buttons: Dict[str, bool]
    direction: Dict[str, float]
    sequence_id: int = 0

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert die ControllerSample-Instanz zu einem Dictionary.
        
        Returns:
            Dictionary mit allen Attributen als Key-Value-Paare
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ControllerSample':
        """
        Erstellt eine ControllerSample-Instanz aus einem Dictionary.
        
        Args:
            data: Dictionary mit den Attributwerten
            
        Returns:
            Neue ControllerSample-Instanz
            
        Raises:
            TypeError: Wenn erforderliche Felder fehlen
        """
        return cls(**data)


@dataclass
class LidarFrame:
    """
    Einzelner LIDAR-Scan mit Quality-Metriken.
    
    Repräsentiert einen kompletten LIDAR-Scan inklusive aller Messpunkte und
    automatisch berechneter Qualitätsmetriken zur Datenvalidierung.
    
    Attributes:
        timestamp: Unix-Zeitstempel des Scans
        scan_data: Liste von Tupeln (quality, angle, distance) für jeden Messpunkt
        quality_metrics: Dictionary mit berechneten Qualitätsindikatoren
        frame_id: Fortlaufende Frame-Nummer zur Identifikation (default: 0)
    
    Quality Metrics:
        - valid_points: Anzahl gültiger Messpunkte (distance > 0)
        - total_points: Gesamtanzahl aller Messpunkte
        - coverage: Verhältnis gültige/total Punkte (0.0 bis 1.0)
        - avg_quality: Durchschnittliche Quality aller gültigen Punkte
        - min_distance: Minimale gemessene Distanz
        - max_distance: Maximale gemessene Distanz
    
    Example:
        >>> scan_data = [(0.95, 0.0, 1.5), (0.90, 45.0, 2.3), (0.85, 90.0, 3.1)]
        >>> frame = LidarFrame(timestamp=time.time(), scan_data=scan_data)
        >>> print(frame.quality_metrics['coverage'])
        1.0
    """
    timestamp: float
    scan_data: List[Tuple[float, float, float]]
    quality_metrics: Dict[str, float] = field(default_factory=dict)
    frame_id: int = 0

    def __post_init__(self):
        """
        Post-Initialisierungs-Hook zur automatischen Berechnung der Quality-Metriken.
        
        Wird automatisch nach der Instanziierung aufgerufen. Berechnet
        Quality-Metriken falls diese noch leer sind und Scan-Daten vorhanden.
        """
        if not self.quality_metrics and self.scan_data:
            self.quality_metrics = self._calculate_quality_metrics()

    def _calculate_quality_metrics(self) -> Dict[str, float]:
        """
        Berechnet Quality-Metriken aus den Scan-Daten.
        
        Analysiert die LIDAR-Scandaten und berechnet verschiedene
        Qualitätsindikatoren zur Bewertung der Scan-Qualität.
        
        Returns:
            Dictionary mit berechneten Quality-Metriken
            
        Note:
            Punkte mit distance <= 0 werden als ungültig betrachtet und
            aus den Berechnungen ausgeschlossen.
        """
        if not self.scan_data:
            return {
                "valid_points": 0,
                "total_points": 0,
                "coverage": 0.0,
                "avg_quality": 0.0,
                "min_distance": 0.0,
                "max_distance": 0.0
            }

        valid_points = sum(1 for q, a, d in self.scan_data if d > 0)
        total_points = len(self.scan_data)
        coverage = valid_points / total_points if total_points > 0 else 0.0
        
        valid_qualities = [q for q, a, d in self.scan_data if d > 0]
        avg_quality = sum(valid_qualities) / len(valid_qualities) if valid_qualities else 0.0
        
        valid_distances = [d for _, _, d in self.scan_data if d > 0]
        min_distance = min(valid_distances) if valid_distances else 0.0
        max_distance = max(valid_distances) if valid_distances else 0.0

        return {
            "valid_points": valid_points,
            "total_points": total_points,
            "coverage": coverage,
            "avg_quality": avg_quality,
            "min_distance": min_distance,
            "max_distance": max_distance
        }

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert die LidarFrame-Instanz zu einem Dictionary.
        
        Returns:
            Dictionary mit allen Attributen als Key-Value-Paare
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LidarFrame':
        """
        Erstellt eine LidarFrame-Instanz aus einem Dictionary.
        
        Args:
            data: Dictionary mit den Attributwerten
            
        Returns:
            Neue LidarFrame-Instanz mit automatisch berechneten Quality-Metriken
        """
        return cls(**data)


@dataclass
class Checkpoint:
    """
    Strategischer Wegpunkt für Path-Validierung.
    
    Ein Checkpoint markiert einen wichtigen Punkt in der Aufzeichnung und
    speichert den kompletten Zustand von Controller und LIDAR zu diesem Zeitpunkt.
    
    Attributes:
        timestamp: Unix-Zeitstempel des Checkpoints
        position: Tuple mit (x, y, orientation) in Metern und Radiant
        controller_state: Kompletter Controller-Zustand als ControllerSample
        lidar_snapshot: Kompletter LIDAR-Scan als LidarFrame
        checkpoint_id: Eindeutige Checkpoint-Identifikation
        notes: Optionale Beschreibung/Notizen zum Checkpoint (default: "")
    
    Example:
        >>> checkpoint = Checkpoint(
        ...     timestamp=time.time(),
        ...     position=(1.5, 2.3, 0.785),
        ...     controller_state=controller_sample,
        ...     lidar_snapshot=lidar_frame,
        ...     checkpoint_id="START_POINT",
        ...     notes="Startpunkt der Testfahrt"
        ... )
    """
    timestamp: float
    position: Tuple[float, float, float]
    controller_state: ControllerSample
    lidar_snapshot: LidarFrame
    checkpoint_id: str
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert den Checkpoint zu einem Dictionary.
        
        Serialisiert den Checkpoint inklusive verschachtelter Controller-
        und LIDAR-Daten für die Speicherung.
        
        Returns:
            Dictionary mit allen Attributen und serialisierten Unterobjekten
        """
        data = asdict(self)
        data['controller_state'] = self.controller_state.to_dict()
        data['lidar_snapshot'] = self.lidar_snapshot.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Checkpoint':
        """
        Erstellt einen Checkpoint aus einem Dictionary.
        
        Deserialisiert den Checkpoint inklusive verschachtelter Controller-
        und LIDAR-Daten aus der Speicherung.
        
        Args:
            data: Dictionary mit den Attributwerten
            
        Returns:
            Neue Checkpoint-Instanz mit rekonstruierten Unterobjekten
        """
        controller_data = data.pop('controller_state')
        lidar_data = data.pop('lidar_snapshot')
        return cls(
            controller_state=ControllerSample.from_dict(controller_data),
            lidar_snapshot=LidarFrame.from_dict(lidar_data),
            **data
        )


@dataclass
class CalibrationData:
    """
    Kalibrierungsdaten für Controller und LIDAR.
    
    Speichert alle Kalibrierungsparameter, die zur Korrektur von Hardware-
    spezifischen Offsets und Abweichungen benötigt werden.
    
    Attributes:
        controller_calibration: Dictionary mit Controller-spezifischen Parametern
                              (z.B. Trigger-Offsets, Button-Delays, Dead-Zones)
        lidar_calibration: Dictionary mit LIDAR-spezifischen Parametern
                          (z.B. Winkel-Offsets, Distanz-Korrekturen)
        calibration_timestamp: Zeitstempel der Kalibrierung
        calibration_quality: Qualitätsbewertung 0.0-1.0 (default: 1.0)
    
    Example:
        >>> calib = CalibrationData(
        ...     controller_calibration={
        ...         'left_trigger_offset': 0.01,
        ...         'right_trigger_offset': -0.02,
        ...         'deadzone': 0.05
        ...     },
        ...     lidar_calibration={
        ...         'angle_offset': 0.5,
        ...         'distance_scale': 1.002
        ...     },
        ...     calibration_timestamp=time.time(),
        ...     calibration_quality=0.95
        ... )
    """
    controller_calibration: Dict[str, float]
    lidar_calibration: Dict[str, float]
    calibration_timestamp: float
    calibration_quality: float = 1.0

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert die Kalibrierungsdaten zu einem Dictionary.
        
        Returns:
            Dictionary mit allen Kalibierungsparametern
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CalibrationData':
        """
        Erstellt CalibrationData-Instanz aus einem Dictionary.
        
        Args:
            data: Dictionary mit den Kalibierungsparametern
            
        Returns:
            Neue CalibrationData-Instanz
        """
        return cls(**data)


@dataclass
class EnvironmentMetadata:
    """
    Umgebungsbedingungen und Session-Metadaten.
    
    Speichert alle relevanten Informationen über die Aufzeichnungs-Session,
    Hardware-Konfiguration und Umgebungsbedingungen.
    
    Attributes:
        session_id: Eindeutige Session-Identifikation (z.B. UUID oder Zeitstempel)
        start_timestamp: Start-Zeitstempel der Aufzeichnung
        end_timestamp: End-Zeitstempel der Aufzeichnung (Optional, None während Aufnahme)
        environment_conditions: Dictionary mit Umgebungsparametern
                               (z.B. Temperatur, Luftfeuchtigkeit, Lichtverhältnisse)
        hardware_info: Dictionary mit Hardware-Informationen
                      (z.B. Controller-ID, LIDAR-Model, Firmware-Versionen)
        software_version: Version des Recording-Systems (default: DATA_FORMAT_VERSION)
        recording_settings: Dictionary mit Recording-Einstellungen
                           (z.B. Sample-Rates, Kompression, Buffer-Größen)
    
    Example:
        >>> metadata = EnvironmentMetadata(
        ...     session_id="2025-10-10_203000_robot01",
        ...     start_timestamp=time.time(),
        ...     environment_conditions={
        ...         'temperature_celsius': 22.5,
        ...         'humidity_percent': 45.0,
        ...         'ambient_light_lux': 350
        ...     },
        ...     hardware_info={
        ...         'controller_model': 'Xbox Wireless Controller',
        ...         'lidar_model': 'RPLIDAR C1',
        ...         'lidar_firmware': '1.29'
        ...     },
        ...     recording_settings={
        ...         'controller_rate_hz': 50,
        ...         'lidar_rate_hz': 5,
        ...         'compression': 'gzip'
        ...     }
        ... )
    """
    session_id: str
    start_timestamp: float
    end_timestamp: Optional[float] = None
    environment_conditions: Dict[str, Any] = field(default_factory=dict)
    hardware_info: Dict[str, str] = field(default_factory=dict)
    software_version: str = DATA_FORMAT_VERSION
    recording_settings: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert die Metadaten zu einem Dictionary.
        
        Returns:
            Dictionary mit allen Metadaten
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'EnvironmentMetadata':
        """
        Erstellt EnvironmentMetadata-Instanz aus einem Dictionary.
        
        Args:
            data: Dictionary mit den Metadaten
            
        Returns:
            Neue EnvironmentMetadata-Instanz
        """
        return cls(**data)


@dataclass
class PathRecordingHeader:
    """
    Header-Information für Path Recording Dateien.
    
    Der Header enthält alle Meta-Informationen über die Aufzeichnungsdatei
    und ermöglicht die Validierung und korrekte Interpretation der Daten.
    
    Attributes:
        magic: Magic Bytes zur Dateityp-Identifikation (default: HEADER_MAGIC)
        version: Datenformat-Version (default: DATA_FORMAT_VERSION)
        timestamp: Erstellungszeitpunkt der Datei
        calibration_data: Optionale Kalibrierungsdaten
        metadata: Optionale Session-Metadaten
        checksum_algorithm: Verwendeter Checksum-Algorithmus (default: "sha256")
        compression_type: Verwendeter Kompressionstyp (default: CompressionType.GZIP)
    
    Example:
        >>> header = PathRecordingHeader(
        ...     calibration_data=calibration,
        ...     metadata=environment_metadata,
        ...     compression_type=CompressionType.GZIP
        ... )
    """
    magic: bytes = HEADER_MAGIC
    version: str = DATA_FORMAT_VERSION
    timestamp: float = field(default_factory=time.time)
    calibration_data: Optional[CalibrationData] = None
    metadata: Optional[EnvironmentMetadata] = None
    checksum_algorithm: str = "sha256"
    compression_type: str = CompressionType.GZIP

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert den Header zu einem Dictionary.
        
        Serialisiert den Header inklusive verschachtelter Kalibrierungs-
        und Metadaten für die Speicherung.
        
        Returns:
            Dictionary mit allen Header-Informationen
        """
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
    def from_dict(cls, data: Dict[str, Any]) -> 'PathRecordingHeader':
        """
        Erstellt einen Header aus einem Dictionary.
        
        Deserialisiert den Header inklusive verschachtelter Kalibrierungs-
        und Metadaten aus der Speicherung.
        
        Args:
            data: Dictionary mit den Header-Informationen
            
        Returns:
            Neue PathRecordingHeader-Instanz mit rekonstruierten Unterobjekten
        """
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
            **{k: v for k, v in data.items() if k in ['magic', 'version', 'timestamp', 
                                                       'checksum_algorithm', 'compression_type']}
        )


@dataclass
class PathRecordingData:
    """
    Hauptdatencontainer für komplette Aufzeichnung.
    
    Zentraler Container, der alle Aufzeichnungsdaten zusammenhält:
    Header-Informationen, Controller-Samples, LIDAR-Frames und Checkpoints.
    Bietet Methoden zum Hinzufügen von Daten und zur Statistikberechnung.
    
    Attributes:
        header: PathRecordingHeader mit allen Meta-Informationen
        controller_samples: Liste aller aufgezeichneten Controller-Samples
        lidar_frames: Liste aller aufgezeichneten LIDAR-Frames
        checkpoints: Liste aller gesetzten Checkpoints
    
    Example:
        >>> # Initialisierung
        >>> header = PathRecordingHeader(metadata=session_metadata)
        >>> recording = PathRecordingData(header=header)
        >>> 
        >>> # Daten hinzufügen
        >>> recording.add_controller_sample(controller_sample)
        >>> recording.add_lidar_frame(lidar_frame)
        >>> recording.add_checkpoint(checkpoint)
        >>> 
        >>> # Statistiken abrufen
        >>> stats = recording.get_stats()
        >>> print(f"Aufnahmedauer: {stats['duration_seconds']:.2f}s")
    """
    header: PathRecordingHeader
    controller_samples: List[ControllerSample] = field(default_factory=list)
    lidar_frames: List[LidarFrame] = field(default_factory=list)
    checkpoints: List[Checkpoint] = field(default_factory=list)

    def add_controller_sample(self, sample: ControllerSample) -> None:
        """
        Fügt ein Controller-Sample mit automatischer Sequence-ID hinzu.
        
        Die Sequence-ID wird automatisch inkrementiert basierend auf dem
        letzten Sample. Bei dem ersten Sample wird die ID auf 0 gesetzt.
        
        Args:
            sample: ControllerSample-Instanz (sequence_id wird überschrieben)
            
        Note:
            Die Sequence-ID im übergebenen Sample wird automatisch gesetzt
            und muss nicht vorher konfiguriert werden.
        """
        if self.controller_samples:
            sample.sequence_id = self.controller_samples[-1].sequence_id + 1
        else:
            sample.sequence_id = 0
        self.controller_samples.append(sample)

    def add_lidar_frame(self, frame: LidarFrame) -> None:
        """
        Fügt einen LIDAR-Frame mit automatischer Frame-ID hinzu.
        
        Die Frame-ID wird automatisch inkrementiert basierend auf dem
        letzten Frame. Bei dem ersten Frame wird die ID auf 0 gesetzt.
        
        Args:
            frame: LidarFrame-Instanz (frame_id wird überschrieben)
            
        Note:
            Die Frame-ID im übergebenen Frame wird automatisch gesetzt
            und muss nicht vorher konfiguriert werden.
        """
        if self.lidar_frames:
            frame.frame_id = self.lidar_frames[-1].frame_id + 1
        else:
            frame.frame_id = 0
        self.lidar_frames.append(frame)

    def add_checkpoint(self, checkpoint: Checkpoint) -> None:
        """
        Fügt einen Checkpoint zur Aufzeichnung hinzu.
        
        Args:
            checkpoint: Checkpoint-Instanz mit allen erforderlichen Daten
        """
        self.checkpoints.append(checkpoint)

    def get_stats(self) -> Dict[str, Any]:
        """
        Berechnet und gibt Statistiken über die Aufzeichnung zurück.
        
        Analysiert die aufgezeichneten Daten und berechnet verschiedene
        Statistiken wie Anzahl der Samples, Aufnahmedauer und Sample-Raten.
        
        Returns:
            Dictionary mit folgenden Keys:
            - status: "empty" wenn keine Daten vorhanden
            - controller_samples: Anzahl Controller-Samples
            - lidar_frames: Anzahl LIDAR-Frames
            - checkpoints: Anzahl Checkpoints
            - duration_seconds: Gesamtdauer der Aufzeichnung in Sekunden
            - controller_sample_rate: Durchschnittliche Controller-Sample-Rate in Hz
            - lidar_sample_rate: Durchschnittliche LIDAR-Sample-Rate in Hz
            
        Example:
            >>> stats = recording.get_stats()
            >>> print(f"Samples: {stats['controller_samples']}")
            >>> print(f"Rate: {stats['controller_sample_rate']:.1f} Hz")
        """
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

        # Berechne Controller-Statistiken
        if self.controller_samples:
            start_time = self.controller_samples[0].timestamp
            end_time = self.controller_samples[-1].timestamp
            duration = end_time - start_time
            stats["duration_seconds"] = duration
            if duration > 0:
                stats["controller_sample_rate"] = len(self.controller_samples) / duration

        # Berechne LIDAR-Statistiken
        if self.lidar_frames:
            start_time = self.lidar_frames[0].timestamp
            end_time = self.lidar_frames[-1].timestamp
            duration = end_time - start_time
            if duration > 0:
                stats["lidar_sample_rate"] = len(self.lidar_frames) / duration

        return stats

    def to_dict(self) -> Dict[str, Any]:
        """
        Konvertiert die komplette Aufzeichnung zu einem Dictionary.
        
        Serialisiert alle Daten inklusive Header, Samples, Frames und
        Checkpoints für die JSON-Speicherung.
        
        Returns:
            Dictionary mit allen Aufzeichnungsdaten
        """
        return {
            'header': self.header.to_dict(),
            'controller_samples': [sample.to_dict() for sample in self.controller_samples],
            'lidar_frames': [frame.to_dict() for frame in self.lidar_frames],
            'checkpoints': [checkpoint.to_dict() for checkpoint in self.checkpoints]
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'PathRecordingData':
        """
        Erstellt eine PathRecordingData-Instanz aus einem Dictionary.
        
        Deserialisiert eine komplette Aufzeichnung aus einem Dictionary,
        wie es von to_dict() erstellt wurde.
        
        Args:
            data: Dictionary mit allen Aufzeichnungsdaten
            
        Returns:
            Neue PathRecordingData-Instanz mit allen rekonstruierten Daten
        """
        header = PathRecordingHeader.from_dict(data['header'])
        controller_samples = [
            ControllerSample.from_dict(s) 
            for s in data.get('controller_samples', [])
        ]
        lidar_frames = [
            LidarFrame.from_dict(f) 
            for f in data.get('lidar_frames', [])
        ]
        checkpoints = [
            Checkpoint.from_dict(c) 
            for c in data.get('checkpoints', [])
        ]
        
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
    """
    Konfiguriert das Logging-System für Path Recording.
    
    Erstellt einen Logger mit File- und Console-Handlern für strukturiertes
    Logging aller System-Ereignisse.
    
    Args:
        log_file: Pfad zur Log-Datei (default: "path_recording.log")
        
    Returns:
        Konfigurierter Logger mit File- und Console-Output
        
    Example:
        >>> logger = setup_logging("my_recording.log")
        >>> logger.info("Recording started")
        >>> logger.warning("Low battery")
        >>> logger.error("Sensor disconnected")
    """
    logger = logging.getLogger("PathRecording")
    logger.setLevel(logging.INFO)

    # File Handler für detailliertes Logging
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)

    # Console Handler für wichtige Meldungen
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    # Formatter mit Zeitstempel und Kontext
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # Verhindere doppelte Handler bei mehrfachem Aufruf
    if not logger.handlers:
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

    return logger


# Erfolgreiche Modul-Initialisierung
print("✅ Path Recording Datenstrukturen erfolgreich geladen (v1.1.0)")
