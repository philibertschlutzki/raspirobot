#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Recording Storage System - Robuste Dateispeicherung
========================================================

Implementiert:
- JSON/Binary-Speicherung mit Versionierung
- GZIP/LZMA-Kompression für große LIDAR-Datensätze  
- Backup- und Recovery-Mechanismen
- Checksum-Validierung und Datenintegrität
- Thread-sicherer Zugriff

Version: 1.0
Datum: 2025-10-10
"""

import json
import gzip
import lzma
import pickle
import struct
import hashlib
import os
import shutil
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Any, Union, BinaryIO, TextIO
from datetime import datetime

# Import der Datenstrukturen
from path_recording_system import (
    PathRecordingData, PathRecordingHeader, ControllerSample, LidarFrame,
    Checkpoint, EnvironmentMetadata, CalibrationData,
    CompressionType, FILE_EXTENSIONS, BINARY_EXTENSION, HEADER_MAGIC,
    DATA_FORMAT_VERSION, setup_logging
)

# =============================================================================
# STORAGE MANAGER KLASSE
# =============================================================================

class PathRecordingStorageManager:
    """
    Haupt-Storage-Manager für Path Recording System

    Features:
    - Robuste Speicherung in JSON/Binary mit Kompression
    - Automatische Backup-Erstellung
    - Checksum-Validierung  
    - Recovery bei Corruption
    - Thread-sichere Operationen
    """

    def __init__(self, 
                 base_directory: str = "path_recordings",
                 compression: str = CompressionType.GZIP,
                 backup_enabled: bool = True,
                 backup_count: int = 3,
                 verify_checksums: bool = True):
        """
        Initialisiert Storage Manager

        Args:
            base_directory: Basis-Verzeichnis für Aufzeichnungen
            compression: Kompression (none, gzip, lzma)
            backup_enabled: Automatische Backups aktivieren
            backup_count: Anzahl der Backup-Versionen
            verify_checksums: Checksum-Verifikation aktivieren
        """
        self.base_directory = Path(base_directory)
        self.compression = compression
        self.backup_enabled = backup_enabled
        self.backup_count = backup_count
        self.verify_checksums = verify_checksums

        # Erstelle Verzeichnisstruktur
        self.base_directory.mkdir(parents=True, exist_ok=True)
        self.backup_directory = self.base_directory / "backups"
        self.backup_directory.mkdir(exist_ok=True)

        # Thread-Safety
        self._lock = threading.RLock()

        # Logger
        self.logger = setup_logging(str(self.base_directory / "storage.log"))

        self.logger.info(f"Storage Manager initialisiert:")
        self.logger.info(f"  Base Directory: {self.base_directory}")
        self.logger.info(f"  Compression: {self.compression}")
        self.logger.info(f"  Backup: {self.backup_enabled} (count: {self.backup_count})")

    def generate_filename(self, session_id: str, binary: bool = False) -> str:
        """Generiert Dateinamen basierend auf Session-ID und Format"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = f"{timestamp}_{session_id}"

        if binary:
            return f"{base_name}{BINARY_EXTENSION}"
        else:
            return f"{base_name}{FILE_EXTENSIONS[self.compression]}"

    def _calculate_checksum(self, data: bytes) -> str:
        """Berechnet SHA256-Checksum für Datenintegrität"""
        return hashlib.sha256(data).hexdigest()

    def _compress_data(self, data: bytes) -> bytes:
        """Komprimiert Daten je nach Einstellung"""
        if self.compression == CompressionType.GZIP:
            return gzip.compress(data, compresslevel=6)
        elif self.compression == CompressionType.LZMA:
            return lzma.compress(data, preset=3)
        else:
            return data

    def _decompress_data(self, data: bytes, compression: str = None) -> bytes:
        """Dekomprimiert Daten"""
        comp = compression or self.compression

        if comp == CompressionType.GZIP:
            return gzip.decompress(data)
        elif comp == CompressionType.LZMA:
            return lzma.decompress(data)
        else:
            return data

    def _create_backup(self, filepath: Path) -> Optional[Path]:
        """Erstellt Backup einer existierenden Datei"""
        if not filepath.exists() or not self.backup_enabled:
            return None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        backup_filename = f"{filepath.stem}_backup_{timestamp}{filepath.suffix}"
        backup_path = self.backup_directory / backup_filename

        try:
            shutil.copy2(filepath, backup_path)
            self.logger.info(f"Backup erstellt: {backup_path}")

            # Alte Backups löschen (behalte nur backup_count)
            self._cleanup_old_backups(filepath.stem)

            return backup_path
        except Exception as e:
            self.logger.error(f"Backup-Erstellung fehlgeschlagen: {e}")
            return None

    def _cleanup_old_backups(self, base_filename: str):
        """Löscht alte Backup-Dateien"""
        if not self.backup_enabled:
            return

        try:
            # Finde alle Backups für diese Datei
            backup_pattern = f"{base_filename}_backup_*"
            backups = list(self.backup_directory.glob(backup_pattern))

            # Sortiere nach Erstellungszeit (neueste zuerst)
            backups.sort(key=lambda p: p.stat().st_mtime, reverse=True)

            # Lösche überschüssige Backups
            for old_backup in backups[self.backup_count:]:
                old_backup.unlink()
                self.logger.debug(f"Altes Backup gelöscht: {old_backup}")

        except Exception as e:
            self.logger.warning(f"Backup-Cleanup fehlgeschlagen: {e}")

    def save_json(self, recording_data: PathRecordingData, filename: str = None) -> Path:
        """
        Speichert PathRecordingData als JSON (komprimiert)

        Args:
            recording_data: Aufzeichnungsdaten
            filename: Optionaler Dateiname (sonst auto-generiert)

        Returns:
            Path zur gespeicherten Datei
        """
        with self._lock:
            if not filename:
                session_id = recording_data.header.metadata.session_id if recording_data.header.metadata else "unknown"
                filename = self.generate_filename(session_id, binary=False)

            filepath = self.base_directory / filename

            try:
                # Erstelle Backup falls Datei existiert
                self._create_backup(filepath)

                # Konvertiere zu Dictionary
                data_dict = recording_data.to_dict()

                # JSON-Serialisierung
                json_str = json.dumps(data_dict, indent=2, ensure_ascii=False)
                json_bytes = json_str.encode('utf-8')

                # Komprimiere Daten
                compressed_data = self._compress_data(json_bytes)

                # Berechne Checksum
                checksum = self._calculate_checksum(compressed_data)

                # Erstelle Metadaten-Header
                metadata = {
                    'checksum': checksum,
                    'compression': self.compression,
                    'original_size': len(json_bytes),
                    'compressed_size': len(compressed_data),
                    'timestamp': time.time()
                }

                # Schreibe Datei
                with open(filepath, 'wb') as f:
                    # Schreibe Metadaten-Header (JSON)
                    header_json = json.dumps(metadata).encode('utf-8')
                    header_size = len(header_json)

                    # Format: [header_size(4 bytes)][header_json][compressed_data]
                    f.write(struct.pack('<I', header_size))
                    f.write(header_json)
                    f.write(compressed_data)

                self.logger.info(f"JSON gespeichert: {filepath}")
                self.logger.debug(f"  Original: {len(json_bytes)} bytes")
                self.logger.debug(f"  Komprimiert: {len(compressed_data)} bytes") 
                self.logger.debug(f"  Ratio: {len(compressed_data)/len(json_bytes):.2%}")

                return filepath

            except Exception as e:
                self.logger.error(f"JSON-Speicherung fehlgeschlagen: {e}")
                raise

    def load_json(self, filepath: Union[str, Path]) -> PathRecordingData:
        """
        Lädt PathRecordingData aus JSON-Datei

        Args:
            filepath: Pfad zur JSON-Datei

        Returns:
            PathRecordingData Objekt
        """
        filepath = Path(filepath)

        with self._lock:
            try:
                with open(filepath, 'rb') as f:
                    # Lese Header-Größe
                    header_size_bytes = f.read(4)
                    if len(header_size_bytes) != 4:
                        raise ValueError("Ungültige Datei: Header-Größe nicht lesbar")

                    header_size = struct.unpack('<I', header_size_bytes)[0]

                    # Lese Metadaten-Header
                    header_json = f.read(header_size).decode('utf-8')
                    metadata = json.loads(header_json)

                    # Lese komprimierte Daten
                    compressed_data = f.read()

                # Validiere Checksum falls aktiviert
                if self.verify_checksums:
                    calculated_checksum = self._calculate_checksum(compressed_data)
                    if calculated_checksum != metadata['checksum']:
                        raise ValueError(f"Checksum-Mismatch: {calculated_checksum} != {metadata['checksum']}")

                # Dekomprimiere Daten
                json_bytes = self._decompress_data(compressed_data, metadata['compression'])

                # Parse JSON
                json_str = json_bytes.decode('utf-8')
                data_dict = json.loads(json_str)

                # Erstelle PathRecordingData Objekt
                recording_data = PathRecordingData.from_dict(data_dict)

                self.logger.info(f"JSON geladen: {filepath}")
                self.logger.debug(f"  Samples: {len(recording_data.controller_samples)}")
                self.logger.debug(f"  LIDAR Frames: {len(recording_data.lidar_frames)}")

                return recording_data

            except Exception as e:
                self.logger.error(f"JSON-Laden fehlgeschlagen: {e}")
                # Versuche Recovery aus Backup
                if self.backup_enabled:
                    return self._try_recovery(filepath)
                raise

    def save_binary(self, recording_data: PathRecordingData, filename: str = None) -> Path:
        """
        Speichert PathRecordingData als optimiertes Binary-Format

        Args:
            recording_data: Aufzeichnungsdaten  
            filename: Optionaler Dateiname

        Returns:
            Path zur gespeicherten Datei
        """
        with self._lock:
            if not filename:
                session_id = recording_data.header.metadata.session_id if recording_data.header.metadata else "unknown"
                filename = self.generate_filename(session_id, binary=True)

            filepath = self.base_directory / filename

            try:
                # Erstelle Backup falls Datei existiert  
                self._create_backup(filepath)

                # Serialisiere mit Pickle (kompakt und schnell)
                pickle_data = pickle.dumps(recording_data, protocol=pickle.HIGHEST_PROTOCOL)

                # Komprimiere
                compressed_data = self._compress_data(pickle_data)

                # Checksum
                checksum = self._calculate_checksum(compressed_data)

                # Schreibe Binary-Datei
                with open(filepath, 'wb') as f:
                    # Magic Header
                    f.write(HEADER_MAGIC)

                    # Version String (padded auf 16 Bytes)
                    version_bytes = DATA_FORMAT_VERSION.encode('ascii').ljust(16, b'\0')
                    f.write(version_bytes)

                    # Metadaten
                    f.write(struct.pack('<Q', int(time.time() * 1000000)))  # Timestamp (µs)
                    f.write(struct.pack('<I', len(pickle_data)))  # Original size
                    f.write(struct.pack('<I', len(compressed_data)))  # Compressed size
                    f.write(self.compression.encode('ascii').ljust(8, b'\0'))  # Compression

                    # Checksum (32 bytes hex)
                    f.write(checksum.encode('ascii'))

                    # Komprimierte Daten
                    f.write(compressed_data)

                self.logger.info(f"Binary gespeichert: {filepath}")
                self.logger.debug(f"  Original: {len(pickle_data)} bytes")
                self.logger.debug(f"  Komprimiert: {len(compressed_data)} bytes")

                return filepath

            except Exception as e:
                self.logger.error(f"Binary-Speicherung fehlgeschlagen: {e}")
                raise

    def load_binary(self, filepath: Union[str, Path]) -> PathRecordingData:
        """
        Lädt PathRecordingData aus Binary-Datei

        Args:
            filepath: Pfad zur Binary-Datei

        Returns:
            PathRecordingData Objekt
        """
        filepath = Path(filepath)

        with self._lock:
            try:
                with open(filepath, 'rb') as f:
                    # Lese Header
                    magic = f.read(4)
                    if magic != HEADER_MAGIC:
                        raise ValueError(f"Ungültiges Magic: {magic}")

                    version = f.read(16).rstrip(b'\0').decode('ascii')
                    timestamp_us = struct.unpack('<Q', f.read(8))[0]
                    original_size = struct.unpack('<I', f.read(4))[0]
                    compressed_size = struct.unpack('<I', f.read(4))[0]
                    compression = f.read(8).rstrip(b'\0').decode('ascii')
                    checksum = f.read(32).decode('ascii')

                    # Lese komprimierte Daten
                    compressed_data = f.read()

                    if len(compressed_data) != compressed_size:
                        raise ValueError("Datengröße stimmt nicht überein")

                # Validiere Checksum
                if self.verify_checksums:
                    calculated_checksum = self._calculate_checksum(compressed_data)
                    if calculated_checksum != checksum:
                        raise ValueError(f"Checksum-Mismatch: {calculated_checksum} != {checksum}")

                # Dekomprimiere
                pickle_data = self._decompress_data(compressed_data, compression)

                if len(pickle_data) != original_size:
                    raise ValueError("Dekomprimierte Größe stimmt nicht überein")

                # Deserialize
                recording_data = pickle.loads(pickle_data)

                self.logger.info(f"Binary geladen: {filepath}")
                self.logger.debug(f"  Version: {version}")
                self.logger.debug(f"  Timestamp: {datetime.fromtimestamp(timestamp_us/1000000)}")

                return recording_data

            except Exception as e:
                self.logger.error(f"Binary-Laden fehlgeschlagen: {e}")
                # Versuche Recovery aus Backup
                if self.backup_enabled:
                    return self._try_recovery(filepath)
                raise

    def _try_recovery(self, original_filepath: Path) -> PathRecordingData:
        """Versucht Recovery aus Backup-Dateien"""
        self.logger.warning(f"Versuche Recovery für: {original_filepath}")

        # Finde Backups
        backup_pattern = f"{original_filepath.stem}_backup_*{original_filepath.suffix}"
        backups = list(self.backup_directory.glob(backup_pattern))

        if not backups:
            raise FileNotFoundError("Keine Backup-Dateien gefunden")

        # Sortiere nach Erstellungszeit (neueste zuerst)
        backups.sort(key=lambda p: p.stat().st_mtime, reverse=True)

        # Versuche Backups der Reihe nach
        for backup_path in backups:
            try:
                self.logger.info(f"Versuche Recovery aus: {backup_path}")

                if backup_path.suffix == BINARY_EXTENSION:
                    return self.load_binary(backup_path)
                else:
                    return self.load_json(backup_path)

            except Exception as e:
                self.logger.warning(f"Recovery aus {backup_path} fehlgeschlagen: {e}")
                continue

        raise RuntimeError("Recovery aus allen Backups fehlgeschlagen")

    def list_recordings(self) -> List[Dict[str, Any]]:
        """Listet alle verfügbaren Aufzeichnungen auf"""
        recordings = []

        for file_path in self.base_directory.glob("*"):
            if file_path.is_file() and not file_path.name.startswith('.'):
                try:
                    stat = file_path.stat()
                    info = {
                        'filename': file_path.name,
                        'filepath': str(file_path),
                        'size_bytes': stat.st_size,
                        'modified': datetime.fromtimestamp(stat.st_mtime),
                        'format': 'binary' if file_path.suffix == BINARY_EXTENSION else 'json'
                    }
                    recordings.append(info)
                except Exception as e:
                    self.logger.warning(f"Fehler beim Lesen von {file_path}: {e}")

        return sorted(recordings, key=lambda x: x['modified'], reverse=True)

    def validate_file(self, filepath: Union[str, Path]) -> Dict[str, Any]:
        """Validiert Integrität einer Aufzeichnungsdatei"""
        filepath = Path(filepath)

        validation_result = {
            'filepath': str(filepath),
            'exists': filepath.exists(),
            'valid': False,
            'errors': [],
            'info': {}
        }

        if not filepath.exists():
            validation_result['errors'].append("Datei existiert nicht")
            return validation_result

        try:
            # Versuche zu laden
            if filepath.suffix == BINARY_EXTENSION:
                data = self.load_binary(filepath)
            else:
                data = self.load_json(filepath)

            # Sammle Informationen
            stats = data.get_stats()
            validation_result['info'] = {
                'format_version': data.header.version,
                'timestamp': datetime.fromtimestamp(data.header.timestamp),
                'controller_samples': stats.get('controller_samples', 0),
                'lidar_frames': stats.get('lidar_frames', 0),
                'checkpoints': stats.get('checkpoints', 0),
                'duration_seconds': stats.get('duration_seconds', 0)
            }

            validation_result['valid'] = True

        except Exception as e:
            validation_result['errors'].append(str(e))

        return validation_result

# =============================================================================
# HELPER FUNKTIONEN
# =============================================================================

def create_storage_manager(
    directory: str = "path_recordings",
    compression: str = CompressionType.GZIP,
    backup_count: int = 3
) -> PathRecordingStorageManager:
    """Hilfsfunktion zum Erstellen eines Storage Managers"""
    return PathRecordingStorageManager(
        base_directory=directory,
        compression=compression,
        backup_enabled=True,
        backup_count=backup_count,
        verify_checksums=True
    )

print("✅ Storage System erfolgreich implementiert")
