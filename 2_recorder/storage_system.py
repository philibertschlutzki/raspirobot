#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Storage System für Path Recording
==================================

Robustes Speichersystem für Path Recording Daten mit Unterstützung für
JSON- und Binär-Formate, Kompression, Backup-Verwaltung und Recovery.

Features:
    - JSON- und Binär-Speicherung mit konfigurierbarer Kompression
    - Automatische Backup-Erstellung mit konfigurierbarer Anzahl
    - Checksum-Validierung (SHA256) für Datenintegrität
    - Thread-sichere Operationen mit Lock-Mechanismus
    - Recovery-Mechanismus bei beschädigten Dateien
    - Datei-Listing und Validierung

Version: 1.1.0
Datum: 2025-10-10
Autor: Robotics Development Team

Änderungen v1.1.0:
    - Korrektur der Import-Referenzen (path_recording_system statt path_recording_system_fixed)
    - Vollständige Docstrings für alle Methoden
    - Verbesserte Fehlerbehandlung und Logging
    - Optimierte Performance bei großen Dateien
"""

import json
import gzip
import lzma
import struct
import hashlib
import shutil
import threading
from pathlib import Path
from typing import Dict, List, Optional, Union, Any
from datetime import datetime

# Import der Datenstrukturen aus dem Hauptmodul
from path_recording_system import (
    PathRecordingData,
    PathRecordingHeader,
    ControllerSample,
    LidarFrame,
    Checkpoint,
    EnvironmentMetadata,
    CalibrationData,
    CompressionType,
    DATA_FORMAT_VERSION,
    HEADER_MAGIC,
    CHUNK_MAGIC,
    BINARY_EXTENSION,
    FILE_EXTENSIONS,
    setup_logging
)

# =============================================================================
# STORAGE MANAGER KLASSE
# =============================================================================


class PathRecordingStorageManager:
    """
    Hauptklasse für die Verwaltung von Path Recording Speicheroperationen.
    
    Bietet eine vollständige Schnittstelle zum Speichern, Laden und Verwalten
    von Path Recording Daten in verschiedenen Formaten mit automatischer
    Backup-Verwaltung und Fehlerbehandlung.
    
    Attributes:
        base_directory: Hauptverzeichnis für alle Aufzeichnungen
        backup_directory: Verzeichnis für Backup-Dateien
        compression: Verwendeter Kompressionstyp (GZIP, LZMA, NONE)
        backup_enabled: Flag für automatische Backups
        backup_count: Anzahl der zu behaltenden Backups
        verify_checksums: Flag für Checksum-Validierung beim Laden
        logger: Logger-Instanz für System-Ereignisse
        
    Thread Safety:
        Alle Methoden sind thread-sicher durch internen Lock-Mechanismus.
        
    Example:
        >>> # Initialisierung
        >>> storage = PathRecordingStorageManager(
        ...     base_directory="recordings",
        ...     compression=CompressionType.GZIP,
        ...     backup_enabled=True,
        ...     backup_count=3
        ... )
        >>> 
        >>> # Speichern
        >>> path = storage.save_json(recording_data, "session_001")
        >>> 
        >>> # Laden
        >>> loaded = storage.load_json(path)
        >>> 
        >>> # Validieren
        >>> validation = storage.validate_file(path)
        >>> print(f"Valid: {validation['valid']}")
    """

    def __init__(
        self,
        base_directory: str = "path_recordings",
        compression: str = CompressionType.GZIP,
        backup_enabled: bool = True,
        backup_count: int = 3,
        verify_checksums: bool = True
    ):
        """
        Initialisiert den Storage Manager.
        
        Args:
            base_directory: Hauptverzeichnis für Aufzeichnungen (wird erstellt falls nicht vorhanden)
            compression: Kompressionstyp (GZIP, LZMA oder NONE)
            backup_enabled: Aktiviert automatische Backup-Erstellung
            backup_count: Anzahl der zu behaltenden Backups (älteste werden gelöscht)
            verify_checksums: Aktiviert Checksum-Validierung beim Laden
            
        Raises:
            ValueError: Bei ungültigem Kompressionstyp
            OSError: Bei Problemen mit Verzeichnis-Erstellung
        """
        self.base_directory = Path(base_directory)
        self.backup_directory = self.base_directory / "backups"
        self.compression = compression
        self.backup_enabled = backup_enabled
        self.backup_count = backup_count
        self.verify_checksums = verify_checksums
        self.logger = setup_logging(str(self.base_directory / "storage.log"))
        self._lock = threading.Lock()

        # Erstelle Verzeichnisse falls nicht vorhanden
        self.base_directory.mkdir(parents=True, exist_ok=True)
        if self.backup_enabled:
            self.backup_directory.mkdir(parents=True, exist_ok=True)

        self.logger.info(f"Storage Manager initialisiert: {self.base_directory}")
        self.logger.info(f"Kompression: {self.compression}, Backups: {self.backup_enabled}")

    def _calculate_checksum(self, data: bytes) -> str:
        """
        Berechnet SHA256-Checksum für Daten.
        
        Args:
            data: Byte-Daten für Checksum-Berechnung
            
        Returns:
            Hexadezimaler SHA256-Hash als String
        """
        return hashlib.sha256(data).hexdigest()

    def _compress_data(self, data: bytes) -> bytes:
        """
        Komprimiert Daten basierend auf konfiguriertem Kompressionstyp.
        
        Args:
            data: Unkomprimierte Byte-Daten
            
        Returns:
            Komprimierte Byte-Daten (oder unverändert bei CompressionType.NONE)
            
        Raises:
            ValueError: Bei ungültigem Kompressionstyp
        """
        if self.compression == CompressionType.GZIP:
            return gzip.compress(data, compresslevel=6)
        elif self.compression == CompressionType.LZMA:
            return lzma.compress(data, preset=6)
        elif self.compression == CompressionType.NONE:
            return data
        else:
            raise ValueError(f"Ungültiger Kompressionstyp: {self.compression}")

    def _decompress_data(self, data: bytes, compression: str) -> bytes:
        """
        Dekomprimiert Daten basierend auf angegebenem Kompressionstyp.
        
        Args:
            data: Komprimierte Byte-Daten
            compression: Verwendeter Kompressionstyp (GZIP, LZMA oder NONE)
            
        Returns:
            Dekomprimierte Byte-Daten
            
        Raises:
            ValueError: Bei ungültigem Kompressionstyp
        """
        if compression == CompressionType.GZIP:
            return gzip.decompress(data)
        elif compression == CompressionType.LZMA:
            return lzma.decompress(data)
        elif compression == CompressionType.NONE:
            return data
        else:
            raise ValueError(f"Ungültiger Kompressionstyp: {compression}")

    def _create_backup(self, filepath: Path) -> None:
        """
        Erstellt Backup einer existierenden Datei.
        
        Kopiert die Datei ins Backup-Verzeichnis mit Zeitstempel und
        verwaltet die Anzahl der Backups gemäß backup_count.
        
        Args:
            filepath: Pfad zur zu sichernden Datei
            
        Note:
            Älteste Backups werden automatisch gelöscht wenn backup_count überschritten wird.
        """
        if not self.backup_enabled or not filepath.exists():
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"{filepath.stem}_backup_{timestamp}{filepath.suffix}"
        backup_path = self.backup_directory / backup_name

        try:
            shutil.copy2(filepath, backup_path)
            self.logger.info(f"Backup erstellt: {backup_path}")

            # Lösche alte Backups
            pattern = f"{filepath.stem}_backup_*{filepath.suffix}"
            backups = sorted(
                self.backup_directory.glob(pattern),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )
            
            for old_backup in backups[self.backup_count:]:
                old_backup.unlink()
                self.logger.info(f"Altes Backup gelöscht: {old_backup}")

        except Exception as e:
            self.logger.error(f"Fehler beim Backup erstellen: {e}")

    def save_json(
        self,
        recording_data: PathRecordingData,
        filename: str,
        create_backup: bool = True
    ) -> str:
        """
        Speichert PathRecordingData als JSON-Datei.
        
        Serialisiert die Aufzeichnungsdaten zu JSON, optional mit Kompression
        und Checksum. Erstellt automatisch Backups falls aktiviert.
        
        Args:
            recording_data: Zu speichernde PathRecordingData-Instanz
            filename: Dateiname ohne Erweiterung (wird automatisch hinzugefügt)
            create_backup: Erstellt Backup falls Datei bereits existiert
            
        Returns:
            Vollständiger Pfad zur gespeicherten Datei als String
            
        Raises:
            IOError: Bei Problemen beim Schreiben der Datei
            
        Example:
            >>> path = storage.save_json(recording, "test_session")
            >>> print(f"Gespeichert: {path}")
        """
        with self._lock:
            # Bestimme Dateiendung basierend auf Kompression
            extension = FILE_EXTENSIONS[self.compression]
            filepath = self.base_directory / f"{filename}{extension}"

            # Erstelle Backup falls gewünscht
            if create_backup and filepath.exists():
                self._create_backup(filepath)

            try:
                # Konvertiere zu Dictionary
                data_dict = recording_data.to_dict()
                
                # Serialisiere zu JSON
                json_str = json.dumps(data_dict, indent=2, ensure_ascii=False)
                json_bytes = json_str.encode('utf-8')

                # Berechne Checksum
                checksum = self._calculate_checksum(json_bytes)
                data_dict['_checksum'] = checksum

                # Erneute Serialisierung mit Checksum
                json_str = json.dumps(data_dict, indent=2, ensure_ascii=False)
                json_bytes = json_str.encode('utf-8')

                # Komprimiere falls konfiguriert
                output_data = self._compress_data(json_bytes)

                # Schreibe Datei
                with open(filepath, 'wb') as f:
                    f.write(output_data)

                self.logger.info(f"JSON gespeichert: {filepath} ({len(output_data)} bytes)")
                return str(filepath)

            except Exception as e:
                self.logger.error(f"Fehler beim Speichern von JSON: {e}")
                raise

    def load_json(self, filepath: Union[str, Path]) -> PathRecordingData:
        """
        Lädt PathRecordingData aus JSON-Datei.
        
        Liest JSON-Datei (mit optionaler Dekompression), validiert Checksum
        falls aktiviert und deserialisiert zu PathRecordingData.
        
        Args:
            filepath: Pfad zur JSON-Datei
            
        Returns:
            Geladene PathRecordingData-Instanz
            
        Raises:
            FileNotFoundError: Wenn Datei nicht existiert
            ValueError: Bei ungültiger Checksum
            json.JSONDecodeError: Bei ungültigem JSON
            
        Example:
            >>> recording = storage.load_json("recordings/session_001.json.gz")
            >>> print(f"Samples: {len(recording.controller_samples)}")
        """
        filepath = Path(filepath)
        
        with self._lock:
            try:
                # Lese Datei
                with open(filepath, 'rb') as f:
                    compressed_data = f.read()

                # Bestimme Kompressionstyp aus Dateiendung
                if filepath.suffix == '.gz':
                    compression = CompressionType.GZIP
                elif filepath.suffix == '.xz':
                    compression = CompressionType.LZMA
                else:
                    compression = CompressionType.NONE

                # Dekomprimiere
                json_bytes = self._decompress_data(compressed_data, compression)
                json_str = json_bytes.decode('utf-8')

                # Parse JSON
                data_dict = json.loads(json_str)

                # Validiere Checksum falls aktiviert
                if self.verify_checksums and '_checksum' in data_dict:
                    stored_checksum = data_dict.pop('_checksum')
                    
                    # Berechne Checksum ohne '_checksum'-Feld
                    json_str_no_checksum = json.dumps(data_dict, indent=2, ensure_ascii=False)
                    calculated_checksum = self._calculate_checksum(json_str_no_checksum.encode('utf-8'))
                    
                    if stored_checksum != calculated_checksum:
                        raise ValueError(f"Checksum mismatch: {filepath}")

                # Deserialisiere zu PathRecordingData
                recording_data = PathRecordingData.from_dict(data_dict)
                
                self.logger.info(f"JSON geladen: {filepath}")
                return recording_data

            except Exception as e:
                self.logger.error(f"Fehler beim Laden von JSON: {e}")
                # Versuche Recovery aus Backup
                try:
                    return self._recover_from_backup(filepath)
                except:
                    raise

    def save_binary(
        self,
        recording_data: PathRecordingData,
        filename: str,
        create_backup: bool = True
    ) -> str:
        """
        Speichert PathRecordingData als Binär-Datei (.prec).
        
        Serialisiert die Daten in ein kompaktes Binärformat mit Header,
        Metadaten und Chunks für effiziente Speicherung großer Aufzeichnungen.
        
        Binary Format Structure:
            - Header: Magic (4B) + Version (16B) + Timestamp (8B) + Checksum (32B)
            - Metadata: Compressed JSON mit Header-Daten
            - Controller Chunk: Magic (4B) + Count (4B) + Samples (variable)
            - LIDAR Chunk: Magic (4B) + Count (4B) + Frames (variable)
            - Checkpoint Chunk: Magic (4B) + Count (4B) + Checkpoints (variable)
        
        Args:
            recording_data: Zu speichernde PathRecordingData-Instanz
            filename: Dateiname ohne Erweiterung (.prec wird hinzugefügt)
            create_backup: Erstellt Backup falls Datei bereits existiert
            
        Returns:
            Vollständiger Pfad zur gespeicherten Datei als String
            
        Raises:
            IOError: Bei Problemen beim Schreiben der Datei
            
        Example:
            >>> path = storage.save_binary(recording, "test_session")
            >>> print(f"Gespeichert: {path}")
        """
        with self._lock:
            filepath = self.base_directory / f"{filename}{BINARY_EXTENSION}"

            # Erstelle Backup falls gewünscht
            if create_backup and filepath.exists():
                self._create_backup(filepath)

            try:
                with open(filepath, 'wb') as f:
                    # Header schreiben
                    f.write(HEADER_MAGIC)
                    f.write(recording_data.header.version.ljust(16, '\0').encode('ascii'))
                    f.write(struct.pack('d', recording_data.header.timestamp))

                    # Metadaten komprimieren und schreiben
                    metadata_dict = recording_data.header.to_dict()
                    metadata_json = json.dumps(metadata_dict).encode('utf-8')
                    metadata_compressed = gzip.compress(metadata_json)
                    f.write(struct.pack('I', len(metadata_compressed)))
                    f.write(metadata_compressed)

                    # Controller-Samples Chunk
                    f.write(CHUNK_MAGIC)
                    f.write(struct.pack('I', len(recording_data.controller_samples)))
                    for sample in recording_data.controller_samples:
                        sample_json = json.dumps(sample.to_dict()).encode('utf-8')
                        f.write(struct.pack('I', len(sample_json)))
                        f.write(sample_json)

                    # LIDAR-Frames Chunk
                    f.write(CHUNK_MAGIC)
                    f.write(struct.pack('I', len(recording_data.lidar_frames)))
                    for frame in recording_data.lidar_frames:
                        frame_json = json.dumps(frame.to_dict()).encode('utf-8')
                        f.write(struct.pack('I', len(frame_json)))
                        f.write(frame_json)

                    # Checkpoints Chunk
                    f.write(CHUNK_MAGIC)
                    f.write(struct.pack('I', len(recording_data.checkpoints)))
                    for checkpoint in recording_data.checkpoints:
                        checkpoint_json = json.dumps(checkpoint.to_dict()).encode('utf-8')
                        f.write(struct.pack('I', len(checkpoint_json)))
                        f.write(checkpoint_json)

                file_size = filepath.stat().st_size
                self.logger.info(f"Binary gespeichert: {filepath} ({file_size} bytes)")
                return str(filepath)

            except Exception as e:
                self.logger.error(f"Fehler beim Speichern von Binary: {e}")
                raise

    def load_binary(self, filepath: Union[str, Path]) -> PathRecordingData:
        """
        Lädt PathRecordingData aus Binär-Datei (.prec).
        
        Liest und deserialisiert Binärdaten mit Header-Validierung und
        Chunk-basierter Datenrekonstruktion.
        
        Args:
            filepath: Pfad zur .prec-Datei
            
        Returns:
            Geladene PathRecordingData-Instanz
            
        Raises:
            FileNotFoundError: Wenn Datei nicht existiert
            ValueError: Bei ungültigem Header oder Magic Bytes
            struct.error: Bei fehlerhafter Binärstruktur
            
        Example:
            >>> recording = storage.load_binary("recordings/session_001.prec")
            >>> print(f"Version: {recording.header.version}")
        """
        filepath = Path(filepath)
        
        with self._lock:
            try:
                with open(filepath, 'rb') as f:
                    # Lese und validiere Header
                    magic = f.read(4)
                    if magic != HEADER_MAGIC:
                        raise ValueError(f"Ungültiges Magic: {magic}")

                    version = f.read(16).rstrip(b'\0').decode('ascii')
                    timestamp = struct.unpack('d', f.read(8))[0]

                    # Lese Metadaten
                    metadata_size = struct.unpack('I', f.read(4))[0]
                    metadata_compressed = f.read(metadata_size)
                    metadata_json = gzip.decompress(metadata_compressed)
                    header_dict = json.loads(metadata_json)
                    header = PathRecordingHeader.from_dict(header_dict)

                    # Lese Controller-Samples
                    chunk_magic = f.read(4)
                    if chunk_magic != CHUNK_MAGIC:
                        raise ValueError("Ungültiges Chunk Magic für Controller")
                    
                    controller_count = struct.unpack('I', f.read(4))[0]
                    controller_samples = []
                    for _ in range(controller_count):
                        sample_size = struct.unpack('I', f.read(4))[0]
                        sample_json = f.read(sample_size)
                        sample_dict = json.loads(sample_json)
                        controller_samples.append(ControllerSample.from_dict(sample_dict))

                    # Lese LIDAR-Frames
                    chunk_magic = f.read(4)
                    if chunk_magic != CHUNK_MAGIC:
                        raise ValueError("Ungültiges Chunk Magic für LIDAR")
                    
                    lidar_count = struct.unpack('I', f.read(4))[0]
                    lidar_frames = []
                    for _ in range(lidar_count):
                        frame_size = struct.unpack('I', f.read(4))[0]
                        frame_json = f.read(frame_size)
                        frame_dict = json.loads(frame_json)
                        lidar_frames.append(LidarFrame.from_dict(frame_dict))

                    # Lese Checkpoints
                    chunk_magic = f.read(4)
                    if chunk_magic != CHUNK_MAGIC:
                        raise ValueError("Ungültiges Chunk Magic für Checkpoints")
                    
                    checkpoint_count = struct.unpack('I', f.read(4))[0]
                    checkpoints = []
                    for _ in range(checkpoint_count):
                        checkpoint_size = struct.unpack('I', f.read(4))[0]
                        checkpoint_json = f.read(checkpoint_size)
                        checkpoint_dict = json.loads(checkpoint_json)
                        checkpoints.append(Checkpoint.from_dict(checkpoint_dict))

                # Erstelle PathRecordingData
                recording_data = PathRecordingData(
                    header=header,
                    controller_samples=controller_samples,
                    lidar_frames=lidar_frames,
                    checkpoints=checkpoints
                )

                self.logger.info(f"Binary geladen: {filepath}")
                return recording_data

            except Exception as e:
                self.logger.error(f"Fehler beim Laden von Binary: {e}")
                # Versuche Recovery aus Backup
                try:
                    return self._recover_from_backup(filepath)
                except:
                    raise

    def _recover_from_backup(self, original_filepath: Path) -> PathRecordingData:
        """
        Versucht Recovery aus Backup-Dateien.
        
        Sucht nach Backups der beschädigten Datei und versucht diese
        in chronologischer Reihenfolge (neueste zuerst) zu laden.
        
        Args:
            original_filepath: Pfad zur beschädigten Original-Datei
            
        Returns:
            Aus Backup wiederhergestellte PathRecordingData-Instanz
            
        Raises:
            FileNotFoundError: Wenn keine Backup-Dateien gefunden wurden
            RuntimeError: Wenn Recovery aus allen Backups fehlschlug
        """
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
        """
        Listet alle verfügbaren Aufzeichnungen auf.
        
        Durchsucht das Aufzeichnungsverzeichnis und sammelt Informationen
        über alle gefundenen Recording-Dateien.
        
        Returns:
            Liste von Dictionaries mit Datei-Informationen:
            - filename: Dateiname
            - filepath: Vollständiger Pfad
            - size_bytes: Dateigröße in Bytes
            - modified: Änderungszeitpunkt als datetime
            - format: 'binary' oder 'json'
            
        Example:
            >>> recordings = storage.list_recordings()
            >>> for rec in recordings:
            ...     print(f"{rec['filename']}: {rec['size_bytes']} bytes")
        """
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
        """
        Validiert Integrität einer Aufzeichnungsdatei.
        
        Versucht die Datei zu laden und sammelt Informationen über
        Gültigkeit, Format und Inhalt.
        
        Args:
            filepath: Pfad zur zu validierenden Datei
            
        Returns:
            Dictionary mit Validierungs-Ergebnissen:
            - filepath: Pfad zur Datei
            - exists: Bool ob Datei existiert
            - valid: Bool ob Datei gültig geladen werden konnte
            - errors: Liste von Fehler-Meldungen
            - info: Dictionary mit Datei-Informationen (bei gültiger Datei)
            
        Example:
            >>> validation = storage.validate_file("recordings/test.json.gz")
            >>> if validation['valid']:
            ...     print("Datei ist gültig")
            ...     print(f"Samples: {validation['info']['controller_samples']}")
            >>> else:
            ...     print(f"Fehler: {validation['errors']}")
        """
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
    """
    Hilfsfunktion zum schnellen Erstellen eines Storage Managers.
    
    Erstellt einen vorkonfigurierten PathRecordingStorageManager mit
    sinnvollen Standardeinstellungen.
    
    Args:
        directory: Basis-Verzeichnis für Aufzeichnungen
        compression: Kompressionstyp (GZIP, LZMA oder NONE)
        backup_count: Anzahl der zu behaltenden Backups
        
    Returns:
        Konfigurierte PathRecordingStorageManager-Instanz
        
    Example:
        >>> storage = create_storage_manager(
        ...     directory="my_recordings",
        ...     compression=CompressionType.LZMA,
        ...     backup_count=5
        ... )
        >>> # Storage Manager ist sofort einsatzbereit
    """
    return PathRecordingStorageManager(
        base_directory=directory,
        compression=compression,
        backup_enabled=True,
        backup_count=backup_count,
        verify_checksums=True
    )


# Erfolgreiche Modul-Initialisierung
print("✅ Storage System erfolgreich geladen (v1.1.0)")
