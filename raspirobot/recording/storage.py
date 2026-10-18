"""Speichern und Laden von Aufnahmen als (komprimiertes) JSON mit SHA256-Prüfsumme."""

import gzip
import hashlib
import json
import lzma
import shutil
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Union

from .data_format import FILE_EXTENSIONS, CompressionType, PathRecordingData, setup_logging


class PathRecordingStorageManager:
    """Schreibt und liest Aufnahmen in ``base_directory``.

    Beim Überschreiben einer bestehenden Datei wird vorher eine Kopie in
    ``base_directory/backups`` abgelegt; es bleiben ``backup_count`` Kopien.
    Die Prüfsumme wird über das JSON ohne das Feld ``_checksum`` berechnet.
    """

    def __init__(
        self,
        base_directory: str = "path_recordings",
        compression: str = CompressionType.GZIP,
        backup_enabled: bool = True,
        backup_count: int = 3,
        verify_checksums: bool = True,
    ):
        self.base_directory = Path(base_directory)
        self.backup_directory = self.base_directory / "backups"
        self.compression = compression
        self.backup_enabled = backup_enabled
        self.backup_count = backup_count
        self.verify_checksums = verify_checksums
        self._lock = threading.Lock()

        self.base_directory.mkdir(parents=True, exist_ok=True)
        if self.backup_enabled:
            self.backup_directory.mkdir(parents=True, exist_ok=True)
        self.logger = setup_logging(str(self.base_directory / "storage.log"))

        self.logger.info(f"Storage Manager initialisiert: {self.base_directory}")
        self.logger.info(f"Kompression: {self.compression}, Backups: {self.backup_enabled}")

    @staticmethod
    def _calculate_checksum(data: bytes) -> str:
        return hashlib.sha256(data).hexdigest()

    def _compress_data(self, data: bytes) -> bytes:
        if self.compression == CompressionType.GZIP:
            return gzip.compress(data, compresslevel=6)
        if self.compression == CompressionType.LZMA:
            return lzma.compress(data, preset=6)
        if self.compression == CompressionType.NONE:
            return data
        raise ValueError(f"Ungültiger Kompressionstyp: {self.compression}")

    @staticmethod
    def _decompress_data(data: bytes, compression: str) -> bytes:
        if compression == CompressionType.GZIP:
            return gzip.decompress(data)
        if compression == CompressionType.LZMA:
            return lzma.decompress(data)
        if compression == CompressionType.NONE:
            return data
        raise ValueError(f"Ungültiger Kompressionstyp: {compression}")

    def _create_backup(self, filepath: Path) -> None:
        if not self.backup_enabled or not filepath.exists():
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = self.backup_directory / f"{filepath.stem}_backup_{timestamp}{filepath.suffix}"

        try:
            shutil.copy2(filepath, backup_path)
            self.logger.info(f"Backup erstellt: {backup_path}")

            backups = sorted(
                self.backup_directory.glob(f"{filepath.stem}_backup_*{filepath.suffix}"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            for old_backup in backups[self.backup_count:]:
                old_backup.unlink()
                self.logger.info(f"Altes Backup gelöscht: {old_backup}")
        except Exception as e:
            self.logger.error(f"Fehler beim Backup erstellen: {e}")

    def save_json(self, recording_data: PathRecordingData, filename: str, create_backup: bool = True) -> str:
        """Speichert die Aufnahme als ``<filename><Endung>`` und gibt den Pfad zurück."""
        with self._lock:
            filepath = self.base_directory / f"{filename}{FILE_EXTENSIONS[self.compression]}"

            if create_backup and filepath.exists():
                self._create_backup(filepath)

            try:
                data_dict = recording_data.to_dict()
                json_bytes = json.dumps(data_dict, indent=2, ensure_ascii=False).encode("utf-8")
                data_dict["_checksum"] = self._calculate_checksum(json_bytes)
                json_bytes = json.dumps(data_dict, indent=2, ensure_ascii=False).encode("utf-8")
                output_data = self._compress_data(json_bytes)

                with open(filepath, "wb") as f:
                    f.write(output_data)

                self.logger.info(f"JSON gespeichert: {filepath} ({len(output_data)} bytes)")
                return str(filepath)
            except Exception as e:
                self.logger.error(f"Fehler beim Speichern von JSON: {e}")
                raise

    def load_json(self, filepath: Union[str, Path]) -> PathRecordingData:
        """Lädt eine Aufnahme; bei Fehlern wird ein Backup versucht.

        Raises:
            ValueError: Prüfsumme stimmt nicht (und kein Backup ladbar).
        """
        filepath = Path(filepath)
        try:
            return self._load_json_file(filepath)
        except Exception as e:
            self.logger.error(f"Fehler beim Laden von JSON: {e}")
            try:
                return self._recover_from_backup(filepath)
            except FileNotFoundError:
                raise e

    def _load_json_file(self, filepath: Path) -> PathRecordingData:
        with self._lock:
            with open(filepath, "rb") as f:
                raw = f.read()

            if filepath.suffix == ".gz":
                compression = CompressionType.GZIP
            elif filepath.suffix == ".xz":
                compression = CompressionType.LZMA
            else:
                compression = CompressionType.NONE

            data_dict = json.loads(self._decompress_data(raw, compression).decode("utf-8"))

            if self.verify_checksums and "_checksum" in data_dict:
                stored = data_dict.pop("_checksum")
                calculated = self._calculate_checksum(
                    json.dumps(data_dict, indent=2, ensure_ascii=False).encode("utf-8")
                )
                if stored != calculated:
                    raise ValueError(f"Checksum mismatch: {filepath}")

            recording_data = PathRecordingData.from_dict(data_dict)
            self.logger.info(f"JSON geladen: {filepath}")
            return recording_data

    def _recover_from_backup(self, original_filepath: Path) -> PathRecordingData:
        self.logger.warning(f"Versuche Recovery für: {original_filepath}")
        backups = sorted(
            self.backup_directory.glob(f"{original_filepath.stem}_backup_*{original_filepath.suffix}"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if not backups:
            raise FileNotFoundError("Keine Backup-Dateien gefunden")

        for backup_path in backups:
            try:
                self.logger.info(f"Versuche Recovery aus: {backup_path}")
                return self._load_json_file(backup_path)
            except Exception as e:
                self.logger.warning(f"Recovery aus {backup_path} fehlgeschlagen: {e}")

        raise RuntimeError("Recovery aus allen Backups fehlgeschlagen")

    def list_recordings(self) -> List[Dict[str, Any]]:
        """Dateien in ``base_directory``, neueste zuerst."""
        recordings = []
        for file_path in self.base_directory.glob("*"):
            if file_path.is_file() and not file_path.name.startswith("."):
                stat = file_path.stat()
                recordings.append(
                    {
                        "filename": file_path.name,
                        "filepath": str(file_path),
                        "size_bytes": stat.st_size,
                        "modified": datetime.fromtimestamp(stat.st_mtime),
                    }
                )
        return sorted(recordings, key=lambda x: x["modified"], reverse=True)
