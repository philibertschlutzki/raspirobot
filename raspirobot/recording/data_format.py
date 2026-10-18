"""Datenstrukturen einer Fahrtaufzeichnung.

Das Format ist identisch mit dem der Referenz (``path_recording_system.py``,
Formatversion 1.1.0); vorhandene Aufnahmen lassen sich weiter laden.
"""

import logging
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple

DATA_FORMAT_VERSION = "1.1.0"
HEADER_MAGIC = b"PREC"


class CompressionType:
    """Kompressionsarten der JSON-Datei."""

    NONE = "none"
    GZIP = "gzip"
    LZMA = "lzma"


FILE_EXTENSIONS = {
    CompressionType.NONE: ".json",
    CompressionType.GZIP: ".json.gz",
    CompressionType.LZMA: ".json.xz",
}


@dataclass
class ControllerSample:
    """Ein Controller-Zustand.

    Attributes:
        timestamp: Unix-Zeit in Sekunden.
        left_trigger: Rohwert der pygame-Achse, -1.0 (losgelassen) bis 1.0.
        right_trigger: wie left_trigger.
        buttons: Die Steuerung schreibt ``A``, ``X``, ``Y`` (Tastenzustand, aus
            pygame als 0/1) und ``B`` (True, wenn Rückwärtsmodus aktiv ist).
        direction: Die Steuerung schreibt ``forward`` (1.0 vorwärts, -1.0
            rückwärts) und ``tank_turn`` (1.0, wenn A gehalten wird).
        sequence_id: Wird von ``PathRecordingData.add_controller_sample`` fortlaufend gesetzt.
    """

    timestamp: float
    left_trigger: float
    right_trigger: float
    buttons: Dict[str, bool]
    direction: Dict[str, float]
    sequence_id: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ControllerSample":
        return cls(**data)


@dataclass
class LidarFrame:
    """Ein vollständiger LIDAR-Umlauf.

    Attributes:
        timestamp: Unix-Zeit in Sekunden beim Empfang des Scans.
        scan_data: Messpunkte ``(quality, angle_deg, distance_mm)`` wie von
            ``RPLidar.iter_scans`` geliefert (Winkel 0-360 Grad, Distanz in mm).
        quality_metrics: Wird beim Erzeugen automatisch berechnet, falls leer.
        frame_id: Wird von ``PathRecordingData.add_lidar_frame`` fortlaufend gesetzt.
    """

    timestamp: float
    scan_data: List[Tuple[float, float, float]]
    quality_metrics: Dict[str, float] = field(default_factory=dict)
    frame_id: int = 0

    def __post_init__(self):
        if not self.quality_metrics and self.scan_data:
            self.quality_metrics = self._calculate_quality_metrics()

    def _calculate_quality_metrics(self) -> Dict[str, float]:
        if not self.scan_data:
            return {
                "valid_points": 0,
                "total_points": 0,
                "coverage": 0.0,
                "avg_quality": 0.0,
                "min_distance": 0.0,
                "max_distance": 0.0,
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
            "max_distance": max_distance,
        }

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "LidarFrame":
        return cls(**data)


@dataclass
class Checkpoint:
    """Wegpunkt. Teil des Dateiformats; die Steuerung legt derzeit keine an."""

    timestamp: float
    position: Tuple[float, float, float]
    controller_state: ControllerSample
    lidar_snapshot: LidarFrame
    checkpoint_id: str
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["controller_state"] = self.controller_state.to_dict()
        data["lidar_snapshot"] = self.lidar_snapshot.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Checkpoint":
        controller_data = data.pop("controller_state")
        lidar_data = data.pop("lidar_snapshot")
        return cls(
            controller_state=ControllerSample.from_dict(controller_data),
            lidar_snapshot=LidarFrame.from_dict(lidar_data),
            **data,
        )


@dataclass
class CalibrationData:
    """Kalibrierwerte, wie sie die Steuerung in den Header schreibt."""

    controller_calibration: Dict[str, float]
    lidar_calibration: Dict[str, float]
    calibration_timestamp: float
    calibration_quality: float = 1.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CalibrationData":
        return cls(**data)


@dataclass
class EnvironmentMetadata:
    """Session-Metadaten einer Aufnahme."""

    session_id: str
    start_timestamp: float
    end_timestamp: Optional[float] = None
    environment_conditions: Dict[str, Any] = field(default_factory=dict)
    hardware_info: Dict[str, str] = field(default_factory=dict)
    software_version: str = DATA_FORMAT_VERSION
    recording_settings: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "EnvironmentMetadata":
        return cls(**data)


@dataclass
class PathRecordingHeader:
    """Header einer Aufnahme."""

    magic: bytes = HEADER_MAGIC
    version: str = DATA_FORMAT_VERSION
    timestamp: float = field(default_factory=time.time)
    calibration_data: Optional[CalibrationData] = None
    metadata: Optional[EnvironmentMetadata] = None
    checksum_algorithm: str = "sha256"
    compression_type: str = CompressionType.GZIP

    def to_dict(self) -> Dict[str, Any]:
        data = {
            "magic": self.magic.decode("ascii"),
            "version": self.version,
            "timestamp": self.timestamp,
            "checksum_algorithm": self.checksum_algorithm,
            "compression_type": self.compression_type,
        }
        if self.calibration_data:
            data["calibration_data"] = self.calibration_data.to_dict()
        if self.metadata:
            data["metadata"] = self.metadata.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PathRecordingHeader":
        calibration_data = None
        metadata = None

        if "calibration_data" in data:
            calibration_data = CalibrationData.from_dict(data.pop("calibration_data"))
        if "metadata" in data:
            metadata = EnvironmentMetadata.from_dict(data.pop("metadata"))

        data["magic"] = data["magic"].encode("ascii")

        return cls(
            calibration_data=calibration_data,
            metadata=metadata,
            **{
                k: v
                for k, v in data.items()
                if k in ["magic", "version", "timestamp", "checksum_algorithm", "compression_type"]
            },
        )


@dataclass
class PathRecordingData:
    """Komplette Aufnahme: Header, Controller-Samples, LIDAR-Frames, Checkpoints."""

    header: PathRecordingHeader
    controller_samples: List[ControllerSample] = field(default_factory=list)
    lidar_frames: List[LidarFrame] = field(default_factory=list)
    checkpoints: List[Checkpoint] = field(default_factory=list)

    def add_controller_sample(self, sample: ControllerSample) -> None:
        """Hängt ein Sample an und überschreibt dessen sequence_id fortlaufend."""
        if self.controller_samples:
            sample.sequence_id = self.controller_samples[-1].sequence_id + 1
        else:
            sample.sequence_id = 0
        self.controller_samples.append(sample)

    def add_lidar_frame(self, frame: LidarFrame) -> None:
        """Hängt einen Frame an und überschreibt dessen frame_id fortlaufend."""
        if self.lidar_frames:
            frame.frame_id = self.lidar_frames[-1].frame_id + 1
        else:
            frame.frame_id = 0
        self.lidar_frames.append(frame)

    def add_checkpoint(self, checkpoint: Checkpoint) -> None:
        self.checkpoints.append(checkpoint)

    def get_stats(self) -> Dict[str, Any]:
        """Anzahl und Raten der Samples; ``{"status": "empty"}`` ohne Daten."""
        if not self.controller_samples and not self.lidar_frames:
            return {"status": "empty"}

        stats = {
            "controller_samples": len(self.controller_samples),
            "lidar_frames": len(self.lidar_frames),
            "checkpoints": len(self.checkpoints),
            "duration_seconds": 0.0,
            "controller_sample_rate": 0.0,
            "lidar_sample_rate": 0.0,
        }

        if self.controller_samples:
            duration = self.controller_samples[-1].timestamp - self.controller_samples[0].timestamp
            stats["duration_seconds"] = duration
            if duration > 0:
                stats["controller_sample_rate"] = len(self.controller_samples) / duration

        if self.lidar_frames:
            duration = self.lidar_frames[-1].timestamp - self.lidar_frames[0].timestamp
            if duration > 0:
                stats["lidar_sample_rate"] = len(self.lidar_frames) / duration

        return stats

    def to_dict(self) -> Dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "controller_samples": [s.to_dict() for s in self.controller_samples],
            "lidar_frames": [f.to_dict() for f in self.lidar_frames],
            "checkpoints": [c.to_dict() for c in self.checkpoints],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PathRecordingData":
        return cls(
            header=PathRecordingHeader.from_dict(data["header"]),
            controller_samples=[ControllerSample.from_dict(s) for s in data.get("controller_samples", [])],
            lidar_frames=[LidarFrame.from_dict(f) for f in data.get("lidar_frames", [])],
            checkpoints=[Checkpoint.from_dict(c) for c in data.get("checkpoints", [])],
        )


def setup_logging(log_file: str = "path_recording.log") -> logging.Logger:
    """Liefert den Logger ``PathRecording`` mit Datei- und Konsolenausgabe.

    Handler werden nur beim ersten Aufruf angehängt; spätere Aufrufe mit
    anderer Datei ändern das Ziel nicht.
    """
    logger = logging.getLogger("PathRecording")
    logger.setLevel(logging.INFO)

    if not logger.handlers:
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

    return logger
