"""Fahrtaufzeichnung: Controller-Samples und LIDAR-Frames in eine .json.gz-Datei."""

import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from . import config
from .hardware.xbox_controller import ControllerInput
from .log import logger
from .recording.data_format import (
    CalibrationData,
    CompressionType,
    ControllerSample,
    EnvironmentMetadata,
    LidarFrame,
    PathRecordingData,
    PathRecordingHeader,
    setup_logging,
)
from .recording.storage import PathRecordingStorageManager

# Steht in den Metadaten jeder Aufnahme (environment_conditions.robot_configuration).
ROBOT_CONFIGURATION = "xbox_controller_lidar_v3.0.1"


def create_session_id() -> str:
    return f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_robot01"


def create_recording_data(session_id: str, lidar_connected: bool) -> PathRecordingData:
    """Neue leere Aufnahme mit denselben Header-Werten wie die Referenz v3.0.1."""
    calibration_data = CalibrationData(
        controller_calibration={
            "left_trigger_offset": 0.0,
            "right_trigger_offset": 0.0,
            "deadzone": 0.05,
            "max_duty_cycle": config.MAX_DUTY_CYCLE,
        },
        lidar_calibration={
            "offset_x": config.LIDAR_OFFSET_X,
            "offset_y": config.LIDAR_OFFSET_Y,
            "angle_offset": config.LIDAR_ANGLE_OFFSET,
        },
        calibration_timestamp=time.time(),
        calibration_quality=1.0,
    )
    metadata = EnvironmentMetadata(
        session_id=session_id,
        start_timestamp=time.time(),
        end_timestamp=None,
        environment_conditions={
            "recording_location": "indoor",
            "robot_configuration": ROBOT_CONFIGURATION,
        },
        hardware_info={
            "controller_model": "Xbox Wireless Controller",
            "robot_platform": "RaspberryPi_Tank_Robot",
            "motor_type": "DC_with_PWM",
            "sensors": "HC-SR04_ultrasonic + RPLIDAR_C1",
            "lidar_connected": lidar_connected,
        },
        recording_settings={
            "controller_rate_hz": config.RECORDING_SAMPLE_RATE_HZ,
            "lidar_rate_hz": config.LIDAR_UPDATE_HZ,
            "pose_tracking": True,
            "compression": CompressionType.GZIP,
        },
    )
    header = PathRecordingHeader(
        calibration_data=calibration_data, metadata=metadata, compression_type=CompressionType.GZIP
    )
    return PathRecordingData(header=header)


class Recorder:
    """Start/Stop einer Aufnahme und Puffern der LIDAR-Frames.

    LIDAR-Frames kommen aus dem LIDAR-Thread in einen Puffer
    (LIDAR_FRAME_BUFFER_SIZE, bei Überlauf fällt der älteste Frame weg) und
    werden im Steuerungs-Loop (max. 5 pro Zyklus) sowie beim Stoppen in die
    Aufnahme übernommen.
    """

    def __init__(self, status_indicator, reset_pose, lidar_connected):
        """
        Args:
            status_indicator: ``StatusIndicator`` für LED/Buzzer.
            reset_pose: Funktion, die die Pose beim Start auf (0, 0, 0) setzt.
            lidar_connected: Funktion, die den LIDAR-Verbindungsstatus liefert.
        """
        self._status = status_indicator
        self._reset_pose = reset_pose
        self._lidar_connected = lidar_connected
        self._lock = threading.Lock()
        self._active = False
        self.storage: Optional[PathRecordingStorageManager] = None
        self.data: Optional[PathRecordingData] = None
        self.session_id = ""
        self.start_time = 0.0
        self.lidar_frame_buffer: queue.Queue = queue.Queue(maxsize=config.LIDAR_FRAME_BUFFER_SIZE)

    def setup(self, base_dir: str = config.RECORDING_BASE_DIR) -> None:
        """Legt das Verzeichnis an und richtet Logging und Speicher ein."""
        print("[SETUP] Initialisiere Recording-System...")
        try:
            Path(base_dir).mkdir(parents=True, exist_ok=True)
            setup_logging(str(Path(base_dir) / "xbox_lidar_recording.log"))
            self.storage = PathRecordingStorageManager(
                base_directory=base_dir,
                compression=CompressionType.GZIP,
                backup_enabled=True,
                backup_count=3,
                verify_checksums=True,
            )
            logger.info("Recording-System initialisiert")
            print(f"[SETUP] ✅ Recording-System bereit: {base_dir}")
        except Exception as e:
            print(f"[SETUP] ❌ Recording-System Fehler: {e}")
            logger.error(f"Recording-System Fehler: {e}")
            raise

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def offer_lidar_scan(self, timestamp: float, scan_data, scan_index: int) -> None:
        """Vom LIDAR-Thread aufgerufen; puffert den Scan nur während einer Aufnahme."""
        if not self.active:
            return
        frame = LidarFrame(timestamp=timestamp, scan_data=scan_data, frame_id=scan_index)
        try:
            self.lidar_frame_buffer.put_nowait(frame)
        except queue.Full:
            try:
                self.lidar_frame_buffer.get_nowait()
                self.lidar_frame_buffer.put_nowait(frame)
            except queue.Empty:
                pass

    def _clear_buffer(self) -> None:
        while not self.lidar_frame_buffer.empty():
            try:
                self.lidar_frame_buffer.get_nowait()
            except queue.Empty:
                break

    def start(self) -> bool:
        if not self.storage:
            print("[RECORDING] ❌ Recording-System nicht verfügbar")
            return False

        with self._lock:
            if self._active:
                print("[RECORDING] ⚠️ Recording bereits aktiv")
                return False
            self._active = True

        try:
            self.session_id = create_session_id()
            self.start_time = time.time()
            self._clear_buffer()
            self._reset_pose()
            self.data = create_recording_data(self.session_id, self._lidar_connected())

            self._status.beep(duration=0.1, count=2)
            self._status.set_status("recording")

            print(f"[RECORDING] 🔴 Recording gestartet: {self.session_id}")
            logger.info(f"Recording gestartet: {self.session_id}")
            return True
        except Exception as e:
            print(f"[RECORDING] ❌ Fehler: {e}")
            logger.error(f"Recording Start fehlgeschlagen: {e}")
            with self._lock:
                self._active = False
            return False

    def stop(self, force_save: bool = False) -> bool:
        """Beendet die Aufnahme und speichert sie, wenn sie mindestens
        MIN_RECORDING_DURATION_SEC lang war oder ``force_save`` gesetzt ist.

        Returns:
            True, wenn gespeichert wurde.
        """
        with self._lock:
            if not self._active:
                return False
            # Sofort zurücksetzen, damit der LIDAR-Thread keine neuen Frames mehr puffert.
            self._active = False

        try:
            print("[RECORDING] ⏹️ Stoppe Recording...")
            time.sleep(0.2)  # laufende Puffer-Schreibvorgänge abschließen lassen

            frames_flushed = 0
            flush_timeout = time.time() + 2.0
            print("[RECORDING] 📦 Flush LIDAR-Frame-Buffer...")
            while time.time() < flush_timeout:
                try:
                    frame = self.lidar_frame_buffer.get_nowait()
                    if self.data:
                        self.data.add_lidar_frame(frame)
                        frames_flushed += 1
                except queue.Empty:
                    break
            if frames_flushed > 0:
                print(f"[RECORDING] ✅ {frames_flushed} LIDAR-Frames nachträglich gespeichert")

            self._status.set_status("saving")

            duration = time.time() - self.start_time
            stats = self.data.get_stats() if self.data else {}
            controller_samples = stats.get("controller_samples", 0)
            lidar_frames = stats.get("lidar_frames", 0)

            is_valid = duration >= config.MIN_RECORDING_DURATION_SEC or force_save

            if is_valid and self.data and self.storage:
                if self.data.header.metadata:
                    self.data.header.metadata.end_timestamp = time.time()
                filepath = self.storage.save_json(self.data, self.session_id, create_backup=True)

                print(f"[RECORDING] ✅ Gespeichert: {filepath}")
                print("[RECORDING] 📊 Statistiken:")
                print(f"    Dauer: {duration:.2f}s")
                print(f"    Controller: {controller_samples} samples")
                print(f"    LIDAR: {lidar_frames} frames")
                logger.info(
                    f"Recording gespeichert: {filepath} (Controller: {controller_samples}, LIDAR: {lidar_frames})"
                )
                self._status.beep(duration=0.2, count=1)
            else:
                print(f"[RECORDING] ⚠️ Verworfen (zu kurz: {duration:.2f}s)")
                logger.warning(f"Recording verworfen: {duration:.2f}s")
                self._status.beep(duration=0.1, count=3)

            self._status.set_status("off")
            return is_valid
        except Exception as e:
            print(f"[RECORDING] ❌ Fehler: {e}")
            logger.error(f"Recording Stop fehlgeschlagen: {e}")
            with self._lock:
                self._active = False
            self._status.set_status("off")
            return False

    def add_sample(self, controller: ControllerInput) -> None:
        """Ein Controller-Sample plus bis zu 5 gepufferte LIDAR-Frames übernehmen."""
        if not self.active or not self.data:
            return
        try:
            sample = ControllerSample(
                timestamp=time.time(),
                left_trigger=controller.trigger_left,
                right_trigger=controller.trigger_right,
                buttons={
                    "A": controller.a_pressed,
                    "B": not controller.is_moving_forward,
                    "X": controller.x_pressed,
                    "Y": controller.y_pressed,
                },
                direction={
                    "forward": 1.0 if controller.is_moving_forward else -1.0,
                    "tank_turn": 1.0 if controller.a_pressed else 0.0,
                },
                sequence_id=0,
            )
            self.data.add_controller_sample(sample)

            for _ in range(5):
                try:
                    self.data.add_lidar_frame(self.lidar_frame_buffer.get_nowait())
                except queue.Empty:
                    break
        except Exception as e:
            logger.error(f"Recording Daten-Fehler: {e}")
