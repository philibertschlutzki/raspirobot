"""RPLIDAR über die Bibliothek ``rplidar`` (Paket rplidar-roboticia)."""

import threading
import time
from typing import Callable, List, Tuple

from .. import config
from ..log import logger

try:
    from rplidar import RPLidar, RPLidarException

    LIDAR_AVAILABLE = True
    print("✅ RPLIDAR Bibliothek erfolgreich importiert")

    class C1Lidar(RPLidar):
        """RPLIDAR C1: Der Motor läuft ohne PWM-Kommando.

        Die Bibliothek schickt beim Scan-Start das A2-Kommando SET_PWM. Antwortet
        der C1 direkt danach auf die Health-Abfrage, geht die Antwort häufig
        verloren ("Descriptor length mismatch"). Deshalb hier nur DTR setzen.
        """

        def start_motor(self):
            self._serial.setDTR(False)
            self.motor_running = True

        def stop_motor(self):
            self._serial.setDTR(True)
            self.motor_running = False

except ImportError as e:
    print(f"⚠️ RPLIDAR nicht verfügbar: {e}")
    LIDAR_AVAILABLE = False
    C1Lidar = None

    class RPLidarException(Exception):
        """Platzhalter, wenn die Bibliothek fehlt."""


ScanPoint = Tuple[float, float, float]  # (quality, angle_deg, distance_mm)


class Lidar:
    """Verbindung zum LIDAR und der jeweils letzte vollständige Scan."""

    def __init__(self):
        self.device = None
        self.connected = False
        self._lock = threading.Lock()
        self._latest_scan: List[ScanPoint] = []

    @property
    def latest_scan(self) -> List[ScanPoint]:
        """Letzter Scan. Die Liste wird pro Scan ersetzt, nie verändert."""
        with self._lock:
            return self._latest_scan

    def connect(self) -> bool:
        """Verbindet, prüft den Health-Status und startet den Motor (2 s Anlaufzeit)."""
        if not LIDAR_AVAILABLE:
            print("[SETUP] ⚠️ RPLIDAR Bibliothek nicht verfügbar")
            return False

        print("[SETUP] Initialisiere RPLIDAR C1...")
        try:
            self.device = C1Lidar(config.LIDAR_PORT, baudrate=config.LIDAR_BAUDRATE, timeout=config.LIDAR_SCAN_TIMEOUT)

            info, health = self._handshake()
            print(f"[SETUP] LIDAR Info: {info}")

            status = health[0] if isinstance(health, tuple) else health.get("status", "Unknown")
            if status == "Error":
                raise RuntimeError("LIDAR meldet Fehlerzustand")

            self.device.start_motor()
            time.sleep(2.0)

            self.connected = True
            print("[SETUP] ✅ RPLIDAR C1 erfolgreich initialisiert")
            logger.info("RPLIDAR C1 initialisiert")
            return True
        except Exception as e:
            print(f"[SETUP] ❌ LIDAR Fehler: {e}")
            logger.error(f"LIDAR Initialisierung fehlgeschlagen: {e}")
            self.device = None
            self.connected = False
            return False

    def _handshake(self, attempts: int = 3):
        """Liest Info und Health. Ein noch scannender Sensor (z. B. nach einem Absturz) wird vorher gestoppt."""
        for attempt in range(1, attempts + 1):
            try:
                self.device.stop()
                time.sleep(0.1)
                self.device.clean_input()
                return self.device.get_info(), self.device.get_health()
            except RPLidarException as e:
                if attempt == attempts:
                    raise
                logger.warning(f"LIDAR Handshake {attempt}/{attempts} fehlgeschlagen: {e}")
                time.sleep(0.3)

    def run(self, should_run: Callable[[], bool], on_scan: Callable[[float, List[ScanPoint], int], None]) -> None:
        """Liest Scans, bis ``should_run()`` False ist oder ein Fehler auftritt.

        Für jeden Scan wird ``latest_scan`` ersetzt und danach
        ``on_scan(timestamp, scan, scan_index)`` aufgerufen.
        Ein abgebrochener Scan wird bis zu ``LIDAR_MAX_RESTARTS`` Mal in Folge neu gestartet;
        danach wird ``connected`` auf False gesetzt.
        """
        if not self.device or not self.connected:
            print("[THREAD] LIDAR Thread nicht gestartet (LIDAR nicht verfügbar)")
            return

        print(f"[THREAD] LIDAR Thread gestartet ({config.LIDAR_UPDATE_HZ} Hz)")
        scan_count = 0
        restarts = 0
        try:
            while should_run():
                try:
                    for scan in self.device.iter_scans(max_buf_meas=5000, min_len=10):
                        restarts = 0
                        if not should_run():
                            print("[THREAD] LIDAR Thread: Shutdown erkannt")
                            break
                        try:
                            timestamp = time.time()
                            scan_data = list(scan)
                            with self._lock:
                                self._latest_scan = scan_data
                            on_scan(timestamp, scan_data, scan_count)
                            scan_count += 1
                        except Exception as e:
                            logger.warning(f"LIDAR Scan-Verarbeitung Fehler: {e}")
                    break
                except (RPLidarException, OSError) as e:
                    restarts += 1
                    print(f"[LIDAR] ⚠️ Scan abgebrochen ({e}), Neustart {restarts}/{config.LIDAR_MAX_RESTARTS}")
                    logger.warning(f"LIDAR Scan abgebrochen: {e} (Neustart {restarts})")
                    if restarts > config.LIDAR_MAX_RESTARTS:
                        raise
                    self._recover()
        except (RPLidarException, OSError) as e:
            print(f"[LIDAR] ❌ LIDAR Fehler: {e}")
            logger.error(f"LIDAR Fehler: {e}")
            self.connected = False
        except Exception as e:
            print(f"[LIDAR] ❌ Unerwarteter Fehler: {e}")
            logger.error(f"LIDAR Unerwarteter Fehler: {e}")
        finally:
            print(f"[THREAD] LIDAR Thread beendet ({scan_count} Scans verarbeitet)")

    def _recover(self) -> None:
        """Bringt den Sensor nach einem abgebrochenen Scan in den Ruhezustand zurück."""
        try:
            self.device.stop()
            time.sleep(0.2)
            self.device.clean_input()
        except Exception as e:
            logger.warning(f"LIDAR Wiederherstellung: {e}")

    def shutdown(self) -> None:
        """Scan stoppen, warten, Motor stoppen, warten, trennen."""
        if not (self.device and self.connected):
            return
        try:
            print("[CLEANUP] Stoppe LIDAR Scan...")
            self.device.stop()
            print(f"[CLEANUP] Warte {config.LIDAR_STOP_DELAY_SEC}s auf Scan-Stop...")
            time.sleep(config.LIDAR_STOP_DELAY_SEC)

            print("[CLEANUP] Stoppe LIDAR Motor...")
            self.device.stop_motor()
            print(f"[CLEANUP] Warte {config.LIDAR_MOTOR_STOP_DELAY_SEC}s auf Motor-Stop...")
            time.sleep(config.LIDAR_MOTOR_STOP_DELAY_SEC)

            print("[CLEANUP] Trenne LIDAR...")
            self.device.disconnect()
            print("[CLEANUP] ✅ LIDAR korrekt beendet")
        except Exception as e:
            print(f"[CLEANUP] ⚠️ LIDAR Stop-Fehler: {e}")
