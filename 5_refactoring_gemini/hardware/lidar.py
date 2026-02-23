import os
import time
from typing import List, Tuple

try:
    from ..interfaces import ILidarSensor
    from ..config import LIDAR_PORT, LIDAR_BAUDRATE
    from ..logger import get_logger
except ImportError:
    from interfaces import ILidarSensor
    from config import LIDAR_PORT, LIDAR_BAUDRATE
    from logger import get_logger

is_pc_mode = os.environ.get("PC_TEST_MODE") == "1"

if not is_pc_mode:
    try:
        from rplidar import RPLidar, RPLidarException
    except ImportError:
        print("Warning: rplidar library not found. Simulating PC Mode.")
        is_pc_mode = True

class RealLidarSensor(ILidarSensor):
    def __init__(self):
        self.pc_mode = is_pc_mode
        self.lidar = None
        self.iterator = None
        self.running = False
        try:
            self.logger = get_logger()
        except Exception:
            self.logger = None
        self.last_scan_time = 0.0

    def start(self):
        if self.pc_mode:
            self.running = True
            print("LIDAR started (PC Mode)")
            return

        try:
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
            self.lidar.start_motor()
            time.sleep(2.0)
            self.iterator = self.lidar.iter_scans(max_buf_meas=5000, min_len=10)
            self.running = True
        except Exception as e:
            print(f"Error starting LIDAR: {e}")
            self.pc_mode = True # Fallback

    def stop(self):
        self.running = False
        if self.pc_mode: return
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except: pass
            self.lidar = None
            self.iterator = None

    def get_latest_scan(self) -> List[Tuple[int, float, float]]:
        """Returns list of (quality, angle, distance). Blocks until scan available."""
        if self.pc_mode:
            time.sleep(0.1) # Simulate delay
            return []

        if not self.running or not self.iterator:
            return []

        try:
            scan = next(self.iterator)

            current_time = time.time()
            # print(f"DEBUG_LIDAR: t={current_time}, last={self.last_scan_time}")
            if self.last_scan_time > 0:
                dt = current_time - self.last_scan_time
                # print(f"DEBUG_LIDAR: dt={dt}")
                if dt > 0.15: # < 6.6 Hz
                    hz = 1.0 / dt
                    if self.logger:
                        self.logger.warn("LIDAR_LOW_FREQ", {"hz": round(hz, 2), "dt": round(dt, 3)})
                    # print(f"Warning: Low Lidar Frequency: {round(hz, 2)} Hz")

            self.last_scan_time = current_time

            # scan is list of (quality, angle, distance)
            return [(quality, angle, dist) for (quality, angle, dist) in scan]
        except Exception as e:
            print(f"LIDAR read error: {e}")
            # Try to recover? logic handles this in orchestrator or here?
            # Orchestrator handles timeouts.
            # Here we might want to re-raise or return empty.
            # Returning empty might trigger logic in Orchestrator.
            return []
