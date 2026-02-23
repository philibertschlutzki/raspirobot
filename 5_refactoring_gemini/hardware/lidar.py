import os
import time
from typing import List, Tuple

try:
    from ..interfaces import ILidarSensor
    from ..config import LIDAR_PORT, LIDAR_BAUDRATE
except ImportError:
    from interfaces import ILidarSensor
    from config import LIDAR_PORT, LIDAR_BAUDRATE

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

    def start(self):
        if self.pc_mode:
            self.running = True
            print("LIDAR started (PC Mode)")
            return

        try:
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
            self.iterator = self.lidar.iter_scans()
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

    def get_latest_scan(self) -> List[Tuple[float, float]]:
        """Returns list of (angle, distance). Blocks until scan available."""
        if self.pc_mode:
            time.sleep(0.1) # Simulate delay
            return []

        if not self.running or not self.iterator:
            return []

        try:
            scan = next(self.iterator)
            # scan is list of (quality, angle, distance)
            # Return (angle, distance)
            return [(angle, dist) for (_, angle, dist) in scan]
        except Exception as e:
            print(f"LIDAR read error: {e}")
            # Try to recover? logic handles this in orchestrator or here?
            # Orchestrator handles timeouts.
            # Here we might want to re-raise or return empty.
            # Returning empty might trigger logic in Orchestrator.
            return []
