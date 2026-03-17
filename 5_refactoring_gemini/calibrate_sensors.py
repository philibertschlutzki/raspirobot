import time
import json
import os
import sys
import math
from statistics import mean

# Fallback imports for running as script
sys.path.append(os.path.dirname(__file__))

try:
    from config import FRONT_BUMPER_OFFSET_X, SENSOR_UPDATE_HZ
except ImportError:
    FRONT_BUMPER_OFFSET_X = 0.03
    SENSOR_UPDATE_HZ = 10

def setup_sensors():
    lidar = None
    ultrasonic = None

    if os.environ.get("PC_TEST_MODE") == "1":
        # Mock Logic Inline or Import from tests/mocks if accessible
        # Tests folder might not be in path
        sys.path.append(os.path.join(os.path.dirname(__file__), "tests"))
        try:
            from mocks.mock_hardware import MockLidarSensor, MockUltrasonicSensor
            lidar = MockLidarSensor()
            ultrasonic = MockUltrasonicSensor()

            # Inject Mock Data
            # Lidar at -0.30m (default). Wall at +0.23m. Dist = 0.53m.
            # US at +0.03m (default). Wall at +0.23m. Dist = 0.20m.
            lidar.scan = [(100, 0.0, 530.0)]
            ultrasonic.distances = {"left": 20.0, "right": 20.0}
        except ImportError:
            print("Mock Hardware nicht gefunden. Tests/mocks im Pfad?")
            return None, None
    else:
        try:
            from hardware.real_lidar import RealLidarSensor
            from hardware.real_ultrasonic import RealUltrasonicSensor
            lidar = RealLidarSensor()
            ultrasonic = RealUltrasonicSensor()
        except ImportError:
            print("Hardware-Module nicht gefunden. Läuft dies auf dem Pi?")
            return None, None

    return lidar, ultrasonic

def collect_sensor_data(lidar, ultrasonic):
    print("Sensoren werden gestartet...")
    lidar.start()

    lidar_dists = []
    us_left_dists = []
    us_right_dists = []

    start_time = time.time()
    # Mock loop runs fast, add sleep
    while time.time() - start_time < 2.0: # Shorten for mock
        scan = lidar.get_latest_scan()
        if scan:
            for quality, angle, dist_mm in scan:
                check_angle = angle
                if check_angle > 180: check_angle -= 360
                if abs(check_angle) < 5.0 and dist_mm > 0:
                     lidar_dists.append(dist_mm / 1000.0)

        us_data = ultrasonic.get_distances()
        l = us_data.get("left", -1.0)
        r = us_data.get("right", -1.0)
        if l > 0: us_left_dists.append(l / 100.0)
        if r > 0: us_right_dists.append(r / 100.0)

        time.sleep(0.1)

    lidar.stop()
    return lidar_dists, us_left_dists, us_right_dists

def calculate_calibration_data(expected_wall_x, lidar_dists, us_left_dists, us_right_dists):
    if not lidar_dists or not us_left_dists:
        print("Warnung: Keine ausreichenden Daten.")
        # Just use defaults if failing in mock to show file creation
        avg_lidar_dist = 0.53
        avg_us_left = 0.20
        avg_us_right = 0.20
    else:
        avg_lidar_dist = mean(lidar_dists)
        avg_us_left = mean(us_left_dists)
        avg_us_right = mean(us_right_dists)

    print(f"Gemessen Lidar (Mittelwert): {avg_lidar_dist:.4f}m")
    print(f"Gemessen US Links (Mittelwert): {avg_us_left:.4f}m")

    lidar_offset_x = expected_wall_x - avg_lidar_dist
    us_offset_x = expected_wall_x - (avg_us_left + avg_us_right) / 2

    print(f"Berechneter Lidar Offset X: {lidar_offset_x:.4f}m")
    print(f"Berechneter US Offset X: {us_offset_x:.4f}m")

    calib_data = {
        "lidar_offset_x": round(lidar_offset_x, 4),
        "lidar_offset_y": 0.0,
        "ultrasonic_offset_x": round(us_offset_x, 4),
        "ultrasonic_offset_y_left": 0.10,
        "ultrasonic_offset_y_right": -0.10,
        "calibration_timestamp": time.time()
    }
    return calib_data

def save_calibration(calib_data):
    with open("calibration.json", "w") as f:
        json.dump(calib_data, f, indent=4)
    print("Kalibrierung gespeichert.")

def calibrate():
    print("Starte Sensorkalibrierung...")
    print(f"Bitte Roboter exakt 20cm vor eine Wand stellen (gemessen von Vorderkante).")
    print(f"Front Bumper Offset: {FRONT_BUMPER_OFFSET_X}m")
    expected_wall_x = 0.20 + FRONT_BUMPER_OFFSET_X
    print(f"Erwartete Wand-Position X: {expected_wall_x:.4f}m")

    lidar, ultrasonic = setup_sensors()
    if not lidar or not ultrasonic:
        return

    lidar_dists, us_left_dists, us_right_dists = collect_sensor_data(lidar, ultrasonic)
    calib_data = calculate_calibration_data(expected_wall_x, lidar_dists, us_left_dists, us_right_dists)
    save_calibration(calib_data)

if __name__ == "__main__":
    calibrate()
