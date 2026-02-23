import threading
import time
import math
import uuid
from typing import List, Tuple, Dict, Any, Optional
from threading import Lock

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None

try:
    from .interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
    from .odometry import OdometryEngine
    from .recorder import DataRecorder
    from .logger import Logger, get_logger
    from .data_models import RobotPose, ControllerSample, LidarFrame, PathRecordingData, LogLevel
    from .config import *
except ImportError:
    from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
    from odometry import OdometryEngine
    from recorder import DataRecorder
    from logger import Logger, get_logger
    from data_models import RobotPose, ControllerSample, LidarFrame, PathRecordingData, LogLevel
    from config import *

class RobotOrchestrator:
    def __init__(self,
                 motor_controller: IMotorController,
                 lidar_sensor: ILidarSensor,
                 ultrasonic_sensor: IUltrasonicSensor,
                 input_controller: IInputController,
                 odometry_engine: OdometryEngine,
                 recorder: DataRecorder):

        self.motor = motor_controller
        self.lidar = lidar_sensor
        self.ultrasonic = ultrasonic_sensor
        self.input = input_controller
        self.odometry = odometry_engine
        self.recorder = recorder
        self.logger = get_logger()

        # State
        self.is_running = False
        self.is_recording = False
        self.is_moving_forward = True
        self.lidar_error_active = False
        self.session_id = None
        self.session_start_time = None

        # Sensor Timeout Tracking
        self.us_last_valid_time = {"left": time.time(), "right": time.time()}
        self.us_error_active = False

        # Data Buffers
        self.latest_scan = []
        self.min_lidar_distance_m = 1000.0
        self.min_us_distance_cm = 1000.0
        self.us_distances = {"left": 1000.0, "right": 1000.0}
        self.lidar_last_update = 0.0

        # Thread sync
        self.scan_lock = Lock()
        self.us_lock = Lock()
        self.state_buffer = [] # ControllerSamples
        self.frames = [] # LidarFrames
        self.buffer_lock = Lock()

        # Threads
        self.t_lidar = threading.Thread(target=self._lidar_thread, daemon=True)
        self.t_us = threading.Thread(target=self._ultrasonic_thread, daemon=True)

    def start(self):
        self.is_running = True
        self.lidar.start()
        self.t_lidar.start()
        self.t_us.start()
        self.logger.info("ROBOT_START", {"status": "started"})
        try:
            self._main_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self.is_running = False
        self.lidar.stop()
        self.motor.stop()
        if self.is_recording:
            self.stop_recording()

        if GPIO:
            try:
                GPIO.cleanup()
                self.logger.info("GPIO_CLEANUP", {})
            except Exception as e:
                self.logger.error("GPIO_CLEANUP_ERROR", {"error": str(e)})

        self.logger.info("ROBOT_STOP", {"status": "stopped"})

    def _lidar_thread(self):
        self.lidar_last_update = time.time()
        while self.is_running:
            try:
                scan = self.lidar.get_latest_scan()
                current_time = time.time()

                if not scan:
                    # Check timeout
                    if current_time - self.lidar_last_update > 0.3: # 300ms
                         if not self.lidar_error_active:
                             self.logger.error("LIDAR_TIMEOUT_STOP", {"dt": current_time - self.lidar_last_update})
                             self.lidar_error_active = True
                    time.sleep(0.01)
                    continue

                # Frequency Check
                dt = current_time - self.lidar_last_update
                if dt > 0:
                    freq = 1.0 / dt
                    if abs(freq - LIDAR_TARGET_FREQUENCY) > 2.0:
                         self.logger.warn("LIDAR_FREQ_UNSTABLE", {"hz": freq, "target": LIDAR_TARGET_FREQUENCY})

                self.lidar_last_update = current_time
                if self.lidar_error_active:
                    self.lidar_error_active = False
                    self.logger.info("LIDAR_RECOVERED", {})

                # Process scan for min distance
                min_dist = 1000.0
                scan_data = []
                # scan is list of (quality, angle, distance)
                for quality, angle, dist in scan:
                    # RPLidar yields mm usually. Config assumes mm input and converts to meters?
                    # v3.0.2 logic: dist / 1000.0
                    dist_m = dist / 1000.0

                    # Front check logic from v3.0.2:
                    check_angle = angle
                    if check_angle > 180: check_angle -= 360
                    if -45 <= check_angle <= 45:
                        if dist_m > 0:
                            min_dist = min(min_dist, dist_m)

                    scan_data.append((angle, dist))

                with self.scan_lock:
                    self.latest_scan = scan_data
                    self.min_lidar_distance_m = min_dist

                # Create Frame if recording
                if self.is_recording and not self.lidar_error_active:
                    with self.buffer_lock:
                         samples = list(self.state_buffer)
                         self.state_buffer.clear()

                    frame = LidarFrame(
                        timestamp=current_time,
                        frame_id=len(self.frames),
                        scan_data=scan_data,
                        controller_states=samples
                    )
                    self.frames.append(frame)

            except Exception as e:
                self.logger.error("LIDAR_THREAD_ERROR", {"error": str(e)})
                time.sleep(0.1)

    def _ultrasonic_thread(self):
        while self.is_running:
            try:
                distances = self.ultrasonic.get_distances()
                # Distances are now raw, can be -1.0 if error/timeout

                # Calculate simple min distance for quick checks (ignoring errors for now, handled in drive)
                d_l = distances.get("left", -1.0)
                d_r = distances.get("right", -1.0)

                valid_vals = []
                if d_l > 0: valid_vals.append(d_l)
                if d_r > 0: valid_vals.append(d_r)

                min_dist = 1000.0
                if valid_vals:
                    min_dist = min(valid_vals)

                with self.us_lock:
                    self.us_distances = distances
                    self.min_us_distance_cm = min_dist

                time.sleep(1.0 / SENSOR_UPDATE_HZ)
            except Exception as e:
                self.logger.error("US_THREAD_ERROR", {"error": str(e)})

    def start_recording(self):
        self.is_recording = True
        self.session_id = str(uuid.uuid4())
        self.session_start_time = time.time()
        self.frames = []
        self.state_buffer = []
        self.logger.info("RECORDING_START", {"session_id": self.session_id})

    def stop_recording(self):
        self.is_recording = False
        self.logger.info("RECORDING_STOP", {"frames": len(self.frames)})

        session = PathRecordingData(
            session_id=self.session_id if self.session_id else "unknown",
            start_timestamp=self.session_start_time if self.session_start_time else time.time(),
            end_timestamp=time.time(),
            hardware_info={
                "platform": "Raspberry Pi 5",
                "capabilities": {
                    "has_lidar": not self.lidar_error_active,
                    "has_pose": True,
                    "lidar_error_active": self.lidar_error_active
                }
            },
            frames=self.frames
        )
        self.recorder.save_session_async(session)

    def _main_loop(self):
        dt = 1.0 / CONTROL_LOOP_HZ
        while self.is_running:
            loop_start = time.time()

            # 1. Input
            inp = self.input.get_state()
            if inp.get("connected"):
                buttons = inp.get("buttons", {})
                axes = inp.get("axes", {})

                # X + Y -> Toggle Recording
                if buttons.get("X") and buttons.get("Y"):
                     if not self.is_recording:
                         self.start_recording()
                         time.sleep(0.5)
                     else:
                         self.stop_recording()
                         time.sleep(0.5)

                # B -> Direction
                if buttons.get("B"):
                     self.is_moving_forward = not self.is_moving_forward
                     self.logger.info("DIRECTION_CHANGE", {"forward": self.is_moving_forward})
                     time.sleep(0.2)

                # Start -> Stop
                if buttons.get("START"):
                     self.is_running = False
                     break

                # Drive
                trig_l = axes.get("LEFT_TRIGGER", -1.0)
                trig_r = axes.get("RIGHT_TRIGGER", -1.0)
                self._drive(trig_l, trig_r)
            else:
                # No controller, stop motors?
                # Or keep going if autonomous? Assuming manual control for now.
                self.motor.set_speed(0, 0)

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _drive(self, trig_l, trig_r):
        current_time = time.time()

        if self.lidar_error_active:
             self.motor.stop()
             return

        # 1. Ultrasonic Failure Logic
        us_fail_factor = 1.0
        with self.us_lock:
            distances = self.us_distances.copy()
            us_dist = self.min_us_distance_cm

        # Track valid updates
        any_sensor_timeout = False
        critical_stop = False

        for key in ["left", "right"]:
            val = distances.get(key, -1.0)
            if val > 0:
                self.us_last_valid_time[key] = current_time
            else:
                elapsed = current_time - self.us_last_valid_time.get(key, current_time)
                if elapsed > SENSOR_TIMEOUT_STOP_SEC:
                    if not self.us_error_active:
                        self.logger.error("US_CRITICAL_STOP", {"sensor": key, "elapsed": elapsed})
                        self.us_error_active = True
                    critical_stop = True
                elif elapsed > SENSOR_TIMEOUT_WARN_SEC:
                    any_sensor_timeout = True

        if critical_stop:
            self.motor.stop()
            return

        if self.us_error_active:
             self.us_error_active = False
             self.logger.info("US_RECOVERED", {})

        if any_sensor_timeout:
            us_fail_factor = SENSOR_FAIL_SPEED_FACTOR

        # Map trigger (-1 to 1) to (0 to 1) throttle
        throttle_l = (trig_l + 1) / 2
        throttle_r = (trig_r + 1) / 2

        # Directions
        speed_l = throttle_l
        speed_r = throttle_r

        if not self.is_moving_forward:
            speed_l = -speed_l
            speed_r = -speed_r

        # Collision Avoidance
        with self.scan_lock:
            lidar_dist = self.min_lidar_distance_m

        factor = 1.0
        if self.is_moving_forward:
             if us_dist < 15.0:
                 factor = 0.0
                 # Log only if we are actually trying to move
                 if abs(speed_l) > 0.1 or abs(speed_r) > 0.1:
                     self.logger.warn("ULTRASONIC_EMERGENCY_STOP", {"dist": us_dist})
             elif us_dist < COLLISION_DISTANCE_CM or lidar_dist < LIDAR_COLLISION_DISTANCE_M:
                 factor = 0.5

        # Apply factors
        speed_l *= factor * us_fail_factor
        speed_r *= factor * us_fail_factor

        self.motor.set_speed(speed_l, speed_r)

        # Update Odometry
        pose = self.odometry.update(speed_l, speed_r)

        # Buffer State if recording
        if self.is_recording:
            sample = ControllerSample(
                timestamp=time.time(),
                left_duty=abs(speed_l) * MAX_DUTY_CYCLE,
                right_duty=abs(speed_r) * MAX_DUTY_CYCLE,
                is_moving_forward=self.is_moving_forward,
                pose=pose
            )
            with self.buffer_lock:
                self.state_buffer.append(sample)
