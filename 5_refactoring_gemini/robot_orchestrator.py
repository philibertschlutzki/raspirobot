import threading
import time
import math
import uuid
import json
import os
from typing import List, Tuple, Dict, Any, Optional
from threading import Lock
from enum import Enum

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
    from .mcl_engine import MCLEngine, LocalizationResult
    from .path_planner import AStarPlanner
    from .pure_pursuit import PurePursuitController
except ImportError:
    from interfaces import IMotorController, ILidarSensor, IUltrasonicSensor, IInputController
    from odometry import OdometryEngine
    from recorder import DataRecorder
    from logger import Logger, get_logger
    from data_models import RobotPose, ControllerSample, LidarFrame, PathRecordingData, LogLevel
    from config import *
    from mcl_engine import MCLEngine, LocalizationResult
    from path_planner import AStarPlanner
    from pure_pursuit import PurePursuitController

class NavigationState(Enum):
    IDLE = "IDLE"           # Manual control or stopped
    FOLLOWING = "FOLLOWING" # Autonomous path following
    REVERSING = "REVERSING" # Ultrasonic emergency reverse
    REPLANNING = "REPLANNING" # Calculating new path
    BLOCKED = "BLOCKED"     # Cannot find path

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

        # Load Calibration
        self.calibration = self._load_calibration()
        self.lidar_offset_x = self.calibration.get("lidar_offset_x", LIDAR_OFFSET_X)
        self.lidar_offset_y = self.calibration.get("lidar_offset_y", LIDAR_OFFSET_Y)
        self.us_offsets = {
            "left": (self.calibration.get("ultrasonic_offset_x", ULTRASONIC_OFFSET_X),
                     self.calibration.get("ultrasonic_offset_y_left", ULTRASONIC_OFFSET_Y_LEFT)),
            "right": (self.calibration.get("ultrasonic_offset_x", ULTRASONIC_OFFSET_X),
                      self.calibration.get("ultrasonic_offset_y_right", ULTRASONIC_OFFSET_Y_RIGHT))
        }

        # State
        self.is_running = False
        self.is_recording = False
        self.is_moving_forward = True
        self.lidar_error_active = False
        self.session_id = None
        self.session_start_time = None
        self.frames_count = 0

        # Navigation State
        self.nav_state = NavigationState.IDLE
        self.mcl: Optional[MCLEngine] = None
        self.planner: Optional[AStarPlanner] = None
        self.controller: Optional[PurePursuitController] = None
        self.current_pose = RobotPose(x=0, y=0, theta=0, timestamp=0)
        self.original_path: List[Tuple[float, float]] = []
        self.active_path: List[Tuple[float, float]] = []
        self.reverse_start_pose: Optional[RobotPose] = None
        self.autonomous_enabled = False

        # Sensor Timeout Tracking
        self.us_last_valid_time = {"left": time.time(), "right": time.time()}
        self.us_error_active = False

        # Data Buffers
        self.latest_scan = []
        self.min_lidar_distance_m = 1000.0
        self.min_us_distance_cm = 1000.0
        self.us_distances = {"left": 1000.0, "right": 1000.0}
        self.lidar_last_update = 0.0

        # Loop Timing
        self.consecutive_overruns = 0
        self.max_overruns_allowed = 5
        self.loop_target_dt = 1.0 / CONTROL_LOOP_HZ
        self.max_loop_time = 0.02 # 20ms

        # Thread sync
        self.scan_lock = Lock()
        self.us_lock = Lock()
        self.state_buffer = [] # ControllerSamples
        self.buffer_lock = Lock()

        # Threads
        self.t_lidar = threading.Thread(target=self._lidar_thread, daemon=True)
        self.t_us = threading.Thread(target=self._ultrasonic_thread, daemon=True)

    def _load_calibration(self) -> Dict[str, float]:
        calib_file = "calibration.json"
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    data = json.load(f)
                    self.logger.info("CALIBRATION_LOADED", data)
                    return data
            except Exception as e:
                self.logger.error("CALIBRATION_LOAD_ERROR", {"error": str(e)})
        return {}

    def setup_navigation(self, map_path: str, initial_pose: RobotPose):
        try:
            self.mcl = MCLEngine(map_path, initial_pose)
            self.planner = AStarPlanner(self.mcl.map_grid, us_offsets=self.us_offsets)
            self.controller = PurePursuitController()
            self.current_pose = initial_pose
            self.odometry.set_pose(initial_pose)
            self.logger.info("NAV_SETUP_COMPLETE", {})
        except Exception as e:
            self.logger.error("NAV_SETUP_FAILED", {"error": str(e)})

    def set_goal_path(self, path: List[Tuple[float, float]]):
        self.original_path = path
        self.active_path = path

    def set_autonomous(self, enabled: bool):
        self.autonomous_enabled = enabled
        if enabled:
            self.nav_state = NavigationState.FOLLOWING
            self.logger.info("AUTONOMY_ENABLED", {})
        else:
            self.nav_state = NavigationState.IDLE
            self.motor.set_speed(0, 0)
            self.logger.info("AUTONOMY_DISABLED", {})

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
                    dist_m = dist / 1000.0
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
                        frame_id=self.frames_count,
                        scan_data=scan_data,
                        controller_states=samples
                    )
                    self.frames_count += 1
                    # Stream Frame
                    self.recorder.log_frame(frame)

            except Exception as e:
                self.logger.error("LIDAR_THREAD_ERROR", {"error": str(e)})
                time.sleep(0.1)

    def _ultrasonic_thread(self):
        while self.is_running:
            try:
                distances = self.ultrasonic.get_distances()
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
        self.frames_count = 0
        self.state_buffer = []
        self.logger.info("RECORDING_START", {"session_id": self.session_id})

        hardware_info={
            "platform": "Raspberry Pi 5",
            "capabilities": {
                "has_lidar": not self.lidar_error_active,
                "has_pose": True,
                "lidar_error_active": self.lidar_error_active
            },
            "calibration": self.calibration
        }
        self.recorder.start_session(self.session_id, hardware_info)

    def stop_recording(self):
        self.is_recording = False
        self.logger.info("RECORDING_STOP", {"frames": self.frames_count})
        self.recorder.stop_session()

    def _main_loop(self):
        while self.is_running:
            loop_start = time.time()

            # 1. Update Pose (Odometry)
            # In Manual mode, _drive calls odometry.update().
            # In Autonomous mode, we must also call odometry.update().

            # 2. MCL Update
            if self.mcl:
                current_odom = self.odometry.get_pose()
                if not hasattr(self, '_last_odom'):
                    self._last_odom = current_odom

                dx = current_odom.x - self._last_odom.x
                dy = current_odom.y - self._last_odom.y
                dtheta = current_odom.theta - self._last_odom.theta
                dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

                c = math.cos(self._last_odom.theta)
                s = math.sin(self._last_odom.theta)
                dx_local = dx * c + dy * s
                dy_local = -dx * s + dy * c

                self.mcl.predict((dx_local, dy_local, dtheta))
                self._last_odom = current_odom

                with self.scan_lock:
                    scan_data = list(self.latest_scan)

                if scan_data:
                    res = self.mcl.update(scan_data, current_odom)
                    self.current_pose = res.pose
                    if res.unexpected_obstacle_detected:
                        if self.autonomous_enabled and self.nav_state == NavigationState.FOLLOWING:
                            self.logger.warn("MCL_OBSTACLE_DETECTED", {})
                            self.nav_state = NavigationState.REPLANNING
                            self.motor.stop()
                else:
                    self.current_pose = current_odom

            # 3. Input Handling
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

                # Toggle Autonomous (e.g., Button A)
                if buttons.get("A"):
                     self.set_autonomous(not self.autonomous_enabled)
                     time.sleep(0.5)

                # B -> Direction (Manual Only)
                if buttons.get("B") and not self.autonomous_enabled:
                     self.is_moving_forward = not self.is_moving_forward
                     self.logger.info("DIRECTION_CHANGE", {"forward": self.is_moving_forward})
                     time.sleep(0.2)

                # Start -> Stop
                if buttons.get("START"):
                     self.is_running = False
                     break

                # Manual Drive if not Autonomous
                if not self.autonomous_enabled:
                    trig_l = axes.get("LEFT_TRIGGER", -1.0)
                    trig_r = axes.get("RIGHT_TRIGGER", -1.0)
                    self._drive_manual(trig_l, trig_r)
                else:
                    self._drive_autonomous()
            else:
                 if not self.autonomous_enabled:
                    self.motor.set_speed(0, 0)
                    self.odometry.update(0, 0)
                 else:
                    self._drive_autonomous()

            # Timing & Overrun Detection
            elapsed = time.time() - loop_start

            if elapsed > self.max_loop_time:
                self.logger.warn("LOOP_OVERRUN", {"elapsed": elapsed})
                self.consecutive_overruns += 1
            else:
                self.consecutive_overruns = 0

            if self.consecutive_overruns > 5:
                self.logger.error("CRITICAL_OVERRUN_STOP", {"consecutive": self.consecutive_overruns})
                self.motor.stop()
                self.consecutive_overruns = 0 # Reset to allow recovery or just stop?
                # Requirement says "bremst der Roboter (PWM hart auf 0.0) und loggt..."
                # It doesn't say "shut down". Just stop.

            sleep_time = self.loop_target_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _drive_autonomous(self):
        if not self.mcl or not self.planner:
            return

        with self.us_lock:
            us_dist = self.min_us_distance_cm
            us_data = self.us_distances.copy()

        # Update planner with US data
        self.planner.update_ultrasonic_data(us_data)

        # State Machine
        if self.nav_state == NavigationState.FOLLOWING:
            # Check Ultrasonic
            if us_dist < 15.0:
                self.logger.warn("US_EMERGENCY_REVERSE", {"dist": us_dist})
                self.nav_state = NavigationState.REVERSING
                self.reverse_start_pose = self.current_pose
                self.motor.stop()
                return

            # Get Command
            l, r = self.controller.get_command(self.current_pose, self.active_path)
            self.motor.set_speed(l, r)
            self.odometry.update(l, r)

            # Check completion of detour
            if self.active_path is not self.original_path:
                if self.active_path:
                    end_p = self.active_path[-1]
                    dist_to_end = math.sqrt((self.current_pose.x - end_p[0])**2 + (self.current_pose.y - end_p[1])**2)
                    if dist_to_end < 0.3: # Reached end of detour
                        self.active_path = self.original_path
                        self.logger.info("RESUMING_ORIGINAL_PATH", {})

        elif self.nav_state == NavigationState.REVERSING:
            # Move backward slowly
            speed = -0.2

            # Phase 8: Max 50 cm reverse limit
            dist_traveled = math.sqrt((self.current_pose.x - self.reverse_start_pose.x)**2 +
                                      (self.current_pose.y - self.reverse_start_pose.y)**2)

            # Lidar collision check (Lidar is bumper when reversing)
            with self.scan_lock:
                lidar_min = self.min_lidar_distance_m

            if lidar_min < 0.2: # Stop if lidar sees obstacle closely behind/around
                 self.logger.error("REVERSE_LIDAR_STOP", {"dist": lidar_min})
                 self.motor.stop()
                 self.nav_state = NavigationState.BLOCKED
                 return

            if dist_traveled >= 0.50:
                 self.logger.warn("REVERSE_LIMIT_REACHED", {"dist": dist_traveled})
                 self.motor.stop()
                 self.nav_state = NavigationState.REPLANNING
                 return

            self.motor.set_speed(speed, speed)
            self.odometry.update(speed, speed)

            if us_dist > 30.0 or dist_traveled > 0.15:
                self.logger.info("REVERSING_COMPLETE", {"dist": dist_traveled, "us": us_dist})
                self.motor.stop()
                self.nav_state = NavigationState.REPLANNING

        elif self.nav_state == NavigationState.REPLANNING:
            obs_dist = 0.20
            ox = self.reverse_start_pose.x + 0.20 * math.cos(self.reverse_start_pose.theta)
            oy = self.reverse_start_pose.y + 0.20 * math.sin(self.reverse_start_pose.theta)

            self.planner.inject_virtual_obstacle(ox, oy)
            goal_point = self._find_target_on_original_path(self.current_pose, lookahead=2.0)
            goal_pose = RobotPose(x=goal_point[0], y=goal_point[1], theta=0, timestamp=0)

            new_path = self.planner.plan(self.current_pose, goal_pose)

            if new_path:
                self.active_path = new_path
                self.nav_state = NavigationState.FOLLOWING
                self.logger.info("REPLAN_SUCCESS", {"len": len(new_path)})
            else:
                self.nav_state = NavigationState.BLOCKED
                self.logger.error("PATH_BLOCKED", {})
                self.motor.stop()

        elif self.nav_state == NavigationState.BLOCKED:
            self.motor.stop()
            self.odometry.update(0, 0)

    def _find_target_on_original_path(self, current_pose, lookahead=2.0) -> Tuple[float, float]:
        if not self.original_path:
            return (current_pose.x, current_pose.y) # Fallback

        min_dist_sq = float('inf')
        closest_idx = 0
        for i, (px, py) in enumerate(self.original_path):
            d2 = (px - current_pose.x)**2 + (py - current_pose.y)**2
            if d2 < min_dist_sq:
                min_dist_sq = d2
                closest_idx = i

        for i in range(closest_idx, len(self.original_path)):
            px, py = self.original_path[i]
            d2 = (px - current_pose.x)**2 + (py - current_pose.y)**2
            if d2 > lookahead**2:
                return (px, py)

        return self.original_path[-1]

    def _drive_manual(self, trig_l, trig_r):
        current_time = time.time()

        if self.lidar_error_active:
             self.motor.stop()
             return

        # 1. Ultrasonic Failure Logic
        us_fail_factor = 1.0
        with self.us_lock:
            distances = self.us_distances.copy()
            us_dist = self.min_us_distance_cm

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

        throttle_l = (trig_l + 1) / 2
        throttle_r = (trig_r + 1) / 2

        speed_l = throttle_l
        speed_r = throttle_r

        if not self.is_moving_forward:
            speed_l = -speed_l
            speed_r = -speed_r

        with self.scan_lock:
            lidar_dist = self.min_lidar_distance_m

        factor = 1.0
        if self.is_moving_forward:
             if us_dist < 15.0:
                 factor = 0.0
                 if abs(speed_l) > 0.1 or abs(speed_r) > 0.1:
                     self.logger.warn("ULTRASONIC_EMERGENCY_STOP", {"dist": us_dist})
             elif us_dist < COLLISION_DISTANCE_CM or lidar_dist < LIDAR_COLLISION_DISTANCE_M:
                 factor = 0.5

        speed_l *= factor * us_fail_factor
        speed_r *= factor * us_fail_factor

        self.motor.set_speed(speed_l, speed_r)
        self.odometry.update(speed_l, speed_r)

        if self.is_recording:
            sample = ControllerSample(
                timestamp=time.time(),
                left_duty=abs(speed_l) * MAX_DUTY_CYCLE,
                right_duty=abs(speed_r) * MAX_DUTY_CYCLE,
                is_moving_forward=self.is_moving_forward,
                pose=self.odometry.get_pose()
            )
            with self.buffer_lock:
                self.state_buffer.append(sample)
