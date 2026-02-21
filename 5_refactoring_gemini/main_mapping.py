import RPi.GPIO as GPIO
import pygame
import threading
import time
import math
import uuid
import os
from rpi_hardware_pwm import HardwarePWM
from rplidar import RPLidar, RPLidarException
from dataclasses import asdict

from .config import *
from .data_models import RobotPose, ControllerState, LidarFrame, RecordingSession
from .storage import StorageManager

class RaspiRobotMapper:
    def __init__(self):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DIR_LEFT_PIN, GPIO.OUT)
        GPIO.setup(DIR_RIGHT_PIN, GPIO.OUT)

        # Setup PWM
        self.pwm_left = HardwarePWM(pwm_channel=PWM_CHANNEL_LEFT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        self.pwm_right = HardwarePWM(pwm_channel=PWM_CHANNEL_RIGHT, hz=PWM_FREQUENCY, chip=PWM_CHIP)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Setup Pygame
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()
        self.controller = None
        if pygame.joystick.get_count() > CONTROLLER_ID:
            self.controller = pygame.joystick.Joystick(CONTROLLER_ID)
            self.controller.init()
        else:
            print("No controller found!")

        # Storage
        self.storage = StorageManager()

        # State
        self.is_running = True
        self.is_recording = False
        self.lidar_error_active = False
        self.is_moving_forward = True # Default direction

        self.pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=time.time())
        self.state_buffer = [] # Buffer for ControllerStates
        self.frames = [] # Buffer for LidarFrames in current session
        self.session_id = None
        self.session_start_time = None

        self.buffer_lock = threading.Lock()
        self.us_lock = threading.Lock()

        self.min_us_distance_cm = 1000.0
        self.min_lidar_distance_m = 1000.0

        # Odometry helpers
        self.current_duty_left = 0.0
        self.current_duty_right = 0.0

        # Threads
        self.t_us = threading.Thread(target=self.ultrasonic_thread, daemon=True)
        self.t_lidar = threading.Thread(target=self.lidar_thread, daemon=True)
        self.t_us.start()
        self.t_lidar.start()

    def ultrasonic_measure(self, trig, echo):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        start = time.time()
        stop = time.time()

        timeout = start + 0.04 # 40ms timeout ~ 6.8m

        while GPIO.input(echo) == 0:
            start = time.time()
            if start > timeout: return 1000.0

        while GPIO.input(echo) == 1:
            stop = time.time()
            if stop > timeout: return 1000.0

        elapsed = stop - start
        distance = (elapsed * SOUND_SPEED) / 2
        return distance

    def ultrasonic_thread(self):
        # Setup US Pins locally if needed, but GPIO setup is global usually.
        # But good practice to ensure pins are set.
        GPIO.setup(TRIG_LEFT, GPIO.OUT)
        GPIO.setup(ECHO_LEFT, GPIO.IN)
        GPIO.setup(TRIG_RIGHT, GPIO.OUT)
        GPIO.setup(ECHO_RIGHT, GPIO.IN)

        while self.is_running:
            try:
                d1 = self.ultrasonic_measure(TRIG_LEFT, ECHO_LEFT)
                time.sleep(0.01) # Avoid interference
                d2 = self.ultrasonic_measure(TRIG_RIGHT, ECHO_RIGHT)

                with self.us_lock:
                    self.min_us_distance_cm = min(d1, d2)
            except Exception as e:
                print(f"US Error: {e}")

            time.sleep(1.0 / SENSOR_UPDATE_HZ)

    def lidar_thread(self):
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)

        while self.is_running:
            try:
                for scan in lidar.iter_scans():
                    if not self.is_running: break

                    # scan is list of (quality, angle, distance)
                    front_min = 1000.0
                    scan_data = []

                    for (_, angle, dist) in scan:
                        if dist > 0:
                            # Normalize angle 0-360 if needed, but RPLidar gives 0-360
                            # Check front sector -45 to 45
                            # 0 is usually front for RPLidar A1/A2/C1? Assuming yes.
                            # Angles > 315 are also front (315-360 is -45 to 0)

                            check_angle = angle
                            if check_angle > 180: check_angle -= 360

                            if -45 <= check_angle <= 45:
                                front_min = min(front_min, dist / 1000.0)

                            scan_data.append((angle, dist))

                    self.min_lidar_distance_m = front_min

                    # Option A: Resume
                    if self.lidar_error_active:
                        self.lidar_error_active = False
                        print("LIDAR recovered.")

                    if self.is_recording:
                        current_time = time.time()

                        with self.buffer_lock:
                            states = list(self.state_buffer)
                            self.state_buffer.clear()

                        serialized_states = [asdict(s) for s in states]

                        frame = LidarFrame(
                            timestamp=current_time,
                            frame_id=len(self.frames),
                            scan_data=scan_data,
                            controller_states=serialized_states
                        )

                        self.frames.append(asdict(frame))

            except (RPLidarException, Exception) as e:
                print(f"LIDAR Error: {e}")
                self.lidar_error_active = True

                # Try to reconnect
                try:
                    lidar.stop()
                    lidar.disconnect()
                    time.sleep(1.0)
                    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
                except:
                    time.sleep(1.0)

        lidar.stop()
        lidar.disconnect()

    def update_odometry(self, dt):
        # Basic odometry based on duty cycle
        MAX_SPEED = 0.5 # m/s (estimated)

        vl = (self.current_duty_left / MAX_DUTY_CYCLE) * MAX_SPEED
        vr = (self.current_duty_right / MAX_DUTY_CYCLE) * MAX_SPEED

        if not self.is_moving_forward:
            vl = -vl
            vr = -vr

        v = (vl + vr) / 2
        omega = (vr - vl) / WHEEL_BASE_M

        self.pose.x += v * math.cos(self.pose.theta) * dt
        self.pose.y += v * math.sin(self.pose.theta) * dt
        self.pose.theta += omega * dt
        self.pose.timestamp = time.time()

    def apply_motor_control(self, trigger_l, trigger_r, tank_turn):
        if self.lidar_error_active:
            self.pwm_left.change_duty_cycle(0)
            self.pwm_right.change_duty_cycle(0)
            self.current_duty_left = 0.0
            self.current_duty_right = 0.0
            return

        throttle_l = (trigger_l + 1) / 2
        throttle_r = (trigger_r + 1) / 2

        pwm_l = throttle_l * MAX_DUTY_CYCLE
        pwm_r = throttle_r * MAX_DUTY_CYCLE

        # Collision Avoidance
        with self.us_lock:
            us_dist = self.min_us_distance_cm

        lidar_dist = self.min_lidar_distance_m

        factor = 1.0
        if self.is_moving_forward:
            if us_dist < 15.0:
                factor = 0.0
            elif us_dist < COLLISION_DISTANCE_CM or lidar_dist < LIDAR_COLLISION_DISTANCE_M:
                factor = 0.5

        pwm_l *= factor
        pwm_r *= factor

        # Direction Control
        if self.is_moving_forward:
            level = GPIO.LOW if DIR_HIGH_IS_BACKWARD else GPIO.HIGH
            GPIO.output(DIR_LEFT_PIN, level)
            GPIO.output(DIR_RIGHT_PIN, level)
        else:
            level = GPIO.HIGH if DIR_HIGH_IS_BACKWARD else GPIO.LOW
            GPIO.output(DIR_LEFT_PIN, level)
            GPIO.output(DIR_RIGHT_PIN, level)

        self.pwm_left.change_duty_cycle(pwm_l)
        self.pwm_right.change_duty_cycle(pwm_r)

        self.current_duty_left = pwm_l
        self.current_duty_right = pwm_r

    def start_recording(self):
        self.is_recording = True
        self.session_id = str(uuid.uuid4())
        self.session_start_time = time.time()
        self.frames = []
        print(f"Recording started: {self.session_id}")

    def stop_recording(self):
        self.is_recording = False
        print("Recording stopped. Saving...")

        session = RecordingSession(
            session_id=self.session_id,
            start_timestamp=self.session_start_time,
            end_timestamp=time.time(),
            hardware_info={"platform": "Raspberry Pi 5"},
            frames=self.frames
        )

        self.storage.save_session(session)
        print("Saved.")

    def run(self):
        print("RaspiRobotMapper starting...")
        try:
            clock = pygame.time.Clock()
            while self.is_running:
                dt = clock.tick(CONTROL_LOOP_HZ) / 1000.0

                # Event handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.is_running = False
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == BUTTON_START:
                            self.is_running = False
                        elif event.button == BUTTON_B:
                            self.is_moving_forward = not self.is_moving_forward
                            print(f"Direction: {'Forward' if self.is_moving_forward else 'Backward'}")

                if self.controller:
                    b_x = self.controller.get_button(BUTTON_X)
                    b_y = self.controller.get_button(BUTTON_Y)

                    if b_x and b_y:
                        if not self.is_recording:
                            self.start_recording()
                            time.sleep(0.5)
                        else:
                            self.stop_recording()
                            time.sleep(0.5)

                    trig_l = self.controller.get_axis(AXIS_LEFT_TRIGGER)
                    trig_r = self.controller.get_axis(AXIS_RIGHT_TRIGGER)

                    self.apply_motor_control(trig_l, trig_r, False)

                self.update_odometry(dt)

                # Buffer State
                if self.is_recording and not self.lidar_error_active:
                    state = ControllerState(
                        timestamp=time.time(),
                        left_duty=self.current_duty_left,
                        right_duty=self.current_duty_right,
                        is_moving_forward=self.is_moving_forward,
                        pose=asdict(self.pose)
                    )
                    with self.buffer_lock:
                        self.state_buffer.append(state)

        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Main Loop Error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        self.is_running = False
        try:
            self.pwm_left.stop()
            self.pwm_right.stop()
        except: pass
        GPIO.cleanup()
        pygame.quit()
        print("Cleanup done.")

if __name__ == "__main__":
    robot = RaspiRobotMapper()
    robot.run()
