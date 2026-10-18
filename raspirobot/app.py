"""Manuelle Fahrt mit Xbox-Controller, Kollisionsschutz und Aufzeichnung.

Ablauf und Steuerlogik entsprechen ``xbox_controller_with_recording_v3.0.1.py``.

Threads:
    Sensor (Ultraschall, 10 Hz), LIDAR (je Scan), Pose (50 Hz), LED (20 Hz)
    und der Steuerungs-Loop im Hauptthread (50 Hz).
"""

import math
import signal
import sys
import threading
import time
import traceback
from pathlib import Path

import pygame
import RPi.GPIO as GPIO

from . import config
from .collision import CollisionStatus, analyze_collision
from .drive import compute_drive_command, limit_for_collision, trigger_to_duty_cycle
from .hardware import ultrasonic
from .hardware.lidar import Lidar
from .hardware.motors import Motors
from .hardware.status_led import StatusIndicator
from .hardware.ultrasonic import UltrasonicSensors
from .hardware.xbox_controller import XboxController
from .localization import Localizer
from .log import logger
from .odometry import PoseTracker
from .recorder import Recorder


class RobotApp:
    # Unterklassen setzen True, damit die Odometrie die Richtung je Rad auswertet (Tank-Turn als Drehung).
    USE_WHEEL_DIRECTIONS = False

    def __init__(self):
        self.running = False
        self.shutdown_event = threading.Event()

        self.ultrasonic = UltrasonicSensors()
        self.motors = Motors()
        self.controller = XboxController()
        self.lidar = Lidar()
        self.status = StatusIndicator()
        self.pose = PoseTracker(
            is_moving_forward=lambda: self.controller.state.is_moving_forward,
            use_wheel_directions=self.USE_WHEEL_DIRECTIONS,
        )
        self.localizer = Localizer(self.pose.get_pose) if config.SLAM_ENABLED else None
        self.recorder = self._create_recorder()
        self._threads = []

    def _create_recorder(self) -> Recorder:
        return Recorder(
            status_indicator=self.status,
            reset_pose=self.pose.reset,
            lidar_connected=lambda: self.lidar.connected,
        )

    def _on_lidar_scan(self, timestamp: float, scan, scan_index: int) -> None:
        """Wird vom LIDAR-Thread je Scan aufgerufen; beide Empfänger blockieren nicht."""
        self.recorder.offer_lidar_scan(timestamp, scan, scan_index)
        if self.localizer:
            self.localizer.offer_scan(timestamp, scan, scan_index)

    def should_run(self) -> bool:
        return self.running and not self.shutdown_event.is_set()

    # ------------------------------------------------------------------ Setup

    def setup_gpio(self) -> None:
        print("[SETUP] Initialisiere GPIO...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        ultrasonic.setup_pins()
        self.motors.setup_direction_pins()
        self.status.setup_pins()
        signal.signal(signal.SIGINT, self._handle_sigint)
        logger.info("GPIO initialisiert")
        print("[SETUP] GPIO initialisiert - Sensoren stabilisieren (2s)...")
        time.sleep(2)

    def _handle_sigint(self, sig, frame) -> None:
        print("\n[SHUTDOWN] SIGINT empfangen - Programm wird beendet...")
        self.shutdown_event.set()

        if self.recorder.active and self.recorder.data:
            duration = time.time() - self.recorder.start_time
            if duration >= config.MIN_RECORDING_DURATION_SEC:
                print("[SHUTDOWN] 🔄 Speichere aktive Aufzeichnung...")
                self.recorder.stop(force_save=True)
            else:
                print("[SHUTDOWN] ⚠️ Recording zu kurz - wird verworfen")
                self.recorder.stop(force_save=False)

        self.cleanup()
        sys.exit(0)

    def _start_thread(self, name: str, target) -> None:
        thread = threading.Thread(target=target, args=(self.should_run,), daemon=True, name=name)
        thread.start()
        self._threads.append((name, thread))

    # -------------------------------------------------------------- Steuerung

    def _current_collision(self) -> CollisionStatus:
        return analyze_collision(
            self.ultrasonic.distance_left, self.ultrasonic.distance_right, self.lidar.latest_scan
        )

    def _apply_motor_speed(self, left_duty: float, right_duty: float) -> None:
        collision = self._current_collision()
        if collision.collision_critical:
            print("[COLLISION] 🛑 KRITISCH - Alle Motoren gestoppt!")
        left_duty, right_duty = limit_for_collision(left_duty, right_duty, collision)
        self.pose.set_duty(left_duty, right_duty)
        self.motors.set_duty(left_duty, right_duty)

    def _read_controller(self) -> None:
        """Liest den Controller und führt Recording-Toggle und Beenden aus."""
        pygame.event.pump()

        if not self.controller.is_connected():
            print("[ERROR] Controller Verbindung verloren!")
            self.running = False
            return

        try:
            state = self.controller.read()
            if state.recording_toggle_requested:
                if self.recorder.active:
                    print("[INPUT] ⏹️ Stoppe Recording...")
                    self.recorder.stop(force_save=False)
                else:
                    print("[INPUT] 🔴 Starte Recording...")
                    self.recorder.start()
            if state.quit_requested:
                print("[INPUT] 🎮 Start-Button - Beende...")
                self.running = False
                self.shutdown_event.set()
        except Exception as e:
            print(f"[ERROR] Controller Fehler: {e}")
            logger.error(f"Controller Fehler: {e}")
            self.running = False

    def control_step(self):
        """Ein Zyklus: Controller lesen, Motoren setzen. Gibt die Duty Cycles vor Kollisionsbegrenzung zurück."""
        self._read_controller()
        state = self.controller.state

        left_duty = trigger_to_duty_cycle(state.trigger_left)
        right_duty = trigger_to_duty_cycle(state.trigger_right)

        cmd = compute_drive_command(left_duty, right_duty, state.a_pressed, state.is_moving_forward)
        self.motors.set_direction(cmd.left_backward, cmd.right_backward)
        self.pose.set_wheel_directions(cmd.left_backward, cmd.right_backward)
        self._apply_motor_speed(cmd.left_duty, cmd.right_duty)
        return left_duty, right_duty

    def _print_banner(self) -> None:
        print(f"[CONTROL] Steuerungs-Loop gestartet ({config.CONTROL_LOOP_HZ} Hz)")
        print("\n" + "=" * 80)
        print("🎮 STEUERUNG:")
        print("  Left/Right Trigger → Motor-Steuerung")
        print("  A Button (halten) → Tank-Turn")
        print("  B Button → Richtung (Vorwärts ⇄ Rückwärts)")
        print("  Y + X Buttons → Recording Start/Stop")
        print("  Start Button → Beenden")
        print("\n🔴 RECORDING:")
        print("  Y + X gleichzeitig → Recording Toggle")
        print(f"  Mindestdauer: {config.MIN_RECORDING_DURATION_SEC}s")
        print(f"  Speicherort: {config.RECORDING_BASE_DIR}/")
        print("\n🛡️ SICHERHEIT:")
        print(f"  Ultraschall: {config.COLLISION_DISTANCE_CM}cm")
        print(f"  LIDAR: {config.LIDAR_COLLISION_DISTANCE_M * 100}cm")
        print(f'  LIDAR Status: {"✅ Verbunden" if self.lidar.connected else "❌ Nicht verfügbar"}')
        print("=" * 80 + "\n")

    def _print_status(self, left_duty: float, right_duty: float) -> None:
        state = self.controller.state
        collision = self._current_collision()
        pose = self.pose.get_pose()

        collision_emoji = "🔴" if collision.collision_detected else "🟢"
        direction_emoji = "⬆️" if state.is_moving_forward else "⬇️"
        mode_text = "🔄 TANK" if state.a_pressed else "➡️ NORM"

        if self.recorder.active:
            duration = time.time() - self.recorder.start_time
            stats = self.recorder.data.get_stats() if self.recorder.data else {}
            recording_text = (
                f" | 🔴 REC:{duration:.1f}s C:{stats.get('controller_samples', 0)} L:{stats.get('lidar_frames', 0)}"
            )
        else:
            recording_text = " | ⚪ REC:OFF"

        lidar_text = f"LIDAR:{len(self.lidar.latest_scan)} pts" if self.lidar.connected else "LIDAR:N/A"

        slam_text = ""
        if self.localizer and self.lidar.connected:
            loc = self.localizer.get_pose()
            slam_text = (
                f" | SLAM[{loc.status} {loc.score:.2f}]:X={loc.x:.2f}m Y={loc.y:.2f}m "
                f"θ={math.degrees(loc.theta):.0f}°"
            )

        print(
            f"[STATUS] {collision_emoji} {mode_text} {direction_emoji} | "
            f"Trigger:L={state.trigger_left:+.2f} R={state.trigger_right:+.2f} | "
            f"PWM:L={left_duty:.1f}% R={right_duty:.1f}% | "
            f"US:L={collision.ultrasonic_left:.0f}cm R={collision.ultrasonic_right:.0f}cm | "
            f"{lidar_text} | "
            f"Odom:X={pose.x:.2f}m Y={pose.y:.2f}m θ={math.degrees(pose.theta):.0f}°"
            f"{slam_text}"
            f"{recording_text}"
        )

    def control_loop(self) -> None:
        self._print_banner()

        sleep_time = 1.0 / config.CONTROL_LOOP_HZ
        status_counter = 0
        recording_sample_counter = 0
        recording_sample_interval = config.CONTROL_LOOP_HZ // config.RECORDING_SAMPLE_RATE_HZ

        while self.should_run():
            loop_start = time.time()

            left_duty, right_duty = self.control_step()

            recording_sample_counter += 1
            if recording_sample_counter >= recording_sample_interval:
                recording_sample_counter = 0
                if self.recorder.active:
                    self.recorder.add_sample(self.controller.state)

            status_counter += 1
            if status_counter >= config.CONTROL_LOOP_HZ:  # einmal pro Sekunde
                status_counter = 0
                self._print_status(left_duty, right_duty)

            sleep_duration = sleep_time - (time.time() - loop_start)
            if sleep_duration > 0:
                time.sleep(sleep_duration)

    # ---------------------------------------------------------------- Cleanup

    def cleanup(self) -> None:
        print("[CLEANUP] Starte Ressourcen-Freigabe...")
        self.running = False
        self.shutdown_event.set()

        if self.recorder.active:
            print("[CLEANUP] 💾 Sichere Recording mit vollständigem Buffer-Flush...")
            self.recorder.stop(force_save=True)

        for name, thread in self._threads:
            if thread.is_alive():
                print(f"[CLEANUP] Warte auf {name} Thread...")
                thread.join(timeout=config.THREAD_SHUTDOWN_TIMEOUT_SEC)
                if thread.is_alive():
                    print(f"[CLEANUP] ⚠️ {name} Thread Timeout - fortfahren...")

        self.motors.stop()
        self.lidar.shutdown()
        self._save_map()
        self.status.all_off()
        self.controller.close()

        try:
            GPIO.cleanup()
            print("[CLEANUP] GPIO bereinigt")
        except Exception as e:
            print(f"[CLEANUP] GPIO Cleanup Fehler: {e}")

        logger.info("System heruntergefahren")
        print("[CLEANUP] ✅ Cleanup abgeschlossen")

    def _save_map(self) -> None:
        if not self.localizer or self.localizer.grid.scans_integrated == 0:
            return
        base = Path(config.RECORDING_BASE_DIR) / "maps" / time.strftime("map_%Y%m%d_%H%M%S")
        try:
            npz, pgm = self.localizer.save_map(str(base))
            print(f"[CLEANUP] 🗺️ Karte gespeichert: {pgm}")
            logger.info(f"Karte gespeichert: {npz}")
        except Exception as e:
            print(f"[CLEANUP] ⚠️ Karte nicht gespeichert: {e}")
            logger.error(f"Karte nicht gespeichert: {e}")

    # ------------------------------------------------------------------- Main

    def run(self) -> None:
        print("\n" + "=" * 80)
        print("🤖 XBOX CONTROLLER MIT LIDAR-INTEGRATION UND RECORDING")
        print("=" * 80 + "\n")

        try:
            self.setup_gpio()
            self.motors.setup_pwm()
            self.controller.connect()
            lidar_ok = self.lidar.connect()
            self.recorder.setup()

            print("\n[INIT] ✅ Initialisierung abgeschlossen - Starte Threads...")
            self.running = True
            self.shutdown_event.clear()

            self._start_thread("Sensor", self.ultrasonic.run)
            if lidar_ok:
                if self.localizer:
                    self._start_thread("Localization", self.localizer.run)
                self._start_thread("LIDAR", lambda should_run: self.lidar.run(should_run, self._on_lidar_scan))
            self._start_thread("Pose", self.pose.run)
            if config.LED_ENABLED:
                self._start_thread("LED", self.status.run)

            time.sleep(1.0)
            self.control_loop()
        except KeyboardInterrupt:
            print("\n[SHUTDOWN] Keyboard Interrupt...")
        except Exception as e:
            print(f"\n[ERROR] Unerwarteter Fehler: {e}")
            logger.error(f"Fehler in main(): {e}")
            traceback.print_exc()
        finally:
            self.cleanup()
