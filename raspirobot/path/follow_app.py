"""Fährt einen mit ``run_path_recording.py`` aufgezeichneten Weg autonom ab.

Modi:
    weg       (Standard) Regelung auf die Wegpunkte aus ``*_path.json.gz``,
              Pose aus SLAM, ohne LIDAR aus der Odometrie.
    eingaben  Spielt die Controller-Eingaben der Aufnahme ``*.json.gz``
              zeitgetreu ab, ohne Positionsregelung.

Der Controller bleibt Not-Aus: A startet, B, Start oder ein gedrückter Trigger
halten sofort an und beenden. Kollisionsschutz wie bei der manuellen Fahrt;
dauerhafte Hindernisse, verlorene Pose, zu große Abweichung vom Weg oder
fehlender Fortschritt brechen ab. Beim Beenden stehen die Motoren.
"""

import gzip
import json
import math
import time
from pathlib import Path
from typing import Optional

import pygame

from .. import config
from ..app import RobotApp
from ..drive import limit_for_collision
from ..log import logger
from ..recording.storage import PathRecordingStorageManager
from .follower import PHASE_DONE, STOP, InputReplayer, PathFollower
from .waypoints import SOURCE_SLAM, PoseSource, densify, load_path

MODE_PATH = "weg"
MODE_INPUTS = "eingaben"

STATE_WAITING = "WARTEN"
STATE_DRIVING = "FAHRT"
STATE_FINISHED = "FERTIG"
STATE_ABORTED = "ABBRUCH"


class PathFollowApp(RobotApp):
    USE_WHEEL_DIRECTIONS = True

    def __init__(self, filepath, mode: str = MODE_PATH):
        super().__init__()
        self.mode = mode
        self.filepath = Path(filepath)
        self.pose_source = PoseSource(self.pose.get_pose, self.localizer, slam_available=lambda: self.lidar.connected)
        self.follower: Optional[PathFollower] = None
        self.replayer: Optional[InputReplayer] = None
        self.recorded_source = None

        self.state = STATE_WAITING
        self.result = ""
        self.last_step = None
        self._start_time = 0.0
        self._replay_elapsed = 0.0
        self._last_tick = 0.0
        self._a_was_pressed = False
        self._blocked_since: Optional[float] = None
        self._pose_bad_since: Optional[float] = None
        self._best_progress = 0.0
        self._progress_time = 0.0
        self._trace: list = []
        self._trace_time = 0.0

    # ----------------------------------------------------------------- Laden

    def load(self) -> None:
        if self.mode == MODE_PATH:
            path = load_path(self.filepath)
            # Lücken (z. B. aus SLAM-Sprüngen) auffüllen, damit die Suche nach dem
            # nächstgelegenen Wegpunkt nicht an groben Abständen hängt.
            self.follower = PathFollower(densify(path.waypoints))
            self.recorded_source = path.pose_source
            print(f"[FOLLOW] Weg geladen: {self.filepath} ({len(path.waypoints)} Wegpunkte, "
                  f"{path.length_m():.2f} m, aufgezeichnet mit {path.pose_source.upper()})")
        elif self.mode == MODE_INPUTS:
            storage = PathRecordingStorageManager(str(self.filepath.parent), backup_enabled=False)
            recording = storage.load_json(self.filepath)
            self.replayer = InputReplayer(recording.controller_samples)
            print(f"[FOLLOW] Aufnahme geladen: {self.filepath} ({len(self.replayer.samples)} Samples, "
                  f"{self.replayer.duration:.1f} s)")
        else:
            raise ValueError(f"Unbekannter Modus: {self.mode}")

    # ------------------------------------------------------------- Zustände

    def _stop_motors(self) -> None:
        self.motors.set_direction(False, False)
        self.pose.set_wheel_directions(False, False)
        self.pose.set_duty(0.0, 0.0)
        self.motors.set_duty(0.0, 0.0)

    def _record_trace(self, now: float, cmd, left: float, right: float) -> None:
        """Alle 0,1 s ein Eintrag; ``_write_trace`` speichert sie beim Beenden zur Auswertung."""
        if now - self._trace_time < 0.1 or self.mode != MODE_PATH:
            return
        self._trace_time = now
        sample = self.pose_source.sample()
        s = self.last_step
        self._trace.append({
            "t": round(now - self._start_time, 3),
            "x": round(sample.pose[0], 4), "y": round(sample.pose[1], 4), "theta": round(sample.pose[2], 4),
            "status": sample.status, "age": round(sample.age, 2),
            "score": None if sample.score is None else round(sample.score, 3),
            "odom": [round(v, 4) for v in sample.odom],
            "phase": s.phase if s else "", "index": s.index if s else -1, "target": s.target_index if s else -1,
            "deviation": round(s.deviation_m, 3) if s else None,
            "heading_error": round(s.heading_error, 3) if s else None,
            "cmd": [cmd.left_backward, cmd.right_backward, round(cmd.left_duty, 2), round(cmd.right_duty, 2)],
            "duty": [round(left, 2), round(right, 2)],
        })

    def _write_trace(self, message: str) -> None:
        if not self._trace:
            return
        name = f"{self.filepath.name.split('.')[0]}_follow_{time.strftime('%Y%m%d_%H%M%S')}.json.gz"
        target = Path(config.RECORDING_BASE_DIR) / name
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            data = {"format": "raspirobot-follow-trace", "version": 1, "path_file": self.filepath.name,
                    "result": message, "source": self.pose_source.source, "trace": self._trace}
            target.write_bytes(gzip.compress(json.dumps(data).encode("utf-8")))
            print(f"[FOLLOW] 📝 Fahrprotokoll: {target}")
        except Exception as e:
            logger.error(f"Fahrprotokoll nicht gespeichert: {e}")

    def _finish(self, state: str, message: str) -> None:
        self._write_trace(message)
        self._stop_motors()
        self.state = state
        self.result = message
        self.running = False
        icon = "✅" if state == STATE_FINISHED else "🛑"
        print(f"[FOLLOW] {icon} {message}")
        (logger.info if state == STATE_FINISHED else logger.warning)(f"Abfahren beendet: {message}")

    def abort(self, message: str) -> None:
        self._finish(STATE_ABORTED, message)

    def _begin(self) -> bool:
        if self.mode == MODE_PATH:
            if self.recorded_source == SOURCE_SLAM and not self.pose_source.slam_ready():
                print("[FOLLOW] ⏳ SLAM noch nicht bereit (LIDAR prüfen) - A erneut drücken")
                return False
            source = None
            if self.recorded_source == SOURCE_SLAM:
                source = self.pose_source.restart_slam()  # frische Karte wie beim Aufzeichnen
            source = source or self.pose_source.set_origin()
            if source != self.recorded_source:
                print(f"[FOLLOW] ⚠️ Aufgezeichnet mit {self.recorded_source.upper()}, "
                      f"abgefahren mit {source.upper()}")
        self.state = STATE_DRIVING
        now = time.time()
        self._start_time = now
        self._last_tick = now
        self._replay_elapsed = 0.0
        self._progress_time = now
        print("[FOLLOW] ▶️ Fahrt gestartet")
        return True

    # ----------------------------------------------------------- Controller

    def _read_controller(self) -> None:
        """Not-Aus und Start. Ohne Controller wird abgebrochen."""
        pygame.event.pump()
        if not self.controller.is_connected():
            self.abort("Controller-Verbindung verloren")
            return
        js = self.controller.joystick
        if js.get_button(config.BUTTON_B) or js.get_button(config.BUTTON_START):
            self.abort("Not-Aus per Controller")
            self.shutdown_event.set()
            return
        if self.state == STATE_DRIVING and (
            js.get_axis(config.AXIS_LEFT_TRIGGER) > config.FOLLOW_ABORT_TRIGGER
            or js.get_axis(config.AXIS_RIGHT_TRIGGER) > config.FOLLOW_ABORT_TRIGGER
        ):
            self.abort("Not-Aus: Trigger gedrückt")
            return
        a_pressed = bool(js.get_button(config.BUTTON_A))
        if self.state == STATE_WAITING and a_pressed and not self._a_was_pressed:
            self._begin()
        self._a_was_pressed = a_pressed

    # ------------------------------------------------------------ Steuerung

    def _next_command(self, now: float):
        if self.mode == MODE_INPUTS:
            # Die Zeitachse steht, solange ein Hindernis den Roboter aufhält.
            if self._blocked_since is None:
                self._replay_elapsed += now - self._last_tick
            self._last_tick = now
            cmd = self.replayer.command_at(self._replay_elapsed)
            if cmd is None:
                self._finish(STATE_FINISHED, "Aufnahme vollständig abgespielt")
            return cmd

        if not self.pose_source.is_reliable():
            if self._pose_bad_since is None:
                self._pose_bad_since = now
                print("[FOLLOW] ⚠️ Pose unsicher (SLAM LOST oder veraltet) - warte")
            if now - self._pose_bad_since > config.FOLLOW_POSE_LOST_TIMEOUT_SEC:
                self.abort("Position verloren")
            return None
        if self._pose_bad_since is not None:
            # Wartezeit zählt nicht als fehlender Fortschritt.
            self._progress_time += now - self._pose_bad_since
            self._pose_bad_since = None

        pose = self.pose_source.get()[0]
        step = self.follower.update(pose)
        self.last_step = step
        if step.phase == PHASE_DONE:
            self._finish(STATE_FINISHED, f"Ziel erreicht ({self.follower.length_m:.2f} m)")
            return None
        if step.deviation_m > config.FOLLOW_MAX_DEVIATION_M:
            self.abort(f"Zu weit vom Weg entfernt ({step.deviation_m:.2f} m)")
            return None
        if step.progress_m > self._best_progress + 0.05:
            self._best_progress = step.progress_m
            self._progress_time = now
        elif now - self._progress_time > config.FOLLOW_NO_PROGRESS_TIMEOUT_SEC:
            self.abort("Kein Fortschritt auf dem Weg")
            return None
        return step.command

    def control_step(self):
        self._read_controller()
        if self.state != STATE_DRIVING:
            self._stop_motors()
            return 0.0, 0.0

        now = time.time()
        try:
            cmd = self._next_command(now)
        except Exception as e:
            self.abort(f"Fehler im Regler: {e}")
            raise
        if self.state != STATE_DRIVING:
            return 0.0, 0.0
        if cmd is None:
            cmd = STOP

        collision = self._current_collision()
        moving = cmd.left_duty > 0 or cmd.right_duty > 0
        if collision.collision_critical and moving:
            if self._blocked_since is None:
                self._blocked_since = now
                print("[COLLISION] 🛑 Hindernis - warte")
            if now - self._blocked_since > config.FOLLOW_OBSTACLE_TIMEOUT_SEC:
                self.abort("Hindernis bleibt im Weg")
                return 0.0, 0.0
        elif self._blocked_since is not None:
            self._progress_time += now - self._blocked_since
            self._blocked_since = None

        self.motors.set_direction(cmd.left_backward, cmd.right_backward)
        self.pose.set_wheel_directions(cmd.left_backward, cmd.right_backward)
        left, right = limit_for_collision(cmd.left_duty, cmd.right_duty, collision)
        self.pose.set_duty(left, right)
        self.motors.set_duty(left, right)
        self._record_trace(now, cmd, left, right)
        return cmd.left_duty, cmd.right_duty

    # --------------------------------------------------------------- Ausgabe

    def _print_banner(self) -> None:
        print("\n" + "=" * 80)
        print(f"🧭 WEG ABFAHREN ({self.mode}): {self.filepath}")
        print("  Roboter an den Startpunkt der Aufnahme stellen, gleiche Ausrichtung.")
        print("  A → Start | B / Start → Not-Aus und Beenden | Trigger während der Fahrt → Not-Aus")
        print(f"  Kollisionsschutz: Ultraschall {config.COLLISION_DISTANCE_CM}cm, "
              f"LIDAR {config.LIDAR_COLLISION_DISTANCE_M * 100:.0f}cm")
        print(f'  LIDAR: {"✅ Verbunden" if self.lidar.connected else "❌ Nicht verfügbar"}')
        print("=" * 80 + "\n")

    def _print_status(self, left_duty: float, right_duty: float) -> None:
        collision = self._current_collision()
        text = f"[FOLLOW] {self.state}"
        if self.mode == MODE_PATH:
            (x, y, theta), status, _ = self.pose_source.get()
            text += f" | {self.pose_source.source.upper()}[{status}] X={x:.2f}m Y={y:.2f}m θ={math.degrees(theta):.0f}°"
            if self.last_step:
                s = self.last_step
                text += (f" | {s.phase} WP {s.index}/{len(self.follower.waypoints) - 1} "
                         f"{s.progress_m:.2f}/{self.follower.length_m:.2f}m Abw={s.deviation_m:.2f}m")
        elif self.state == STATE_DRIVING:
            text += f" | {self._replay_elapsed:.1f}/{self.replayer.duration:.1f}s"
        text += (f" | PWM:L={left_duty:.1f}% R={right_duty:.1f}% | "
                 f"US:L={collision.ultrasonic_left:.0f}cm R={collision.ultrasonic_right:.0f}cm")
        print(text)

    def run(self) -> None:
        self.load()
        super().run()
        if self.result:
            print(f"[FOLLOW] Ergebnis: {self.result}")
