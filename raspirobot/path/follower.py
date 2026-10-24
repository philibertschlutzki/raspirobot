"""Regler zum Abfahren eines aufgezeichneten Wegs (reine Logik, ohne Hardware).

``PathFollower``: Pure Pursuit auf den Wegpunkten. Der Zielpunkt liegt
``FOLLOW_LOOKAHEAD_M`` voraus auf dem Weg; ist der Richtungsfehler größer als
``FOLLOW_TURN_IN_PLACE_RAD``, dreht der Roboter auf der Stelle, sonst fährt er
einen Bogen auf den Zielpunkt. Wegstücke, die rückwärts aufgezeichnet wurden,
fährt er rückwärts. Am Ende dreht er auf die aufgezeichnete Endausrichtung.

``InputReplayer``: spielt die aufgezeichneten Controller-Eingaben zeitgetreu
ab, ohne Positionsregelung (geht auch mit Aufnahmen ohne Wegdatei).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

from .. import config
from ..drive import DriveCommand, compute_drive_command, trigger_to_duty_cycle
from ..localization.geometry import Pose, wrap_angle
from .waypoints import Waypoint

PHASE_FOLLOW = "FOLGEN"
PHASE_TURN = "DREHEN"
PHASE_ALIGN = "AUSRICHTEN"
PHASE_DONE = "ZIEL"


@dataclass(frozen=True)
class FollowStep:
    command: DriveCommand
    phase: str
    index: int              # nächstgelegener Wegpunkt
    target_index: int       # Zielpunkt des Reglers
    progress_m: float       # zurückgelegte Weglänge bis ``index``
    deviation_m: float      # Abstand zum nächstgelegenen Wegpunkt
    heading_error: float    # rad

    @property
    def finished(self) -> bool:
        return self.phase == PHASE_DONE


STOP = DriveCommand(False, False, 0.0, 0.0)


def _turn_command(direction: float, duty: float) -> DriveCommand:
    """Auf der Stelle drehen; ``direction`` > 0 gegen den Uhrzeigersinn (links)."""
    left_backward = direction > 0
    return DriveCommand(left_backward, not left_backward, duty, duty)


class PathFollower:
    def __init__(
        self,
        waypoints: Sequence[Waypoint],
        drive_duty: float = config.FOLLOW_DUTY,
        turn_duty: float = config.FOLLOW_TURN_DUTY,
        lookahead_m: float = config.FOLLOW_LOOKAHEAD_M,
        align_final_heading: bool = config.FOLLOW_ALIGN_FINAL_HEADING,
    ):
        if len(waypoints) < 2:
            raise ValueError("Mindestens zwei Wegpunkte nötig")
        self.waypoints: List[Waypoint] = list(waypoints)
        self.drive_duty = min(drive_duty, config.MAX_DUTY_CYCLE)
        self.turn_duty = min(turn_duty, config.MAX_DUTY_CYCLE)
        self.lookahead_m = lookahead_m
        self.align_final_heading = align_final_heading

        self._s = [0.0]
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            self._s.append(self._s[-1] + math.hypot(b.x - a.x, b.y - a.y))
        self.length_m = self._s[-1]

        self.index = 0
        self.phase = PHASE_FOLLOW
        self._turning = False

    # ---------------------------------------------------------------- Hilfen

    def _dist(self, pose: Pose, i: int) -> float:
        w = self.waypoints[i]
        return math.hypot(w.x - pose[0], w.y - pose[1])

    def _update_index(self, pose: Pose) -> None:
        """Nächstgelegener Wegpunkt, nur vorwärts und höchstens FOLLOW_SEARCH_AHEAD_M voraus.

        So springt der Index bei Wegen, die sich kreuzen oder zum Start
        zurückführen, nicht auf einen späteren Abschnitt.
        """
        limit = self._s[self.index] + config.FOLLOW_SEARCH_AHEAD_M
        best, best_d = self.index, self._dist(pose, self.index)
        i = self.index + 1
        while i < len(self.waypoints) and self._s[i] <= limit:
            d = self._dist(pose, i)
            if d < best_d:
                best, best_d = i, d
            i += 1
        self.index = best

    def _target_index(self, pose: Pose) -> int:
        for i in range(self.index, len(self.waypoints)):
            if self._dist(pose, i) >= self.lookahead_m:
                return i
            # Bei einem Richtungswechsel (vorwärts/rückwärts) nicht darüber hinaus zielen.
            if i + 1 < len(self.waypoints) and self.waypoints[i + 1].backward != self.waypoints[self.index].backward:
                return i
        return len(self.waypoints) - 1

    def _step(self, cmd, phase, target, pose, heading_error) -> FollowStep:
        self.phase = phase
        return FollowStep(
            command=cmd,
            phase=phase,
            index=self.index,
            target_index=target,
            progress_m=self._s[self.index],
            deviation_m=self._dist(pose, self.index),
            heading_error=heading_error,
        )

    # ------------------------------------------------------------------ Kern

    def update(self, pose: Pose) -> FollowStep:
        last = len(self.waypoints) - 1
        if self.phase == PHASE_DONE:
            return self._step(STOP, PHASE_DONE, last, pose, 0.0)

        if self.phase != PHASE_ALIGN:
            self._update_index(pose)
            remaining = self.length_m - self._s[self.index]
            if self._dist(pose, last) <= config.FOLLOW_GOAL_TOLERANCE_M and remaining <= 2 * self.lookahead_m:
                self.index = last
                self.phase = PHASE_ALIGN
                self._turning = False

        if self.phase == PHASE_ALIGN:
            error = wrap_angle(self.waypoints[last].theta - pose[2])
            if not self.align_final_heading or abs(error) <= config.FOLLOW_FINAL_HEADING_TOLERANCE_RAD:
                return self._step(STOP, PHASE_DONE, last, pose, error)
            return self._step(_turn_command(error, self.turn_duty), PHASE_ALIGN, last, pose, error)

        target = self._target_index(pose)
        wp = self.waypoints[target]
        backward = wp.backward
        heading = pose[2] + (math.pi if backward else 0.0)
        alpha = float(wrap_angle(math.atan2(wp.y - pose[1], wp.x - pose[0]) - heading))

        # Hysterese: einmal auf der Stelle drehen, bis der Fehler klein ist.
        threshold = config.FOLLOW_TURN_DONE_RAD if self._turning else config.FOLLOW_TURN_IN_PLACE_RAD
        self._turning = abs(alpha) > threshold
        if self._turning:
            return self._step(_turn_command(alpha, self.turn_duty), PHASE_TURN, target, pose, alpha)

        # Pure Pursuit: Krümmung k = 2 sin(alpha) / L, Radgeschwindigkeit v (1 ∓ k b/2).
        dist = max(self._dist(pose, target), 1e-3)
        k = 2.0 * math.sin(alpha) / dist
        half = k * config.WHEEL_BASE_M / 2.0
        v_left = self.drive_duty * (1.0 - half)
        v_right = self.drive_duty * (1.0 + half)
        scale = max(v_left, v_right) / self.drive_duty
        if scale > 1.0:
            v_left, v_right = v_left / scale, v_right / scale
        v_left = max(config.MIN_DUTY_CYCLE, min(config.MAX_DUTY_CYCLE, v_left))
        v_right = max(config.MIN_DUTY_CYCLE, min(config.MAX_DUTY_CYCLE, v_right))

        if backward:
            # Rückwärts ist das physische linke Rad das virtuelle rechte.
            cmd = DriveCommand(True, True, v_right, v_left)
        else:
            cmd = DriveCommand(False, False, v_left, v_right)
        return self._step(cmd, PHASE_FOLLOW, target, pose, alpha)


class InputReplayer:
    """Gibt zu einer Zeit seit Start den Fahrbefehl des aufgezeichneten Controller-Samples zurück."""

    def __init__(self, samples):
        if not samples:
            raise ValueError("Aufnahme enthält keine Controller-Samples")
        self.samples = list(samples)
        self._t0 = self.samples[0].timestamp
        self.duration = self.samples[-1].timestamp - self._t0
        self._i = 0

    def command_at(self, elapsed: float) -> Optional[DriveCommand]:
        """None, wenn die Aufnahme zu Ende ist."""
        if elapsed > self.duration:
            return None
        while self._i + 1 < len(self.samples) and self.samples[self._i + 1].timestamp - self._t0 <= elapsed:
            self._i += 1
        s = self.samples[self._i]
        forward = s.direction.get("forward", 1.0) > 0
        tank = bool(s.buttons.get("A", False))
        return compute_drive_command(
            trigger_to_duty_cycle(s.left_trigger), trigger_to_duty_cycle(s.right_trigger), tank, forward
        )
