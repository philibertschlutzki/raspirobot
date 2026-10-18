"""Simulierte Umgebung für die Positionsbestimmung: Raum mit Hindernissen, LIDAR per Raycast."""

import math
from dataclasses import dataclass

import numpy as np

from raspirobot import config
from raspirobot.localization.geometry import compose, relative, wrap_angle


def box(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def room(width=8.0, height=6.0, obstacles=True):
    """Raum mit Wänden von (-2, -3) bis (width-2, height-3), damit der Start nahe der Mitte liegt."""
    segs = box(-2.0, -3.0, width - 2.0, height - 3.0)
    if obstacles:
        segs += box(1.0, 0.5, 1.8, 1.3)        # Kiste
        segs += box(-0.5, -2.2, 0.3, -1.6)     # zweite Kiste
        segs += [((3.0, -3.0), (3.0, -1.2))]   # Trennwand
        segs += [((-2.0, 1.5), (-0.8, 1.5))]   # Wandstück
    return np.array(segs, dtype=float)  # (S, 2, 2)


def corridor(length=30.0, width=1.6):
    """Langer, merkmalsloser Flur entlang x."""
    return np.array(box(-2.0, -width / 2, length, width / 2), dtype=float)


def raycast(segments, origin, angles, max_range=12.0):
    """Distanz je Strahl (inf, wenn nichts getroffen)."""
    d = np.column_stack((np.cos(angles), np.sin(angles)))                  # (A, 2)
    p = segments[:, 0, :]                                                  # (S, 2)
    e = segments[:, 1, :] - p                                              # (S, 2)
    o = np.asarray(origin)
    denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]  # (A, S)
    diff = p[None, :, :] - o                                               # (1, S, 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (diff[..., 0] * e[None, :, 1] - diff[..., 1] * e[None, :, 0]) / denom
        u = (diff[..., 0] * d[:, None, 1] - diff[..., 1] * d[:, None, 0]) / denom
    hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf)
    dist = t.min(axis=1)
    return np.where(dist <= max_range, dist, np.inf)


def simulate_scan(segments, robot_pose, rng=None, noise_m=0.01, beams=360):
    """Scan im Format des Treibers ``(quality, angle_deg, distance_mm)`` für eine Roboterpose."""
    lidar = compose(robot_pose, (config.LIDAR_OFFSET_X, config.LIDAR_OFFSET_Y, config.LIDAR_ANGLE_OFFSET and 0.0))
    angle_deg = np.arange(beams) * (360.0 / beams)
    # Sensor zählt im Uhrzeigersinn; Weltwinkel gegen den Uhrzeigersinn
    world_angle = lidar[2] + config.LIDAR_ANGLE_SIGN * np.radians(angle_deg)
    dist = raycast(segments, (lidar[0], lidar[1]), world_angle)
    if rng is not None and noise_m:
        dist = dist + rng.normal(0.0, noise_m, size=dist.shape)
    out = []
    for a, d in zip(angle_deg, dist):
        if math.isfinite(d):
            out.append((15.0, float(a), float(d * 1000.0)))
    return out


@dataclass
class FakeOdom:
    x: float
    y: float
    theta: float
    epoch: int = 0


class Drive:
    """Wahre Pose plus fehlerbehaftete Odometrie."""

    def __init__(self, rng, scale_error=0.08, turn_error=0.06, noise=0.004, noise_theta=0.003):
        self.true = (0.0, 0.0, 0.0)
        self.odom = (0.0, 0.0, 0.0)
        self.rng = rng
        self.scale_error, self.turn_error = scale_error, turn_error
        self.noise, self.noise_theta = noise, noise_theta

    def step(self, v, omega, dt=0.1):
        """Fährt mit Geschwindigkeit v (m/s) und Drehrate omega (rad/s)."""
        delta = (v * dt * math.cos(omega * dt / 2), v * dt * math.sin(omega * dt / 2), omega * dt)
        self.true = compose(self.true, delta)
        noisy = (
            delta[0] * (1 + self.scale_error) + self.rng.normal(0, self.noise),
            delta[1] + self.rng.normal(0, self.noise),
            delta[2] * (1 + self.turn_error) + self.rng.normal(0, self.noise_theta),
        )
        self.odom = compose(self.odom, noisy)

    def odom_obj(self, epoch=0):
        return FakeOdom(self.odom[0], self.odom[1], self.odom[2], epoch)


def pose_error(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(float(wrap_angle(a[2] - b[2])))
