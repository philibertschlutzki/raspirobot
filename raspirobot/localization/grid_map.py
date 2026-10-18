"""Belegungskarte (Log-Odds) mit Likelihood-Feld für das Scan-Matching."""

import math
from pathlib import Path
from typing import Tuple

import numpy as np

from .. import config

L_FREE = -0.4
L_OCCUPIED = 0.85
L_MIN, L_MAX = -4.0, 4.0
OCCUPIED_THRESHOLD = 0.4


class OccupancyMap:
    """Quadratische Karte, Weltursprung in der Mitte.

    ``log_odds[iy, ix]``: > 0 belegt, < 0 frei, 0 unbekannt.
    ``field``: 0..1, je näher an einer belegten Zelle desto größer (Gauß).
    """

    def __init__(
        self,
        size_m: float = config.SLAM_MAP_SIZE_M,
        resolution_m: float = config.SLAM_RESOLUTION_M,
        sigma_m: float = config.SLAM_LIKELIHOOD_SIGMA_M,
    ):
        self.resolution = resolution_m
        self.size = int(round(size_m / resolution_m))
        self.half = self.size / 2.0
        self.log_odds = np.zeros((self.size, self.size), dtype=np.float32)
        # Das Feld hat einen Rand aus Nullen, damit ``sample`` ohne Grenzprüfung auskommt.
        self._padded = np.zeros((self.size + 2, self.size + 2), dtype=np.float32)
        self.field = self._padded[1:-1, 1:-1]

        radius = int(math.ceil(3 * sigma_m / resolution_m))
        offsets = np.arange(-radius, radius + 1)
        dy, dx = np.meshgrid(offsets, offsets, indexing="ij")
        dist2 = (dx * dx + dy * dy) * resolution_m ** 2
        weights = np.exp(-dist2 / (2 * sigma_m ** 2))
        keep = dist2 <= (3 * sigma_m) ** 2
        self._kernel = [(int(a), int(b), float(w)) for a, b, w in zip(dy[keep], dx[keep], weights[keep])]
        self.scans_integrated = 0

    # ------------------------------------------------------------ Koordinaten

    def world_to_grid(self, xy: np.ndarray) -> np.ndarray:
        """Kontinuierliche Gitterkoordinaten (Zellmitte bei ganzzahl + 0,5)."""
        return xy / self.resolution + self.half

    def _in_bounds(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        return (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)

    # ---------------------------------------------------------------- Abfrage

    def sample(self, xy: np.ndarray) -> np.ndarray:
        """Bilinear interpolierter Feldwert an beliebig geformten Weltpunkten (..., 2).

        Außerhalb der Karte ist der Wert 0.
        """
        g = self.world_to_grid(xy) - 0.5
        # Punkte außerhalb auf den Nullrand klemmen; +1 gleicht den Rand im Array aus.
        upper = self.size - 0.01
        gx = np.clip(g[..., 0], -1.0, upper)
        gy = np.clip(g[..., 1], -1.0, upper)
        x0 = np.floor(gx)
        y0 = np.floor(gy)
        fx = (gx - x0).astype(np.float32)
        fy = (gy - y0).astype(np.float32)
        stride = self.size + 2
        xi = x0.astype(np.int64) + 1
        yi = y0.astype(np.int64) + 1
        flat = self._padded.reshape(-1)
        idx = yi * stride + xi
        v00 = flat[idx]
        v10 = flat[idx + 1]
        v01 = flat[idx + stride]
        v11 = flat[idx + stride + 1]
        top = v00 + (v10 - v00) * fx
        bottom = v01 + (v11 - v01) * fx
        return top + (bottom - top) * fy

    @property
    def occupied_count(self) -> int:
        return int(np.count_nonzero(self.log_odds > OCCUPIED_THRESHOLD))

    # ------------------------------------------------------------- Aktualisieren

    def integrate(self, origin: Tuple[float, float], endpoints: np.ndarray) -> None:
        """Trägt einen Scan ein: Zellen entlang der Strahlen frei, Endpunkte belegt.

        Jede Zelle wird pro Scan höchstens einmal verändert. Endpunkte
        außerhalb der Karte werden verworfen.
        """
        if endpoints.shape[0] == 0:
            return
        origin_arr = np.asarray(origin, dtype=np.float64)
        delta = endpoints - origin_arr
        length = np.hypot(delta[:, 0], delta[:, 1])
        ok = length > 1e-6
        delta, length, endpoints = delta[ok], length[ok], endpoints[ok]
        if length.size == 0:
            return
        direction = delta / length[:, None]

        step = self.resolution * 0.5
        steps = np.arange(0.0, length.max(), step)
        t = steps[None, :]
        along = np.broadcast_to(t, (length.size, steps.size))
        mask = along < (length[:, None] - self.resolution * 0.75)
        ray_idx, step_idx = np.nonzero(mask)
        pts = origin_arr + direction[ray_idx] * steps[step_idx][:, None]
        free_cells = self._unique_cells(pts)

        hit_cells = self._unique_cells(endpoints)
        free_cells = np.setdiff1d(free_cells, hit_cells, assume_unique=True)

        flat = self.log_odds.reshape(-1)
        flat[free_cells] = np.maximum(flat[free_cells] + L_FREE, L_MIN)
        was_occupied = flat[hit_cells] > OCCUPIED_THRESHOLD
        flat[hit_cells] = np.minimum(flat[hit_cells] + L_OCCUPIED, L_MAX)
        newly = hit_cells[~was_occupied & (flat[hit_cells] > OCCUPIED_THRESHOLD)]
        self._stamp(newly)

        self.scans_integrated += 1
        if self.scans_integrated % config.SLAM_MAP_REBUILD_EVERY == 0:
            self.rebuild_field()

    def _unique_cells(self, xy: np.ndarray) -> np.ndarray:
        g = np.floor(self.world_to_grid(xy)).astype(np.int64)
        ix, iy = g[:, 0], g[:, 1]
        ok = self._in_bounds(ix, iy)
        return np.unique(iy[ok] * self.size + ix[ok])

    def _stamp(self, cells: np.ndarray) -> None:
        """Legt den Gauß-Kern um die Zellen (flache Indizes) ins Feld."""
        if cells.size == 0:
            return
        cy, cx = np.divmod(cells, self.size)
        flat = self._padded.reshape(-1)
        for dy, dx, w in self._kernel:
            ny, nx = cy + dy, cx + dx
            ok = self._in_bounds(nx, ny)
            idx = (ny[ok] + 1) * (self.size + 2) + nx[ok] + 1
            flat[idx] = np.maximum(flat[idx], w)

    def rebuild_field(self) -> None:
        """Berechnet das Feld aus allen belegten Zellen neu (entfernt Geister dynamischer Hindernisse)."""
        self._padded[:] = 0.0
        self._stamp(np.flatnonzero(self.log_odds.reshape(-1) > OCCUPIED_THRESHOLD))

    # ------------------------------------------------------------------ Export

    def to_image(self) -> np.ndarray:
        """uint8-Bild (Zeile 0 = größtes y): belegt 0, unbekannt 128, frei 255."""
        img = np.full(self.log_odds.shape, 128, dtype=np.uint8)
        img[self.log_odds < -OCCUPIED_THRESHOLD] = 255
        img[self.log_odds > OCCUPIED_THRESHOLD] = 0
        return img[::-1]

    def save(self, base_path: str) -> Tuple[Path, Path]:
        """Schreibt ``<base>.npz`` (Log-Odds) und ``<base>.pgm`` (Bild)."""
        base = Path(base_path)
        base.parent.mkdir(parents=True, exist_ok=True)
        npz = base.with_suffix(".npz")
        pgm = base.with_suffix(".pgm")
        np.savez_compressed(npz, log_odds=self.log_odds, resolution=self.resolution)
        img = self.to_image()
        with open(pgm, "wb") as f:
            f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii"))
            f.write(img.tobytes())
        return npz, pgm
