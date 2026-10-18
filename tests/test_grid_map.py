import numpy as np

from raspirobot.localization.grid_map import OCCUPIED_THRESHOLD, OccupancyMap


def wall_endpoints(x=2.0, n=41):
    ys = np.linspace(-1.0, 1.0, n)
    return np.column_stack((np.full(n, x), ys))


def test_integrate_marks_hits_occupied_and_ray_free():
    grid = OccupancyMap(size_m=10.0, resolution_m=0.05)
    grid.integrate((0.0, 0.0), wall_endpoints())

    def cell(x, y):
        g = np.floor(grid.world_to_grid(np.array([[x, y]]))).astype(int)[0]
        return grid.log_odds[g[1], g[0]]

    assert cell(2.0, 0.0) > OCCUPIED_THRESHOLD
    assert cell(1.0, 0.0) < 0
    assert cell(-1.0, 0.0) == 0          # hinter dem Sensor unbekannt
    assert cell(3.0, 0.0) == 0           # hinter der Wand unbekannt


def test_field_peaks_at_wall_and_decays():
    grid = OccupancyMap(size_m=10.0, resolution_m=0.05)
    grid.integrate((0.0, 0.0), wall_endpoints())
    on = grid.sample(np.array([[2.025, 0.0]]))[0]
    near = grid.sample(np.array([[2.15, 0.0]]))[0]
    far = grid.sample(np.array([[3.0, 0.0]]))[0]
    assert on > 0.9 > near > far
    assert far == 0.0


def test_sample_outside_map_is_zero_and_shape_preserved():
    grid = OccupancyMap(size_m=4.0, resolution_m=0.05)
    pts = np.array([[[100.0, 0.0], [-100.0, 3.0]], [[0.0, 1e6], [0.0, -1e6]]])
    out = grid.sample(pts)
    assert out.shape == (2, 2) and not out.any()


def test_points_outside_map_are_ignored():
    grid = OccupancyMap(size_m=4.0, resolution_m=0.05)
    grid.integrate((0.0, 0.0), np.array([[50.0, 0.0], [0.0, -50.0]]))
    assert grid.occupied_count == 0


def test_cell_changes_once_per_scan():
    grid = OccupancyMap(size_m=10.0, resolution_m=0.05)
    # Viele Strahlen auf dieselbe Zelle erhöhen sie nur einmal.
    grid.integrate((0.0, 0.0), np.tile([[2.0, 0.0]], (50, 1)))
    assert grid.log_odds.max() <= 1.0


def test_rebuild_removes_ghost_of_vanished_obstacle():
    grid = OccupancyMap(size_m=10.0, resolution_m=0.05)
    grid.integrate((0.0, 0.0), np.array([[2.0, 0.0]]))
    assert grid.sample(np.array([[2.025, 0.025]]))[0] > 0.5
    for _ in range(6):  # Strahlen gehen nun durch die Zelle hindurch bis zu einer weiter entfernten Wand
        grid.integrate((0.0, 0.0), np.array([[4.0, 0.0]]))
    grid.rebuild_field()
    assert grid.sample(np.array([[2.025, 0.025]]))[0] < 0.1


def test_save_writes_npz_and_pgm(tmp_path):
    grid = OccupancyMap(size_m=4.0, resolution_m=0.05)
    grid.integrate((0.0, 0.0), np.array([[1.0, 0.0]]))
    npz, pgm = grid.save(str(tmp_path / "sub" / "map"))
    assert npz.exists() and pgm.exists()
    loaded = np.load(npz)
    assert loaded["log_odds"].shape == (80, 80)
    header = pgm.read_bytes()[:16]
    assert header.startswith(b"P5\n80 80\n255\n")
