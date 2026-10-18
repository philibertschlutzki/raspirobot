import numpy as np

import sim
from raspirobot import config
from raspirobot.app import RobotApp


def test_scan_reaches_recorder_and_localizer_and_status_shows_slam(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    app = RobotApp()
    app.lidar.connected = True
    scan = sim.simulate_scan(sim.room(), (0.0, 0.0, 0.0), np.random.default_rng(0))

    app._on_lidar_scan(1.0, scan, 0)
    app.localizer.process(*app.localizer._pending)   # was der Worker-Thread täte

    app._print_status(0.0, 0.0)
    assert "SLAM[OK" in capsys.readouterr().out

    app._save_map()
    maps = list((tmp_path / config.RECORDING_BASE_DIR / "maps").glob("map_*.pgm"))
    assert len(maps) == 1


def test_no_map_written_without_scans(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    app = RobotApp()
    app._save_map()
    assert not (tmp_path / config.RECORDING_BASE_DIR / "maps").exists()
