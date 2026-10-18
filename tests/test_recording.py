import gzip
import json
import time

import pytest

from raspirobot import config
from raspirobot.hardware.status_led import StatusIndicator
from raspirobot.hardware.xbox_controller import ControllerInput
from raspirobot.recorder import Recorder
from raspirobot.recording.storage import PathRecordingStorageManager


def make_recorder(tmp_path):
    r = Recorder(StatusIndicator(), reset_pose=lambda: None, lidar_connected=lambda: True)
    r.setup(str(tmp_path))
    return r


def test_recording_roundtrip(tmp_path, monkeypatch):
    r = make_recorder(tmp_path)
    assert r.start()
    r.offer_lidar_scan(time.time(), [(15, 10.0, 1200.0), (0, 20.0, 800.0)], 7)
    r.add_sample(ControllerInput(trigger_left=0.5, a_pressed=1, is_moving_forward=False))
    r.start_time -= config.MIN_RECORDING_DURATION_SEC
    assert r.stop()

    files = list(tmp_path.glob("*.json.gz"))
    assert len(files) == 1
    raw = json.loads(gzip.decompress(files[0].read_bytes()))
    sample = raw["controller_samples"][0]
    assert sample["buttons"] == {"A": 1, "B": True, "X": False, "Y": False}
    assert sample["direction"] == {"forward": -1.0, "tank_turn": 1.0}
    assert raw["lidar_frames"][0]["frame_id"] == 0
    assert raw["lidar_frames"][0]["quality_metrics"]["valid_points"] == 2

    loaded = PathRecordingStorageManager(base_directory=str(tmp_path)).load_json(files[0])
    assert len(loaded.controller_samples) == 1


def test_short_recording_is_discarded(tmp_path):
    r = make_recorder(tmp_path)
    r.start()
    assert not r.stop()
    assert not list(tmp_path.glob("*.json.gz"))


def test_scans_ignored_when_not_recording(tmp_path):
    r = make_recorder(tmp_path)
    r.offer_lidar_scan(time.time(), [(15, 10.0, 1200.0)], 0)
    assert r.lidar_frame_buffer.empty()


def test_checksum_mismatch_raises(tmp_path):
    r = make_recorder(tmp_path)
    r.start()
    r.add_sample(ControllerInput())
    r.start_time -= config.MIN_RECORDING_DURATION_SEC
    r.stop()
    path = next(tmp_path.glob("*.json.gz"))
    raw = json.loads(gzip.decompress(path.read_bytes()))
    raw["controller_samples"][0]["left_trigger"] = 0.123
    path.write_bytes(gzip.compress(json.dumps(raw, indent=2).encode()))
    with pytest.raises(ValueError):
        PathRecordingStorageManager(base_directory=str(tmp_path)).load_json(path)
