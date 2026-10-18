import pytest

import fakes
from raspirobot import config
from raspirobot.hardware import lidar as lidar_mod
from raspirobot.hardware.lidar import C1Lidar, Lidar, RPLidarException


class FakeSerial:
    def __init__(self):
        self.dtr = None

    def setDTR(self, value):
        self.dtr = value


def test_c1_motor_control_sends_no_pwm_command():
    device = C1Lidar.__new__(C1Lidar)   # ohne Verbindungsaufbau
    device._serial = FakeSerial()
    device.start_motor()                # würde über _set_pwm eine AssertionError auslösen
    assert device._serial.dtr is False and device.motor_running is True
    device.stop_motor()
    assert device._serial.dtr is True and device.motor_running is False


class ScriptedDevice:
    """iter_scans: wirft die angegebenen Fehler der Reihe nach, liefert dann Scans."""

    def __init__(self, failures_before_scans, scans=3, fail_forever=False):
        self.failures = list(failures_before_scans)
        self.scans = scans
        self.fail_forever = fail_forever
        self.recoveries = 0

    def iter_scans(self, max_buf_meas, min_len):
        if self.fail_forever:
            raise RPLidarException("Descriptor length mismatch")
        if self.failures:
            raise self.failures.pop(0)
        for i in range(self.scans):
            yield [(15, float(i), 1000.0)]
        raise RPLidarException("Sensor getrennt")   # nach den Scans bricht der Strom ab

    def stop(self):
        self.recoveries += 1

    def clean_input(self):
        pass


def make_lidar(device):
    lidar = Lidar()
    lidar.device = device
    lidar.connected = True
    return lidar


def test_run_restarts_after_aborted_scan_start(monkeypatch):
    monkeypatch.setattr(lidar_mod.time, "sleep", lambda s: None)
    device = ScriptedDevice([RPLidarException("Descriptor length mismatch")] * 2)
    lidar = make_lidar(device)
    received = []
    runs = {"n": 0}

    def should_run():
        runs["n"] += 1
        return len(received) < 3

    lidar.run(should_run, lambda ts, scan, idx: received.append(idx))
    assert received == [0, 1, 2]
    assert device.recoveries >= 2
    assert lidar.connected is True


def test_run_gives_up_after_max_restarts(monkeypatch):
    monkeypatch.setattr(lidar_mod.time, "sleep", lambda s: None)
    device = ScriptedDevice([], fail_forever=True)
    lidar = make_lidar(device)
    lidar.run(lambda: True, lambda *a: None)
    assert lidar.connected is False
    assert device.recoveries == config.LIDAR_MAX_RESTARTS


def test_restart_counter_resets_after_successful_scans(monkeypatch):
    monkeypatch.setattr(lidar_mod.time, "sleep", lambda s: None)

    class Flaky(ScriptedDevice):
        calls = 0

        def iter_scans(self, max_buf_meas, min_len):
            Flaky.calls += 1
            if Flaky.calls > config.LIDAR_MAX_RESTARTS * 2:
                raise OSError("Ende des Tests")
            yield [(15, 0.0, 1000.0)]
            raise RPLidarException("kurzer Abbruch")

    device = Flaky([])
    lidar = make_lidar(device)
    count = {"n": 0}
    lidar.run(lambda: True, lambda *a: count.__setitem__("n", count["n"] + 1))
    # Mehr Abbrüche als LIDAR_MAX_RESTARTS insgesamt sind ok, solange zwischendurch Scans kamen.
    assert count["n"] > config.LIDAR_MAX_RESTARTS


def test_run_stops_on_shutdown_without_error():
    device = ScriptedDevice([], scans=5)
    lidar = make_lidar(device)
    seen = []
    lidar.run(lambda: len(seen) < 2, lambda ts, scan, idx: seen.append(idx))
    assert seen == [0, 1] and lidar.connected is True


def test_run_survives_failing_scan_callback():
    device = ScriptedDevice([], scans=3)
    lidar = make_lidar(device)
    seen = []

    def on_scan(ts, scan, idx):
        seen.append(idx)
        if len(seen) == 1:
            raise ValueError("Empfänger kaputt")

    lidar.run(lambda: len(seen) < 3, on_scan)
    assert len(seen) == 3          # der Thread läuft nach dem Fehler im Callback weiter
    assert lidar.latest_scan


def test_connect_without_device_reports_failure():
    lidar = Lidar()
    assert lidar.connect() is False
    assert lidar.connected is False
