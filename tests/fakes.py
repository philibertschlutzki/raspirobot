"""Ersatzmodule für RPi.GPIO, rpi_hardware_pwm, pygame und rplidar, damit die
Steuerlogik ohne Raspberry Pi getestet werden kann."""

import sys
import types


class FakeGPIO(types.ModuleType):
    BCM = "BCM"
    OUT = "OUT"
    IN = "IN"
    HIGH = 1
    LOW = 0

    def __init__(self):
        super().__init__("RPi.GPIO")
        self.outputs = []
        self.levels = {}
        self.input_values = {}

    def setmode(self, mode):
        pass

    def setwarnings(self, flag):
        pass

    def setup(self, pin, mode):
        pass

    def output(self, pin, value):
        self.outputs.append((pin, value))
        self.levels[pin] = value

    def input(self, pin):
        return self.input_values.get(pin, 0)

    def cleanup(self):
        pass


class FakeHardwarePWM:
    instances = []

    def __init__(self, pwm_channel, hz, chip=0):
        self.channel = pwm_channel
        self.duty = None
        self.history = []
        FakeHardwarePWM.instances.append(self)

    def start(self, duty):
        self.change_duty_cycle(duty)

    def change_duty_cycle(self, duty):
        if not 0 <= duty <= 100:
            raise ValueError(duty)
        self.duty = duty
        self.history.append(duty)

    def stop(self):
        self.duty = 0


class FakeJoystick:
    def __init__(self, idx=0):
        self.axes = {}
        self.buttons = {}
        self._init = False

    def init(self):
        self._init = True

    def get_init(self):
        return self._init

    def get_name(self):
        return "Fake Controller"

    def get_axis(self, i):
        return self.axes.get(i, -1.0)

    def get_button(self, i):
        return self.buttons.get(i, 0)

    def quit(self):
        self._init = False


def install():
    gpio = FakeGPIO()
    rpi = types.ModuleType("RPi")
    rpi.GPIO = gpio
    sys.modules["RPi"] = rpi
    sys.modules["RPi.GPIO"] = gpio

    pwm_mod = types.ModuleType("rpi_hardware_pwm")
    pwm_mod.HardwarePWM = FakeHardwarePWM
    sys.modules["rpi_hardware_pwm"] = pwm_mod

    pg = types.ModuleType("pygame")
    pg.init = lambda: None
    pg.quit = lambda: None
    pg.event = types.SimpleNamespace(pump=lambda: None)
    pg.joystick = types.SimpleNamespace(
        init=lambda: None, get_count=lambda: 1, Joystick=FakeJoystick
    )
    sys.modules["pygame"] = pg

    rpl = types.ModuleType("rplidar")

    class RPLidarException(Exception):
        pass

    class RPLidar:
        def __init__(self, *a, **k):
            raise RPLidarException("kein Gerät im Test")

        def _set_pwm(self, pwm):
            raise AssertionError("SET_PWM darf beim C1 nicht gesendet werden")

    rpl.RPLidar = RPLidar
    rpl.RPLidarException = RPLidarException
    sys.modules["rplidar"] = rpl
    return gpio
