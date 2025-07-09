import time from threading import Thread

class PatrolMode: def init(self, sensors, motors): self.sensors = sensors self.motors = motors

def execute(self):
    data = self.sensors.data
    # Hindernisse <30 cm: ausweichen
    if min(data['front'], data['left'], data['right']) < 30:
        self.motors.set_speed(-50, 50)
    else:
        self.motors.set_speed(50, 50)
class FollowMode:
def init(self, sensors, motors): self.sensors = sensors self.motors = motors

def execute(self):
    if self.sensors.data['person']:
        self.motors.set_speed(50, 50)
    else:
        self.motors.set_speed(0, 0)
class ModeController(Thread):
def init(self, sensor_svc, motor_svc, websocket=None): super().init(daemon=True) self.sensor_svc = sensor_svc self.motor_svc = motor_svc self.modes = { 'patrol': PatrolMode(sensor_svc, motor_svc), 'follow': FollowMode(sensor_svc, motor_svc) } self.current = 'patrol' self.websocket = websocket

def run(self):
    while True:
        if self.websocket:
            self.current = self.websocket.get_current_mode()
        self.modes[self.current].execute()
        time.sleep(0.1)
```