import time
class PatrolMode:
    def __init__(self, sensor_service, motor_service):
        self.sensors = sensor_service
        self.motors  = motor_service
    def execute(self):
        # Hinderniserkennung + Ausweichlogik
        data = self.sensors.data
        # … Implementierung …
        
class FollowMode:
    def __init__(self, sensor_service, motor_service):
        self.sensors = sensor_service
        self.motors  = motor_service
    def execute(self):
        # Personenverfolgung basierend auf microwave-Daten
        # … Implementierung …

class ModeController(Thread):
    def __init__(self, sensor_svc, motor_svc, websocket):
        super().__init__()
        self.modes = {'patrol': PatrolMode(sensor_svc, motor_svc),
                      'follow': FollowMode(sensor_svc, motor_svc)}
        self.current = 'patrol'
        self.ws = websocket
    def run(self):
        while True:
            mode = self.ws.get_current_mode()
            self.current = mode
            self.modes[mode].execute()
            time.sleep(0.1)