class MicrowaveSensor:
def __init__(self, gpio, pins):
self.gpio = gpio
self.pin = pins['data']

    def detect_motion(self):
        return bool(self.gpio.read_pin(self.pin))
    ```

### app/services/sensor_service.py

Thread für kontinuierliche Sensorerfassung:

import time from threading import Thread, Lock

class SensorService(Thread): def init(self, ultrasonic, microwave): super().init(daemon=True) self.ultrasonic = ultrasonic self.microwave = microwave self.data_lock = Lock() self.data = {}

def run(self):
    while True:
        readings = {
            pos: self.ultrasonic.read_distance(pos)
            for pos in ['front','right','back','left']
        }
        person = self.microwave.detect_motion()
        with self.data_lock:
            self.data = {**readings, 'person': person}
        time.sleep(0.05)  # 20 Hz
```