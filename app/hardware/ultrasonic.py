import time

class UltrasonicSensor: def init(self, gpio, pins): self.gpio = gpio self.pins = pins

def read_distance(self, position):
    trig = self.pins[position]['trigger']
    echo = self.pins[position]['echo']
    # Trigger-Impuls
    self.gpio.write_pin(trig, True)
    time.sleep(0.00001)
    self.gpio.write_pin(trig, False)
    # Echo messen
    start = time.time()
    while not self.gpio.read_pin(echo):
        start = time.time()
    while self.gpio.read_pin(echo):
        end = time.time()
    duration = end - start
    # Entfernung berechnen (m/s = 34300 cm/s)
    return (duration * 34300) / 2
```
