# Robot Controller

Dieses Repository enthält die vollständige Implementierung einer modularen Robotersteuerung auf Basis eines Raspberry Pi 5. Ziel ist, Sensordaten in Echtzeit in einem Web-GUI darzustellen, zwischen Patrouillen- und Personenverfolgungsmodus umzuschalten und alle Berechnungen direkt auf dem Pi durchzuführen.

---

## Inhaltsverzeichnis

1. [Installation](#installation)  
2. [Verzeichnisstruktur](#verzeichnisstruktur)  
3. [Konfiguration](#konfiguration)  
4. [Module & Dateien](#module--dateien)  
   1. [app/main.py](#appmainpy)  
   2. [config/gpio_pins.yaml](#configgpiopinsyaml)  
   3. [app/hardware/gpio_interface.py](#apphardwaregpio_interfacepy)  
   4. [app/hardware/ultrasonic.py](#apphardwareultrasonicpy)  
   5. [app/hardware/microwave.py](#apphardwaremicrowavepy)  
   6. [app/services/sensor_service.py](#appservicessensor_servicepy)  
   7. [app/services/motor_service.py](#appservicesmotor_servicepy)  
   8. [app/services/mode_controller.py](#appservicesmode_controllerpy)  
   9. [app/web/server.py](#appwebserverpy)  
   10. [app/web/api.py](#appwebapipy)  
5. [Frontend](#frontend)  
6. [Systemd-Service](#systemd-service)  
7. [Tests](#tests)  
8. [Requirements](#requirements)  

---

## Installation

1. Repository klonen  
```

git clone https://github.com/yourusername/robot_controller.git
cd robot_controller

```  
2. Virtuelle Umgebung erstellen und aktivieren  
```

python3 -m venv venv
source venv/bin/activate

```  
3. Abhängigkeiten installieren  
```

pip install -r requirements.txt

```  

---

## Verzeichnisstruktur

```

robot_controller/
├── README.md
├── requirements.txt
├── config/
│   └── gpio_pins.yaml
├── app/
│   ├── __init__.py
│   ├── main.py
│   ├── hardware/
│   │   ├── __init__.py
│   │   ├── gpio_interface.py
│   │   ├── ultrasonic.py
│   │   └── microwave.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── sensor_service.py
│   │   ├── motor_service.py
│   │   └── mode_controller.py
│   └── web/
│       ├── __init__.py
│       ├── server.py
│       └── api.py
├── tests/
│   ├── __init__.py
│   ├── test_sensor_service.py
│   ├── test_motor_service.py
│   └── test_mode_controller.py
└── systemd/
└── robot.service

```

---

## Konfiguration

### config/gpio_pins.yaml

Konfigurationsdatei für alle GPIO-Pin-Nummern.

```

ultrasonic:
front: {trigger: 23, echo: 24}
right: {trigger: 25, echo: 8}
back:  {trigger: 7,  echo: 12}
left:  {trigger: 16, echo: 20}
microwave:
data: 21
motor:
left:  {pwm: 18, dir: 27, en: 22}
right: {pwm: 13, dir: 19, en: 26}

```

---

## Module & Dateien

### app/main.py

Startpunkt der Anwendung:

```

import yaml
import threading
from app.hardware.gpio_interface import GPIOInterface
from app.hardware.ultrasonic import UltrasonicSensor
from app.hardware.microwave import MicrowaveSensor
from app.services.sensor_service import SensorService
from app.services.motor_service import MotorService
from app.services.mode_controller import ModeController
from app.web.server import run_server

def main():
\# 1. GPIO-Pins laden
with open("config/gpio_pins.yaml") as f:
pins = yaml.safe_load(f)

    # 2. Hardware-Interfaces initialisieren
    gpio = GPIOInterface(pins)
    ultrasonic = UltrasonicSensor(gpio, pins['ultrasonic'])
    microwave = MicrowaveSensor(gpio, pins['microwave'])
    motor_svc = MotorService(gpio)
    
    # 3. Services starten
    sensor_svc = SensorService(ultrasonic, microwave)
    sensor_svc.start()
    mode_ctrl = ModeController(sensor_svc, motor_svc)
    mode_ctrl.start()
    
    # 4. Webserver starten (blockierend)
    run_server(sensor_svc, mode_ctrl)
    if __name__ == "__main__":
main()

```

### app/hardware/gpio_interface.py

Kapselt Low-Level GPIO-Funktionen:

```

import RPi.GPIO as GPIO

class GPIOInterface:
def __init__(self, config):
GPIO.setmode(GPIO.BCM)
self.config = config
self._setup_pins()

    def _setup_pins(self):
        # Ultrasonic- und Motor-Pins initialisieren
        for name, cfg in self.config['ultrasonic'].items():
            GPIO.setup(cfg['trigger'], GPIO.OUT)
            GPIO.setup(cfg['echo'], GPIO.IN)
        GPIO.setup(self.config['microwave']['data'], GPIO.IN)
        for side in ['left','right']:
            c = self.config['motor'][side]
            GPIO.setup(c['pwm'], GPIO.OUT)
            GPIO.setup(c['dir'], GPIO.OUT)
            GPIO.setup(c['en'], GPIO.OUT)
            setattr(self, f"{side}_pwm", GPIO.PWM(c['pwm'], 1000))
            getattr(self, f"{side}_pwm").start(0)
    
    def read_pin(self, pin):
        return GPIO.input(pin)
    
    def write_pin(self, pin, value):
        GPIO.output(pin, value)
    
    def set_pwm(self, side, percent):
        pwm = getattr(self, f"{side}_pwm")
        pwm.ChangeDutyCycle(abs(percent))
        self.write_pin(self.config['motor'][side]['dir'], percent >= 0)
        self.write_pin(self.config['motor'][side]['en'], True)
    ```

### app/hardware/ultrasonic.py

Ultraschallsensor HC-SR04-Logik:

```

import time

class UltrasonicSensor:
def __init__(self, gpio, pins):
self.gpio = gpio
self.pins = pins

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

### app/hardware/microwave.py

Mikrowellensensor RP03D-Logik:

```

class MicrowaveSensor:
def __init__(self, gpio, pins):
self.gpio = gpio
self.pin = pins['data']

    def detect_motion(self):
        return bool(self.gpio.read_pin(self.pin))
    ```

### app/services/sensor_service.py

Thread für kontinuierliche Sensorerfassung:

```

import time
from threading import Thread, Lock

class SensorService(Thread):
def __init__(self, ultrasonic, microwave):
super().__init__(daemon=True)
self.ultrasonic = ultrasonic
self.microwave = microwave
self.data_lock = Lock()
self.data = {}

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

### app/services/motor_service.py

Steuerung der Motoren via PWM:

```

class MotorService:
def __init__(self, gpio):
self.gpio = gpio

    def set_speed(self, left_percent, right_percent):
        self.gpio.set_pwm('left', left_percent)
        self.gpio.set_pwm('right', right_percent)
    ```

### app/services/mode_controller.py

Betriebsmodi-Logik und Thread:

```

import time
from threading import Thread

class PatrolMode:
def __init__(self, sensors, motors):
self.sensors = sensors
self.motors = motors

    def execute(self):
        data = self.sensors.data
        # Hindernisse <30 cm: ausweichen
        if min(data['front'], data['left'], data['right']) < 30:
            self.motors.set_speed(-50, 50)
        else:
            self.motors.set_speed(50, 50)
    class FollowMode:
def __init__(self, sensors, motors):
self.sensors = sensors
self.motors = motors

    def execute(self):
        if self.sensors.data['person']:
            self.motors.set_speed(50, 50)
        else:
            self.motors.set_speed(0, 0)
    class ModeController(Thread):
def __init__(self, sensor_svc, motor_svc, websocket=None):
super().__init__(daemon=True)
self.sensor_svc = sensor_svc
self.motor_svc = motor_svc
self.modes = {
'patrol': PatrolMode(sensor_svc, motor_svc),
'follow': FollowMode(sensor_svc, motor_svc)
}
self.current = 'patrol'
self.websocket = websocket

    def run(self):
        while True:
            if self.websocket:
                self.current = self.websocket.get_current_mode()
            self.modes[self.current].execute()
            time.sleep(0.1)
    ```

### app/web/server.py

Flask-Server mit SocketIO-Integration:

```

from flask import Flask
from flask_socketio import SocketIO
from app.web.api import setup_api

def run_server(sensor_svc, mode_ctrl):
app = Flask(__name__, static_folder="../static", template_folder="../static")
socketio = SocketIO(app, cors_allowed_origins="*")
setup_api(app, socketio, sensor_svc, mode_ctrl)
socketio.run(app, host="0.0.0.0", port=5000)

```

### app/web/api.py

REST- und WebSocket-Endpunkte:

```

from flask import jsonify, request, send_from_directory

def setup_api(app, socketio, sensor_svc, mode_ctrl):
@app.route('/sensors')
def get_sensors():
with sensor_svc.data_lock:
return jsonify(sensor_svc.data)

    @app.route('/')
    def index():
        return send_from_directory(app.static_folder, 'index.html')
    
    @socketio.on('mode_change')
    def on_mode_change(msg):
        mode = msg.get('mode')
        if mode in mode_ctrl.modes:
            mode_ctrl.current = mode
            socketio.emit('mode', {'mode': mode})
    ```

---

## Frontend

Legen Sie unter `app/static/` folgende Dateien an:

- **index.html**: Dashboard mit Canvas-Visualisierung und Modus-Schaltern  
- **js/app.js**: WebSocket-Verbindung, Fetch-Aufrufe an `/sensors`, UI-Updates  
- **css/style.css**: Layout und Responsive-Design  

(_Beispiel: index.html lädt Socket.IO, verbindet, zeigt Sensordaten und Buttons zum Wechseln zwischen Patrol und Follow._)

---

## Systemd-Service

Datei: `systemd/robot.service`

```

[Unit]
Description=Robot Controller Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 /path/to/robot_controller/app/main.py
Restart=always
User=pi

[Install]
WantedBy=multi-user.target

```

Aktivieren:

```

sudo cp systemd/robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot
sudo systemctl start robot

```

---

## Tests

Unit-Tests mit **pytest** in `tests/`:

- `test_sensor_service.py`  
- `test_motor_service.py`  
- `test_mode_controller.py`  

Ausführen: `pytest`

---

## Requirements

Datei: `requirements.txt`

```

RPi.GPIO>=0.7.0
flask>=2.0.0
flask-socketio>=5.0.0
pytest>=6.0.0
pyyaml>=6.0

```

---

Mit dieser Dokumentation und den bereitgestellten Dateien lässt sich die komplette Robotersteuerungssuite auf einem Raspberry Pi 5 einrichten und ausführen.```

