<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" class="logo" width="120"/>

# Codestruktur zur Umsetzung der Softwarearchitektur

Die folgende Verzeichnis- und Modulstruktur bildet die beschriebene Schichtenarchitektur ab. Sie ermöglicht klare Verantwortlichkeiten, einfache Erweiterbarkeit und Testbarkeit.

```
robot_controller/
├── README.md
├── requirements.txt
├── config/
│   └── gpio_pins.yaml
├── app/                        
│   ├── __init__.py             
│   ├── main.py                 # Startpunkt: Initialisierung und Start aller Dienste
│   ├── services/
│   │   ├── __init__.py
│   │   ├── sensor_service.py   # Sensordaten erfassen (HC-SR04, RP03D)
│   │   ├── motor_service.py    # Motorsteuerung (ZX-11H)
│   │   └── mode_controller.py  # Modus-Logik (PatrolMode, FollowMode)
│   ├── hardware/
│   │   ├── __init__.py
│   │   ├── gpio_interface.py    # Abstraktion aller GPIO-Funktionen
│   │   ├── ultrasonic.py        # HC-SR04 Trigger/Echo-Logik
│   │   └── microwave.py         # RP03D Mikrowellen-Logik
│   └── web/
│       ├── __init__.py
│       ├── server.py           # Flask-App + WebSocket-Setup
│       ├── api.py              # REST/WS-Endpoints für GUI
│       └── static/
│           ├── css/
│           ├── js/
│           └── index.html      # Dashboard-Frontend
├── tests/
│   ├── __init__.py         
│   ├── test_sensor_service.py
│   ├── test_motor_service.py
│   └── test_mode_controller.py
└── systemd/
    └── robot.service           # Systemd-Unit für Autostart
```


## Modul- und Dateiübersicht

### config/gpio_pins.yaml

Konfiguriert die GPIO-Pins der Sensoren und Motorcontroller:

```yaml
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


### app/main.py

- Liest Konfiguration ein
- Initialisiert `GPIOInterface`
- Startet `SensorService`, `MotorService` und `ModeController` in eigenen Threads
- Startet Flask-Server


### app/services/sensor_service.py

```python
from threading import Thread, Lock
class SensorService(Thread):
    def __init__(self, ultrasonic, microwave):
        super().__init__()
        self.ultrasonic = ultrasonic
        self.microwave = microwave
        self.data_lock = Lock()
        self.data = {}
    def run(self):
        while True:
            d_front  = self.ultrasonic.read_distance('front')
            d_right  = self.ultrasonic.read_distance('right')
            d_back   = self.ultrasonic.read_distance('back')
            d_left   = self.ultrasonic.read_distance('left')
            person   = self.microwave.detect_motion()
            with self.data_lock:
                self.data = {'front': d_front, 'right': d_right,
                             'back': d_back,  'left': d_left,
                             'person': person}
            time.sleep(0.05)  # 20 Hz
```


### app/services/motor_service.py

```python
class MotorService:
    def __init__(self, gpio):
        self.gpio = gpio
    def set_speed(self, left_percent, right_percent):
        # Konvertiert Prozent in PWM-Werte und steuert Richtung
        self.gpio.set_pwm('left', left_percent)
        self.gpio.set_pwm('right', right_percent)
```


### app/services/mode_controller.py

```python
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
```


### app/hardware/gpio_interface.py

Enthält Low-Level GPIO-Abstraktion mit RPi.GPIO oder gpiozero.

### app/web/server.py \& api.py

- `server.py`: Initialisiert Flask, WebSocket (z. B. Flask-SocketIO)
- `api.py`: Endpoints zum Abruf aktueller Sensordaten und Moduswechsel

```python
from flask import Flask, jsonify
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/sensors')
def get_sensors():
    return jsonify(sensor_service.data)

@socketio.on('mode_change')
def on_mode_change(data):
    # data['mode'] enthält 'patrol' oder 'follow'
    socketio.emit('mode', data)
```


### tests/

Unit-Tests für alle Services mit pytest.

### systemd/robot.service

```ini
[Unit]
Description=Robot Controller Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 /path/to/robot_controller/app/main.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Mit dieser Struktur sind alle Komponenten klar getrennt, Multithreading gewährleistet Performance, und das Web-GUI kann über REST/WebSocket Echtzeitdaten darstellen und Modi umschalten.

