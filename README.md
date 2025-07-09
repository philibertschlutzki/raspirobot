
## Softwarearchitektur für Roboter mit Raspberry Pi 5

### Systemübersicht

Das entwickelte Robotersystem basiert auf einer **modularen Schichtenarchitektur**, die eine klare Trennung zwischen Hardware-Abstraktionsschicht, Service-Schicht, Anwendungslogik und Web-Interface gewährleistet. Die Architektur ist darauf ausgelegt, alle Anforderungen zu erfüllen: Sensordaten in einem Web-GUI anzuzeigen, zwischen Personenverfolgung und Patroullieren umzuschalten und alle Berechnungen auf dem Raspberry Pi durchzuführen.

### Hardware-Komponenten und GPIO-Zuordnung

**Raspberry Pi 5 8GB** fungiert als zentrale Steuereinheit mit **40 GPIO-Pins** für die Sensoranbindung. Die **4x HC-SR04 Ultraschallsensoren** werden strategisch positioniert: Front (GPIO 23/24), Rechts (GPIO 25/8), Hinten (GPIO 7/12) und Links (GPIO 16/20). Die **2x ZX-11H Motor Controller** nutzen PWM-fähige Pins: Links (PWM GPIO 18, Direction GPIO 27, Enable GPIO 22) und Rechts (PWM GPIO 13, Direction GPIO 19, Enable GPIO 26). Der **RP03D Mikrowellensensor** ist an GPIO 21 angeschlossen.

### Systemarchitektur

```
┌─────────────────────────────────────┐
│         Web-GUI Layer               │
│    (Flask Web Interface)            │
├─────────────────────────────────────┤
│       Application Layer             │
│  (Betriebsmodus-Controller)         │
├─────────────────────────────────────┤
│        Service Layer                │
│   (Sensor/Motor Services)           │
├─────────────────────────────────────┤
│       Hardware Layer                │
│  (GPIO/Physical Components)         │
└─────────────────────────────────────┘
```


### Implementierungsdetails

#### Hardware-Abstraktionsschicht

Die **ZX-11H Motor Controller** werden über PWM-Signale mit 1kHz Frequenz gesteuert, wobei Geschwindigkeit (-100 bis +100), Richtung und Enable-Status separat kontrolliert werden. Die **HC-SR04 Sensoren** verwenden das bewährte Trigger-Echo-Prinzip: 10µs Trigger-Impuls, Echo-Zeit-Messung und Distanzberechnung über Schallgeschwindigkeit (343 m/s). Der **RP03D Mikrowellensensor** arbeitet mit 24GHz Frequenz für präzise Bewegungserkennung.

#### Service-Schicht mit Multithreading

Das **SensorService** implementiert kontinuierliche Sensorüberwachung mit 20Hz Update-Rate in einem separaten Thread[^15][^16]. Thread-sichere Datenverarbeitung wird durch Locks gewährleistet[^15][^17]. Das **MotorService** steuert die differentielle Lenkung mit PWM-basierter Geschwindigkeitskontrolle.

#### Betriebsmodi

**PatrolMode** implementiert autonome Patrouillierung mit Hinderniserkennung: Bei Hindernissen unter 30cm wird automatisch eine Ausweichstrategie aktiviert. **FollowMode** realisiert Personenverfolgung basierend auf Mikrowellensensor-Daten mit Suchverhalten bei Zielverlust.

### Web-Interface

Das **Flask-basierte Web-Interface** bietet eine vollständige Roboter-Steuerung. **WebSocket-Integration** ermöglicht Echtzeitdaten-Updates ohne Seitenneuladung. Das **responsive Dashboard** zeigt Live-Sensordaten, eine interaktive Umgebungskarte, Modus-Umschaltung und manuelle Steuerung.

#### Dashboard-Features

- **Echtzeit-Sensoranzeige**: Alle 4 Ultraschallsensoren und Mikrowellensensor-Status
- **Interaktive Umgebungskarte**: Canvas-basierte Visualisierung der Roboter-Umgebung
- **Modus-Umschaltung**: Ein-Klick-Wechsel zwischen Patrol und Follow-Modus
- **Manuelle Steuerung**: Richtungstasten für direkte Roboter-Kontrolle
- **Live-System-Logs**: Kontinuierliche Anzeige von Systemereignissen


### Threading-Konzept

Das System verwendet **drei parallele Threads**:

- **Sensor-Thread**: Kontinuierliche Datenerfassung (10-20 Hz)
- **Mode-Thread**: Verhaltensentscheidungen (5-10 Hz)
- **Web-Thread**: GUI-Updates (1-2 Hz)


### Deployment und Konfiguration

Das System wird als **Systemd-Service** konfiguriert für automatischen Start beim Booten. **Graceful Shutdown** sorgt für saubere Ressourcenfreigabe. **Logging-Framework** dokumentiert alle Systemereignisse und Fehler.

### Sicherheit und Erweiterbarkeit

**Sicherheitsmaßnahmen** umfassen Notaus-Funktionen, Sensor-Überwachung und Kollisionsvermeidung. Die **modulare Architektur** ermöglicht einfache Erweiterungen um neue Sensoren, KI-Integration oder Multi-Robot-Koordination.

Diese Architektur stellt eine vollständige, produktionsreife Lösung dar, die alle Anforderungen erfüllt und gleichzeitig eine solide Basis für zukünftige Erweiterungen bietet.

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


---

## GPIO-Pin-Belegung

| Komponente       | Pin         | Funktion           |
|------------------|-------------|--------------------|
| Motor links PWM  | 18          | PWM für linken Motor  |
| Motor links Dir. | 27          | Richtung links       |
| Motor links En.  | 22          | Enable links         |
| Motor rechts PWM | 13          | PWM für rechten Motor |
| Motor rechts Dir.| 19          | Richtung rechts      |
| Motor rechts En. | 26          | Enable rechts        |
| US-Front Trigger | 23          | Trigger Front        |
| US-Front Echo    | 24          | Echo Front           |
| US-Rechts Trigger| 25          | Trigger Rechts       |
| US-Rechts Echo   | 8           | Echo Rechts          |
| US-Back Trigger  | 7           | Trigger Back         |
| US-Back Echo     | 12          | Echo Back            |
| US-Links Trigger | 16          | Trigger Links        |
| US-Links Echo    | 20          | Echo Links           |
| Mikrowelle       | 21          | Output               |

---



