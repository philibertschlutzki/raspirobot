
## Softwarearchitektur für Roboter mit Raspberry Pi 5

### Systemübersicht

Das entwickelte Robotersystem basiert auf einer **modularen Schichtenarchitektur**, die eine klare Trennung zwischen Hardware-Abstraktionsschicht, Service-Schicht, Anwendungslogik und Web-Interface gewährleistet[^1][^2][^3]. Die Architektur ist darauf ausgelegt, alle Anforderungen zu erfüllen: Sensordaten in einem Web-GUI anzuzeigen, zwischen Personenverfolgung und Patroullieren umzuschalten und alle Berechnungen auf dem Raspberry Pi durchzuführen.

### Hardware-Komponenten und GPIO-Zuordnung

**Raspberry Pi 5 8GB** fungiert als zentrale Steuereinheit mit **40 GPIO-Pins** für die Sensoranbindung[^1][^2][^3]. Die **4x HC-SR04 Ultraschallsensoren** werden strategisch positioniert: Front (GPIO 23/24), Rechts (GPIO 25/8), Hinten (GPIO 7/12) und Links (GPIO 16/20)[^4][^5][^6]. Die **2x ZX-11H Motor Controller** nutzen PWM-fähige Pins: Links (PWM GPIO 18, Direction GPIO 27, Enable GPIO 22) und Rechts (PWM GPIO 13, Direction GPIO 19, Enable GPIO 26)[^7][^8][^9]. Der **RP03D Mikrowellensensor** ist an GPIO 21 angeschlossen[^10][^11][^12].

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

Die **ZX-11H Motor Controller** werden über PWM-Signale mit 1kHz Frequenz gesteuert, wobei Geschwindigkeit (-100 bis +100), Richtung und Enable-Status separat kontrolliert werden[^7][^9]. Die **HC-SR04 Sensoren** verwenden das bewährte Trigger-Echo-Prinzip: 10µs Trigger-Impuls, Echo-Zeit-Messung und Distanzberechnung über Schallgeschwindigkeit (343 m/s)[^4][^5][^13]. Der **RP03D Mikrowellensensor** arbeitet mit 24GHz Frequenz für präzise Bewegungserkennung[^10][^14][^12].

#### Service-Schicht mit Multithreading

Das **SensorService** implementiert kontinuierliche Sensorüberwachung mit 20Hz Update-Rate in einem separaten Thread[^15][^16]. Thread-sichere Datenverarbeitung wird durch Locks gewährleistet[^15][^17]. Das **MotorService** steuert die differentielle Lenkung mit PWM-basierter Geschwindigkeitskontrolle[^16][^18][^19].

#### Betriebsmodi

**PatrolMode** implementiert autonome Patrouillierung mit Hinderniserkennung: Bei Hindernissen unter 30cm wird automatisch eine Ausweichstrategie aktiviert. **FollowMode** realisiert Personenverfolgung basierend auf Mikrowellensensor-Daten mit Suchverhalten bei Zielverlust[^10][^11].

### Web-Interface

Das **Flask-basierte Web-Interface** bietet eine vollständige Roboter-Steuerung[^20][^21][^22]. **WebSocket-Integration** ermöglicht Echtzeitdaten-Updates ohne Seitenneuladung[^20][^23]. Das **responsive Dashboard** zeigt Live-Sensordaten, eine interaktive Umgebungskarte, Modus-Umschaltung und manuelle Steuerung[^20][^24].

#### Dashboard-Features

- **Echtzeit-Sensoranzeige**: Alle 4 Ultraschallsensoren und Mikrowellensensor-Status
- **Interaktive Umgebungskarte**: Canvas-basierte Visualisierung der Roboter-Umgebung
- **Modus-Umschaltung**: Ein-Klick-Wechsel zwischen Patrol und Follow-Modus
- **Manuelle Steuerung**: Richtungstasten für direkte Roboter-Kontrolle
- **Live-System-Logs**: Kontinuierliche Anzeige von Systemereignissen


### Threading-Konzept

Das System verwendet **drei parallele Threads**[^15][^25][^17]:

- **Sensor-Thread**: Kontinuierliche Datenerfassung (10-20 Hz)
- **Mode-Thread**: Verhaltensentscheidungen (5-10 Hz)
- **Web-Thread**: GUI-Updates (1-2 Hz)


### Deployment und Konfiguration

Das System wird als **Systemd-Service** konfiguriert für automatischen Start beim Booten[^26]. **Graceful Shutdown** sorgt für saubere Ressourcenfreigabe[^17]. **Logging-Framework** dokumentiert alle Systemereignisse und Fehler.

### Sicherheit und Erweiterbarkeit

**Sicherheitsmaßnahmen** umfassen Notaus-Funktionen, Sensor-Überwachung und Kollisionsvermeidung. Die **modulare Architektur** ermöglicht einfache Erweiterungen um neue Sensoren, KI-Integration oder Multi-Robot-Koordination.

Diese Architektur stellt eine vollständige, produktionsreife Lösung dar, die alle Anforderungen erfüllt und gleichzeitig eine solide Basis für zukünftige Erweiterungen bietet.

<div style="text-align: center">⁂</div>

[^1]: https://www.richardelectronics.com/blog/projects/raspberry pi/raspberry-pi-5-pinout-specs-datasheet-projects

[^2]: https://www.kiwi-electronics.com/en/raspberry-pi-5-8gb-11580

[^3]: https://www.raspberrypi.com/products/raspberry-pi-5/

[^4]: https://components101.com/sensors/ultrasonic-sensor-working-pinout-datasheet

[^5]: https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/

[^6]: https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/

[^7]: https://docs.cirkitdesigner.com/component/4cf35777-c70f-44c9-9828-838f50915cea/zs-x11h-v2-350w-motor-controller

[^8]: https://www.youtube.com/watch?v=TaCukBg4Hj8

[^9]: https://electropeak.com/zs-x11h-dc-brushless-motor-drive-module-400w-9-60v

[^10]: https://core-electronics.com.au/guides/using-mmwave-radar-to-detect-and-track-humans-raspberry-pi-guide/

[^11]: https://techatronic.com/interfacing-rcwl-0516-microwave-sensor-with-arduino/

[^12]: https://thepihut.com/products/microwave-motion-sensor-rcwl-0516

[^13]: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf

[^14]: https://www.youtube.com/watch?v=r4xcSuhZol0

[^15]: https://makergram.com/community/topic/29/multi-thread-handling-for-normal-processes-using-python

[^16]: https://xbonfiremonitor.com/raspberry-pi-gpio-pins/

[^17]: https://www.youtube.com/watch?v=1q0EaTkztIs

[^18]: https://www.instructables.com/Playing-With-Electronics-Raspberry-GPIO-Zero-Libra/

[^19]: https://projects.raspberrypi.org/en/projects/physical-computing/14

[^20]: https://www.youtube.com/watch?v=lCpynCSu1lc

[^21]: https://www.reddit.com/r/raspberry_pi/comments/638yq2/best_way_to_control_gpio_throug_web_interface/

[^22]: https://www.youtube.com/watch?v=HyjUCdK_5Js

[^23]: https://www.aranacorp.com/en/create-a-web-interface-to-control-your-raspberry-pi/

[^24]: https://github.com/ThisIsQasim/WebGPIO

[^25]: https://www.youtube.com/watch?v=p9cGt-41HXI

[^26]: https://www.ls-homeprojects.co.uk/how-to-make-a-web-interface-for-raspberry-pi-with-flask/

[^27]: https://botland.store/raspberry-pi-5-modules-and-kits/23905-raspberry-pi-5-8gb-5056561803326.html

[^28]: https://www.autobotic.com.my/raspberry-pi-products/raspberry-pi-5-model-b-8gb

[^29]: http://attach01.oss-us-west-1.aliyuncs.com/IC/Datasheet/GY20512.pdf

[^30]: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html

[^31]: https://sigmanortec.ro/en/bldc-control-module-3-phases-400w-9-60v-motor-driver-with-hall-sensor

[^32]: http://grobotronics.com/images/companies/1/HC-SR04Users_Manual.pdf

[^33]: https://vilros.com/pages/raspberry-pi-5-pinout

[^34]: https://www.aliexpress.com/item/1005008384564868.html

[^35]: https://ch.farnell.com/multicomp/hc-sr04/ultraschallsensor-40khz-4-5m/dp/4162009

[^36]: https://forum.arduino.cc/t/zs-x11h-bldc-motor-driver-arduino-gnd-connection/1097421

[^37]: https://www.one-lux.com/wp-content/uploads/2018/02/Installation-Instructions-1.pdf

[^38]: https://raspberrypi.stackexchange.com/questions/98060/connecting-rpi-to-a-motion-sensor

[^39]: https://mrperfect.dk/docs/ER-1.pdf

[^40]: https://forum.arduino.cc/t/12v-relay-sensor-to-pull-up-ardiono-gpio/984549

[^41]: https://files.pepperl-fuchs.com/selector_files/navi/productInfo/pds/224402_eng.pdf

[^42]: https://raspberry-valley.azurewebsites.net/Connecting-the-PIR-Sensor/

[^43]: https://www.eaton.com/content/dam/eaton/products/lighting-and-controls/mains-lighting/microwave-sensors/english/eaton-lighting-k882v-microwavemotionsensor-installation-sheet.pdf

[^44]: https://projecthub.arduino.cc/tarantula3/all-about-rcwl-0516-microwave-radar-motion-sensor-5aa86d

[^45]: https://soldered.com/product/microwave-motion-sensor/

[^46]: https://forums.raspberrypi.com/viewtopic.php?t=348544

[^47]: https://community.openhab.org/t/microwave-radar-motion-sensor/51456

[^48]: https://www.youtube.com/watch?v=cSI9vedf870

[^49]: https://www.instructables.com/PIR-Motion-Sensor-How-to-Use-PIRs-With-Arduino-Ras/

[^50]: https://www.youtube.com/watch?v=VOFjigia91o

[^51]: https://www.tinytronics.nl/en/sensors/motion

[^52]: https://community.element14.com/technologies/embedded/b/blog/posts/experimenting-with-microwave-based-sensors-for-presence-detection

[^53]: https://stackoverflow.com/questions/75251028/threading-using-python-in-raspberry-pi-for-gpio-inputs

[^54]: https://forums.raspberrypi.com/viewtopic.php?t=289203

[^55]: https://forums.raspberrypi.com/viewtopic.php?t=191650

[^56]: https://forums.raspberrypi.com/viewtopic.php?t=338134

[^57]: https://github.com/gavinlyonsrepo/RpiMotorLib

[^58]: https://raspberrypi.stackexchange.com/questions/140824/threading-using-python-in-raspberry-pi-for-gpio-inputs

[^59]: https://makersportal.com/blog/raspberry-pi-stepper-motor-control-with-nema-17

[^60]: https://forum-raspberrypi.de/forum/thread/55005-multi-threading-mit-kernzuweisung-bzw-periodisierung-moeglich/

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

