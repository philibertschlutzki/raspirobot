# Projekt-Spezifikation: RaspiRobot Refactoring & Autonomie (Phase 4-6)

## Fokus: Integration Phase 1-3, Modulare Architektur, TDD, Pydantic V2, SLAM & CI/CD
**Zielverzeichnis:** `5_refactoring_gemini/`
**Zielgruppe:** Jules Coding Agent (Strikte Implementierungsanweisung)
**Kontext:** Überführung des bewährten Prototypen `v3.0.2.py` in eine saubere, testbare und CI/CD-fähige Codebase.

---

## 1. Test-Driven Development (TDD) & CI/CD Pipeline
Für **jedes** neu erstellte Modul muss unmittelbar ein entsprechendes Testskript (`pytest`) geschrieben werden. Sämtliche Test-Dateien liegen isoliert im Verzeichnis `5_refactoring_gemini/tests/`.
* **PC-Test-Modus (Hardware-Bypass):** Nutze die Umgebungsvariable `PC_TEST_MODE=1`. Wenn diese gesetzt ist, dürfen hardwarenahe Module (`RPi.GPIO`, `rplidar`, `rpi_hardware_pwm`) **nicht** importiert werden. In diesem Fall werden ausschließlich Mocks geladen.
* **GitHub Actions Pipeline:** Erstelle als finalen Schritt die Datei `.github/workflows/pytest_pipeline.yml`. Dieser Workflow muss bei jedem Push greifen, `PC_TEST_MODE=1` setzen, alle Pytests ausführen und zwingend den `log_validator.py` über die Test-Logs laufen lassen.

---

## 2. Globales strukturiertes Logging & Validierung
* **Globaler JSONL-Logger:** Erstelle eine globale Logger-Instanz (Singleton) nach Best Practices in `logger.py`.
* **JSONL-Schema (Zwingend):** Jedes Log-Event muss strikt dieses Schema einhalten: `{"timestamp": float, "level": "INFO|WARN|ERROR", "event_name": "...", "data": {...}}`
* **Log-Validator (CLI):** Erstelle `tests/log_validator.py` (eigenständiges CLI-Tool, z.B. `python log_validator.py tests/logs/run_1.jsonl --require ULTRASONIC_EMERGENCY_STOP`). Exit-Code 0 (Pass) oder 1 (Fail).
* **Housekeeping:** `pytest` löscht generierte `.jsonl`-Dateien bei einem erfolgreichen Durchlauf automatisch. Schlägt ein Test fehl, wird das Log in `tests/logs/` dauerhaft persistiert und muss ins Git committet werden.

---

## 3. Datenmodellierung (Pydantic V2)
* Ersetze alle bestehenden `dataclasses` durch strikte **Pydantic V2-Modelle** (in `data_models.py`).
* Nutze zwingend V2-Syntax (z.B. `.model_dump()` anstelle des alten `.dict()`).
* Gilt insbesondere für den Payload des JSONL-Loggers, `RobotPose`, `ControllerSample`, `LidarFrame` und das komplette `PathRecordingData` für das `.json.gz` Recording.

---

## 4. Architektur-Vorgaben: Interfaces & Mocking
* **`interfaces.py`:** Definiere abstrakte Basisklassen (`abc.ABC`).
  * `IMotorController`: Erwartet normierte Werte (-1.0 bis 1.0).
  * `ILidarSensor`: Liefert Scans und Frequenz-Infos.
  * `IUltrasonicSensor`: Liefert Distanz in cm.
  * `IInputController`: Kapselt Controller-Eingaben.
* **`tests/mocks/`:** Erstelle für alle Interfaces lauffähige Mock-Klassen für die TDD-Pipeline.

---

## 5. Kapselung der Phase 1-3 (Hardware, Odometrie & Recording)
Basierend auf der Logik von `v3.0.2.py`:
* **`hardware/` (Konkrete Adapter):** Erstelle `RealMotorController`, `RealRPLidar`, `RealUltrasonic` und `XboxInput`. 
  * *Hinweis zu XboxInput:* Implementiere zwingend den Headless-Workaround (`os.environ["SDL_VIDEODRIVER"] = "dummy"`), da dieser Adapter auf dem PC oder in der CI/CD ohne Monitor laufen können muss (wird nur zur Pfadaufzeichnung benötigt).
* **`OdometryEngine`:** Testbare Klasse. Berechnet X/Y/Theta aus PWM-Werten und Zeit-Delta (`dt`).
* **`DataRecorder`:** Kapselt das asynchrone Wegschreiben der Pydantic-Modelle als `.json.gz`.

---

## 6. Der zentrale Orchestrator (Control Loop & Threading)
* **`robot_orchestrator.py`:** Ersetzt die alte `main_mapping.py` vollständig. Instanziiert **keine** Hardware selbst, sondern fordert alle Abhängigkeiten per Dependency Injection an.
* **Thread-Management:** Die Hardware-Klassen starten *keine* eigenen Threads. Der `RobotOrchestrator` übernimmt das Verwalten der Hintergrund-Threads (Lidar, Ultraschall) sowie die synchrone 50Hz Hauptschleife.
* Triggert bei Konflikten Event-Logs (z.B. `ULTRASONIC_EMERGENCY_STOP` bei < 15cm, `LIDAR_TIMEOUT_STOP` bei > 300ms Latenz).

---

## 7. SLAM & Simulator (Phase 4-6)
* **`slam_engine.py`:** Vektorisierter Point-to-Point ICP (Iterative Closest Point) via NumPy/SciPy. Initial Guess kommt aus der `OdometryEngine`.
* **Grid:** 400x400 Zellen (0.05m Resolution), Startpunkt `(0,0)` liegt bei Array-Index `[200, 200]`.
* **`simulate_perfect_run.py`:** Generiert deterministische Referenzdaten (Raum 6x4m, Hindernis 1m Dreieck). Raycasting inline implementiert. Speichert als `simulated_perfect_run.json.gz`.
* **`simulator_replay.py`:** Generator mit Matplotlib-Echtzeitanzeige (jedes 10. Frame). Speichert finale Artefakte in `results/` (`final_map.npy`, `final_map.png`).

---

## 8. Chronologische Ausführungsanweisung für Jules
*Führe diese Schritte nacheinander und vollautonom aus:*

1. **Abhängigkeiten:** Überschreibe/aktualisiere die Datei `requirements.txt` im Root-Verzeichnis (`philibertschlutzki/raspirobot/requirements.txt`) und füge `pydantic>=2.0`, `pytest` und `scipy` hinzu.
2. Erstelle `data_models.py` (Pydantic V2).
3. Erstelle globalen Logger (`logger.py`) & `tests/log_validator.py`. -> Erstelle Unit-Tests.
4. Erstelle `interfaces.py` & Mock-Klassen (`tests/mocks/`). -> Erstelle Unit-Tests.
5. Erstelle `OdometryEngine` & `DataRecorder`. -> Erstelle Unit-Tests.
6. Erstelle konkrete Adapter in `hardware/` (inkl. `PC_TEST_MODE` Fallback & Headless-Pygame Workaround).
7. **Altlasten entfernen & Orchestrierung:** Lösche die alte Datei `5_refactoring_gemini/main_mapping.py` eigenständig. Erstelle stattdessen `robot_orchestrator.py` (inkl. Threading & Fallbacks). -> Erstelle Integrationstests (via Mocks).
8. Erstelle `simulate_perfect_run.py` (Offline-Simulator). -> Erstelle Unit-Tests.
9. Erstelle `slam_engine.py` (ICP + Occupancy Grid). -> Erstelle Unit-Tests.
10. Erstelle `simulator_replay.py` (Visualisierung & Artefakte). -> Erstelle Unit-Tests.
11. Erstelle die GitHub Actions Pipeline (`.github/workflows/pytest_pipeline.yml`).


# RaspiRobot - Systemanforderungen, Spezifikationen und Abnahmekriterien

Dieses Dokument definiert die vollständigen technischen Spezifikationen, Entwicklungsphasen und strikten Abnahmekriterien für das RaspiRobot-Projekt. Ziel ist eine autonome, native Python-basierte Point-to-Point Navigationsplattform ohne Abhängigkeit von ROS, optimiert für den Raspberry Pi 5.

Dieses Dokument ist **bindend**. Bei der Implementierung durch KI-Agenten oder Entwickler gibt es keinen Interpretationsspielraum bezüglich der definierten Architekturen, Parameter und Hardware-Schnittstellen.

---

## 1. Hardware-Konfiguration & Pinout (Raspberry Pi 5)
Die Hardware-Schnittstellen basieren auf dem bewährten Prototypen (v3.0.2) und müssen zwingend exakt so allokiert werden:

* **Ultraschall (HC-SR04) via `RPi.GPIO`:**
  * Links: `TRIG = 12`, `ECHO = 13`
  * Rechts: `TRIG = 23`, `ECHO = 24`
* **Motorsteuerung (H-Brücke):**
  * **Richtung (DIR) via `RPi.GPIO`:** `DIR_LEFT_PIN = 20`, `DIR_RIGHT_PIN = 21`
  * **Geschwindigkeit (PWM) via `rpi_hardware_pwm`:**
    * Chip: `PWM_CHIP = 0`
    * Links: `PWM_CHANNEL_LEFT = 0`
    * Rechts: `PWM_CHANNEL_RIGHT = 1`
* **LIDAR (RPLidar C1):**
  * Port: `/dev/ttyUSB0`
  * Baudrate: `460800`
  * Python-Paket: `rplidar` / `rplidar-robotiq` (fest in `requirements.txt`).
* **Eingabe:** Xbox Wireless Controller. In Headless-Umgebungen (CI/CD) muss zwingend `os.environ["SDL_VIDEODRIVER"] = "dummy"` gesetzt werden, um `pygame`-Abstürze zu verhindern.

---

## 2. Test- & Validierungsstrategie (CI/CD Quality Gates)
Das Projekt nutzt einen zweistufigen Testansatz, der vollständig in **GitHub Actions** (`ubuntu-latest`) integriert ist.

1. **Linting & Code Style:** Zwingende Prüfung durch `ruff` & `black`.
2. **Hardware-Bypass (PC-Modus):** Die globale Umgebungsvariable `PC_TEST_MODE=1` lädt automatisch alle Mock-Klassen. Importe von hardwarenahen Bibliotheken (`rpi_hardware_pwm`, `RPi.GPIO`, `pygame`-Video) müssen hinter dieser Prüfung gekapselt werden, um in der CI-Pipeline lauffähig zu bleiben.
3. **Testabdeckung:** `pytest` mit `pytest-cov` erzwingt ein hartes Limit von **mindestens 80% Coverage**.
4. **Log-Housekeeping (Audit-Vorgabe):** In `tests/conftest.py` muss ein Hook implementiert sein. Erzeugte `.jsonl`-Logfiles von bestandenen Tests (`report.passed`) müssen sofort via `os.remove()` gelöscht werden. Nur bei Fehlschlägen werden die Logs im Ordner `tests/logs/` dauerhaft persistiert.
5. **Strukturiertes Logging:** Ein Singleton-Logger in `logger.py` schreibt strikt im JSONL-Format: `{"timestamp": float, "level": "INFO|WARN|ERROR", "event_name": str, "data": dict}`. Der Test `log_validator.py` prüft dies als CI-Schritt.

---

## 3. Architektur & Datenmodellierung
* **Dependency Injection:** Hardware-Komponenten werden über abstrakte Basisklassen (`interfaces.py`) geladen und in den zentralen `RobotOrchestrator` injiziert. Dies ermöglicht das vollständige Mocken der Hardware.
* **Pydantic V2:** Alle Datenstrukturen (Posen, Sensor-Daten, Recording-Frames) sind zwingend als Pydantic V2 Modelle mit `model_config = ConfigDict(frozen=True)` (unveränderlich) zu definieren. Serialisierung geschieht ausschließlich über `.model_dump(mode='json')`.
* **Main-Loop:** Der zentrale Orchestrator-Control-Loop läuft synchron mit exakt **50 Hz**.
* **Sensor-I/O & Threading:** Lidar und Ultraschall laufen in asynchronen Background-Threads und puffern ihre Daten thread-safe. Der 50Hz Main-Loop blockiert **niemals** bei Sensor- oder Datei-I/O.

---

## 4. Aktuelle Entwicklungs-Roadmap (Autonomie-Engine)

### Phase 4: Daten-Erfassungs-Engine (Mapping-Modus)
**Ziel:** Robuste Aufzeichnung hochwertiger Sensordaten in komprimierten JSON-Archiven.
* **Speicher-Management:** Um I/O-Spikes und Frame-Drops der 50Hz-Schleife zu verhindern, werden Lidar-, Controller- und Odometrie-Daten gepuffert und strikt in einem asynchronen Background-Thread in **500 MB Chunks** via `gzip` komprimiert und gespeichert.
* **Namenskonvention:** Generierung eindeutiger, timestamp-basierter Dateinamen zur Vermeidung von Konflikten (z.B. `run_20251010_120000_part1.json.gz`).

### Phase 5: Offline-Simulator & Data-Replay
**Ziel:** PC-basierte Entwicklungsumgebung zum iterativen Testen von Algorithmen direkt im RAM.
* **Anforderungen:** Einlesen der `.json.gz`-Chunks via Python-Generatoren.
* **Abnahmekriterium (Determinismus):** Zweifaches Abspielen einer Aufzeichnung muss bitgenau den identischen Output generieren.

### Phase 6: Native 2D SLAM & Mapping-Engine
**Ziel:** Eigener SLAM-Algorithmus zur Erstellung einer globalen Karte.
* **Autorität:** Das LIDAR-Scan-Matching eliminiert den Odometrie-Drift und hat die absolute Pose-Autorität. PWM-Werte sind ausschließlich der Initial-Guess.
* **Berechnung:** Vollständig vektorisiert via `numpy`/`scipy` (z.B. Point-to-Point ICP). Keine unperformanten Python-Schleifen über Punktwolken.
* **Grid-Spezifikationen:** Das Occupancy Grid wird statisch als 2D `.npy` Array (400x400 Zellen) mit einer Resolution von 0.05m (20x20m Fläche) allokiert.
* **Origin:** Der logische Startpunkt des Roboters (x=0, y=0) liegt zwingend auf dem Mittelpunkt-Index `[200, 200]`.

### Phase 7: Globale Lokalisierung (MCL - Kidnapped Robot)
**Ziel:** Selbstständige Pose-Ermittlung auf der bestehenden `.npy`-Karte.
* **Anforderungen:** Implementierung eines Partikelfilters (Monte Carlo Localization).
* **Konfigurierbarkeit:** Um die CPU des Pi 5 nicht zu überlasten, müssen die Partikel-Grenzen (`NUM_PARTICLES`) zwingend über eine `config.yaml` einstellbar sein (Best Practice: max. 1000-2000 initial, 100-200 beim Live-Tracking).

### Phase 8: Point-to-Point Navigations-Engine (Global)
**Ziel:** Vollautonome Fahrt zu einer Koordinate via CLI-Befehl (z. B. `python navigate.py --target-x 2.5 --target-y 1.0`).
* **Anforderungen:**
  * **Global Planner:** A* (A-Star) oder Dijkstra auf dem 400x400 Occupancy Grid.
  * **Local Planner (DWA):** Der Dynamic Window Approach umfährt dynamische Lidar-Hindernisse unter strikter Einhaltung des Roboter-Kollisionsradius.
  * **Zweistufiger Fail-Safe (Ultraschall):**
    * *Stufe 1 (DWA Replanning):* Ultraschall-Daten (z.B. < 30cm) werden in die Costmap des DWA injiziert, damit dieser Ausweichrouten selbstständig plant.
    * *Stufe 2 (Hardware Override):* Unterschreitet der Ultraschall den kritischen Wert von **< 15 cm**, wird unabhängig vom DWA ein Hardware-Notstopp ausgelöst (PWM hart auf `0.0`, Rückwärts-Korrektur).
  * **Ziel-Metrik:** Erfolgreich erreicht, wenn der Roboter stabil steht und die Pose innerhalb von 30 cm um das Ziel liegt.

### Phase 9: Performance-Optimierung & Hardening
**Ziel:** Refactoring für den produktiven Dauereinsatz.
* **Anforderungen:** Konsequentes Auslagern rechenintensiver Matrix-Operationen durch tiefe Numpy-Vektorisierung und strikte Trennung von asynchronem I/O und mathematischer Logik im Main-Loop.
