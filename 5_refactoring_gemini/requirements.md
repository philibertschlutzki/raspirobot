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
