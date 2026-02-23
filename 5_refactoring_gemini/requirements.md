# Projekt-Spezifikation: RaspiRobot Autonomie-Refactoring (Phase 4-6)

## Fokus: Modulare Architektur, Hardware-Abstraktion, Mapping & Test-Driven Development (TDD)
**Zielverzeichnis:** `5_refactoring_gemini/`
**Zielgruppe:** Jules Coding Agent (Strikte Implementierungsanweisung)

Diese Spezifikation definiert die Architektur und die konkreten Entwicklungsschritte zur Überführung der bestehenden Prototypen in eine saubere, testbare und CI/CD-fähige Codebase. 

---

## 1. Test-Driven Development (TDD) & Kapselung
Für **jedes** neu erstellte Entwicklungs-Skript muss unmittelbar danach ein entsprechendes Abnahme-Testskript für `pytest` geschrieben werden, bevor das nächste Feature implementiert wird.
* **Ordnerstruktur:** Sämtliche Test-Dateien müssen isoliert im dedizierten Verzeichnis `5_refactoring_gemini/tests/` abgelegt werden. Logdateien landen im Unterordner `5_refactoring_gemini/tests/logs/`.

---

## 2. Strukturiertes Logging & Event-Validierung (.jsonl)
Die Codebase muss von Beginn an so konzipiert sein, dass sie System-Events aufzeichnet und diese deterministisch validiert werden können.
* **Fixes JSONL-Schema (Zwingend):** Jede Log-Zeile muss exakt diesem Format entsprechen:
  `{"timestamp": float, "level": "INFO|WARN|ERROR", "event_name": "...", "data": {...}}`
* **Log-Validator (CLI):** Erstelle als allererstes das Skript `5_refactoring_gemini/tests/log_validator.py`. Dieses Skript wird als **eigenständiges CLI-Tool** aufgerufen (z. B. `python log_validator.py tests/logs/run_1.jsonl --require ULTRASONIC_EMERGENCY_STOP`). Es parst die Logs und gibt Exit-Code 0 (Pass) oder 1 (Fail) zurück.
* **Housekeeping (Cleanup & Git-Persistenz):** Die `pytest`-Skripte müssen so konfiguriert sein, dass die generierten `.jsonl`-Dateien bei einem **erfolgreichen Testdurchlauf automatisch gelöscht** werden. Schlägt ein Test jedoch fehl, **müssen die Logs dauerhaft persistiert werden**. Diese fehlgeschlagenen Logs dürfen NICHT in die `.gitignore` aufgenommen werden, sondern **müssen zwingend in das Git-Repository committet werden**, um die Fehler in der Historie nachvollziehbar zu machen.

---

## 3. Architektur-Vorgaben & Hardware-Abstraktion
Die Architektur muss vollständig entkoppelt sein.
* **Datei `interfaces.py`:** Erstelle abstrakte Basisklassen (via `abc.ABC`).
  * `class IMotorController(ABC):` Erwartet ausschließlich normierte Werte von `-1.0` (100% rückwärts) bis `1.0` (100% vorwärts). 
  * `class ILidarSensor(ABC):` Methode `get_scan() -> List[Tuple[float, float]]` (Winkel, Distanz).
  * `class IUltrasonicSensor(ABC):` Methode `get_distance() -> float`.
* **Dependency Injection:** Die Zentrale instanziiert Hardware nicht selbst, sondern erhält die Interfaces im Konstruktor.
* **Mock-Klassen (`tests/mocks/`):** Erstelle Mocks, die diese Interfaces bedienen.
* **TDD-Schritt:** Erstelle nach diesem Block sofort `tests/test_interfaces.py` und `tests/test_mocks.py`.

---

## 4. SLAM & Mapping-Engine (Algorithmus)
Nutze einen **Vektorisierten Point-to-Point ICP-Algorithmus (Iterative Closest Point)** kombiniert mit einem probabilistischen Occupancy Grid (`numpy` / `scipy`).
* **Initial Guess:** Die grobe Odometrie dient dem ICP *ausschließlich* als Startpunkt für die Scan-Ausrichtung (Drift-Eliminierung).
* **Grid-Spezifikationen:** * Allokiertes Array der Größe 400x400 Zellen (Datentyp `np.int8` oder `np.float32`).
  * Resolution = fix 0.05m (5cm).
  * Center-Origin: Startpunkt `(x=0.0, y=0.0, theta=0.0)` liegt zwingend auf Index `[200, 200]`.
* **TDD-Schritt:** Erstelle `tests/test_slam_engine.py`, das prüft, ob Array-Dimensionen und Odometrie korrekt funktionieren. Das Event `MAPPING_STARTED` muss im `.jsonl` via CLI-Validator validiert werden.

---

## 5. Offline-Simulator & Testdaten-Generierung
Für deterministische Tests muss der Agent perfekte Referenzdaten simulieren.
* **Datei `simulate_perfect_run.py`:** Ein Skript, das eine deterministische Fahrt generiert.
  * **Umgebung:** Rechteckiger Raum **6x4 Meter**. Im Zentrum befindet sich ein **Dreieck mit 1 Meter Seitenlänge** als Hindernis.
  * **Trajektorie:** Navigation um dieses Dreieck herum.
  * **Raycasting:** 2D-Strahlenschnitte gegen Wände und Dreieck berechnen. **Wichtig:** Logik wird direkt und linear in `simulate_perfect_run.py` geschrieben (keine Auslagerung in Utils).
  * **Output:** Speicherung als `path_recordings/simulated_perfect_run.json.gz`.
* **TDD-Schritt:** Erstelle `tests/test_simulation.py`.

---

## 6. Echtzeit-Visualisierung & Replay
* **Datei `simulator_replay.py`:** Ein Python-Generator, der die `.json.gz` einliest.
* **Echtzeit-Anzeige (Matplotlib):** Occupancy Grid und Pose werden dynamisch aktualisiert. 
  * Update nur bei jedem **10. Frame**. CLI-Flag (`--visualize`) deaktiviert die UI.
* **Prüfbare Artefakte:** Zwingende Ablage in `results/`: `final_map.npy`, `final_map.png`, `map_metadata.json`.
* **TDD-Schritt:** Erstelle `tests/test_replay_artifacts.py`.

---

## 7. Sicherheit & Fallbacks
* **Ultraschall:** Distanz < 15 cm triggert sofortigen Stopp. Event `ULTRASONIC_EMERGENCY_STOP` wird geloggt.
* **LIDAR Latenz:** Simulierte Latenzen > 300ms lösen Notstopp aus. Event `LIDAR_TIMEOUT_STOP` wird geloggt.
* **TDD-Schritt:** Erstelle `tests/test_safety_fallbacks.py`, triggere die Mocks und prüfe die Events zwingend über einen Aufruf des `log_validator.py`. Bei Fehler muss das Log im Repo verbleiben (und committet werden!).

---

## 8. Chronologische Arbeitsanweisung für Jules
1. Erstelle `tests/log_validator.py` als CLI-Tool für fixes `.jsonl`-Schema.
2. Erstelle `interfaces.py` -> Erstelle Test (inkl. Cleanup-Logik für erfolgreiche Logs).
3. Erstelle `tests/mocks/` -> Erstelle Test.
4. Erstelle `simulate_perfect_run.py` (6x4m, 1m Dreieck, Raycasting inline) -> Erstelle Test.
5. Erstelle `slam_engine.py` (ICP + Grid) -> Erstelle Test.
6. Erstelle `simulator_replay.py` (Matplotlib alle 10 Frames, Artefakte) -> Erstelle Test.
7. Refaktoriere die `main_mapping.py` auf Dependency Injection.
