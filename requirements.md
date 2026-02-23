# RaspiRobot - Systemanforderungen und Spezifikationen

Dieses Dokument definiert die vollständigen technischen Spezifikationen und Entwicklungsphasen für das RaspiRobot-Projekt. Ziel ist eine autonome, native Python-basierte Point-to-Point Navigationsplattform ohne Abhängigkeit von ROS oder Rad-Encodern.

## Historischer Kontext: Phase 1 bis 3 (Abgeschlossen)
Diese Phasen wurden bereits erfolgreich implementiert und dienen als architektonisches Fundament.
* **Phase 1 (Datenstruktur):** Zeitstempel-basierte Speicherung von Controller-Eingaben (50Hz) und synchronisierten LIDAR-Scans (10Hz) im `.json.gz` Format.
* **Phase 2 (Controller-Logik):** Robuste Steuerung per Xbox Controller mit parallelen Recording-Threads, ohne die `control_loop` Latenz zu beeinflussen.
* **Phase 3 (Sensorfusion & Hardware):** Integration von RPLIDAR C1 und 4x Ultraschall-Sensoren (HC-SR04). Asynchrones Multithreading für sichere Motorsteuerung (PWM) mit Graceful Degradation.

---

## Aktuelle Entwicklungs-Roadmap (Autonomie-Engine)

### Phase 4: Daten-Erfassungs-Engine (Mapping-Modus)
**Ziel:** Robuste Aufzeichnung hochwertiger Sensordaten für die spätere Offline-Verarbeitung.
**Anforderungen:**
- Parallele Aufzeichnung von LIDAR-Frames und Motor-PWM-Werten (als grobe Odometrie-Vorhersage).
- Ultraschall dient ausschließlich als Multi-Layer Fallback für Kollisionsvermeidung während der manuellen Fahrt, hat aber keine Autorität über die Karten-Generierung.
- Speicherung in versionierten `.json.gz` Archiven.

### Phase 5: Offline-Simulator & Data-Replay
**Ziel:** Eine PC-basierte Entwicklungsumgebung, um SLAM- und Navigations-Algorithmen ohne Hardware-Einsatz zu iterieren.
**Anforderungen:**
- Einlesen der `.json.gz` Dateien.
- Bereitstellung der Sensordaten an die nachfolgenden Algorithmen via Python-Generatoren direkt im RAM (exakte Reproduktion des originalen Timings ist optional, Fokus liegt auf korrekter Datenreihenfolge).
- Ausgabe und Visualisierung der Map-Generierung (z.B. via `matplotlib` als Debug-Tool).

### Phase 6: Native 2D SLAM & Mapping-Engine
**Ziel:** Eigener SLAM-Algorithmus in Python zur Erstellung einer globalen Wohnungskarte.
**Anforderungen:**
- **Autorität:** Die absolute Autorität über die Roboter-Pose und das Mapping liegt beim LIDAR-Scan-Matching. PWM-Werte dürfen nur als initialer Guess (Vorhersage) verwendet werden.
- **Algorithmus:** Freie Wahl des optimalen 2D-Scan-Matching-Algorithmus (z.B. ICP oder Correlative Scan Matching, gerne unter Nutzung von `scipy.optimize` oder `numpy`), der den Odometrie-Drift vollständig eliminiert.
- **Karten-Format:** Ausgabe als statisches 2D-Occupancy Grid in einem numerischen `numpy`-Array (`.npy`) gepaart mit einer `.json`-Metadaten-Datei.
- **Grid-Spezifikationen:** Statisch allokiertes Array der Größe 400x400 Zellen. Die Zellengröße (Resolution) beträgt fix 0.05m (5cm), was einer Fläche von 20x20 Metern entspricht.
- **Ursprungs-Mapping (Center-Origin):** Der logische Startpunkt des Roboters (x=0, y=0) beim Start des Mappings wird zwingend auf den exakten Mittelpunkt des Arrays (Index `[200, 200]`) gemappt, um Fahrten in alle Himmelsrichtungen (inkl. rückwärts) ohne negative Array-Indizes zu ermöglichen.

### Phase 7: Globale Lokalisierungs-Engine (Kidnapped Robot)
**Ziel:** Der Roboter muss seine Pose auf der bestehenden `.npy`-Karte selbstständig ermitteln können, egal wo er abgesetzt wird.
**Anforderungen:**
- Implementierung eines Partikelfilters (Monte Carlo Localization - MCL).
- Der Algorithmus streut Partikel über die Karte und lässt diese durch Abgleich von aktuellen LIDAR-Scans mit der geladenen Karte konvergieren.
- Erfolgsmetrik: Der Algorithmus muss die wahre Pose (x, y, theta) mathematisch stabil isolieren.

### Phase 8: Point-to-Point Navigations-Engine
**Ziel:** Vollautonome Fahrt zu einem Zielpunkt auf der Karte.
**Anforderungen:**
- **Konfiguration:** Kinematische Parameter (Roboter-Fußabdruck/Kollisionsradius, Radstand=20cm, Max-Speed=0.5m/s) müssen zwingend in einer dedizierten Konfigurationsdatei (z.B. `config.yaml` oder `.json`) ausgelagert sein.
- **Global Planner:** Implementierung von A* (A-Star) oder Dijkstra auf dem Occupancy Grid zur Berechnung der globalen Trajektorie.
- **Local Planner:** Implementierung einer dynamischen Ausweichlogik (Dynamic Window Approach - DWA) unter strikter Berücksichtigung des Kollisionsradius.
- **Zielerreichungs-Metrik:** Das Ziel gilt als erfolgreich erreicht, wenn der Roboter stabil steht und die berechnete Pose innerhalb eines Radius von 30 cm um die CLI-Zielkoordinate liegt. Die oberste Priorität ist Kollisionsvermeidung.
- **User Interface:** Ausschließlich CLI-basiert (z.B. `python navigate.py --target-x 2.5 --target-y 1.0`).

### Phase 9: Performance-Optimierung & Hardening (Abschluss)



# RaspiRobot - Systemanforderungen, Spezifikationen und Abnahmekriterien

Dieses Dokument definiert die vollständigen technischen Spezifikationen, Entwicklungsphasen und strikten Abnahmekriterien für das RaspiRobot-Projekt. Ziel ist eine autonome, native Python-basierte Point-to-Point Navigationsplattform ohne Abhängigkeit von ROS oder Rad-Encodern, optimiert für den Raspberry Pi 5.

## Test- & Validierungsstrategie (CI/CD Quality Gates)
Das Projekt nutzt einen zweistufigen Testansatz, der vollständig in **GitHub Actions** (`ubuntu-latest`) integriert wird. Die Pipeline setzt harte "Quality Gates" voraus:

1. **Linting & Code Style (Pre-Test):** Bevor Tests starten, wird der Code zwingend durch Linter/Formatter (`ruff` & `black`) geprüft. Die Konfiguration orientiert sich an den Best Practices von "Jules Coding Agent" (Typisierung, saubere Docstrings, modulares Design). Schlägt das Linting fehl, bricht die Pipeline ab.
2. **Mocking & CI-Pipeline:** Die Umgebungsvariable `export RASPI_ENV=testing` lädt automatisch Mock-Klassen für alle Hardware-Komponenten. Der Testrunner ist **`pytest`**.
3. **Testabdeckung (Coverage):** Die Ausführung via `pytest-cov` erzwingt ein hartes Limit von **mindestens 80% Code Coverage**. Fällt die Abdeckung darunter, schlägt der CI-Run fehl.
4. **Timeouts & Virtuelle Zeit:** Um Wartezeiten (z. B. 40s bei Navigations-Abbrüchen) in der CI zu vermeiden, nutzen Mock-Tests beschleunigte "virtuelle Zeit" (Time-Mocking). Unabhängig davon hat die gesamte GitHub Actions Pipeline ein hartes **Timeout von 10 Minuten**.
5. **Hardware Test-Scripts:** Physische Sensoren werden lokal auf dem Pi über dedizierte Scripts in `tests/hardware_validation/` kalibriert (z. B. Ermittlung des Sensor-Rauschens).
6. **Strukturiertes Logging:** Events werden als **JSON Lines (`.jsonl`)** protokolliert. Ein Validator-Script (`log_validator.py`) parst diese Logs nach dem Testlauf und gibt `PASS` oder `FAIL` zurück.

---

## Historischer Kontext: Phase 1 bis 3 (Fundament & Refactoring)

* **Datenstruktur:** Speicherung von Controller (50Hz) und RPLidar C1 Scans (10Hz) als `.json.gz`.
* **Sensorfusion:** HC-SR04 Ultraschall-Sensoren mit Median-Filterung gegen "Geister-Werte".
* **Dynamische Geschwindigkeitsanpassung (RPLidar C1 Best Practice):** - Latenz **< 120 ms**: 100% Max-Speed.
  - Latenz **120 ms bis 300 ms**: Lineare Drosselung (Graceful Degradation) der PWM-Werte bis auf 10% Speed.
  - Latenz **> 300 ms**: Motor-Notstopp (0% PWM), bis wieder aktuelle Daten vorliegen.

**Abnahmekriterien & Testfälle:**
- **TC 1.1 (Hardware-Test):** Das Script in `tests/hardware_validation/` ermittelt auf dem Pi die reale Standardabweichung des Sensor-Rauschens.
- **TC 1.2 (Error-Handling Filter):** `pytest` speist "0cm"-Ausreißer in den Ultraschall-Mock ein. Der Filter muss diese ignorieren (kein Notstopp).
- **TC 1.3 (Dynamische Geschwindigkeit):** `pytest` simuliert eine Latenz von >120ms im Lidar-Mock. Der Controller muss die proportional gedrosselte PWM-Zahl loggen.

---

## Aktuelle Entwicklungs-Roadmap & Abnahmekriterien

### Phase 4: Daten-Erfassungs-Engine (Mapping-Modus)
**Ziel:** Robuste Aufzeichnung (LIDAR + Motor-PWM) in `.json.gz` Archiven.
**Abnahmekriterium:** - **TC 4.1:** Eine gemockte 5-Minuten-Fahrt erzeugt ein fehlerfreies `.json.gz`-Archiv ohne Frame-Drops > 0.5 Sekunden.

### Phase 5: Offline-Simulator & Data-Replay
**Ziel:** PC-Umgebung zum iterativen Testen von SLAM/Navigation aus dem RAM.
**Abnahmekriterium:**
- **TC 5.1 (Determinismus):** Zweifaches Abspielen einer `.json.gz`-Datei muss bitgenau den identischen Output-Stream generieren.

### Phase 6: Native 2D SLAM & Mapping-Engine
**Ziel:** ICP/SLAM in Python für ein 400x400 Occupancy Grid (`.npy`, Resolution 5cm, Ursprung `[200,200]`).
**Abnahmekriterium:**
- **TC 6.1 (Mapping Drift):** Der physische Startpunkt ist durch exakte Wandabstände definiert. Nach einer simulierten Loop-Closure-Fahrt darf die berechnete Endpose maximal **+/- 5 cm** (1 Pixel Offset) von den realen Wandabständen abweichen.

### Phase 7: Globale Lokalisierungs-Engine (Kidnapped Robot)
**Ziel:** Monte Carlo Localization (MCL) nach dem Einschalten.
**Abnahmekriterium:**
- **TC 7.1 (Pose-Konvergenz):** Simulation von 10 zufälligen Startorten. Erfolgskriterium: 95% der Partikelwolke konvergiert innerhalb einer **10 cm Bounding-Box**, Theta-Abweichung < 5 Grad. Dies muss in 9 von 10 Fällen nach max. einer 360-Grad-Drehung eintreten.

### Phase 8: Point-to-Point Navigations-Engine
**Ziel:** Vollautonome Fahrt (A*/Dijkstra & DWA) via CLI-Befehl.
**Abnahmekriterien:**
- **TC 8.1 (Kollisionsfreiheit):** Anfahren von 5 gemockten Koordinaten ohne Wandkontakt im Occupancy Grid.
- **TC 8.2 (Re-Routing & Timeouts):** Wird der Weg versperrt, stoppt das System und startet Re-Routings mit gestaffeltem Backoff (in virtueller Zeit simuliert): 1. Versuch nach **10s**, 2. Versuch nach **20s**, 3. Versuch nach **40s**. Der `log_validator.py` erzwingt einen CI-Abbruch (`FAIL`), wenn die Log-Zeitstempel der `REROUTE_ATTEMPT`-Events abweichen. Nach 5 Fehlversuchen folgt `NAVIGATION_ABORTED`.
- **TC 8.3 (Unreachable Target):** Liegt das Ziel im Hindernis, fährt der Roboter bis auf **10 cm** heran, stoppt und loggt ein `TARGET_PARTIALLY_REACHED`-Event in der `.jsonl`-Datei.

### Phase 9: Performance-Optimierung & Hardening
**Ziel:** Numpy-Vektorisierung und Multiprocessing zur Latenz-Reduzierung.
**Abnahmekriterium:**
- **TC 9.1 (Pfadabweichungs-Metrik):** Vergleich eines "idealen" Referenzpfades mit dem durch die optimierte Engine generierten Pfad. Die maximale Abweichung an der extremsten Stelle darf **10% der zurückgelegten Gesamtstrecke** nicht überschreiten (Beispiel: Bei 5 Metern Gesamtfahrt darf die größte Abweichung zur Ideallinie maximal 50 cm betragen).
**Ziel:** Vorbereitung für den produktiven Einsatz.
**Anforderungen:**
- Refactoring rechenintensiver Matrix-Operationen (Vektorisierung durch Numpy).
- Erst in dieser Phase wird Geschwindigkeit eine relevante Metrik. Mathematische Korrektheit der Algorithmen aus Phase 6-8 darf dabei nicht beeinträchtigt werden.
