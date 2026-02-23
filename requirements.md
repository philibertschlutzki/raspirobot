# RaspiRobot - Systemanforderungen, Spezifikationen und Abnahmekriterien

Dieses Dokument definiert die vollständigen technischen Spezifikationen, Entwicklungsphasen und strikten Abnahmekriterien für das RaspiRobot-Projekt. Ziel ist eine autonome, native Python-basierte Point-to-Point Navigationsplattform ohne Abhängigkeit von ROS oder Rad-Encodern, optimiert für den Raspberry Pi 5.

---

## 1. Test- & Validierungsstrategie (CI/CD Quality Gates)
Das Projekt nutzt einen zweistufigen Testansatz, der vollständig in **GitHub Actions** (`ubuntu-latest`) integriert ist. Die Pipeline setzt harte "Quality Gates" voraus:

1. **Linting & Code Style (Pre-Test):** Code wird zwingend durch `ruff` & `black` geprüft (Typisierung, Docstrings, modulares Design). Fehlschlag führt zum Pipeline-Abbruch.
2. **Mocking & CI-Pipeline:** Die Umgebungsvariable `export RASPI_ENV=testing` lädt automatisch Mock-Klassen für Hardware-Komponenten. Testrunner ist **`pytest`**.
3. **Testabdeckung (Coverage):** `pytest-cov` erzwingt ein Limit von **mindestens 80% Code Coverage**.
4. **Timeouts & Virtuelle Zeit:** Um Wartezeiten (z. B. Backoffs bei Re-Routing) in der CI zu vermeiden, nutzen Mock-Tests Time-Mocking-Bibliotheken (wie `freezegun` oder `pytest-asyncio`). Die GitHub Actions Pipeline hat ein hartes Timeout von 10 Minuten.
5. **Hardware Test-Scripts:** Physische Sensoren werden lokal auf dem Pi über dedizierte Scripts in `tests/hardware_validation/` kalibriert.
6. **Strukturiertes Logging:** Events werden als **JSON Lines (`.jsonl`)** protokolliert. Ein Validator (`log_validator.py`) parst diese Logs nach dem Testlauf und gibt `PASS` oder `FAIL` zurück.

---

## 2. Historischer Kontext: Phase 1 bis 3 (Fundament & Refactoring)
*Abgeschlossene Phasen als architektonisches Fundament.*

* **Architektur:** Zeitstempel-basierte Aufzeichnung von Controller (50Hz) und RPLidar C1 Scans (10Hz) als `.json.gz`. Sensorfusion mit HC-SR04 Ultraschall-Sensoren (Median-Filterung).
* **Dynamische Geschwindigkeitsanpassung (Lidar):**
  - Latenz **< 120 ms**: 100% Max-Speed.
  - Latenz **120 ms bis 300 ms**: Lineare Drosselung (Graceful Degradation) bis auf 10% Speed.
  - Latenz **> 300 ms**: Motor-Notstopp (0% PWM).
* **Zugehörige Abnahmekriterien (TC 1.x):**
  - **TC 1.1:** Lokales Script ermittelt reales Sensor-Rauschen auf dem Pi.
  - **TC 1.2:** Ultraschall-Filterung ignoriert simulierte "0cm"-Ausreißer ohne Notstopp.
  - **TC 1.3:** Korrektes Logging der PWM-Drosselung bei simulierter Lidar-Latenz >120ms.

---

## 3. Aktuelle Entwicklungs-Roadmap (Autonomie-Engine)

### Phase 4: Daten-Erfassungs-Engine (Mapping-Modus)
**Ziel:** Robuste Aufzeichnung hochwertiger Sensordaten in `.json.gz` Archiven für Offline-Verarbeitung.
* **Anforderungen:** Parallele Aufzeichnung von LIDAR und Motor-PWM (als grobe Odometrie). Ultraschall dient nur als Multi-Layer Fallback für Kollisionsvermeidung bei manueller Fahrt (keine Autorität über Mapping).
* **Abnahmekriterium (TC 4.1):** Eine gemockte 5-Minuten-Fahrt erzeugt ein fehlerfreies Archiv ohne Frame-Drops > 0.5 Sekunden.

### Phase 5: Offline-Simulator & Data-Replay
**Ziel:** PC-basierte Entwicklungsumgebung zum iterativen Testen von Algorithmen direkt im RAM.
* **Anforderungen:** Einlesen der `.json.gz` via Python-Generatoren. Bereitstellung der Sensordaten in korrekter Reihenfolge. Visuelle Debug-Ausgabe (z.B. `matplotlib`).
* **Abnahmekriterium (TC 5.1 - Determinismus):** Zweifaches Abspielen einer Datei muss bitgenau den identischen Output-Stream generieren.

### Phase 6: Native 2D SLAM & Mapping-Engine
**Ziel:** Eigener SLAM-Algorithmus (z.B. ICP) in Python zur Erstellung einer globalen Wohnungskarte.
* **Anforderungen:**
  - Absolute Pose-Autorität liegt beim LIDAR-Scan-Matching. PWM-Werte sind nur der initiale Vorhersage-Guess.
  - Ausgabe als statisches `numpy`-Array (`.npy`) gepaart mit `.json`-Metadaten.
  - **Grid-Spezifikationen:** 400x400 Zellen. Resolution fix 0.05m (20x20 Meter). Startpunkt (x=0, y=0) mappt zwingend auf den Array-Mittelpunkt `[200, 200]`.
* **Abnahmekriterium (TC 6.1 - Mapping Drift):** Nach simulierter Loop-Closure-Fahrt darf die berechnete Endpose max. **+/- 5 cm** (1 Pixel) von den realen, durch Wände definierten physischen Start-Abständen abweichen.

### Phase 7: Globale Lokalisierung & Dynamische Navigation
**Ziel:** Der Roboter ermittelt seine Pose via Monte Carlo Localization (Kidnapped Robot) und umfährt dynamische, unbekannte Hindernisse über eine lokale Costmap autonom.
* **Anforderungen:**
  - **Konfiguration & Map-Validierung:** Hindernis-Inflation ist konfigurierbar (z.B. `OBSTACLE_INFLATION_RADIUS_M = 0.3`). Beim Laden der Karte wird validiert: >= 5% "Freiraum" und >= 0.5% "Wände". Sonst Startabbruch (`MapQualityException`).
  - **MCL (Partikelfilter):** Vektorisierte Berechnung via NumPy. Nutzung eines Likelihood-Fields (via `scipy.ndimage.distance_transform_edt`) statt Raycasting. Adaptives Resampling.
  - **Strikte Trennung:** MCL berechnet nur die Pose. Entdeckte dynamische Lidar-Hindernisse (Distanz-Cutoff im Likelihood-Field) triggern lediglich ein Event (`unexpected_obstacle_detected`).
  - **Local Path Planning & Pure Pursuit:** Bei einem Hindernis-Event berechnet ein A*-Algorithmus auf einer lokalen 3x3m Costmap (inkl. Hindernis-Inflation) eine Umgehungsroute (Ziel 2m voraus). Wegpunkte werden geglättet und via "Pure Pursuit"-Controller abgefahren.
  - **Ultraschall-Sensorfusion (Replanning):** Wirft der Ultraschall bei < 15cm einen Notstopp, fährt der Roboter langsam max. 15cm rückwärts (bis Sensor >30cm meldet). Das Hindernis wird in die Costmap injiziert und eine Neu-Berechnung (Replanning) gestartet, um Deadlocks zu vermeiden.
* **Abnahmekriterien:**
  - **TC 7.1 (Pose-Konvergenz):** Simulation von 10 zufälligen Startorten. In 9 von 10 Fällen konvergieren 95% der Partikel nach max. einer 360-Grad-Drehung innerhalb einer **10 cm Bounding-Box** (Theta-Abweichung < 5 Grad).
  - **TC 7.2 (Hindernis-Umgehung Lidar):** Ein unerwartetes Hindernis wird ins Lidar injiziert. A* berechnet in < 500ms eine kollisionsfreie Vektor-Kurve unter Berücksichtigung der Costmap-Inflation.
  - **TC 7.3 (Ultraschall-Replanning):** Ultraschall-Mock wirft Stop. Assert auf korrekten "Reverse"-Befehl, Costmap-Injection und erneuten A*-Call.

### Phase 8: Point-to-Point Navigations-Engine (Global)
**Ziel:** Vollautonome Fahrt von Raum A nach Raum B auf der globalen Karte via CLI-Befehl.
* **Anforderungen:**
  - **Global Planner:** A* (A-Star) oder Dijkstra plant die grundlegende Route über das gesamte 400x400 Occupancy Grid.
  - **Local Planner / DWA:** Der Dynamic Window Approach (DWA) folgt dem globalen Pfad und passt Geschwindigkeit/Rotationsraten unter strikter Berücksichtigung des Fußabdrucks (Kollisionsradius) dynamisch an.
  - CLI-basiert (z.B. `python navigate.py --target-x 2.5 --target-y 1.0`).
* **Abnahmekriterien:**
  - **TC 8.1 (Kollisionsfreiheit):** Anfahren von 5 gemockten globalen Koordinaten ohne Wandkontakt im Occupancy Grid.
  - **TC 8.2 (Re-Routing & Timeouts):** Wird der globale Weg blockiert, stoppt das System und startet globales Re-Routing mit gestaffeltem Backoff (simuliert via virtueller Zeit): 1. Versuch nach **10s**, 2. Versuch nach **20s**, 3. Versuch nach **40s**. Nach 5 Fehlversuchen folgt `NAVIGATION_ABORTED` (Validierung via Log-Timestamp-Check).
  - **TC 8.3 (Unreachable Target):** Liegt das finale Ziel innerhalb eines Hindernisses, fährt der Roboter bis auf **10 cm** heran, stoppt und loggt `TARGET_PARTIALLY_REACHED`.

### Phase 9: Performance-Optimierung & Hardening (Abschluss)
**Ziel:** Refactoring für den produktiven Dauereinsatz auf dem Raspberry Pi 5 unter Einhaltung mathematischer Korrektheit.
* **Anforderungen:** Konsequentes Refactoring rechenintensiver Matrix-Operationen durch tiefergehende Numpy-Vektorisierung und Multiprocessing (Auslagerung von Sensor-I/O), um Latenzen bei der dynamischen Planung endgültig zu minimieren.
* **Abnahmekriterium (TC 9.1 - Pfadabweichungs-Metrik):** Vergleich eines vorberechneten, "idealen" Referenzpfades mit dem durch die echtzeit-optimierte Engine gefahrenen Pfad unter hoher CPU-Simulationslast. Die Abweichung darf **10% der zurückgelegten Gesamtstrecke** nicht überschreiten (z. B. bei 5m Strecke max. 50cm Abweichung an der extremsten Stelle).
