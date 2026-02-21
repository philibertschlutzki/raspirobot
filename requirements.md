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
**Ziel:** Vorbereitung für den produktiven Einsatz.
**Anforderungen:**
- Refactoring rechenintensiver Matrix-Operationen (Vektorisierung durch Numpy).
- Erst in dieser Phase wird Geschwindigkeit eine relevante Metrik. Mathematische Korrektheit der Algorithmen aus Phase 6-8 darf dabei nicht beeinträchtigt werden.
