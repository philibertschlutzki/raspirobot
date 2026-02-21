# Analyse, Architektur und Roadmap zur Autonomie

Dieses Dokument definiert den strategischen Pfad vom aktuellen Status Quo (manuelle Fahrt mit synchronisierter Sensordaten-Aufzeichnung) hin zum finalen Ziel: **Eine vollständig autonome, native Python-basierte Point-to-Point Navigation.**

## 1. Status Quo Analyse (v3.0.2)

Das System verfügt über ein extrem stabiles, hardwarenahes Fundament:

* **Multithreading:** Ultraschall, LIDAR (mit Frequenzüberwachung), Motorsteuerung und Recording laufen asynchron und threadsicher.
* **Sensorfusion:** Kollisionen werden hybrid über Ultraschall und LIDAR-Zonen im Nahbereich zuverlässig erkannt.
* **Datenqualität:** Komprimierte JSON-Aufzeichnungen (`.json.gz`) beinhalten saubere 10Hz LIDAR-Frames, Metadaten und Motor-PWM-Werte.

**Architektur-Entscheidung:** Das bisherige Ziel "Teach-and-Repeat" (blindes Abspielen von Controller-Eingaben) wird durch echte Autonomie ersetzt. Da keine Rad-Encoder verbaut sind, muss der unvermeidbare Drift der reinen PWM-Odometrie vollständig durch den LIDAR korrigiert werden.

---

## 2. Entwicklungs-Roadmap

Um komplexe Algorithmen mit KI-Agenten effizient zu entwickeln, wird die Entwicklung von der Hardware auf den Desktop entkoppelt.

### Phase 5: Offline-Simulator & Data-Player

Bevor autonome Logik auf dem Pi läuft, muss sie iterativ am PC entwickelt werden können.

* **Ziel:** Ein Python-Tool (`data_player.py`), das die bestehenden `recording.json.gz` Dateien einliest.
* **Funktion:** Der Player simuliert den Roboter-Stream und wirft die LIDAR-Scans und PWM-Werte exakt im originalen Timing-Intervall (z.B. 10Hz) an eine API-Schnittstelle aus.
* **Nutzen:** Erlaubt das Testen und Visualisieren (z.B. via `matplotlib` oder `Pygame`) von Map-Building und SLAM in Sekunden statt in Minuten realer Fahrzeit.

### Phase 6: Native Python SLAM-Engine (Mapping)

Anstatt Blackbox-Bibliotheken (ROS/Gmapping) zu nutzen, wird ein eigener Mapping-Algorithmus entwickelt.

* **Konzept:** Eine Mapping-Fahrt wird manuell mit dem Xbox-Controller durchgeführt.
* **Algorithmus:** Nutzung von Iterative Closest Point (ICP) oder korrelationsbasiertem Scan-Matching, um die LIDAR-Scans übereinander zu legen und den Odometrie-Drift auszugleichen.
* **Speicherung:** Generierung eines 2D Occupancy Grids. Dieses wird maschinenlesbar, ressourcenschonend als numerisches Array (`.npy` via Numpy) zusammen mit einer Metadaten-Datei (`.json` für Resolution/Origin) gespeichert.

### Phase 7: Global Localization (Lösung des "Kidnapped Robot Problem")

Der Roboter muss beim Einschalten wissen, wo er sich auf der zuvor gespeicherten Karte befindet, unabhängig vom physischen Startpunkt.

* **Konzept:** Implementierung eines Partikelfilters (Monte Carlo Localization - MCL).
* **Ablauf:** Der Roboter streut initial tausende virtuelle "Partikel" (mögliche Positionen) über die geladene `.npy`-Karte. Durch Drehung oder erste Bewegungen werden aktuelle LIDAR-Scans mit der Karte abgeglichen. Unplausible Partikel werden gelöscht, bis die Wolke auf die reale Position des Roboters konvergiert.

### Phase 8: Point-to-Point Navigations-Engine

Ersetzung der manuellen Steuerung durch vollautonome Zielfindung.

* **Eingabe:** Ein einfaches CLI-Command übergibt die X/Y-Zielkoordinaten im Karten-Referenzsystem.
* **Global Planner:** Implementierung von A* (A-Star) oder Dijkstra auf dem `.npy`-Grid, um den kürzesten bekannten Weg vom aktuellen MCL-Standort zum Ziel zu finden.
* **Local Planner:** Implementierung einer dynamischen Ausweichlogik (z.B. Dynamic Window Approach - DWA). Weicht spontanen Hindernissen (z.B. laufende Personen) aus und führt den Roboter weich auf die globale Route zurück.

### Phase 9: Performance-Optimierung & Hardening

Erst wenn die Logik (bei sehr langsamer Fahrt) mathematisch korrekt funktioniert, wird auf Geschwindigkeit optimiert.

* **Profilierung:** Identifizierung von Bottlenecks in der Python-Ausführung.
* **Optimierung:** Auslagerung rechenintensiver Grid-Berechnungen (z.B. Raycasting für den Partikelfilter) in vektorisierte Numpy-Operationen.
* **Architektur:** Verlagerung spezifischer Engine-Teile in dedizierte Multiprocessing-Prozesse, um alle Kerne des Raspberry Pi 5 auszunutzen.

---

## 3. Test- & Validierungsstrategie

Jede Phase erfordert strikte Abnahmekriterien:

1. **Phase 5 (Simulator):** Das Replay muss exakt deterministisch sein (gleiche Datei erzeugt 100% gleichen Output-Stream).
2. **Phase 6 (SLAM):** Die generierte `.npy`-Karte darf bei Schleifenfahrten ("Loop Closure") keine doppelten Wände oder starken Verschmierungen (>10cm Drift) aufweisen.
3. **Phase 7 (Localization):** Der Roboter muss seine Position in 9 von 10 Fällen nach maximal einer 360-Grad-Drehung korrekt auf der Karte verorten können.
4. **Phase 8 (Navigation):** Anfahren von 5 unterschiedlichen Koordinaten ohne Kollision bei dynamisch in den Weg gestellten Hindernissen.
