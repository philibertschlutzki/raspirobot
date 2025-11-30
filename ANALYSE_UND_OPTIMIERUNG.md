# Analyse und Optimierung der Pfadaufzeichnung und Wiedergabe

## 1. Analyse der Pfadaufzeichnungen

Die Analyse der vorhandenen Aufzeichnungen im Ordner `path_recordings/` zeigt unterschiedliche Datenstrukturen, was auf verschiedene Versionen oder Konfigurationen des Aufzeichnungssystems hindeutet.

### Befunde

*   **Inkonsistenz der Daten:**
    *   Einige Aufzeichnungen (z.B. `...203612...`) enthalten vollständige `lidar_frames` und Metadaten.
    *   Andere Aufzeichnungen (z.B. `...204615...`) enthalten **keine** `lidar_frames` und keine vorberechneten `pose`-Daten in den `controller_samples`.
*   **Abhängigkeit von Odometrie:**
    *   Wenn keine LIDAR-Daten vorhanden sind, ist das Replay-Script blind für die Umgebung und verlässt sich rein auf "Dead Reckoning" (Replay der Motorbefehle). Dies führt ohne Feedback-Schleife schnell zu Abweichungen.
*   **Fehlende Validierung:**
    *   Das aktuelle System prüft beim Laden nicht strikt, ob alle für den gewünschten Modus (z.B. "Smart Replay") notwendigen Daten vorhanden sind.

### Optimierungsvorschläge für die Aufzeichnung (Recorder)

1.  **Erzwungene Schemata:** Der Recorder sollte Metadaten über die *Fähigkeiten* der Aufzeichnung speichern (z.B. `has_lidar: true`, `has_pose: true`).
2.  **LIDAR-Frequenz:** Für robustes Scan-Matching sollte die LIDAR-Aufzeichnungsfrequenz geprüft werden (aktuell ca. 10Hz, was gut ist).
3.  **Pose-Estimation während der Aufnahme:** Es ist vorteilhaft, wenn der Recorder bereits eine Pose schätzt und speichert, anstatt dies nur dem Replay-Skript zu überlassen.

## 2. Detailliertes Konzept für optimierte Aufzeichnung (Umgesetzt in v3.0.2)

Basierend auf der Analyse wurden in Version 3.0.2 des Recording-Skripts folgende Maßnahmen implementiert, um die Datenqualität für autonome Wiedergabe zu maximieren:

### A. Metadaten-Erweiterung ("Capabilities")
Um Inkonsistenzen zu vermeiden, schreibt der Recorder nun explizite `capabilities` in den Header jeder Datei.

*   **Implementierung:** Das `hardware_info`-Objekt wurde um ein `capabilities`-Dictionary erweitert:
    ```json
    "capabilities": {
        "has_lidar": true,
        "has_pose": true,
        "has_ultrasonic": true
    }
    ```
*   **Nutzen:** Das Replay-Skript (`path_replay_full_v3.py`) kann beim Laden sofort entscheiden, welcher Modus (LIDAR-Matching, reine Odometrie oder Hybrid) verwendet werden soll, ohne die Daten erst scannen zu müssen.

### B. Integrierte Pose-Estimation (Odometrie)
Anstatt die Roboter-Pose erst nachträglich beim Replay zu schätzen, berechnet der Recorder die Pose nun in Echtzeit während der Fahrt.

*   **Vorteil:** Dies entlastet das Replay-Skript und stellt sicher, dass die "Ground Truth" (sofern Odometrie als solche gelten kann) direkt mit den Sensor-Daten synchronisiert ist.
*   **Synchronisation:** Die Pose (`x`, `y`, `theta`) wird exakt zum Zeitpunkt des Controller-Samplings gespeichert und als Teil des `ControllerSample`-Objekts abgelegt.

### C. LIDAR-Frequenz-Monitoring
Eine stabile Scan-Rate ist kritisch für SLAM und Scan-Matching.

*   **Mechanismus:** Ein neuer Algorithmus im `lidar_update_thread` berechnet die gleitende Durchschnittsfrequenz der eintreffenden Scans.
*   **Feedback:** Das System warnt den Benutzer über die Konsole, wenn die Frequenz signifikant von den gewünschten 10Hz abweicht (z.B. durch USB-Latenzen oder CPU-Last). Dies verhindert unbemerktes "Low-Quality-Recording".

## 3. Fehlerhandhabung und Robustheit (für v3 Script)

Für `path_replay_full_v3.py` werden folgende Mechanismen implementiert:

*   **Graceful Degradation:**
    *   LIDAR fällt aus -> Fallback auf Odometrie (Warnung).
    *   Odometrie wird unplausibel -> Nothalt.
*   **Recovery nach Ausweichen:**
    *   Nach einer Kollisionsvermeidung (z.B. Bremsen/Ausweichen) merkt sich das System den Index des aktuellen Samples.
    *   Ein Path-Planner (z.B. Pure Pursuit Logik) versucht, den Roboter zurück auf die Trajektorie des nächsten Samples zu lenken, anstatt stur die alten Befehle abzuspielen.
*   **Input-Validierung:**
    *   Prüfung der JSON-Struktur vor dem Start.
    *   Warnung bei fehlenden Sensordaten in der Aufzeichnung.
