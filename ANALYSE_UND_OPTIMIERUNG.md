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

## 2. Konzept für virtuelles Testvorgehen

Da Tests auf der echten Hardware zeitaufwendig und riskant sind, wird ein Mocking-Ansatz empfohlen.

### Ansatz: Hardware-Abstraktions-Layer (HAL) Mocking

Anstatt die Hardware direkt anzusprechen, sollte `path_replay_full_v3.py` Hardware-Klassen verwenden, die sich "mocken" lassen.

**Komponenten:**

1.  **VirtualLidar:**
    *   Lädt eine echte Aufzeichnung (`lidar_frames`).
    *   Gibt im Test-Modus basierend auf der aktuellen "virtuellen Zeit" oder "virtuellen Position" den passenden Scan zurück.
    *   Kann Rauschen (Noise) hinzufügen, um Robustheit zu testen.

2.  **VirtualMotorController:**
    *   Akzeptiert PWM-Befehle.
    *   Simuliert ein einfaches kinematisches Modell (Differential Drive), um die virtuelle Position (`robot_x`, `robot_y`, `theta`) zu aktualisieren.

3.  **VirtualEnvironment (Collision Simulation):**
    *   Definiert virtuelle Hindernisse.
    *   Wenn die virtuelle Position einem Hindernis zu nahe kommt, melden die virtuellen Ultraschallsensoren oder das virtuelle LIDAR entsprechend kurze Distanzen.

### Test-Szenarien

1.  **Happy Path:** Replay einer Aufzeichnung in einer perfekten Welt (kein Schlupf). Ziel: Endposition == Zielposition.
2.  **Sensor Failure:** Mitten im Replay liefert das LIDAR `None` oder Fehler. Das Script darf nicht abstürzen.
3.  **Obstacle Avoidance:** Ein virtuelles Hindernis wird auf dem Pfad platziert. Das Script muss stoppen oder ausweichen und dann versuchen, zum Pfad zurückzukehren.

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
