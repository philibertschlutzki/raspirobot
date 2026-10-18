# RaspiRobot

Manuelle Fahrt eines Roboters mit Differentialantrieb per Xbox-Controller, mit
Kollisionsschutz aus zwei Ultraschallsensoren und einem RPLIDAR sowie
Aufzeichnung von Controller-Eingaben und LIDAR-Scans als `.json.gz`.

Der Code ist eine Umstrukturierung von
`3_Lidarmapper/xbox_controller_with_recording_v3.0.1.py` (im Git-Verlauf bis
Commit `8bee9fa`). Steuerlogik, Schwellwerte, Pins und Dateiformat sind
unverändert; die Gleichheit von Motor-Ausgaben, Kollisionsbewertung,
Odometrie und Recording-Header wurde gegen die Originaldatei geprüft.

## Start

Das Programm bricht ab, wenn es nicht als root läuft.

```bash
cd ~/raspirobot
sudo /home/johndoe/.venv/bin/python run_xbox_recording.py
```

Aufnahmen landen in `path_recordings/` relativ zum aktuellen
Arbeitsverzeichnis (beim Aufruf oben also `~/raspirobot/path_recordings/`).

## Bedienung

| Eingabe | Wirkung |
| :-- | :-- |
| Linker / rechter Trigger | Duty Cycle linker / rechter Motor, 0 bis 12 % |
| A halten | Tank-Turn: der stärker gedrückte Trigger gibt Drehrichtung und Duty Cycle vor, Motoren gegenläufig |
| B | Fahrtrichtung vorwärts ⇄ rückwärts |
| X + Y gleichzeitig | Aufnahme starten / stoppen |
| Start | Programm beenden; eine laufende Aufnahme wird unabhängig von ihrer Dauer gespeichert |
| Strg+C | Programm beenden; eine laufende Aufnahme ab 5 s wird gespeichert |

Aufnahmen unter 5 s werden beim Stoppen per X + Y verworfen.

## Hardware laut Code

Pin-Nummern sind BCM-Nummern, alle Werte stehen in `raspirobot/config.py`.

| Funktion | Konfiguration |
| :-- | :-- |
| Ultraschall links (HC-SR04) | Trigger GPIO12, Echo GPIO13 |
| Ultraschall rechts (HC-SR04) | Trigger GPIO23, Echo GPIO24 |
| Motor-Richtung links / rechts | GPIO20 / GPIO21, HIGH = vorwärts |
| Motor-PWM links / rechts | `pwmchip0`, Kanal 0 / 1, 1000 Hz |
| Status-LED | GPIO22 (aktiv) |
| Buzzer | GPIO25 (im Code deaktiviert) |
| LIDAR | `/dev/ttyUSB0`, 460800 Baud |
| Controller | pygame-Joystick 0; Trigger-Achsen 2 / 5; Tasten A=0, B=1, X=2, Y=3, Start=7 |

Welche GPIO-Pins hinter PWM-Kanal 0 und 1 liegen, legt das Device-Tree-Overlay
des Pi fest und ergibt sich nicht aus dem Code.

## Funktionsweise

Threads: Ultraschall (Zieltakt 10 Hz), LIDAR (ein Durchlauf je Scan), Localization
(neuester Scan), Pose (50 Hz), LED (20 Hz). Der Steuerungs-Loop läuft im Hauptthread mit 50 Hz und gibt
einmal pro Sekunde eine Statuszeile aus.

**Kollisionsschutz** (`raspirobot/collision.py`, `raspirobot/drive.py`):

- Ultraschall unter 30 cm oder LIDAR unter 0,30 m: beide Duty Cycles halbiert.
- Ultraschall unter 15 cm oder LIDAR unter 0,15 m: beide Motoren auf 0.
- Der LIDAR wird nur im Winkelbereich −45° bis +45° ausgewertet, roh wie vom
  Sensor geliefert.

**Odometrie** (`raspirobot/odometry.py`): Koppelnavigation aus den gesetzten
Duty Cycles (keine Rad-Encoder). Annahme 100 % Duty = 0,5 m/s, Radabstand
0,20 m. Die Pose wird beim Start einer Aufnahme auf (0, 0, 0) gesetzt.

**Positionsbestimmung** (`raspirobot/localization/`): Die Odometrie liefert die
Vorhersage, ein Abgleich jedes LIDAR-Scans mit einer selbst aufgebauten
Belegungskarte (2,5 cm Auflösung, 40 × 40 m, Start in der Mitte) korrigiert sie.
Der Rahmen der Karte ist die Pose beim ersten gültigen Scan (0, 0, 0); sie wird
nicht mit der Aufnahme zurückgesetzt. Die Statuszeile zeigt
`SLAM[Status Wertung]:X Y θ`:

- `OK`: Match ist gut (Wertung ≥ 0,60), Pose und Karte wurden aktualisiert.
- `DEGRADED`: Pose stammt aus der Odometrie, weil der Scan zu wenige Punkte hatte,
  nicht zur Karte passte oder die Verschiebung nicht eindeutig ist (Flur ohne
  Merkmale). In einem solchen Flur driftet die Position entlang des Flurs wie die Odometrie.
- `LOST`: fünf Fehlmatches in Folge; der Roboter sucht jeden Scan global
  (±0,8 m, alle Winkel) und übernimmt erst ab Wertung 0,75.

Beim Beenden werden `path_recordings/maps/map_<zeit>.npz` (Log-Odds) und `.pgm`
(Bild) geschrieben. Ohne Encoder bleibt die Genauigkeit begrenzt: gegen eine
Kartenänderung durch bewegte Hindernisse schützt nur die periodische
Neuberechnung des Likelihood-Felds; Schleifenschluss gibt es nicht.

**Aufnahme** (`raspirobot/recorder.py`, `raspirobot/recording/`): Pro
Steuerzyklus ein Controller-Sample (Trigger, Tasten, Richtung) und bis zu fünf
gepufferte LIDAR-Scans `(quality, angle_deg, distance_mm)`. Die Datei ist
gzip-komprimiertes JSON mit SHA256-Prüfsumme (Feld `_checksum`), Formatversion
1.1.0. Laden:

```python
from raspirobot.recording.storage import PathRecordingStorageManager
rec = PathRecordingStorageManager("path_recordings").load_json("path_recordings/<datei>.json.gz")
```

## Bekannte Einschränkungen (aus dem Code abgeleitet, bewusst nicht geändert)

1. **LIDAR-Sektor:** Der RPLIDAR liefert Winkel von 0° bis 360°. Die Prüfung
   `−45° ≤ Winkel ≤ 45°` erfasst daher nur 0° bis 45°; der Bereich 315° bis
   360° wird nicht ausgewertet. Ob 0° zur Fahrzeugfront zeigt, hängt vom Einbau ab.
2. **Die Pose (Odometrie wie SLAM) wird nicht aufgezeichnet**, obwohl der Header
   `pose_tracking: true` enthält. Für das Nachfahren eines Pfads fehlen damit
   Positionsdaten in den Aufnahmen.
3. **Odometrie beim Tank-Turn:** Die Richtungspins gehen nicht ein. Beim
   Drehen auf der Stelle (beide Duty Cycles gleich) berechnet die Odometrie
   eine Geradeausfahrt.
4. **Ultraschall-Ausfall:** Ungültige Messungen behalten den letzten Wert; es
   gibt keine Erkennung veralteter Werte.
5. **Metadaten ohne Wirkung:** `deadzone: 0.05` und die LIDAR-Offsets
   (`LIDAR_OFFSET_X = 0.15`) werden nur in den Header geschrieben, im Code
   aber nirgends angewendet.
6. Ein Strg+C führt das Aufräumen zweimal aus (Signal-Handler und `finally`);
   dabei können Fehlermeldungen beim zweiten LIDAR-Stopp erscheinen.

## Beispielaufnahmen

`path_recordings/` enthält drei Aufnahmen vom 27.11.2025:

| Datei | Erzeugt von | LIDAR | Controller-Samples | LIDAR-Frames |
| :-- | :-- | :-- | --: | --: |
| `20251127_203612_robot01.json.gz` | v3.0 | verbunden | 1913 | 171 |
| `20251127_204615_robot01.json.gz` | v3.0.1 | nicht verbunden | 694 | 0 |
| `20251127_204739_robot01.json.gz` | v3.0.1 | nicht verbunden | 1250 | 0 |

## LIDAR prüfen

```bash
<venv>/bin/python -m raspirobot.lidar_check      # bewegt keine Motoren
```

Zeigt Scanrate und die nächste Distanz vorn/links/hinten/rechts im Roboter-Rahmen.
Hand links vor den Sensor halten: "links" muss die kleinste Distanz zeigen. Sonst
`LIDAR_ANGLE_SIGN` oder `LIDAR_ANGLE_OFFSET` in `config.py` anpassen. Dieselben
Werte steuern die Positionsbestimmung; falsche Vorzeichen spiegeln die Karte.

Der RPLIDAR C1 bekommt kein PWM-Kommando mehr (`C1Lidar` in `hardware/lidar.py`):
Die Bibliothek schickt es beim Scan-Start, der C1 verschluckt dann die nächste
Antwort und der Scan bricht mit `Descriptor length mismatch` ab. Abgebrochene
Scans werden bis zu `LIDAR_MAX_RESTARTS` Mal neu gestartet.

## Tests

Die Tests laufen ohne Raspberry Pi; `tests/fakes.py` ersetzt GPIO, PWM,
pygame und rplidar.

```bash
pip install -r requirements-dev.txt
pytest tests/
```

## Struktur

```
run_xbox_recording.py        Einstieg (root-Prüfung, startet RobotApp)
raspirobot/
  config.py                  alle Konstanten
  app.py                     Setup, Threads, Steuerungs-Loop, Cleanup
  drive.py                   Trigger → Duty Cycle, Tank-Turn, Kollisionsbegrenzung
  collision.py               Bewertung Ultraschall + LIDAR
  odometry.py                Pose aus Duty Cycles
  localization/              Scan-Matching gegen Belegungskarte (geometry, grid_map, matcher, localizer)
  lidar_check.py             LIDAR-Prüfung ohne Motoren
  recorder.py                Start/Stopp/Samples einer Aufnahme
  recording/data_format.py   Datenklassen des Dateiformats
  recording/storage.py       Speichern/Laden mit Prüfsumme und Backups
  hardware/                  ultrasonic, motors, lidar, xbox_controller, status_led
tests/                       pytest, ohne Hardware
```
