# RaspiRobot 🤖

Ein vollständiges Robotersystem basierend auf **Raspberry Pi 5** mit LIDAR-Mapping, Xbox Controller-Steuerung und intelligenter Sensorfusion für autonome Navigation und Pfadaufzeichnung.

## ✨ Features

- **LIDAR-basiertes Mapping** mit RPLIDAR C1 für 360° Umgebungserfassung
- **Xbox Controller Integration** für manuelle Steuerung mit Wireless Support
- **Intelligente Sensorfusion** kombiniert LIDAR- und Ultraschall-Sensordaten
- **Autonome Navigation** mit Hinderniserkennung und Kollisionsvermeidung
- **Robustes Pfadaufzeichnung & Replay System** mit Recovery-Strategien
- **Echtzeitdatenanalyse** mit Performance-Monitoring
- **Modulare Architektur** für einfache Erweiterungen

## 🛠️ Hardware-Komponenten

### Hauptkomponenten

- **Raspberry Pi 5 (8GB)** - Zentrale Steuereinheit
- **RPLIDAR C1** - 360° Laser-Distanzsensor
- **4x HC-SR04** - Ultraschall-Sensoren (Front, Back, Links, Rechts)
- **2x ZS-X11H Motor Controller** - Differentialantrieb
- **Xbox Wireless Controller** - Manuelle Steuerung
- **RP03D Mikrowellensensor** - Bewegungserkennung

### GPIO-Pin-Belegung

| Komponente | Pin | Funktion |
| :-- | :-- | :-- |
| Motor Links PWM | 18 | PWM-Signal linker Motor |
| Motor Links Direction | 27 | Richtungssteuerung links |
| Motor Links Enable | 22 | Enable-Signal links |
| Motor Rechts PWM | 13 | PWM-Signal rechter Motor |
| Motor Rechts Direction | 19 | Richtungssteuerung rechts |
| Motor Rechts Enable | 26 | Enable-Signal rechts |
| Ultraschall Front Trigger | 23 | Trigger-Signal vorne |
| Ultraschall Front Echo | 24 | Echo-Signal vorne |
| Ultraschall Rechts Trigger | 25 | Trigger-Signal rechts |
| Ultraschall Rechts Echo | 8 | Echo-Signal rechts |
| Ultraschall Back Trigger | 7 | Trigger-Signal hinten |
| Ultraschall Back Echo | 12 | Echo-Signal hinten |
| Ultraschall Links Trigger | 16 | Trigger-Signal links |
| Ultraschall Links Echo | 20 | Echo-Signal links |
| Mikrowellensensor | 21 | Bewegungsdetektion |

## 🚀 Installation

### Voraussetzungen

```bash
# System-Updates
sudo apt update && sudo apt upgrade -y

# Python-Abhängigkeiten
sudo apt install python3-pip python3-venv git -y

# Hardware-Bibliotheken
sudo apt install python3-rpi.gpio python3-serial -y
```

### Repository klonen

```bash
git clone https://github.com/philibertschlutzki/raspirobot.git
cd raspirobot
```

### Python-Umgebung einrichten

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Hardware-Konfiguration

```bash
# GPIO-Berechtigung für Benutzer
sudo usermod -a -G gpio $USER

# Serial-Interface aktivieren
sudo raspi-config
# Interface Options > Serial Port > Enable
```

## 🎮 Verwendung

### 1. Robustes Path Replay (Empfohlen)

Das neue v3-System bietet verbesserte Fehlerbehandlung und Recovery-Funktionen.

```bash
cd 4_path_follower
sudo python3 path_replay_full_v3.py path_recordings/recording_name.json.gz
```

**Features:**
- **Adaptives Replay:** Funktioniert auch ohne LIDAR-Daten oder Pose-Informationen (Fallback auf Odometrie).
- **Collision Recovery:** Weicht Hindernissen aus und steuert automatisch zurück auf den ursprünglichen Pfad.
- **Hardware-Abstraktion:** Robustes Exception Handling verhindert Abstürze bei Sensor-Problemen.

### 2. LIDAR-Controller mit Sensorfusion

```bash
cd 3_Lidarmapper
python3 lidar_controller_fusion.py
```

### 3. Xbox Controller mit Aufzeichnung

```bash
cd 3_Lidarmapper
python3 xbox_controller_with_recording_v2.0.py
```

### 4. Pfadaufzeichnungssystem

```bash
cd 1_storage
python3 path_recording_system.py
```

## 📁 Projektstruktur

```
raspirobot/
├── 1_storage/                    # Datenspeicherung und -verwaltung
│   ├── storage_system.py         # Hauptsystem für Datenspeicherung
│   └── path_recording_system.py  # Pfadaufzeichnung
├── 2_recorder/                   # Aufzeichnungssysteme
├── 3_Lidarmapper/               # LIDAR-Integration
│   ├── lidar_controller_fusion.py           # Hauptcontroller mit Sensorfusion
│   └── xbox_controller_with_recording_v2.0.py # Xbox-Steuerung
├── 4_path_follower/             # Wiedergabe-Systeme
│   ├── path_replay_full_v2.py   # Legacy (LIDAR only)
│   └── path_replay_full_v3.py   # Robustes Replay System (Empfohlen)
├── tests/                       # Allgemeine Tests
└── README.md                    # Projektdokumentation
```

## 🔧 Technische Details

### Sensorfusion-Algorithmus

Das System kombiniert LIDAR-Daten (360° Scan) mit Ultraschall-Sensoren für maximale Präzision.

### Recovery-Strategie (v3)

Das v3-System verwendet eine State-Machine für robustes Verhalten:
1. **REPLAY:** Normales Abfahren der aufgezeichneten Steuerbefehle.
2. **AVOID:** Bei Hindernis (<20cm) wird gestoppt oder ausgewichen.
3. **RECOVER:** Sobald der Weg frei ist, berechnet ein Path-Planner (Pure Pursuit) den Weg zum nächsten Punkt auf der Original-Trajektorie.
4. **RESUME:** Wenn der Roboter wieder auf dem Pfad ist, wird das Replay fortgesetzt.

## 🧪 Testing

### Unit Tests ausführen

```bash
cd 1_storage
python3 1_storage_unit_test.py
```

### Simulation / Mocking

Das `path_replay_full_v3.py` Skript erkennt automatisch fehlende Hardware-Bibliotheken und wechselt in einen eingeschränkten Modus, was grundlegende Tests auch ohne Roboter ermöglicht (siehe `ANALYSE_UND_OPTIMIERUNG.md`).

## 📊 Performance-Monitoring

Das System bietet detaillierte Performance-Metriken:
- **LIDAR Scan Rate:** ~10 Hz
- **Sensor Fusion Rate:** ~20 Hz
- **Motor Update Rate:** ~50 Hz

## 🔒 Sicherheitsfeatures

- **Notaus-Funktion** über Xbox Controller (Start + Back)
- **Automatische Kollisionsvermeidung** bei < 30cm Hindernisabstand
- **Watchdog-Timer** für System-Überwachung
- **Graceful Shutdown** bei kritischen Fehlern
- **Sensor-Redundanz** durch LIDAR + Ultraschall

## 🤝 Contributing

1. Fork des Repositories
2. Feature-Branch erstellen (`git checkout -b feature/AmazingFeature`)
3. Änderungen committen (`git commit -m 'Add some AmazingFeature'`)
4. Branch pushen (`git push origin feature/AmazingFeature`)
5. Pull Request öffnen

## 📝 Lizenz

Dieses Projekt steht unter der MIT-Lizenz. Siehe `LICENSE` Datei für Details.

## 📞 Support

Bei Fragen oder Problemen:
- **Issues:** [GitHub Issues](https://github.com/philibertschlutzki/raspirobot/issues)

***

**Status:** 🟢 Aktiv entwickelt | **Version:** 3.0 | **Python:** 3.9+
