# RaspiRobot 🤖

Eine fortschrittliche Entwicklungsplattform für autonome Indoor-Navigation basierend auf einem **Raspberry Pi 5**. Das Projekt fokussiert sich auf die Entwicklung einer nativen, vollständig in Python geschriebenen SLAM- und Navigations-Engine – ganz ohne Rückgriff auf fertige Frameworks wie ROS oder Rad-Encoder (Odometrie).

Das System nutzt fortschrittliche Sensorfusion (RPLIDAR C1 + Ultraschall) und KI-gestützte Algorithmen-Entwicklung, um das "Kidnapped Robot Problem" zu lösen und Point-to-Point Navigation in unbekannten Umgebungen zu ermöglichen.

## ✨ Kern-Vision & Features

- **Native Python SLAM-Engine:** Eigenentwickeltes Simultaneous Localization and Mapping (basierend auf Numpy/ICP), das Map-Drift ausschließlich über LIDAR-Scan-Matching korrigiert.
- **Global Localization (Kidnapped Robot):** Der Roboter kann an einem beliebigen Punkt in der zuvor kartografierten Wohnung abgesetzt werden und findet seine Position selbstständig (z.B. via Monte Carlo Localization / Partikelfilter).
- **Point-to-Point Navigation:** Zieleingabe über CLI (z.B. `--x 2.5 --y 1.0`). Eine Kombination aus globalem Planer (A*/Dijkstra) und lokalem Planer sorgt für die sichere, hindernisvermeidende Fahrt zum Ziel.
- **Offline-Simulation & Data Replay:** Ein integrierter Simulator ermöglicht es, auf dem RPi aufgezeichnete Sensor-Datensätze (`.json.gz`) am PC abzuspielen, um Navigations- und SLAM-Algorithmen rasant iterieren zu können.
- **Hardware-Abstraktion & Sensorfusion:** Robuste Multithreading-Architektur, die Ultraschall-Nahbereichssensorik (<30cm) mit 360°-LIDAR-Daten fusioniert.

## 🛠️ Hardware-Komponenten

- **Zentrale Steuereinheit:** Raspberry Pi 5 (8GB)
- **Hauptsensor:** RPLIDAR C1 (360° Laser-Distanzsensor)
- **Nahbereichssensorik:** 4x HC-SR04 Ultraschall-Sensoren (Front, Back, Links, Rechts)
- **Antrieb:** 2x DC-Motoren mit ZS-X11H Controllern (Differentialantrieb, PWM-gesteuert)
- **Manuelle Kontrolle:** Xbox Wireless Controller (für Mapping-Fahrten)

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

```bash
# System-Updates
sudo apt update && sudo apt upgrade -y

# Python-Abhängigkeiten & Hardware-Bibliotheken
sudo apt install python3-pip python3-venv git python3-rpi.gpio python3-serial -y

# Repository klonen
git clone https://github.com/philibertschlutzki/raspirobot.git
cd raspirobot

# Python-Umgebung einrichten
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 🎮 Workflow & Verwendung

Das Projekt befindet sich in aktiver Entwicklung. Der aktuelle Workflow teilt sich in Datenerfassung, Offline-Entwicklung und (zukünftig) Autonomie.

### 1. Mapping & Datenerfassung (Aktueller Fokus)

Fahre den Roboter manuell durch die Umgebung, um hochwertige LIDAR-Daten für die Offline-SLAM-Entwicklung aufzuzeichnen.

```bash
cd 3_Lidarmapper
sudo python3 xbox_controller_with_recording_v3.0.2.py
```

### 2. Offline-Simulator (In Entwicklung)

Spiele die aufgezeichneten `.json.gz`-Dateien am Entwicklungs-PC ab, um die Python-SLAM-Algorithmen ohne reale Roboter-Hardware zu trainieren und zu testen.

```bash
cd 5_simulation
python3 data_player.py --record path_recordings/mein_datensatz.json.gz
```

### 3. Autonome Navigation (Roadmap)

Startet die Lokalisierung und fährt autonom zum definierten Ziel.

```bash
cd 6_autonomy
python3 navigate.py --x 2.5 --y 1.0
```

## 🗺️ Projekt-Roadmap

Für eine detaillierte technische Übersicht der anstehenden Entwicklungsphasen (Offline-Simulator -> SLAM -> Lokalisierung -> Path Planning), siehe die [ANALYSE_UND_OPTIMIERUNG.md](ANALYSE_UND_OPTIMIERUNG.md).

## 🔒 Sicherheitsfeatures & Robustheit

* Hardware-Notaus über Controller.
* "Graceful Degradation": Das System warnt bei abfallender LIDAR-Frequenz und stoppt bei Sensorverlust.
* Dynamische Hindernisvermeidung durch gekoppelte Ultraschall/LIDAR-Kollisionszonen.

## 🤝 Contributing & KI-Entwicklung

Dieses Projekt wird maßgeblich durch die Zusammenarbeit mit fortschrittlichen KI-Coding-Agenten entwickelt. Fokus liegt auf sauberem, modularem Python-Code und algorithmischem Verständnis statt auf Blackbox-Frameworks.

**Lizenz:** MIT License
