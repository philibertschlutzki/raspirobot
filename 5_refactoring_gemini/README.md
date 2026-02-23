# RaspiRobot Refactoring (Phase 4-6)

Dies ist die Dokumentation für das refactorte RaspiRobot-System im Verzeichnis `5_refactoring_gemini/`. Das Ziel dieses Refactorings war die Umstellung von monolithischen Skripten auf eine modulare, testbare Architektur mit klaren Schnittstellen und robuster Datenvalidierung.

## 1. Projektübersicht

Die neue Architektur basiert auf folgenden Kernprinzipien:

*   **Dependency Injection (DI):** Alle Hardware-Komponenten (Motoren, Lidar, Ultraschall) werden über abstrakte Interfaces (`interfaces.py`) in den `RobotOrchestrator` injiziert. Dies ermöglicht einfaches Mocking für Tests.
*   **Pydantic V2:** Sämtliche Datenstrukturen (Posen, Sensor-Daten, Log-Einträge) sind als unveränderliche Pydantic-Modelle (`data_models.py`) definiert, was Typsicherheit und automatische Validierung garantiert.
*   **Orchestrator Pattern:** Der `RobotOrchestrator` (`robot_orchestrator.py`) fungiert als zentrale Steuereinheit, die Sensordaten fusioniert, die Odometrie aktualisiert und Steuerbefehle an die Motoren sendet. Er läuft in einer präzisen 50Hz-Schleife.
*   **Asynchrones Recording:** Sensordaten werden in einem separaten Thread gepuffert und effizient als komprimierte JSONL-Dateien (`.json.gz`) weggeschrieben, ohne den Regelkreis zu blockieren.

## 2. Setup & Abhängigkeiten

Das Projekt benötigt Python 3.9+. Die Abhängigkeiten sind in `requirements.txt` definiert.

Installation:
```bash
pip install -r ../requirements.txt
```

Wichtige Pakete:
*   `pydantic>=2.0`: Datenmodellierung
*   `rplidar-robotic`: Treiber für den RPLidar
*   `RPi.GPIO`: Hardware-Zugriff auf Raspberry Pi Pins
*   `pygame`: Einlesen des Xbox-Controllers
*   `numpy`, `scipy`: Mathematische Berechnungen für SLAM/Odometrie

## 3. Hardware-Anforderungen

Das System ist für folgende Hardware-Konfiguration ausgelegt:

*   **Computer:** Raspberry Pi 5
*   **LIDAR:** RPLidar C1 (via USB/UART)
*   **Sensoren:** 2x HC-SR04 Ultraschall-Sensoren (Links/Rechts) zur Nahbereichsabsicherung (< 15cm).
*   **Eingabegerät:** Xbox Wireless Controller (via Bluetooth/xpadneo).
*   **Antrieb:** 2x DC-Motoren mit PWM-Ansteuerung über H-Brücke.
*   **Stromversorgung:** Adäquate LiPo-Versorgung für Motoren und Pi.

## 4. Ausführung auf dem Roboter

Um den Roboter im Produktionsmodus zu starten, sind Root-Rechte (für GPIO/PWM) erforderlich.

```bash
cd 5_refactoring_gemini/
sudo python3 robot_orchestrator.py
```

Der Orchestrator initialisiert automatisch alle Hardware-Treiber. Stellen Sie sicher, dass der Xbox-Controller verbunden ist, bevor Sie das Skript starten.

**Steuerung:**
*   **Start/Select:** Beendet das Programm.
*   **X + Y:** Startet/Stoppt die Aufnahme (Recording).
*   **B:** Wechselt die Fahrtrichtung (Vorwärts/Rückwärts).
*   **Trigger (L/R):** Gas geben (Differential Steering).

## 5. PC-Test-Modus (Hardware-Bypass)

Für die Entwicklung auf einem Laptop oder in CI/CD-Umgebungen existiert ein `PC_TEST_MODE`. Dieser simuliert alle Hardware-Schnittstellen (Mocking).

Aktivierung:
```bash
export PC_TEST_MODE=1
python3 robot_orchestrator.py
```

In diesem Modus:
*   Werden keine GPIOs initialisiert.
*   Liefert der Lidar-Treiber leere oder simulierte Scans.
*   Werden Motorbefehle nur geloggt, nicht ausgeführt.
*   Wird `pygame` mit dem `dummy` Videotreiber initialisiert, falls kein Display vorhanden ist.

**CI/CD Pipeline:**
Unsere GitHub Actions Pipeline nutzt diesen Modus, um Unit-Tests (`pytest`) auszuführen, ohne dass Hardware angeschlossen sein muss.

## 6. Logging & Validation

Das System verwendet ein strukturiertes JSONL-Logging (`logger.py`). Alle Ereignisse (Start, Stopp, Fehler, Sensor-Warnungen) werden in eine Log-Datei geschrieben.

Validierung der Logs:
Ein separates Tool (`../tests/log_validator.py`) kann genutzt werden, um sicherzustellen, dass die Logs dem definierten Schema entsprechen und keine kritischen Fehler übersehen wurden.

```bash
python3 ../tests/log_validator.py --file <logfile.jsonl>
```

## 7. Recording & SLAM (Ausblick)

Während der Fahrt aufgezeichnete Daten werden als `.json.gz` gespeichert. Diese Dateien enthalten:
*   Zeitstempel & Hardware-Metadaten.
*   Rohe LIDAR-Scans.
*   Controller-Eingaben und Odometrie-Schätzungen.

**Offline-Simulator:**
Diese Aufnahmen können mit `simulator_replay.py` (oder ähnlichen Tools im Verzeichnis) abgespielt werden, um Algorithmen (wie SLAM oder Pfadplanung) offline zu testen und zu optimieren, ohne den Roboter physisch bewegen zu müssen.
