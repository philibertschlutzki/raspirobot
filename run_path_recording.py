#!/usr/bin/env python3
"""Manuelle Fahrt wie run_xbox_recording.py, jede Aufnahme speichert zusätzlich den Weg.

    sudo <venv>/bin/python run_path_recording.py

Abfahren danach mit run_path_follow.py. Muss als root laufen (GPIO/PWM).
"""

import os
import sys

if __name__ == "__main__":
    if os.geteuid() != 0:
        print("❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!")
        print(f"📝 Verwendung: sudo {sys.executable} {os.path.abspath(__file__)}")
        sys.exit(1)

    from raspirobot.path.teach import TeachApp

    TeachApp().run()
