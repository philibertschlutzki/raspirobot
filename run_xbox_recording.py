#!/usr/bin/env python3
"""Startet die manuelle Fahrt mit Aufzeichnung. Muss als root laufen (GPIO/PWM).

    sudo <venv>/bin/python run_xbox_recording.py
"""

import os
import sys

if __name__ == "__main__":
    if os.geteuid() != 0:
        print("❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!")
        print(f"📝 Verwendung: sudo {sys.executable} {os.path.abspath(__file__)}")
        sys.exit(1)

    from raspirobot.app import RobotApp

    RobotApp().run()
