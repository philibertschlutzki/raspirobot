#!/usr/bin/env python3
"""Fährt einen mit run_path_recording.py aufgezeichneten Weg autonom ab.

    sudo <venv>/bin/python run_path_follow.py                      # neuester Weg
    sudo <venv>/bin/python run_path_follow.py path_recordings/<session>_path.json.gz
    sudo <venv>/bin/python run_path_follow.py --modus eingaben path_recordings/<session>.json.gz

Der Roboter fährt erst nach Druck auf A los. B oder Start: Not-Aus und Beenden.
Muss als root laufen (GPIO/PWM).
"""

import argparse
import os
import sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Aufgezeichneten Weg autonom abfahren")
    parser.add_argument("datei", nargs="?", help="Wegdatei *_path.json.gz (Modus weg) oder Aufnahme *.json.gz (Modus eingaben)")
    parser.add_argument("--modus", choices=["weg", "eingaben"], default="weg",
                        help="weg: Regelung auf die Wegpunkte (Standard); eingaben: Controller-Eingaben abspielen")
    args = parser.parse_args()

    if os.geteuid() != 0:
        print("❌ [ERROR] Dieses Script muss mit sudo ausgeführt werden!")
        print(f"📝 Verwendung: sudo {sys.executable} {os.path.abspath(__file__)} [datei]")
        sys.exit(1)

    from raspirobot import config
    from raspirobot.path.follow_app import PathFollowApp
    from raspirobot.path.waypoints import find_latest_path

    datei = args.datei
    if datei is None:
        if args.modus != "weg":
            parser.error("Im Modus eingaben muss eine Aufnahme angegeben werden")
        latest = find_latest_path(config.RECORDING_BASE_DIR)
        if latest is None:
            print(f"❌ [ERROR] Keine Wegdatei in {config.RECORDING_BASE_DIR}/ gefunden")
            sys.exit(1)
        datei = str(latest)

    app = PathFollowApp(datei, mode=args.modus)
    app.run()
    sys.exit(0 if app.state == "FERTIG" else 1)
