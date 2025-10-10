#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Xbox Controller Input Script für Raspberry Pi
Zeigt alle Eingaben von einem Microsoft Xbox Wireless Controller an
Unterstützt sowohl Xbox 360 als auch Xbox One Controller mit USB Dongle

Abhängigkeiten:
- pygame: sudo apt install python3-pygame
- xboxdrv: sudo apt install xboxdrv

Verwendung:
1. Controller mit USB Dongle verbinden
2. sudo python3 xbox_controller_reader.py
"""

import pygame
import sys
import time
import threading
from typing import Optional, Dict, Any

class XboxControllerReader:
    """
    Klasse zum Lesen von Xbox Controller Eingaben auf Raspberry Pi
    """

    def __init__(self, controller_id: int = 0):
        """
        Initialisiert den Xbox Controller Reader

        Args:
            controller_id: ID des Controllers (normalerweise 0 für ersten Controller)
        """
        self.controller_id = controller_id
        self.joystick: Optional[pygame.joystick.Joystick] = None
        self.running = False
        self.last_values: Dict[str, Any] = {}

        # Button Mapping für Xbox Controller
        self.button_mapping = {
            0: 'A',
            1: 'B', 
            2: 'X',
            3: 'Y',
            4: 'LB (Left Bumper)',
            5: 'RB (Right Bumper)',
            6: 'Back/View',
            7: 'Start/Menu',
            8: 'Left Stick Click',
            9: 'Right Stick Click'
        }

        # Achsen Mapping
        self.axis_mapping = {
            0: 'Left Stick X',
            1: 'Left Stick Y', 
            2: 'Left Trigger',
            3: 'Right Stick X',
            4: 'Right Stick Y',
            5: 'Right Trigger'
        }

    def initialize(self) -> bool:
        """
        Initialisiert pygame und den Controller

        Returns:
            True wenn erfolgreich, False bei Fehler
        """
        try:
            pygame.init()
            pygame.joystick.init()

            # Prüfe ob Controller verfügbar ist
            if pygame.joystick.get_count() == 0:
                print("❌ Kein Controller gefunden!")
                print("Stelle sicher, dass:")
                print("1. Der Controller mit dem USB Dongle verbunden ist")
                print("2. xboxdrv installiert ist: sudo apt install xboxdrv")
                print("3. Das Script mit sudo ausgeführt wird")
                return False

            # Controller initialisieren
            self.joystick = pygame.joystick.Joystick(self.controller_id)
            self.joystick.init()

            print(f"✅ Controller gefunden: {self.joystick.get_name()}")
            print(f"   Buttons: {self.joystick.get_numbuttons()}")
            print(f"   Achsen: {self.joystick.get_numaxes()}")
            print(f"   D-Pads: {self.joystick.get_numhats()}")
            print()

            return True

        except Exception as e:
            print(f"❌ Fehler beim Initialisieren: {e}")
            return False

    def get_button_state(self, button_id: int) -> bool:
        """
        Gibt den Zustand eines Buttons zurück

        Args:
            button_id: ID des Buttons

        Returns:
            True wenn gedrückt, False wenn nicht gedrückt
        """
        if self.joystick and button_id < self.joystick.get_numbuttons():
            return bool(self.joystick.get_button(button_id))
        return False

    def get_axis_value(self, axis_id: int) -> float:
        """
        Gibt den Wert einer Achse zurück

        Args:
            axis_id: ID der Achse

        Returns:
            Achsenwert zwischen -1.0 und 1.0 (bei Triggern 0.0 bis 1.0)
        """
        if self.joystick and axis_id < self.joystick.get_numaxes():
            return self.joystick.get_axis(axis_id)
        return 0.0

    def get_dpad_state(self, hat_id: int = 0) -> tuple:
        """
        Gibt den Zustand des D-Pads zurück

        Args:
            hat_id: ID des D-Pads (normalerweise 0)

        Returns:
            Tuple (x, y) mit Werten -1, 0, oder 1
        """
        if self.joystick and hat_id < self.joystick.get_numhats():
            return self.joystick.get_hat(hat_id)
        return (0, 0)

    def print_current_state(self):
        """
        Gibt den aktuellen Zustand aller Controller-Eingaben aus
        """
        if not self.joystick:
            return

        # Events verarbeiten (wichtig für pygame)
        pygame.event.pump()

        changed = False
        current_values = {}

        # Button States
        button_states = []
        for i in range(self.joystick.get_numbuttons()):
            pressed = self.get_button_state(i)
            button_name = self.button_mapping.get(i, f'Button {i}')
            current_values[f'button_{i}'] = pressed

            if pressed:
                button_states.append(f"{button_name}")

        # Axis Values (nur bei Änderungen > 0.1)
        axis_states = []
        for i in range(self.joystick.get_numaxes()):
            value = self.get_axis_value(i)
            axis_name = self.axis_mapping.get(i, f'Axis {i}')
            current_values[f'axis_{i}'] = value

            # Deadzone für Analog Sticks
            deadzone = 0.1
            if abs(value) > deadzone:
                axis_states.append(f"{axis_name}: {value:.3f}")

        # D-Pad State
        dpad_x, dpad_y = self.get_dpad_state()
        current_values['dpad'] = (dpad_x, dpad_y)

        dpad_states = []
        if dpad_x != 0 or dpad_y != 0:
            direction = ""
            if dpad_y == 1: direction += "Oben "
            if dpad_y == -1: direction += "Unten "
            if dpad_x == 1: direction += "Rechts "
            if dpad_x == -1: direction += "Links "
            dpad_states.append(f"D-Pad: {direction.strip()}")

        # Prüfe auf Änderungen
        if current_values != self.last_values:
            changed = True
            self.last_values = current_values.copy()

        # Ausgabe nur bei Änderungen oder wenn Eingaben aktiv sind
        if changed and (button_states or axis_states or dpad_states):
            print("🎮 Controller Eingaben:")

            if button_states:
                print(f"   🔘 Buttons: {', '.join(button_states)}")

            if axis_states:
                print(f"   🕹️  Achsen: {', '.join(axis_states)}")

            if dpad_states:
                print(f"   ⬆️  {', '.join(dpad_states)}")

            print("-" * 50)

    def run_monitoring(self):
        """
        Startet das kontinuierliche Monitoring der Controller-Eingaben
        """
        if not self.initialize():
            return

        print("🎮 Xbox Controller Monitoring gestartet")
        print("Drücke Tasten auf dem Controller um Eingaben zu sehen")
        print("Drücke Ctrl+C zum Beenden")
        print("=" * 50)

        self.running = True

        try:
            while self.running:
                self.print_current_state()
                time.sleep(0.05)  # 20 Hz Update-Rate

        except KeyboardInterrupt:
            print("\n🛑 Monitoring beendet")

        finally:
            self.cleanup()

    def cleanup(self):
        """
        Räumt pygame Ressourcen auf
        """
        self.running = False
        if self.joystick:
            self.joystick.quit()
        pygame.quit()
        print("✅ Cleanup abgeschlossen")

def main():
    """
    Hauptfunktion des Scripts
    """
    print("🎮 Xbox Controller Reader für Raspberry Pi")
    print("=" * 50)

    controller = XboxControllerReader()
    controller.run_monitoring()

if __name__ == "__main__":
    main()
