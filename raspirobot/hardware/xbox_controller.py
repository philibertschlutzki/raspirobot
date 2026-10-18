"""Xbox-Controller über pygame."""

from dataclasses import dataclass

import pygame

from .. import config


@dataclass
class ControllerInput:
    """Zustand nach einem Lesevorgang.

    Trigger sind pygame-Rohwerte (-1.0 losgelassen bis 1.0 voll gedrückt).
    Die ``*_requested``-Felder sind nur in dem Zyklus True, in dem das
    Ereignis erkannt wurde.
    """

    trigger_left: float = -1.0
    trigger_right: float = -1.0
    a_pressed: bool = False
    x_pressed: bool = False
    y_pressed: bool = False
    is_moving_forward: bool = True
    recording_toggle_requested: bool = False
    quit_requested: bool = False


class XboxController:
    def __init__(self):
        self.joystick = None
        self.state = ControllerInput()
        self._b_was_pressed = False
        self._x_was_pressed = False
        self._y_was_pressed = False

    def connect(self) -> None:
        """Initialisiert pygame und öffnet Controller ``CONTROLLER_ID``.

        Raises:
            RuntimeError: Kein Controller gefunden.
        """
        print("[SETUP] Initialisiere Xbox Controller...")
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("[ERROR] Kein Xbox Controller gefunden!")

        self.joystick = pygame.joystick.Joystick(config.CONTROLLER_ID)
        self.joystick.init()
        print(f"[SETUP] Controller verbunden: {self.joystick.get_name()}")

    def is_connected(self) -> bool:
        return bool(self.joystick and self.joystick.get_init())

    def read(self) -> ControllerInput:
        """Liest Trigger und Tasten.

        - B (Flanke): schaltet ``is_moving_forward`` um.
        - X und Y gleichzeitig (Flanke): setzt ``recording_toggle_requested``.
        - Start: setzt ``quit_requested``.

        Exceptions von pygame werden an den Aufrufer weitergereicht. Vor dem
        Aufruf muss ``pygame.event.pump()`` gelaufen und ``is_connected()``
        True sein.
        """
        s = self.state
        s.recording_toggle_requested = False
        s.quit_requested = False

        s.trigger_left = self.joystick.get_axis(config.AXIS_LEFT_TRIGGER)
        s.trigger_right = self.joystick.get_axis(config.AXIS_RIGHT_TRIGGER)
        s.a_pressed = self.joystick.get_button(config.BUTTON_A)

        b_pressed = self.joystick.get_button(config.BUTTON_B)
        if b_pressed and not self._b_was_pressed:
            s.is_moving_forward = not s.is_moving_forward
            direction_text = "⬆️ VORWÄRTS" if s.is_moving_forward else "⬇️ RÜCKWÄRTS"
            print(f"[INPUT] 🔄 Richtung: {direction_text}")
        self._b_was_pressed = b_pressed

        x_pressed = self.joystick.get_button(config.BUTTON_X)
        y_pressed = self.joystick.get_button(config.BUTTON_Y)
        if x_pressed and y_pressed and not (self._x_was_pressed and self._y_was_pressed):
            s.recording_toggle_requested = True
        s.x_pressed = x_pressed
        s.y_pressed = y_pressed
        self._x_was_pressed = x_pressed
        self._y_was_pressed = y_pressed

        if self.joystick.get_button(config.BUTTON_START):
            s.quit_requested = True

        return s

    def close(self) -> None:
        if self.joystick:
            try:
                self.joystick.quit()
            except Exception:
                pass
        try:
            pygame.quit()
        except Exception:
            pass
