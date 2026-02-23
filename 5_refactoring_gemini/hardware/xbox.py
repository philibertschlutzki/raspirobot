import os
import time

try:
    from ..interfaces import IInputController
    from ..config import *
except ImportError:
    from interfaces import IInputController
    from config import *

# Headless setup - crucial for server/CI environments
os.environ["SDL_VIDEODRIVER"] = "dummy"

try:
    import pygame
except ImportError:
    print("Warning: pygame not found. Simulating PC Mode without inputs.")
    pygame = None

class XboxInput(IInputController):
    def __init__(self):
        self.controller = None
        if pygame:
            try:
                pygame.init()
                pygame.joystick.init()
                if pygame.joystick.get_count() > CONTROLLER_ID:
                    self.controller = pygame.joystick.Joystick(CONTROLLER_ID)
                    self.controller.init()
                    print(f"Xbox Controller initialized: {self.controller.get_name()}")
                else:
                    print("No Xbox controller found.")
            except Exception as e:
                print(f"Error initializing pygame joystick: {e}")

    def get_state(self):
        """Returns current state of relevant buttons and axes."""
        state = {
            "connected": False,
            "buttons": {},
            "axes": {}
        }

        if not self.controller or not pygame:
            return state

        try:
            pygame.event.pump() # Internally process event queue to update joystick states

            # Map buttons based on config
            state["connected"] = True
            state["buttons"] = {
                "A": self.controller.get_button(BUTTON_A),
                "B": self.controller.get_button(BUTTON_B),
                "X": self.controller.get_button(BUTTON_X),
                "Y": self.controller.get_button(BUTTON_Y),
                "START": self.controller.get_button(BUTTON_START),
            }
            state["axes"] = {
                "LEFT_TRIGGER": self.controller.get_axis(AXIS_LEFT_TRIGGER),
                "RIGHT_TRIGGER": self.controller.get_axis(AXIS_RIGHT_TRIGGER),
            }
        except Exception as e:
            # Controller might have disconnected
            # print(f"Controller read error: {e}") # Log spam?
            state["connected"] = False

        return state
