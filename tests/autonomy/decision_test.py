#!/usr/bin/env python3
# decision_logic_test.py

from autonomy_controller import decide

scenarios = [
    ("frei", 120.0, 130.0),        # -> FORWARD_SHORT
    ("links nah", 30.0, 80.0),     # -> TURN_RIGHT
    ("rechts nah", 90.0, 28.0),    # -> TURN_LEFT
    ("beide sehr nah", 15.0, 18.0),# -> BACK_SHORT
    ("unsicher", None, 60.0),      # -> HALT
    ("grenzwertig", 45.0, 48.0),   # -> HALT (vorsichtig)
]

for name, dl, dr in scenarios:
    cmd = decide(dl, dr)
    print(f"{name:16s} dL={dl}, dR={dr} -> {cmd}")
