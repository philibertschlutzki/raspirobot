# decisions.py
DIST_TOO_CLOSE = 25.0  # cm
DIST_CLOSE = 50.0      # cm
SIDE_DIFF = 10.0       # cm

def decide(d_left, d_right):
    if d_left is None or d_right is None:
        return "HALT"
    if d_left < DIST_TOO_CLOSE and d_right < DIST_TOO_CLOSE:
        return "BACK_SHORT"
    if d_left < DIST_CLOSE and (d_right - d_left) >= SIDE_DIFF:
        return "TURN_RIGHT"
    if d_right < DIST_CLOSE and (d_left - d_right) >= SIDE_DIFF:
        return "TURN_LEFT"
    if d_left > DIST_CLOSE and d_right > DIST_CLOSE:
        return "FORWARD_SHORT"
    return "HALT"
