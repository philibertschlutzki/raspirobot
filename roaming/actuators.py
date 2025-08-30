# actuators.py
import time

DRY_RUN = False
pwm_left = None
pwm_right = None

def setup_motors():
    global pwm_left, pwm_right
    if DRY_RUN: return
    import back_180_fwd as drv
    pwm_left, pwm_right = drv.setup()

def cleanup_motors():
    if DRY_RUN: return
    try:
        if pwm_left: pwm_left.stop()
        if pwm_right: pwm_right.stop()
    except Exception:
        pass

def run_command(cmd):
    print(f"[ACT] {cmd}")
    if DRY_RUN:
        time.sleep(0.2); return
    import back_180_fwd as drv
    SPEED, TURN = drv.SPEED_DUTY, drv.TURN_DUTY
    TURN_S = max(0.5, drv.TURN_DURATION_S * 0.6)
    if cmd == "FORWARD_SHORT":
        drv.soft_run_section(pwm_left, pwm_right, backward=False, duration_s=1.0, target_duty=SPEED)
    elif cmd == "BACK_SHORT":
        drv.soft_run_section(pwm_left, pwm_right, backward=True, duration_s=0.8, target_duty=SPEED)
    elif cmd == "TURN_LEFT":
        drv.soft_turn_section(pwm_left, pwm_right, clockwise=False, duration_s=TURN_S, target_duty=TURN)
    elif cmd == "TURN_RIGHT":
        drv.soft_turn_section(pwm_left, pwm_right, clockwise=True,  duration_s=TURN_S, target_duty=TURN)
    elif cmd == "HALT":
        drv.soft_run_section(pwm_left, pwm_right, backward=False, duration_s=0.0, target_duty=0)
