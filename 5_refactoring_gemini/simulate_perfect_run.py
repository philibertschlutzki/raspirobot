import math
import time
import gzip
import json
import uuid
import sys
import os

try:
    from .data_models import PathRecordingData, LidarFrame, RobotPose, ControllerSample
    from .odometry import OdometryEngine
except ImportError:
    # If run as script or imported as top-level module
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from data_models import PathRecordingData, LidarFrame, RobotPose, ControllerSample
    from odometry import OdometryEngine

# Room: Rect 0,0 to 6,4
WALLS = [
    ((0, 0), (6, 0)),
    ((6, 0), (6, 4)),
    ((6, 4), (0, 4)),
    ((0, 4), (0, 0))
]

# Obstacle: Triangle at center
OBSTACLES = [
    ((2.5, 1.5), (3.5, 1.5)),
    ((3.5, 1.5), (3.0, 2.5)),
    ((3.0, 2.5), (2.5, 1.5))
]

def ray_intersect(origin, direction, p1, p2):
    """Returns distance to intersection or None."""
    v1 = origin
    v2 = (origin[0] + math.cos(direction), origin[1] + math.sin(direction))

    x1, y1 = v1
    x2, y2 = v2
    x3, y3 = p1
    x4, y4 = p2

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    if t > 0 and 0 <= u <= 1:
        # t is scaling factor for direction vector (length 1). So t is distance.
        return t
    return None

def generate_scan(pose: RobotPose):
    scan_data = []
    # 360 rays? Or RPLidar resolution (e.g. 1 deg).
    # Let's do 1 degree steps.
    for angle_deg in range(0, 360):
        angle_rad = math.radians(angle_deg)
        # Global ray direction
        ray_dir = pose.theta + angle_rad

        min_dist = 1000.0
        hit = False

        # Check Walls
        for w1, w2 in WALLS:
            d = ray_intersect((pose.x, pose.y), ray_dir, w1, w2)
            if d is not None:
                min_dist = min(min_dist, d)
                hit = True

        # Check Obstacles
        for o1, o2 in OBSTACLES:
            d = ray_intersect((pose.x, pose.y), ray_dir, o1, o2)
            if d is not None:
                min_dist = min(min_dist, d)
                hit = True

        if hit and min_dist < 10.0: # Max range
            scan_data.append((float(angle_deg), min_dist * 1000.0))

    return scan_data

def simulate_run(output_path="simulated_perfect_run.json.gz"):
    frames = []

    # Path: Circle around (3, 2) radius 1.5
    center_x, center_y = 3.0, 2.0
    radius = 1.5

    steps = 100
    total_time = 10.0 # 10 seconds
    dt = total_time / steps

    start_time = time.time()

    for i in range(steps):
        t = i * dt
        # Angle parameter for circle
        alpha = (2 * math.pi * i) / steps

        x = center_x + radius * math.cos(alpha)
        y = center_y + radius * math.sin(alpha)
        theta = alpha + math.pi/2

        pose = RobotPose(x=x, y=y, theta=theta, timestamp=start_time + t)

        scan = generate_scan(pose)

        # Dummy controller state
        sample = ControllerSample(
            timestamp=start_time + t,
            left_duty=0.5,
            right_duty=0.5,
            is_moving_forward=True,
            pose=pose
        )

        frame = LidarFrame(
            timestamp=start_time + t,
            frame_id=i,
            scan_data=scan,
            controller_states=[sample]
        )
        frames.append(frame)

    session = PathRecordingData(
        session_id=str(uuid.uuid4()),
        start_timestamp=start_time,
        end_timestamp=start_time + total_time,
        hardware_info={"simulated": True},
        frames=frames
    )

    json_str = session.model_dump_json()
    with gzip.open(output_path, 'wt', encoding='utf-8') as f:
        f.write(json_str)

    print(f"Simulation saved to {output_path}")
    return output_path

if __name__ == "__main__":
    simulate_run()
