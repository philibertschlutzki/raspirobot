import os
import json
import gzip
import numpy as np
import time
import sys

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

try:
    from .data_models import PathRecordingData, RobotPose
    from .slam_engine import SLAMEngine
except ImportError:
    # Add path logic
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from data_models import PathRecordingData, RobotPose
    from slam_engine import SLAMEngine

def run_replay(input_file: str, output_dir: str = "results", show_gui: bool = True):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"Loading {input_file}...")
    try:
        with gzip.open(input_file, 'rt', encoding='utf-8') as f:
            data = json.load(f)
            session = PathRecordingData(**data)
    except Exception as e:
        print(f"Error loading file: {e}")
        return

    print(f"Loaded {len(session.frames)} frames.")

    slam = SLAMEngine()

    can_plot = HAS_PLOT

    # GUI Setup
    if show_gui and can_plot:
        try:
            plt.ion()
            fig, ax = plt.subplots()
        except:
            can_plot = False

    last_pose = None

    for i, frame in enumerate(session.frames):
        # Get Odometry Guess
        if frame.controller_states:
            guess_pose = frame.controller_states[-1].pose
        else:
            if last_pose:
                guess_pose = last_pose
            else:
                guess_pose = RobotPose(x=0, y=0, theta=0, timestamp=frame.timestamp)

        # 1. Get scan
        local_points = slam.scan_to_xy(frame.scan_data)

        # 2. Update Pose via ICP
        pose = slam.icp(local_points, guess_pose)
        last_pose = pose

        # 3. Update Map
        slam.update_map(pose, frame.scan_data)

        if show_gui and can_plot and i % 10 == 0:
            try:
                ax.clear()
                if len(slam.reference_points) > 0:
                    ax.scatter(slam.reference_points[:, 0], slam.reference_points[:, 1], s=1, c='black')
                ax.plot(pose.x, pose.y, 'ro')
                global_scan = slam.transform_points(local_points, pose)
                if len(global_scan) > 0:
                    ax.scatter(global_scan[:, 0], global_scan[:, 1], s=1, c='red')

                ax.set_title(f"Frame {i}")
                plt.pause(0.001)
            except Exception as e:
                print(f"Plotting error: {e}")
                can_plot = False

    # Save artifacts
    np.save(os.path.join(output_dir, "final_map.npy"), slam.grid)

    if can_plot:
        try:
            plt.ioff()
            fig, ax = plt.subplots()
            if len(slam.reference_points) > 0:
                 ax.scatter(slam.reference_points[:, 0], slam.reference_points[:, 1], s=1, c='black')
            ax.set_aspect('equal')
            plt.title("Final Map")
            plt.savefig(os.path.join(output_dir, "final_map.png"))
            plt.close()
        except Exception as e:
            print(f"Error saving PNG: {e}")
    else:
        print("Matplotlib not found or disabled, skipping PNG save.")

    print("Replay finished.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        run_replay(sys.argv[1])
    else:
        print("Usage: python simulator_replay.py <input_json_gz>")
