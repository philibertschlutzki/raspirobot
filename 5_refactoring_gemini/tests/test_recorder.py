import os
import json
import gzip
import pytest
import shutil
import time
import sys
from pathlib import Path

# Add 5_refactoring_gemini/ to sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))

from data_models import LidarFrame, ControllerSample, RobotPose
from recorder import DataRecorder

def test_recorder_streaming():
    output_dir = "test_recordings_streaming"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    recorder = DataRecorder(output_dir=output_dir)

    session_id = "stream_sess_1"
    hardware_info = {"mock": True}

    # 1. Start Session
    recorder.start_session(session_id, hardware_info)

    # 2. Log Frames
    # Create a mock frame
    pose = RobotPose(x=0.0, y=0.0, theta=0.0, timestamp=100.0)
    sample = ControllerSample(
        timestamp=100.0,
        left_duty=0.5,
        right_duty=0.5,
        is_moving_forward=True,
        pose=pose
    )
    frame = LidarFrame(
        timestamp=100.0,
        frame_id=0,
        scan_data=[(0.0, 1.0)],
        controller_states=[sample]
    )

    recorder.log_frame(frame)
    time.sleep(0.1) # Give thread time to write

    frame2 = LidarFrame(
        timestamp=101.0,
        frame_id=1,
        scan_data=[(90.0, 2.0)],
        controller_states=[]
    )
    recorder.log_frame(frame2)
    time.sleep(0.1)

    # 3. Stop Session
    recorder.stop_session()

    # Verify file
    filename = f"{session_id}.jsonl.gz"
    filepath = os.path.join(output_dir, filename)
    assert os.path.exists(filepath)

    # Read back (JSONL)
    lines = []
    with gzip.open(filepath, 'rt', encoding='utf-8') as f:
        for line in f:
            lines.append(json.loads(line))

    assert len(lines) >= 3 # Metadata + 2 frames + (optional end metadata)

    assert lines[0]["session_id"] == session_id
    assert lines[0]["type"] == "metadata"

    # Check frames
    # Frame 1 might be index 1
    # Check for correct frame_id
    frame_ids = [l.get("frame_id") for l in lines if "frame_id" in l]
    assert 0 in frame_ids
    assert 1 in frame_ids

    # Cleanup
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
