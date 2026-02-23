import os
import json
import gzip
import pytest
import shutil
import time
import sys

# Add 5_refactoring_gemini/ to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from data_models import PathRecordingData, LidarFrame
from recorder import DataRecorder

def test_recorder_save_session():
    # Setup test data
    # PathRecordingData requires valid types, so empty list is fine for frames
    # but hardware_info needs dict.

    session_data = PathRecordingData(
        session_id="test_sess_1",
        start_timestamp=100.0,
        end_timestamp=200.0,
        hardware_info={},
        frames=[]
    )

    output_dir = "test_recordings_tmp"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    recorder = DataRecorder(output_dir=output_dir)

    # Save async
    thread = recorder.save_session_async(session_data, filename="test.json.gz")
    thread.join()

    filepath = os.path.join(output_dir, "test.json.gz")
    assert os.path.exists(filepath)

    # Read back and verify
    with gzip.open(filepath, 'rt', encoding='utf-8') as f:
        content = f.read()
        loaded_data = json.loads(content)

    assert loaded_data["session_id"] == "test_sess_1"

    # Cleanup
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
