import os
import json
import gzip
import pytest
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from simulate_perfect_run import simulate_run
from data_models import PathRecordingData

def test_simulation_generates_valid_file():
    output_file = "test_simulation.json.gz"
    if os.path.exists(output_file):
        os.remove(output_file)

    try:
        path = simulate_run(output_file)
        assert os.path.exists(path)

        with gzip.open(path, 'rt', encoding='utf-8') as f:
            data = json.load(f)
            # Validate with Pydantic
            session = PathRecordingData(**data)

        assert len(session.frames) == 100
        assert session.hardware_info.get("simulated") is True
    finally:
        if os.path.exists(output_file):
            os.remove(output_file)
