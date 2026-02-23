import os
import pytest
import sys
import shutil

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from simulate_perfect_run import simulate_run
from simulator_replay import run_replay

def test_replay_produces_artifacts():
    # 1. Generate simulation
    sim_file = "test_sim_replay.json.gz"
    if os.path.exists(sim_file):
        os.remove(sim_file)
    simulate_run(sim_file)

    # 2. Run Replay
    output_dir = "test_results_replay"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    # Run headless
    run_replay(sim_file, output_dir, show_gui=False)

    # 3. Verify Artifacts
    assert os.path.exists(os.path.join(output_dir, "final_map.npy"))

    # Check PNG if matplotlib available
    try:
        import matplotlib
        # If installed, check png
        assert os.path.exists(os.path.join(output_dir, "final_map.png"))
    except ImportError:
        pass

    # Cleanup
    if os.path.exists(sim_file):
        os.remove(sim_file)
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
