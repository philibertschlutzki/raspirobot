import pytest
import os
import sys
import shutil
import numpy as np
import tempfile
from pathlib import Path

# Add parent directory to path to import modules from 5_refactoring_gemini
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulate_perfect_run import simulate_run
from simulator_replay import run_replay

def test_tc_5_1_determinism():
    """
    TC 5.1: Zweifaches Abspielen muss bitgenau den identischen Output liefern.
    """
    # Setup temporary directory for test artifacts
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)

        # 1. Generate synthetic data
        input_file = temp_path / "test_run.json.gz"

        print(f"Generating synthetic data at {input_file}")
        generated_file = simulate_run(output_path=str(input_file))
        assert os.path.exists(generated_file)

        output_dir_1 = temp_path / "run1"
        output_dir_2 = temp_path / "run2"

        # 2. Run Replay 1
        print("Starting Replay 1...")
        run_replay(str(generated_file), output_dir=str(output_dir_1), show_gui=False)

        # 3. Run Replay 2
        print("Starting Replay 2...")
        run_replay(str(generated_file), output_dir=str(output_dir_2), show_gui=False)

        # 4. Compare Outputs
        map1_path = output_dir_1 / "final_map.npy"
        map2_path = output_dir_2 / "final_map.npy"

        assert map1_path.exists(), "Run 1 did not produce final_map.npy"
        assert map2_path.exists(), "Run 2 did not produce final_map.npy"

        print("Loading maps for comparison...")
        map1 = np.load(map1_path)
        map2 = np.load(map2_path)

        # Bit-exact comparison
        print("Asserting array equality...")
        np.testing.assert_array_equal(map1, map2, err_msg="Maps are not identical (determinism failure)")
