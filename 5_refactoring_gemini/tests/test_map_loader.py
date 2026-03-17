import os
import sys
import pytest
import numpy as np

# Add parent directory to sys.path to allow importing map_loader
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from map_loader import load_and_validate_map, MapQualityException

def test_load_nonexistent_file():
    with pytest.raises(FileNotFoundError, match="Map file not found"):
        load_and_validate_map("nonexistent_file.npy")

def test_load_invalid_file(tmp_path):
    # Create an invalid file (not a numpy array)
    invalid_file = tmp_path / "invalid.npy"
    invalid_file.write_text("This is not a valid numpy file")

    with pytest.raises(MapQualityException, match="Failed to load map file"):
        load_and_validate_map(str(invalid_file))

def test_empty_map(tmp_path):
    empty_file = tmp_path / "empty.npy"
    np.save(str(empty_file), np.array([]))

    with pytest.raises(MapQualityException, match="Map is empty"):
        load_and_validate_map(str(empty_file))

def test_insufficient_free_space(tmp_path):
    # Create a 10x10 array of all obstacles (> 0.5)
    data = np.full((10, 10), 0.85)
    insufficient_free_file = tmp_path / "no_free.npy"
    np.save(str(insufficient_free_file), data)

    with pytest.raises(MapQualityException, match="Map has insufficient free space"):
        load_and_validate_map(str(insufficient_free_file))

def test_insufficient_obstacles(tmp_path):
    # Create a 10x10 array of all free space (< -0.1)
    data = np.full((10, 10), -0.4)
    insufficient_obstacles_file = tmp_path / "no_obstacles.npy"
    np.save(str(insufficient_obstacles_file), data)

    with pytest.raises(MapQualityException, match="Map has insufficient obstacles"):
        load_and_validate_map(str(insufficient_obstacles_file))

def test_valid_map(tmp_path):
    # Create a 10x10 array (100 cells)
    # 10 cells of free space (-0.4) -> 10%
    # 5 cells of obstacles (0.85) -> 5%
    # 85 cells of unknown (0.0)
    data = np.zeros((10, 10))
    data[0, :10] = -0.4  # First row is free space
    data[1, :5] = 0.85   # Half of second row is obstacles

    valid_file = tmp_path / "valid.npy"
    np.save(str(valid_file), data)

    result = load_and_validate_map(str(valid_file))
    assert isinstance(result, np.ndarray)
    assert result.shape == (10, 10)
    assert np.array_equal(result, data)
