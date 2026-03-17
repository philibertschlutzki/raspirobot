import pytest
import numpy as np
import os
import sys

# Ensure sys.path includes the module directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from map_loader import load_and_validate_map, MapQualityException


def test_load_and_validate_map_invalid_format(tmp_path):
    """Test that loading a corrupted or invalid .npy file raises MapQualityException."""
    invalid_file = tmp_path / "corrupted_map.npy"
    # Write arbitrary non-numpy data
    with open(invalid_file, "w") as f:
        f.write("This is not a valid numpy array file")

    with pytest.raises(MapQualityException) as exc_info:
        load_and_validate_map(str(invalid_file))

    assert "Failed to load map file" in str(exc_info.value)

def test_load_and_validate_map_file_not_found():
    """Test that a missing map file raises FileNotFoundError."""
    with pytest.raises(FileNotFoundError) as exc_info:
        load_and_validate_map("non_existent_map_file.npy")

    assert "Map file not found" in str(exc_info.value)

def test_load_and_validate_map_empty(tmp_path):
    """Test that an empty map raises MapQualityException."""
    empty_map_file = tmp_path / "empty_map.npy"
    np.save(str(empty_map_file), np.array([]))

    with pytest.raises(MapQualityException) as exc_info:
        load_and_validate_map(str(empty_map_file))

    assert "Map is empty" in str(exc_info.value)

def test_load_and_validate_map_insufficient_free_space(tmp_path):
    """Test that a map with < 5% free space raises MapQualityException."""
    # Create a 10x10 map all occupied (0.85)
    grid = np.full((10, 10), 0.85)
    # Add just 4 free cells (4%)
    grid[0:2, 0:2] = -0.4

    map_file = tmp_path / "no_free_space.npy"
    np.save(str(map_file), grid)

    with pytest.raises(MapQualityException) as exc_info:
        load_and_validate_map(str(map_file))

    assert "Map has insufficient free space" in str(exc_info.value)

def test_load_and_validate_map_insufficient_obstacles(tmp_path):
    """Test that a map with < 0.5% obstacles raises MapQualityException."""
    # Create a 20x20 map all free (-0.4)
    # Total cells: 400
    # 0.5% of 400 is 2 cells.
    grid = np.full((20, 20), -0.4)
    # Add just 1 occupied cell (0.25%)
    grid[0, 0] = 0.85

    map_file = tmp_path / "no_obstacles.npy"
    np.save(str(map_file), grid)

    with pytest.raises(MapQualityException) as exc_info:
        load_and_validate_map(str(map_file))

    assert "Map has insufficient obstacles" in str(exc_info.value)

def test_load_and_validate_map_valid(tmp_path):
    """Test that a valid map loads successfully."""
    # Create a 20x20 map
    # Total cells: 400
    grid = np.full((20, 20), -0.4) # All free initially
    # Need > 5% free space (20+ cells). We have almost 400.
    # Need > 0.5% obstacles (2+ cells). Let's add 10 cells (2.5%).
    grid[5:10, 5:7] = 0.85

    map_file = tmp_path / "valid_map.npy"
    np.save(str(map_file), grid)

    loaded_grid = load_and_validate_map(str(map_file))
    np.testing.assert_array_equal(loaded_grid, grid)
