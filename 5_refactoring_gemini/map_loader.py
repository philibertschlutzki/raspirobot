import numpy as np
import os

class MapQualityException(Exception):
    pass

def load_and_validate_map(filepath: str) -> np.ndarray:
    """
    Loads a .npy occupancy grid (Log-Odds) and validates it.

    :param filepath: Path to the .npy file.
    :return: The loaded map as a numpy array.
    :raises FileNotFoundError: If the file does not exist.
    :raises MapQualityException: If the map does not meet quality criteria.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Map file not found: {filepath}")

    try:
        occupancy_grid = np.load(filepath)
    except Exception as e:
        raise MapQualityException(f"Failed to load map file: {e}")

    total_cells = occupancy_grid.size
    if total_cells == 0:
        raise MapQualityException("Map is empty.")

    # Log-Odds interpretation:
    # Free < 0 (LO_FREE = -0.4)
    # Occupied > 0 (LO_OCC = 0.85)
    # Using thresholds to be robust against noise/initialization (0)

    free_cells = np.sum(occupancy_grid < -0.1)
    obstacle_cells = np.sum(occupancy_grid > 0.5)

    free_ratio = free_cells / total_cells
    obstacle_ratio = obstacle_cells / total_cells

    if free_ratio < 0.05:
        raise MapQualityException(f"Map has insufficient free space: {free_ratio:.2%} < 5%")

    if obstacle_ratio < 0.005:
        raise MapQualityException(f"Map has insufficient obstacles: {obstacle_ratio:.2%} < 0.5%")

    return occupancy_grid
