import numpy as np
from scipy.ndimage import binary_dilation
from scipy.interpolate import splprep, splev
import heapq
import math
from typing import List, Tuple, Optional

try:
    from .data_models import RobotPose
    from .config import OBSTACLE_INFLATION_RADIUS_M
except ImportError:
    from data_models import RobotPose
    from config import OBSTACLE_INFLATION_RADIUS_M

class AStarPlanner:
    def __init__(self, global_map_grid: np.ndarray, resolution: float = 0.05):
        self.global_map = global_map_grid
        self.resolution = resolution
        self.inflation_radius = OBSTACLE_INFLATION_RADIUS_M

        # Local map settings (3x3m)
        self.local_size_m = 3.0
        self.local_grid_size = int(self.local_size_m / self.resolution) # 60
        self.local_origin_offset = self.local_grid_size // 2 # Center

        self.virtual_obstacles: List[Tuple[float, float]] = []

    def inject_virtual_obstacle(self, x: float, y: float):
        """Injects a virtual obstacle at global coordinates (x, y)."""
        self.virtual_obstacles.append((x, y))

    def clear_virtual_obstacles(self):
        self.virtual_obstacles = []

    def plan(self, start_pose: RobotPose, goal_pose: RobotPose) -> List[Tuple[float, float]]:
        """
        Plans a path from start_pose to goal_pose using A* on a local costmap.
        Returns a list of (x, y) global coordinates.
        """
        # 1. Extract Local Map
        local_grid, local_start, local_goal = self._extract_local_map(start_pose, goal_pose)

        # 2. Add Virtual Obstacles
        self._add_virtual_obstacles(local_grid, start_pose)

        # 3. Inflate Obstacles
        # Calculate radius in pixels
        radius_px = int(self.inflation_radius / self.resolution)
        structure = np.ones((2*radius_px+1, 2*radius_px+1)) # Square kernel or disk
        # Create disk structure
        y, x = np.ogrid[-radius_px:radius_px+1, -radius_px:radius_px+1]
        mask = x*x + y*y <= radius_px*radius_px
        structure[~mask] = 0

        # Binary dilation (assuming obstacle=1, free=0)
        # Global map is Log-Odds (Obstacle > 0.5)
        # Create binary local map
        binary_local = (local_grid > 0.5).astype(int)
        inflated_grid = binary_dilation(binary_local, structure=structure).astype(int)

        # 4. A* Search
        path_pixels = self._astar(inflated_grid, local_start, local_goal)

        if not path_pixels:
            return []

        # 5. Convert to Global Coords
        # path_pixels is list of (row, col)
        global_path = []
        for r, c in path_pixels:
            # Convert local (r, c) to relative (dx, dy) meters
            dy = (r - self.local_origin_offset) * self.resolution
            dx = (c - self.local_origin_offset) * self.resolution

            # Rotate to global frame?
            # No, _extract_local_map extracts an axis-aligned patch from global map?
            # Or a robot-centric patch?
            # "Local Costmap" usually implies robot-centric.
            # But extracting axis-aligned patch is easier and sufficient if 3x3m covers rotation.
            # Let's use axis-aligned patch centered on robot global position.

            global_x = start_pose.x + dx
            global_y = start_pose.y + dy
            global_path.append((global_x, global_y))

        # 6. Smooth Path
        if len(global_path) > 3:
            return self._smooth_path(global_path)
        return global_path

    def _extract_local_map(self, start_pose: RobotPose, goal_pose: RobotPose) -> Tuple[np.ndarray, Tuple[int, int], Tuple[int, int]]:
        # Global Map Indices
        # Global origin is (200, 200) pixels
        # x_px = x / res + 200
        origin_px = 200

        start_c = int(start_pose.x / self.resolution) + origin_px
        start_r = int(start_pose.y / self.resolution) + origin_px

        goal_c = int(goal_pose.x / self.resolution) + origin_px
        goal_r = int(goal_pose.y / self.resolution) + origin_px

        # Local Bounds (Axis-Aligned around Start)
        half_size = self.local_origin_offset
        r_min = start_r - half_size
        r_max = start_r + half_size
        c_min = start_c - half_size
        c_max = start_c + half_size

        # Extract Patch (Handle boundaries)
        # Initialize with unknown/obstacle? Free is safer for OOB?
        # Usually OOB is obstacle.
        local_grid = np.zeros((self.local_grid_size, self.local_grid_size), dtype=float) + 10.0 # High value

        # Calculate intersection with global map
        # Global map size 400x400
        gm_r_min = max(0, r_min)
        gm_r_max = min(400, r_max)
        gm_c_min = max(0, c_min)
        gm_c_max = min(400, c_max)

        # Indices in local grid
        loc_r_min = gm_r_min - r_min
        loc_r_max = loc_r_min + (gm_r_max - gm_r_min)
        loc_c_min = gm_c_min - c_min
        loc_c_max = loc_c_min + (gm_c_max - gm_c_min)

        if gm_r_max > gm_r_min and gm_c_max > gm_c_min:
             local_grid[loc_r_min:loc_r_max, loc_c_min:loc_c_max] = \
                 self.global_map[gm_r_min:gm_r_max, gm_c_min:gm_c_max]

        local_start = (half_size, half_size)

        # Goal in local coords
        # goal_r - r_min = goal_r - (start_r - half_size) = (goal_r - start_r) + half_size
        l_goal_r = goal_r - r_min
        l_goal_c = goal_c - c_min

        # Clamp goal to local grid if outside
        l_goal_r = max(0, min(self.local_grid_size - 1, l_goal_r))
        l_goal_c = max(0, min(self.local_grid_size - 1, l_goal_c))

        return local_grid, local_start, (l_goal_r, l_goal_c)

    def _add_virtual_obstacles(self, local_grid: np.ndarray, start_pose: RobotPose):
        # Convert global virtual obstacles to local grid coords
        origin_px = 200
        start_c = int(start_pose.x / self.resolution) + origin_px
        start_r = int(start_pose.y / self.resolution) + origin_px

        r_min = start_r - self.local_origin_offset
        c_min = start_c - self.local_origin_offset

        for vx, vy in self.virtual_obstacles:
            vr = int(vy / self.resolution) + origin_px
            vc = int(vx / self.resolution) + origin_px

            lr = vr - r_min
            lc = vc - c_min

            if 0 <= lr < self.local_grid_size and 0 <= lc < self.local_grid_size:
                # Mark as obstacle (Log-Odds high)
                local_grid[lr, lc] = 5.0

    def _astar(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        # grid: 1=obstacle, 0=free
        rows, cols = grid.shape

        if grid[start[0], start[1]] == 1:
            # Start is in obstacle. Try to find nearest free cell?
            # Or just fail? Fail for now.
            return []

        if grid[goal[0], goal[1]] == 1:
            # Goal is in obstacle. Search near goal?
            # Simple fallback: Fail
            return []

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dr, current[1] + dc)

                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if grid[neighbor[0], neighbor[1]] == 1:
                        continue

                    tentative_g_score = g_score[current] + math.sqrt(dr*dr + dc*dc)

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return [] # No path

    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def _reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        # B-Spline interpolation
        try:
            points = np.array(path)
            # Remove duplicates
            # Unique consecutive
            unique_points = [points[0]]
            for i in range(1, len(points)):
                if np.linalg.norm(points[i] - points[i-1]) > 0.01:
                     unique_points.append(points[i])
            points = np.array(unique_points)

            if len(points) < 3:
                return path

            tck, u = splprep(points.T, s=0.05, k=min(3, len(points)-1))
            u_new = np.linspace(0, 1, num=len(path)*2)
            x_new, y_new = splev(u_new, tck)

            return list(zip(x_new, y_new))
        except Exception:
            # Fallback if spline fails
            return path
