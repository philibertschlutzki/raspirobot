from dataclasses import dataclass, asdict, field
from typing import List, Tuple, Dict, Any

@dataclass
class RobotPose:
    x: float
    y: float
    theta: float
    timestamp: float

@dataclass
class ControllerState:
    timestamp: float
    left_duty: float
    right_duty: float
    is_moving_forward: bool
    pose: Dict[str, Any]

@dataclass
class LidarFrame:
    timestamp: float
    frame_id: int
    scan_data: List[Tuple[float, float]]
    controller_states: List[Dict[str, Any]]

@dataclass
class RecordingSession:
    session_id: str
    start_timestamp: float
    end_timestamp: float
    hardware_info: Dict[str, Any]
    frames: List[Dict[str, Any]]
