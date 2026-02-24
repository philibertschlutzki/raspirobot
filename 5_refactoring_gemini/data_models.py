from typing import List, Tuple, Dict, Any, Optional
from pydantic import BaseModel, ConfigDict
from enum import Enum

class LogLevel(str, Enum):
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"

class RobotPose(BaseModel):
    x: float
    y: float
    theta: float
    timestamp: float
    model_config = ConfigDict(frozen=True)

class MotorCommand(BaseModel):
    left_pwm: float
    right_pwm: float
    timestamp: float
    model_config = ConfigDict(frozen=True)

class ControllerSample(BaseModel):
    timestamp: float
    left_duty: float
    right_duty: float
    is_moving_forward: bool
    pose: RobotPose
    model_config = ConfigDict(frozen=True)

class LidarFrame(BaseModel):
    timestamp: float
    frame_id: int
    # List of (quality, angle, distance)
    scan_data: List[Tuple[int, float, float]]
    controller_states: List[ControllerSample]
    model_config = ConfigDict(frozen=True)

class PathRecordingData(BaseModel):
    session_id: str
    start_timestamp: float
    end_timestamp: float
    hardware_info: Dict[str, Any]
    frames: List[LidarFrame]
    model_config = ConfigDict(frozen=True)

class LogEntry(BaseModel):
    timestamp: float
    level: LogLevel
    event_name: str
    data: Dict[str, Any]
    model_config = ConfigDict(frozen=True)
