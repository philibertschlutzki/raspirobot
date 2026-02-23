from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any, Optional

class IMotorController(ABC):
    @abstractmethod
    def set_speed(self, left: float, right: float) -> None:
        """Sets motor speed. Values between -1.0 and 1.0."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stops both motors."""
        pass

class ILidarSensor(ABC):
    @abstractmethod
    def start(self) -> None:
        """Starts the LIDAR motor/scanning."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stops the LIDAR."""
        pass

    @abstractmethod
    def get_latest_scan(self) -> List[Tuple[int, float, float]]:
        """Returns the latest complete scan as a list of (quality, angle, distance)."""
        pass

class IUltrasonicSensor(ABC):
    @abstractmethod
    def get_distances(self) -> Dict[str, float]:
        """Returns distance in cm as {'left': float, 'right': float}."""
        pass

class IInputController(ABC):
    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """Returns current controller state (buttons, axes)."""
        pass
