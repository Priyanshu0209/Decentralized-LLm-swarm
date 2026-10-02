import abc
from typing import Optional, Dict, Any, List

class BackendInterface(abc.ABC):
    """
    Abstract base class for all DroneNode backends.
    Guarantees a uniform interface regardless of simulation vs real hardware.
    """
    
    @abc.abstractmethod
    def connect(self) -> bool:
        """Connect to the backend. Return True if successful."""
        raise NotImplementedError

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the backend."""
        raise NotImplementedError
        
    @abc.abstractmethod
    def reconnect(self) -> bool:
        """Attempt to reconnect to the backend. Return True if successful."""
        raise NotImplementedError
        
    @abc.abstractmethod
    def is_connected(self) -> bool:
        """Return True if currently connected."""
        raise NotImplementedError
        
    @abc.abstractmethod
    def is_armed(self) -> bool:
        """Return True if the drone is currently armed."""
        raise NotImplementedError
        
    @abc.abstractmethod
    def is_in_air(self) -> bool:
        """Return True if the drone is currently in the air."""
        raise NotImplementedError
        
    @abc.abstractmethod
    def is_offboard_active(self) -> bool:
        """Return True if the drone is currently in offboard mode."""
        raise NotImplementedError

    @abc.abstractmethod
    def arm(self) -> bool:
        """Arm the drone."""
        raise NotImplementedError

    @abc.abstractmethod
    def disarm(self) -> bool:
        """Disarm the drone."""
        raise NotImplementedError

    @abc.abstractmethod
    def takeoff(self, altitude: Optional[float] = None) -> bool:
        """Take off to a specified altitude."""
        raise NotImplementedError

    @abc.abstractmethod
    def land(self) -> bool:
        """Land the drone at current position."""
        raise NotImplementedError

    @abc.abstractmethod
    def rtl(self) -> bool:
        """Return to Launch."""
        raise NotImplementedError
        
    @abc.abstractmethod
    def emergency_stop(self) -> bool:
        """Emergency stop the drone."""
        raise NotImplementedError

    @abc.abstractmethod
    def start_offboard(self) -> bool:
        """Start offboard mode."""
        raise NotImplementedError

    @abc.abstractmethod
    def stop_offboard(self) -> bool:
        """Stop offboard mode."""
        raise NotImplementedError

    @abc.abstractmethod
    def move_velocity(self, north_m_s: float, east_m_s: float, down_m_s: float, yaw_deg: float) -> bool:
        """Send a velocity setpoint in offboard mode."""
        raise NotImplementedError

    @abc.abstractmethod
    def move_position(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> bool:
        """Send a position setpoint in offboard mode."""
        raise NotImplementedError

    @abc.abstractmethod
    def hover(self) -> bool:
        """Command the drone to hover in place."""
        raise NotImplementedError

    @abc.abstractmethod
    def upload_mission(self, waypoints: List[Dict[str, float]]) -> bool:
        """Upload a mission (list of waypoints) to the drone."""
        raise NotImplementedError

    @abc.abstractmethod
    def start_mission(self) -> bool:
        """Start the uploaded mission."""
        raise NotImplementedError

    @abc.abstractmethod
    def pause_mission(self) -> bool:
        """Pause the current mission."""
        raise NotImplementedError

    @abc.abstractmethod
    def resume_mission(self) -> bool:
        """Resume a paused mission."""
        raise NotImplementedError

    @abc.abstractmethod
    def cancel_mission(self) -> bool:
        """Cancel the current mission."""
        raise NotImplementedError
