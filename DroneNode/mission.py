#!/usr/bin/env python3
"""
Mission Manager for DroneAgent

Handles the mission waypoints, tracks progress, and determines the next target.
"""

import math
from typing import List, Tuple, Optional
import logging

class MissionManager:
    def __init__(self, config: dict, logger: logging.Logger):
        """
        Initialize mission manager.
        
        Args:
            config: Configuration dictionary (contains mission waypoints)
            logger: Logger instance
        """
        self.config = config
        self.logger = logger.getChild("Mission")
        self.waypoints = self._load_waypoints()
        self.current_waypoint_index = 0
        self.waypoint_radius = config.get('mission', {}).get('waypoint_radius', 2.0)  # meters
        self.mission_id = config.get('mission', {}).get('mission_id', 0)
        self.mission_complete_action = config.get('mission', {}).get('mission_complete_action', 'RTL')
    
    def _load_waypoints(self) -> List[tuple]:
        """Load waypoints from config."""
        waypoints = self.config.get('mission', {}).get('waypoints', [])
        # Convert to tuple of (lat, lon, alt) for now; we'll need to convert to local later
        # For simplicity, we'll assume the mission manager works in GPS coordinates
        # and the planner/decision will handle coordinate conversions.
        return [tuple(wp) for wp in waypoints]
        
    def upload_mission(self, waypoints: List[list]):
        """Upload a new set of waypoints and reset mission progress."""
        self.waypoints = [tuple(wp) for wp in waypoints]
        self.current_waypoint_index = 0
        self.logger.info(f"Mission uploaded with {len(self.waypoints)} waypoints.")
        
    def cancel_mission(self):
        """Cancel the current mission by clearing waypoints."""
        self.waypoints = []
        self.current_waypoint_index = 0
        self.logger.info("Mission cancelled.")
    
    def get_next_waypoint(self, drone_id: int, telemetry, all_waypoints: List[tuple]) -> Optional[tuple]:
        """
        Get the next waypoint to target.
        
        Args:
            drone_id: ID of this drone (not used in simple mission, but could be for distributed missions)
            telemetry: Telemetry object to get current GPS position
            all_waypoints: List of mission waypoints (latitude, longitude, altitude)
            
        Returns:
            Tuple (latitude, longitude, altitude) of the next waypoint, or None if mission complete
        """
        if not all_waypoints or self.current_waypoint_index >= len(all_waypoints):
            # Mission complete
            return None
        
        # Get current GPS position
        current_pos = telemetry.get_gps() if telemetry else None
        if not current_pos:
            # If we don't have GPS, we can't determine progress
            return all_waypoints[self.current_waypoint_index]
        
        # Check if we've reached the current waypoint
        current_wp = all_waypoints[self.current_waypoint_index]
        distance = self._haversine_distance(
            current_pos[0], current_pos[1],  # lat, lon
            current_wp[0], current_wp[1]     # wp lat, lon
        )
        # Also check altitude difference
        alt_diff = abs(current_pos[2] - current_wp[2]) if len(current_pos) > 2 and len(current_wp) > 2 else 0
        
        if distance < self.waypoint_radius and alt_diff < 2.0:  # 2m altitude tolerance
            self.logger.info(f"Reached waypoint {self.current_waypoint_index}: {current_wp}")
            self.current_waypoint_index += 1
            # If we just completed the last waypoint, mission is done
            if self.current_waypoint_index >= len(all_waypoints):
                self.logger.info("Mission completed")
                return None
        
        # Return current target waypoint
        current_wp = all_waypoints[self.current_waypoint_index]
        
        home = telemetry.get_home() if telemetry else None
        if home:
            from utils import gps_to_ned
            return gps_to_ned(current_wp[0], current_wp[1], current_wp[2], home[0], home[1], home[2])
            
        # Fallback (though decision engine expects NED)
        return current_wp
    
    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate the great-circle distance between two points on Earth.
        
        Args:
            lat1, lon1: Latitude and longitude of point 1 in degrees
            lat2, lon2: Latitude and longitude of point 2 in degrees
            
        Returns:
            Distance in meters
        """
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def is_mission_complete(self) -> bool:
        """Check if the mission is complete."""
        return self.current_waypoint_index >= len(self.waypoints)
    
    def get_mission_progress(self) -> float:
        """Get mission progress as a fraction (0.0 to 1.0)."""
        if not self.waypoints:
            return 0.0
        return min(1.0, self.current_waypoint_index / len(self.waypoints))
    
    def get_remaining_waypoints(self) -> List[tuple]:
        """Get the list of remaining waypoints."""
        return self.waypoints[self.current_waypoint_index:]

if __name__ == "__main__":
    # Simple test
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Test")
    
    config = {
        'mission': {
            'waypoints': [
                [47.397742, 8.545594, 50.0],
                [47.397900, 8.545000, 50.0],
                [47.398000, 8.546000, 50.0]
            ],
            'mission_id': 1,
            'waypoint_radius': 2.0,
            'mission_complete_action': 'RTL'
        }
    }
    
    mission = MissionManager(config, logger)
    print(f"Number of waypoints: {len(mission.waypoints)}")
    print(f"First waypoint: {mission.waypoints[0]}")
