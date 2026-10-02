#!/usr/bin/env python3
"""
Collision Avoidance Module for DroneAgent

Computes avoidance vectors to prevent collisions with neighbors and obstacles.
"""

import math
from typing import List, Tuple, Optional, Dict
import logging

class CollisionAvoidance:
    def __init__(self, config: dict, logger: logging.Logger):
        """
        Initialize collision avoidance module.
        
        Args:
            config: Configuration dictionary
            logger: Logger instance
        """
        self.config = config
        self.logger = logger.getChild("Collision")
        self.safe_distance = config.get('safe_distance', 5.0)  # meters
        self.max_avoidance_force = config.get('max_avoidance_force', 2.0)  # m/s^2
        self.avoidance_distance = self.safe_distance * 1.5  # Start avoiding earlier
    
        self.time_horizon = 2.0  # seconds for velocity obstacle prediction
    def get_avoidance_vector(self, 
                           own_position: Tuple[float, float, float],
                           own_velocity: Tuple[float, float, float],
                           neighbors: Dict[int, dict]) -> Tuple[float, float, float]:
        """
        Compute avoidance vector to maintain safe distance from neighbors.
        
        Args:
            own_position: (north, east, down) of own drone in meters
            own_velocity: (north, east, down) velocity in m/s
            neighbors: dict of neighbor_id -> {'last_packet': DronePacket, 'last_heard': timestamp}
                      or dict of neighbor_id -> DronePacket (simplified)
            
        Returns:
            Tuple (avoid_north, avoidance_east, avoidance_down) representing
            the desired acceleration adjustment to avoid collisions.
        """
        avoid_north, avoid_east, avoid_down = 0.0, 0.0, 0.0
        count = 0
        
        for neighbor_id, neighbor_data in neighbors.items():
            # Extract neighbor state
            if isinstance(neighbor_data, dict) and 'last_packet' in neighbor_data:
                packet = neighbor_data['last_packet']
                if packet is None:
                    continue
                # Get position and velocity from packet
                # Assuming packet has methods to get position and velocity
                neighbor_pos = self._get_position_from_packet(packet)
                neighbor_vel = self._get_velocity_from_packet(packet)
            else:
                # Assume neighbor_data is already a packet-like object
                neighbor_pos = self._get_position_from_packet(neighbor_data)
                neighbor_vel = self._get_velocity_from_packet(neighbor_data)
            
            if neighbor_pos is None or neighbor_vel is None:
                continue
            
            # Relative position and velocity
            rel_pos = (
                neighbor_pos[0] - own_position[0],
                neighbor_pos[1] - own_position[1],
                0.0  # Force 2D avoidance to prevent altitude drift
            )
            rel_vel = (
                neighbor_vel[0] - own_velocity[0],
                neighbor_vel[1] - own_velocity[1],
                neighbor_vel[2] - own_velocity[2]
            )
            
            # Distance
            dist = math.sqrt(rel_pos[0]**2 + rel_pos[1]**2 + rel_pos[2]**2)
            
            # If within avoidance distance, compute repulsive force
            if dist < self.avoidance_distance and dist > 0.1:  # Avoid division by zero
                # Simple proportional repulsion
                # Force increases as distance decreases
                if dist < self.safe_distance:
                    # Strong repulsion when too close
                    factor = self.max_avoidance_force * (1.0 - dist / self.safe_distance)
                else:
                    # Gradual repulsion in the warning zone
                    factor = self.max_avoidance_force * (self.avoidance_distance - dist) / (self.avoidance_distance - self.safe_distance)
                
                # Direction is away from neighbor
                if dist > 0:
                    avoid_north -= factor * (rel_pos[0] / dist)
                    avoid_east -= factor * (rel_pos[1] / dist)
                    avoid_down -= factor * (rel_pos[2] / dist)
                count += 1
        
        # If we have multiple neighbors, we could average or take the maximum
        # For simplicity, we'll sum the avoidance vectors (which is what we did)
        # But we should clamp the total avoidance to avoid excessive acceleration
        avoidance_magnitude = math.sqrt(avoid_north**2 + avoid_east**2 + avoid_down**2)
        if avoidance_magnitude > self.max_avoidance_force:
            scale = self.max_avoidance_force / avoidance_magnitude
            avoid_north *= scale
            avoid_east *= scale
            avoid_down *= scale
        
        return (avoid_north, avoid_east, avoid_down)
    
    def _get_position_from_packet(self, packet) -> Optional[Tuple[float, float, float]]:
        """Extract position from a drone packet."""
        try:
            # Assuming packet has attributes or methods for position
            # We'll use the local position from the packet (north, east, down)
            # In the packet definition, we have local_pos_x, local_pos_y, local_pos_z
            return (packet.local_pos_x, packet.local_pos_y, packet.local_pos_z)
        except AttributeError:
            # Try dictionary-style access
            try:
                return (packet['local_pos_x'], packet['local_pos_y'], packet['local_pos_z'])
            except (TypeError, KeyError):
                return None
    
    def _get_velocity_from_packet(self, packet) -> Optional[Tuple[float, float, float]]:
        """Extract velocity from a drone packet."""
        try:
            return (packet.velocity_x, packet.velocity_y, packet.velocity_z)
        except AttributeError:
            try:
                return (packet['velocity_x'], packet['velocity_y'], packet['velocity_z'])
            except (TypeError, KeyError):
                return None

# Alternative: Velocity Obstacle method (more complex)
# We'll stick with the simple potential field for now.

