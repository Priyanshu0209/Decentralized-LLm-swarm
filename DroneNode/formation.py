#!/usr/bin/env python3
"""
Formation Module for DroneAgent

Calculates the target position for a drone within a formation pattern.
Supports 24 distinct formation topologies requested for the swarm.
"""

import math
from typing import List, Tuple, Optional, Dict
import logging

class Formation:
    def __init__(self, config: dict, logger: logging.Logger):
        self.config = config
        self.logger = logger.getChild("Formation")
        
        # Default Parameters
        self.params = {
            'type': config.get('mission', {}).get('formation', 'v'),
            'spacing': config.get('formation_spacing', 5.0),
            'heading': config.get('formation_heading', 0.0), # Radians
            'rotation': config.get('formation_rotation', 0.0), # Radians
            'altitude_offset': config.get('formation_altitude_offset', 0.0),
            'expansion': config.get('formation_expansion', 1.0),
        }

    def update_parameters(self, new_params: dict):
        """Live update formation parameters."""
        for k, v in new_params.items():
            if k in self.params:
                self.params[k] = v
        self.logger.info(f"Formation parameters updated: {self.params}")

    def get_formation_position(self, drone_id: int, neighbors: dict, telemetry, time_t: float = 0.0) -> Optional[tuple]:
        """Calculate the desired target position in NED coordinates."""
        if not neighbors and drone_id != 1:
            return None
            
        positions = []
        for nid, state in neighbors.items():
            if state is not None:
                pos = self._get_position_from_state(state)
                if pos:
                    positions.append(pos)
                    
        if not positions:
            if len(neighbors) == 0:
                return telemetry.get_local_position() if telemetry else None
            return None

        # Leaderless Center Calculation (Centroid) including self
        if telemetry:
            my_pos = telemetry.get_local_position()
            if my_pos:
                positions.append(my_pos)
        n = len(positions)
        north_sum = sum(p[0] for p in positions)
        east_sum = sum(p[1] for p in positions)
        down_sum = sum(p[2] for p in positions)
        # Use my own altitude instead of average to prevent downward drift
        my_down = my_pos[2] if telemetry and my_pos else 0.0
        center = (north_sum / n, east_sum / n, my_down)
        
        # Calculate local offset dynamically based on active neighbors
        active_ids = list(neighbors.keys()) + [drone_id]
        active_ids.sort()
        try:
            index = active_ids.index(drone_id)
        except ValueError:
            index = 0
        total_drones = len(active_ids)
        
        offset = self._calculate_offset(index, total_drones, time_t)
        if offset is None:
            return None
            
        # Apply expansion, rotation, and altitude offset
        exp = self.params['expansion']
        rot = self.params['rotation'] + self.params['heading']
        
        ox = offset[0] * exp
        oy = offset[1] * exp
        oz = offset[2] + self.params['altitude_offset']
        
        # Rotate offset (yaw)
        cos_rot = math.cos(rot)
        sin_rot = math.sin(rot)
        rot_north = ox * cos_rot - oy * sin_rot
        rot_east = ox * sin_rot + oy * cos_rot
        
        # Translate to global center
        return (center[0] + rot_north, center[1] + rot_east, center[2] + oz)

    def _get_position_from_state(self, state) -> Optional[tuple]:
        if hasattr(state, 'get_local_position'):
            return state.get_local_position()
        elif isinstance(state, dict):
            if 'last_packet' in state:
                p = state['last_packet']
                if hasattr(p, 'local_pos_x'):
                    return (p.local_pos_x, p.local_pos_y, p.local_pos_z)
            elif 'local_pos' in state:
                return state['local_pos']
        elif hasattr(state, 'local_pos_x'):
            return (state.local_pos_x, state.local_pos_y, state.local_pos_z)
        elif hasattr(state, 'local_pos'):
            return state.local_pos
        return None

    def _calculate_offset(self, index: int, n: int, t: float) -> Optional[tuple]:
        """Mathematical definitions for all 24 formations (North, East, Down)."""
        ftype = self.params['type'].lower().replace(" ", "_")
        d = self.params['spacing']
        
        if ftype == "line" or ftype == "side_by_side":
            half_length = (n - 1) * d / 2.0
            return (0.0, -half_length + index * d, 0.0)
            
        elif ftype == "column" or ftype == "single_file" or ftype == "follow":
            half_length = (n - 1) * d / 2.0
            return (half_length - index * d, 0.0, 0.0)
            
        elif ftype == "v":
            if index == 0: return (0.0, 0.0, 0.0)
            side = -1 if index % 2 == 1 else 1
            pos = (index + 1) // 2
            return (-pos * d * 0.866, side * pos * d * 0.5, 0.0)
            
        elif ftype == "wedge":
            # Like V but filled
            row = int((-1 + math.sqrt(1 + 8 * (index + 1))) / 2)
            idx_in_row = index - (row * (row + 1)) // 2
            return (-row * d * 0.866, (idx_in_row - row / 2.0) * d, 0.0)
            
        elif ftype == "arrow":
            if index == 0: return (d, 0.0, 0.0)
            side = -1 if index % 2 == 1 else 1
            pos = (index + 1) // 2
            return (-pos * d * 0.866, side * pos * d * 0.5, 0.0)
            
        elif ftype == "diamond":
            if index == 0: return (d, 0.0, 0.0)
            if index == 1: return (0.0, -d, 0.0)
            if index == 2: return (0.0, d, 0.0)
            if index == 3: return (-d, 0.0, 0.0)
            return (0.0, 0.0, 0.0)
            
        elif ftype in ["triangle", "equilateral_triangle", "isosceles_triangle"]:
            if index == 0: return (d, 0.0, 0.0)
            if index == 1: return (-d*0.5, -d*0.866, 0.0)
            if index == 2: return (-d*0.5, d*0.866, 0.0)
            return (0.0, 0.0, 0.0)
            
        elif ftype in ["square", "rectangle"]:
            if n <= 1: return (0.0, 0.0, 0.0)
            side_len = math.ceil(math.sqrt(n))
            row = index // side_len
            col = index % side_len
            return ((row - side_len/2.0)*d, (col - side_len/2.0)*d, 0.0)
            
        elif ftype == "grid":
            cols = math.ceil(math.sqrt(n))
            row = index // cols
            col = index % cols
            return (-row * d, (col - (cols-1)/2.0) * d, 0.0)
            
        elif ftype == "circle":
            angle = (2 * math.pi * index) / max(1, n)
            r = (n * d) / (2 * math.pi) if n > 1 else 0
            return (r * math.cos(angle), r * math.sin(angle), 0.0)
            
        elif ftype == "arc":
            if n <= 1: return (0.0, 0.0, 0.0)
            angle = math.pi * (index / (n - 1))
            r = (n * d) / math.pi
            return (r * math.sin(angle), r * math.cos(angle), 0.0)
            
        elif ftype == "echelon_left":
            return (-index * d, -index * d, 0.0)
            
        elif ftype == "echelon_right":
            return (-index * d, index * d, 0.0)
            
        elif ftype == "fan":
            if index == 0: return (0.0, 0.0, 0.0)
            angle = math.pi * ((index - 1) / max(1, n - 2)) if n > 2 else math.pi/2
            return (d * math.sin(angle), d * math.cos(angle), 0.0)
            
        elif ftype == "spiral":
            theta = index * math.pi / 2.0
            r = d * (1 + 0.5 * index)
            return (r * math.cos(theta), r * math.sin(theta), 0.0)
            
        elif ftype == "orbit":
            # Dynamic circular orbit using time_t
            omega = 0.5 # rad/s
            angle = (2 * math.pi * index) / max(1, n) + (omega * t)
            r = max(d, (n * d) / (2 * math.pi))
            return (r * math.cos(angle), r * math.sin(angle), 0.0)
            
        elif ftype in ["free_formation", "dynamic_formation"]:
            return (0.0, 0.0, 0.0)
            
        # Default fallback
        return (0.0, 0.0, 0.0)
