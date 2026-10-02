#!/usr/bin/env python3
"""
Decision Engine for DroneAgent

Arbitrates between different behaviors (formation keeping, mission following, 
obstacle avoidance, etc.) to produce a desired state setpoint.
"""

import threading
import time
import math
from typing import Optional, Tuple, List
import logging
from formation import Formation
from mission import MissionManager
from collision import CollisionAvoidance

class DecisionEngine:
    def __init__(self, config: dict, logger: logging.Logger, state_machine=None, telemetry=None, communication=None):
        """
        Initialize decision engine.
        
        Args:
            config: Configuration dictionary
            logger: Logger instance
        """
        self.config = config
        self.logger = logger.getChild("Decision")
        self.drone_id = config['drone_id']
        
        # Submodules
        self.formation = Formation(config, logger)
        self.mission_manager = MissionManager(config, logger)
        self.collision_avoidance = CollisionAvoidance(config, logger)
        
        # Current state inputs (updated by other modules)
        self._current_state = None  # from state_machine
        self.state_machine = state_machine
        self._telemetry = None      # from telemetry module
        self.telemetry = telemetry
        self._neighbors = {}        # drone_id -> state packet
        self.communication = communication
        self._mission_waypoints = [] # from mission manager
        self._formation_target = None # from formation module
        self._mission_target = None   # from mission manager
        self._obstacle_avoidance_vector = (0.0, 0.0, 0.0) # from collision avoidance
        
        # Output: desired state for the planner
        self._desired_position = (0.0, 0.0, 0.0)  # north, east, down
        self._desired_velocity = (0.0, 0.0, 0.0)
        self._desired_yaw = 0.0  # radians
        
        # Control flags
        self._stop_event = threading.Event()
        self._thread = None
        self._lock = threading.RLock()
    
    def start(self):
        """Start the decision engine loop."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._decision_loop, name="Decision")
        self._thread.start()
        self.logger.info("Decision engine started")
    
    def stop(self):
        """Stop the decision engine loop."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.logger.info("Decision engine stopped")
    
    def update_state(self, state):
        """Update current drone state (from state machine)."""
        with self._lock:
            self._current_state = state

    def update_parameters(self, params: dict):
        """Live update configuration parameters."""
        with self._lock:
            if 'max_velocity' in params:
                self.config['max_velocity'] = params['max_velocity']
            self.formation.update_parameters(params)
    
    def update_telemetry(self, telemetry):
        """Update telemetry data."""
        with self._lock:
            self._telemetry = telemetry
    
    def update_neighbors(self, neighbors: dict):
        """Update neighbor states.
        
        Args:
            neighbors: dict mapping drone_id to DronePacket or None
        """
        with self._lock:
            self._neighbors = neighbors
    
    def update_mission_waypoints(self, waypoints: list):
        """Update mission waypoints from mission manager."""
        with self._lock:
            self._mission_waypoints = waypoints
    
    def _decision_loop(self):
        """Main decision loop: arbitrate between behaviors."""
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    state = self.state_machine.get_state() if self.state_machine else self._current_state
                    telemetry = self.telemetry if self.telemetry else self._telemetry
                    neighbors = self.communication.get_neighbors() if self.communication else self._neighbors
                
                # If we don't have essential data, hold position
                if not telemetry:
                    self._hold_position()
                    time.sleep(0.1)
                    continue
                
                # Get current position
                current_pos = telemetry.get_local_position()
                if not current_pos:
                    self._hold_position()
                    time.sleep(0.1)
                    continue
                
                # Initialize target as current position (hold)
                target_pos = current_pos
                target_vel = (0.0, 0.0, 0.0)
                target_yaw = telemetry.get_heading() or 0.0
                
                # Apply collision avoidance (always active)
                avoidance = self.collision_avoidance.get_avoidance_vector(
                    current_pos, telemetry.get_velocity(), neighbors
                )
                
                # State-based behavior
                if state:
                    state_name = state.name if hasattr(state, 'name') else str(state)
                    if state_name == "FORMATION":
                        # Formation keeping (pass time.time() for dynamic formations like Orbit)
                        formation_target = self.formation.get_formation_position(
                            self.drone_id, neighbors, telemetry, time_t=time.time()
                        )
                        if formation_target:
                            target_pos = formation_target
                            # For formation, we might want to move toward the target with some velocity
                            # For simplicity, we'll use position control and let the planner handle velocity
                    elif state_name == "MISSION":
                        # Mission following
                        mission_target = self.mission_manager.get_next_waypoint(
                            self.drone_id, telemetry, self._mission_waypoints
                        )
                        if mission_target:
                            target_pos = mission_target
                    # Add more states as needed (OFFBOARD, RTL, etc.)
                
                # Combine with collision avoidance: adjust target position to avoid obstacles
                if avoidance != (0.0, 0.0, 0.0):
                    # Apply avoidance as an offset to the target position
                    avoidance_scale = 0.5  # meters of avoidance per unit vector
                    adjusted_target = (
                        target_pos[0] + avoidance[0] * avoidance_scale,
                        target_pos[1] + avoidance[1] * avoidance_scale,
                        target_pos[2] + avoidance[2] * avoidance_scale
                    )
                    target_pos = adjusted_target
                
                # Calculate desired velocity to move toward target (simple proportional control)
                pos_error = (
                    target_pos[0] - current_pos[0],
                    target_pos[1] - current_pos[1],
                    target_pos[2] - current_pos[2]
                )
                # Simple P controller
                kp = 1.0  # proportional gain
                desired_vel = (
                    pos_error[0] * kp,
                    pos_error[1] * kp,
                    pos_error[2] * kp
                )
                # Clamp velocity to max speed
                max_speed = self.config.get('max_velocity', 5.0)
                speed = math.sqrt(desired_vel[0]**2 + desired_vel[1]**2 + desired_vel[2]**2)
                if speed > max_speed:
                    scale = max_speed / speed
                    desired_vel = (
                        desired_vel[0] * scale,
                        desired_vel[1] * scale,
                        desired_vel[2] * scale
                    )
                
                # Update output
                with self._lock:
                    self._desired_position = target_pos
                    self._desired_velocity = desired_vel
                    self._desired_yaw = target_yaw
                    
                self._stop_event.wait(0.05)
            except Exception as e:
                self.logger.error(f"Error in decision loop: {e}")
                self._stop_event.wait(0.1)
    
    def get_desired_state(self) -> tuple:
        """
        Get the desired state from the decision engine.
        
        Returns:
            tuple: (position_ned, velocity_ned, yaw)
                position_ned: (north, east, down) in meters
                velocity_ned: (north, east, down) in m/s
                yaw: in radians
        """
        with self._lock:
            return (self._desired_position, self._desired_velocity, self._desired_yaw)
    
    def _hold_position(self):
        """Hold current position."""
        with self._lock:
            if self._telemetry:
                current_pos = self._telemetry.get_local_position()
                if current_pos:
                    self._desired_position = current_pos
                self._desired_velocity = (0.0, 0.0, 0.0)
                self._desired_yaw = self._telemetry.get_heading() or 0.0

if __name__ == "__main__":
    # This module requires other modules to test
    pass
