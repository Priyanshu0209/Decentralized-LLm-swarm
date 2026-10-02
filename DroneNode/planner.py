#!/usr/bin/env python3
"""
Planner Module for DroneAgent

Generates smooth trajectories to reach target states while respecting 
dynamic constraints (velocity, acceleration limits).
"""

import math
from typing import Tuple, Optional
import logging

class Planner:
    def __init__(self, config: dict, logger: logging.Logger):
        """
        Initialize planner.
        
        Args:
            config: Configuration dictionary
            logger: Logger instance
        """
        self.config = config
        self.logger = logger.getChild("Planner")
        self.max_velocity = config.get('max_velocity', 5.0)      # m/s
        self.max_acceleration = config.get('max_acceleration', 2.0)  # m/s^2
        self.max_jerk = config.get('max_jerk', 5.0)             # m/s^3 (optional)
        self.yaw_rate_limit = math.radians(config.get('max_yaw_rate', 30.0))  # rad/s
        
        # State for trajectory generation
        self._current_position = (0.0, 0.0, 0.0)   # north, east, down
        self._current_velocity = (0.0, 0.0, 0.0)
        self._current_acceleration = (0.0, 0.0, 0.0)
        self._target_position = (0.0, 0.0, 0.0)
        self._target_velocity = (0.0, 0.0, 0.0)
        self._current_yaw = 0.0
        self._target_yaw = 0.0
        
        self._last_update_time = 0.0
    
    def update_state(self, position: Tuple[float, float, float], 
                     velocity: Tuple[float, float, float],
                     yaw: float):
        """
        Update the current state of the drone.
        
        Args:
            position: (north, east, down) in meters
            velocity: (north, east, down) in m/s
            yaw: heading in radians
        """
        self._current_position = position
        self._current_velocity = velocity
        self._current_yaw = yaw
        self._last_update_time = self._last_update_time or 0.0  # Initialize if needed
    
    def set_target(self, position: Tuple[float, float, float], 
                   velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                   yaw: float = 0.0):
        """
        Set the target state to reach.
        
        Args:
            position: (north, east, down) target in meters
            velocity: (north, east, down) target velocity in m/s (default: hover)
            yaw: target heading in radians
        """
        self._target_position = position
        self._target_velocity = velocity
        self._target_yaw = yaw
    
    def get_control_command(self, dt: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], float]:
        """
        Compute the control command (position, velocity, yaw) to send to the controller.
        This uses a simple trapezoidal velocity profile for each axis.
        
        Args:
            dt: Time step since last call in seconds
            
        Returns:
            Tuple of (position_command, velocity_command, yaw_command)
                position_command: (north, east, down) in meters
                velocity_command: (north, east, down) in m/s
                yaw_command: heading in radians
        """
        if self._last_update_time == 0.0:
            # First call, initialize
            self._last_update_time = dt
            return self._current_position, self._current_velocity, self._current_yaw
        
        # Update time
        current_time = self._last_update_time + dt
        self._last_update_time = current_time
        
        # For each axis, compute desired velocity using a simple guidance law
        # We'll implement a proportional-derivative (PD) controller for position
        # and feedforward velocity.
        
        # Position gains
        kp_pos = 2.0  # proportional gain
        kd_pos = 1.0  # derivative gain
        
        # Yaw gains
        kp_yaw = 2.0
        kd_yaw = 1.0
        
        # Calculate position error
        pos_error = (
            self._target_position[0] - self._current_position[0],
            self._target_position[1] - self._current_position[1],
            self._target_position[2] - self._current_position[2]
        )
        
        # Calculate velocity error (desired velocity minus current velocity)
        vel_error = (
            self._target_velocity[0] - self._current_velocity[0],
            self._target_velocity[1] - self._current_velocity[1],
            self._target_velocity[2] - self._current_velocity[2]
        )
        
        # PD control for velocity command
        vx_cmd = kp_pos * pos_error[0] + kd_pos * vel_error[0] + self._target_velocity[0]
        vy_cmd = kp_pos * pos_error[1] + kd_pos * vel_error[1] + self._target_velocity[1]
        vz_cmd = kp_pos * pos_error[2] + kd_pos * vel_error[2] + self._target_velocity[2]
        
        # Actually, let's do a simpler approach: compute desired acceleration to reach target
        # Then integrate to get velocity, respecting limits.
        
        # We'll compute the desired acceleration using a double integrator model
        # with saturation.
        
        # For simplicity in this implementation, we'll use a basic approach:
        # If far from target, go at max speed; if close, slow down.
        
        # Calculate distance to target
        dx = self._target_position[0] - self._current_position[0]
        dy = self._target_position[1] - self._current_position[1]
        dz = self._target_position[2] - self._current_position[2]
        dist_xy = math.sqrt(dx*dx + dy*dy)
        dist_z = abs(dz)
        
        # Slow down when approaching target
        slow_down_dist = 2.0  # meters
        speed_factor = 1.0
        if dist_xy < slow_down_dist:
            speed_factor = dist_xy / slow_down_dist
        if dist_z < slow_down_dist:
            speed_factor = min(speed_factor, dist_z / slow_down_dist)
        
        # Desired velocity towards target
        max_speed_xy = self.max_velocity * speed_factor
        max_speed_z = self.max_velocity * speed_factor  # Assume same limit for vertical
        
        # Avoid division by zero
        if dist_xy > 0.1:
            vx_des = (dx / dist_xy) * max_speed_xy
            vy_des = (dy / dist_xy) * max_speed_xy
        else:
            vx_des = 0.0
            vy_des = 0.0
        
        if dist_z > 0.1:
            vz_des = (dz / dist_z) * max_speed_z if dz > 0 else -(dz / dist_z) * max_speed_z
            # Note: down is positive, so if target is above (negative dz), we want negative vz (up)
            # Actually, if target is above, dz = target_down - current_down < 0 (since up is negative down)
            # So we want to move up (negative velocity in down axis)
            # Let's correct: vz_des = (dz / dist_z) * max_speed_z, but we want to move in the direction of dz
            # If dz is negative (target above), vz_des should be negative (up)
            # So:
            vz_des = (dz / dist_z) * max_speed_z
        else:
            vz_des = 0.0
        
        # Limit velocity to maximum
        vx_cmd = self._limit_velocity(vx_des, self._current_velocity[0], self.max_acceleration, dt)
        vy_cmd = self._limit_velocity(vy_des, self._current_velocity[1], self.max_acceleration, dt)
        vz_cmd = self._limit_velocity(vz_des, self._current_velocity[2], self.max_acceleration, dt)
        
        # Yaw control
        yaw_error = self._target_yaw - self._current_yaw
        # Normalize yaw error to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        max_yaw_change = self.yaw_rate_limit * dt
        if yaw_error > max_yaw_change:
            yaw_error = max_yaw_change
        elif yaw_error < -max_yaw_change:
            yaw_error = -max_yaw_change
        yaw_cmd = self._current_yaw + yaw_error
        
        # Position command: we can either send position setpoint or velocity setpoint
        # For offboard mode, we can send velocity setpoint directly
        # We'll return velocity command for now; the controller can handle it
        # But the offboard controller expects either position or velocity setpoint.
        # We'll return both and let the caller decide.
        # For simplicity, we'll return position command as current position + velocity * dt
        # and velocity command as calculated.
        pos_cmd = (
            self._current_position[0] + vx_cmd * dt,
            self._current_position[1] + vy_cmd * dt,
            self._current_position[2] + vz_cmd * dt
        )
        vel_cmd = (vx_cmd, vy_cmd, vz_cmd)
        yaw_cmd = yaw_cmd
        
        return (pos_cmd, vel_cmd, yaw_cmd)
    
    def _limit_velocity(self, desired: float, current: float, max_accel: float, dt: float) -> float:
        """
        Limit the change in velocity based on maximum acceleration.
        
        Args:
            desired: Desired velocity
            current: Current velocity
            max_accel: Maximum acceleration (m/s^2)
            dt: Time step
            
        Returns:
            Limited velocity
        """
        max_delta_v = max_accel * dt
        delta_v = desired - current
        if delta_v > max_delta_v:
            return current + max_delta_v
        elif delta_v < -max_delta_v:
            return current - max_delta_v
        else:
            return desired
    
    def reset(self):
        """Reset planner state."""
        self._current_position = (0.0, 0.0, 0.0)
        self._current_velocity = (0.0, 0.0, 0.0)
        self._current_acceleration = (0.0, 0.0, 0.0)
        self._target_position = (0.0, 0.0, 0.0)
        self._target_velocity = (0.0, 0.0, 0.0)
        self._current_yaw = 0.0
        self._target_yaw = 0.0
        self._last_update_time = 0.0

if __name__ == "__main__":
    # Simple test
    import logging
    import time
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Test")
    
    config = {
        'max_velocity': 5.0,
        'max_acceleration': 2.0,
        'max_yaw_rate': 30.0
    }
    
    planner = Planner(config, logger)
    
    # Simulate starting at origin
    planner.update_state((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    
    # Set target to 10m north, 0 east, 0 down
    planner.set_target((10.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    
    # Simulate 2 seconds of flight at 50Hz
    for i in range(100):
        pos_cmd, vel_cmd, yaw_cmd = planner.get_control_command(0.02)  # 20ms step
        print(f"t={i*0.02:.2f}s: pos_cmd={pos_cmd}, vel_cmd={vel_cmd}, yaw={yaw_cmd:.2f}")
        # In a real simulation, we would update the state based on vel_cmd
        # For simplicity, we'll just integrate
        # But we'll skip for this test
        time.sleep(0.01)
