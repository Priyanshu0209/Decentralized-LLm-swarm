#!/usr/bin/env python3
"""
Utility Functions for DroneAgent

Common helper functions used across modules.
"""

import math
import time
import yaml
from typing import Tuple, Optional, Any

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two points on Earth.
    
    Args:
        lat1, lon1: Latitude and longitude of point 1 in degrees
        lat2, lon2: Latitude and longitude of point 2 in degrees
        
    Returns:
        Distance in meters
    """
    R = 6371000.0  # Earth radius in meters
    
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def gps_to_ned(lat: float, lon: float, alt: float, home_lat: float, home_lon: float, home_alt: float) -> Tuple[float, float, float]:
    """Convert GPS coordinates to local NED coordinates relative to home."""
    R = 6371000.0
    lat1 = math.radians(home_lat)
    lat2 = math.radians(lat)
    dlat = lat2 - lat1
    dlon = math.radians(lon - home_lon)
    
    north = dlat * R
    east = dlon * R * math.cos(lat1)
    down = home_alt - alt
    return (north, east, down)

def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi].
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in radians
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between a minimum and maximum.
    
    Args:
        value: Value to clamp
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        
    Returns:
        Clamped value
    """
    return max(min_val, min(max_val, value))

def load_config(config_path: str) -> dict:
    """
    Load configuration from YAML file.
    
    Args:
        config_path: Path to YAML configuration file
        
    Returns:
        Dictionary containing configuration
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config if config is not None else {}
    except Exception as e:
        print(f"Error loading config from {config_path}: {e}")
        return {}

def save_config(config: dict, config_path: str):
    """
    Save configuration to YAML file.
    
    Args:
        config: Dictionary to save
        config_path: Path to YAML file
    """
    try:
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    except Exception as e:
        print(f"Error saving config to {config_path}: {e}")

def get_time_us() -> int:
    """Get current time in microseconds since epoch."""
    return int(time.time() * 1_000_000)

def get_time_ms() -> int:
    """Get current time in milliseconds since epoch."""
    return int(time.time() * 1_000)

def is_time_expired(timestamp_ms: int, timeout_ms: int) -> bool:
    """
    Check if a timestamp has expired.
    
    Args:
        timestamp_ms: Timestamp in milliseconds
        timeout_ms: Timeout in milliseconds
        
    Returns:
        True if current time - timestamp > timeout
    """
    return (get_time_ms() - timestamp_ms) > timeout_ms

def interpolate_point(p1: Tuple[float, float, float], 
                     p2: Tuple[float, float, float], 
                     t: float) -> Tuple[float, float, float]:
    """
    Linear interpolation between two points.
    
    Args:
        p1: First point (x, y, z)
        p2: Second point (x, y, z)
        t: Interpolation factor [0, 1]
        
    Returns:
        Interpolated point
    """
    return (
        p1[0] + t * (p2[0] - p1[0]),
        p1[1] + t * (p2[1] - p1[1]),
        p1[2] + t * (p2[2] - p1[2])
    )

def vec3_magnitude(v: Tuple[float, float, float]) -> float:
    """Calculate magnitude of a 3D vector."""
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def vec3_normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Normalize a 3D vector."""
    mag = vec3_magnitude(v)
    if mag == 0:
        return (0.0, 0.0, 0.0)
    return (v[0]/mag, v[1]/mag, v[2]/mag)

def vec3_dot(v1: Tuple[float, float, float], v2: Tuple[float, float, float]) -> float:
    """Dot product of two 3D vectors."""
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

def vec3_cross(v1: Tuple[float, float, float], v2: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Cross product of two 3D vectors."""
    return (
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]
    )

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """
    Convert Euler angles (in radians) to quaternion.
    
    Returns:
        Quaternion as (w, x, y, z)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return (w, x, y, z)

def quaternion_to_euler(w: float, x: float, y: float, z: float) -> tuple:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

if __name__ == "__main__":
    # Simple tests
    print("Testing haversine distance:")
    dist = haversine_distance(47.397742, 8.545594, 47.397900, 8.545000)
    print(f"Distance: {dist:.2f} meters")
    
    print("\nTesting angle normalization:")
    print(f"Normalize 3π: {normalize_angle(3 * math.pi)}")
    
    print("\nTesting clamp:")
    print(f"Clamp 15, 0, 10: {clamp(15, 0, 10)}")
    
    print("\nTesting vector operations:")
    v = (3.0, 4.0, 0.0)
    print(f"Vector {v} magnitude: {vec3_magnitude(v)}")
    print(f"Vector {v} normalized: {vec3_normalize(v)}")
