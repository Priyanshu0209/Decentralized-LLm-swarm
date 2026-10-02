import yaml
import os
import copy
from typing import Dict, Any, List

class ConfigurationError(Exception):
    """Raised when a mandatory configuration key is missing or invalid."""
    pass  # Intentionally empty block

def deep_merge(dict1: Dict[Any, Any], dict2: Dict[Any, Any]) -> Dict[Any, Any]:
    """
    Deep merges dict2 into dict1.
    If both values are dicts, they are merged recursively.
    Otherwise, the value in dict2 overwrites the value in dict1.
    """
    result = copy.deepcopy(dict1)
    for key, value in dict2.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result

class ConfigValidator:
    """Validates the structure, types, and logic of the configuration."""
    
    # Define mandatory keys and their expected types
    SCHEMA = {
        "drone_id": int,
        "mode": str,
        "connection.type": str,
        "connection.url": str,
        "udp_port": int,
        "mesh_port": int,
        "heartbeat_rate": (int, float),
        "neighbor_list": list,
        "planner_rate": (int, float),
        "telemetry_rate": (int, float),
        "failsafe.action": str,
        "logging.level": str,
        "vehicle.name": str,
        "mission.takeoff_altitude": (int, float),
        "mission.formation": str,
        "collision_avoidance.enabled": bool
    }

    @classmethod
    def validate(cls, config: Dict[str, Any]):
        cls._validate_schema(config)
        cls._validate_logic(config)

    @classmethod
    def _validate_schema(cls, config: Dict[str, Any]):
        for key_path, expected_type in cls.SCHEMA.items():
            keys = key_path.split('.')
            current = config
            for i, key in enumerate(keys):
                if not isinstance(current, dict) or key not in current:
                    missing_path = ".".join(keys[:i+1])
                    raise ConfigurationError(f"Missing required configuration key: {missing_path}")
                
                if i == len(keys) - 1:
                    # Final key, check type
                    val = current[key]
                    if not isinstance(val, expected_type):
                        raise ConfigurationError(
                            f"Invalid type for '{key_path}'. Expected {expected_type}, got {type(val).__name__}."
                        )
                current = current[key]

    @classmethod
    def _validate_logic(cls, config: Dict[str, Any]):
        import re
        
        # Validate mode
        if config["mode"] not in ["simulation", "real"]:
            raise ConfigurationError("mode must be 'simulation' or 'real'")
            
        # Validate ports
        if config["udp_port"] == config["mesh_port"]:
            raise ConfigurationError("udp_port and mesh_port cannot be the same")
            
        if config["udp_port"] <= 1024 or config["mesh_port"] <= 1024:
            raise ConfigurationError("Ports should be > 1024 to avoid privilege issues")
            
        # Validate URLs/IPs
        url = config["connection"]["url"]
        if config["connection"]["type"] == "udp":
            if not url.startswith("udp://"):
                raise ConfigurationError("UDP connection URL must start with udp://")
                
        # Validate failsafe
        if config["failsafe"]["action"] not in ["rtl", "land", "hover"]:
            raise ConfigurationError("Invalid failsafe.action")
            
        # Validate logging level
        if config["logging"]["level"] not in ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]:
            raise ConfigurationError("Invalid logging.level")

def load_config(config_paths: List[str], validate: bool = True) -> Dict[str, Any]:
    """
    Load and merge multiple YAML configuration files.
    The latter files in the list override settings from the earlier files.
    
    Args:
        config_paths: List of paths to YAML configuration files.
        validate: Whether to run the ConfigValidator on the final merged config.
        
    Returns:
        Merged configuration dictionary.
    """
    merged_config = {}
    
    for path in config_paths:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Configuration file not found: {path}")
            
        with open(path, 'r') as f:
            config_data = yaml.safe_load(f)
            if config_data:
                merged_config = deep_merge(merged_config, config_data)
                
    if validate:
        ConfigValidator.validate(merged_config)
        
    return merged_config

if __name__ == "__main__":
    # Test
    paths = ["config/base.yaml", "config/simulation.yaml", "config/drone1.yaml"]
    try:
        config = load_config(paths)
        print("Config validated successfully.")
    except Exception as e:
        print(f"Error: {e}")
