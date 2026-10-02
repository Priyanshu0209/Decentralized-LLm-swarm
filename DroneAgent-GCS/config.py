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
    """Validates the structure and mandatory keys of the configuration."""
    
    # Define mandatory keys. Dot notation implies nested dictionaries.
    REQUIRED_KEYS = [
        "drone_id",
        "mode",
        "connection.type",
        "connection.url",
        "udp_port",
        "mesh_port",
        "heartbeat_rate",
        "neighbor_list",
        "planner_rate",
        "telemetry_rate",
        "failsafe.action",
        "logging.level",
        "vehicle.name",
        "mission.takeoff_altitude",
        "mission.formation",
        "collision_avoidance.enabled"
    ]

    @classmethod
    def validate(cls, config: Dict[str, Any]):
        for key_path in cls.REQUIRED_KEYS:
            keys = key_path.split('.')
            current = config
            for i, key in enumerate(keys):
                if not isinstance(current, dict) or key not in current:
                    missing_path = ".".join(keys[:i+1])
                    raise ConfigurationError(f"Missing required configuration key: {missing_path}")
                current = current[key]

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
