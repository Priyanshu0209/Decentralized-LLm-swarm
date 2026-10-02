import logging
from backend_interface import BackendInterface
from simulation_backend import SimulationBackend
from real_backend import RealDroneBackend

def BackendFactory(config: dict, logger: logging.Logger) -> BackendInterface:
    """
    Instantiates and returns the appropriate BackendInterface backend
    based on the configuration.
    
    Args:
        config: Configuration dictionary.
        logger: Logger instance.
        
    Returns:
        BackendInterface implementation (SimulationBackend or RealDroneBackend).
    """
    mode = config.get("mode", "simulation").lower()
    
    if mode in ["simulation", "airsim"]:
        logger.info(f"BackendFactory: Creating SimulationBackend for {mode}")
        return SimulationBackend(config, logger)
    elif mode == "real":
        logger.info("BackendFactory: Creating RealDroneBackend")
        return RealDroneBackend(config, logger)
    else:
        logger.error(f"BackendFactory: Unknown mode '{mode}', defaulting to SimulationBackend")
        return SimulationBackend(config, logger)
