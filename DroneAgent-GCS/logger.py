#!/usr/bin/env python3
"""
Logger Module for DroneAgent

Configures logging to multiple files and console.
"""

import logging
import os
from datetime import datetime
from logging.handlers import RotatingFileHandler

class DroneLogger:
    def __init__(self, drone_id: int, log_dir: str = "./logs", config: dict = None):
        """
        Initialize logger for a drone.
        
        Args:
            drone_id: ID of the drone (for log file naming)
            log_dir: Directory to store log files
            config: Optional config dict to load logging params
        """
        self.drone_id = drone_id
        self.log_dir = log_dir
        self.config = config or {}
        
        # Create log directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create a logger
        self.logger = logging.getLogger(f"Drone{drone_id}")
        self.logger.setLevel(logging.DEBUG)
        
        # Clear any existing handlers
        self.logger.handlers.clear()
        
        # Create formatters
        detailed_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        simple_formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - %(message)s'
        )
        
        # Console handler (info and above)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(simple_formatter)
        self.logger.addHandler(console_handler)
        
        # File handlers for different log types
        log_types = [
            ('communication', logging.DEBUG),
            ('mission', logging.INFO),
            ('decision', logging.INFO),
            ('planner', logging.INFO),
            ('collision', logging.WARNING),
            ('telemetry', logging.INFO),
            ('error', logging.ERROR)
        ]
        
        # Load rotation config from yaml
        logging_config = self.config.get('logging', {})
        max_bytes = logging_config.get('maxBytes', 10485760)  # default 10MB
        backup_count = logging_config.get('backupCount', 5)   # default 5 backups
        
        for log_name, level in log_types:
            log_file = os.path.join(self.log_dir, f"drone{drone_id}_{log_name}.log")
            file_handler = RotatingFileHandler(
                log_file, maxBytes=max_bytes, backupCount=backup_count
            )
            file_handler.setLevel(level)
            file_handler.setFormatter(detailed_formatter)
            self.logger.addHandler(file_handler)
        
        # Prevent propagation to root logger to avoid duplicate logs
        self.logger.propagate = False
        
        self.logger.info(f"Logger initialized for drone {drone_id}")
    
    def get_logger(self) -> logging.Logger:
        """Get the configured logger instance."""
        return self.logger
    
    # Convenience methods for logging to specific files (if needed)
    # In practice, we rely on the logger's name and level routing via handlers
    # But we can also create specific loggers if desired.

if __name__ == "__main__":
    # Simple test
    logger = DroneLogger(1).get_logger()
    logger.info("This is an info message")
    logger.debug("This is a debug message")
    logger.warning("This is a warning")
    logger.error("This is an error")
