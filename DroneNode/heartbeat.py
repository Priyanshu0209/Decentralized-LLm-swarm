#!/usr/bin/env python3
"""
Heartbeat Module for DroneAgent

Responsible for generating and sending periodic heartbeat packets containing
the drone's current state (position, velocity, battery, etc.) to neighbors.
"""

import threading
import time
import logging
from typing import Optional
from packet import DronePacket

class Heartbeat:
    def __init__(self, config: dict, logger: logging.Logger,
                 communication=None, telemetry=None, state_machine=None):
        """
        Initialize heartbeat module.
        
        Args:
            config: Configuration dictionary
            logger: Logger instance
            communication: Communication module instance (for sending packets)
            telemetry: Telemetry module instance (for getting drone state)
            state_machine: State machine instance (for getting current state)
        """
        self.config = config
        self.logger = logger.getChild("Heartbeat")
        self.communication = communication
        self.telemetry = telemetry
        self.state_machine = state_machine
        
        self.drone_id = config['drone_id']
        self.heartbeat_rate = config.get('heartbeat_rate', 10.0)  # Hz
        self.interval = 1.0 / self.heartbeat_rate if self.heartbeat_rate > 0 else 1.0
        
        self._running = False
        self._thread = None
        self._packet_sequence = 0
        self._last_gps = (0.0, 0.0, 0.0)
        self._last_local_pos = (0.0, 0.0, 0.0)
        self._last_velocity = (0.0, 0.0, 0.0)
        self._last_heading = 0.0
        self._battery = 1.0
        self._health = 1.0
        self._mission_id = 0
        self._formation_index = 0
        self._status_flags = 0
    
    def start(self):
        """Start heartbeat (set running flag)."""
        self._running = True
        self._thread = threading.Thread(target=self.monitor_loop, daemon=True, name="Heartbeat")
        self._thread.start()
        self.logger.info(f"Heartbeat started at {self.heartbeat_rate} Hz")

    def stop(self):
        """Stop heartbeat (clear running flag)."""
        self._running = False
        self.logger.info("Heartbeat stopped")
    
    def monitor_loop(self):
        """Main loop: generate and send heartbeat packet at fixed interval."""
        while self._running:
            start_time = time.time()
            try:
                self._send_heartbeat()
            except Exception as e:
                self.logger.error(f"Error in heartbeat: {e}")

            # Sleep for the remainder of the interval
            elapsed = time.time() - start_time
            sleep_time = max(0, self.interval - elapsed)
            time.sleep(sleep_time)
    
    def _send_heartbeat(self):
        """Construct and send a heartbeat packet."""
        # Update state from subsystems if available
        self._update_state()
        
        # Create packet
        packet = DronePacket(
            drone_id=self.drone_id,
            timestamp=int(time.time() * 1_000_000),  # microseconds
            gps_lat=self._last_gps[0],
            gps_lon=self._last_gps[1],
            gps_alt=self._last_gps[2],
            local_pos_x=self._last_local_pos[0],
            local_pos_y=self._last_local_pos[1],
            local_pos_z=self._last_local_pos[2],
            velocity_x=self._last_velocity[0],
            velocity_y=self._last_velocity[1],
            velocity_z=self._last_velocity[2],
            heading=self._last_heading,  # radians
            altitude=-self._last_local_pos[2],  # assuming down is negative, so altitude = -z
            battery=self._battery,
            health=self._health,
            mission_id=self._mission_id,
            formation_index=self._formation_index,
            status_flags=self._status_flags,
            packet_number=self._packet_sequence
        )
        
        # Send via communication module
        if self.communication:
            self.communication.send_packet(packet)
            self._packet_sequence += 1
        else:
            self.logger.debug("No communication module available to send heartbeat")
    
    def _update_state(self):
        """Update internal state from telemetry and other subsystems."""
        # Update from telemetry if available
        if self.telemetry:
            gps = self.telemetry.get_gps()
            if gps is not None: self._last_gps = gps
            
            local_pos = self.telemetry.get_local_position()
            if local_pos is not None: self._last_local_pos = local_pos
            
            vel = self.telemetry.get_velocity()
            if vel is not None: self._last_velocity = vel
            
            heading = self.telemetry.get_heading()
            if heading is not None: self._last_heading = heading
            
            batt = self.telemetry.get_battery()
            if batt is not None: self._battery = batt
            
            health = self.telemetry.get_health()
            if health is not None: self._health = health
        
        # Update from state machine if available
        if self.state_machine:
            flags = self.state_machine.get_status_flags()
            if flags is not None: self._status_flags = flags
            
            mission_id = self.state_machine.get_mission_id()
            if mission_id is not None: self._mission_id = mission_id
            
            # Formation index might come from formation or mission
            formation_index = self.state_machine.get_formation_index()
            if formation_index is not None: self._formation_index = formation_index
    
    # Setter methods for testing or external updates
    def set_gps(self, lat: float, lon: float, alt: float):
        self._last_gps = (lat, lon, alt)
    
    def set_local_position(self, x: float, y: float, z: float):
        self._last_local_pos = (x, y, z)
    
    def set_velocity(self, vx: float, vy: float, vz: float):
        self._last_velocity = (vx, vy, vz)
    
    def set_heading(self, heading_rad: float):
        self._last_heading = heading_rad
    
    def set_battery(self, battery: float):
        self._battery = max(0.0, min(1.0, battery))
    
    def set_health(self, health: float):
        self._health = max(0.0, min(1.0, health))
    
    def set_mission_id(self, mission_id: int):
        self._mission_id = mission_id
    
    def set_formation_index(self, index: int):
        self._formation_index = index
    
    def set_status_flags(self, flags: int):
        self._status_flags = flags

if __name__ == "__main__":
    # Simple test
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Test")
    
    config = {
        'drone_id': 1,
        'heartbeat_rate': 10.0
    }
    
    hb = Heartbeat(config, logger)
    hb.set_gps(47.397742, 8.545594, 500.0)
    hb.set_local_position(0.0, 0.0, -50.0)  # 50m above home
    hb.set_velocity(0.0, 0.0, 0.0)
    hb.set_heading(0.0)
    hb.set_battery(0.9)
    hb.set_health(0.95)
    hb.set_mission_id(1)
    hb.set_formation_index(0)
    hb.set_status_flags(0x03)  # Armed and guided
    
    hb.start()
    try:
        time.sleep(2)
    finally:
        hb.stop()
