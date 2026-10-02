#!/usr/bin/env python3
"""
Flight Control Manager for DroneAgent

The single source of truth for commanding the PX4 backend.
Subsystems like StateMachine and MovementController publish desired states or velocities,
and this manager safely applies them, handling mode transitions and arbitration.
"""

import threading
import time
import logging
import math

class FlightControlManager:
    def __init__(self, config: dict, logger: logging.Logger, backend=None):
        self.config = config
        self.logger = logger.getChild("FCM")
        self.backend = backend
        
        self._lock = threading.RLock()
        
        # Desired State
        self._desired_mode = "BOOT"
        self._desired_velocity = None  # (north, east, down, yaw)
        self._desired_velocity_body = None # (forward, right, down, yawspeed)
        
        self._stop_event = threading.Event()
        self._thread = None
        self.control_rate = config.get('control_rate', 50)  # Hz
        
        self.logger.info("Flight Control Manager initialized.")

    def set_desired_mode(self, mode: str):
        """Set the high-level desired mode (e.g., TAKEOFF, LAND, RTL, HOVER, OFFBOARD, EMERGENCY, ARM)."""
        with self._lock:
            if self._desired_mode != mode:
                self.logger.info(f"Desired mode changed: {self._desired_mode} -> {mode}")
                self._desired_mode = mode
                
                # If we switch out of offboard, clear velocity setpoints
                if mode not in ["OFFBOARD", "HOVER"]:
                    self._desired_velocity = None
                    self._desired_velocity_body = None

    def set_desired_velocity(self, n: float, e: float, d: float, yaw_deg: float):
        """Set desired NED velocity."""
        with self._lock:
            self._desired_velocity = (n, e, d, yaw_deg)
            self._desired_velocity_body = None
            if self._desired_mode not in ["TAKEOFF", "LAND", "RTL", "EMERGENCY"]:
                self._desired_mode = "OFFBOARD"

    def set_desired_velocity_body(self, forward: float, right: float, down: float, yawspeed: float):
        """Set desired body velocity."""
        with self._lock:
            self._desired_velocity_body = (forward, right, down, yawspeed)
            self._desired_velocity = None
            if self._desired_mode not in ["TAKEOFF", "LAND", "RTL", "EMERGENCY"]:
                self._desired_mode = "OFFBOARD"

    def arm(self):
        self.set_desired_mode("ARM")
        
    def disarm(self):
        self.set_desired_mode("DISARM")

    def takeoff(self, alt: float):
        self.config['mission']['takeoff_altitude'] = alt
        self.set_desired_mode("TAKEOFF")
        
    def land(self):
        self.set_desired_mode("LAND")
        
    def rtl(self):
        self.set_desired_mode("RTL")
        
    def emergency_stop(self):
        self.set_desired_mode("EMERGENCY")
        
    def hover(self):
        self.set_desired_mode("HOVER")

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._control_loop, name="FCM_Loop", daemon=True)
        self._thread.start()
        self.logger.info("Flight Control Manager started.")

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.logger.info("Flight Control Manager stopped.")

    def _control_loop(self):
        """Continuously apply the desired state to the backend."""
        last_mode = None
        
        while not self._stop_event.is_set():
            start_time = time.time()
            try:
                with self._lock:
                    mode = self._desired_mode
                    vel = self._desired_velocity
                    vel_body = self._desired_velocity_body
                    
                if not self.backend or not self.backend.is_connected():
                    time.sleep(0.1)
                    continue
                    
                # Handle one-shot actions on mode transition
                if mode != last_mode:
                    try:
                        self._handle_mode_transition(mode)
                    except Exception as e:
                        self.logger.error(f"Error handling transition to {mode}: {e}")
                    last_mode = mode
                    
                # Handle continuous offboard streaming
                if mode == "OFFBOARD" or mode == "HOVER":
                    try:
                        if not self.backend.is_offboard_active():
                            self.backend.start_offboard()
                            
                        if mode == "HOVER":
                            self.backend.move_velocity(0.0, 0.0, 0.0, 0.0)
                        elif vel_body is not None:
                            self.backend.move_velocity_body(*vel_body)
                        elif vel is not None:
                            self.backend.move_velocity(*vel)
                        else:
                            self.backend.move_velocity(0.0, 0.0, 0.0, 0.0)
                    except Exception as e:
                        self.logger.error(f"Error in offboard control: {e}")
                        
                # Enforce control rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.control_rate) - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Error in FCM loop: {e}")
                time.sleep(0.1)
                
    def _handle_mode_transition(self, mode: str):
        self.logger.info(f"FCM executing backend transition for mode: {mode}")
        if mode not in ["OFFBOARD", "HOVER"] and self.backend.is_offboard_active():
            self.backend.stop_offboard()
            
        if mode == "ARM":
            self.backend.arm()
        elif mode == "DISARM":
            self.backend.disarm()
        elif mode == "TAKEOFF":
            alt = self.config.get('mission', {}).get('takeoff_altitude', 10.0)
            self.backend.takeoff(alt)
        elif mode == "LAND":
            self.backend.land()
        elif mode == "RTL":
            self.backend.rtl()
        elif mode == "EMERGENCY":
            self.backend.emergency_stop()
        elif mode == "HOVER":
            if not self.backend.is_offboard_active():
                self.backend.start_offboard()
            self.backend.move_velocity(0.0, 0.0, 0.0, 0.0)
