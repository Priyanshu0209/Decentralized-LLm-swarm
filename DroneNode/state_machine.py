#!/usr/bin/env python3
"""
State Machine for DroneAgent

Defines the drone's behavior states and transitions:
BOOT, CONNECTING, DISCOVERING, WAITING_MISSION, READY, ARMING, TAKEOFF,
OFFBOARD, FORMATION, MISSION, RTL, LAND, EMERGENCY, FAILSAFE

Each state has entry/exit actions and transition conditions.
"""

import threading
import time
from enum import Enum, auto
from typing import Optional, Callable, Dict, Any
import logging
import inspect
from mavsdk_backend_base import ControlPriority

class DroneState(Enum):
    BOOT = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    READY = auto()
    ARMING = auto()
    ARMED = auto()
    TAKEOFF = auto()
    HOVER = auto()
    MISSION = auto()
    FORMATION = auto()
    RTL = auto()
    LANDING = auto()
    DISARM = auto()
    EMERGENCY = auto()

class StateMachine:
    def __init__(self, config: dict, logger: logging.Logger, backend=None, telemetry=None, communication=None, fcm=None):
        """
        Initialize the state machine.
        
        Args:
            config: Configuration dictionary
            logger: Logger instance
        """
        self.config = config
        self.logger = logger.getChild("StateMachine")
        self.state = DroneState.BOOT
        self._lock = threading.RLock()
        self._transition_callbacks: Dict[tuple, Callable] = {}
        self._state_callbacks: Dict[DroneState, tuple] = {}  # state -> (enter, exit)
        self._stop_event = threading.Event()
        
        # Initialize hardware backend and subsystems for flight control
        self.backend = backend
        self.telemetry = telemetry
        self.communication = communication
        self.fcm = fcm

        # State-related data that might be updated by other modules
        self._status_flags = 0
        self._mission_id = 0
        self._formation_index = 0
        self._running = False
        
        # Define state transition table: (from_state, trigger) -> to_state
        self._transitions = {
            # Boot sequence
            (DroneState.BOOT, 'system_ready'): DroneState.CONNECTING,
            (DroneState.CONNECTING, 'backend_connected'): DroneState.CONNECTED,
            (DroneState.CONNECTED, 'ready'): DroneState.READY,
            
            # Arming and takeoff
            (DroneState.READY, 'arm_command'): DroneState.ARMING,
            (DroneState.ARMING, 'armed'): DroneState.ARMED,
            (DroneState.ARMED, 'takeoff_command'): DroneState.TAKEOFF,
            (DroneState.TAKEOFF, 'takeoff_complete'): DroneState.HOVER,
            (DroneState.TAKEOFF, 'hover'): DroneState.HOVER,
            
            # Offboard, Formation and mission
            (DroneState.HOVER, 'formation_ready'): DroneState.FORMATION,
            (DroneState.HOVER, 'formation_command'): DroneState.FORMATION,
            (DroneState.HOVER, 'mission_start'): DroneState.MISSION,
            (DroneState.FORMATION, 'mission_start'): DroneState.MISSION,
            (DroneState.MISSION, 'formation_ready'): DroneState.FORMATION,
            (DroneState.MISSION, 'formation_command'): DroneState.FORMATION,
            
            # Mission execution
            (DroneState.MISSION, 'mission_complete'): DroneState.HOVER,
            (DroneState.MISSION, 'pause_command'): DroneState.HOVER,
            (DroneState.FORMATION, 'pause_command'): DroneState.HOVER,
            (DroneState.RTL, 'pause_command'): DroneState.HOVER,
            (DroneState.LANDING, 'pause_command'): DroneState.HOVER,
            
            # Return and landing
            (DroneState.HOVER, 'rtl_command'): DroneState.RTL,
            (DroneState.MISSION, 'rtl_command'): DroneState.RTL,
            (DroneState.FORMATION, 'rtl_command'): DroneState.RTL,
            (DroneState.RTL, 'home_reached'): DroneState.LANDING,
            
            (DroneState.HOVER, 'land_command'): DroneState.LANDING,
            (DroneState.MISSION, 'land_command'): DroneState.LANDING,
            (DroneState.FORMATION, 'land_command'): DroneState.LANDING,
            (DroneState.RTL, 'land_command'): DroneState.LANDING,
            (DroneState.TAKEOFF, 'land_command'): DroneState.LANDING,
            (DroneState.LANDING, 'landed'): DroneState.DISARM,
            (DroneState.RTL, 'landed'): DroneState.DISARM,
            (DroneState.DISARM, 'ready'): DroneState.READY,
            
            # Direct disarm (only allowed before takeoff or safely on ground)
            (DroneState.ARMED, 'disarm_command'): DroneState.DISARM,
            (DroneState.READY, 'disarm_command'): DroneState.READY,
            (DroneState.DISARM, 'arm_command'): DroneState.ARMING,
            
            # Emergency handling (can happen from any state)
            (DroneState.BOOT, 'emergency'): DroneState.EMERGENCY,
            (DroneState.CONNECTING, 'emergency'): DroneState.EMERGENCY,
            (DroneState.CONNECTED, 'emergency'): DroneState.EMERGENCY,
            (DroneState.READY, 'emergency'): DroneState.EMERGENCY,
            (DroneState.ARMING, 'emergency'): DroneState.EMERGENCY,
            (DroneState.ARMED, 'emergency'): DroneState.EMERGENCY,
            (DroneState.TAKEOFF, 'emergency'): DroneState.EMERGENCY,
            (DroneState.HOVER, 'emergency'): DroneState.EMERGENCY,
            (DroneState.FORMATION, 'emergency'): DroneState.EMERGENCY,
            (DroneState.MISSION, 'emergency'): DroneState.EMERGENCY,
            (DroneState.RTL, 'emergency'): DroneState.EMERGENCY,
            (DroneState.LANDING, 'emergency'): DroneState.EMERGENCY,
            (DroneState.DISARM, 'emergency'): DroneState.EMERGENCY,
            
            # Recovery from failsafe/emergency
            (DroneState.EMERGENCY, 'recover'): DroneState.READY,
        }
        
        # Set up default state callbacks
        self._setup_state_callbacks()
        
        self.logger.info("StateMachine initialized")
    
    def _setup_state_callbacks(self):
        """Define entry and exit actions for each state."""
        self._state_callbacks = {
            DroneState.BOOT: (self._enter_boot, self._exit_boot),
            DroneState.CONNECTING: (self._enter_connecting, self._exit_connecting),
            DroneState.CONNECTED: (self._enter_connected, self._exit_connected),
            DroneState.READY: (self._enter_ready, self._exit_ready),
            DroneState.ARMING: (self._enter_arming, self._exit_arming),
            DroneState.ARMED: (self._enter_armed, self._exit_armed),
            DroneState.TAKEOFF: (self._enter_takeoff, self._exit_takeoff),
            DroneState.HOVER: (self._enter_hover, self._exit_hover),
            DroneState.FORMATION: (self._enter_formation, self._exit_formation),
            DroneState.MISSION: (self._enter_mission, self._exit_mission),
            DroneState.RTL: (self._enter_rtl, self._exit_rtl),
            DroneState.LANDING: (self._enter_landing, self._exit_landing),
            DroneState.DISARM: (self._enter_disarm, self._exit_disarm),
            DroneState.EMERGENCY: (self._enter_emergency, self._exit_emergency),
        }
    
    def transition(self, trigger: str, context: Optional[Dict[str, Any]] = None) -> bool:
        """
        Trigger a state transition.
        
        Args:
            trigger: The trigger name (e.g., 'arm_command')
            context: Optional context data for the transition
            
        Returns:
            True if transition occurred, False if no transition defined
        """
        with self._lock:
            current_state = self.state
            
            self.logger.debug(f"[STATE_MACHINE] BEFORE TRANSITION: Current State: {current_state.name}, Trigger: {trigger}, Arguments: {context}")
            
            # Special case for mission complete returning to formation
            if current_state == DroneState.MISSION and trigger == 'mission_complete':
                if getattr(self, 'previous_state', None) == DroneState.FORMATION:
                    next_state = DroneState.FORMATION
                    key_exists = True
                else:
                    next_state = DroneState.RTL
                    key_exists = True
            else:
                key = (current_state, trigger)
                if key in self._transitions:
                    next_state = self._transitions[key]
                    key_exists = True
                else:
                    key_exists = False
                    
            if key_exists:
                self.logger.info(f"Transition: {current_state.name} --[{trigger}]--> {next_state.name}")
                
                # Exit current state
                exit_action = self._state_callbacks[current_state][1]
                exit_cb_name = exit_action.__name__ if exit_action else "None"
                if exit_action:
                    try:
                        exit_action()
                    except Exception as e:
                        self.logger.exception(f"Exception in exit callback {exit_cb_name}: {e}")
                
                # Enter new state
                self.previous_state = self.state
                self.state = next_state
                enter_action = self._state_callbacks[next_state][0]
                enter_cb_name = enter_action.__name__ if enter_action else "None"
                
                self.logger.debug(f"[STATE_MACHINE] AFTER TRANSITION: Success: True, New State: {next_state.name}, Entry callback: {enter_cb_name}, Exit callback: {exit_cb_name}")
                
                if enter_action:
                    try:
                        sig = inspect.signature(enter_action)
                        if len(sig.parameters) > 0:
                            enter_action(context)
                        else:
                            enter_action()
                    except Exception as e:
                        self.logger.exception(f"Exception in entry callback {enter_cb_name}: {e}")
                
                return True
            else:
                self.logger.debug(f"[STATE_MACHINE] AFTER TRANSITION: Success: False, No transition defined for {current_state.name} --[{trigger}]--> ?")
                return False
    
    def get_state(self) -> DroneState:
        """Get current state."""
        return self.state
    
    def get_status_flags(self) -> int:
        return self._status_flags
        
    def set_status_flags(self, flags: int):
        self._status_flags = flags

    def get_mission_id(self) -> int:
        return self._mission_id

    def set_mission_id(self, mission_id: int):
        self._mission_id = mission_id
        
    def get_formation_index(self) -> int:
        return self._formation_index
        
    def set_formation_index(self, index: int):
        self._formation_index = index
    
    # State entry/exit actions
    def _enter_boot(self, context=None):
        self.logger.info("Booting up...")
        # Initialize hardware, load config, etc.
    
    def _exit_boot(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_connecting(self, context=None):
        self.logger.info("Connecting to Hardware Backend...")
        # Start Backend connection
        if self.backend:
            self.backend.connect()
    
    def _exit_connecting(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_landing(self, context=None):
        self.logger.info("Landing...")
        if self.fcm:
            self.fcm.land()
    
    def _exit_landing(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_ready(self, context=None):
        self.logger.info("Ready to arm")
        # Reset state variables allowing clean restart
        self._status_flags = 0
        if self.backend:
            pass # handled by FCM/Backend naturally
    
    def _exit_ready(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_arming(self, context=None):
        self.logger.info("Starting arming sequence...")
        if self.fcm:
            self.fcm.arm()

    def _exit_arming(self):
        self.logger.debug("State action is intentionally empty.")

    def _enter_armed(self, context=None):
        self.logger.info("Drone is ARMED.")

    def _exit_armed(self):
        self.logger.debug("State action is intentionally empty.")

    def _enter_takeoff(self, context=None):
        self.logger.info(f"Taking off to {self.config['mission']['takeoff_altitude']}m...")
        if self.fcm:
            self.fcm.takeoff(self.config['mission']['takeoff_altitude'])
    
    def _exit_takeoff(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_formation(self, context=None):
        formation_type = self.config['mission']['formation']
        self.logger.info(f"Forming {formation_type} formation...")
        if self.fcm:
            self.fcm.set_desired_mode("FORMATION") # Or just let MovementController publish velocity
    
    def _exit_formation(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_mission(self, context=None):
        self.logger.info("Starting mission...")
        if self.fcm:
            self.fcm.set_desired_mode("MISSION")
    
    def _exit_mission(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_rtl(self, context=None):
        self.logger.info("Returning to launch...")
        if self.fcm:
            self.fcm.rtl()
    
    def _exit_rtl(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_emergency(self, context=None):
        self.logger.critical("EMERGENCY STATE ACTIVATED")
        if self.fcm:
            self.fcm.emergency_stop()
    
    def _exit_emergency(self):
        self.logger.debug("State action is intentionally empty.")
    
    def _enter_connected(self, context=None):
        self.logger.info("Hardware connected.")

    def _exit_connected(self):
        self.logger.debug("State action is intentionally empty.")

    def _enter_hover(self, context=None):
        self.logger.info("Entering HOVER state")
        if self.fcm:
            self.fcm.hover()
            
    def _exit_hover(self):
        self.logger.debug("State action is intentionally empty.")

    def _enter_disarm(self, context=None):
        self.logger.info("Disarming...")
        if self.fcm:
            self.fcm.disarm()

    def _exit_disarm(self):
        self.logger.debug("State action is intentionally empty.")

    def update_loop(self):
        """Loop to check conditions for automatic transitions."""
        if not hasattr(self, '_stop_event'):
            self._stop_event = threading.Event()
        self._stop_event.clear()
        
        while not self._stop_event.is_set():
            try:
                self.update()
                self._stop_event.wait(0.1)
            except Exception as e:
                self.logger.error(f"Error in state machine update loop: {e}")
                self._stop_event.wait(0.5)

    def stop(self):
        if hasattr(self, '_stop_event'):
            self._stop_event.set()
        self._running = False

    def update(self):
        """
        Called periodically to check for state-based transitions and safety rules.
        """
        with self._lock:
            current_state = self.state

        if current_state in [DroneState.BOOT, DroneState.CONNECTING]:
            return

        # Backend Health Check
        if self.backend and not self.backend.is_connected():
            if current_state not in [DroneState.EMERGENCY]:
                self.logger.error("Backend connection lost! Triggering EMERGENCY.")
                self.transition('emergency')
            else:
                self.logger.info("Attempting to reconnect backend...")
                if self.backend.reconnect():
                    self.logger.info("Backend reconnected successfully! Recovering from EMERGENCY...")
                    self.transition('recover')
            return

        # Auto-transition to READY from CONNECTED or DISARM
        if current_state in [DroneState.CONNECTED, DroneState.DISARM]:
            if self.telemetry:
                gps = self.telemetry.get_gps()
                batt = self.telemetry.get_battery()
                if gps and (gps[0] != 0.0 or gps[1] != 0.0) and batt and batt >= self.config.get('battery_emergency', 0.10):
                    self.logger.info("Telemetry healthy (GPS Lock + Battery OK). Auto-transitioning to READY.")
                    self.transition('ready')

        # Telemetry Safety Checks
        if self.telemetry:
            # Battery Failsafe
            batt = self.telemetry.get_battery()
            if batt is not None and batt < self.config.get('battery_emergency', 0.10):
                if current_state not in [DroneState.LANDING, DroneState.EMERGENCY]:
                    self.logger.warning("CRITICAL BATTERY! Triggering EMERGENCY (Land).")
                    self.transition('emergency')
                    return
            elif batt is not None and batt < self.config.get('battery_rtl', 0.20):
                if current_state in [DroneState.MISSION, DroneState.FORMATION]:
                    self.logger.warning("Low battery! Triggering RTL.")
                    self.transition('rtl_command')

            # GPS Failsafe
            gps = self.telemetry.get_gps()
            if gps is None or (gps[0] == 0.0 and gps[1] == 0.0):
                if current_state in [DroneState.MISSION, DroneState.FORMATION]:
                    self.logger.warning("GPS Signal Lost! Triggering EMERGENCY.")
                    self.transition('emergency')
                    return
            
            # Auto-transitions for Arming and Takeoff
            if current_state == DroneState.ARMING and self.backend.is_armed():
                self.logger.info("Drone is armed! Transitioning to TAKEOFF.")
                self.transition('armed')
                
            if current_state == DroneState.TAKEOFF and self.telemetry.is_in_air():
                alt = self.telemetry.get_local_position()[2]
                target_alt = -self.config['mission'].get('takeoff_altitude', 10.0)
                if alt <= target_alt * 0.9: # Z is negative
                    self.logger.info("Takeoff altitude reached! Transitioning to HOVER.")
                    self.transition('takeoff_complete')
                    
            # Auto-Landing detection
            if current_state in [DroneState.LANDING, DroneState.RTL] and not self.telemetry.is_in_air():
                self.logger.info("Drone has landed physically. Transitioning to READY.")
                self.transition('landed')

        # Update status flags for heartbeat
        flags = 0
        if self.backend:
            if self.backend.is_armed():
                flags |= 0x01  # STATUS_ARMED
            if self.backend.is_offboard_active():
                flags |= 0x04  # STATUS_OFFBOARD
            # Add other flags if needed
        self.set_status_flags(flags)

        # Neighbor Safety Checks (Communication timeout)
        if self.communication and current_state == DroneState.FORMATION:
            active_neighbors = self.communication.get_neighbor_list()
            # If we lost all neighbors during a formation, transition to a safe state
            if len(active_neighbors) == 0 and len(self.config.get('neighbor_list', [])) > 0:
                self.logger.warning("All neighbors lost! Recalculating or continuing mission autonomously.")
                # We can either stay in FORMATION and it becomes a 1-drone formation (handled by formation.py)
                # Or transition to mission. Handled by decision engine natively!

if __name__ == "__main__":
    # Simple test
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Test")
    
    config = {
        'drone_id': 1,
        'mission': {
            'takeoff_altitude': 10.0,
            'formation': 'V',
            'mission_id': 1
        }
    }
    
    sm = StateMachine(config, logger)
    sm.transition('system_ready')
    sm.transition('backend_connected')
    sm.transition('neighbors_found')
    sm.transition('mission_received')
    print(f"Current state: {sm.get_state()}")
    sm.transition('arm_command')
    print(f"Current state: {sm.get_state()}")
