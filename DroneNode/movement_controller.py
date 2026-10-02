#!/usr/bin/env python3
"""
Movement Controller for DroneAgent

Handles sending movement commands to the MAVSDK controller based on
desired state from the decision engine/planner.
"""

import threading
import time
import math
import logging
from typing import Optional, Tuple
from mavsdk_backend_base import ControlPriority


class MovementController:
    def __init__(self, config: dict, logger: logging.Logger,
                 backend=None, fcm=None, decision_engine=None,
                 state_machine=None, **kwargs):
        """
        Initialize movement controller.

        Args:
            config: Configuration dictionary
            logger: Logger instance
            backend: BackendInterface instance
            decision_engine: DecisionEngine instance (for getting desired state)
            state_machine: StateMachine instance (for checking state)
            planner: Planner instance (for generating smooth trajectories)
            telemetry: Telemetry instance (for providing current state to planner)
        """
        self.config = config
        self.logger = logger.getChild("MovementController")
        self.backend = backend
        self.fcm = fcm
        self.decision_engine = decision_engine
        self.state_machine = state_machine
        self.planner = kwargs.get('planner')
        self.telemetry = kwargs.get('telemetry')

        self._stop_event = threading.Event()
        self._thread = None

        # Control parameters
        self.control_rate = config.get('control_rate', 50)  # Hz
        
        # Manual override state
        self._manual_velocity = None
        self._manual_velocity_time = 0.0

    def set_manual_velocity_override(self, vx: float, vy: float, vz: float, yaw: float):
        """Temporarily override decision engine with a manual velocity command."""
        self.logger.info(f"[DEBUG] Velocity setpoint stored: vx={vx}, vy={vy}, vz={vz}, yaw={yaw}")
        if vx == 0.0 and vy == 0.0 and vz == 0.0 and yaw == 0.0:
            self._manual_velocity = None
            pass
        else:
            self._manual_velocity = (vx, vy, vz, yaw)
        self._manual_velocity_time = time.time()

    def start(self):
        """Start the movement controller loop."""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._control_loop, name="MovementControl", daemon=True)
        self._thread.start()
        self.logger.info("Movement controller started")

    def stop(self):
        """Stop the movement controller loop."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.logger.info("Movement controller stopped")

    def _control_loop(self):
        """Main loop: get desired state and send to MAVSDK controller."""
        while not self._stop_event.is_set():
            start_time = time.time()
            try:
                # Only send commands if in OFFBOARD state and armed
                state_name = self.state_machine.get_state().name if self.state_machine else "UNKNOWN"
                self.logger.debug(f"[DEBUG] Current control mode: {state_name}")
                if (self.state_machine and
                    state_name in ["ARMED", "TAKEOFF", "HOVER", "OFFBOARD", "FORMATION", "MISSION"] and
                    self.backend and self.backend.is_armed()):
                    
                    # Flush manual velocity if we drop to HOVER and haven't pressed anything recently
                    # Actually, we flush manual velocity if transitioning from a state that clears it.
                    # For now, explicit stop/hover clears it.
                    
                    self.logger.debug("[DEBUG] Movement accepted")
                    # Get desired state from decision engine
                    pos_ned, vel_ned, yaw = None, None, None
                    if self.decision_engine:
                        pos_ned, vel_ned, yaw = self.decision_engine.get_desired_state()

                    if pos_ned is not None and vel_ned is not None:
                        # Convert yaw to degrees for offboard
                        yaw_deg = math.degrees(yaw)
                        cmd_vel_n, cmd_vel_e, cmd_vel_d = vel_ned[0], vel_ned[1], vel_ned[2]
                        
                        if self.planner and self.telemetry:
                            current_pos = self.telemetry.get_local_position()
                            current_vel = self.telemetry.get_velocity()
                            current_yaw = self.telemetry.get_heading()
                            
                            if current_pos and current_vel and current_yaw is not None:
                                self.planner.update_state(current_pos, current_vel, current_yaw)
                                self.planner.set_target(pos_ned, vel_ned, yaw)
                                dt = 1.0 / self.control_rate
                                _, cmd_vel, cmd_yaw = self.planner.get_control_command(dt)
                                
                                cmd_vel_n, cmd_vel_e, cmd_vel_d = cmd_vel[0], cmd_vel[1], cmd_vel[2]
                                yaw_deg = math.degrees(cmd_yaw)
                        
                        if self._manual_velocity and hasattr(self.backend, 'move_velocity_body'):
                            vx, vy, vz, manual_yaw_rate = self._manual_velocity
                            
                            if self.telemetry:
                                current_yaw = self.telemetry.get_heading() or 0.0
                                # Convert planner NED velocity to Body frame
                                cos_yaw = math.cos(current_yaw)
                                sin_yaw = math.sin(current_yaw)
                                
                                body_forward_planner = cmd_vel_n * cos_yaw + cmd_vel_e * sin_yaw
                                body_right_planner = -cmd_vel_n * sin_yaw + cmd_vel_e * cos_yaw
                                
                                # Combine planner body velocity with manual body velocity
                                total_forward = body_forward_planner + vx
                                total_right = body_right_planner + vy
                                total_down = cmd_vel_d + vz
                                
                                # P-controller for planner yaw to convert to yaw rate
                                planner_yaw_error = yaw - current_yaw
                                planner_yaw_error = (planner_yaw_error + math.pi) % (2 * math.pi) - math.pi
                                planner_yaw_rate = math.degrees(planner_yaw_error) * 2.0
                                
                                total_yaw_rate = planner_yaw_rate + manual_yaw_rate
                                
                                try:
                                    if self.fcm:
                                        self.fcm.set_desired_velocity_body(total_forward, total_right, total_down, total_yaw_rate)
                                except Exception as e:
                                    self.logger.error(f"Failed to send combined velocity: {e}")
                            else:
                                # Fallback if telemetry unavailable
                                try:
                                    if self.fcm:
                                        self.fcm.set_desired_velocity_body(vx, vy, vz, manual_yaw_rate)
                                except Exception as e:
                                    self.logger.error(f"Failed to send manual velocity: {e}")
                        else:
                            # No manual override, send pure planner NED velocity
                            try:
                                if self.fcm:
                                    self.fcm.set_desired_velocity(cmd_vel_n, cmd_vel_e, cmd_vel_d, yaw_deg)
                            except Exception as e:
                                self.logger.error(f"Failed to send planner velocity: {e}")
                    elif self._manual_velocity:
                        # Fallback for manual override without decision engine
                        vx, vy, vz, manual_yaw_rate = self._manual_velocity
                        try:
                            if self.fcm:
                                self.fcm.set_desired_velocity_body(vx, vy, vz, manual_yaw_rate)
                        except Exception as e:
                            self.logger.error(f"Failed to send manual velocity: {e}")
                else:
                    # Clear offboard flag if we are no longer in an offboard state
                    # Ensure manual velocity is flushed when the state machine switches away
                    if state_name not in ["OFFBOARD", "HOVER", "FORMATION", "MISSION"]:
                        self._manual_velocity = None
                        if hasattr(self, 'backend') and self.backend:
                            self.backend.release_control("MANUAL")
                    pass  # Intentionally empty block

                # Sleep to maintain rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.control_rate) - elapsed)
                self._stop_event.wait(sleep_time)
            except Exception as e:
                self.logger.error(f"Error in control loop: {e}")
                self._stop_event.wait(0.1)


if __name__ == "__main__":
    # Simple test (requires actual drone or SITL running)
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Test")

    config = {
        'control_rate': 50
    }

    controller = MovementController(config, logger)
    controller.start()
    try:
        time.sleep(5)
    finally:
        controller.stop()