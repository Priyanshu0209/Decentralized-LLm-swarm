#!/usr/bin/env python3
"""
DroneNode Entry Point

Runs on each Raspberry Pi.
- Connects to Pixhawk via MAVSDK
- Runs Decision Engine, Movement Controller, Formation, Collision Avoidance
- Listens for GCS CommandPackets over UDP
- Broadcasts telemetry/heartbeat to mesh
- Fully autonomous: continues mission if GCS disconnects
- No GUI, no map, no browser
"""

import threading
import signal
import sys
import os
import time
from typing import Dict, Any
import argparse

# Ensure the DroneNode directory is on the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import load_config, ConfigurationError
from backend_factory import BackendFactory
from communication import Communication
from heartbeat import Heartbeat
from telemetry import Telemetry
from decision import DecisionEngine
from planner import Planner
from movement_controller import MovementController
from mission import MissionManager
from logger import DroneLogger
from state_machine import StateMachine
from flight_control_manager import FlightControlManager


class DroneNode:
    """
    Autonomous drone controller for Raspberry Pi.
    
    Manages all flight subsystems headlessly.
    Receives commands from GCS via CommandPackets.
    Continues mission autonomously if GCS disconnects.
    """

    def __init__(self, config_paths: list):
        self.config = self._load_config(config_paths)
        self.logger = DroneLogger(self.config.get('drone_id', 1), config=self.config).get_logger()

        # Initialize subsystems
        self.backend = BackendFactory(self.config, self.logger)
        self.fcm = FlightControlManager(self.config, self.logger, backend=self.backend)
        self.communication = Communication(self.config, self.logger)
        self.telemetry = Telemetry(self.config, self.logger, backend=self.backend)
        self.backend.set_telemetry(self.telemetry)
        self.state_machine = StateMachine(
            self.config, self.logger,
            backend=self.backend,
            fcm=self.fcm,
            telemetry=self.telemetry,
            communication=self.communication
        )
        self.mission_manager = MissionManager(self.config, self.logger)
        self.decision_engine = DecisionEngine(self.config, self.logger, state_machine=self.state_machine, telemetry=self.telemetry, communication=self.communication)
        self.planner = Planner(self.config, self.logger)
        self.movement_controller = MovementController(
            self.config, self.logger,
            backend=self.backend,
            fcm=self.fcm,
            decision_engine=self.decision_engine,
            state_machine=self.state_machine,
            planner=self.planner,
            telemetry=self.telemetry
        )
        self.heartbeat = Heartbeat(
            self.config, self.logger,
            communication=self.communication,
            telemetry=self.telemetry,
            state_machine=self.state_machine
        )

        # Thread management
        self.threads: Dict[str, threading.Thread] = {}
        self.shutdown_event = threading.Event()

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.logger.info("DroneNode initialized")

    def _load_config(self, config_paths: list) -> Dict[str, Any]:
        """Load configuration from YAML files."""
        try:
            config = load_config(config_paths)
            config['heartbeat_rate'] = config.get('heartbeat_rate', 10)
            config['communication_timeout'] = config.get('communication_timeout', 1.0)
            config['safe_distance'] = config.get('safe_distance', 5.0)
            return config
        except ConfigurationError as e:
            print(f"ConfigurationError: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Failed to load config: {e}")
            sys.exit(1)

    def start(self):
        """Start all subsystems."""
        self.logger.info("Starting DroneNode...")

        # Start Backend first and ensure it succeeds
        if not self.backend.connect():
            self.logger.error("Backend connection failed. Exiting.")
            sys.exit(1)

        # Start Flight Control Manager
        self.fcm.start()

        # Start telemetry
        self.telemetry.start()

        # Start Mesh Communication
        self.communication.start()
        threading.Thread(target=self.communication.transmit_loop, daemon=True, name="CommTx").start()
        threading.Thread(target=self.communication.receive_loop, daemon=True, name="CommRx").start()

        # Start Mission
        if hasattr(self, 'mission_manager') and hasattr(self.mission_manager, 'start'):
            self.mission_manager.start()

        # Start Planner (if applicable)
        if hasattr(self, 'planner') and hasattr(self.planner, 'start'):
            self.planner.start()

        # Start Decision Engine
        self.decision_engine.start()

        # Start Formation (handled by movement controller/decision)
        if hasattr(self, 'formation') and hasattr(self.formation, 'start'):
            self.formation.start()

        # Start Collision Avoidance
        if hasattr(self, 'collision_avoidance') and hasattr(self.collision_avoidance, 'start'):
            self.collision_avoidance.start()

        # Start State Machine, Movement, Heartbeat
        self.state_machine_thread = threading.Thread(target=self.state_machine.update_loop, daemon=True, name="StateMachine")
        self.state_machine_thread.start()
        
        # Initialize state machine startup sequence
        self.state_machine.transition('system_ready')
        self.state_machine.transition('backend_connected')

        self.movement_controller.start()
        self.heartbeat.start()

        self.logger.info("All subsystems started")

        # Start command processor loop for GCS commands
        threading.Thread(target=self.command_processor_loop, daemon=True, name="CmdProcessor").start()
        
        # Start comprehensive watchdog
        threading.Thread(target=self._comprehensive_watchdog_loop, daemon=True, name="Watchdog").start()

        # DroneNode runs headlessly
        self.logger.info("DroneNode running headlessly. Waiting for commands...")
        try:
            while not self.shutdown_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        finally:
            self.shutdown()

    def _comprehensive_watchdog_loop(self):
        """Monitor MAVSDK connection and all critical subsystem threads."""
        consecutive_failures = 0
        max_retries = 10
        base_delay = 2.0
        max_delay = 30.0
        import random
        
        while not self.shutdown_event.is_set():
            # 1. Check Backend Connection
            if not self.backend.is_connected():
                consecutive_failures += 1
                delay = min(base_delay * (1.5 ** (consecutive_failures - 1)), max_delay)
                total_delay = delay + random.uniform(0, 1.0)
                
                self.logger.warning(f"Watchdog: MAVSDK disconnected. Reconnect attempt {consecutive_failures}/{max_retries} in {total_delay:.1f}s...")
                time.sleep(total_delay)
                
                self.telemetry.stop()
                if self.backend.reconnect():
                    self.logger.info("Watchdog reconnect successful. Restarting telemetry...")
                    consecutive_failures = 0
                    self.telemetry.start()
                    self.telemetry._reconnecting = False 
                else:
                    self.logger.error("Watchdog reconnect failed.")
                    if consecutive_failures >= max_retries:
                        self.logger.critical("Max reconnect attempts reached. Continuing to retry...")
                        # Do NOT shut down DroneNode, keep retrying to allow self-healing
                        consecutive_failures = max_retries - 1
            else:
                consecutive_failures = 0

            # 2. Check Critical Threads
            # Heartbeat
            if hasattr(self, 'heartbeat') and self.heartbeat._thread and not self.heartbeat._thread.is_alive():
                self.logger.error("Watchdog: Heartbeat thread died! Restarting...")
                self.heartbeat.stop()
                self.heartbeat.start()

            # Decision Engine
            if hasattr(self, 'decision_engine') and self.decision_engine._thread and not self.decision_engine._thread.is_alive():
                self.logger.error("Watchdog: DecisionEngine thread died! Restarting...")
                self.decision_engine.stop()
                self.decision_engine.start()

            # Movement Controller
            if hasattr(self, 'movement_controller') and self.movement_controller._thread and not self.movement_controller._thread.is_alive():
                self.logger.error("Watchdog: MovementController thread died! Restarting...")
                self.movement_controller.stop()
                self.movement_controller.start()

            # Telemetry
            if hasattr(self, 'telemetry') and self.telemetry._thread and not self.telemetry._thread.is_alive():
                # Don't restart if intentionally stopped for reconnect
                if getattr(self.telemetry, '_running', False):
                    self.logger.error("Watchdog: Telemetry thread died! Restarting...")
                    self.telemetry.stop()
                    self.telemetry.start()

            # Note: StateMachine runs natively via threading.Thread(target=self.state_machine.update_loop).
            # I will ensure state machine thread is tracked.
            if hasattr(self, 'state_machine_thread') and not self.state_machine_thread.is_alive():
                self.logger.error("Watchdog: StateMachine thread died! Restarting...")
                self.state_machine.stop()
                self.state_machine_thread = threading.Thread(target=self.state_machine.update_loop, daemon=True, name="StateMachine")
                self.state_machine_thread.start()

            time.sleep(2.0)

    def command_processor_loop(self):
        """Listen to recv_queue for CommandPackets from GCS."""
        from packet import CommandPacket
        import queue
        while not self.shutdown_event.is_set():
            packet = None
            # 1. Check emergency queue first (non-blocking)
            try:
                packet_tuple = self.communication.emergency_recv_queue.get_nowait()
                if packet_tuple:
                    packet, addr = packet_tuple
                    self.logger.critical(f"PRIORITY EMERGENCY COMMAND RECEIVED: {packet.command}")
            except queue.Empty:
                pass
            
            # 2. Check standard queue if no emergency
            if not packet:
                packet_tuple = self.communication.get_received_packet(timeout=0.5)
                if packet_tuple:
                    packet, addr = packet_tuple
                    
            if packet and isinstance(packet, CommandPacket):
                # Process in a new thread to avoid blocking the network queue
                threading.Thread(target=self.execute_command, args=(packet,), daemon=True).start()
                    
    def execute_command(self, packet):
        """Execute a received GCS command with 4-stage ACK pipeline."""
        from packet import CommandAckPacket
        self.logger.info(f"Executing GCS command: {packet.command} with args: {packet.args}")
        
        start_time = time.time()
        
        def send_ack(status: str, error_code: int = 0):
            if packet.requires_ack:
                exec_time_ms = (time.time() - start_time) * 1000.0
                ack = CommandAckPacket(
                    sequence_number=packet.sequence_number,
                    status=status,
                    execution_time_ms=exec_time_ms,
                    error_code=error_code,
                    drone_id=self.config.get('drone_id', 1)
                )
                self.communication.send_packet(ack)
                self.logger.debug(f"Sent ACK for seq {packet.sequence_number}: status={status}")

        try:
            cmd = packet.command
            args = packet.args
            
            # STAGE 1: ACCEPTED
            send_ack('accepted')
            
            if cmd == 'arm':
                # Real drone safety checks
                if not self.backend.is_connected():
                    self.logger.error("Cannot arm: Backend disconnected")
                    send_ack('failed', error_code=3)
                    return
                    
                gps = self.telemetry.get_gps()
                if not gps or (gps[0] == 0.0 and gps[1] == 0.0):
                    self.logger.error("Cannot arm: No GPS lock")
                    send_ack('failed', error_code=4)
                    return
                    
                batt = self.telemetry.get_battery()
                if batt and batt < self.config.get('battery_emergency', 0.10):
                    self.logger.error("Cannot arm: Battery critical")
                    send_ack('failed', error_code=5)
                    return

                send_ack('executing')
                if not self.state_machine.transition('arm_command'):
                    self.logger.error(f"Cannot arm: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                
                # Wait for completion
                timeout = 10.0
                start_wait = time.time()
                while time.time() - start_wait < timeout:
                    if self.backend.is_armed() or self.state_machine.get_state().name == "TAKEOFF":
                        send_ack('completed')
                        return
                    time.sleep(0.1)
                send_ack('failed', error_code=6) # Timeout

            elif cmd == 'disarm':
                send_ack('executing')
                if not self.state_machine.transition('disarm_command'):
                    self.logger.error(f"Cannot disarm: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'takeoff':
                if not self.backend.is_armed() and self.state_machine.get_state().name != "READY":
                    self.logger.error("Cannot takeoff: Drone not ready or armed")
                    send_ack('failed', error_code=7)
                    return
                    
                if args and 'alt' in args:
                    self.config['mission']['takeoff_altitude'] = args.get('alt', 10.0)
                    
                send_ack('executing')
                if self.state_machine.get_state().name == "READY":
                    if not self.state_machine.transition('arm_command'):
                        self.logger.error("Cannot takeoff: arm_command transition rejected")
                        send_ack('failed', error_code=8)
                        return
                    
                    # Wait for arming to complete
                    timeout = 10.0
                    start_wait = time.time()
                    while time.time() - start_wait < timeout:
                        if self.backend.is_armed():
                            break
                        time.sleep(0.1)
                        
                    if not self.backend.is_armed():
                        self.logger.error("Takeoff failed: Could not auto-arm")
                        send_ack('failed', error_code=6)
                        return
                        
                if not self.state_machine.transition('takeoff_command'):
                    self.logger.error(f"Cannot takeoff: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                
                send_ack('completed')
                
            elif cmd == 'land':
                send_ack('executing')
                if not self.state_machine.transition('land_command'):
                    self.logger.error(f"Cannot land: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'rtl':
                send_ack('executing')
                if not self.state_machine.transition('rtl_command'):
                    self.logger.error(f"Cannot rtl: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'pause' or cmd == 'mission_pause':
                send_ack('executing')
                if not self.state_machine.transition('pause_command'):
                    self.logger.error(f"Cannot pause: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'resume' or cmd == 'mission_resume':
                send_ack('executing')
                if not self.state_machine.transition('mission_start'):
                    self.logger.error(f"Cannot resume: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'start_mission' or cmd == 'mission_start':
                send_ack('executing')
                if not self.state_machine.transition('mission_start'):
                    self.logger.error(f"Cannot start mission: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'kill' or cmd == 'emergency':
                send_ack('executing')
                if not self.state_machine.transition('emergency'):
                    self.logger.error(f"Cannot emergency stop: State machine rejected transition from {self.state_machine.get_state().name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('completed')
                
            elif cmd == 'transition':
                send_ack('executing')
                if self.state_machine.transition(args.get('state')):
                    send_ack('completed')
                else:
                    send_ack('failed', error_code=8)
                    
            elif cmd == 'upload_mission':
                send_ack('executing')
                if hasattr(self, 'decision_engine'):
                    waypoints = args.get('waypoints', [])
                    self.decision_engine.mission_manager.upload_mission(waypoints)
                send_ack('completed')
                
            elif cmd == 'cancel_mission':
                send_ack('executing')
                if hasattr(self, 'decision_engine'):
                    self.decision_engine.mission_manager.cancel_mission()
                # Pause mission (drop to hover)
                self.state_machine.transition('pause_command')
                send_ack('completed')
                
            elif cmd == 'formation_change':
                state_name = self.state_machine.get_state().name
                if state_name not in ["HOVER", "FORMATION", "MISSION"]:
                    self.logger.error(f"Cannot change formation in state {state_name}")
                    send_ack('failed', error_code=8)
                    return
                send_ack('executing')
                if hasattr(self, 'decision_engine') and hasattr(self.decision_engine, 'formation'):
                    self.decision_engine.formation.update_parameters({
                        'type': args.get('shape', 'v'),
                        'spacing': args.get('spacing', 5.0),
                        'heading': args.get('heading', 0.0),
                        'rotation': args.get('rotation', 0.0),
                        'altitude_offset': args.get('alt_offset', 0.0)
                    })
                    self.state_machine.transition('formation_ready')
                send_ack('completed')
                
            elif cmd == 'parameter_update':
                send_ack('executing')
                for key, value in args.items():
                    self.config[key] = value
                if hasattr(self, 'decision_engine'):
                    self.decision_engine.update_parameters(args)
                if hasattr(self, 'collision_avoidance'):
                    self.collision_avoidance.update_parameters(args)
                if hasattr(self, 'heartbeat'):
                    self.heartbeat.update_parameters(args)
                self.logger.info(f"Parameters updated: {args}")
                send_ack('completed')
                
            elif cmd == 'velocity_ned':
                # Validate state first
                state_name = self.state_machine.get_state().name
                if state_name not in ["ARMED", "TAKEOFF", "HOVER", "OFFBOARD", "FORMATION", "MISSION"]:
                    self.logger.warning(f"Ignored movement command in invalid state: {state_name}")
                    send_ack('failed', error_code=8)
                    return
                    
                # Continuous command, no 'executing' state needed, just completed
                if hasattr(self, 'movement_controller') and hasattr(self.movement_controller, 'set_manual_velocity_override'):
                    self.movement_controller.set_manual_velocity_override(
                        args.get('vx', 0), args.get('vy', 0), args.get('vz', 0), args.get('yaw', 0)
                    )
                send_ack('completed')
                
            elif cmd == 'set_offboard':
                send_ack('executing')
                if self.state_machine.get_state().name == "TAKEOFF":
                    if not self.state_machine.transition('takeoff_complete'):
                        self.logger.error(f"Cannot set offboard: takeoff_complete transition rejected")
                        send_ack('failed', error_code=8)
                        return
                send_ack('completed')
                
            else:
                self.logger.warning(f"Unknown command received: {cmd}")
                send_ack('failed', error_code=1)
                
        except Exception as e:
            self.logger.error(f"Error executing command {packet.command}: {e}")
            send_ack('failed', error_code=2)

    def signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        self.logger.info(f"Received signal {signum}, initiating shutdown...")
        self.shutdown()

    def shutdown(self):
        """Gracefully shut down all subsystems."""
        self.logger.info("Shutting down DroneNode...")
        self.shutdown_event.set()

        self.heartbeat.stop()
        self.movement_controller.stop()
        self.decision_engine.stop()
        self.state_machine.stop()
        self.telemetry.stop()
        self.communication.stop()
        self.fcm.stop()
        self.backend.stop()

        self.logger.info("DroneNode shutdown complete")


def main():
    parser = argparse.ArgumentParser(description="DroneNode - Autonomous Drone Controller")
    parser.add_argument("-c", "--config", nargs="+", help="Path to config YAML files")
    args = parser.parse_args()

    configs = args.config if args.config else ["config/drone1.yaml"]

    # Ensure strictly ordered configuration loading
    final_configs = ["config/base.yaml"]
    
    # Mode
    if any("real.yaml" in c for c in configs):
        final_configs.append("config/real.yaml")
    else:
        final_configs.append("config/simulation.yaml")
        
    # Transport
    if any("airsim.yaml" in c for c in configs):
        final_configs.append("config/airsim.yaml")
        
    # Network
    final_configs.append("config/network_config.yaml")
    
    # Specific drone instances or arbitrary overrides
    for c in configs:
        if c not in final_configs:
            final_configs.append(c)
            
    drone = DroneNode(final_configs)
    drone.start()


if __name__ == "__main__":
    main()
