import sys
import time
import signal
import threading
from concurrent.futures import Future
import logging
from typing import Dict, Any

from config import load_config, ConfigurationError
from communication import Communication
from logger import DroneLogger

from dataclasses import dataclass

class GCSFuture(Future):
    """A future that resolves when a CommandAckPacket is received."""
    pass

@dataclass
class PendingCommandInfo:
    future: GCSFuture
    packet: Any  # CommandPacket
    creation_time: float
    last_sent_time: float
    retry_count: int
    max_retries: int
    timeout: float
    ack_received: bool = False

class GCSBackend:
    def __init__(self, agent):
        self.agent = agent
        
    def connect(self):
        self.agent.logger.info("GCSBackend connect handled natively")
        
    def stop(self):
        self.agent.logger.info("GCSBackend stop handled natively")
        
    def arm(self):
        return self.agent.send_command_and_wait('arm', target_id=self.agent.target_id)
        
    def disarm(self):
        return self.agent.send_command_and_wait('disarm', target_id=self.agent.target_id)
        
    def takeoff(self, alt):
        return self.agent.send_command_and_wait('takeoff', {'alt': alt}, target_id=self.agent.target_id)
        
    def land(self):
        return self.agent.send_command_and_wait('land', target_id=self.agent.target_id)
        
    def return_to_launch(self):
        return self.agent.send_command_and_wait('rtl', target_id=self.agent.target_id)
        
    def pause(self):
        return self.agent.send_command_and_wait('pause', target_id=self.agent.target_id)
        
    def resume(self):
        return self.agent.send_command_and_wait('resume', target_id=self.agent.target_id)
        
    def emergency_stop(self):
        return self.agent.send_command_and_wait('kill', target_id=self.agent.target_id)
        
    def set_velocity_ned(self, vx, vy, vz, yaw=0):
        # We don't typically wait for ACKs for continuous movement commands to avoid blocking
        self.agent.communication.broadcast_command('velocity_ned', {'vx': vx, 'vy': vy, 'vz': vz, 'yaw': yaw}, target_id=self.agent.target_id)
        return GCSFuture()
        
    def set_offboard_mode(self):
        return self.agent.send_command_and_wait('set_offboard', target_id=self.agent.target_id)

class GCSMissionManager:
    def __init__(self, agent):
        self.agent = agent
        
    def upload_mission(self, wps):
        self.agent.communication.broadcast_command('upload_mission', {'waypoints': wps}, target_id=getattr(self.agent, 'target_id', 0))
        
    def cancel_mission(self):
        self.agent.communication.broadcast_command('cancel_mission', target_id=getattr(self.agent, 'target_id', 0))

class GCSStateMachine:
    def __init__(self, agent):
        self.agent = agent
        
    def transition(self, state):
        self.agent.communication.broadcast_command('transition', {'state': state}, target_id=getattr(self.agent, 'target_id', 0))

class GCSAgent:
    def __init__(self, config_paths: list):
        self.config = self._load_config(config_paths)
        self.config['drone_id'] = 0  # GCS acts as ID 0
        self.logger = DroneLogger(self.config['drone_id'], config=self.config).get_logger()
        
        # Core communication
        self.communication = Communication(self.config, self.logger)
        
        # Futures for command ACKs
        self.pending_commands: Dict[int, PendingCommandInfo] = {}
        self.pending_commands_lock = threading.Lock()
        
        # Interface proxies for GUI compatibility
        self.backend = GCSBackend(self)
        self.mission_manager = GCSMissionManager(self)
        self.state_machine = GCSStateMachine(self)
        
        self.shutdown_event = threading.Event()
        self.target_id = 0
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def send_command_and_wait(self, cmd: str, args: dict = None, target_id: int = 0) -> GCSFuture:
        """Sends a command and returns a Future that resolves when ACKed."""
        from packet import CommandPacket
        with self.communication.sequence_lock:
            self.communication.outgoing_packet_sequence += 1
            seq = self.communication.outgoing_packet_sequence
            
        packet = CommandPacket(
            target_id=target_id,
            command=cmd,
            args=args,
            source_id=self.config['drone_id'],
            sequence_number=seq
        )
        
        fut = GCSFuture()
        try:
            self.communication.send_queue.put_nowait(packet)
            
            # Emergency commands don't retry to avoid spamming the priority queue, but we track them for cleanup
            max_retries = 0 if cmd in ('kill', 'emergency', 'rtl', 'land') else self.config.get('command_max_retries', 3)
            timeout = self.config.get('command_timeout', 3.0)
            
            with self.pending_commands_lock:
                self.pending_commands[seq] = PendingCommandInfo(
                    future=fut,
                    packet=packet,
                    creation_time=time.time(),
                    last_sent_time=time.time(),
                    retry_count=0,
                    max_retries=max_retries,
                    timeout=timeout,
                    ack_received=False
                )
        except Exception as e:
            fut.set_exception(RuntimeError(f"Failed to enqueue command: {e}"))
        return fut
        
    def _process_incoming(self):
        """Thread to process incoming telemetry and ACKs."""
        from packet import CommandAckPacket
        import queue
        while not self.shutdown_event.is_set():
            # Drain emergency queue to prevent memory leak
            try:
                while True:
                    self.communication.emergency_recv_queue.get_nowait()
            except queue.Empty:
                pass

            packet_tuple = self.communication.get_received_packet(timeout=0.1)
            if packet_tuple:
                packet, addr = packet_tuple
                if isinstance(packet, CommandAckPacket):
                    self.logger.info(f"Received ACK for seq {packet.sequence_number}: status={packet.status}")
                    with self.pending_commands_lock:
                        if packet.sequence_number in self.pending_commands:
                            info = self.pending_commands[packet.sequence_number]
                            if packet.status == 'completed':
                                self.pending_commands.pop(packet.sequence_number)
                                info.future.set_result(True)
                            elif packet.status == 'failed':
                                self.pending_commands.pop(packet.sequence_number)
                                info.future.set_exception(RuntimeError(f"Command failed: {packet.error_code}"))
                            else:
                                # Status is 'accepted' or 'executing'
                                # Cancel retry immediately as instructed
                                info.ack_received = True
                                info.last_sent_time = time.time()
                                info.creation_time = time.time()

    def _retry_watchdog_loop(self):
        """Thread to handle ACK timeouts and retransmissions with exponential backoff and jitter."""
        import random
        while not self.shutdown_event.is_set():
            now = time.time()
            to_remove = []
            
            with self.pending_commands_lock:
                for seq, info in self.pending_commands.items():
                    # Calculate current dynamic timeout based on retry count (exponential backoff)
                    current_timeout = info.timeout * (1.5 ** info.retry_count) + random.uniform(0.0, 0.5)
                    
                    # Absolute max time is arbitrary based on max_retries, let's say sum of all potential timeouts
                    # Actually, simple absolute check: if it's been around longer than max possible wait, kill it.
                    max_total_wait = sum([info.timeout * (1.5 ** i) for i in range(info.max_retries + 1)]) + (info.max_retries * 0.5)
                    if now - info.creation_time > max_total_wait:
                        self.logger.warning(f"Command seq {seq} ({info.packet.command}) completely timed out.")
                        info.future.set_exception(TimeoutError("Command ACK timed out"))
                        to_remove.append(seq)
                        continue
                        
                    # Check if we should retry
                    if now - info.last_sent_time > current_timeout:
                        if info.ack_received:
                            # Just wait for completion, no more retries
                            pass
                        elif info.retry_count < info.max_retries:
                            info.retry_count += 1
                            info.last_sent_time = now
                            self.logger.info(f"Retrying command seq {seq} ({info.packet.command}), attempt {info.retry_count} (next timeout: {current_timeout:.2f}s)")
                            try:
                                self.communication.send_queue.put_nowait(info.packet)
                            except Exception as e:
                                self.logger.exception(f"Failed to retry command {seq}: {e}")
                        else:
                            # Reached max retries
                            self.logger.warning(f"Command seq {seq} reached max retries.")
                            info.future.set_exception(TimeoutError("Max retries exceeded"))
                            to_remove.append(seq)

                for seq in to_remove:
                    self.pending_commands.pop(seq, None)
                    
            time.sleep(0.5)
        
    def _load_config(self, config_paths: list) -> Dict[str, Any]:
        try:
            return load_config(config_paths, validate=False)
        except Exception as e:
            print(f"Failed to load config: {e}")
            sys.exit(1)
            
    def start(self):
        self.logger.info("Starting GCSAgent...")
        self.communication.start()
        threading.Thread(target=self.communication.transmit_loop, daemon=True, name="CommTx").start()
        threading.Thread(target=self.communication.receive_loop, daemon=True, name="CommRx").start()
        threading.Thread(target=self._process_incoming, daemon=True, name="GCSProcess").start()
        threading.Thread(target=self._retry_watchdog_loop, daemon=True, name="GCSRetryWatchdog").start()
        
        # Start GUI
        if self.config.get('run_gui', True):
            from PySide6.QtWidgets import QApplication
            from gui.main_window import MainWindow
            
            app = QApplication.instance()
            if not app:
                app = QApplication(sys.argv)
                
            try:
                import os
                style_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gui", "styles.qss")
                with open(style_path, "r") as f:
                    app.setStyleSheet(f.read())
            except FileNotFoundError:
                self.logger.warning("GUI style sheet not found.")
                
            main_window = MainWindow(self)
            main_window.show()
            
            self.logger.info("Starting PySide6 GUI Loop")
            app.exec()
            if not self.shutdown_event.is_set():
                self.shutdown()
        else:
            try:
                while not self.shutdown_event.is_set():
                    time.sleep(0.1)
            except KeyboardInterrupt:
                self.logger.info("Keyboard interrupt received")
            finally:
                self.shutdown()
                
    def signal_handler(self, signum, frame):
        self.shutdown()
        
    def shutdown(self):
        self.shutdown_event.set()
        self.communication.stop()
        self.logger.info("GCSAgent shutdown complete")
        sys.exit(0)
