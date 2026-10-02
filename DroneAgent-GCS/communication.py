#!/usr/bin/env python3
"""
Communication Module for DroneAgent

Handles UDP-based mesh communication between drones.
Responsibilities:
- Sending and receiving drone state packets
- Packet queuing and retry mechanism
- Neighbor discovery and maintenance
- Duplicate packet detection
- Sequence number validation
- Timestamp validation
"""

import socket
import threading
import queue
import time
import struct
from typing import Dict, Optional, Tuple, List
import logging
from packet import DronePacket, CommandPacket, CommandAckPacket, STATUS_ARMED, STATUS_GUIDED_MODE

class Communication:
    def __init__(self, config: dict, logger: logging.Logger):
        """
        Initialize communication module.
        
        Args:
            config: Configuration dictionary from config.yaml
            logger: Logger instance
        """
        self.config = config
        self.logger = logger.getChild("Communication")
        self.drone_id = config['drone_id']
        self.udp_port = config['udp_port']
        self.ip_address = config['ip_address']
        self.neighbor_list = config.get('neighbor_list', [])
        self.heartbeat_rate = config.get('heartbeat_rate', 10.0)  # Hz
        self.communication_timeout = config.get('communication_timeout', 1.0)  # seconds
        
        # Socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(1.0)  # Non-blocking with 1s timeout for recv
        
        # Queues
        queue_size = config.get('queue_size', 100)
        self.send_queue = queue.Queue(maxsize=queue_size)  # Packets to send
        self.recv_queue = queue.Queue(maxsize=queue_size)  # Received packets
        self.emergency_recv_queue = queue.Queue(maxsize=queue_size) # High priority commands
        
        # Neighbor table: drone_id -> {'last_packet': DronePacket, 'last_heard': timestamp}
        self.neighbors: Dict[int, dict] = {}
        # Initialize with known neighbors from config
        self.neighbors = {}  # ID -> {'last_packet': packet, 'last_heard': time}

        self.ack_cache = {}  # seq -> CommandAckPacket
        
        # Sequence tracking for duplicate detection
        self.seq_window_size = config.get('seq_window_size', 50)
        self.seen_cmd_seqs: Dict[int, set] = {}
        self.max_cmd_seq: Dict[int, int] = {}
        self.seen_packet_seqs: Dict[int, set] = {}
        self.max_packet_seq: Dict[int, int] = {}
        
        # Outgoing packet sequence number
        self.outgoing_packet_sequence = 0
        self.sequence_lock = threading.Lock()
        self._lock = threading.Lock()
        
        # Control flags
        self.running = False
        self.send_thread = None
        self.recv_thread = None
        
        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.packets_dropped = 0
        
        self.logger.info(f"Communication initialized on {self.ip_address}:{self.udp_port}")
    
    def start(self):
        """Start communication (set running flag)."""
        self.running = True
        self.logger.info("Communication started")
    
    def stop(self):
        """Stop communication (clear running flag and close socket)."""
        self.running = False
        self.sock.close()
        self.logger.info("Communication stopped")
    
    def transmit_loop(self):
        """Main send loop: processes send queue."""
        while self.running:
            try:
                # Wait for queued packets with timeout
                try:
                    packet = self.send_queue.get(timeout=1.0)
                    self._send_packet(packet)
                except queue.Empty:
                    continue
            except Exception as e:
                self.logger.error(f"Error in send loop: {e}")
    
    def receive_loop(self):
        """Main receive loop: listens for incoming packets and processes them."""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)  # Buffer size
                self._process_packet(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.logger.error(f"Error in receive loop: {e}")
                time.sleep(0.001)
    
    def _process_packet(self, data: bytes, addr: Tuple[str, int]):
        """
        Process an incoming packet.
        
        Args:
            data: Raw packet bytes
            addr: Tuple of (ip, port) of sender
        """
        if data.startswith(b'\xFF'):
            packet = CommandPacket.from_bytes(data)
            if packet is None:
                self.packets_dropped += 1
                self.logger.debug(f"Dropped invalid command packet from {addr}")
                return
                
            # Duplicate command detection
            with self._lock:
                if packet.source_id not in self.seen_cmd_seqs:
                    self.seen_cmd_seqs[packet.source_id] = set()
                    self.max_cmd_seq[packet.source_id] = -1
                    
                # Reject old packets
                if packet.sequence_number < self.max_cmd_seq[packet.source_id] - self.seq_window_size:
                    self.packets_dropped += 1
                    return
                # Reject duplicates
                if packet.sequence_number in self.seen_cmd_seqs[packet.source_id]:
                    self.packets_dropped += 1
                    # Resend ACK if available
                    if packet.sequence_number in self.ack_cache:
                        try:
                            self.send_queue.put_nowait(self.ack_cache[packet.sequence_number])
                        except queue.Full:
                            pass
                    return
                
                self.seen_cmd_seqs[packet.source_id].add(packet.sequence_number)
                if packet.sequence_number > self.max_cmd_seq[packet.source_id]:
                    self.max_cmd_seq[packet.source_id] = packet.sequence_number
                
                # Cleanup old seqs
                min_seq = self.max_cmd_seq[packet.source_id] - self.seq_window_size
                self.seen_cmd_seqs[packet.source_id] = {s for s in self.seen_cmd_seqs[packet.source_id] if s >= min_seq}

            # Check if command is for this drone or broadcast (target_id=0)
            if packet.target_id == 0 or packet.target_id == self.drone_id:
                target_q = self.emergency_recv_queue if packet.command in ('kill', 'emergency', 'rtl', 'land') else self.recv_queue
                try:
                    target_q.put_nowait((packet, addr))
                    self.packets_received += 1
                except queue.Full:
                    self.packets_dropped += 1
                    self.logger.warning(f"Receive queue full, dropping command packet {packet.command}")
            return

        # Check if it's an ACK
        if data.startswith(b'\xFF'): # Command packet
            packet = CommandPacket.from_bytes(data)
            if packet is None:
                self.packets_dropped += 1
                return
                
            # IGNORE LOOPBACK
            if packet.source_id == self.drone_id:
                return

            # Validate timestamp  
            # IGNORE LOOPBACK
            if getattr(packet, 'drone_id', -1) == self.drone_id:
                return
                
            try:
                self.recv_queue.put_nowait((packet, addr))
                self.packets_received += 1
            except queue.Full:
                self.packets_dropped += 1
            return

        packet = DronePacket.from_bytes(data)
        if packet is None:
            self.packets_dropped += 1
            self.logger.debug(f"Dropped invalid packet from {addr}")
            return
        
        # Validate timestamp
        if not packet.is_timestamp_valid(max_age_seconds=self.communication_timeout * 2):
            self.packets_dropped += 1
            self.logger.debug(f"Dropped stale packet from drone {packet.drone_id}")
            return
        
        # Check for duplicate using sliding window
        with self._lock:
            if packet.drone_id not in self.seen_packet_seqs:
                self.seen_packet_seqs[packet.drone_id] = set()
                self.max_packet_seq[packet.drone_id] = -1
                
            # Reject old packets
            if packet.packet_number < self.max_packet_seq[packet.drone_id] - self.seq_window_size:
                self.packets_dropped += 1
                return
            # Reject duplicates
            if packet.packet_number in self.seen_packet_seqs[packet.drone_id]:
                self.packets_dropped += 1
                self.logger.debug(f"Dropped duplicate packet from drone {packet.drone_id} (seq {packet.packet_number})")
                return
                
            self.seen_packet_seqs[packet.drone_id].add(packet.packet_number)
            if packet.packet_number > self.max_packet_seq[packet.drone_id]:
                self.max_packet_seq[packet.drone_id] = packet.packet_number
            
            # Cleanup old seqs
            min_seq = self.max_packet_seq[packet.drone_id] - self.seq_window_size
            self.seen_packet_seqs[packet.drone_id] = {s for s in self.seen_packet_seqs[packet.drone_id] if s >= min_seq}
        
            # Update neighbor info
            self.neighbors[packet.drone_id] = {
                'last_packet': packet,
                'last_heard': time.time()
            }
        
        # Telemetry is handled internally; do not push to recv_queue to avoid clogging
        self.packets_received += 1
    
    def _send_packet(self, packet):
        """
        Send a packet via UDP broadcast.
        
        Args:
            packet: DronePacket or CommandPacket to send
        """
        try:
            data = packet.to_bytes()
            # Send to all neighbors? For simplicity, we broadcast to the local network
            # In a real deployment, we might use multicast or unicast to known neighbors.
            # We'll use the broadcast address. Note: this requires the interface to support broadcast.
            self.sock.sendto(data, ('<broadcast>', self.udp_port))
            self.packets_sent += 1
            if isinstance(packet, DronePacket):
                self.logger.debug(f"Sent packet from drone {packet.drone_id} seq {packet.packet_number}")
            elif hasattr(packet, 'command'):
                self.logger.debug(f"Sent command {packet.command} to target {packet.target_id}")
        except Exception as e:
            self.logger.error(f"Failed to send packet: {e}")
    

    
    def send_packet(self, packet: DronePacket):
        """
        Public method to enqueue a packet for sending.

        Args:
            packet: DronePacket to send
        """
        try:
            if type(packet).__name__ == 'CommandAckPacket':
                with self._lock:
                    self.ack_cache[packet.sequence_number] = packet
                    if len(self.ack_cache) > 100:
                        oldest = min(self.ack_cache.keys())
                        del self.ack_cache[oldest]
            self.send_queue.put_nowait(packet)
        except queue.Full:
            self.logger.warning("Send queue full, dropping packet")

    def broadcast_command(self, cmd: str, args: dict = None, target_id: int = 0) -> int:
        """
        Send a CommandPacket over the network.
        
        Args:
            cmd: Command string (e.g., 'arm', 'takeoff')
            args: Optional command arguments
            target_id: Target drone ID, 0 for broadcast
            
        Returns:
            Sequence number of the sent command
        """
        with self.sequence_lock:
            self.outgoing_packet_sequence += 1
            seq = self.outgoing_packet_sequence
            
        packet = CommandPacket(
            target_id=target_id,
            command=cmd,
            args=args,
            source_id=self.drone_id,
            sequence_number=seq
        )
        try:
            self.send_queue.put_nowait(packet)
            return seq
        except queue.Full:
            self.logger.warning("Send queue full, dropping command packet")
            return -1
    
    def get_received_packet(self, timeout: float = 0.1) -> Optional[tuple]:
        """
        Get a received packet from the queue.
        
        Args:
            timeout: Maximum time to wait for a packet
            
        Returns:
            Tuple of (packet, addr) or None if timeout
        """
        try:
            return self.recv_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_neighbor_packet(self, drone_id: int) -> Optional[DronePacket]:
        """
        Get the most recent packet from a neighbor.
        
        Args:
            drone_id: ID of the neighbor drone
            
        Returns:
            DronePacket if available and not timed out, else None
        """
        with self._lock:
            neighbor = self.neighbors.get(drone_id)
            if not neighbor or neighbor['last_packet'] is None:
                return None
            
            # Check if packet is too old
            if time.time() - neighbor['last_heard'] > self.communication_timeout:
                return None
            
            return neighbor['last_packet']
    
    def get_neighbor_list(self) -> List[int]:
        """
        Get list of currently active neighbor IDs.
        
        Returns:
            List of drone IDs that have been heard from recently
        """
        active = []
        now = time.time()
        with self._lock:
            for drone_id, info in self.neighbors.items():
                if now - info['last_heard'] <= self.communication_timeout:
                    active.append(drone_id)
        return active
    
    def get_statistics(self) -> dict:
        """Get communication statistics."""
        return {
            'packets_sent': self.packets_sent,
            'packets_received': self.packets_received,
            'packets_dropped': self.packets_dropped,
            'neighbor_count': len(self.get_neighbor_list())
        }

if __name__ == "__main__":
    # Simple test
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Test")
    
    config = {
        'drone_id': 1,
        'udp_port': 14560,
        'ip_address': '0.0.0.0',
        'neighbor_list': [2, 3],
        'heartbeat_rate': 10.0,
        'communication_timeout': 1.0
    }
    
    comm = Communication(config, logger)
    comm.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        comm.stop()
