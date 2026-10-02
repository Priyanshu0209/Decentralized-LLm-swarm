import sys
import os
import time
import pytest
import logging
from unittest.mock import MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from communication import Communication
from packet import DronePacket, CommandPacket

@pytest.fixture
def comm_config():
    return {
        'drone_id': 1,
        'udp_port': 14560,
        'ip_address': '0.0.0.0',
        'neighbor_list': [],
        'heartbeat_rate': 10.0,
        'communication_timeout': 1.0,
        'seq_window_size': 5
    }

@pytest.fixture
def logger():
    return logging.getLogger("TestComm")

def create_dummy_drone_packet(drone_id, packet_number):
    return DronePacket(
        drone_id=drone_id,
        timestamp=int(time.time() * 1_000_000),
        gps_lat=0.0, gps_lon=0.0, gps_alt=0.0,
        local_pos_x=0.0, local_pos_y=0.0, local_pos_z=0.0,
        velocity_x=0.0, velocity_y=0.0, velocity_z=0.0,
        heading=0.0, altitude=0.0, battery=1.0, health=1.0,
        mission_id=0, formation_index=0, status_flags=0,
        packet_number=packet_number
    )

def test_communication_initialization(comm_config, logger):
    comm = Communication(comm_config, logger)
    assert comm.drone_id == 1
    assert comm.seq_window_size == 5
    comm.stop()

def test_sliding_window_duplicate_drone_packet(comm_config, logger):
    comm = Communication(comm_config, logger)
    
    packet = create_dummy_drone_packet(drone_id=2, packet_number=10)
    data = packet.to_bytes()
    addr = ('127.0.0.1', 12345)
    
    # Process packet 10
    comm._process_packet(data, addr)
    assert comm.packets_received == 1
    assert comm.packets_dropped == 0
    
    # Process packet 10 again (duplicate)
    comm._process_packet(data, addr)
    assert comm.packets_received == 1
    assert comm.packets_dropped == 1
    
    # Process packet 8 (out of order, but within window of 5)
    packet_8 = create_dummy_drone_packet(drone_id=2, packet_number=8)
    comm._process_packet(packet_8.to_bytes(), addr)
    assert comm.packets_received == 2
    assert comm.packets_dropped == 1
    
    # Process packet 4 (too old, outside window 10 - 5 = 5)
    packet_4 = create_dummy_drone_packet(drone_id=2, packet_number=4)
    comm._process_packet(packet_4.to_bytes(), addr)
    assert comm.packets_received == 2
    assert comm.packets_dropped == 2
    
    comm.stop()

def test_sliding_window_command_packet(comm_config, logger):
    comm = Communication(comm_config, logger)
    addr = ('127.0.0.1', 12345)
    
    cmd_10 = CommandPacket(target_id=1, command='takeoff', args={'alt': 10}, source_id=0, sequence_number=10)
    comm._process_packet(cmd_10.to_bytes(), addr)
    assert comm.packets_received == 1
    
    # Duplicate
    comm._process_packet(cmd_10.to_bytes(), addr)
    assert comm.packets_received == 1
    assert comm.packets_dropped == 1
    
    # Old packet outside window (seq < 10 - 5)
    cmd_4 = CommandPacket(target_id=1, command='land', args={}, source_id=0, sequence_number=4)
    comm._process_packet(cmd_4.to_bytes(), addr)
    assert comm.packets_received == 1
    assert comm.packets_dropped == 2
    
    # Reordered packet inside window
    cmd_8 = CommandPacket(target_id=1, command='pause', args={}, source_id=0, sequence_number=8)
    comm._process_packet(cmd_8.to_bytes(), addr)
    assert comm.packets_received == 2
    
    comm.stop()

def test_emergency_queue_routing(comm_config, logger):
    comm = Communication(comm_config, logger)
    addr = ('127.0.0.1', 12345)
    
    # Normal command goes to recv_queue
    cmd_takeoff = CommandPacket(target_id=1, command='takeoff', args={'alt': 10}, source_id=0, sequence_number=1)
    comm._process_packet(cmd_takeoff.to_bytes(), addr)
    assert not comm.recv_queue.empty()
    assert comm.emergency_recv_queue.empty()
    
    # Emergency command goes to emergency_recv_queue
    cmd_kill = CommandPacket(target_id=1, command='kill', args={}, source_id=0, sequence_number=2)
    comm._process_packet(cmd_kill.to_bytes(), addr)
    assert not comm.emergency_recv_queue.empty()
    
    comm.stop()
