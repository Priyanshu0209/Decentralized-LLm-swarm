import pytest
import asyncio
import threading
import time
import socket
from unittest.mock import patch
from drone_main import DroneNode
import logging

def test_lifecycle_clean_shutdown():
    """Verify that starting and stopping DroneNode cleans up all threads and sockets."""
    logging.getLogger().setLevel(logging.CRITICAL) # Suppress spam
    
    # Capture initial threads
    initial_threads = {t.name for t in threading.enumerate()}
    
    with patch('simulation_backend.SimulationBackend.connect', return_value=True), \
         patch('drone_main.DroneNode._comprehensive_watchdog_loop'): # avoid watchdog restarting things during test
        
        node = DroneNode(["config/base.yaml", "config/simulation.yaml", "config/drone1.yaml"])

        # Start node
        node_thread = threading.Thread(target=node.start, daemon=True, name="MainNode")
        node_thread.start()
        
        # Wait for subsystems to boot
        time.sleep(2)
        
        # Verify threads are active
        active_threads = {t.name for t in threading.enumerate()}
        assert "CommTx" in active_threads or "CommRx" in active_threads
        assert "Heartbeat" in active_threads
        assert "Decision" in active_threads
        assert "MovementControl" in active_threads
        
        # Trigger shutdown
        node.shutdown()
        node_thread.join(timeout=5)
        
    # Wait a tiny bit for OS socket cleanup and thread joins
    time.sleep(1)
    
    final_threads = {t.name for t in threading.enumerate()}
    
    # Ensure all newly created threads (except maybe ThreadPoolWorkers) are dead
    for t in final_threads:
        if t not in initial_threads and not t.startswith("ThreadPool") and not t.startswith("asyncio"):
            # Core subsystems MUST die
            assert t not in ["CommTx", "CommRx", "Heartbeat", "Decision", "MovementControl", "Telemetry", "StateMachine", "MainNode"]
            
    # Sockets check - we can't easily introspect OS sockets in pure python without psutil, 
    # but we can verify the communication module socket is closed
    assert node.communication.sock.fileno() == -1 or getattr(node.communication.sock, '_closed', True)
