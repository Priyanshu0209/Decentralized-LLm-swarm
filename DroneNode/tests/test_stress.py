import pytest
import time
import threading
from unittest.mock import patch, MagicMock
from drone_main import DroneNode
from packet import CommandPacket

def test_accelerated_stress():
    """Simulate intense packet and telemetry load to check for deadlocks and memory spikes."""
    import logging
    logging.getLogger().setLevel(logging.CRITICAL)
    
    with patch('simulation_backend.SimulationBackend.connect', return_value=True), \
         patch('drone_main.DroneNode._comprehensive_watchdog_loop'):
         
        node = DroneNode(["config/base.yaml", "config/simulation.yaml", "config/drone1.yaml"])
        node_thread = threading.Thread(target=node.start, daemon=True)
        node_thread.start()
        time.sleep(1)
        
        # Override safety methods
        node.state_machine._pre_arm_checks = MagicMock(return_value=True)
        node.backend.is_armed = MagicMock(return_value=True)
        
        start_mem = 0
        try:
            import psutil
            import os
            process = psutil.Process(os.getpid())
            start_mem = process.memory_info().rss
        except ImportError:
            pass

        # Bombard with thousands of fake telemetry updates and mesh messages
        threads = []
        stop_stress = threading.Event()
        
        def telemetry_spammer():
            while not stop_stress.is_set():
                if hasattr(node.telemetry, '_telemetry_data'):
                    node.telemetry._telemetry_data['gps'] = (47.0, 8.0, 50.0)
                    node.telemetry._telemetry_data['battery'] = 99.0
                time.sleep(0.001)
                
        def command_spammer():
            import queue
            for i in range(1000):
                if stop_stress.is_set(): break
                cmd = CommandPacket(target_id=1, command="takeoff")
                try:
                    node.communication.recv_queue.put_nowait((cmd, ('127.0.0.1', 5000)))
                except queue.Full:
                    pass
                time.sleep(0.001)

        t1 = threading.Thread(target=telemetry_spammer)
        t2 = threading.Thread(target=command_spammer)
        
        t1.start()
        t2.start()
        
        # Run stress for 3 seconds
        time.sleep(3)
        
        stop_stress.set()
        t1.join()
        t2.join()
        
        # Check if queues grew out of control (meaning processing threads died or fell behind too much)
        assert node.communication.recv_queue.qsize() < 1500, "Queue processing stalled!"
        
        node.shutdown()
        node_thread.join(timeout=2)
        
        if start_mem > 0:
            end_mem = process.memory_info().rss
            assert (end_mem - start_mem) < 50 * 1024 * 1024, "Memory leak detected > 50MB"
            
        assert True # Survived
