import pytest
import time
import threading
from unittest.mock import patch, MagicMock
from drone_main import DroneNode

def test_watchdog_restarts_threads():
    """Verify that watchdog detects dead threads and restarts them."""
    import logging
    logging.getLogger().setLevel(logging.CRITICAL)
    
    with patch('simulation_backend.SimulationBackend.connect', return_value=True):
        node = DroneNode(["config/base.yaml", "config/simulation.yaml", "config/drone1.yaml"])

        node_thread = threading.Thread(target=node.start, daemon=True)
        node_thread.start()
        time.sleep(1)
        
        # Keep original thread refs to prove they changed
        old_heartbeat_thread = node.heartbeat._thread
        old_decision_thread = node.decision_engine._thread
        old_movement_thread = node.movement_controller._thread
        
        # Inject faults: Kill the threads natively
        node.heartbeat.stop()
        node.decision_engine.stop()
        node.movement_controller.stop()
        
        # Wait for threads to naturally die
        time.sleep(0.5)
        
        assert not node.heartbeat._thread.is_alive()
        assert not node.decision_engine._thread.is_alive()
        assert not node.movement_controller._thread.is_alive()
        
        # Watchdog checks every 2 seconds. Give it up to 3 seconds.
        time.sleep(3)
        
        # Verify Watchdog recreated them
        assert node.heartbeat._thread is not old_heartbeat_thread
        assert node.decision_engine._thread is not old_decision_thread
        assert node.movement_controller._thread is not old_movement_thread
        
        assert node.heartbeat._thread.is_alive()
        assert node.decision_engine._thread.is_alive()
        assert node.movement_controller._thread.is_alive()
        
        node.shutdown()
        node_thread.join(timeout=2)
