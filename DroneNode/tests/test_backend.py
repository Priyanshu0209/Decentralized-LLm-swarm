import sys
import os
import asyncio
import threading
import pytest
from unittest.mock import MagicMock, AsyncMock, patch
import logging

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from mavsdk_backend_base import MAVSDKBackendBase

class DummyBackend(MAVSDKBackendBase):
    def _init_mavsdk(self):
        self._drone = MagicMock()
        self._drone.connect = AsyncMock()
        
        # Mock core connection state
        self._drone.core = MagicMock()
        async def mock_connection_state():
            state = MagicMock()
            state.is_connected = True
            yield state
        self._drone.core.connection_state = mock_connection_state
        
        # Mock telemetry health
        self._drone.telemetry = MagicMock()
        async def mock_health():
            health = MagicMock()
            health.is_global_position_ok = True
            health.is_home_position_ok = True
            yield health
        self._drone.telemetry.health = mock_health
        
        # Mock telemetry battery
        async def mock_battery():
            bat = MagicMock()
            bat.remaining_percent = 0.95
            yield bat
        self._drone.telemetry.battery = mock_battery
        
        # Mock in_air
        async def mock_in_air():
            yield True
        self._drone.telemetry.in_air = mock_in_air
        
        # Action mocks
        self._drone.action = MagicMock()
        self._drone.action.arm = AsyncMock()
        self._drone.action.disarm = AsyncMock()
        self._drone.action.takeoff = AsyncMock()
        self._drone.action.set_takeoff_altitude = AsyncMock()
        self._drone.action.land = AsyncMock()
        
    async def _connect(self):
        await self._drone.connect(system_address="udp://:14540")
        self._connected = True

@pytest.fixture
def config():
    return {'drone_id': 1, 'connection': {'url': 'udp://:14540'}}

@pytest.fixture
def logger():
    return logging.getLogger("TestBackend")

def test_backend_connect(config, logger):
    backend = DummyBackend(config, logger)
    assert not backend.is_connected()
    
    # Run connect
    success = backend.connect()
    assert success is True
    assert backend.is_connected()
    assert backend._thread.is_alive()
    
    # Teardown
    backend.disconnect()

def test_backend_arm_disarm(config, logger):
    backend = DummyBackend(config, logger)
    backend.connect()
    
    # Arm
    success = backend.arm()
    assert success is True
    backend._drone.action.arm.assert_called_once()
    assert backend.is_armed()
    
    # Disarm
    success = backend.disarm()
    assert success is True
    backend._drone.action.disarm.assert_called_once()
    assert not backend.is_armed()
    
    backend.disconnect()

def test_backend_takeoff_land(config, logger):
    backend = DummyBackend(config, logger)
    backend.connect()
    
    # Inject mock telemetry
    mock_telemetry = MagicMock()
    mock_telemetry.is_in_air.return_value = True
    backend.set_telemetry(mock_telemetry)
    
    # Takeoff
    success = backend.takeoff(15.0)
    assert success is True
    backend._drone.action.set_takeoff_altitude.assert_called_once_with(15.0)
    backend._drone.action.takeoff.assert_called_once()
    assert backend.is_in_air()
    
    # Land
    mock_telemetry.is_in_air.return_value = False
    success = backend.land()
    assert success is True
    backend._drone.action.land.assert_called_once()
    assert not backend.is_in_air()
    
    backend.disconnect()
