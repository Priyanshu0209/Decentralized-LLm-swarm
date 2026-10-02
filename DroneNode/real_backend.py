import asyncio
import logging
import time
import threading
from typing import Optional, Tuple
import mavsdk
from mavsdk import offboard
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw



from mavsdk_backend_base import MAVSDKBackendBase

class RealDroneBackend(MAVSDKBackendBase):
    def __init__(self, config: dict, logger: logging.Logger):
        super().__init__(config, logger.getChild("RealDroneBackend"))

    def _init_mavsdk(self):
        self._drone = mavsdk.System(port=0)

    async def _connect(self):
        try:
            self.logger.info(f"Connecting to Real Hardware via {self.connection_url}")
            await asyncio.wait_for(self._drone.connect(system_address=self.connection_url), timeout=5.0)
        except asyncio.TimeoutError:
            self.logger.error("Timeout: MAVSDK server failed to start")
            return
        except Exception as e:
            self.logger.error(f"Failed to start MAVSDK server: {e}")
            return
            
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.logger.info(f"Waiting for drone discovery (Attempt {attempt+1}/{max_retries})...")
                
                async def wait_for_discovery():
                    async for state in self._drone.core.connection_state():
                        if state.is_connected:
                            return True
                    return False
                    
                discovered = await asyncio.wait_for(wait_for_discovery(), timeout=10.0)
                
                if discovered:
                    self.logger.info("Real Drone discovered!")
                    
                    async def wait_for_health():
                        async for health in self._drone.telemetry.health():
                            if health.is_global_position_ok and health.is_home_position_ok:
                                return True
                        return False
                        
                    self.logger.info("Waiting for global position estimate...")
                    healthy = await asyncio.wait_for(wait_for_health(), timeout=30.0)
                    
                    if healthy:
                        self.logger.info("Real Drone Global position estimate OK")
                        self._connected = True
                        return
            except asyncio.TimeoutError:
                self.logger.warning(f"Connection state or health check timed out (Attempt {attempt+1}).")
            except Exception as e:
                self.logger.error(f"Connection check error: {e}")
                
            if attempt < max_retries - 1:
                await asyncio.sleep(2.0)
                
        self.logger.error("Failed to connect to Real Hardware after maximum retries.")
