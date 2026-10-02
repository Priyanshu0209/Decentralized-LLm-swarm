import asyncio
import logging
import time
import threading
import subprocess
import os
import sys
import shutil
from typing import Optional, Tuple
import mavsdk
from mavsdk import offboard
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw



import asyncio
import logging
import time
import subprocess
import os
import sys
import shutil
import socket
import inspect
import traceback
import mavsdk

from mavsdk_backend_base import MAVSDKBackendBase

class SimulationBackend(MAVSDKBackendBase):
    def __init__(self, config: dict, logger: logging.Logger):
        super().__init__(config, logger.getChild("SimulationBackend"))
        
        self.grpc_port = config.get('port', 50051)
        self.host = config.get('host', 'localhost')
        mode = config.get('mode', 'simulation').lower()
        
        self._server_process = None
        self._use_existing_server = False
        self._embedded_server = False
        self._sys_kwargs = {}
        
        self.logger.info("Startup logs:")
        self.logger.info(f"Backend mode: {mode}")
        self.logger.info(f"Host: {self.host}")
        self.logger.info(f"gRPC port: {self.grpc_port}")
        self.logger.info(f"Connection URL: {self.connection_url}")
        self.logger.info(f"Drone ID: {self.drone_id}")

    def _init_mavsdk(self):
        mode = self.config.get('mode', 'simulation').lower()
        
        server_running = False
        self.logger.info("Waiting for external MAVSDK...")
        
        for attempt in range(30):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.0)
                if s.connect_ex((self.host, self.grpc_port)) == 0:
                    server_running = True
                    break
            time.sleep(1.0)
                
        if server_running:
            self.logger.info("External MAVSDK detected.")
            self.logger.info("Using external MAVSDK.")
            self._use_existing_server = True
            self._embedded_server = False
        else:
            self.logger.info("External MAVSDK not found after timeout.")
            self.logger.info("Starting embedded MAVSDK...")
            self._use_existing_server = False
            self._embedded_server = True
            
        sys_sig = inspect.signature(mavsdk.System.__init__)
        
        self._sys_kwargs = {}
        if 'mavsdk_server_address' in sys_sig.parameters:
            self.logger.info("Using MAVSDK System(mavsdk_server_address=...) API")
            self._drone = mavsdk.System(mavsdk_server_address=self.host, port=self.grpc_port)
        else:
            self.logger.info("Using newer MAVSDK System() API")
            self._drone = mavsdk.System()
            conn_sig = inspect.signature(self._drone.connect)
            if 'system_address' in conn_sig.parameters:
                self._sys_kwargs = {'system_address': self.connection_url}

    async def _connect(self):
        max_retries = 5
        retry_delay = 2.0
        
        if not self._use_existing_server and 'system_address' not in self._sys_kwargs:
            mavsdk_server_path = shutil.which("mavsdk_server")
            if not mavsdk_server_path:
                try:
                    import mavsdk.bin
                    mavsdk_server_path = os.path.join(os.path.dirname(mavsdk.bin.__file__), "mavsdk_server")
                    if not os.path.exists(mavsdk_server_path) and sys.platform.startswith('linux'):
                        import glob
                        matches = glob.glob(mavsdk_server_path + "*")
                        if matches:
                            mavsdk_server_path = matches[0]
                except ImportError:
                    pass  # Intentionally empty block
            
            if not mavsdk_server_path or not os.path.exists(mavsdk_server_path):
                self.logger.error("Could not find mavsdk_server executable. Is MAVSDK installed?")
                return
                
            self.logger.info(f"Launching embedded MAVSDK server subprocess: {mavsdk_server_path}")
            self._server_process = subprocess.Popen(
                [mavsdk_server_path, "-p", str(self.grpc_port), self.connection_url],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            await asyncio.sleep(1.0)
            
            if self._server_process.poll() is not None:
                self.logger.error("Embedded MAVSDK server failed to start or crashed immediately.")
                return
        
        if 'system_address' not in self._sys_kwargs:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)
                if s.connect_ex((self.host, self.grpc_port)) != 0:
                    self.logger.error(f"Cannot reach MAVSDK server at {self.host}:{self.grpc_port}")
                    return
            self.logger.info(f"Connecting to MAVSDK server at {self.host}:{self.grpc_port}")
        else:
            self.logger.info(f"Connecting to MAVSDK using system_address API at {self.connection_url}")
        
        try:
            self.logger.info("Attempting System.connect()")
            await asyncio.wait_for(self._drone.connect(**self._sys_kwargs), timeout=30.0)
        except asyncio.TimeoutError:
            self.logger.error("Timeout: MAVSDK System failed to connect")
            return
        except Exception as e:
            self.logger.exception(f"Failed to connect to MAVSDK server: {repr(e)}\n{traceback.format_exc()}")
            return
            
        for attempt in range(max_retries):
            self.logger.info(f"Waiting connection (Attempt {attempt + 1}/{max_retries})")
            discovered = False
            try:
                async def wait_for_discovery():
                    self.logger.info("Waiting for vehicle...")
                    async for state in self._drone.core.connection_state():
                        if state.is_connected:
                            self.logger.info("Vehicle discovered")
                            self.logger.info("Connected")
                            if hasattr(state, 'uuid'):
                                self.logger.info(f"UUID: {state.uuid}")
                            return True
                    return False
                discovered = await asyncio.wait_for(wait_for_discovery(), timeout=30.0)
            except asyncio.TimeoutError:
                self.logger.warning("Connection state check timed out.")
            except Exception as e:
                self.logger.exception(f"Connection state error: {repr(e)}\n{traceback.format_exc()}")
                
            if discovered:
                if not self._use_existing_server:
                    self.logger.info("Embedded server started successfully and is accepting connections.")
                
                try:
                    info = await self._drone.info.get_identification()
                    self.logger.info(f"UUID: {info.hardware_uid}")
                except Exception as e:
                    self.logger.exception(f"Failed to get UUID: {e}")
                    import traceback
                    traceback.print_exc()

                self.logger.info("Waiting for Health...")
                healthy = False
                try:
                    async def wait_for_health():
                        async for health in self._drone.telemetry.health():
                            if health.is_global_position_ok and health.is_home_position_ok:
                                self.logger.info(f"Health: GPS OK, Global position OK")
                                return True
                            else:
                                self.logger.info(f"Health: waiting for conditions...")
                        return False
                    healthy = await asyncio.wait_for(wait_for_health(), timeout=30.0)
                except asyncio.TimeoutError:
                    self.logger.warning("Health check timed out.")
                except Exception as e:
                    self.logger.exception(f"Health check error: {repr(e)}\n{traceback.format_exc()}")
                    
                if healthy:
                    self.logger.info("Ready")
                    self._connected = True
                    return
                else:
                    self.logger.warning("Health conditions not met.")
                    
            if attempt < max_retries - 1:
                self.logger.info(f"Retrying connection in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
                
        self.logger.error("Failed to connect to SITL simulation after maximum retries.")

    def stop(self):
        super().stop()
        if self._server_process is not None:
            self.logger.info("Terminating embedded MAVSDK server subprocess...")
            self._server_process.terminate()
            try:
                self._server_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._server_process.kill()
            self._server_process = None
