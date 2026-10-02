import asyncio
import logging
import time
import threading
from typing import Optional, Tuple, Dict, Any, List
import mavsdk
from mavsdk import offboard
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw, VelocityBodyYawspeed

from backend_interface import BackendInterface

from enum import IntEnum

class ControlPriority(IntEnum):
    NONE = 0
    HOVER = 1
    DECISION = 2
    MANUAL = 3
    TAKEOFF = 4
    LAND = 4
    RTL = 5
    EMERGENCY = 6

class MAVSDKBackendBase(BackendInterface):
    """
    Common base class for MAVSDK backends (Simulation and Real).
    Handles all common logic like arming, disarming, offboard mode, etc.
    Subclasses must implement _connect() and specific initialization.
    """
    def __init__(self, config: dict, logger: logging.Logger):
        self.config = config
        self.logger = logger
        self.drone_id = config.get('drone_id', 1)
        
        self._connected = False
        self._armed = False
        self._in_air = False
        self._offboard_active = False
        
        self._telemetry_task = None
        self._offboard_task = None
        self._connection_task = None
        self._stop_event = None
        self._thread = None
        
        # Setpoints
        self._position_setpoint = None
        self._velocity_setpoint = None
        self._velocity_body_setpoint = None

        # Control Arbitration
        self._current_owner = "NONE"
        self._current_priority = ControlPriority.NONE
        
        # Connection specifics
        conn_config = config.get('connection', {})
        self.connection_url = conn_config.get('url')
        if not self.connection_url:
            raise ValueError("connection.url is missing in configuration")
            
        self._drone = None
        self._loop_instance = None
        self.telemetry = None
    
    def set_telemetry(self, telemetry):
        self.telemetry = telemetry
    
    @property
    def drone(self):
        return self._drone
        
    @property
    def loop(self):
        return self._loop_instance

    def is_connected(self) -> bool:
        return self._connected

    def is_armed(self) -> bool:
        return self._armed
        
    def is_in_air(self) -> bool:
        return self._in_air
        
    def is_offboard_active(self) -> bool:
        return self._offboard_active

    def connect(self) -> bool:
        self.logger.info(f"Connecting {self.__class__.__name__}...")
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run_loop, daemon=True, name=f"{self.__class__.__name__}Thread")
            self._thread.start()
        
        # Wait until connected to prevent telemetry plugin errors
        start_time = time.time()
        timeout = 60.0
        while not self.is_connected() and time.time() - start_time < timeout:
            if not self._thread.is_alive():
                break
            time.sleep(0.1)
            
        if self.is_connected():
            self.logger.info(f"{self.__class__.__name__} successfully connected to MAVSDK.")
            return True
        else:
            self.logger.error(f"{self.__class__.__name__} connection failed.")
            return False
            
    def reconnect(self) -> bool:
        self.logger.warning(f"Reconnecting {self.__class__.__name__} (Soft reconnect without tearing down event loop)...")
        
        # Do NOT call self.stop() which kills the server and the event loop.
        self._connected = False
        self._armed = False
        self._in_air = False
        self._offboard_active = False
        
        if self._telemetry_task:
            self._telemetry_task.cancel()
            self._telemetry_task = None
            
        if self._offboard_task:
            self._offboard_task.cancel()
            self._offboard_task = None
            
        self._position_setpoint = None
        self._velocity_setpoint = None
        
        # Trigger an internal async reconnect on the existing event loop
        if self._loop_instance and self._loop_instance.is_running():
            try:
                future = asyncio.run_coroutine_threadsafe(self._reconnect_async(), self._loop_instance)
                return future.result(timeout=60.0)
            except Exception as e:
                self.logger.error(f"Async reconnect failed: {e}")
                return False
        else:
            # If loop is dead, fallback to hard connect
            return self.connect()

    async def _reconnect_async(self):
        """Perform the actual connection retry in the asyncio loop."""
        self.logger.info("Attempting async reconnection...")
        await self._connect()
        if self._connected:
            self._telemetry_task = self._loop_instance.create_task(self._telemetry_loop())
            return True
        return False
        
    def _run_loop(self):
        self._loop_instance = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_instance)
        self._stop_event = asyncio.Event()
        
        self._init_mavsdk()
        
        self._loop_instance.run_until_complete(self._main())
        
    def _init_mavsdk(self):
        """Override this in subclasses to initialize self._drone differently if needed."""
        self._drone = mavsdk.System()
        
    async def _main(self):
        await self._connect()
        if self._connected:
            self._telemetry_task = self._loop_instance.create_task(self._telemetry_loop())
            await self._stop_event.wait()
            if self._telemetry_task:
                self._telemetry_task.cancel()
            if self._offboard_task:
                self._offboard_task.cancel()

    async def _connect(self):
        """Override this in subclasses to handle connection logic."""
        raise NotImplementedError

    def stop(self):
        self.logger.info(f"Stopping {self.__class__.__name__}...")
        if self._loop_instance and self._loop_instance.is_running() and self._stop_event:
            self._loop_instance.call_soon_threadsafe(self._stop_event.set)
        if self._thread:
            self._thread.join(timeout=5.0)

    def disconnect(self):
        self.stop()

    async def _telemetry_loop(self):
        # Deprecated: Telemetry is now handled centrally by telemetry.py
        pass  # Intentionally empty block

    def _future_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.logger.error(f"MAVSDK Background Task Error: {e}")

    def arm(self):
        self.logger.info("Starting arming sequence...")
        self.logger.info("Executing backend.arm()")
        try:
            future = asyncio.run_coroutine_threadsafe(self._arm(), self._loop_instance)
            future.result(timeout=10.0)
            self.logger.info("backend.arm() completed")
            return True
        except Exception as e:
            self.logger.exception(f"Failed to arm drone: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _arm(self):
        try:
            self.logger.info("Forcing Hold mode to clear flight states before arming...")
            await self._drone.action.hold()
            await asyncio.sleep(0.5)
            await self._drone.action.arm()
            self._armed = True
        except ActionError as e:
            self.logger.error(f'MAVSDK ActionError in method: {e}')
            raise

    def disarm(self):
        self.logger.info("Starting disarm sequence...")
        self.logger.info("Executing backend.disarm()")
        try:
            future = asyncio.run_coroutine_threadsafe(self._disarm(), self._loop_instance)
            future.result(timeout=10.0)
            self.logger.info("backend.disarm() completed")
            return True
        except Exception as e:
            self.logger.exception(f"Failed to disarm drone: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _disarm(self):
        self._armed = False
        try:
            await self._drone.action.disarm()
        except ActionError as e:
            self.logger.warning(f'MAVSDK ActionError during disarm (ignoring): {e}')

    def takeoff(self, altitude: Optional[float] = None) -> bool:
        self.logger.info(f"Taking off to {altitude}m...")
        self.logger.info("Executing backend.takeoff()")
        try:
            future = asyncio.run_coroutine_threadsafe(self._takeoff(altitude), self._loop_instance)
            future.result(timeout=300.0)
            self.logger.info("backend.takeoff() completed")
            return True
        except Exception as e:
            self.logger.exception(f"Failed to takeoff: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _takeoff(self, altitude: float):
        if not self.is_connected():
            raise Exception("Cannot takeoff: Backend not connected.")
        
        if not self._armed:
            self.logger.info("Auto-arming before takeoff...")
            try:
                await self._drone.action.hold()
                await asyncio.sleep(0.5)
                await self._drone.action.arm()
                self._armed = True
            except ActionError as e:
                self.logger.error(f"Auto-arm failed: {e}")
                raise Exception("Takeoff aborted: Could not arm.")

        await self._drone.action.set_takeoff_altitude(altitude)
        await self._drone.action.takeoff()
        
        # Poll injected telemetry cache instead of subscribing
        while not self._stop_event.is_set():
            if self.telemetry and self.telemetry.is_in_air():
                self._in_air = True
                break
            await asyncio.sleep(0.5)

    def land(self):
        self.logger.info("Landing drone...")
        self.logger.info("Executing backend.land()")
        try:
            future = asyncio.run_coroutine_threadsafe(self._land(), self._loop_instance)
            future.result(timeout=300.0)
            self.logger.info("backend.land() completed")
            return True
        except Exception as e:
            self.logger.exception(f"Failed to land: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _land(self):
        try:
            await self._drone.action.land()
            self._offboard_active = False
            
            # Poll injected telemetry cache instead of subscribing
            while not self._stop_event.is_set():
                if self.telemetry and not self.telemetry.is_in_air():
                    self._in_air = False
                    break
                await asyncio.sleep(0.5)
        except ActionError as e:
            self.logger.error(f'MAVSDK ActionError in method: {e}')
            raise

    def rtl(self):
        self.logger.info("Returning to launch...")
        self.logger.info("Executing backend.rtl()")
        try:
            future = asyncio.run_coroutine_threadsafe(self._rtl(), self._loop_instance)
            future.result(timeout=300.0)
            self.logger.info("backend.rtl() completed")
            return True
        except Exception as e:
            self.logger.exception(f"Failed to RTL: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _rtl(self):
        try:
            await self._drone.action.return_to_launch()
            self._offboard_active = False
        except ActionError as e:
            self.logger.error(f'MAVSDK ActionError in method: {e}')
            raise

    def start_offboard(self):
        self.logger.info("Setting offboard mode on drone...")
        self.logger.info("Executing backend.offboard.start()")
        try:
            future = asyncio.run_coroutine_threadsafe(self._start_offboard(), self._loop_instance)
            future.result(timeout=10.0)
            self.logger.info("backend.offboard.start() completed")
            return True
        except Exception as e:
            self.logger.exception(f"Failed to set offboard mode: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _start_offboard(self):
        try:
            if self._offboard_active and self._offboard_task and not self._offboard_task.done():
                self.logger.info("Offboard mode is already active, ignoring start request.")
                return

            if getattr(self, '_velocity_body_setpoint', None) is not None:
                n, e, d, y = self._velocity_body_setpoint
                await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(n, e, d, y))
            elif self._velocity_setpoint is not None:
                n, e, d, y = self._velocity_setpoint
                await self._drone.offboard.set_velocity_ned(VelocityNedYaw(n, e, d, y))
            elif self._position_setpoint is not None:
                n, e, d, y = self._position_setpoint
                await self._drone.offboard.set_position_ned(PositionNedYaw(n, e, d, y))
            else:
                await self._drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                
            await self._drone.offboard.start()
            self._offboard_active = True
            if self._offboard_task is None or self._offboard_task.done():
                self._offboard_task = self._loop_instance.create_task(self._offboard_control_loop())
        except Exception as e:
            self.logger.error(f"Offboard start failed: {e}")
            raise

    def stop_offboard(self):
        self.logger.info("Stopping offboard mode on drone...")
        try:
            future = asyncio.run_coroutine_threadsafe(self._stop_offboard(), self._loop_instance)
            future.result(timeout=10.0)
            return True
        except Exception as e:
            self.logger.warning(f"Failed to cleanly stop offboard mode (ignoring): {e}")
            return False

    async def _stop_offboard(self):
        await self._drone.offboard.stop()
        self._offboard_active = False
        if self._offboard_task:
            self._offboard_task.cancel()
            self._offboard_task = None

    def request_control(self, owner: str, priority: ControlPriority) -> bool:
        """Arbitrate flight control ownership (Legacy, handled by FCM now)."""
        return True

    def release_control(self, owner: str):
        """Release ownership (Legacy, handled by FCM now)."""
        pass

    def emergency_stop(self):
        self.logger.critical("DRONE EMERGENCY STOP!")
        try:
            future = asyncio.run_coroutine_threadsafe(self._emergency_stop(), self._loop_instance)
            future.result(timeout=10.0)
            return True
        except Exception as e:
            self.logger.exception(f"Emergency stop failed: {e}")
            import traceback
            traceback.print_exc()
            raise

    async def _emergency_stop(self):
        self._armed = False
        self._in_air = False
        self._offboard_active = False
        try:
            await self._drone.action.kill()
        except ActionError as e:
            self.logger.warning(f'MAVSDK ActionError during kill (ignoring): {e}')

    def failsafe(self):
        self.logger.critical("DRONE FAILSAFE activated")
        gps_ok = False
        if self.telemetry:
            gps = self.telemetry.get_gps()
            if gps and (gps[0] != 0.0 or gps[1] != 0.0):
                gps_ok = True
        
        if gps_ok:
            self.logger.info("Failsafe: GPS is OK. Triggering Return to Launch.")
            self.return_to_launch()
        else:
            self.logger.critical("Failsafe: GPS is LOST. Triggering Emergency Land.")
            self.land()

    def pause(self):
        self.logger.info("Pausing vehicle...")
        try:
            future = asyncio.run_coroutine_threadsafe(self._pause(), self._loop_instance)
            future.result(timeout=10.0)
            return True
        except Exception as e:
            self.logger.warning(f"Pause failed (ignoring): {e}")
            return False
            
    async def _pause(self):
        self._offboard_active = False
        if self._offboard_task:
            self._offboard_task.cancel()
            self._offboard_task = None
        try:
            await self._drone.action.hold()
        except ActionError as e:
            self.logger.error(f"MAVSDK Pause ActionError: {e}")
            # Do not raise, we already cleaned up our internal state
            
    def resume(self):
        self.logger.info("Resuming mission...")
        try:
            future = asyncio.run_coroutine_threadsafe(self._resume(), self._loop_instance)
            future.result(timeout=10.0)
            return True
        except Exception as e:
            self.logger.exception(f"Resume failed: {e}")
            import traceback
            traceback.print_exc()
            raise
            
    async def _resume(self):
        # We use custom offboard mission tracking via decision.py
        # Do not call MAVSDK native mission API
        pass

    def move_velocity(self, north_m_s: float, east_m_s: float, down_m_s: float, yaw_deg: float) -> bool:
        if not self._offboard_active:
            self.logger.debug("Offboard not fully active, buffering velocity setpoint")
        self._velocity_setpoint = (north_m_s, east_m_s, down_m_s, yaw_deg)
        self._position_setpoint = None
        self._velocity_body_setpoint = None
        return True

    def move_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yawspeed_deg_s: float) -> bool:
        if not self._offboard_active:
            self.logger.debug("Offboard not fully active, buffering body velocity setpoint")
        self._velocity_body_setpoint = (forward_m_s, right_m_s, down_m_s, yawspeed_deg_s)
        self._velocity_setpoint = None
        self._position_setpoint = None
        return True

    def move_position(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> bool:
        if not self._offboard_active:
            self.logger.debug("Offboard not fully active, buffering position setpoint")
        self._position_setpoint = (north_m, east_m, down_m, yaw_deg)
        self._velocity_setpoint = None
        self._velocity_body_setpoint = None
        return True
        
    def hover(self) -> bool:
        self.logger.info("Hovering...")
        if self._offboard_active:
            self.move_velocity(0.0, 0.0, 0.0, 0.0)
        else:
            self.pause()
        return True
        
    def upload_mission(self, waypoints: List[Dict[str, float]]) -> bool:
        pass  # Intentionally empty block
        
    def start_mission(self) -> bool:
        pass  # Intentionally empty block
        
    def pause_mission(self) -> bool:
        return self.pause()
        
    def resume_mission(self) -> bool:
        return self.resume()
        
    def cancel_mission(self) -> bool:
        return self.pause()
        
    def health(self) -> bool:
        pass  # Intentionally empty block

    async def _offboard_control_loop(self):
        self.logger.info("Starting drone offboard control loop")
        while not self._stop_event.is_set() and self._offboard_active:
            try:
                if self._position_setpoint is not None:
                    north, east, down, yaw_deg = self._position_setpoint
                    await self._drone.offboard.set_position_ned(
                        PositionNedYaw(north, east, down, yaw_deg)
                    )
                elif getattr(self, '_velocity_body_setpoint', None) is not None:
                    forward, right, down, yawspeed = self._velocity_body_setpoint
                    await self._drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(forward, right, down, yawspeed)
                    )
                elif self._velocity_setpoint is not None:
                    north, east, down, yaw_deg = self._velocity_setpoint
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(north, east, down, yaw_deg)
                    )
                else:
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
            except Exception as e:
                self.logger.error(f"Offboard loop command error: {e}")
                # Do not exit loop on single command failure, MAVSDK might recover.
            
            await asyncio.sleep(0.1) # 10Hz
