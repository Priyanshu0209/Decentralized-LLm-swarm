#!/usr/bin/env python3
"""
Telemetry Module for DroneAgent

Subscribes to MAVSDK telemetry and provides the current drone state.
"""

import threading
import asyncio
import logging
import math
from typing import Optional, Tuple
import mavsdk
from mavsdk import telemetry

class Telemetry:
    def __init__(self, config: dict, logger: logging.Logger, backend=None):
        """
        Initialize telemetry module.

        Args:
            config: Configuration dictionary
            logger: Logger instance
            backend: BackendInterface instance (for accessing the drone object)
        """
        self.config = config
        self.logger = logger.getChild("Telemetry")
        self.backend = backend
        self.drone = backend.drone if backend else None
        self._lock = threading.Lock()

        # Telemetry values
        self._gps = (0.0, 0.0, 0.0)  # lat, lon, alt
        self._local_position = (0.0, 0.0, 0.0)  # north, east, down (relative to home)
        self._velocity = (0.0, 0.0, 0.0)  # north, east, down
        self._heading = 0.0  # radians
        self._battery = 1.0  # 0.0 to 1.0
        self._health = True  # True if healthy
        self._in_air = False
        self._armed = False
        self._home_position = None
        self._flight_mode = ""

        # Subscription tasks
        self._running = False
        self._tasks = []
        self._loop = None
        self._thread = None
        self._reconnecting = False

    def start(self):
        """Start telemetry background tasks using the backend's event loop."""
        if not self.backend or not self.backend.loop:
            self.logger.warning("No backend event loop available")
            return
        
        self._running = True
        self.drone = self.backend.drone  # Grab the newly initialized drone object!
        self._loop = self.backend.loop
        
        # We must schedule the tasks on the event loop in a thread-safe way
        asyncio.run_coroutine_threadsafe(self._start_tasks_coro(), self._loop)
        self.logger.info("Telemetry background tasks scheduled")

    async def _start_tasks_coro(self):
        try:
            # Set telemetry rates to reduce MAVSDK server load
            await self.drone.telemetry.set_rate_position(10.0)
            await self.drone.telemetry.set_rate_position_velocity_ned(10.0)
            await self.drone.telemetry.set_rate_attitude(10.0)
            await self.drone.telemetry.set_rate_battery(1.0)
            await self.drone.telemetry.set_rate_in_air(5.0)
        except Exception as e:
            self.logger.warning(f"Could not set telemetry rates: {e}")

        self._tasks = [
            self._loop.create_task(self._gps_loop()),
            self._loop.create_task(self._position_velocity_ned_loop()),
            self._loop.create_task(self._attitude_euler_angles()),
            self._loop.create_task(self._battery_loop()),
            self._loop.create_task(self._health_loop()),
            self._loop.create_task(self._in_air_loop()),
            self._loop.create_task(self._armed_loop()),
            self._loop.create_task(self._home_loop()),
        ]

    def stop(self):
        """Stop telemetry subscriptions."""
        self._running = False
        if self._loop:
            asyncio.run_coroutine_threadsafe(self._shutdown_loop(), self._loop)
        self.logger.info("Telemetry stopped")

    async def _shutdown_loop(self):
        """Cancel all tasks and wait for them to finish."""
        for task in self._tasks:
            if not task.done():
                task.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()

    def _trigger_reconnect(self, e: Exception):
        """Notify the main watchdog that the connection was lost."""
        error_msg = str(e)
        if "UNAVAILABLE" in error_msg or "Connection refused" in error_msg or "AioRpcError" in error_msg:
            with self._lock:
                if self._reconnecting:
                    return
                self._reconnecting = True
            
            self.logger.warning(f"MAVSDK Connection lost ({error_msg})! Alerting watchdog...")
            if self.backend:
                self.backend._connected = False

    async def _gps_loop(self):
        """Subscribe to GPS telemetry."""
        try:
            async for position in self.drone.telemetry.position():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                self._gps = (position.latitude_deg, position.longitude_deg, position.relative_altitude_m)
                # Note: relative_altitude_m is above takeoff, not above ground
                await asyncio.sleep(0.1)  # 10Hz
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"GPS telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _position_velocity_ned_loop(self):
        """Subscribe to position and velocity in NED frame."""
        try:
            async for position_velocity in self.drone.telemetry.position_velocity_ned():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                self._local_position = (
                    position_velocity.position.north_m,
                    position_velocity.position.east_m,
                    position_velocity.position.down_m
                )
                self._velocity = (
                    position_velocity.velocity.north_m_s,
                    position_velocity.velocity.east_m_s,
                    position_velocity.velocity.down_m_s
                )
                await asyncio.sleep(0.1)  # 10Hz
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"Position velocity NED telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _attitude_euler_angles(self):
        """Subscribe to attitude (for yaw)."""
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                # Convert yaw from degrees to radians
                self._heading = math.radians(attitude.yaw_deg)
                await asyncio.sleep(0.1)  # 10Hz
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"Attitude telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _battery_loop(self):
        """Subscribe to battery telemetry."""
        try:
            async for battery in self.drone.telemetry.battery():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                self._battery = battery.remaining_percent / 100.0
                await asyncio.sleep(1.0)  # 1Hz is enough for battery
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"Battery telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _health_loop(self):
        """Subscribe to health telemetry."""
        try:
            async for health in self.drone.telemetry.health():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                self._health = health.is_global_position_ok and health.is_home_position_ok
                await asyncio.sleep(0.5)  # 2Hz
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"Health telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _in_air_loop(self):
        """Subscribe to in-air state."""
        try:
            async for in_air in self.drone.telemetry.in_air():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                self._in_air = in_air
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"In-air telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _armed_loop(self):
        """Subscribe to armed state."""
        try:
            async for armed in self.drone.telemetry.armed():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                self._armed = armed
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"Armed telemetry error: {e}")
            self._trigger_reconnect(e)

    async def _home_loop(self):
        try:
            async for home in self.drone.telemetry.home():
                if not self._running:
                    break
                if not self.backend.is_connected():
                    await asyncio.sleep(1.0)
                    continue
                with self._lock:
                    self._home_position = (home.latitude_deg, home.longitude_deg, home.absolute_altitude_m)
        except asyncio.CancelledError:
            self.logger.debug("Task cancelled")
        except Exception as e:
            self.logger.error(f"Home telemetry error: {e}")
            self._trigger_reconnect(e)

    # Getter methods
    def get_gps(self) -> Optional[Tuple[float, float, float]]:
        """Get GPS coordinates (latitude, longitude, altitude)."""
        return self._gps

    def get_local_position(self) -> Optional[Tuple[float, float, float]]:
        """Get local position (north, east, down) relative to home."""
        with self._lock:
            if self._local_position == (0.0, 0.0, 0.0):
                return None
            return self._local_position

    def get_home(self) -> Optional[Tuple[float, float, float]]:
        """Get home GPS position (lat, lon, alt)."""
        with self._lock:
            return self._home_position

    def get_velocity(self) -> Optional[Tuple[float, float, float]]:
        """Get velocity (north, east, down)."""
        return self._velocity

    def get_heading(self) -> Optional[float]:
        """Get heading in radians."""
        return self._heading

    def get_battery(self) -> Optional[float]:
        """Get battery level (0.0 to 1.0)."""
        return self._battery

    def get_health(self) -> Optional[float]:
        """Get health indicator (0.0 to 1.0)."""
        return self._health

    def is_armed(self) -> Optional[bool]:
        """Check if drone is armed."""
        return self._armed

    def is_in_air(self) -> Optional[bool]:
        """Check if drone is in air."""
        return self._in_air

    def get_state(self) -> dict:
        """Get the complete current telemetry state as a dictionary in a thread-safe manner."""
        import time
        with self._lock:
            gps_lat, gps_lon, gps_alt = self._gps if self._gps else (0.0, 0.0, 0.0)
            loc_x, loc_y, loc_z = self._local_position if self._local_position else (0.0, 0.0, 0.0)
            vel_x, vel_y, vel_z = self._velocity if self._velocity else (0.0, 0.0, 0.0)
            
            state = {
                'latitude': gps_lat,
                'longitude': gps_lon,
                'altitude': gps_alt,
                'local_x': loc_x,
                'local_y': loc_y,
                'local_z': loc_z,
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': self._heading if self._heading is not None else 0.0,
                'battery': self._battery if self._battery is not None else 0.0,
                'armed': bool(self._armed),
                'in_air': bool(self._in_air),
                'flight_mode': self._flight_mode if self._flight_mode else "UNKNOWN",
                'health': bool(self._health),
                'gps_fix': True,
                'heading': self._heading if self._heading is not None else 0.0,
                'velocity': (vel_x, vel_y, vel_z),
                'neighbor_count': 0,
                'timestamp': time.time()
            }
        return state

if __name__ == "__main__":
    # This module requires a running MAVSDK connection to test
    print("Telemetry module")