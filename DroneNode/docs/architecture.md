# Architecture Overview

## Objective
The DroneNode architecture provides a unified execution engine for both AirSim simulation and real PX4 drone deployment. It abstracts away hardware idiosyncrasies behind the `HardwareInterface`, ensuring that the core decision logic, state machine, and communication stack never require source code modifications when switching environments.

## Components

### 1. `HardwareInterface` (`backend.py`)
An abstract base class (ABC) defining the universal contract that all backends must fulfill:
- `connect()`, `reconnect()`, `is_connected()`
- `arm()`, `disarm()`, `takeoff(altitude)`, `land()`, `return_to_launch()`
- `is_armed()`
- `set_offboard_mode()`
- `set_position_ned()`, `set_velocity_ned()`

### 2. Backends
- **`MAVSDKBackendBase`**: Inherits from `HardwareInterface` and handles common MAVSDK asyncio event loops, actions, and offboard loops.
- **`SimulationBackend`**: Overrides specific methods for SITL or AirSim environments (e.g. bypassing physical pre-flight checks).
- **`RealDroneBackend`**: Implements strict checks suitable for physical flight (e.g. strict GPS constraints).

### 3. Core Subsystems
The system is heavily threaded, managed centrally by `drone_main.DroneNode`:
- **`StateMachine`**: Manages flight transitions (e.g., READY -> ARMING -> ARMED -> TAKEOFF -> LAND). Ensures state constraints are respected.
- **`Telemetry`**: Polls vehicle telemetry (GPS, Battery, Flight Mode, In-Air) and broadcasts it to the mesh network via GCS.
- **`Heartbeat`**: Continuously broadcasts UDP heartbeat packets to the GCS network to indicate node liveness.
- **`Communication`**: A dedicated `socket` listener thread intercepting UDP commands and pushing them onto the `recv_queue`.
- **`DecisionEngine`**: Evaluates active states and goals (e.g. during MISSION or FORMATION mode) to compute setpoints.
- **`MovementController`**: Translates setpoints into physical offboard MAVSDK commands.

### 4. Comprehensive Watchdog
Runs within `DroneNode._comprehensive_watchdog_loop` and guarantees thread integrity.
- It detects MAVSDK disconnections and restarts the MAVSDK backend.
- It monitors thread liveness (`heartbeat`, `telemetry`, `decision_engine`, `movement_controller`). If a thread dies, it is cleanly restarted.

## Workflow
1. Start `python3 drone_main.py --config config/base.yaml config/simulation.yaml config/drone1.yaml`.
2. `DroneNode` parses YAML and initiates `BackendFactory`.
3. Threads are spawned. The Node enters `DroneState.BOOT`.
4. Once MAVSDK connects and telemetry stabilizes, it transitions to `DroneState.READY`.
5. The GCS sends a `CommandPacket` (e.g., `arm`).
6. `Communication` pushes it to `recv_queue`.
7. The `command_processor_loop` intercepts it, acknowledges it with `executing`, and requests a state transition.
8. The `StateMachine` initiates the MAVSDK command (e.g., `_drone.action.arm()`).
9. On success, state is transitioned to `ARMED`, and a `completed` ACK is sent.
