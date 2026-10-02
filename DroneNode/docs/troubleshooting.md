# Troubleshooting Guide

## General Diagnostic Strategy
Always check the standard out logs. The `DroneNode` employs verbose logging across all subsystems. 

## Common Issues

### 1. "Backend connection lost! Triggering EMERGENCY."
- **Cause**: The connection between the Node and the MAVSDK Server or Pixhawk dropped.
- **Resolution**: 
  - The internal Watchdog will attempt to reconnect up to 10 times using exponential backoff.
  - If using a physical drone, check the telemetry serial cable.
  - If using SITL, ensure the simulator hasn't crashed.

### 2. "Cannot arm: No GPS lock"
- **Cause**: The `StateMachine` refuses to transition to `ARMED` if the telemetry indicates the GPS has fewer than the required satellites or lacks a 3D fix.
- **Resolution**:
  - In simulation, wait a few more seconds for the EKF to settle and the simulated GPS to lock.
  - In reality, ensure the drone is outside and has a clear view of the sky.

### 3. Missing ACKs in GCS
- **Cause**: The GCS issues a command (e.g., `arm 1`), but receives no `executing` or `completed` response.
- **Resolution**:
  - Verify network integrity. The system uses UDP broadcasts (`255.255.255.255`) and direct unicast replies. Firewalls may drop these packets.
  - Ensure `network_config.yaml` or `base.yaml` has the correct `mesh_port` (14561).

### 4. "Watchdog: Heartbeat thread died! Restarting..."
- **Cause**: An unhandled exception crashed one of the critical subsystem threads.
- **Resolution**:
  - The watchdog will automatically restart the thread, so the drone should remain operational.
  - Inspect the tracebacks in the logs just before this message to patch the underlying software fault.

### 5. "Receive queue full, dropping packet"
- **Cause**: The node is receiving UDP commands faster than the `command_processor_loop` can execute them.
- **Resolution**:
  - This typically happens during stress testing or if the GCS is spamming commands in a loop without waiting for ACKs.
  - Respect the ACK lifecycle (Wait for `completed` before sending the next structural command).
