# Deployment Guide

## Universal Binary Principle
The code running in the simulation environment is identical to the code running on the real physical drone. Do NOT change source files to accommodate the environment. Only the YAML configurations should be swapped.

## Prerequisites
- Python 3.10+
- `pip install -r requirements.txt` (Includes `mavsdk`, `pyyaml`)
- A running MAVSDK-Server (either running natively via `mavsdk` Python wrapper or externally as a binary).

## YAML Configuration Structure
The system relies on deeply merged YAML configurations:
- `base.yaml`: Contains universal settings (e.g., ports, timeout thresholds).
- `simulation.yaml` or `real.yaml`: Environment-specific overrides (e.g., connection types `udp` vs `serial`).
- `droneX.yaml`: Drone-specific configurations (e.g., `drone_id: 1`, localized ports).

## Running in AirSim / SITL
1. Start AirSim or PX4 SITL on your local machine.
2. The default SITL uses `udp://:14540` for Drone 1.
3. Start the node:
```bash
python3 drone_main.py --config config/base.yaml config/simulation.yaml config/drone1.yaml
```

## Running on Real Hardware
1. Ensure the companion computer (e.g., Raspberry Pi, Jetson) is connected to the Pixhawk flight controller via Serial (TELEM2).
2. Create or verify `config/real.yaml` contains the correct Serial URL, e.g., `serial:///dev/ttyAMA0:921600`.
3. Start the node:
```bash
python3 drone_main.py --config config/base.yaml config/real.yaml config/drone1.yaml
```

## Running the GCS (Ground Control Station)
From the GCS machine (must be on the same network):
```bash
cd /home/priyanshu/DroneAgent-GCS
python3 main.py
```
Use the GCS CLI to broadcast commands (e.g., `arm 1`, `takeoff 1`).
