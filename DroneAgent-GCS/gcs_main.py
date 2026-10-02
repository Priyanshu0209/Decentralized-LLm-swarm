#!/usr/bin/env python3
"""
DroneAgent Ground Control Station (GCS) Entry Point

Runs ONLY on the Laptop.
- PySide6 GUI for mission upload, telemetry viewing, swarm monitoring
- Sends CommandPackets to DroneNodes over UDP mesh network
- No MAVSDK, no Pixhawk, no movement controller, no decision engine
"""

import sys
import os
import argparse

# Ensure the GCS directory is on the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gcs_agent import GCSAgent

def main():
    parser = argparse.ArgumentParser(description="DroneAgent Ground Control Station")
    parser.add_argument("-c", "--config", nargs="+", help="Path to config YAML files")
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI (terminal only)")
    args = parser.parse_args()

    configs = args.config if args.config else []

    # Ensure base.yaml is always loaded
    if not any("base.yaml" in c for c in configs):
        configs.insert(0, "config/base.yaml")

    # Ensure network config is loaded
    if not any("network_config.yaml" in c for c in configs):
        configs.append("config/network_config.yaml")

    # If --no-gui, inject into config
    agent = GCSAgent(configs)
    if args.no_gui:
        agent.config['run_gui'] = False

    agent.start()

if __name__ == "__main__":
    main()
