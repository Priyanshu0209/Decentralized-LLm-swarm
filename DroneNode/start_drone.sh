#!/bin/bash
# Start DroneNode (Headless Autonomous Controller)
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Run deploy_drone.sh first."
    exit 1
fi

exec venv/bin/python3 drone_main.py "$@"
