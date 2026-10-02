#!/bin/bash
# Start DroneAgent Ground Control Station
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Run deploy_laptop.sh first."
    exit 1
fi

exec venv/bin/python3 gcs_main.py "$@"
