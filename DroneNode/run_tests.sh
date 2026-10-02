#!/bin/bash
set -e

echo "Running DroneNode Tests..."

# Activate venv if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Check if pytest is installed
if ! command -v pytest &> /dev/null
then
    echo "pytest could not be found, installing in venv..."
    pip install pytest pytest-asyncio
fi

# Run tests
cd "$(dirname "$0")"
python -m pytest tests/ -v

echo "All tests passed successfully!"
