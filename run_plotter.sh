#!/bin/bash

# Liberty Pad Serial Plotter Launcher
# This script activates the virtual environment and runs the plotting script

echo "Liberty Pad Serial Data Plotter"
echo "==============================="

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Error: Virtual environment not found. Please run:"
    echo "python3 -m venv venv"
    echo "source venv/bin/activate"
    echo "pip install pyserial matplotlib"
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

# Check if ESP32 is connected
if [ ! -e "/dev/cu.usbmodem1101" ]; then
    echo "Warning: ESP32 not found at /dev/cu.usbmodem1101"
    echo "Available serial ports:"
    ls /dev/cu.* 2>/dev/null || echo "No USB serial ports found"
    echo ""
    echo "You can specify a different port with: --port /dev/cu.yourport"
fi

echo "Starting plotter..."
echo "Press Ctrl+C to stop"
echo ""

# Run the plotter with arguments passed to this script
python3 serial_plotter.py "$@"