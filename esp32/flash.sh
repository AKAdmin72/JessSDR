#!/bin/bash
# Flashes the ESP32-S3 UI firmware through the board's COM port (CH343).
# Usage: ./flash [monitor]   - "monitor" also opens the serial monitor
#                              (exit with Ctrl+]).
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ESPPORT:-/dev/ttyACM0}"
. "$HOME/esp/esp-idf/export.sh" >/dev/null
cd "$PROJECT_DIR"
idf.py -p "$PORT" flash $1
