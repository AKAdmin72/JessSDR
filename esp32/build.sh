#!/bin/bash
# Builds the ESP32-S3 UI firmware with ESP-IDF (installed in ~/esp/esp-idf).
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HOME/esp/esp-idf/export.sh" >/dev/null
cd "$PROJECT_DIR"
idf.py build
