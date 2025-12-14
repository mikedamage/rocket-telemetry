#!/bin/bash
# Setup script for arduino-cli
# Installs ESP32 platform and required libraries

set -e

echo "======================================"
echo "Arduino CLI Setup for Rocket Telemetry"
echo "======================================"
echo ""

# Initialize arduino-cli config
echo "1. Initializing arduino-cli configuration..."
arduino-cli config init 2>/dev/null || true

# Add ESP32 board manager URL
echo "2. Adding ESP32 board manager URL..."
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Update board index
echo "3. Updating board index..."
arduino-cli core update-index

# Install ESP32 platform
echo "4. Installing ESP32 platform (this may take a few minutes)..."
arduino-cli core install esp32:esp32

echo ""
echo "5. Installing required libraries..."

# Libraries for RocketWifi
echo "   - Adafruit BME280 Library"
arduino-cli lib install "Adafruit BME280 Library" 2>/dev/null || echo "     (already installed)"

echo "   - Adafruit Unified Sensor"
arduino-cli lib install "Adafruit Unified Sensor" 2>/dev/null || echo "     (already installed)"

echo "   - ESPAsyncWebServer"
arduino-cli lib install "ESPAsyncWebServer" 2>/dev/null || echo "     (already installed)"

echo "   - AsyncTCP"
arduino-cli lib install "AsyncTCP" 2>/dev/null || echo "     (already installed)"

echo "   - ArduinoJson"
arduino-cli lib install "ArduinoJson" 2>/dev/null || echo "     (already installed)"

echo "   - NimBLE-Arduino"
arduino-cli lib install "NimBLE-Arduino" 2>/dev/null || echo "     (already installed)"

echo "   - TinyPICO Helper Library"
arduino-cli lib install "TinyPICO Helper Library" 2>/dev/null || echo "     (already installed)"

echo ""
echo "======================================"
echo "✓ Setup complete!"
echo "======================================"
echo ""
echo "You can now compile the sketches:"
echo "  ./build-rocket.sh"
echo "  ./build-ground-station.sh"
echo ""
echo "Or manually:"
echo "  arduino-cli compile --fqbn esp32:esp32:tinypico RocketWifi/"
echo "  arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 GroundStationWifiLR/"
echo ""
