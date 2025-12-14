# Arduino CLI Setup Guide

This guide covers how to compile the RocketWifi and GroundStationWifiLR sketches using arduino-cli.

## Prerequisites

You already have arduino-cli installed at `/opt/homebrew/bin/arduino-cli`.

## 1. Install ESP32 Board Platform

Both sketches use ESP32 boards, so you need to install the ESP32 platform:

```bash
# Add ESP32 board manager URL
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Update board index
arduino-cli core update-index

# Install ESP32 platform (this may take a few minutes)
arduino-cli core install esp32:esp32
```

## 2. Install Required Libraries

### For RocketWifi (TinyPICO)

```bash
# Sensor libraries
arduino-cli lib install "Adafruit BME280 Library"
arduino-cli lib install "Adafruit Unified Sensor"

# Web server libraries
arduino-cli lib install "ESPAsyncWebServer"
arduino-cli lib install "AsyncTCP"

# JSON library
arduino-cli lib install "ArduinoJson"

# BLE library
arduino-cli lib install "NimBLE-Arduino"

# TinyPICO helper library
arduino-cli lib install "TinyPICO Helper Library"
```

### For GroundStationWifiLR (Seeed XIAO ESP32S3)

```bash
# JSON library (if not already installed)
arduino-cli lib install "ArduinoJson"

# Note: WiFi, WiFiUdp, HTTPClient, and esp_wifi are built-in to ESP32 core
```

## 3. Board FQBNs (Fully Qualified Board Names)

### RocketWifi - TinyPICO
```
esp32:esp32:tinypico
```

### GroundStationWifiLR - Seeed XIAO ESP32S3
```
esp32:esp32:XIAO_ESP32S3
```

To list all available ESP32 boards:
```bash
arduino-cli board listall esp32
```

## 4. Compile the Sketches

### Compile RocketWifi

```bash
cd /Users/mike/dev/rocket-telemetry

arduino-cli compile \
  --fqbn esp32:esp32:tinypico \
  --warnings all \
  RocketWifi/
```

### Compile GroundStationWifiLR

```bash
cd /Users/mike/dev/rocket-telemetry

arduino-cli compile \
  --fqbn esp32:esp32:XIAO_ESP32S3 \
  --warnings all \
  GroundStationWifiLR/
```

## 5. Upload to Board

### Find Connected Board

```bash
arduino-cli board list
```

This will show connected boards and their ports (e.g., `/dev/cu.usbserial-*` or `/dev/ttyUSB*`).

### Upload RocketWifi

```bash
arduino-cli upload \
  --fqbn esp32:esp32:tinypico \
  --port /dev/cu.usbserial-XXXXXXXX \
  RocketWifi/
```

### Upload GroundStationWifiLR

```bash
arduino-cli upload \
  --fqbn esp32:esp32:XIAO_ESP32S3 \
  --port /dev/cu.usbserial-XXXXXXXX \
  GroundStationWifiLR/
```

## 6. Combined Compile + Upload

You can combine both steps:

```bash
# RocketWifi
arduino-cli compile --upload \
  --fqbn esp32:esp32:tinypico \
  --port /dev/cu.usbserial-XXXXXXXX \
  RocketWifi/

# GroundStationWifiLR
arduino-cli compile --upload \
  --fqbn esp32:esp32:XIAO_ESP32S3 \
  --port /dev/cu.usbserial-XXXXXXXX \
  GroundStationWifiLR/
```

## 7. Serial Monitor

To view serial output:

```bash
arduino-cli monitor --port /dev/cu.usbserial-XXXXXXXX --config baudrate=115200
```

Press Ctrl+C to exit the monitor.

## Troubleshooting

### Library Not Found

If a library isn't found during compilation, search for it:

```bash
arduino-cli lib search <library-name>
```

Then install the correct one:

```bash
arduino-cli lib install "<exact-library-name>"
```

### Board Not Found

If the FQBN isn't recognized, list all boards:

```bash
arduino-cli board listall
```

### Permission Denied (Linux/Mac)

You may need to add your user to the dialout group or use sudo:

```bash
# Mac: Grant permissions in System Settings > Privacy & Security
# Linux: Add user to dialout group
sudo usermod -a -G dialout $USER
# Then log out and back in
```

## Build Scripts

You can create shell scripts to automate compilation:

### build-rocket.sh
```bash
#!/bin/bash
arduino-cli compile --fqbn esp32:esp32:tinypico RocketWifi/
```

### build-ground-station.sh
```bash
#!/bin/bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 GroundStationWifiLR/
```

Make them executable:
```bash
chmod +x build-rocket.sh build-ground-station.sh
```

## Additional Resources

- [arduino-cli Documentation](https://arduino.github.io/arduino-cli/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [TinyPICO Documentation](https://www.tinypico.com/)
- [Seeed XIAO ESP32S3 Wiki](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
