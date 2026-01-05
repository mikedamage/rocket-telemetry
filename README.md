# Rocket Telemetry System

## Summary

This project implements a wireless telemetry system for a model rocket using ESP32 microcontrollers. The system consists of a sensor package aboard the rocket that transmits temperature, pressure, and humidity data via ESP-NOW to a ground station, which relays the telemetry to a laptop for logging and analysis.

The rocket-based system samples environmental data multiple times per second during flight and stores it both locally to onboard flash memory and transmits it in real-time to the ground. The ground station acts as a relay between the rocket and a USB-connected laptop running data logging software. When a predetermined timeout has elapsed, the rocket stops logging telemetry and activates a BLE beacon to help you locate the landing site.

## Implementation Details

### Rocket Hardware
- **Rocket Model**: Estes Cosmic Cargo kit
- **Engine**: C6-5 model rocket engine
- **Total Mass at Liftoff**: 72 grams (36g rocket + 36g telemetry package)
- **Microcontroller**: TinyPICO v2 ESP32 development board
- **Sensor**: BME-280 (temperature, barometric pressure, and humidity via I2C)
- **Power**: 1000mAh Li-Poly 3.7V battery
- **Storage**: LittleFS partition on onboard flash for CSV logging

### Ground Station Hardware
- **Microcontroller**: Seeed XIAO ESP32S3 development board
- **Antenna**: 5dBi omnidirectional 2.4GHz antenna (IPEX connector)
- **Communication**: ESP-NOW using ESP32 proprietary 802.11 LR mode
- **Computer Link**: USB serial UART connection to laptop

### Laptop Data Logger
- Python-based serial port receiver and command client
- Logs telemetry data to CSV files
- Sends text commands to the rocket via the ground station relay

## Quick Setup

### Prerequisites
- Install [uv](https://docs.astral.sh/uv/) for Python environment management
- Copy `.envrc.example` to `.envrc` and configure your device MAC addresses and runtime parameters
  - Optionally, customize values in `.envrc` to change telemetry timeout, sample rate, ESP-NOW channel, BLE beacon UUID, etc.

### Building and Flashing

1. Set up the Python environment:
   ```bash
   uv sync
   ```

2. Build firmware:
   ```bash
   # Rocket firmware
   cd rocket && uv run pio run

   # Ground station firmware
   cd ground-station && uv run pio run
   ```

3. Flash devices (use correct USB serial device paths):
   ```bash
   # Note: You may need to specify the USB device to flash unless you plug the boards in one at a time

   # Rocket (TinyPICO)
   cd rocket && uv run pio run -t upload

   # Ground station (XIAO ESP32S3)
   cd ground-station && uv run pio run -t upload
   ```

### Debugging

Serial console output can be monitored via `uv run pio device monitor` from inside either the `rocket` or `ground-station` directories.

### Customization
- Board models can be customized by editing `platformio.ini` files in the respective firmware directories
- Runtime parameters (MAC addresses, sensor intervals, ESP-NOW channel, etc.) are configured via environment variables listed in `.envrc.example`
