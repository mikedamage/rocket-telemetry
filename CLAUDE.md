# Project Summary

This repo contains software for a telemetry system that will be placed in the payload section of a small model rocket. The microcontroller and sensor package in the rocket communicates with a second microcontroller on the ground via ESP-NOW messages over 802.11LR. The ground station will be connected to a laptop via USB serial and forward the data it receives to a program running on the laptop that will log it to a CSV file.

# Rocket Specs

The rocket is an Estes Cosmic Cargo rocket kit. It weighs 36 grams without payload or engine. The telemetry package weighs an additional 36 grams. The rocket will be powered by a C6-5 model rocket engine.

# Technical Details

## Rocket Telemetry Package

- The microcontroller abord the rocket is a TinyPico v2 dev board.
- Connected to the board via I2C is a BME-280 temperature, barometric pressure, and humidity sensor.
- The sensor will be sampled several times per second and its readings transmitted as ESP-NOW messages to the ground station.
- Readings are also appended to a CSV file stored in a LittleFS partition on the chip's onboard flash memory.

## Ground Station

- The ground station will be an ESP32S3 (specifically a Seeed XIAO ESP32S3 board) with a 5dBi omnidirectional 2.4GHz antenna connected to its IPEX socket.
- It will act as an ESP-NOW peer using the ESP32 proprietary 802.11 LR mode.
- ESP-NOW messages received from the rocket will be forwarded to the laptop via the UART connection

## Laptop Data Logger

- The laptop's data logger program will listen to the USB serial port socket corresponding to the ground station.
- Each reading will be converted to a CSV string and appended to the CSV file specified at runtime.
- The data logger program will also listen for simple text commands on stdin and send them to the ground station ESP to be relayed to the rocket.

# Workflow

## Commands

- Compile the rocket code: `cd RocketWifi && source .venv/bin/activate && pio run`
- Compile ground station code: `cd GroundStationWifiLR && source .venv/bin/activate && pio run`
