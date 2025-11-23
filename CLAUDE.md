# Project Summary

This repo contains software for a telemetry system that will be placed in the payload section of a small model rocket. The microcontroller and sensor package in the rocket connects to a second microcontroller on the ground via 2.4GHz WiFi. The ground station will be connected to a laptop via USB serial and forward the data it receives to a program running on the laptop that will log it to a CSV file.

# Rocket Specs

The rocket is an Estes Cosmic Cargo rocket kit. It weighs 36 grams without payload or engine. The telemetry package weighs an additional 36 grams. The rocket will be powered by a C6-5 model rocket engine.

# Technical Details

## Rocket Telemetry Package

- The microcontroller abord the rocket is a TinyPico v2 dev board.
- Connected to the board via I2C is a BME-280 temperature, barometric pressure, and humidity sensor.
- The sensor will be sampled several times per second and its readings stored in a buffer.
- When the buffer is full, all readings are appended to a CSV file stored in a LittleFS partition on the chip's onboard flash memory.
- Data will also be sent via UDP to the ground station's IP address.

## Ground Station

- The ground station will be an ESP32S3 (specifically a Seeed XIAO ESP32S3 board) with a 5dBi omnidirectional 2.4GHz antenna connected to its IPEX socket.
- It will act as a WiFi AP using the ESP32 proprietary 802.11 LR mode.
- UDP packets received from the rocket will be forwarded to the laptop via the UART connection

## Laptop Data Logger

- The laptop's data logger program will listen to the USB serial port socket corresponding to the ground station.
- Each reading will be converted to a CSV string and appended to the CSV file specified at runtime.
- Information about each packet will be logged to STDOUT
