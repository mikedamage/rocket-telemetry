/**
 * Ground Station WiFi LR - ESP-NOW Version
 *
 * Receives telemetry data from rocket via ESP-NOW and forwards to laptop via USB serial.
 * Receives control commands from laptop via serial and forwards to rocket via ESP-NOW.
 */

#include <Arduino.h>
#include "espnow_comms.h"
#include "../../shared/espnow_protocol.h"

// Ensure MAC address is defined at build time
#ifndef ROCKET_MAC
#error "ROCKET_MAC must be set as an environment variable (e.g., 0x7C,0xDF,0xA1,0x11,0x22,0x33)"
#endif

// Parse rocket MAC address from build flags
const uint8_t rocketMacAddress[6] = { ROCKET_MAC };

// Serial command buffer
const size_t SERIAL_BUFFER_SIZE = 256;
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferIndex = 0;

/**
 * Log a debug message to Serial with "LOG: " prefix
 */
void logDebug(const char* format, ...) {
  Serial.print("LOG: ");

  va_list args;
  va_start(args, format);

  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  Serial.println(buffer);

  va_end(args);
}

/**
 * Telemetry callback - called when sensor reading is received from rocket
 *
 * Formats and outputs as CSV with RSSI column:
 * DATA: timestamp,temperature,pressure,altitude,humidity,rssi
 */
void onTelemetryReceived(const SensorReading& reading, int8_t rssi) {
  // Convert fixed-point integers back to floating point for CSV output
  float temp = reading.temperature / 100.0f;
  float pressure = reading.pressure / 1.0f;  // Already in hPa
  float altitude = reading.altitude / 100.0f;
  float humidity = reading.humidity / 100.0f;

  // Output CSV format with RSSI as final column
  Serial.printf("DATA: %lu,%.2f,%.2f,%.2f,%.2f,%d\n",
                reading.timestamp,
                temp,
                pressure,
                altitude,
                humidity,
                rssi);
}

/**
 * Parse and execute a command from serial
 *
 * Supported commands (case-insensitive):
 * - START
 * - STOP
 * - RECALIBRATE
 * - STATS (local command - show statistics)
 */
void processSerialCommand(const char* command) {
  // Convert to uppercase for case-insensitive comparison
  String cmd = String(command);
  cmd.toUpperCase();
  cmd.trim();

  if (cmd == "START") {
    sendCommand(CommandCode::START);
  } else if (cmd == "STOP") {
    sendCommand(CommandCode::STOP);
  } else if (cmd == "RECALIBRATE") {
    sendCommand(CommandCode::RECALIBRATE);
  } else if (cmd == "STATS") {
    // Local command - show statistics
    logDebug("Telemetry packets received: %lu", getTelemetryReceivedCount());
    logDebug("Commands sent: %lu", getCommandsSentCount());
  } else if (cmd.startsWith("LOCAL")) {
    // Handle LOCAL status command (compatibility)
    if (cmd.indexOf("STATUS") >= 0) {
      logDebug("Ground station MAC: %s", WiFi.macAddress().c_str());
      logDebug("ESP-NOW channel: %d", ESPNOW_CHANNEL);
      logDebug("Telemetry received: %lu", getTelemetryReceivedCount());
      logDebug("Commands sent: %lu", getCommandsSentCount());
    }
  } else if (cmd.length() > 0) {
    logDebug("Unknown command: %s", command);
    logDebug("Valid commands: START, STOP, RECALIBRATE, STATS");
  }
}

/**
 * Process incoming serial data
 */
void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';
        processSerialCommand(serialBuffer);
        serialBufferIndex = 0;
      }
    } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialBufferIndex++] = c;
    } else {
      logDebug("ERROR: Command too long");
      serialBufferIndex = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // Allow serial to stabilize

  logDebug("Ground Station WiFi LR (ESP-NOW) starting...");

  // Initialize ESP-NOW and register rocket
  if (!initESPNow(rocketMacAddress, onTelemetryReceived)) {
    logDebug("FATAL: ESP-NOW initialization failed");
    while (1) {
      delay(1000);
    }
  }

  logDebug("Ground station ready");
  logDebug("Waiting for telemetry from rocket...");
  logDebug("Available commands: START, STOP, RECALIBRATE, STATS");
}

void loop() {
  // Process serial commands from laptop
  processSerialInput();

  // ESP-NOW telemetry reception is handled via callback (onTelemetryReceived)
  // No polling needed

  delay(1);  // Small delay to prevent watchdog issues
}
