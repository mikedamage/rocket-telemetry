/**
 * Ground Station WiFi LR - ESP-NOW Version
 *
 * Receives telemetry data from rocket via ESP-NOW and forwards to laptop via
 * USB serial. Receives control commands from laptop via serial and forwards to
 * rocket via ESP-NOW.
 */

#include "../../shared/espnow_protocol.h"
#include "espnow_comms.h"
#include <Arduino.h>

// Ensure MAC address is defined at build time
#ifndef ROCKET_MAC
#error                                                                         \
    "ROCKET_MAC must be set as an environment variable (e.g., 0x7C,0xDF,0xA1,0x11,0x22,0x33)"
#endif

// Parse rocket MAC address from build flags
const uint8_t rocketMacAddress[6] = {ROCKET_MAC};

/**
 * Runtime state for the ground station
 * Groups all mutable state in one place for clarity
 */
struct GroundStationState {
  static const size_t SERIAL_BUFFER_SIZE = 256;
  char serialBuffer[SERIAL_BUFFER_SIZE] = {0};
  size_t serialBufferIndex = 0;
  bool discardingOverflow = false; // True when consuming overflow input
};

static GroundStationState state;

/**
 * Log a debug message to Serial with "LOG: " prefix
 */
void logDebug(const char *format, ...) {
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
void onTelemetryReceived(const SensorReading &reading, int8_t rssi) {
  // Values are already floats - output directly with appropriate precision
  // Output CSV format with RSSI as final column
  Serial.printf("DATA: %lu,%.2f,%.2f,%.4f,%.2f,%d\n", reading.timestamp,
                reading.temperature, reading.pressure, reading.altitude,
                reading.humidity, rssi);
}

/**
 * File chunk callback - called when file chunk is received from rocket
 *
 * Forwards the chunk data to the laptop via serial
 */
void onFileChunkReceived(const FileChunk &chunk) {
  // Write chunk data directly to serial
  Serial.write(chunk.data, chunk.dataLength);

  // If this is the last chunk, log completion
  if (chunk.flags & FILE_CHUNK_FLAG_LAST) {
    logDebug("File download complete (chunk %d)", chunk.sequenceNumber);
  }
}

/**
 * Parse and execute a command from serial
 *
 * Supported commands (case-insensitive):
 * - START
 * - STOP
 * - RECALIBRATE
 * - DOWNLOAD
 * - TRUNCATE
 * - STATS (local command - show statistics)
 */
void processSerialCommand(const char *command) {
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
  } else if (cmd == "DOWNLOAD") {
    sendCommand(CommandCode::DOWNLOAD);
  } else if (cmd == "TRUNCATE") {
    sendCommand(CommandCode::TRUNCATE);
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
    logDebug(
        "Valid commands: START, STOP, RECALIBRATE, DOWNLOAD, TRUNCATE, STATS");
  }
}

/**
 * Process incoming serial data
 */
void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    // If we're discarding overflow, consume until newline
    if (state.discardingOverflow) {
      if (c == '\n' || c == '\r') {
        state.discardingOverflow = false;
      }
      // Discard this character
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (state.serialBufferIndex > 0) {
        state.serialBuffer[state.serialBufferIndex] = '\0';
        processSerialCommand(state.serialBuffer);
        state.serialBufferIndex = 0;
      }
    } else if (state.serialBufferIndex <
               GroundStationState::SERIAL_BUFFER_SIZE - 1) {
      state.serialBuffer[state.serialBufferIndex++] = c;
    } else {
      // Buffer overflow - discard this character and all subsequent until
      // newline
      logDebug("ERROR: Command too long, discarding input until newline");
      state.serialBufferIndex = 0;
      state.discardingOverflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // Allow serial to stabilize

  logDebug("Ground Station WiFi LR (ESP-NOW) starting...");

  // Initialize ESP-NOW and register rocket
  // Error handling strategy: Critical errors (ESP-NOW init) halt the system
  // for safety. Non-critical errors (serial parsing) are logged and ignored.
  if (!initESPNow(rocketMacAddress, onTelemetryReceived, onFileChunkReceived)) {
    logDebug("FATAL: ESP-NOW initialization failed");
    while (1) {
      delay(1000); // Halt - requires physical reset
    }
  }

  logDebug("Ground station ready");
  logDebug("Waiting for telemetry from rocket...");
  logDebug("Available commands: START, STOP, RECALIBRATE, DOWNLOAD, TRUNCATE, "
           "STATS");
}

void loop() {
  // Process serial commands from laptop
  processSerialInput();

  // ESP-NOW telemetry reception is handled via callback (onTelemetryReceived)
  // No polling needed

  // No explicit delay needed - serial processing is fast and ESP-NOW uses
  // callbacks. If watchdog resets occur, add delay(1) or increase watchdog
  // timeout.
}
