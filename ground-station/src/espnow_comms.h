/**
 * ESP-NOW Communication Module - Ground Station
 *
 * Handles ESP-NOW initialization, receiving telemetry from rocket,
 * and sending commands to rocket.
 */

#ifndef ESPNOW_COMMS_H
#define ESPNOW_COMMS_H

#include "../../shared/espnow_protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/**
 * Callback function type for received telemetry
 * Parameters: SensorReading data, RSSI value
 */
typedef void (*TelemetryCallback)(const SensorReading &reading, int8_t rssi);

/**
 * Callback function type for received file chunks
 * Parameters: FileChunk data
 */
typedef void (*FileChunkCallback)(const FileChunk &chunk);

/**
 * Initialize ESP-NOW and register rocket as peer
 *
 * @param rocketMAC MAC address of the rocket (6 bytes)
 * @param telemetryCallback Function to call when telemetry is received
 * @param fileChunkCallback Function to call when file chunk is received
 * @return true if initialization successful, false otherwise
 */
bool initESPNow(const uint8_t *rocketMAC, TelemetryCallback telemetryCallback,
                FileChunkCallback fileChunkCallback);

/**
 * Send a command to the rocket
 *
 * @param cmd Command code to send
 * @return true if send initiated successfully (does not guarantee delivery)
 */
bool sendCommand(CommandCode cmd);

/**
 * Get ESP-NOW statistics
 */
uint32_t getTelemetryReceivedCount();
uint32_t getCommandsSentCount();

#endif // ESPNOW_COMMS_H
