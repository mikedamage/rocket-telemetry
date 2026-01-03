/**
 * ESP-NOW Communication Module - Rocket
 *
 * Handles ESP-NOW initialization, sending telemetry to ground station,
 * and receiving commands from ground station.
 */

#ifndef ESPNOW_COMMS_H
#define ESPNOW_COMMS_H

#include "../../shared/espnow_protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/**
 * Callback function type for received commands
 * Parameter: CommandCode
 */
typedef void (*CommandCallback)(CommandCode cmd);

/**
 * Initialize ESP-NOW and register ground station as peer
 *
 * @param groundStationMAC MAC address of the ground station (6 bytes)
 * @param callback Function to call when command is received
 * @return true if initialization successful, false otherwise
 */
bool initESPNow(const uint8_t *groundStationMAC, CommandCallback callback);

/**
 * Send a telemetry reading to the ground station
 *
 * @param reading SensorReading struct to send
 * @return true if send initiated successfully (does not guarantee delivery)
 */
bool sendTelemetry(const SensorReading &reading);

/**
 * Get ESP-NOW statistics
 */
uint32_t getTelemetrySentCount();
uint32_t getTelemetryFailCount();
uint32_t getCommandsReceivedCount();

#endif // ESPNOW_COMMS_H
