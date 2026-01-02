/**
 * ESP-NOW Protocol Definitions
 *
 * Shared between rocket and ground station to ensure consistent binary communication.
 * This header must be kept synchronized across both projects.
 */

#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <Arduino.h>

/**
 * Sensor reading data packet (12 bytes)
 *
 * Transmitted from rocket to ground station via ESP-NOW.
 * Uses fixed-point integers to preserve precision while minimizing payload size.
 */
struct __attribute__((packed)) SensorReading {
    uint32_t timestamp;     // Milliseconds since boot
    int16_t temperature;    // Temperature * 100 (e.g., 2550 = 25.50°C)
    uint16_t pressure;      // Pressure in hPa (e.g., 1013 = 1013 hPa)
    uint16_t altitude;      // Altitude in cm (e.g., 12050 = 120.50m)
    uint16_t humidity;      // Humidity * 100 (e.g., 6500 = 65.00%)
};

// Compile-time verification that struct is exactly 12 bytes
static_assert(sizeof(SensorReading) == 12, "SensorReading must be exactly 12 bytes");

/**
 * Command codes (1 byte each)
 *
 * Transmitted from ground station to rocket via ESP-NOW.
 */
enum class CommandCode : uint8_t {
    START = 0x01,           // Start telemetry transmission
    STOP = 0x02,            // Stop telemetry transmission
    RECALIBRATE = 0x03      // Recalibrate altimeter baseline
};

/**
 * Helper function to validate command codes
 */
inline bool isValidCommand(uint8_t cmd) {
    return (cmd == static_cast<uint8_t>(CommandCode::START) ||
            cmd == static_cast<uint8_t>(CommandCode::STOP) ||
            cmd == static_cast<uint8_t>(CommandCode::RECALIBRATE));
}

/**
 * ESP-NOW configuration constants
 */
#define ESPNOW_CHANNEL 1        // WiFi channel for ESP-NOW communication
#define ESPNOW_MAX_RETRIES 0    // No retries (fire-and-forget for speed)

#endif // ESPNOW_PROTOCOL_H
