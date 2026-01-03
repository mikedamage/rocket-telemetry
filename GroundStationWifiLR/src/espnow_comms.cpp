/**
 * ESP-NOW Communication Module - Ground Station Implementation
 */

#include "espnow_comms.h"

// Static variables for callbacks and statistics
static TelemetryCallback telemetryCallback = nullptr;
static uint8_t rocketAddress[6];
static volatile uint32_t telemetryReceivedCount = 0;
static volatile uint32_t commandsSentCount = 0;

/**
 * ESP-NOW receive callback
 * Called when data is received from rocket
 */
static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    // Validate packet size
    if (len == sizeof(SensorReading) && telemetryCallback != nullptr) {
        const SensorReading* reading = reinterpret_cast<const SensorReading*>(data);
        int8_t rssi = info->rx_ctrl->rssi;

        // Use atomic increment instead of deprecated volatile++
        uint32_t count = telemetryReceivedCount;
        telemetryReceivedCount = count + 1;

        telemetryCallback(*reading, rssi);
    }
}

/**
 * ESP-NOW send callback (ESP32-S3 signature with wifi_tx_info_t)
 * Called after command is sent (success or failure)
 */
static void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
    // Silent operation - don't log every send
    // Could add error counter here if needed
}

bool initESPNow(const uint8_t* rocketMAC, TelemetryCallback callback) {
    if (callback == nullptr) {
        Serial.println("LOG: ERROR - Telemetry callback is null");
        return false;
    }

    telemetryCallback = callback;
    memcpy(rocketAddress, rocketMAC, 6);

    // Initialize WiFi in STA mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Set WiFi channel
    esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (channelResult != ESP_OK) {
        Serial.printf("LOG: ERROR - Failed to set WiFi channel %d (error: %d)\n", ESPNOW_CHANNEL, channelResult);
        return false;
    }

    Serial.printf("LOG: WiFi initialized in STA mode, channel %d\n", ESPNOW_CHANNEL);
    Serial.printf("LOG: Ground station MAC: %s\n", WiFi.macAddress().c_str());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("LOG: ERROR - ESP-NOW init failed");
        return false;
    }

    Serial.println("LOG: ESP-NOW initialized");

    // Register callbacks
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // Add rocket as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, rocketAddress, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("LOG: ERROR - Failed to add rocket as peer");
        return false;
    }

    Serial.printf("LOG: Rocket peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  rocketAddress[0], rocketAddress[1], rocketAddress[2],
                  rocketAddress[3], rocketAddress[4], rocketAddress[5]);

    return true;
}

bool sendCommand(CommandCode cmd) {
    uint8_t cmdByte = static_cast<uint8_t>(cmd);

    esp_err_t result = esp_now_send(rocketAddress, &cmdByte, sizeof(cmdByte));

    if (result == ESP_OK) {
        // Use atomic increment instead of deprecated volatile++
        uint32_t count = commandsSentCount;
        commandsSentCount = count + 1;

        // Log command sent
        const char* cmdName = "UNKNOWN";
        switch (cmd) {
            case CommandCode::START: cmdName = "START"; break;
            case CommandCode::STOP: cmdName = "STOP"; break;
            case CommandCode::RECALIBRATE: cmdName = "RECALIBRATE"; break;
        }
        Serial.printf("LOG: Command sent: %s\n", cmdName);

        return true;
    } else {
        Serial.printf("LOG: ERROR - Command send failed: %d\n", result);
        return false;
    }
}

uint32_t getTelemetryReceivedCount() {
    return telemetryReceivedCount;
}

uint32_t getCommandsSentCount() {
    return commandsSentCount;
}
