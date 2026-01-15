/**
 * ESP-NOW Communication Module - Rocket Implementation
 */

#include "espnow_comms.h"
#include <atomic>

// Static variables for callbacks and statistics
static CommandCallback commandCallback = nullptr;
static uint8_t groundStationAddress[6];
static std::atomic<uint32_t> telemetrySentCount{0};
static std::atomic<uint32_t> telemetryFailCount{0};
static std::atomic<uint32_t> commandsReceivedCount{0};

/**
 * ESP-NOW receive callback
 * Called when command is received from ground station
 */
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data,
                       int len) {
  // Validate packet size (command is 1 byte)
  if (len == sizeof(uint8_t) && commandCallback != nullptr) {
    uint8_t cmdByte = data[0];

    // Validate command code
    if (isValidCommand(cmdByte)) {
      CommandCode cmd = static_cast<CommandCode>(cmdByte);

      commandsReceivedCount.fetch_add(1, std::memory_order_relaxed);

      commandCallback(cmd);
    } else {
      Serial.printf("Received invalid command code: 0x%02X\n", cmdByte);
    }
  }
}

/**
 * ESP-NOW send callback (ESP32 signature with wifi_tx_info_t)
 * Called after telemetry is sent (success or failure)
 */
static void onDataSent(const wifi_tx_info_t *info,
                       esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    uint32_t failCount =
        telemetryFailCount.fetch_add(1, std::memory_order_relaxed) + 1;

    // Only log failures periodically to avoid flooding serial
    if (failCount % 10 == 1) {
      Serial.printf("ESP-NOW send failures: %lu\n", failCount);
    }
  }
}

bool initESPNow(const uint8_t *groundStationMAC, CommandCallback callback) {
  if (callback == nullptr) {
    Serial.println("ERROR: Command callback is null");
    return false;
  }

  commandCallback = callback;
  memcpy(groundStationAddress, groundStationMAC, 6);

  // Initialize WiFi in STA mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Set link protocol to 802.11LR
  WiFi.enableLongRange(true);

  // Set WiFi channel
  WiFi.setChannel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // esp_err_t channelResult =
  //     esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // if (channelResult != ESP_OK) {
  //   Serial.printf("ERROR: Failed to set WiFi channel %d (error: %d)\n",
  //                 ESPNOW_CHANNEL, channelResult);
  //   return false;
  // }

  Serial.printf("WiFi initialized in STA mode, channel %d\n", ESPNOW_CHANNEL);
  Serial.printf("Rocket MAC: %s\n", WiFi.macAddress().c_str());
  WiFi.printDiag(Serial);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return false;
  }

  Serial.println("ESP-NOW initialized");

  // Register callbacks
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add ground station as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, groundStationAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add ground station as peer");
    return false;
  }

  Serial.printf("Ground station peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                groundStationAddress[0], groundStationAddress[1],
                groundStationAddress[2], groundStationAddress[3],
                groundStationAddress[4], groundStationAddress[5]);

  return true;
}

bool sendTelemetry(const SensorReading &reading) {
  esp_err_t result = esp_now_send(
      groundStationAddress, (const uint8_t *)&reading, sizeof(SensorReading));

  if (result == ESP_OK) {
    telemetrySentCount.fetch_add(1, std::memory_order_relaxed);
    return true;
  } else {
    // Don't increment telemetryFailCount here - it's counted in onDataSent
    // callback This prevents double-counting when send is initiated but
    // delivery fails
    Serial.printf("ESP-NOW send initiation failed: %d\n", result);
    return false;
  }
}

bool sendFileChunk(const FileChunk &chunk) {
  esp_err_t result = esp_now_send(groundStationAddress, (const uint8_t *)&chunk,
                                  sizeof(FileChunk));

  if (result == ESP_OK) {
    return true;
  } else {
    Serial.printf("ESP-NOW file chunk send failed: %d\n", result);
    return false;
  }
}

uint32_t getTelemetrySentCount() {
  return telemetrySentCount.load(std::memory_order_relaxed);
}

uint32_t getTelemetryFailCount() {
  return telemetryFailCount.load(std::memory_order_relaxed);
}

uint32_t getCommandsReceivedCount() {
  return commandsReceivedCount.load(std::memory_order_relaxed);
}
