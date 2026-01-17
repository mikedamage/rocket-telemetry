/**
 * ESP-NOW Communication Module - Ground Station Implementation
 *
 * Uses FreeRTOS queue to decouple ESP-NOW reception (ISR context) from
 * UART forwarding, preventing blocking Serial.printf() calls from
 * interfering with packet reception.
 */

#include "espnow_comms.h"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Queue configuration
static const size_t TELEMETRY_QUEUE_SIZE = 20; // Buffer ~400ms at 50Hz
static const size_t FILE_CHUNK_QUEUE_SIZE = 10;

// Queued telemetry packet with RSSI
struct QueuedTelemetry {
  SensorReading reading;
  int8_t rssi;
};

// Static variables for callbacks and statistics
static TelemetryCallback telemetryCallback = nullptr;
static FileChunkCallback fileChunkCallback = nullptr;
static uint8_t rocketAddress[6];
static std::atomic<uint32_t> telemetryReceivedCount{0};
static std::atomic<uint32_t> commandsSentCount{0};
static std::atomic<uint32_t> commandsSendFailCount{0};
static std::atomic<uint32_t> queueOverflowCount{0};

// FreeRTOS handles
static QueueHandle_t telemetryQueue = nullptr;
static QueueHandle_t fileChunkQueue = nullptr;
static TaskHandle_t forwardingTaskHandle = nullptr;

/**
 * Telemetry forwarding task
 * Runs on Core 1, dequeues telemetry and forwards via UART
 */
static void telemetryForwardingTask(void *param) {
  QueuedTelemetry item;
  FileChunk chunk;

  while (true) {
    // Check telemetry queue (higher priority)
    if (xQueueReceive(telemetryQueue, &item, 0) == pdTRUE) {
      if (telemetryCallback != nullptr) {
        telemetryCallback(item.reading, item.rssi);
      }
    }
    // Check file chunk queue
    else if (xQueueReceive(fileChunkQueue, &chunk, 0) == pdTRUE) {
      if (fileChunkCallback != nullptr) {
        fileChunkCallback(chunk);
      }
    }
    // No data - yield briefly
    else {
      vTaskDelay(1);
    }
  }
}

/**
 * ESP-NOW receive callback
 * Called in WiFi task context - must return quickly!
 * Enqueues data for processing by forwarding task.
 */
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data,
                       int len) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Check packet size to determine type
  if (len == sizeof(SensorReading) && telemetryQueue != nullptr) {
    // Regular telemetry packet - enqueue for forwarding
    QueuedTelemetry item;
    memcpy(&item.reading, data, sizeof(SensorReading));
    item.rssi = info->rx_ctrl->rssi;

    telemetryReceivedCount.fetch_add(1, std::memory_order_relaxed);

    if (xQueueSendFromISR(telemetryQueue, &item, &xHigherPriorityTaskWoken) !=
        pdTRUE) {
      queueOverflowCount.fetch_add(1, std::memory_order_relaxed);
    }
  } else if (len == sizeof(FileChunk) && fileChunkQueue != nullptr) {
    // File chunk packet
    const FileChunk *chunk = reinterpret_cast<const FileChunk *>(data);

    // Verify it's actually a file chunk packet
    if (chunk->packetType == static_cast<uint8_t>(PacketType::FILE_CHUNK)) {
      if (xQueueSendFromISR(fileChunkQueue, chunk, &xHigherPriorityTaskWoken) !=
          pdTRUE) {
        queueOverflowCount.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  // Yield if a higher priority task was woken
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * ESP-NOW send callback (ESP32-S3 signature with wifi_tx_info_t)
 * Called after command is sent (success or failure)
 */
static void onDataSent(const wifi_tx_info_t *info,
                       esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    commandsSendFailCount.fetch_add(1, std::memory_order_relaxed);
  }
}

bool initESPNow(const uint8_t *rocketMAC, TelemetryCallback telCallback,
                FileChunkCallback fileCallback) {
  if (telCallback == nullptr) {
    Serial.println("LOG: ERROR - Telemetry callback is null");
    return false;
  }

  telemetryCallback = telCallback;
  fileChunkCallback = fileCallback;
  memcpy(rocketAddress, rocketMAC, 6);

  // Create FreeRTOS queues for decoupling reception from UART forwarding
  telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(QueuedTelemetry));
  fileChunkQueue = xQueueCreate(FILE_CHUNK_QUEUE_SIZE, sizeof(FileChunk));

  if (telemetryQueue == nullptr || fileChunkQueue == nullptr) {
    Serial.println("LOG: ERROR - Failed to create queues");
    return false;
  }

  // Create forwarding task on Core 1 (same as Arduino loop)
  // Priority 2 is higher than Arduino loop (priority 1) for responsive
  // forwarding
  BaseType_t taskResult =
      xTaskCreatePinnedToCore(telemetryForwardingTask, "TelemetryFwd", 4096,
                              nullptr, 2, &forwardingTaskHandle, 1 // Core 1
      );

  if (taskResult != pdPASS) {
    Serial.println("LOG: ERROR - Failed to create forwarding task");
    return false;
  }

  Serial.println("LOG: Telemetry forwarding task started on Core 1");

  // Initialize WiFi in STA mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Set link protocol to 802.11LR
  WiFi.enableLongRange(true);

  // Set WiFi channel
  WiFi.setChannel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  /*
  esp_err_t channelResult =
      esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelResult != ESP_OK) {
    Serial.printf("LOG: ERROR - Failed to set WiFi channel %d (error: %d)\n",
                  ESPNOW_CHANNEL, channelResult);
    return false;
  }
  */

  Serial.printf("LOG: WiFi initialized in STA mode, channel %d\n",
                ESPNOW_CHANNEL);
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
    commandsSentCount.fetch_add(1, std::memory_order_relaxed);

    // Log command sent
    const char *cmdName = "UNKNOWN";
    switch (cmd) {
    case CommandCode::START:
      cmdName = "START";
      break;
    case CommandCode::STOP:
      cmdName = "STOP";
      break;
    case CommandCode::RECALIBRATE:
      cmdName = "RECALIBRATE";
      break;
    case CommandCode::DOWNLOAD:
      cmdName = "DOWNLOAD";
      break;
    case CommandCode::TRUNCATE:
      cmdName = "TRUNCATE";
      break;
    }
    Serial.printf("LOG: Command sent: %s\n", cmdName);

    return true;
  } else {
    Serial.printf("LOG: ERROR - Command send failed: %d\n", result);
    return false;
  }
}

uint32_t getTelemetryReceivedCount() {
  return telemetryReceivedCount.load(std::memory_order_relaxed);
}

uint32_t getCommandsSentCount() {
  return commandsSentCount.load(std::memory_order_relaxed);
}

uint32_t getCommandsSendFailCount() {
  return commandsSendFailCount.load(std::memory_order_relaxed);
}

uint32_t getQueueOverflowCount() {
  return queueOverflowCount.load(std::memory_order_relaxed);
}
