/**
 * Rocket Telemetry System - ESP-NOW Version
 *
 * Collects sensor data from BME280 and transmits via ESP-NOW to ground station.
 * Logs data locally to LittleFS as backup.
 * Activates BLE beacon after timeout for rocket recovery.
 *
 * Threading model:
 * - Core 1: Main loop (sensor reads, ESP-NOW transmission, command handling)
 * - Core 0: CSV logging task (flash writes isolated from time-critical
 * operations)
 */

#include "../../shared/espnow_protocol.h"
#include "espnow_comms.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <LittleFS.h>
#include <NimBLEBeacon.h>
#include <NimBLEDevice.h>
#include <TinyPICO.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// Compile-time debug serial output control
// Set to 0 to disable debug serial output during flight for maximum performance
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 1
#endif

// Ensure MAC address is defined at build time
#ifndef GROUND_STATION_MAC
#error "GROUND_STATION_MAC must be set as an environment variable"
#endif

// Parse ground station MAC address from build flags
const uint8_t groundStationMacAddress[6] = {GROUND_STATION_MAC};

// Configuration constants with sensible defaults
#ifndef READING_INTERVAL
#define READING_INTERVAL 20
#endif

#ifndef TELEMETRY_TIMEOUT
#define TELEMETRY_TIMEOUT 120000
#endif

#ifndef GROUND_REFERENCE_PRESSURE
#define GROUND_REFERENCE_PRESSURE 1013.25f
#endif

#ifndef BEACON_UUID
#define BEACON_UUID "79daf75a-182c-4cc9-ad65-640ad1fd7b3b"
#endif

#define BEACON_MAJOR 1
#define BEACON_MINOR 1

// Hardware objects (singletons representing physical hardware)
Adafruit_BME280 bme;
TinyPICO tp = TinyPICO();
NimBLEBeacon beacon;
NimBLEAdvertising *beaconAdvertising = nullptr;

/**
 * Runtime state for the rocket telemetry system
 * Groups all mutable state in one place for clarity
 */
struct RocketState {
  // Configuration (may be modified at runtime)
  const char *csvFileName =
      "/telemetry.csv"; // Single file design - use TRUNCATE command before each
                        // flight
  unsigned long readingInterval = READING_INTERVAL;
  uint32_t telemetryTimeout = TELEMETRY_TIMEOUT;
  float groundReferencePressure = GROUND_REFERENCE_PRESSURE;
  uint32_t debugOutputInterval = 25; // Print debug info every N readings

  // File system status
  bool fileSystemReady = false;
  bool csvLoggingEnabled = true;

  // Hardware status
  bool sensorReady = false;

  // Telemetry control flags
  volatile bool transmissionEnabled = false;
  bool timeoutElapsed = false;

  // BLE beacon status
  bool beaconActive = false;

  // Command flags (set by ESP-NOW callbacks)
  bool forceActivateBeacon = false;
  bool forceRecalibrateAltimeter = false;
  bool downloadRequested = false;
  bool truncateRequested = false;

  // Timing state
  unsigned long lastReading = 0;
  unsigned long currentTime = 0;
  unsigned long telemetryStartTime = 0;
};

static RocketState state;

// Configuration constants for storage and logging
static const size_t CSV_BYTES_PER_READING = 50; // Estimated bytes per CSV line
static const size_t FLASH_SAFETY_MARGIN =
    1024; // Keep 1KB free as safety buffer
static const uint32_t STORAGE_CHECK_INTERVAL =
    100; // Check storage every N writes

// CSV logging queue configuration
static const size_t CSV_LOG_QUEUE_SIZE = 200; // Buffer ~4 seconds at 50Hz
static const size_t CSV_WRITE_BATCH_SIZE =
    20; // Write this many readings per file open/close
static const size_t CSV_LINE_SIZE = 64; // Max size of one CSV line

// FreeRTOS handles for CSV logging task
static QueueHandle_t csvLogQueue = nullptr;
static TaskHandle_t csvLogTaskHandle = nullptr;
static SemaphoreHandle_t filesystemMutex = nullptr;

/**
 * CSV logging task - runs on Core 0
 * Dequeues sensor readings and writes to flash in batches, isolated from
 * time-critical operations on Core 1. Batching reduces file open/close
 * overhead which is the main bottleneck with LittleFS.
 */
static void csvLoggerTask(void *param) {
  SensorReading readings[CSV_WRITE_BATCH_SIZE];
  char csvBuffer[CSV_WRITE_BATCH_SIZE * CSV_LINE_SIZE];
  uint32_t writeCount = 0;
  size_t batchIndex = 0;

  while (true) {
    // Wait for a reading with timeout to ensure periodic flush
    SensorReading reading;
    BaseType_t received =
        xQueueReceive(csvLogQueue, &reading, pdMS_TO_TICKS(100));

    if (received == pdTRUE) {
      // Check if logging is still enabled
      if (!state.csvLoggingEnabled || !state.fileSystemReady) {
        batchIndex = 0; // Clear batch if logging disabled
        continue;
      }

      // Add to batch
      readings[batchIndex++] = reading;
    }

    // Write batch when full OR on timeout with pending data
    bool shouldWrite = (batchIndex >= CSV_WRITE_BATCH_SIZE) ||
                       (received != pdTRUE && batchIndex > 0);

    if (shouldWrite && state.csvLoggingEnabled && state.fileSystemReady) {
      // Check storage space periodically
      writeCount += batchIndex;
      if (writeCount >= STORAGE_CHECK_INTERVAL) {
        writeCount = 0;
        size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
        if (freeBytes < (CSV_BYTES_PER_READING * CSV_WRITE_BATCH_SIZE) +
                            FLASH_SAFETY_MARGIN) {
          Serial.println(F("Flash storage full - stopping CSV logging"));
          state.csvLoggingEnabled = false;
          batchIndex = 0;
          continue;
        }
      }

      // Build batch buffer
      size_t bufferPos = 0;
      for (size_t i = 0; i < batchIndex; i++) {
        int written =
            snprintf(csvBuffer + bufferPos, sizeof(csvBuffer) - bufferPos,
                     "%lu,%.2f,%.2f,%.4f,%.2f\n", readings[i].timestamp,
                     readings[i].temperature, readings[i].pressure,
                     readings[i].altitude, readings[i].humidity);
        if (written > 0) {
          bufferPos += written;
        }
      }

      // Write entire batch in one file operation
      if (xSemaphoreTake(filesystemMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        File csvFile = LittleFS.open(state.csvFileName, "a");
        if (csvFile) {
          csvFile.write((const uint8_t *)csvBuffer, bufferPos);
          csvFile.close();
        }
        xSemaphoreGive(filesystemMutex);
      }

      batchIndex = 0;
    }
  }
}

// Forward declarations
void calibrateAltimeter();
void takeSensorReading();
void checkTelemetryTimeout();
void initializeBLE();
void activateBLEBeacon();
bool createCsvLog();
size_t littlefsFreeSpace();
void handleFileDownload();
void handleTruncateLog();

/**
 * ESP-NOW command callback
 * Called when command is received from ground station
 */
void onCommandReceived(CommandCode cmd) {
  switch (cmd) {
  case CommandCode::START:
    state.transmissionEnabled = true;
    state.telemetryStartTime = millis();
    state.timeoutElapsed = false; // Reset timeout flag for subsequent starts
    Serial.println(F("START command received - telemetry enabled"));
    break;

  case CommandCode::STOP:
    state.transmissionEnabled = false;
    Serial.println(F("STOP command received - telemetry disabled"));
    break;

  case CommandCode::RECALIBRATE:
    state.forceRecalibrateAltimeter = true;
    Serial.println(F("RECALIBRATE command received"));
    break;

  case CommandCode::DOWNLOAD:
    state.downloadRequested = true;
    Serial.println(F("DOWNLOAD command received - will send flight log"));
    break;

  case CommandCode::TRUNCATE:
    state.truncateRequested = true;
    Serial.println(F("TRUNCATE command received - will clear flight log"));
    break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // Brief delay for serial to initialize, but don't block

  Serial.println(F("TinyPICO Rocket Telemetry System (ESP-NOW) Starting..."));

  // Create FreeRTOS primitives for CSV logging
  csvLogQueue = xQueueCreate(CSV_LOG_QUEUE_SIZE, sizeof(SensorReading));
  filesystemMutex = xSemaphoreCreateMutex();

  if (csvLogQueue == nullptr || filesystemMutex == nullptr) {
    Serial.println(F("ERROR: Failed to create FreeRTOS primitives"));
    while (1) {
      delay(1000);
    }
  }

  // Initialize LittleFS
  if (LittleFS.begin(true)) {
    state.fileSystemReady = true;
    Serial.println(F("LittleFS mounted successfully"));

    // Create CSV header if file doesn't exist
    createCsvLog();

    // Check available space
    size_t totalBytes = LittleFS.totalBytes();
    size_t usedBytes = LittleFS.usedBytes();
    size_t freeBytes = littlefsFreeSpace();
    Serial.printf("Flash storage: %d/%d bytes used\n", usedBytes, totalBytes);
    Serial.printf("Free flash storage: %d bytes\n", freeBytes);
  } else {
    Serial.println(F("LittleFS mount failed"));
    state.fileSystemReady = false;
  }

  // Initialize I2C (TinyPICO uses pins 22 and 21 by default)
  Wire.begin();

  // Initialize BME280
  if (bme.begin(0x76)) {
    // Configure BME280 for high-speed readings
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,     // Operating Mode
                    Adafruit_BME280::SAMPLING_X2,     // Temp. oversampling
                    Adafruit_BME280::SAMPLING_X8,     // Pressure oversampling
                    Adafruit_BME280::SAMPLING_X1,     // Humidity oversampling
                    Adafruit_BME280::FILTER_X8,       // Filtering
                    Adafruit_BME280::STANDBY_MS_0_5); // Standby time

    state.sensorReady = true;

    // Take several pressure readings to establish a ground reference
    calibrateAltimeter();
  } else {
    Serial.println(F("Could not find BME280 sensor!"));
    Serial.println(
        F("Proceeding with ESP-NOW setup, but no readings will be gathered"));
  }

  Serial.print(F("Battery voltage: "));
  Serial.println(tp.GetBatteryVoltage());
  Serial.print(F("Is charging? "));
  Serial.println(tp.IsChargingBattery());

  // Initialize ESP-NOW
  // Error handling strategy: Critical errors (ESP-NOW init) halt the system
  // for safety. Non-critical errors (sensor reads, file writes) are logged
  // and the system continues with degraded functionality.
  if (!initESPNow(groundStationMacAddress, onCommandReceived)) {
    Serial.println(F("FATAL: ESP-NOW initialization failed"));
    while (1) {
      delay(1000); // Halt - requires physical reset
    }
  }

  // Initialize BLE (but don't start beacon yet)
  initializeBLE();

  // Start CSV logging task on Core 0 (background, lower priority)
  BaseType_t taskResult =
      xTaskCreatePinnedToCore(csvLoggerTask, "CSVLogger", 8192, nullptr, 1,
                              &csvLogTaskHandle, 0 // Core 0
      );

  if (taskResult != pdPASS) {
    Serial.println(F("ERROR: Failed to create CSV logger task"));
    // Continue anyway - CSV logging will be disabled but telemetry will work
  } else {
    Serial.println(F("CSV logger task started on Core 0"));
  }

  Serial.println(F("System ready"));
  Serial.println(F("Waiting for START command from ground station..."));
}

void loop() {
  state.currentTime = millis();
  checkTelemetryTimeout();

  if (state.sensorReady && state.forceRecalibrateAltimeter) {
    calibrateAltimeter();
  }

  if (state.downloadRequested) {
    handleFileDownload();
    state.downloadRequested = false;
  }

  if (state.truncateRequested) {
    handleTruncateLog();
    state.truncateRequested = false;
  }

  if (!state.timeoutElapsed && state.transmissionEnabled) {
    // Normal flight operations

    // Take sensor readings at configured interval
    if (state.currentTime - state.lastReading >= state.readingInterval) {
      takeSensorReading();
    }
  }

  if ((state.timeoutElapsed || state.forceActivateBeacon) &&
      !state.beaconActive) {
    activateBLEBeacon();
  }

  // Yield to FreeRTOS scheduler to allow WiFi task (ESP-NOW) to run.
  // When idle: longer delay saves power and ensures reliable command reception.
  // During transmission: I2C sensor reads provide natural yielding, so no
  // additional delay needed to maintain timing accuracy.
  if (!state.transmissionEnabled) {
    delay(10);
  }
}

size_t littlefsFreeSpace() {
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  return totalBytes - usedBytes;
}

bool createCsvLog() {
  if (LittleFS.exists(state.csvFileName)) {
    Serial.println(F("CSV log already exists, not creating new one"));
    return false;
  }
  File file = LittleFS.open(state.csvFileName, "w");
  if (file) {
    file.println(F("timestamp,temperature,pressure,altitude,humidity"));
    file.close();
    return true;
  }
  return false;
}

void calibrateAltimeter() {
  Serial.println(F("Calibrating altimeter to ground reference air pressure."));
  Serial.printf(F("Averaging 20 readings over 10.0 seconds\n"));

  const uint8_t BASELINE_PRESSURE_READINGS = 20;
  float baselinePressureReadings[BASELINE_PRESSURE_READINGS];
  uint8_t baselineReadingIndex = 0;

  while (baselineReadingIndex < BASELINE_PRESSURE_READINGS) {
    baselinePressureReadings[baselineReadingIndex] = bme.readPressure() / 100.0;
    baselineReadingIndex++;
    Serial.print(".");
    delay(500);
  }

  float baselinePressureSum = 0.0;

  for (uint8_t i = 0; i < BASELINE_PRESSURE_READINGS; i++) {
    baselinePressureSum += baselinePressureReadings[i];
  }
  state.groundReferencePressure =
      baselinePressureSum / (float)BASELINE_PRESSURE_READINGS;

  Serial.printf(F("\nGround reference pressure set to %.4f hPa\n"),
                state.groundReferencePressure);
  Serial.println(F("BME280 initialized successfully"));
  state.forceRecalibrateAltimeter = false;
}

void takeSensorReading() {
  if (!state.sensorReady || state.timeoutElapsed)
    return;

  // Read sensor data
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
  float altitude = bme.readAltitude(state.groundReferencePressure); // meters
  float humidity = bme.readHumidity();

  state.lastReading = state.currentTime;

  // Create SensorReading struct
  SensorReading reading;
  reading.timestamp = state.lastReading;
  reading.temperature = temp;
  reading.pressure = pressure;
  // Clamp altitude to 0 for negative values (pressure fluctuations at ground
  // level)
  if (altitude < 0) {
#if DEBUG_SERIAL
    // Log clamping to help detect sensor calibration issues
    static uint32_t clampCount = 0;
    if (++clampCount % 50 == 1) { // Log every 50 clamps to avoid spam
      Serial.printf("Altitude clamped to 0 (was %.2fm) - check calibration\n",
                    altitude);
    }
#endif
    reading.altitude = 0;
  } else {
    reading.altitude = altitude;
  }
  reading.humidity = humidity;

  // Send via ESP-NOW immediately (time-critical)
  if (!sendTelemetry(reading)) {
#if DEBUG_SERIAL
    // Log failures periodically
    if (getTelemetryFailCount() % 10 == 1) {
      Serial.printf("ESP-NOW send failed (total failures: %lu)\n",
                    getTelemetryFailCount());
    }
#endif
  }

  // Enqueue for CSV logging on Core 0 (non-blocking)
  if (state.csvLoggingEnabled && csvLogQueue != nullptr) {
    // Don't block if queue is full - telemetry transmission takes priority
    if (xQueueSend(csvLogQueue, &reading, 0) != pdTRUE) {
#if DEBUG_SERIAL
      static uint32_t queueFullCount = 0;
      if (++queueFullCount % 100 == 1) {
        Serial.println(F("CSV log queue full - some readings may be lost"));
      }
#endif
    }
  }

#if DEBUG_SERIAL
  // Debug output at configured interval
  static uint32_t readingCount = 0;
  readingCount++;
  if (readingCount % state.debugOutputInterval == 0) {
    Serial.printf(F("Reading #%lu: T=%.2f°C, P=%.2fhPa, A=%.4fm, RH=%.2f%% "
                    "(sent: %lu, failed: %lu)\n"),
                  readingCount, temp, pressure, altitude, humidity,
                  getTelemetrySentCount(), getTelemetryFailCount());
  }
#endif
}

void checkTelemetryTimeout() {
  if (state.transmissionEnabled &&
      state.currentTime - state.telemetryStartTime >= state.telemetryTimeout) {
    Serial.println(F("Telemetry timeout elapsed. Stopping transmission."));
    state.timeoutElapsed = true;
    state.transmissionEnabled = false;
  }
}

void initializeBLE() {
  Serial.println(F("Initializing BLE..."));
  NimBLEDevice::init("RocketBeacon");
  beacon.setMajor(BEACON_MAJOR);
  beacon.setMinor(BEACON_MINOR);
  beacon.setSignalPower(0xD3);
  beacon.setProximityUUID(NimBLEUUID(BEACON_UUID));

  NimBLEAddress macAddress = NimBLEDevice::getAddress();

  NimBLEAdvertisementData beaconAdvertisementData;
  beaconAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED
  beaconAdvertisementData.setName("RocketBeacon");
  beaconAdvertisementData.addTxPower();
  beaconAdvertisementData.setManufacturerData(beacon.getData());

  beaconAdvertising = NimBLEDevice::getAdvertising();
  beaconAdvertising->setAdvertisingInterval(160); // 100ms
  beaconAdvertising->setAdvertisementData(beaconAdvertisementData);
  Serial.printf(F("BLE beacon initialized. MAC Address: %s\n"),
                macAddress.toString().c_str());
}

void activateBLEBeacon() {
  Serial.println(F("Activating BLE beacon for rocket recovery..."));
  beaconAdvertising->start();
  state.beaconActive = true;
  Serial.println(F("BLE beacon active - rocket can now be located!"));
}

void handleFileDownload() {
  // File download protocol: Sends chunks with 5ms delay, no ACKs/retries.
  // This is acceptable for post-flight data recovery where reliability is less
  // critical than simplicity. Chunks may be lost if ground station buffer
  // fills.
  if (!state.fileSystemReady) {
    Serial.println(F("Cannot download: file system not ready"));
    return;
  }

  // Take filesystem mutex to prevent concurrent access with CSV logger
  if (xSemaphoreTake(filesystemMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println(F("Cannot download: filesystem busy"));
    return;
  }

  if (!LittleFS.exists(state.csvFileName)) {
    Serial.println(F("Cannot download: flight log does not exist"));
    xSemaphoreGive(filesystemMutex);
    return;
  }

  File csvFile = LittleFS.open(state.csvFileName, "r");
  if (!csvFile) {
    Serial.println(F("Cannot download: failed to open flight log"));
    xSemaphoreGive(filesystemMutex);
    return;
  }

  size_t fileSize = csvFile.size();
  Serial.printf("Starting flight log download (%d bytes)\n", fileSize);

  uint16_t sequenceNumber = 0;
  uint8_t buffer[245];
  size_t totalBytesSent = 0;

  while (csvFile.available()) {
    size_t bytesRead = csvFile.read(buffer, sizeof(buffer));

    FileChunk chunk;
    chunk.packetType = static_cast<uint8_t>(PacketType::FILE_CHUNK);
    chunk.sequenceNumber = sequenceNumber;
    chunk.dataLength = bytesRead;
    memcpy(chunk.data, buffer, bytesRead);

    // Check if this is the last chunk
    if (!csvFile.available()) {
      chunk.flags = FILE_CHUNK_FLAG_LAST;
    } else {
      chunk.flags = 0;
    }

    // Send the chunk
    if (!sendFileChunk(chunk)) {
      Serial.printf("Failed to send chunk %d\n", sequenceNumber);
      break;
    }

    totalBytesSent += bytesRead;
    sequenceNumber++;

    // Small delay to avoid overwhelming the ESP-NOW buffer
    delay(5);
  }

  csvFile.close();
  xSemaphoreGive(filesystemMutex);
  Serial.printf("Download complete: %d chunks, %d bytes sent\n", sequenceNumber,
                totalBytesSent);
}

void handleTruncateLog() {
  if (!state.fileSystemReady) {
    Serial.println(F("Cannot truncate: file system not ready"));
    return;
  }

  // Take filesystem mutex to prevent concurrent access with CSV logger
  if (xSemaphoreTake(filesystemMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    Serial.println(F("Cannot truncate: filesystem busy"));
    return;
  }

  // Delete the existing file
  if (LittleFS.exists(state.csvFileName)) {
    LittleFS.remove(state.csvFileName);
    Serial.println(F("Flight log deleted"));
  }

  // Recreate the file with header
  if (createCsvLog()) {
    Serial.println(F("Flight log recreated with CSV header"));
  } else {
    Serial.println(F("Failed to recreate flight log"));
    xSemaphoreGive(filesystemMutex);
    return;
  }

  xSemaphoreGive(filesystemMutex);

  // Re-enable CSV logging in case it was disabled
  state.csvLoggingEnabled = true;

  // Report storage status
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  size_t freeBytes = littlefsFreeSpace();
  Serial.printf("Storage after truncate: %d/%d bytes used, %d bytes free\n",
                usedBytes, totalBytes, freeBytes);
}
