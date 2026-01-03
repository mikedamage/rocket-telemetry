/**
 * Rocket Telemetry System - ESP-NOW Version
 *
 * Collects sensor data from BME280 and transmits via ESP-NOW to ground station.
 * Logs data locally to LittleFS as backup.
 * Activates BLE beacon after timeout for rocket recovery.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <NimBLEBeacon.h>
#include <TinyPICO.h>
#include "espnow_comms.h"
#include "../../shared/espnow_protocol.h"

// Ensure MAC address is defined at build time
#ifndef GROUND_STATION_MAC
#error "GROUND_STATION_MAC must be set as an environment variable"
#endif

// Parse ground station MAC address from build flags
const uint8_t groundStationMacAddress[6] = { GROUND_STATION_MAC };

// Configuration constants with sensible defaults
const unsigned long readingInterval = 20;           // milliseconds between readings
const uint32_t telemetryTimeout = 120000;          // milliseconds before auto-stop (2 minutes)
float groundReferencePressure = 1013.25f;          // hPa (sea level standard)

// iBeacon UUID for rocket identification
#define BEACON_UUID "79daf75a-182c-4cc9-ad65-640ad1fd7b3b"
#define BEACON_MAJOR 1
#define BEACON_MINOR 1

// Objects
Adafruit_BME280 bme;
TinyPICO tp = TinyPICO();
NimBLEBeacon beacon;
NimBLEAdvertising *beaconAdvertising = nullptr;

// File system
const char *csvFileName = "/telemetry.csv";
bool fileSystemReady = false;
bool csvLoggingEnabled = true;

// Status flags
bool sensorReady = false;
volatile bool transmissionEnabled = false;
bool beaconActive = false;
bool timeoutElapsed = false;
bool forceActivateBeacon = false;
bool forceRecalibrateAltimeter = false;

// Timing
unsigned long lastReading = 0;
unsigned long currentTime = 0;
unsigned long telemetryStartTime = 0;

// Forward declarations
void calibrateAltimeter();
void takeSensorReading();
void checkTelemetryTimeout();
void initializeBLE();
void activateBLEBeacon();
bool createCsvLog();
size_t littlefsFreeSpace();
bool canWriteFlightLog();

/**
 * ESP-NOW command callback
 * Called when command is received from ground station
 */
void onCommandReceived(CommandCode cmd) {
  switch (cmd) {
    case CommandCode::START:
      transmissionEnabled = true;
      telemetryStartTime = millis();
      Serial.println(F("START command received - telemetry enabled"));
      break;

    case CommandCode::STOP:
      transmissionEnabled = false;
      Serial.println(F("STOP command received - telemetry disabled"));
      break;

    case CommandCode::RECALIBRATE:
      forceRecalibrateAltimeter = true;
      Serial.println(F("RECALIBRATE command received"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);  // Brief delay for serial to initialize, but don't block

  Serial.println(F("TinyPICO Rocket Telemetry System (ESP-NOW) Starting..."));

  // Initialize LittleFS
  if (LittleFS.begin(true)) {
    fileSystemReady = true;
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
    fileSystemReady = false;
  }

  // Initialize I2C (TinyPICO uses pins 22 and 21 by default)
  Wire.begin();

  // Initialize BME280
  if (bme.begin(0x76)) {
    // Configure BME280 for high-speed readings
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,      // Operating Mode
                    Adafruit_BME280::SAMPLING_X2,      // Temp. oversampling
                    Adafruit_BME280::SAMPLING_X8,      // Pressure oversampling
                    Adafruit_BME280::SAMPLING_X1,      // Humidity oversampling
                    Adafruit_BME280::FILTER_X8,        // Filtering
                    Adafruit_BME280::STANDBY_MS_0_5);  // Standby time

    sensorReady = true;

    // Take several pressure readings to establish a ground reference
    calibrateAltimeter();
  } else {
    Serial.println(F("Could not find BME280 sensor!"));
    Serial.println(F("Proceeding with ESP-NOW setup, but no readings will be gathered"));
  }

  Serial.print(F("Battery voltage: "));
  Serial.println(tp.GetBatteryVoltage());
  Serial.print(F("Is charging? "));
  Serial.println(tp.IsChargingBattery());

  // Initialize ESP-NOW
  if (!initESPNow(groundStationMacAddress, onCommandReceived)) {
    Serial.println(F("FATAL: ESP-NOW initialization failed"));
    while (1) {
      delay(1000);
    }
  }

  // Initialize BLE (but don't start beacon yet)
  initializeBLE();

  Serial.println(F("System ready"));
  Serial.println(F("Waiting for START command from ground station..."));
}

void loop() {
  currentTime = millis();
  checkTelemetryTimeout();

  if (sensorReady && forceRecalibrateAltimeter) {
    calibrateAltimeter();
  }

  if (!timeoutElapsed && transmissionEnabled) {
    // Normal flight operations

    // Take sensor readings at configured interval
    if (currentTime - lastReading >= readingInterval) {
      takeSensorReading();
    }
  }

  if ((timeoutElapsed || forceActivateBeacon) && !beaconActive) {
    activateBLEBeacon();
  }

  delay(1);  // Small delay to prevent watchdog issues
}

size_t littlefsFreeSpace() {
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  return totalBytes - usedBytes;
}

bool createCsvLog() {
  if (LittleFS.exists(csvFileName)) {
    Serial.println(F("CSV log already exists, not creating new one"));
    return false;
  }
  File file = LittleFS.open(csvFileName, "w");
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
  groundReferencePressure = baselinePressureSum / (float)BASELINE_PRESSURE_READINGS;

  Serial.printf(F("\nGround reference pressure set to %.4f hPa\n"), groundReferencePressure);
  Serial.println(F("BME280 initialized successfully"));
  forceRecalibrateAltimeter = false;
}

void takeSensorReading() {
  if (!sensorReady || timeoutElapsed)
    return;

  // Read sensor data
  float temp = bme.readTemperature() * 100.0F;
  float pressure = bme.readPressure() / 100.0F;                         // hPa
  float altitude = bme.readAltitude(groundReferencePressure) * 100.0F;  // relative to configured ground reference
  float humidity = bme.readHumidity() * 100.0F;

  lastReading = currentTime;

  // Create SensorReading struct
  SensorReading reading;
  reading.timestamp = lastReading;
  reading.temperature = (int16_t)temp;
  reading.pressure = (uint16_t)pressure;
  // Clamp altitude to 0 for negative values (pressure fluctuations at ground level)
  reading.altitude = (uint16_t)(altitude < 0 ? 0 : altitude);
  reading.humidity = (uint16_t)humidity;

  // Send via ESP-NOW immediately
  if (!sendTelemetry(reading)) {
    // Log failures periodically
    if (getTelemetryFailCount() % 10 == 1) {
      Serial.printf("ESP-NOW send failed (total failures: %lu)\n", getTelemetryFailCount());
    }
  }

  // Log to local CSV (independent of transmission)
  if (canWriteFlightLog()) {
    File csvFile = LittleFS.open(csvFileName, "a");
    if (csvFile) {
      char csvLine[100];
      snprintf(csvLine, sizeof(csvLine), "%lu,%d,%u,%u,%u\n",
               reading.timestamp,
               reading.temperature,
               reading.pressure,
               reading.altitude,
               reading.humidity);
      csvFile.print(csvLine);
      csvFile.close();
    }
  }

  // Debug output every 25 readings
  static uint32_t readingCount = 0;
  readingCount++;
  if (readingCount % 25 == 0) {
    Serial.printf(F("Reading #%lu: T=%.2f°C, P=%.0fhPa, A=%.2fm, RH=%.2f%% (sent: %lu, failed: %lu)\n"),
                  readingCount,
                  temp / 100.0,
                  pressure,
                  altitude / 100.0,
                  humidity / 100.0,
                  getTelemetrySentCount(),
                  getTelemetryFailCount());
  }
}

void checkTelemetryTimeout() {
  if (transmissionEnabled && currentTime - telemetryStartTime >= telemetryTimeout) {
    Serial.println(F("Telemetry timeout elapsed. Stopping transmission."));
    timeoutElapsed = true;
    transmissionEnabled = false;
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
  beaconAdvertisementData.setFlags(0x04);  // BR_EDR_NOT_SUPPORTED
  beaconAdvertisementData.setName("RocketBeacon");
  beaconAdvertisementData.addTxPower();
  beaconAdvertisementData.setManufacturerData(beacon.getData());

  beaconAdvertising = NimBLEDevice::getAdvertising();
  beaconAdvertising->setAdvertisingInterval(160);  // 100ms
  beaconAdvertising->setAdvertisementData(beaconAdvertisementData);
  Serial.printf(F("BLE beacon initialized. MAC Address: %s\n"), macAddress.toString().c_str());
}

void activateBLEBeacon() {
  Serial.println(F("Activating BLE beacon for rocket recovery..."));
  beaconAdvertising->start();
  beaconActive = true;
  Serial.println(F("BLE beacon active - rocket can now be located!"));
}

bool canWriteFlightLog() {
  if (!fileSystemReady || !csvLoggingEnabled) {
    return false;
  }

  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;

  // Estimate space needed (roughly 50 bytes per reading)
  size_t estimatedBytes = 50;

  if (freeBytes < estimatedBytes + 1000) {  // Keep 1KB buffer
    Serial.println(F("Flash storage full - stopping CSV logging"));
    csvLoggingEnabled = false;
    return false;
  }

  return true;
}
