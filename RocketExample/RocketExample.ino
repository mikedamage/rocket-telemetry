#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// Heltec WiFi LoRa 32 V3 pin definitions for SX1262
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RST 12
#define LORA_DIO1 14
#define LORA_BUSY 13

// BMP280 I2C pins
#define I2C_SDA 41
#define I2C_SCL 42

// LoRa module instance
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// BMP280 instance
Adafruit_BMP280 bmp;

// Reference pressure at 480 feet (146.3 m) above sea level
#define P0 1007.49 // hPa

// Buffer for telemetry data (10 samples)
#define BUFFER_SIZE 10
struct Telemetry {
  uint16_t pressure;    // hPa * 100 (2 bytes)
  int16_t temperature;  // °C * 10 (2 bytes)
  uint16_t timestamp;   // ms since start (2 bytes)
  uint16_t altitude;    // meters * 100 (2 bytes)
} telemetryBuffer[BUFFER_SIZE];

// Compression buffer (delta-encoded)
uint8_t txBuffer[80]; // ~8 bytes/sample x 10 samples
uint8_t bufferIndex = 0;
unsigned long lastSampleTime = 0;
unsigned long lastTransmitTime = 0;

// Radio state
volatile bool radioOperationPending = false;
volatile bool transmitComplete = false;
volatile bool receiveComplete = false;
uint8_t receivedControlMessage = 0;

// Transmission state
enum TransmitState { STOPPED, RUNNING };
TransmitState transmitState = STOPPED; // Start in STOPPED state

// LoRa settings
#define FREQUENCY 915.0
#define SPREADING_FACTOR 7
#define BANDWIDTH 250.0
#define CODING_RATE 6
#define SYNC_WORD 0x34
#define POWER 22

// Calculate altitude relative to 480 feet
float calculateAltitude(float pressure, float temperature) {
  float ratio = P0 / pressure;
  float altitude = (pow(ratio, 0.19026) - 1.0) / 0.0000225577;
  return altitude; // Meters above 480 feet
}

// Callbacks for non-blocking operations
void onPacketSent() {
  transmitComplete = true;
  radioOperationPending = false;
}

void onPacketReceived() {
  receiveComplete = true;
  radioOperationPending = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize I2C for BMP280
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println(F("BMP280 not found! Check wiring."));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Initialize SX1262
  int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE,
                          SYNC_WORD, POWER, 8, 0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("LoRa init failed, code "));
    Serial.println(state);
    while (1);
  }
  // Set callbacks
  radio.setPacketSentAction(onPacketSent);
  radio.setPacketReceivedAction(onPacketReceived);
  Serial.println(F("LoRa init success!"));

  // Start receiving
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Start receive failed, code "));
    Serial.println(state);
    while (1);
  }
  radioOperationPending = true;
}

void loop() {
  unsigned long currentTime = millis();

  // Sample every 20 ms if RUNNING
  if (transmitState == RUNNING && currentTime - lastSampleTime >= 20) {
    float pressure = bmp.readPressure() / 100.0;
    float temperature = bmp.readTemperature();
    float altitude = calculateAltitude(pressure, temperature);

    telemetryBuffer[bufferIndex].pressure = (uint16_t)(pressure * 100);
    telemetryBuffer[bufferIndex].temperature = (int16_t)(temperature * 10);
    telemetryBuffer[bufferIndex].timestamp = (uint16_t)(currentTime % 65536);
    telemetryBuffer[bufferIndex].altitude = (uint16_t)(altitude * 100);

    bufferIndex++;
    lastSampleTime = currentTime;

    // Transmit when buffer is full and radio is free
    if (bufferIndex >= BUFFER_SIZE && !radioOperationPending) {
      // Delta encoding
      int txIndex = 0;
      uint16_t lastPressure = 0, lastTimestamp = 0, lastAltitude = 0;
      int16_t lastTemperature = 0;

      for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        int16_t deltaPressure = telemetryBuffer[i].pressure - lastPressure;
        int16_t deltaTemperature = telemetryBuffer[i].temperature - lastTemperature;
        int16_t deltaTimestamp = telemetryBuffer[i].timestamp - lastTimestamp;
        int16_t deltaAltitude = telemetryBuffer[i].altitude - lastAltitude;

        txBuffer[txIndex++] = deltaPressure >> 8;
        txBuffer[txIndex++] = deltaPressure & 0xFF;
        txBuffer[txIndex++] = deltaTemperature >> 8;
        txBuffer[txIndex++] = deltaTemperature & 0xFF;
        txBuffer[txIndex++] = deltaTimestamp >> 8;
        txBuffer[txIndex++] = deltaTimestamp & 0xFF;
        txBuffer[txIndex++] = deltaAltitude >> 8;
        txBuffer[txIndex++] = deltaAltitude & 0xFF;

        lastPressure = telemetryBuffer[i].pressure;
        lastTemperature = telemetryBuffer[i].temperature;
        lastTimestamp = telemetryBuffer[i].timestamp;
        lastAltitude = telemetryBuffer[i].altitude;
      }

      // Start non-blocking transmission
      int state = radio.startTransmit(txBuffer, txIndex);
      if (state == RADIOLIB_ERR_NONE) {
        radioOperationPending = true;
        transmitComplete = false;
        lastTransmitTime = currentTime;
        bufferIndex = 0;
      } else {
        Serial.print(F("Start transmit failed, code "));
        Serial.println(state);
      }
    }
  }

  // Handle completed transmission
  if (transmitComplete) {
    Serial.println(F("Transmission successful"));
    transmitComplete = false;
    // Switch to receive mode
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      radioOperationPending = true;
    } else {
      Serial.print(F("Start receive failed, code "));
      Serial.println(state);
      radioOperationPending = false;
    }
  }

  // Handle received control messages
  if (receiveComplete) {
    uint8_t rxBuffer[1];
    int state = radio.readData(rxBuffer, 1);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print(F("Received control message: "));
      Serial.println(rxBuffer[0]);
      receivedControlMessage = rxBuffer[0];

      // Handle control messages
      if (receivedControlMessage == 1) {
        transmitState = RUNNING;
        Serial.println(F("Starting telemetry sampling and transmission"));
        bufferIndex = 0;
        lastSampleTime = currentTime;
      } else if (receivedControlMessage == 2) {
        transmitState = STOPPED;
        Serial.println(F("Stopping telemetry sampling and transmission"));
        bufferIndex = 0;
      }
    } else {
      Serial.print(F("Receive error, code "));
      Serial.println(state);
    }

    receiveComplete = false;
    // Restart receive mode
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      radioOperationPending = true;
    } else {
      Serial.print(F("Start receive failed, code "));
      Serial.println(state);
      radioOperationPending = false;
    }
  }

  // Timeout to restart receive if stuck
  if (radioOperationPending && currentTime - lastTransmitTime >= 500) {
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      radioOperationPending = true;
    } else {
      Serial.print(F("Start receive failed, code "));
      Serial.println(state);
      radioOperationPending = false;
    }
  }
}