#include <RadioLib.h>         // RadioLib for SX1262
#include <Wire.h>             // I2C for BMP280
#include <Adafruit_BMP280.h>  // BMP280 sensor library
#include <math.h>             // For altitude calculation
                              //

#define SAMPLE_INTERVAL 50 // sample sensors every N milliseconds

#define PRINT_SIGNAL_STATS true

// Heltec WiFi LoRa 32 V3 pin definitions for SX1262
#define LORA_NSS 8    // Chip select
#define LORA_SCK 9    // SPI clock
#define LORA_MOSI 10  // SPI MOSI
#define LORA_MISO 11  // SPI MISO
#define LORA_RST 12   // Reset
#define LORA_DIO1 14  // Interrupt (DIO1)
#define LORA_BUSY 13  // Busy pin

// LoRa settings
#define FREQUENCY 918.0  // US band
#define SPREADING_FACTOR 7
#define BANDWIDTH 250.0  // kHz
#define CODING_RATE 6    // 4:6
#define SYNC_WORD 0x34   // Public LoRa sync word
// #define POWER 22         // Max TX power (22dBm == 158mW)
#define POWER 3          // Low power for bench testing (3dBm == 2mW)
#define TXCO_VOLTAGE 1.8
#define PREAMBLE_LENGTH 8

// BMP280 I2C pins
#define I2C_SDA 41
#define I2C_SCL 42

// LoRa module instance
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// BMP280 instance
Adafruit_BMP280 bmp;

// Reference pressure at 480 feet (146.3 m) above sea level
#define P0 1007.49  // hPa, calculated for 480 feet

// Buffer for telemetry data (10 samples)
#define BUFFER_SIZE 10
struct Telemetry {
  uint16_t pressure;    // hPa * 100 (2 bytes)
  int16_t temperature;  // °C * 10 (2 bytes)
  uint16_t timestamp;   // ms since start (2 bytes)
  uint16_t altitude;    // meters * 100 (2 bytes)
} telemetryBuffer[BUFFER_SIZE];

// Compression buffer (delta-encoded)
uint8_t txBuffer[80];  // ~8 bytes/sample x 10 samples
uint8_t bufferIndex = 0;
unsigned long lastSampleTime = 0;
unsigned long lastTransmitTime = 0;
volatile bool isReceiving = false;
volatile bool isTransmitting = false;
volatile bool newControlMessage = false;
volatile bool radioOperationPending = false;
volatile bool shouldRestart = false;
volatile bool transmitComplete = false;
volatile bool receiveComplete = false;
uint8_t controlMessage = 0;

// Transmission state
enum TransmitState { STOPPED,
                     RUNNING };
TransmitState transmitState = STOPPED;  // Start in STOPPED state
TransmitState prevTransmitState = STOPPED;

// LoRa settings
#define FREQUENCY 918.0  // US band
#define SPREADING_FACTOR 7
#define BANDWIDTH 250.0  // kHz
#define CODING_RATE 6    // 4:6
#define SYNC_WORD 0x34   // Public LoRa sync word
// #define POWER 22         // Max TX power (22 dBm == 158 mW)
#define POWER 1 // Low power for bench testing (1 dBm == 1.3 mW)
#define TXCO_VOLTAGE 1.8
#define PREAMBLE_LENGTH 8

void IRAM_ATTR onDio1Action(void) {
  if (isReceiving) {
    receiveComplete = true;
    newControlMessage = true;
    isReceiving = false;
  }
  radioOperationPending = false;
}

// Calculate altitude relative to 480 feet
float calculateAltitude(float pressure, float temperature) {
  // Hypsometric formula adjusted for 480 feet reference
  float ratio = P0 / pressure;
  float altitude = (pow(ratio, 0.19026) - 1.0) / 0.0000225577;
  return altitude;  // Meters above 480 feet
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  /*
  // Initialize I2C for BMP280
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println(F("BMP280 not found! Check wiring."));
    while (1);
  }
  // Configure BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  */
  // Initialize SX1262
  int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE,
                          SYNC_WORD, POWER, PREAMBLE_LENGTH, TXCO_VOLTAGE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("LoRa init failed, code "));
    Serial.println(state);
    while (1)
      ;
  }
  radio.setDio1Action(onDio1Action);
  Serial.println(F("LoRa init success!"));

  // Start in receive mode
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    isReceiving = true;
    radioOperationPending = true;
  } else {
    Serial.print("start receive failed, code ");
    Serial.println(state);
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Sample every 20 ms only if RUNNING
  if (transmitState == RUNNING && currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    /*
    // Read BMP280
    float pressure = bmp.readPressure() / 100.0; // hPa
    float temperature = bmp.readTemperature();   // °C
    float altitude = calculateAltitude(pressure, temperature); // Meters
    */

    // Store in buffer (scale to integers)
    // telemetryBuffer[bufferIndex].pressure = (uint16_t)(pressure * 100);
    // telemetryBuffer[bufferIndex].temperature = (int16_t)(temperature * 10);
    // telemetryBuffer[bufferIndex].altitude = (uint16_t)(altitude * 100);
    telemetryBuffer[bufferIndex].timestamp = (uint16_t)(currentTime % 65536);
    telemetryBuffer[bufferIndex].pressure = (uint16_t)(1000);
    telemetryBuffer[bufferIndex].temperature = (uint16_t)(270);
    telemetryBuffer[bufferIndex].altitude = (uint16_t)0;

    bufferIndex++;
    lastSampleTime = currentTime;

    // Transmit when buffer is full (10 samples, every 500 ms)
    if (bufferIndex >= BUFFER_SIZE && !isReceiving) {
      // Simple delta encoding
      int txIndex = 0;
      uint16_t lastPressure = 0, lastTimestamp = 0, lastAltitude = 0;
      int16_t lastTemperature = 0;

      for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        // Delta encode to reduce size
        int16_t deltaPressure = telemetryBuffer[i].pressure - lastPressure;
        int16_t deltaTemperature = telemetryBuffer[i].temperature - lastTemperature;
        int16_t deltaTimestamp = telemetryBuffer[i].timestamp - lastTimestamp;
        int16_t deltaAltitude = telemetryBuffer[i].altitude - lastAltitude;

        // Store in txBuffer (2 bytes each)
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

      // Transmit over LoRa
      int state = radio.transmit(txBuffer, txIndex);
      if (state == RADIOLIB_ERR_NONE) {
        Serial.printf(F("Transmission successful. currentTime = %d\n"), (int)(currentTime % 65536));
      } else {
        Serial.println(state);
        // delay(10);
        return;
      }

      while (!txDone) {
        delay(1);
      }
      Serial.println("Transmission successful");

      bufferIndex = 0;  // Reset buffer
      lastTransmitTime = currentTime;

      // Switch to receive mode
      Serial.println("receive window start");
      radio.startReceive();
      isReceiving = true;
      radioOperationPending = true;
    }
  }

  // Check for received control messages (8-bit integers)
  if (receiveComplete) {
    uint8_t rxBuffer[1];  // Expecting 1-byte control message
    int state = radio.readData(rxBuffer, 1);
    Serial.println("read rxBuffer");
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print(F("Received control message: "));
      Serial.println(rxBuffer[0]);

  // If an unprocessed control message is present, act on it
  if (controlMessage > 0) {
    switch (controlMessage) {
      case 1:
        transmitState = RUNNING;
        Serial.println("Starting telemetry sampling and transmission");
        bufferIndex = 0;
        lastSampleTime = currentTime;
        radio.standby();
        break;
      case 2:
        transmitState = STOPPED;
        Serial.println(F("Stopping telemetry sampling and transmission"));
        bufferIndex = 0;  // Clear buffer
      }

      isReceiving = false;  // Stop receiving until next cycle
      receiveComplete = false;
      controlMessage = 0;
      radio.standby();
      delay(1);
    }

    // reset control message and await another
    controlMessage = 0;
    rxDone = false;
    radio.standby();
  }
}
