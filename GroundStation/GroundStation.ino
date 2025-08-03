#define HELTEC_NO_RADIOLIB
#define ARDUINO_heltec_wifi_32_lora_V3

#include <RadioLib.h>
#include <WiFi.h>
#include <heltec_unofficial.h>

// Heltec WiFi LoRa 32 V3 pin definitions for SX1262
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RST 12
#define LORA_DIO1 14
#define LORA_BUSY 13

// LoRa settings
#define FREQUENCY 918.0 // US band
#define SPREADING_FACTOR 7
#define BANDWIDTH 250.0 // kHz
#define CODING_RATE 6   // 4:6
#define SYNC_WORD 0x34  // Public LoRa sync word
#define POWER 1 // Low power for bench testing (1 dBm == 1.3 mW)
// #define POWER 22        // Max TX power (22 dBm == 158 mW)
#define TXCO_VOLTAGE 1.8
#define PREAMBLE_LENGTH 8

// WiFi credentials
const char* ssid = "***REMOVED***";         // Replace with your WiFi SSID
const char* password = "***REMOVED***"; // Replace with your WiFi password

// TCP client settings (for forwarding telemetry)
const char* tcpClientServerIP = "192.168.1.249"; // Replace with telemetry server IP
const uint16_t tcpClientPort = 5150;             // Replace with telemetry server port

// TCP server settings (for receiving control messages)
const uint16_t tcpServerPort = 5151; // Port for control message server

// LoRa module instance
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// WiFi client (for forwarding telemetry)
WiFiClient tcpClient;

// WiFi server (for receiving control messages)
WiFiServer tcpServer(tcpServerPort);
WiFiClient tcpServerClient; // Client connected to the TCP server

// Radio state
enum OperatingState {
  STANDBY,
  RECEIVING,
  TRANSMITTING
};

OperatingState operatingState = STANDBY;
volatile bool radioOperationPending = false;
volatile bool transmitComplete = false;
volatile bool receiveComplete = false;
volatile bool channelScanComplete = false;
volatile bool channelClear = false;
uint8_t controlMessage = 0;
unsigned long lastReceiveTime = 0;
bool newControlMessage = false;

// Callbacks
ICACHE_RAM_ATTR void onDio1Action(void) {
  if (operatingState == RECEIVING) {
    receiveComplete = true;
  } else if (operatingState == TRANSMITTING) {
    transmitComplete = true;
  }

  radioOperationPending = false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize WiFi
  Serial.print(F("Connecting to WiFi: "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println(F("\nWiFi connected! IP: "));
  Serial.println(WiFi.localIP());

  // Connect to TCP client server (for telemetry)
  if (tcpClient.connect(tcpClientServerIP, tcpClientPort)) {
    Serial.println(F("Connected to telemetry storage server"));
  } else {
    Serial.println(F("TCP client connection failed"));
  }

  // Start TCP server (for control messages)
  tcpServer.begin();
  Serial.print(F("Control server listening on port "));
  Serial.println(tcpServerPort);

  // Initialize SX1262
  int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, SYNC_WORD, POWER, PREAMBLE_LENGTH, TXCO_VOLTAGE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("LoRa init failed, code "));
    Serial.println(state);
    while (1);
  }
  radio.setDio1Action(onDio1Action);
  Serial.println(F("LoRa init success!"));

  // Start receiving
  radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Start receive failed, code "));
    Serial.println(state);
    while (1);
  }
  operatingState = RECEIVING;
}

void loop() {
  int state;
  unsigned long currentTime = millis();

  // Reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi disconnected, reconnecting..."));
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(F("."));
    }
    Serial.println(F("\nWiFi reconnected"));
    tcpServer.begin(); // Restart TCP server
  }

  // Reconnect TCP client if disconnected
  if (!tcpClient.connected()) {
    Serial.println(F("Telemetry storage server disconnected, reconnecting..."));
    if (tcpClient.connect(tcpClientServerIP, tcpClientPort)) {
      Serial.println(F("Reconnected to telemetry storage server"));
    } else {
      Serial.println(F("Telemetry storage server reconnection failed"));
    }
  }

  // Check for new TCP server clients
  if (tcpServer.hasClient()) {
    if (!tcpServerClient || !tcpServerClient.connected()) {
      if (tcpServerClient) {
        tcpServerClient.stop(); // Close existing client
      }
      tcpServerClient = tcpServer.available();
      Serial.println(F("New control server client connected"));
    }
  }

  // Read control messages from control server client
  if (tcpServerClient && tcpServerClient.connected() && tcpServerClient.available()) {
    controlMessage = tcpServerClient.read();
    newControlMessage = true;
    Serial.print(F("Received TCP control message: "));
    Serial.println(controlMessage);

  }

  if (!radioOperationPending && newControlMessage) {
    Serial.println("transmitting control message");
    operatingState = TRANSMITTING;
    radioOperationPending = true;
    state = radio.transmit(&controlMessage, 1);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("transmission complete");
      transmitComplete = true;
      operatingState = RECEIVING;
      radioOperationPending = false;
      newControlMessage = false;
    } else {
      Serial.print(F("Start transmit failed, code "));
      Serial.println(state);
    }

    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      radioOperationPending = true;
      operatingState = RECEIVING;
    } else {
      Serial.print(F("Start receive falsed, code "));
      Serial.println(state);
      radioOperationPending = false;
      operatingState = STANDBY;
    }
  }

  // Handle received telemetry
  if (receiveComplete) {
    uint8_t rxBuffer[80];
    state = radio.readData(rxBuffer, 80);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Received telemetry packet!"));
      uint16_t lastPressure = 0, lastTimestamp = 0, lastAltitude = 0;
      int16_t lastTemperature = 0;
      for (int i = 0; i < 80; i += 8) {
        int16_t deltaPressure = (rxBuffer[i] << 8) | rxBuffer[i + 1];
        int16_t deltaTemperature = (rxBuffer[i + 2] << 8) | rxBuffer[i + 3];
        int16_t deltaTimestamp = (rxBuffer[i + 4] << 8) | rxBuffer[i + 5];
        int16_t deltaAltitude = (rxBuffer[i + 6] << 8) | rxBuffer[i + 7];

        uint16_t pressure = lastPressure + deltaPressure;
        int16_t temperature = lastTemperature + deltaTemperature;
        uint16_t timestamp = lastTimestamp + deltaTimestamp;
        uint16_t altitude = lastAltitude + deltaAltitude;

        char telemetryStr[100];
        snprintf(telemetryStr, sizeof(telemetryStr),
                 "Pressure=%.2f hPa, Temperature=%.1f °C, Timestamp=%u ms, Altitude=%.2f m\n",
                 pressure / 100.0, temperature / 10.0, timestamp, altitude / 100.0);

        Serial.print(telemetryStr);
        if (tcpClient.connected()) {
          tcpClient.print(telemetryStr);
        } else {
          Serial.println(F("TCP client not connected, skipping send"));
        }

        lastPressure = pressure;
        lastTemperature = temperature;
        lastTimestamp = timestamp;
        lastAltitude = altitude;
      }
    } else {
      Serial.print(F("Receive error, code "));
      Serial.println(state);
      state = radio.startReceive();
      if (state == RADIOLIB_ERR_NONE) {
        operatingState = RECEIVING;
        radioOperationPending = true;
      } else {
        Serial.print(F("Start receive failed, code "));
        Serial.println(state);
        radioOperationPending = false;
      }
    }
    receiveComplete = false;
  }

  // Timeout to restart receive if stuck
  if (radioOperationPending && currentTime - lastReceiveTime >= 500) {
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      operatingState = RECEIVING;
      radioOperationPending = true;
    } else {
      Serial.print(F("Start receive failed, code "));
      Serial.println(state);
      radioOperationPending = false;
    }
    lastReceiveTime = currentTime;
  }
}
