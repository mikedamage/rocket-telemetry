#include <RadioLib.h>
#include <WiFi.h>

// Heltec WiFi LoRa 32 V3 pin definitions for SX1262
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RST 12
#define LORA_DIO1 14
#define LORA_BUSY 13

// WiFi credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// TCP client settings (for forwarding telemetry)
const char* tcpClientServerIP = "192.168.1.100";
const uint16_t tcpClientPort = 5000;

// TCP server settings (for receiving control messages)
const uint16_t tcpServerPort = 5001;

// LoRa module instance
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// WiFi client (for forwarding telemetry)
WiFiClient tcpClient;

// WiFi server (for receiving control messages)
WiFiServer tcpServer(tcpServerPort);
WiFiClient tcpServerClient;

// Radio state
volatile bool radioOperationPending = false;
volatile bool transmitComplete = false;
volatile bool receiveComplete = false;
volatile bool channelScanComplete = false;
volatile bool channelClear = false;
uint8_t controlMessage = 42;
bool newControlMessage = false;

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

  // Connect to TCP client server
  if (tcpClient.connect(tcpClientServerIP, tcpClientPort)) {
    Serial.println(F("Connected to TCP client server"));
  } else {
    Serial.println(F("TCP client connection failed"));
  }

  // Start TCP server
  tcpServer.begin();
  Serial.print(F("TCP server started on port "));
  Serial.println(tcpServerPort);

  // Initialize SX1262
  int state = radio.begin(915.0, 250.0, 7, 6, 0x34, 22, 8, 0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("LoRa init failed, code "));
    Serial.println(state);
    while (1);
  }
  radio.setPacketSentAction(onPacketSent);
  radio.setPacketReceivedAction(onPacketReceived);
  radio.setDio1Action(onChannelScan);
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

// Callbacks
void onPacketSent() {
  transmitComplete = true;
  radioOperationPending = false;
}

void onPacketReceived() {
  receiveComplete = true;
  radioOperationPending = false;
}

void onChannelScan() {
  channelScanComplete = true;
  radioOperationPending = false;
}

void loop() {
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
    tcpServer.begin();
  }

  // Reconnect TCP client if disconnected
  if (!tcpClient.connected()) {
    Serial.println(F("TCP client disconnected, reconnecting..."));
    if (tcpClient.connect(tcpClientServerIP, tcpClientPort)) {
      Serial.println(F("Reconnected to TCP client server"));
    } else {
      Serial.println(F("TCP client reconnection failed"));
    }
  }

  // Check for new TCP server clients
  if (tcpServer.hasClient()) {
    if (!tcpServerClient || !tcpServerClient.connected()) {
      if (tcpServerClient) {
        tcpServerClient.stop();
      }
      tcpServerClient = tcpServer.available();
      Serial.println(F("New TCP server client connected"));
    }
  }

  // Read control messages from TCP server client
  if (tcpServerClient && tcpServerClient.connected() && tcpServerClient.available()) {
    controlMessage = tcpServerClient.read();
    newControlMessage = true;
    Serial.print(F("Received TCP control message: "));
    Serial.println(controlMessage);

    // Attempt immediate transmission if radio is free
    if (!radioOperationPending) {
      int state = radio.startChannelScan();
      if (state == RADIOLIB_ERR_NONE) {
        radioOperationPending = true;
        channelScanComplete = false;
      } else {
        Serial.print(F("Start channel scan failed, code "));
        Serial.println(state);
      }
    }
  }

  // Handle channel scan completion
  if (channelScanComplete) {
    int state = radio.getChannelScanResult();
    if (state == RADIOLIB_CHANNEL_CLEAR && newControlMessage) {
      state = radio.startTransmit(&controlMessage, 1);
      if (state == RADIOLIB_ERR_NONE) {
        radioOperationPending = true;
        transmitComplete = false;
      } else {
        Serial.print(F("Start transmit failed, code "));
        Serial.println(state);
      }
    } else {
      Serial.println(F("Channel busy or no new message, retrying soon..."));
      // Retry channel scan if needed
      if (newControlMessage) {
        state = radio.startChannelScan();
        if (state == RADIOLIB_ERR_NONE) {
          radioOperationPending = true;
          channelScanComplete = false;
        } else {
          Serial.print(F("Start channel scan failed, code "));
          Serial.println(state);
        }
      } else {
        // Return to receive mode
        state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE) {
          radioOperationPending = true;
        } else {
          Serial.print(F("Start receive failed, code "));
          Serial.println(state);
          radioOperationPending = false;
        }
      }
    }
    channelScanComplete = false;
  }

  // Handle completed transmission
  if (transmitComplete) {
    Serial.print(F("Sent control message: "));
    Serial.println(controlMessage);
    newControlMessage = false;
    transmitComplete = false;
    // Return to receive mode
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      radioOperationPending = true;
    } else {
      Serial.print(F("Start receive failed, code "));
      Serial.println(state);
      radioOperationPending = false;
    }
  }

  // Handle received telemetry
  if (receiveComplete) {
    uint8_t rxBuffer[80];
    int state = radio.readData(rxBuffer, 80);
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

      // Send control message if pending
      if (newControlMessage || controlMessage != 42) {
        state = radio.startChannelScan();
        if (state == RADIOLIB_ERR_NONE) {
          radioOperationPending = true;
          channelScanComplete = false;
        } else {
          Serial.print(F("Start channel scan failed, code "));
          Serial.println(state);
          // Fallback to receive mode
          state = radio.startReceive();
          if (state == RADIOLIB_ERR_NONE) {
            radioOperationPending = true;
          } else {
            Serial.print(F("Start receive failed, code "));
            Serial.println(state);
            radioOperationPending = false;
          }
        }
      } else {
        // Send default control message
        uint8_t defaultMessage = 42;
        state = radio.startChannelScan();
        if (state == RADIOLIB_ERR_NONE) {
          radioOperationPending = true;
          channelScanComplete = false;
          controlMessage = defaultMessage;
        } else {
          Serial.print(F("Start channel scan failed, code "));
          Serial.println(state);
          state = radio.startReceive();
          if (state == RADIOLIB_ERR_NONE) {
            radioOperationPending = true;
          } else {
            Serial.print(F("Start receive failed, code "));
            Serial.println(state);
            radioOperationPending = false;
          }
        }
      }
    } else {
      Serial.print(F("Receive error, code "));
      Serial.println(state);
      state = radio.startReceive();
      if (state == RADIOLIB_ERR_NONE) {
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
      radioOperationPending = true;
    } else {
      Serial.print(F("Start receive failed, code "));
      Serial.println(state);
      radioOperationPending = false;
    }
    lastReceiveTime = currentTime;
  }
}
