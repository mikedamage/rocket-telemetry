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
const char* ssid = "YOUR_SSID";         // Replace with your WiFi SSID
const char* password = "YOUR_PASSWORD"; // Replace with your WiFi password

// TCP client settings (for forwarding telemetry)
const char* tcpClientServerIP = "192.168.1.100"; // Replace with telemetry server IP
const uint16_t tcpClientPort = 5000;             // Replace with telemetry server port

// TCP server settings (for receiving control messages)
const uint16_t tcpServerPort = 5001; // Port for control message server

// LoRa module instance
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// WiFi client (for forwarding telemetry)
WiFiClient tcpClient;

// WiFi server (for receiving control messages)
WiFiServer tcpServer(tcpServerPort);
WiFiClient tcpServerClient; // Client connected to the TCP server

bool isReceiving = true;
unsigned long lastReceiveTime = 0;
uint8_t controlMessage = 42; // Default control message
bool newControlMessage = false; // Flag for new TCP control message

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
    Serial.println(F("Connected to TCP client server"));
  } else {
    Serial.println(F("TCP client connection failed"));
  }

  // Start TCP server (for control messages)
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
  Serial.println(F("LoRa init success!"));

  // Start receiving
  radio.startReceive();
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
    tcpServer.begin(); // Restart TCP server
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
        tcpServerClient.stop(); // Close existing client
      }
      tcpServerClient = tcpServer.available();
      Serial.println(F("New TCP server client connected"));
    }
  }

  // Read control messages from TCP server client
  if (tcpServerClient && tcpServerClient.connected() && tcpServerClient.available()) {
    controlMessage = tcpServerClient.read(); // Read 8-bit integer
    newControlMessage = true;
    Serial.print(F("Received TCP control message: "));
    Serial.println(controlMessage);

    // Attempt to send control message immediately if channel is clear
    if (isReceiving) {
      radio.standby(); // Stop receiving to scan channel
      isReceiving = false;
    }
    int state = radio.scanChannel();
    if (state == RADIOLIB_CHANNEL_CLEAR) {
      state = radio.transmit(&controlMessage, 1);
      if (state == RADIOLIB_ERR_NONE) {
        Serial.print(F("Sent control message: "));
        Serial.println(controlMessage);
        newControlMessage = false; // Clear flag after sending
      } else {
        Serial.print(F("Control message failed, code "));
        Serial.println(state);
      }
    } else {
      Serial.println(F("Channel busy, retrying soon..."));
      // Retry in next loop iteration (~10 ms delay)
    }
    radio.startReceive(); // Return to receive mode
    isReceiving = true;
    lastReceiveTime = currentTime;
  }

  // Receive LoRa telemetry
  if (isReceiving) {
    uint8_t rxBuffer[80]; // 10 samples x 8 bytes
    int state = radio.receive(rxBuffer, 80);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Received telemetry packet!"));
      // Decode delta-encoded data
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

        // Format telemetry as string
        char telemetryStr[100];
        snprintf(telemetryStr, sizeof(telemetryStr),
                 "Pressure=%.2f hPa, Temperature=%.1f °C, Timestamp=%u ms, Altitude=%.2f m\n",
                 pressure / 100.0, temperature / 10.0, timestamp, altitude / 100.0);

        // Send to Serial and TCP client
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

      // Send control message if one is pending and channel is clear
      if (newControlMessage) {
        radio.standby(); // Stop receiving to scan channel
        isReceiving = false;
        state = radio.scanChannel();
        if (state == RADIOLIB_CHANNEL_CLEAR) {
          state = radio.transmit(&controlMessage, 1);
          if (state == RADIOLIB_ERR_NONE) {
            Serial.print(F("Sent control message: "));
            Serial.println(controlMessage);
            newControlMessage = false; // Clear flag after sending
          } else {
            Serial.print(F("Control message failed, code "));
            Serial.println(state);
          }
        } else {
          Serial.println(F("Channel busy after telemetry, retrying soon..."));
        }
      } else {
        // Send default control message (42) if no new message
        uint8_t defaultMessage = 42;
        state = radio.scanChannel();
        if (state == RADIOLIB_CHANNEL_CLEAR) {
          state = radio.transmit(&defaultMessage, 1);
          if (state == RADIOLIB_ERR_NONE) {
            Serial.print(F("Sent default control message: "));
            Serial.println(defaultMessage);
          } else {
            Serial.print(F("Control message failed, code "));
            Serial.println(state);
          }
        } else {
          Serial.println(F("Channel busy for default message, skipping..."));
        }
      }

      // Return to receive mode
      radio.startReceive();
      isReceiving = true;
      lastReceiveTime = currentTime;
    }
  }

  // Timeout to prevent getting stuck in receive mode
  if (isReceiving && currentTime - lastReceiveTime >= 500) {
    radio.startReceive(); // Restart receive
    lastReceiveTime = currentTime;
  }
}