/**
 * Ground Station WiFi LR - ESP32 Access Point with 802.11 LR Mode
 *
 * Creates a WiFi AP using ESP32's proprietary 802.11 LR (Long Range) mode
 * and listens for UDP packets containing MsgPack-encoded data,
 * deserializes to JSON, and prints to Serial with newline delimiters.
 *
 * Also receives control commands over serial and forwards them to the rocket's REST API.
 * Command format: "METHOD endpoint 'json_payload'"
 * Example: "POST /calibrate '{"altitude":0}'"
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// AP Configuration
const char* AP_SSID = "RocketTelemetry";
const char* AP_PASSWORD = "telemetry123";

// UDP Configuration
const uint16_t UDP_PORT = 4210;
const size_t UDP_BUFFER_SIZE = 512;

// Rocket IP Configuration (typically first client)
const char* ROCKET_IP = "192.168.4.2";
const uint16_t ROCKET_HTTP_PORT = 80;

WiFiUDP udp;
uint8_t packetBuffer[UDP_BUFFER_SIZE];

// Serial command buffer
const size_t SERIAL_BUFFER_SIZE = 256;
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferIndex = 0;

/**
 * Parse and execute a command from serial
 * Format: "METHOD endpoint 'json_payload'"
 * Example: "POST /calibrate '{\"altitude\":0}'"
 */
void processSerialCommand(const char* command) {
  char cmdCopy[SERIAL_BUFFER_SIZE];
  strncpy(cmdCopy, command, SERIAL_BUFFER_SIZE - 1);
  cmdCopy[SERIAL_BUFFER_SIZE - 1] = '\0';

  // Parse METHOD
  char* method = strtok(cmdCopy, " ");
  if (!method) {
    Serial.println("ERROR: No method specified");
    return;
  }

  // Parse endpoint
  char* endpoint = strtok(NULL, " ");
  if (!endpoint) {
    Serial.println("ERROR: No endpoint specified");
    return;
  }

  // Parse JSON payload (everything after endpoint, trimming quotes)
  char* payload = strtok(NULL, "");

  // Trim leading/trailing whitespace and quotes from payload
  if (payload) {
    while (*payload == ' ' || *payload == '\'') payload++;
    size_t len = strlen(payload);
    while (len > 0 && (payload[len-1] == ' ' || payload[len-1] == '\'')) {
      payload[len-1] = '\0';
      len--;
    }
  }

  // Build URL
  char url[128];
  snprintf(url, sizeof(url), "http://%s:%d%s", ROCKET_IP, ROCKET_HTTP_PORT, endpoint);

  // Make HTTP request
  HTTPClient http;
  http.begin(url);

  int httpCode = -1;

  if (strcmp(method, "GET") == 0) {
    httpCode = http.GET();
  } else if (strcmp(method, "POST") == 0 || strcmp(method, "PUT") == 0) {
    if (!payload || strlen(payload) == 0) {
      Serial.printf("ERROR: %s requires JSON payload\n", method);
      http.end();
      return;
    }
    http.addHeader("Content-Type", "application/json");
    if (strcmp(method, "POST") == 0) {
      httpCode = http.POST(payload);
    } else {
      httpCode = http.PUT(payload);
    }
  } else if (strcmp(method, "DELETE") == 0) {
    httpCode = http.sendRequest("DELETE");
  } else {
    Serial.printf("ERROR: Unknown method '%s'\n", method);
    http.end();
    return;
  }

  // Handle response
  if (httpCode > 0) {
    Serial.printf("HTTP %d: ", httpCode);
    if (http.getSize() > 0) {
      Serial.println(http.getString());
    } else {
      Serial.println("OK");
    }
  } else {
    Serial.printf("HTTP Request failed: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

/**
 * Process incoming serial data
 */
void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';
        processSerialCommand(serialBuffer);
        serialBufferIndex = 0;
      }
    } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialBufferIndex++] = c;
    } else {
      Serial.println("ERROR: Command too long");
      serialBufferIndex = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configure WiFi for AP mode
  WiFi.mode(WIFI_AP);

  // Start the AP
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);

  // Enable 802.11 LR mode for extended range
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);

  // Start UDP server
  udp.begin(UDP_PORT);
}

void loop() {
  // Process serial commands from laptop
  processSerialInput();

  // Process UDP telemetry packets from rocket
  int packetSize = udp.parsePacket();

  if (packetSize > 0) {
    int len = udp.read(packetBuffer, UDP_BUFFER_SIZE);
    if (len > 0) {
      // Deserialize MsgPack to JSON document
      JsonDocument doc;
      DeserializationError error = deserializeMsgPack(doc, packetBuffer, len);

      if (!error) {
        // Serialize to JSON and print to Serial
        serializeJson(doc, Serial);
        Serial.println();
      }
    }
  }
}
