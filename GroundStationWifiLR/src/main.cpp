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

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// Ensure WiFi credentials are defined at build time
#ifndef WIFI_SSID
#error "WIFI_SSID must be set as an environment variable"
#endif

#ifndef WIFI_PSK
#error "WIFI_PSK must be set as an environment variable"
#endif

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
 * Log a debug message to Serial with "LOG: " prefix
 */
void logDebug(const char* format, ...) {
  Serial.print("LOG: ");

  va_list args;
  va_start(args, format);

  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  Serial.println(buffer);

  va_end(args);
}

/**
 * Log telemetry data to Serial with "DATA: " prefix
 */
void logData(JsonDocument& doc) {
  Serial.print("DATA: ");
  serializeJson(doc, Serial);
  Serial.println();
}

/**
 * Parse and execute a command from serial
 * Format: "METHOD endpoint 'json_payload'"
 * Example: "POST /calibrate '{\"altitude\":0}'"
 */
void processSerialCommand(const char* command) {
  logDebug("UART command received: %s", command);

  char cmdCopy[SERIAL_BUFFER_SIZE];
  strncpy(cmdCopy, command, SERIAL_BUFFER_SIZE - 1);
  cmdCopy[SERIAL_BUFFER_SIZE - 1] = '\0';

  // Parse METHOD
  char* method = strtok(cmdCopy, " ");
  if (!method) {
    logDebug("ERROR: No method specified");
    return;
  }

  // Parse endpoint
  char* endpoint = strtok(NULL, " ");
  if (!endpoint) {
    logDebug("ERROR: No endpoint specified");
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

  if (strcmp(method, "LOCAL") == 0) {
    if (strcmp(endpoint, "status") == 0) {
      logDebug("WIFI_SSID: %s", WiFi.SSID().c_str());
      logDebug("WIFI_PSK set? %d", !!WIFI_PSK);
      logDebug("WIFI_CONNECTED? %d", WiFi.status() == WL_CONNECTED);
    } else {
      logDebug("No such local command");
    }
    return;
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
      logDebug("ERROR: %s requires JSON payload", method);
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
    logDebug("ERROR: Unknown method '%s'", method);
    http.end();
    return;
  }

  // Handle response
  if (httpCode > 0) {
    if (http.getSize() > 0) {
      logDebug("API Response HTTP %d: %s", httpCode, http.getString().c_str());
    } else {
      logDebug("API Response HTTP %d: OK", httpCode);
    }
  } else {
    logDebug("HTTP Request failed: %s", http.errorToString(httpCode).c_str());
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
      logDebug("ERROR: Command too long");
      serialBufferIndex = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // Allow serial to stabilize

  logDebug("LOG: Ground Station WiFi LR starting...");

  // Configure WiFi for AP mode
  WiFi.mode(WIFI_AP);
  logDebug("WiFi mode set to AP");
  WiFi.enableLongRange(true);
  logDebug("802.11 LR mode enabled");

  // Start the AP
  WiFi.softAP(WIFI_SSID, WIFI_PSK, 1, 0, 4);
  logDebug("WiFi AP started: SSID=%s, IP=%s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());


  // Start UDP server
  udp.begin(UDP_PORT);
  logDebug("UDP server listening on port %d\n", UDP_PORT);

  logDebug("Ground station ready");
}

void loop() {
  // Process serial commands from laptop
  processSerialInput();

  // Process UDP telemetry packets from rocket
  int packetSize = udp.parsePacket();

  if (packetSize > 0) {
    int len = udp.read(packetBuffer, UDP_BUFFER_SIZE);
    if (len > 0) {
      logDebug("UDP packet received: %d bytes from %s:%d",
               len, udp.remoteIP().toString().c_str(), udp.remotePort());

      // Deserialize MsgPack to JSON document
      JsonDocument doc;
      DeserializationError error = deserializeMsgPack(doc, packetBuffer, len);

      if (!error) {
        // Serialize to JSON and print to Serial with DATA prefix
        logData(doc);
      } else {
        logDebug("ERROR: MsgPack deserialization failed: %s", error.c_str());
      }
    }
  }
}

