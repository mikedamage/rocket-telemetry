#include <WiFi.h>
#include "esp_private/wifi.h"
#include "esp_wifi.h"

void setFixedWifiRate(wifi_interface_t ifx, bool enable, wifi_phy_rate_t rate) {
  esp_wifi_internal_set_fix_rate(ifx, enable, rate);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("GroundStation", "***REMOVED***");

  wifi_config_t wifi_config;
  esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
  esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11N); // ensure 802.11n

  // try to set a fixed data rate of MCS0
  setFixedWifiRate(WIFI_IF_AP, true, WIFI_PHY_RATE_MCS0_LGI);
  Serial.println("Wifi started with MCS 0 data rate and 802.11n");
}

void loop() {
  // put your main code here, to run repeatedly:

}
