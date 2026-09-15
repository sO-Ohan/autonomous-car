#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"

static const char *AP_SSID = "ESP32-RC";
static const char *AP_PASS = "12345678";

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nc3-ap-test booting (bare, no server)");

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(200);

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
  Serial.printf("softAP() returned %s\n", apOk ? "true" : "FALSE");
  Serial.printf("MAC %s, IP %s\n", WiFi.softAPmacAddress().c_str(), WiFi.softAPIP().toString().c_str());
  Serial.println("ready");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 2000) {
    last = millis();
    Serial.printf("[%lus] stations connected: %d\n", millis() / 1000, WiFi.softAPgetStationNum());
  }
}
