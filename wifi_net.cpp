#include "wifi_net.h"
#include "config.h"
#include <WiFi.h>

static const unsigned long k_reconnect_interval_ms = 5000;

void net::init() {
  if (cfg::wifi_ssid().isEmpty()) {
    Serial.println("[net] 未配置 WiFi，等待配网");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
}

void net::update() {
  if (WiFi.status() == WL_CONNECTED || cfg::wifi_ssid().isEmpty()) {
    return;
  }
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= k_reconnect_interval_ms) {
    last = now;
    WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
  }
}

bool net::is_connected() {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress net::local_ip() {
  return WiFi.localIP();
}
