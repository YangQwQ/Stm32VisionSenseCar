#include "wifi_net.h"
#include "config.h"
#include <WiFi.h>

static const unsigned long k_reconnect_interval_ms = 5000;

void net::init() {
  // 无论有无配置，先拉起 lwIP/IP 栈：不调 WiFi.mode() 则 tcpip 未初始化，
  // 后续 httpd_start() 建 socket 会因空互斥量 assert 崩溃。
  WiFi.mode(WIFI_STA);
  if (cfg::wifi_ssid().isEmpty()) {
    Serial.println("[net] 未配置 WiFi，等待配网");
    return;
  }
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
