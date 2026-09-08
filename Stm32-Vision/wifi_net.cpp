#include "wifi_net.h"
#include "config.h"
#include <WiFi.h>

// 单次连接尝试超时：begin 后此期间视为"正在尝试"，静默等待连接结果不重复 begin。
// ESP32 内置 autoconnect 本身会断线重连；手动周期 begin 与它打架会反复打
// "wifi:sta is connecting, cannot set config"，故只做低频兜底。
static const unsigned long k_connect_timeout_ms = 30000;
static unsigned long s_begin_at = 0;  // 最近一次发起连接的时刻（0=从未发起）

void net::init() {
  // 无论有无配置，先拉起 lwIP/IP 栈：不调 WiFi.mode() 则 tcpip 未初始化，
  // 后续 httpd_start() 建 socket 会因空互斥量 assert 崩溃。
  WiFi.mode(WIFI_STA);
  if (cfg::wifi_ssid().isEmpty()) {
    Serial.println("[net] 未配置 WiFi，等待配网");
    return;
  }
  WiFi.setSleep(false);
  s_begin_at = millis();
  WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
}

void net::update() {
  if (cfg::wifi_ssid().isEmpty() || WiFi.status() == WL_CONNECTED) {
    return;
  }
  // 距上次 begin 未超时：连接尝试仍在进行，静默等待，不重复 begin
  if (s_begin_at != 0 && millis() - s_begin_at < k_connect_timeout_ms) {
    return;
  }
  s_begin_at = millis();
  WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
}

void net::reconnect() {
  WiFi.disconnect();  // 断开当前连接，立即以最新配置重建 STA（不重启，BLE 保活）
  WiFi.setSleep(false);
  s_begin_at = millis();
  WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
}

bool net::is_connected() {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress net::local_ip() {
  return WiFi.localIP();
}
