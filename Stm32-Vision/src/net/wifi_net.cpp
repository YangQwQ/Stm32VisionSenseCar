#include "src/net/wifi_net.h"
#include "src/net/config.h"
#include <WiFi.h>
#include <lwip/netif.h>          // struct netif / netif_default（压低 MTU 用）
#include "esp_netif.h"           // esp_netif_get_handle_from_ifkey
#include "esp_netif_net_stack.h" // esp_netif_get_netif_impl：拿到底层 lwIP netif
#include "src/core/board_log.h"

// 单次连接尝试超时：begin 后此期间视为"正在尝试"，静默等待连接结果不重复 begin。
// ESP32 内置 autoconnect 本身会断线重连；手动周期 begin 与它打架会反复打
// "wifi:sta is connecting, cannot set config"，故只做低频兜底。
static const unsigned long k_connect_timeout_ms = 10000;
static unsigned long s_begin_at = 0;  // 最近一次发起连接的时刻（0=从未发起）

void net::init() {
  // 无论有无配置，先拉起 lwIP/IP 栈：不调 WiFi.mode() 则 tcpip 未初始化，
  // 后续 httpd_start() 建 socket 会因空互斥量 assert 崩溃。
  WiFi.mode(WIFI_STA);
  if (cfg::wifi_ssid().isEmpty()) {
    blog::logf(blog::NET, "未配置 WiFi，等待配网");
    return;
  }
  WiFi.setSleep(false);
//   WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 顶格发射功率，避免省电默认压低上行吞吐
  s_begin_at = millis();
  WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
}

void net::clamp_wan_mtu() {
  // netif 指针缓存：esp_netif 的 lwIP netif 是开机建好、全程不销毁的（断线/换网都不重建），
  // 所以取一次就够 —— loop 里每轮调它只剩一次比较，不必每轮去查 ifkey（那要拿锁遍历链表）。
  static struct netif* s_n = nullptr;
  if (!s_n) {
    esp_netif_t* h = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (h) s_n = (struct netif*)esp_netif_get_netif_impl(h);
    if (!s_n) s_n = netif_default;   // 兜底：取不到具名网卡就用 lwIP 默认网卡
  }
  if (s_n && s_n->mtu > kWanMtu) s_n->mtu = kWanMtu;
}

void net::update() {
  if (cfg::wifi_ssid().isEmpty()) {
    return;
  }
  static bool s_was_connected = false;
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !s_was_connected) {
    clamp_wan_mtu();   // 先压 MTU 再报"已连接"：日志里的 MTU 才是真正生效的那个
    blog::logf(blog::NET, "已连接 %s, IP: %s, RSSI: %d dBm, MTU: %d",
                  cfg::wifi_ssid().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                  (int)(netif_default ? netif_default->mtu : 0));
  }
  s_was_connected = connected;
  if (connected) {
    clamp_wan_mtu();   // 幂等兜底：网卡 MTU 若被重置回默认，这里当轮就压回去
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
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 顶格发射功率，避免省电默认压低上行吞吐
  s_begin_at = millis();
  WiFi.begin(cfg::wifi_ssid().c_str(), cfg::wifi_pass().c_str());
}

bool net::is_connected() {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress net::local_ip() {
  return WiFi.localIP();
}
