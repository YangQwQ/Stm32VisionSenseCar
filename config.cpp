#include "config.h"
#include <Preferences.h>

static const char* k_ns = "vision";

struct ConfigData {
  String wifi_ssid;
  String wifi_pass;
  String ai_url;
  String ai_key;
  String ai_model;
  uint32_t uart_baud;
};

static ConfigData g_cfg;

void cfg::init() {
  Preferences prefs;
  prefs.begin(k_ns, true);
  g_cfg.wifi_ssid = prefs.getString("wifi_ssid", "");
  g_cfg.wifi_pass = prefs.getString("wifi_pass", "");
  g_cfg.ai_url = prefs.getString("ai_url", "https://api.deepseek.com/chat/completions");
  g_cfg.ai_key = prefs.getString("ai_key", "");
  g_cfg.ai_model = prefs.getString("ai_model", "deepseek-v4-flash-vision-exp");
  g_cfg.uart_baud = prefs.getUInt("uart_baud", 115200);
  prefs.end();
}

const String& cfg::wifi_ssid() { return g_cfg.wifi_ssid; }
const String& cfg::wifi_pass() { return g_cfg.wifi_pass; }
const String& cfg::ai_url() { return g_cfg.ai_url; }
const String& cfg::ai_key() { return g_cfg.ai_key; }
const String& cfg::ai_model() { return g_cfg.ai_model; }
uint32_t cfg::uart_baud() { return g_cfg.uart_baud; }

void cfg::set_wifi(const String& ssid, const String& pass) {
  g_cfg.wifi_ssid = ssid;
  g_cfg.wifi_pass = pass;
  Preferences prefs;
  prefs.begin(k_ns, false);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  prefs.end();
}

void cfg::set_ai(const String& url, const String& key, const String& model) {
  g_cfg.ai_url = url;
  g_cfg.ai_key = key;
  g_cfg.ai_model = model;
  Preferences prefs;
  prefs.begin(k_ns, false);
  prefs.putString("ai_url", url);
  prefs.putString("ai_key", key);
  prefs.putString("ai_model", model);
  prefs.end();
}
