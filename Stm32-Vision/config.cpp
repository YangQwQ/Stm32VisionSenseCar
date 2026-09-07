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

// 归一化 AI 端点：常见坑是把 base_url（含 /v1 变体）当完整端点填，
// 缺 /chat/completions 会被直接 POST 到根路径或 /v1 → 返回 404。
// 规则：去掉尾部斜杠后，只要没以 /chat/completions 结尾就补全；已带完整端点则原样返回。
static String normalize_ai_url(const String& in) {
  String s = in;
  s.trim();
  while (s.length() > 0 && s.endsWith("/")) s = s.substring(0, s.length() - 1);
  if (s.length() > 0 && !s.endsWith("/chat/completions")) s += "/chat/completions";
  return s;
}

void cfg::init() {
  Preferences prefs;
  prefs.begin(k_ns, true);
  g_cfg.wifi_ssid = prefs.getString("wifi_ssid", "");
  g_cfg.wifi_pass = prefs.getString("wifi_pass", "");
  g_cfg.ai_url = normalize_ai_url(prefs.getString("ai_url", "https://api.deepseek.com/chat/completions"));
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

bool cfg::set_wifi(const String& ssid, const String& pass) {
  if (g_cfg.wifi_ssid == ssid && g_cfg.wifi_pass == pass) return false;  // 未变化，跳过写入
  g_cfg.wifi_ssid = ssid;
  g_cfg.wifi_pass = pass;
  Preferences prefs;
  prefs.begin(k_ns, false);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  prefs.end();
  return true;
}

bool cfg::set_ai(const String& url, const String& key, const String& model) {
  String u = normalize_ai_url(url);
  if (g_cfg.ai_url == u && g_cfg.ai_key == key && g_cfg.ai_model == model)
    return false;  // 未变化，跳过写入
  g_cfg.ai_url = u;
  g_cfg.ai_key = key;
  g_cfg.ai_model = model;
  Preferences prefs;
  prefs.begin(k_ns, false);
  prefs.putString("ai_url", u);
  prefs.putString("ai_key", key);
  prefs.putString("ai_model", model);
  prefs.end();
  // 写入即打印实际生效值，便于排查云端 404/401（key 只显示状态，不泄露）
  Serial.printf("[cfg] AI 配置写入：url=%s key=%s model=%s\n",
                u.c_str(), key.isEmpty() ? "空" : "已配置", model.c_str());
  return true;
}
