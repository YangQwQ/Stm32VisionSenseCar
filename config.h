#pragma once
#include <Arduino.h>

// 配置模块：NVS 持久化 + 内存缓存，未配置的键回退默认值。
namespace cfg {

void init();  // 开机调用一次：从 NVS 载入全部配置

const String& wifi_ssid();
const String& wifi_pass();
const String& ai_url();
const String& ai_key();
const String& ai_model();
uint32_t uart_baud();

void set_wifi(const String& ssid, const String& pass);
void set_ai(const String& url, const String& key, const String& model);

}  // namespace cfg
