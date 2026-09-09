#pragma once
#include <Arduino.h>

// BLE 模块：GATT Server（配网 + 兜底控制 + 状态上报）。架构文档 §5.6。
// UUID 与手机端 Ctrl-App `net/ble/BleProfile.gd` 逐字 mirror——改一侧必须同步另一侧。
// 服务 0000C0DE-…，特征 0000C0E0~C0E6。广播名 VisionS3。
namespace ble {

void init();    // 开机调用一次：注册服务/特征，开始广播。不因未连手机而失败。
void update();  // loop 中调用：WiFi 状态变化时刷新 status 通知
void reply(const char* text);  // 经 status 特征把应答文本通知给已连手机（cmd::Reply 用）
void set_ws_connected(bool on);  // WS 客户端连上/断开时由 app_httpd 调，用于 status.ws
void set_transmission(bool on);  // 任一图传通道活跃时停广播（让 WiFi 独占射频）；全部安静后恢复

}  // namespace ble
