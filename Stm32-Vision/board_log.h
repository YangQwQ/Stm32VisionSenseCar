#pragma once
#include <Arduino.h>

// 统一板端日志模块（namespace blog）：所有串口调试输出统一经 logf() 产出（带来源标记），
// 并按手机端开启的类别路由——仅在对应类别或全局 all 开启时，把该行打包成
// {type:"log",params:{src,text}} 经注册的转发器送给手机（WS 广播 + BLE 通知）。
// 串口始终打印；手机转发默认全关（避免刷屏）。转发在专用任务线程统一排空，线程安全，
// 任意任务（AI worker / ws_stream / 主 loop）调用 logf 都安全。
namespace blog {

enum Cat : uint8_t {          // 日志来源类别
  EXEC,   // 直驱执行/状态（含周期状态行）
  AI,     // 板载 AI 客户端（原 ai_log）
  NET,    // WiFi/网络
  CAM,    // 摄像头
  WS,     // WebSocket/UDP 图传
  CMD,    // 指令分发
  BLE,    // 蓝牙/配网
  SYS,    // 系统级
  CAT_MAX,
};

// 是否转发该类别（手机开 → true）：全局 all 开启或该类别单独开启。
bool enabled(Cat c);

// 设置某类别开关 / 全局 all 开关（由 /log 指令调用）。
void enable(Cat c, bool on);
void set_all(bool on);

// 统一日志输出：始终写串口（带 [类] 前缀）；enabled(c) 时同步转发手机。
void logf(Cat c, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// 转发器：由 app_httpd 启动时注册（WS 广播 + BLE status 通知）。在转发任务线程执行。
typedef void (*SendFn)(const char* json);
void set_forwarder(SendFn fn);

// 启动日志队列与转发任务（setup 早期调用一次；set_forwarder 可在 web 初始化时注册）。
void init();

}  // namespace blog