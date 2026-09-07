#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// UART 模块：大脑板 ↔ 执行板（STM32）帧协议（架构文档 §5.4/§5.5）+ 词表→UART 翻译层（§5.3）。
// 帧格式: AA 55 LEN DEV CMD [PAYLOAD] CRC16
//   LEN   = DEV+CMD+PAYLOAD 字节数
//   DEV   = 0x01 小车子系统 / 0x02 机械臂子系统
//   CRC16 = LEN..PAYLOAD 的 CRC16（MODBUS poly 0x8005 反射版，低字节在前）——多项式以执行板固件联调为准。
// 波特率可配（cfg::uart_baud，NVS）；实例 Serial2（引脚待执行板接线确认）。
namespace uart {

void init();   // 开机调用一次：按配置波特率启动 Serial2
void update(); // loop 中调用：清空 RX，解析并丢弃/记录不完整帧（执行板状态帧解析骨架）
void send_raw(uint8_t dev, uint8_t cmd, const uint8_t* payload, size_t len);

// §5.3 词表 → 帧 翻译入口（move/stop/arm 专用）。返回是否成功组帧发出。
// 由 command 模块在收到 move/stop/arm 时调用。
bool act(const char* type, const JsonObjectConst& params);

// 词表指令是否为「持续型」（执行板会一直动作，直到收到后续 stop/新指令才停）。
// 判定与 act 内部同源；ai 模块用它决定任务终结时是否补发兜底 stop。
bool is_continuous(const char* type, const JsonObjectConst& params);

// 读取最近一条执行板状态帧（0x0A），拼成一行文本（供 AI 上下文拼接）。
// 无状态数据（执行板未接入/未上报）返回 false，buf 不变。
bool read_state(char* buf, size_t cap);

// 执行板上行帧镜像：开发调试用，把收到的执行板帧广播给手机 App（app_httpd 注册 ForwardCb）。
typedef void (*ForwardCb)(const char* json);
void set_forward(bool on);                    // 开关：收到执行板帧时是否转发给手机
void set_forward_cb(ForwardCb cb);            // 注册广播回调（向全部 WS 客户端发文本）

}  // namespace uart
