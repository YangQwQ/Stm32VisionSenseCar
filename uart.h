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

}  // namespace uart
