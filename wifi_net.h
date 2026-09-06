#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// 网络模块：STA 连接（读取 config）+ 断线自动重连。
namespace net {

void init();    // 开机调用一次：若已配置则发起连接（非阻塞）
void update();  // loop 中调用：断线后周期重连

bool is_connected();
IPAddress local_ip();

}  // namespace net
