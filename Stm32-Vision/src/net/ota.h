#pragma once
#include <Arduino.h>
#include <esp_http_server.h>

// OTA 升级（两条入口，共用同一套前置动作：打断 AI、停图传/图传推流、停四轮、停机械臂、
// 停运动验证、停 BLE 广播——升级期间一切抢射频/抢 flash 的活儿都让路）：
//   1) ArduinoOTA（UDP 3232 收邀请 + 回连 PC 现取固件）：Arduino IDE「工具→端口」里的
//      网络端口（形如 "VisionS3 at 192.168.x.x"），或 arduino-cli upload --protocol network。
//   2) HTTP POST /update（80 端口，请求体即裸 .bin）：浏览器/手机直接推固件，不需 IDE。
// 前提：分区表须有两个 app 槽（app0/app1，见 sketch 根目录 partitions.csv）——Update 写
// 「下一个」分区，成功才切换 boot 分区重启；失败则原固件照常运行（不会变砖）。
namespace ota {

void init();      // setup 中 net 之后调用一次：配置 ArduinoOTA 回调（真正 begin 推迟到联网后）
void update();    // loop 中调用：联网后自动起 ArduinoOTA 并驱动 handle()
void http_register(httpd_handle_t server);  // startCameraServer 中调用一次：注册 /update
bool active();    // 是否有 OTA 会话正在写固件：其它模块据此让路（MJPEG 推流退出、周期状态停推）

// 运行中固件的标记，形如 "v1 指纹 fef509d0"（静态缓冲，勿长期持有）。
// 出处与"为什么不用描述符里的 date/version"见 ota.cpp；指令回执里附带它，手机/PC 脚本
// 开日志时就能知道板上跑的是哪份固件，从而判断 OTA 到底生效没有。
const char* fw_stamp();

}  // namespace ota
