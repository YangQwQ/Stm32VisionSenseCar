#include "esp_camera.h"
#include "esp_log.h"
#include "src/net/config.h"
#include "src/net/wifi_net.h"
#include "src/cam/camera.h"
#include "src/exec/direct_exec.h"
#include "src/exec/motion_verify.h"
#include "src/core/command.h"
#include "src/net/ble.h"
#include "src/net/ota.h"
#include "src/ai/ai_client.h"
#include "src/core/board_log.h"
#include "src/core/heap_watch.h"

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// 摄像头型号（CAMERA_MODEL_*）与引脚（camera_pins.h）的配置点已上移到 camera.h——
// 各编译单元都经 camera.h 取得宏与引脚，此处是唯一配置点，不再在 .ino 重复定义。

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // 系统日志(log_i/log_w)默认也走 UART0，与 Arduino Serial 抢口会阻塞手动指令 ack 打印。
  // 静到 WARN 以上，只留错误；需排查置 INFO。
  esp_log_level_set("*", ESP_LOG_WARN);

  cfg::init();
  blog::init();  // 统一日志队列与转发任务（setup 早期拉起，供任意任务 logf 使用）
  // 内部堆水位哨兵：紧跟 blog 拉起，让"最低水位"覆盖整段运行——
  // WiFi RX 缓冲只能落 DMA 可达的内部 RAM，这块板的低点决定链路会不会哑（见 heap_watch.h）。
  // 起点越早，卡死后读到的低点越完整。放在 ble 之前，保证任何指令到达时它已就绪。
  hwatch::init();

  // BLE GATT Server 不依赖摄像头/WiFi——配网阶段无网可用，也要先能连上手机
  ble::init();

  exec::init();  // 直驱执行器：哪吒舵机回中 + 电机 0（不再经执行板）

  if (!cam::init()) {
    blog::logf(blog::CAM, "Camera init failed (继续：BLE 配网/控制仍可用)");
  } else {
// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
#endif
  }

  net::init();
  startCameraServer();

  ota::init();  // 固件升级入口（ArduinoOTA 网络端口 + HTTP /update）；联网后由 update() 自动就绪

  ai::init();  // AI worker 任务（DIRECT 链路；依赖 WiFi 与摄像头）
  mvfy::init();  // 运动到位验证任务（软解放核心0，避免阻塞 loop/WS；依赖摄像头）

  blog::logf(blog::SYS, "Ready!");
}

void loop() {
  // WiFi 断线重连由 network 模块处理
  net::update();
  ble::update();   // 处理 BLE cmd 队列 + WiFi 状态变化上报
  exec::update_tick();  // 直驱连续机械臂动作步进（约 10ms 一拍）
  ai::update();    // 排空 AI 结果队列（回传 Godot ai_result）
  ota::update();   // OTA：联网后起 ArduinoOTA 并驱动 handle()（升级期间此调用会阻塞）
  delay(10);
}
