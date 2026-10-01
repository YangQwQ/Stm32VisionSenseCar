#include "esp_camera.h"
#include "esp_log.h"
#include "src/net/config.h"
#include "src/net/wifi_net.h"
#include "src/cam/camera.h"
#include "src/exec/direct_exec.h"
#include "src/core/command.h"
#include "src/net/ble.h"
#include "src/net/ota.h"
#include "src/ai/ai_client.h"
#include "src/ai/grasp.h"      // 本地自动夹取（标位置 → 自己对准/前进/合爪，不经 AI）
#include "src/core/board_log.h"
#include "src/core/heap_watch.h"
#include <freertos/FreeRTOS.h>   // vTaskPrioritySet：抬 Arduino loop 任务优先级（见 setup 末尾）
#include <freertos/task.h>

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
  grasp::init();  // 本地自动夹取 worker（纯画面闭环，不依赖 AI）

  // ⚠️ 抬 Arduino loop 任务的优先级（默认 1，core 1）。
  // loop() 里跑 exec::update_tick()，它负责两件**对时序敏感**的事：① 定距/定角动作"到点停轮"；
  // ② 机械臂离散定位的 S 形缓动推进。而同在 core 1 的图传软编任务优先级 5、夹取搜索任务优先级 3，
  // 都会抢占默认 1 的 loop —— 于是短脉冲被拉长（真机实测：命令转 8°、实际转出 30~90°，一步就把
  // 目标甩出搜索窗而跟丢）、缓动被补成大跨度（观感"一卡一卡"）。抬到 6（高于图传的 5）后，
  // update_tick 按 ~10ms 稳定推进；core 1 上其余任务(blog1/hwatch1/ping2/ai2)本就低于它。
  vTaskPrioritySet(NULL, 6);

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
