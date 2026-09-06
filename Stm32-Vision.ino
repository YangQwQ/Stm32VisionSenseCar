#include "esp_camera.h"
#include "config.h"
#include "wifi_net.h"
#include "camera.h"
#include "uart.h"
#include "command.h"
#include "ble.h"

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

  cfg::init();

  // BLE GATT Server 不依赖摄像头/WiFi——配网阶段无网可用，也要先能连上手机
  ble::init();

  uart::init();  // Serial2 → 执行板（STM32）

  if (!cam::init()) {
    Serial.println("Camera init failed (继续：BLE 配网/控制仍可用)");
  } else {
// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
#endif
  }

  net::init();
  startCameraServer();

  Serial.println("Ready!");
}

void loop() {
  // WiFi 断线重连由 network 模块处理
  net::update();
  ble::update();   // 处理 BLE cmd 队列 + WiFi 状态变化上报
  cmd::update();   // 延迟重启（配网生效）
  uart::update();  // 收执行板状态帧（骨架）
  delay(10);
}
