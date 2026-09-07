# CLAUDE.md

本文件为当前项目（ESP32-S3-CAM / OV2640 摄像头板）的 AI 助手工作指南。

## 项目概述

基于经典 ESP32-CAM（AI-Thinker 板，ESP32 + OV2640，带 PSRAM）的视觉控制板。板子采集画面，交给多模态大模型（DeepSeek V4 Flash）识别，生成小车与机械臂的控制指令，通过串口（UART）转发给对应的执行板。

### 工作模式

1. **WiFi 直连 AI 模式**：板子经 WiFi 直调多模态 AI 接口，上传画面 → 解析返回控制指令 → UART 转发执行板。⚠️ 板载 AI 客户端（`ai_client`）尚未实现。
2. **蓝牙配置模式**：BLE（GATT Server，广播名 VisionS3）接收配置（WiFi 账号密码 / AI 接口等）→ 存 NVS → 重启生效。✅ 已实现。
3. **手机中转模式（已弃用）**：App 中转调 AI 的 RELAY 已在手机端移除（2026-09）。现行 **DIRECT**：手机只下发 `ai_goal` 目标文本/区域 → 板子执行并回 status；云端 AI 两侧现均为桩。

### 执行板

小车/机械臂执行板通过 UART 与当前板通信。注意：当前板是「视觉/控制大脑」，不是执行板。

## 代码库结构

源自 Espressif 官方 `CameraWebServer` 例程，已按功能模块拆分重构（文件均在 sketch 根目录）：

| 文件 | 作用 |
| --- | --- |
| `Stm32-Vision.ino` | 入口：setup 按 cfg→ble→uart→cam→net→web 初始化；loop 调各模块 update |
| `camera.h/.cpp` | 摄像头初始化 + 抓帧（JPEG 双缓冲，同步取帧 `cam::grab()`）。**型号唯一配置点**：在 `camera.h` 顶部选 `CAMERA_MODEL_*` 并包含 `camera_pins.h`，别处不再重复定义 |
| `camera_pins.h` | 各摄像头型号 GPIO 引脚定义（按 `CAMERA_MODEL_*` 分支） |
| `camera_index.h` | Web 前端页面（HTML/JS 内嵌数组，源自例程，现基本不用） |
| `config.h/.cpp` | WiFi / AI 接口 / `uart_baud` 参数配置，NVS 持久化（不再写死 ssid/password） |
| `wifi_net.h/.cpp` | STA 连接 + 断线重连（namespace `net`）。⚠️ 勿改回 `network`：会与核心库 `Network.h` 在 Windows 大小写不敏感 FS 上遮蔽冲突 |
| `uart.h/.cpp` | 执行板串口帧协议（`AA 55 LEN DEV CMD PAYLOAD CRC16`）+ 词表→帧翻译 |
| `command.h/.cpp` | 统一词表 JSON 分发（与传输解耦、回调应答）；`ai_goal` 现为桩回复 |
| `ble.h/.cpp` | BLE GATT Server：配网 + 兜底控制 + status 通知（UUID 见下「协议参考」） |
| `app_httpd.cpp` | HTTP（MJPEG / 拍照 / LED 灯）+ WS（端口 81：文本=指令 JSON、二进制=JPEG），已接入 `command`/`ble`。人脸检测/识别已停用（宏置 0） |
| `partitions.csv` | 分区表：app0 约 3.8MB，需选带 3MB+ APP 空间的开发板分区选项 |

## 构建要点

- 框架：Arduino（`esp32` 板支持包）。芯片为经典 **ESP32-CAM**（AI-Thinker，带 PSRAM），IDE / 命令行目标一律 `esp32:esp32:esp32cam`（⚠️ 曾误标为 ESP32-S3：`esp32s3` 目标编出的固件无法用于本板，勿用）。
- 当前在 **esp32 core 3.3.11** 下全量编译链接通过。核心 API 已按 3.x 适配，**勿回退 2.x**：
  - LEDC 引脚式：`ledcAttach(pin, freq, res)` / `ledcWrite(pin, duty)`（`ledcSetup/ledcAttachPin` 及 channel 式调用已移除）
  - BLE：`getValue()` 返回 Arduino `String`；无 `getNotifyProperty`；发射功率枚举为 `ESP_PWR_LVL_P9`
  - WS 无 `httpd_ws_client_iterate` → 用 `httpd_get_client_list` + `httpd_ws_get_fd_info` 过滤 `HTTPD_WS_CLIENT_WEBSOCKET`
- 命令行验证（**须与 IDE 板子菜单选项逐字一致**，不同则缓存不共享、来回全量重编）：
  `arduino-cli compile --fqbn "esp32:esp32s3:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app,DebugLevel=none,EraseFlash=none" .`
  （arduino-cli 位于 `D:\Program Files\Arduino IDE\resources\app\lib\backend\resources`，即 IDE 内置同版、缓存目录同源。）
- ⚠️ 实测：即使 fqbn 完全一致，**IDE 验证 ↔ 命令行切换仍常各自全量重编**（esp32 core 整包重编，5–10 分钟）；想省时间就让「主编译入口」固定在一侧，别频繁来回。**改/增/删源文件后首次编译若报多定义或「多个文件 -o」错，删 `C:\Users\Yang\AppData\Local\arduino\sketches\` 下本 sketch 缓存目录再编**。
- 依赖库：**ArduinoJson v7（Benoit Blanchon）**，装在用户 sketchbook `D:\Documents\Arduino\libraries\ArduinoJson`。⚠️ sketch 内 `libraries/ArduinoJson` 子目录 **Arduino 不会自动扫描**，属冗余副本，勿依赖（可删）。
- 分区：**PartitionScheme=huge_app**（3MB APP、无 OTA），已含在上方完整 fqbn 内；真机烧录同此方案。依赖库与编译缓存目录同 IDE（`%LOCALAPPDATA%\arduino\sketches\<sketch哈希>\`）。

## 实现进度（2026-09-06）

各模块已接线并在核心 3.3.11 **编译通过**（此前 Phase B「仅代码复核、未编译」的历史问题已全部解决）。目标板为 **ESP32-S3-CAM**（esp32:esp32s3:esp32s3 + 16M flash / opi psram + huge_app；此前误用的 esp32cam 固件已弃用，勿烧）：

- `camera` / `config` / `wifi_net` ✅ 编译通过
- `uart` / `command` / `ble` / `app_httpd`（web_server）✅ 编译通过（帧/词表语义以架构文档为准）
- `ai_client`（板载多模态 AI HTTP 调用）❌ **未做**；`ai_goal` 仍为 command.cpp 桩回复

> ⚠️ 仍未真机联调（代码就绪、未上硬件）。待联调项：
> ① BLE 配网后板重启，手机需重连一次 BLE 收 ip；
> ② arm 词表 `duration_ms` 按 `dist_cm` 判定（UI 现发 0）；
> ③ 执行板 UART 帧语义（CRC16/CMD 表）以 STM32 固件为准。

## 协议参考

统一架构与完整协议定义（词表 JSON、UART 指令表、BLE GATT UUID）见 `D:\Downloads\Git\vision-control-architecture.md`，开发前必读。
BLE UUID / 广播名与手机 `Ctrl-App/net/ble/BleProfile.gd` **逐字 mirror**：改一侧必须同步另一侧。

## 关联项目

- 手机 App：`D:\Downloads\Git\Ctrl-App`（显示画面 / 指令编辑 / 下发 `ai_goal` / BLE 配网）。

## 约定

- 与用户交流使用中文。
- 编译验证：用户未明确要求时，**不主动跑 arduino-cli 编译验证**（esp32 单次 ~80s+ 起步、IDE↔命令行互切会各自全量重编，耗时无谓）；日常编译/烧录验证默认交给用户在 IDE 里做。确需命令行核对时，用「构建要点」里与 IDE 逐字一致的同一 fqbn。
- 指令协议、注释保持简洁；避免在注释里写死具体数值（参数调整时容易忘改）。
- 大模型返回的指令必须严格校验后再转发，防止异常 JSON 导致执行板误动作。
