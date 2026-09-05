# CLAUDE.md

本文件为当前项目（ESP32-S3 + OV2640 摄像头板）的 AI 助手工作指南。

## 项目概述

基于 ESP32-S3 + OV2640 的视觉控制板。板子采集画面，交给多模态大模型（DeepSeek V4 Flash）识别，生成小车与机械臂的控制指令，通过串口（UART）转发给对应的执行板。

### 三种工作模式

1. **WiFi 直连 AI 模式**：板子通过 WiFi 直接调用多模态 AI 接口，上传画面 → 解析返回的控制指令 → UART 转发给小车/机械臂执行板。
2. **蓝牙配置模式**：板子通过蓝牙（BLE）连接手机，接收配置指令（WiFi 账号密码、AI 接口配置等），存于 NVS。
3. **手机中转模式**：板子通过 WiFi 把画面（MJPEG 流）发给手机 App，App 编辑画面后自行调用 AI 识别并返回指令，指令再通过 WiFi 回传板子 → UART 转发执行。此模式下手机起中转作用，AI 调用在手机端完成。

### 执行板

小车执行板和机械臂执行板通过 UART 与当前板通信（注意：当前板是"视觉/控制大脑"，不是执行板）。

## 当前代码库（商家提供例程）

这是 Espressif 官方 `CameraWebServer` 例程，仅作摄像头 + WiFi + Web 服务器的起点：

| 文件                    | 作用                                            |
| --------------------- | --------------------------------------------- |
| `CameraWebServer.ino` | 入口：摄像头初始化、WiFi 连接、启动 Web 服务器                  |
| `app_httpd.cpp`       | HTTP 服务器：MJPEG 视频流、拍照、人脸检测/识别、LED 灯控制         |
| `camera_index.h`      | Web 前端页面（HTML/JS，内嵌为头文件数组）                    |
| `camera_pins.h`       | 各摄像头型号的 GPIO 引脚定义（按 `CAMERA_MODEL_*` 宏选择）     |
| `partitions.csv`      | Flash 分区表（app0 约 3.8MB，需选带 3MB+ APP 空间的开发板选项） |

### 构建要点

- 框架：Arduino（`esp32` 板支持包），芯片目标 ESP32-S3。

- 板子需带 PSRAM（OV2640 高分辨率 + JPEG 必需）。

- 必须选择分区方案：带 `3MB APP` 空间的（与 `partitions.csv` 匹配）。

- 依赖库：ArduinoJson v7（已放在 sketch 的 `libraries/` 子目录，Arduino IDE 自动识别；若用 VSCode/PlatformIO 需另行引入）。

- 当前 `CameraWebServer.ino` 中 `CAMERA_MODEL_AI_THINKER` 被启用；`ssid/password` 为写死的测试值，后续应改为 NVS 配置 + 蓝牙配网。

## 后续开发计划（待实现）

目标：在例程基础上改造成上面三种模式的完整固件。计划模块如下（实现时按此拆分文件）：

- `camera`：摄像头初始化、抓帧（JPEG，双缓冲 + GRAB\_LATEST，同步取帧接口 `cam::grab()`）✅ 已完成

- `config`：WiFi / AI 接口 / 系统参数配置，NVS 持久化 ✅ 已完成

- `network`：WiFi STA/AP 管理、断线重连 ✅ 已完成（STA 连接 + 断线重连；AP 配网热点待接）

- `ai_client`：多模态 AI HTTP 调用（图片上传 + 指令 JSON 解析）

- `command`：指令协议定义、校验、命令队列

- `uart`：与执行板（同控小车+机械臂）的串口通信（帧协议 + 校验）

- `ble`：BLE 配网 / 配置通道

- `web_server`：MJPEG 流 + 手机中转指令接收接口（复用例程的 app\_httpd）

统一架构与完整协议定义（词表 JSON、UART 指令表、BLE GATT）见 `D:\Downloads\Git\vision-control-architecture.md`，开发前必读。

## 关联项目

- 手机 App：`D:\Downloads\Git\Ctrl-App`（显示画面、编辑、中转调用 AI、BLE 配网）。

## 约定

- 与用户交流使用中文。

- 指令协议、注释保持简洁；避免在注释里写死具体数值（参数调整时容易忘改）。

- 大模型返回的指令必须做严格校验后再转发，防止异常 JSON 导致执行板误动作。

