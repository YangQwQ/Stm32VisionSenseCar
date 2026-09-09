# CLAUDE.md（仓库根 · 总览与索引）

> 本文档基准：仓库 HEAD `07aa659`（2026-09-09）。只覆盖已提交内容；未提交改动不收录。

本仓库是「视觉控制小车」协作工程的**容器仓库**，把三块独立开发的子工程收拢到同一仓库：

| 目录 | 角色 | 硬件 / 技术栈 | 权威文档 |
|---|---|---|---|
| `Stm32-Vision/` | 视觉/控制大脑板（采集画面→AI→转发指令→解析状态） | ESP32-S3-CAM（N16R8，现装 OV3660）· Arduino（esp32 3.3.x） | [`Stm32-Vision/CLAUDE.md`](Stm32-Vision/CLAUDE.md) |
| `Stm32-Executor/` | 执行板（只执行：驱动阿克曼小车 + 机械臂，回传状态） | STM32F103C8T6 · Keil MDK + 标准外设库 | [`Stm32-Executor/CLAUDE.md`](Stm32-Executor/CLAUDE.md) |
| `Mobile-RemoteCtrl/` | 手机遥控 App（图传/摇杆/指令/配网，词表源） | Android · Godot 4.7.1 mono + GDScript | [`Mobile-RemoteCtrl/CLAUDE.md`](Mobile-RemoteCtrl/CLAUDE.md) |

**动手改某个子项目前，先读它自己的 `CLAUDE.md`**——每份都是该子工程的权威指南（目录职责、构建入口、调参宏、已知坑），本文件只做跨工程总览与索引，不重复细节。

## 一句话架构与数据流

手机经 BLE 给大脑板配网（广播名 `VisionS3`）；连上后走 WiFi，WebSocket（端口 81）承载指令/状态/消息（文本 JSON），JPEG 图传帧改走 UDP；大脑板把指令翻译成 UART 帧发给执行板；执行板驱动电机/舵机并周期回传状态帧，大脑板（或 App）据此闭环。

- **大脑板是「视觉/控制大脑」，不是执行板**；执行板只认帧、**不经手词表 JSON**。
- **词表 JSON 只由手机 App 持有**；`词表 → UART 帧` 的翻译在大脑板 `Stm32-Vision/uart` 一侧。
- 当前状态：大脑板 `ai_client` 已实现 DIRECT 板载 AI（板子直调云端多模态模型，任务级迭代闭环），`ai_goal` 真实触发；BLE 配网、WS 指令/UDP 图传、UART 帧链路均已接通；手机端云端 AI（`AIClient.gd`）仍为桩（DIRECT 不经过手机侧）。

## 各子工程文件索引（简）

### Stm32-Vision/ — 视觉大脑板（Arduino sketch，源文件全在根目录）

初始化序：cfg→ble→uart→cam→net→web。详见其 [CLAUDE.md](Stm32-Vision/CLAUDE.md) 的「代码库结构」表。

| 文件 | 作用 |
|---|---|
| `Stm32-Vision.ino` | 入口（setup 按序初始化，loop 调各模块 update） |
| `camera(.h/.cpp)` `camera_pins.h` | 摄像头初始化 + JPEG 双缓冲抓帧；引脚定义唯一配置点在 `camera.h` 顶部选 `CAMERA_MODEL_*` |
| `camera_index.h` | Web 前端页面（源自例程，现基本不用） |
| `config(.h/.cpp)` | WiFi / AI 接口 / `uart_baud` 参数，NVS 持久化 |
| `wifi_net(.h/.cpp)` | STA 连接 + 断线重连（namespace `net`，勿改回 `network`） |
| `uart(.h/.cpp)` | 执行板串口帧协议（`AA 55 LEN DEV CMD PAYLOAD CRC16`）+ 词表→帧翻译 |
| `command(.h/.cpp)` | 统一词表 JSON 分发（与传输解耦）；`ai_goal` 触发 `ai::set_goal` 闭环 |
| `ble(.h/.cpp)` | BLE GATT Server：配网 + 兜底控制 + status 通知（广播名 VisionS3） |
| `app_httpd.cpp` | HTTP（MJPEG/拍照/LED）+ WS（端口 81 文本 JSON）+ UDP 图传帧推送；已接入 command/ble |
| `ai_client(.h/.cpp)` | 板载多模态 AI（DIRECT 直调云端，任务级闭环：move/arm/stop/wait、双帧运动感知、PSRAM 分配 + keep-alive TLS） |
| `partitions.csv` | 分区表（3MB APP，需 `huge_app`） |

### Stm32-Executor/ — 执行板（Keil 工程）

业务逻辑集中在 `Control/`（日常改动集中地）。详见其 [CLAUDE.md](Stm32-Executor/CLAUDE.md) 的「代码结构」与「主循环调度」。

| 目录 | 内容 |
|---|---|
| `User/main.c` | 5ms 主调度（收包/译码/闭环/状态上报时间表） |
| `Control/` | **协议+执行层**：UartFrame(帧/CRC)、Dispatch(译码分发)、AckermannDrive(阿克曼+速度闭环)、Odom(里程计)、Relay(到位判定)、RobotArmAct(机械臂步进)、Status(状态帧) |
| `Hardware/` | 板级驱动：Usart1(115200+环形缓冲)、Board_Timer(TIM2 5ms)、LED、Vehicle_Chassis(底盘+转向舵机)、RobotArm(3 舵机)、ps2(自测用) |
| `NeZha/` | 哪吒扩展板软 I2C 驱动：4 电机 PWM、4 编码器、4 舵机 PWM、灯带 |
| `Library/` `Start/` `System/` | STM32F10x 标准外设库 + CMSIS（不动） |
| `project.uvprojx` | Keil 唯一工程入口（AC5；加文件时同步） |
| `stlink-1.8.0-win32/` | 内嵌 st-flash 命令行烧录工具 |

### Mobile-RemoteCtrl/ — 手机遥控 App（Godot 工程）

通信：BLE 配网/兜底控制、WiFi WS 指令/状态（文本 JSON）+ UDP 图传（连接策略统一收口 `net/DeviceConn.gd`）、云端 AI 桩；`CommandProto` 为唯一命令词表。详见其 [CLAUDE.md](Mobile-RemoteCtrl/CLAUDE.md) 的「目录结构」。

| 路径 | 内容 |
|---|---|
| `Main.tscn/.gd` | App 壳（统一聊天/指令入口、连接状态） |
| `state/AppState.gd` (+LocalStore) | autoload 全局状态 + send_command 统一出口 |
| `net/proto/CommandProto.gd` | **统一命令词表**（static） |
| `net/DeviceConn.gd` | **统一连接层**：自建并持有 BLE/WS/UDP，收敛状态与重连策略（单一事实源；Main/AppState 只订阅其信号） |
| `net/ws/WSCarClient.gd` | WS 传输（端口 81 文本 JSON：指令/状态/snapshot 单帧；视频已走 UDP） |
| `net/ble/BLEClient.gd` + `BleProfile.gd` | BLE GATT 客户端；协议常量表（与大脑板 ble.cpp **逐字 mirror**） |
| `net/video/UDPVideoClient.gd` | UDP 图传接收（JPEG 分片重组 → 上抛 frame_received） |
| `net/ai/AIClient.gd` | 云端 AI 桩（DIRECT 不经手机侧） |
| `ui/` | 摇杆 / 图传 / 图片标注 / 配网弹窗 |
| `addons/gdble*` | GDBLE 蓝牙运行时（含导出插件） |

## 跨子工程同步点（铁律）

改协议/常量前**必须**先读三份子 CLAUDE.md 的「协议参考」/「通信协议速查」，并同步相关侧：

1. **UART 帧协议（帧格式 / CRC-16-MODBUS / DEV / CMD / 状态字段 / flag）**：执行板 `Control/`（实现基准）↔ 大脑板 `Stm32-Vision/uart`。改一侧必改另一侧。
2. **BLE UUID / 广播名（VisionS3）**：手机 `net/ble/BleProfile.gd` ↔ 大脑板 `ble.cpp`，逐字 mirror。
3. **词表 JSON**：只由手机 `CommandProto.gd` 定义；大脑板 `command/uart` 翻译成帧；执行板只认帧。
4. 各子 CLAUDE.md 中还有各自的坑（如大脑板 `namespace net` 勿改回 `network`、esp32 勿回退 2.x / 勿用 esp32cam 目标等），改动前读。

## 外部引用提醒

三份子 CLAUDE.md 里引用了**本仓库外**的原始工作区路径（`D:\Downloads\Git\Ctrl-App`、`D:\Downloads\Git\vision-control-architecture.md`、`D:\Downloads\Git\CLAUDE.md` 等）。在本容器内：

- 手机端 = 本仓库的 `Mobile-RemoteCtrl/`（即外部路径里的 `Ctrl-App`）；
- 架构文档（`vision-control-architecture.md`）**不在本仓库内**，如缺失且需要，找作者或按三份子 CLAUDE.md 的协议节反推。

## 仓库级约定

- 本仓库不配置顶层构建；编译/烧录入口分散：大脑板走 Arduino IDE/arduino-cli，执行板走 Keil MDK，App 走 Godot headless 导出。**不主动跑编译/烧录验证**（耗时无谓），默认交给用户在其 IDE 中做。
- Git：代码提交由用户操作（全局规则），助手可查看与撤回，若用户要求提交，请在提交消息中注意区分涉及部分，如(Vision/Mobile/Executor)，具体可见历史提交。推送时若发现需要先pull，尽量尝试使用git pull --rebase
- 文档维护：每份 `CLAUDE.md` 顶部标注「本文档基准：仓库 HEAD `<短hash>`（日期）」= 该文档对应的代码基准；更新文档前先基于该 hash `git diff` 检查，规则见全局 CLAUDE「CLAUDE.md 维护约定」
