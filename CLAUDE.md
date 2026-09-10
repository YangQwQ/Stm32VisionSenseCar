# CLAUDE.md（仓库根 · 总览与索引）

> 本文档基准：仓库 HEAD `9917330`（2026-09-10）。只覆盖已提交内容；未提交改动不收录。

本仓库是「视觉控制小车」协作工程的**容器仓库**，现收拢两块子工程：

| 目录 | 角色 | 硬件 / 技术栈 | 权威文档 |
|---|---|---|---|
| `Stm32-Vision/` | 视觉/控制大脑板**兼执行器**（采集画面→AI→**直驱**电机/舵机→上报状态） | ESP32-S3-CAM（N16R8 + OV3660）· Arduino（esp32 3.3.x） | [`Stm32-Vision/CLAUDE.md`](Stm32-Vision/CLAUDE.md) |
| `Mobile-RemoteCtrl/` | 手机遥控 App（图传/摇杆/指令/配网，词表源） | Android · Godot 4.7.1 mono + GDScript | [`Mobile-RemoteCtrl/CLAUDE.md`](Mobile-RemoteCtrl/CLAUDE.md) |

> ⚠️ **`Stm32-Executor/` 执行板已整体裁撤，不再是本仓库的一部分。** 因执行板硬件问题，2026-09 起改由大脑板经**软件 I2C 直驱**哪吒扩展板，串口帧链路（`AA 55 LEN DEV CMD PAYLOAD CRC16`）与大脑板 `uart` 模块一并删除。执行板代码只存在于 git 历史（`07aa659` 及之前），**勿按旧文档复原双板架构**。

**动手改某个子项目前，先读它自己的 `CLAUDE.md`**——每份都是该子工程的权威指南（目录职责、构建入口、调参宏、已知坑），本文件只做跨工程总览与索引，不重复细节。

## 一句话架构与数据流

手机经 BLE 给大脑板配网（广播名 `VisionS3`）；连上后走 WiFi，WebSocket（端口 81）承载指令/状态/消息（文本 JSON），JPEG 图传帧改走 UDP；大脑板把词表指令**就地翻译成哪吒扩展板的 I2C 命令**（舵机 PWM / 电机 PWM / 灯光字节）直接落地，状态由本板 `exec::read_state` 合成后回推——**全链路只有一块可编程板**。

- **大脑板既是视觉大脑也是执行器**：`command` 收到手动指令后直接调 `exec::act`，不再有中间板。
- **词表 JSON 只由手机 App 持有**（`net/proto/CommandProto.gd`）；`词表 → 哪吒 I2C 命令` 的翻译在大脑板 `command` + `direct_exec` 一侧。
- 当前状态：BLE 配网、WS 指令/状态、UDP 图传、软件 I2C 直驱（舵机/电机/灯）均已接通；板载 DIRECT AI（`ai_goal`/`ai_oneshot`）已实现并按任务闭环调用 `exec`；手机端云端 AI（`AIClient.gd`）仍为桩（DIRECT 不经过手机侧）。

## 各子工程文件索引（简）

### Stm32-Vision/ — 视觉大脑板兼执行器（Arduino sketch，源文件全在根目录）

初始化序：cfg→ble→**exec**→cam→net→web→ai。详见其 [CLAUDE.md](Stm32-Vision/CLAUDE.md) 的「代码库结构」表。

| 文件 | 作用 |
|---|---|
| `Stm32-Vision.ino` | 入口（setup 按序初始化，loop 调各模块 update + `exec::update_tick`） |
| `camera(.h/.cpp)` `camera_pins.h` | 摄像头初始化 + JPEG 双缓冲抓帧；引脚定义唯一配置点在 `camera.h` 顶部选 `CAMERA_MODEL_*` |
| `camera_index.h` | Web 前端页面（源自例程，现基本不用） |
| `config(.h/.cpp)` | WiFi / AI 接口参数，NVS 持久化 |
| `wifi_net(.h/.cpp)` | STA 连接 + 断线重连（namespace `net`，勿改回 `network`） |
| `direct_exec(.h/.cpp)` | **执行器直驱层**：move/stop/arm/light/reset 落地到哪吒板；机械臂二连杆 IK；本地合成状态文本 |
| `nezha_direct(.h/.cpp)` | 哪吒扩展板软 I2C 驱动（舵机 / 电机 / 灯），协议与硬件一致 |
| `command(.h/.cpp)` | 统一词表 JSON 分发（与传输解耦）；`ai_goal` 触发 `ai::set_goal` 闭环 |
| `ping_svc(.h/.cpp)` | `/ping <目标>` 异步 ICMP 探测（无目标则就地回 pong） |
| `ble(.h/.cpp)` | BLE GATT Server：配网 + 兜底控制 + status 通知（广播名 VisionS3） |
| `app_httpd.cpp` | HTTP（MJPEG/拍照/LED）+ WS（端口 81 文本 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报 |
| `ai_client(.h/.cpp)` | 板载多模态 AI（DIRECT 直调云端，任务级闭环：move/arm/stop/wait、双帧运动感知、PSRAM 分配 + keep-alive TLS） |
| `partitions.csv` | 分区表（3MB APP，需 `huge_app`） |

### Mobile-RemoteCtrl/ — 手机遥控 App（Godot 工程）

通信：BLE 配网/兜底控制、WiFi WS 指令/状态（文本 JSON）+ UDP 图传（连接策略统一收口 `net/DeviceConn.gd`）、云端 AI 桩；`CommandProto` 为唯一命令词表。详见其 [CLAUDE.md](Mobile-RemoteCtrl/CLAUDE.md) 的「目录结构」。

| 路径 | 内容 |
|---|---|
| `Main.tscn/.gd` | App 壳（连接编排、摇杆映射、图传开关） |
| `ui/chat/ChatPanel.gd` | 聊天/指令区（消息日志、指令提示、附件列表、指令解析） |
| `state/AppState.gd` (+LocalStore) | autoload 全局状态 + send_command 统一出口 |
| `net/proto/CommandProto.gd` | **统一命令词表**（static） |
| `net/DeviceConn.gd` | **统一连接层**：自建并持有 BLE/WS/UDP，收敛状态与重连策略（单一事实源；Main/AppState 只订阅其信号） |
| `net/ws/WSCarClient.gd` | WS 传输（端口 81 文本 JSON：指令/状态；视频已走 UDP） |
| `net/ble/BLEClient.gd` + `BleProfile.gd` | BLE GATT 客户端；协议常量表（与大脑板 ble.cpp **逐字 mirror**） |
| `net/video/UDPVideoClient.gd` | UDP 图传接收（JPEG 分片重组 → 上抛 frame_received） |
| `net/ai/AIClient.gd` | 云端 AI 桩（DIRECT 不经手机侧） |
| `ui/` | 摇杆 / 图传 / 图片标注 / 配网弹窗 / 直控面板 |
| `addons/gdble*` | GDBLE 蓝牙运行时（含导出插件） |

## 硬件基线（BOM 速查）

完整清单见根 [`README.md`](README.md) §3。要点：

- **大脑板**：ESP32-S3-CAM（N16R8）+ **OV3660** 摄像头；既跑视觉/网络/AI，也直接输出哪吒 I2C 命令。
- **驱动板**：哪吒（NeZha）扩展板，I2C 从机地址 `0x80`；**大脑板软件 I2C 直连**（SCL=GPIO47 / SDA=GPIO14），无 MCU 中转。
- **执行机构**：4× N20 直流电机（四轮，**无编码器**）+ 4× MG90S 舵机（Servo1 转向 / Servo2 移爪 / Servo3 夹爪 / Servo4 抬落）。
- ⚠️ 因电机无编码器，**直驱下没有里程计**：`distance_cm` / `angle_deg` 等定距定角参数被忽略（仅 `arm` 的 `dist_cm` 按拍数近似）。

## 跨子工程同步点（铁律）

改协议/常量前**必须**先读两份子 CLAUDE.md 的「协议参考」/「通信协议速查」，并同步相关侧：

1. **词表 JSON**（type / params 字段）：只由手机 `net/proto/CommandProto.gd` 定义 ↔ 大脑板 `command.cpp` 的 `type` 分支 + `direct_exec.cpp` 的 params 解析，**两侧逐字对应**。改一侧必改另一侧。
2. **BLE UUID / 广播名（VisionS3）**：手机 `net/ble/BleProfile.gd` ↔ 大脑板 `ble.cpp`，逐字 mirror。
3. **哪吒 I2C 命令表**（从机 `0x80`、舵机/电机/灯光 cmd 字节）：现只有大脑板 `nezha_direct.cpp` 一处实现，无对侧；改动须对照哪吒扩展板硬件协议，别单方面改字节。
4. 各子 CLAUDE.md 中还有各自的坑（如大脑板 `namespace net` 勿改回 `network`、esp32 勿回退 2.x / 勿用 esp32cam 目标等），改动前读。

## 外部引用提醒

子 CLAUDE.md 里引用了**本仓库外**的原始工作区路径（`D:\Downloads\Git\Ctrl-App`、`D:\Downloads\Git\vision-control-architecture.md` 等）。在本容器内：

- 手机端 = 本仓库的 `Mobile-RemoteCtrl/`（即外部路径里的 `Ctrl-App`）；
- 架构文档（`vision-control-architecture.md`）**不在本仓库内**，如缺失且需要，找作者或按两份子 CLAUDE.md 的协议节反推。注意该文档早于执行板裁撤，其中的 UART 帧协议节已失效。

## 仓库级约定

- 本仓库不配置顶层构建；编译/烧录入口分散：大脑板走 Arduino IDE/arduino-cli，App 走 Godot headless 导出。**不主动跑编译/烧录验证**（耗时无谓），默认交给用户在其 IDE 中做。
- Git：代码提交由用户操作（全局规则），助手可查看与撤回，若用户要求提交，请在提交消息中注意区分涉及部分，如(Vision/Mobile)，具体可见历史提交。推送时若发现需要先pull，尽量尝试使用git pull --rebase
- 文档维护：每份 `CLAUDE.md` 顶部标注「本文档基准：仓库 HEAD `<短hash>`（日期）」= 该文档对应的代码基准；更新文档前先基于该 hash `git diff` 检查，规则见全局 CLAUDE「CLAUDE.md 维护约定」
