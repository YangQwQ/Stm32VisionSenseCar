# CLAUDE.md

> 本文档基准：仓库 HEAD `9917330`（2026-09-10）。只覆盖已提交内容；未提交改动不收录。

本文件为当前项目（ESP32-S3-CAM / OV3660 摄像头板）的 AI 助手工作指南。

## 项目概述

基于 **ESP32-S3-CAM（N16R8，带 PSRAM）** 的视觉控制板。板子采集画面，交给多模态大模型（可在配网时配置 URL/Key/模型）识别，生成小车与机械臂的控制指令，**经软件 I2C 直驱哪吒扩展板**落地。

### 工作模式

1. **WiFi 直连 AI 模式**：板子经 WiFi 直调多模态 AI 接口，上传画面 → 解析返回控制指令 → **本板直接执行**（`exec::act`）。✅ 已实现（DIRECT：`ai_goal` → `ai_client` 迭代闭环，真机联调中）。
2. **蓝牙配置模式**：BLE（GATT Server，广播名 VisionS3）接收配置（WiFi 账号密码 / AI 接口等）→ 存 NVS → **在线重建 STA 生效，不整板重启**（BLE 保活，手机无需重连）。✅ 已实现。**生命周期**：手机在 WS 连上后断开本机 BLE 让出射频（WS_ONLY）；广播开关由 `ble::set_transmission` 统一控制——仅当**真在推帧**（UDP 图传 / MJPEG）时停广播让 WiFi 独占射频；WS 指令/状态通道在位时不关广播，保持可发现（手机随时可重连/兜底，`onDisconnect` 尊重该状态）。
3. **手机中转模式（已弃用）**：App 中转调 AI 的 RELAY 已在手机端移除（2026-09）。现行 **DIRECT**：手机只下发 `ai_goal` 目标文本/区域 → 板子 `ai_client` 直调云端 AI 执行闭环并回 `ai_result`；手机端云端 AI（`AIClient.gd`）仍为桩（DIRECT 不经手机侧）。

### 执行板（已裁撤）

**本板现在既是视觉/控制大脑，也是执行器。** 原独立的 STM32 执行板因硬件问题已整体移除：`uart.cpp/.h`、串口帧协议（`AA 55 LEN DEV CMD PAYLOAD CRC16`）、`exec_forward` 等入口全部删除；执行动作改由 `direct_exec`（namespace `exec`）经 `nezha_direct`（namespace `nezha`）软件 I2C 直接下发哪吒扩展板。**不要再按旧文档恢复 UART 双板链路。**

## 代码库结构

源自 Espressif 官方 `CameraWebServer` 例程，已按功能模块拆分重构（文件均在 sketch 根目录）：

| 文件 | 作用 |
| --- | --- |
| `Stm32-Vision.ino` | 入口：setup 按 cfg→ble→exec→cam→net→web→ai 初始化；loop 调各模块 update + `exec::update_tick()` |
| `camera.h/.cpp` | 摄像头初始化 + 抓帧（JPEG 双缓冲，同步取帧 `cam::grab()`）。**型号唯一配置点**：在 `camera.h` 顶部选 `CAMERA_MODEL_*` 并包含 `camera_pins.h`；画质参数（分辨率/JPEG 质量）集中在 `camera.cpp` `init()` 顶部，推流帧率在 `app_httpd.cpp`（`WS_STREAM_FPS`，图传走 UDP） |
| `camera_pins.h` | 各摄像头型号 GPIO 引脚定义（按 `CAMERA_MODEL_*` 分支） |
| `camera_index.h` | Web 前端页面（HTML/JS 内嵌数组，源自例程，现基本不用） |
| `config.h/.cpp` | WiFi / AI 接口配置，NVS 持久化（不再写死 ssid/password）。⚠️ 遗留：`cfg::uart_baud()` 键仍在但已无调用方（UART 链路裁撤后的残留） |
| `wifi_net.h/.cpp` | STA 连接 + 断线重连 + 在线换网 `net::reconnect`（namespace `net`）。⚠️ 勿改回 `network`：会与核心库 `Network.h` 在 Windows 大小写不敏感 FS 上遮蔽冲突 |
| `nezha_direct.h/.cpp` | 哪吒扩展板软 I2C 直驱底座：`set_servo(ch,pwm)` / `set_motor(ch,a,b)` / `led(kind,on)`；从机 `0x80`，协议与哪吒硬件一致 |
| `direct_exec.h/.cpp` | **执行层**（替代原执行板）：`act()` 分发 move/stop/arm/light/reset、连续机械臂动作步进 `update_tick()`、二连杆 IK `arm_pose()`、本地合成状态 `read_state()`、持续型判定 `is_continuous()` |
| `command.h/.cpp` | 统一词表 JSON 分发（与传输解耦、回调应答）；`apply_network` 在线换网生效；`ai_goal`→`ai::set_goal` 触发板载 AI；`exec_log` 开关控制状态推送 |
| `ai_client.h/.cpp` | 板载多模态 AI HTTP 调用（DIRECT 直调云端，任务级闭环：move/arm/stop/wait、双帧运动感知、PSRAM 分配 + keep-alive TLS），动作最终走 `exec::act` |
| `ping_svc.h/.cpp` | `/ping <目标>` 异步 ICMP echo（esp_ping），结果经 cmd 回复通道回报；无目标仍由 command 就地回 `pong` |
| `ble.h/.cpp` | BLE GATT Server：配网 + 兜底控制 + status 通知；广播开关随 `set_transmission`（真在推帧即停）（UUID 见下「协议参考」）|
| `app_httpd.cpp` | HTTP（MJPEG / 拍照 / LED 灯）+ WS（端口 81：文本=指令/状态 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报（默认关，`exec_log` 开启后约 400ms 一条）。人脸检测/识别已停用（宏置 0） |
| `partitions.csv` | 分区表：app0 约 3.8MB，需选带 3MB+ APP 空间的开发板分区选项 |

## 直驱执行层要点（`exec` / `nezha`）

- **接线**：哪吒 SCL ← GPIO47，SDA ← GPIO14；I2C 速率 ≤200kHz，软 I2C 开漏实现（原执行板的 PB6/PB7 须脱离总线）。
- **物理映射**：四轮 M1左后 / M2右后 / M3右前 / M4左前（左轮 `a` 正前、右轮 `b` 正前）；舵机 Servo1 转向 / Servo2 移爪 / Servo3 夹爪 / Servo4 抬落。
- **标定限位**（`direct_exec.cpp` 顶部宏，实测）：转向 150/120/180；移爪中位 200（120..250）；夹爪紧 50 / 松 140；抬落中位 180（115..250）。
- **连续动作**：`lift_up/down`、`reach_forward/backward` 为持续型——`update_tick()` 每拍按 `ARM_STEP_CM`(0.25cm) 沿目标轴步进并保持另一维（reach 保持高度 h、lift 保持 x），经 `arm_pose` 反解联动双舵机下发；到可达域边界/机械限位（末端无实际移动）自动停，收到 `stop(scope="arm")` 或离散动作时清除；`clip/release` 为离散置端。
- **二连杆 IK**：`arm_pose(x,h)`（x=轴前方 cm，h=地面以上 cm）反解 α/β 后查标定表联动左右两舵机；L1=L2=7.5cm、肩轴离地 9.5cm、可达半径 4..15cm。
- **无里程计**：电机无编码器，`move` 的 `distance_cm` / `angle_deg` 被忽略（`is_continuous` 判据里保留这两个字段，仅用于区分是否配 `stop`）；`arm` 的 `dist_cm` 按每 cm ≈ `ARM_CNT_PER_CM` 拍近似。
- **手动指令优先**：`command` 收到 move/stop/arm 先 `ai::cancel(...)` 打断 AI 闭环再 `exec::act`（arm 打断只停轮子）。

## 构建要点

- 框架：Arduino（`esp32` 板支持包）。当前板为 **ESP32-S3-CAM（N16R8，带 PSRAM）**，摄像头选型在 `camera.h` 顶部 `CAMERA_MODEL_ESP32S3_EYE`；IDE / 命令行目标一律 `esp32:esp32s3:esp32s3`+`16M flash / opi psram / huge_app`。⚠️ 曾误用经典 `esp32:esp32:esp32cam`（AI-Thinker）目标，编出固件无法用于本板，勿回退。
- 当前在 **esp32 core 3.3.11** 下全量编译链接通过。核心 API 已按 3.x 适配，**勿回退 2.x**：
  - LEDC 引脚式：`ledcAttach(pin, freq, res)` / `ledcWrite(pin, duty)`（`ledcSetup/ledcAttachPin` 及 channel 式调用已移除）
  - BLE：`getValue()` 返回 Arduino `String`；无 `getNotifyProperty`；发射功率枚举为 `ESP_PWR_LVL_P9`
  - WS 无 `httpd_ws_client_iterate` → 用 `httpd_get_client_list` + `httpd_ws_get_fd_info` 过滤 `HTTPD_WS_CLIENT_WEBSOCKET`
- 命令行验证
  当前已核准的 IDE 板子配置的对应 fqbn：
  `arduino-cli compile --fqbn "esp32:esp32s3:esp32s3:FlashSize=16M,FlashMode=dio,PartitionScheme=huge_app,DebugLevel=debug,PSRAM=opi,EraseFlash=none" .`
  （arduino-cli 位于 `D:\Program Files\Arduino IDE\resources\app\lib\backend\resources`，即 IDE 内置同版、缓存目录同源。若某次 IDE 把 DebugLevel 调回 none/其它，命令行同步改回，避免不共享缓存。）
- ⚠️ mbedTLS 握手内存优化依赖自定义核心库：`Arduino15\packages\esp32\tools\esp32s3-libs\3.3.11` 已按 IDF `defconfig` 重编替换，含三处配置：SSLin/out 缓冲 `CONFIG_MBEDTLS_SSL_IN/OUT_CONTENT_LEN=8192`（规避内部堆碎片导致的握手失败 `-32512`/`-17040`）；`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`（定义 `MBEDTLS_PLATFORM_MEMORY`，激活 `ai::init()` 的 `mbedtls_platform_set_calloc_free` PSRAM hook，把 TLS 缓冲搬去 PSRAM，规避内部堆碎片导致的 AI 请求 `-3`）；lwIP TCP 缓冲 32768/16384。替换时勿用官方同名库覆盖。重编流程与配置见 [`../.trae/README.md`](../.trae/README.md)。链接补丁 `sections.ld` 亦在该目录下，**升级核心版本后需重打**。
- ⚠️ 实测：即使 fqbn 完全一致，**IDE 验证 ↔ 命令行切换仍常各自全量重编**（esp32 core 整包重编，5–10 分钟）；想省时间就让「主编译入口」固定在一侧，别频繁来回。**改/增/删源文件后首次编译若报多定义或「多个文件 -o」错，删 `C:\Users\Yang\AppData\Local\arduino\sketches\` 下本 sketch 缓存目录再编**。
- 依赖库：**ArduinoJson v7（Benoit Blanchon）**，装在用户 sketchbook `D:\Documents\Arduino\libraries\ArduinoJson`。⚠️ sketch 内 `libraries/ArduinoJson` 子目录 **Arduino 不会自动扫描**，属冗余副本，勿依赖（可删）。
- 分区：**PartitionScheme=huge_app**（3MB APP、无 OTA），已含在上方完整 fqbn 内；真机烧录同此方案。依赖库与编译缓存目录同 IDE（`%LOCALAPPDATA%\arduino\sketches\<sketch哈希>\`）。

## 实现进度（2026-09-10）

目标板为 **ESP32-S3-CAM**（esp32:esp32s3:esp32s3 + 16M flash / opi psram + huge_app）。OV3660 摄像头与图传真机联调正常：

- `camera` ✅ 真机验证：OV3660 识别正常（默认倒置/饱和偏高已在 init 回正），JPEG 抓帧与 UDP 图传工作
- `config` / `wifi_net` / `ble` ✅ 真机可用：BLE 配网、图传链路已联调
- `nezha` / `direct_exec` ✅ 真机可用：软件 I2C 直驱四轮（前后/转向）、机械臂三舵机、三路灯光；机械臂标定完成
- `command` / `app_httpd`（web_server）✅ 编译通过，词表分发与 WS/UDP 链路接通
- `ai_client`（板载多模态 AI HTTP 调用）✅ 已实现（DIRECT 直调云端，任务级闭环；动作走 `exec`），真机联调中

> 待联调项：机械臂 `dist_cm` 的拍数近似（`ARM_CNT_PER_CM`）未实测校准。

## 协议参考

统一架构与完整协议定义见仓库根 [`../CLAUDE.md`](../CLAUDE.md) 与手机端 [`../Mobile-RemoteCtrl/net/proto/CommandProto.gd`](../Mobile-RemoteCtrl/net/proto/CommandProto.gd)。**该词表由手机端定义，本板逐字对齐解析**（`command.cpp` 的 `type` 分支 / `direct_exec.cpp` 的 params）。

| type | params | 落点 |
| --- | --- | --- |
| `move` | `throttle`(-1..1) `steering`(-1..1) | 四轮 PWM（×1000）/ 转向舵 150±30 |
| `stop` | `scope` = all/wheels/arm | 停轮 或 清连续机械臂动作 |
| `arm` | `act` = lift_up/lift_down/reach_forward/reach_backward/clip/release、可选 `dist_cm` | 舵机 2/3/4 |
| `light` | `kind` = front/vibe/back，`on` | 哪吒灯命令字节 |
| `reset` | — | 四舵机回中 + 电机 0 |
| `servo` / `motor` / `drive` / `arm_pose` | 见 `command.cpp` | 调试直驱（绕过上层语义） |
| `stream` | `on`、可选 `udp_port` `src_ip` | 图传开关（UDP 目标由板子据此建立） |
| `exec_log` | `on` | 状态周期推送开关（默认关） |
| `config` | `ssid` `password` | NVS + 在线换网 |
| `ping` | 可选 `target` | 无目标回 `pong`；有目标走 `ping_svc` |
| `ai_goal` / `ai_oneshot` / `ai_cancel` | `message`、可选 `annotation` `use_image` | `ai_client` DIRECT 闭环 |

BLE UUID / 广播名与手机 `../Mobile-RemoteCtrl/net/ble/BleProfile.gd` **逐字 mirror**：改一侧必须同步另一侧（服务 `0000C0DE-…`，特征 `C0E0`~`C0E6`，广播名 `VisionS3`）。

哪吒 I2C 命令表现只有本板 `nezha_direct.cpp` 一处实现，**无对侧可 mirror**：改字节前须对照哪吒扩展板硬件协议。

## 关联项目

- 手机 App：`../Mobile-RemoteCtrl`（显示画面 / 指令编辑 / 下发 `ai_goal` / BLE 配网）。

## 约定

- 与用户交流使用中文。
- 编译验证：用户未明确要求时，**不主动跑 arduino-cli 编译验证**（esp32 单次 ~80s+ 起步、IDE↔命令行互切会各自全量重编，耗时无谓）；日常编译/烧录验证默认交给用户在 IDE 里做。确需命令行核对时，用「构建要点」里与 IDE 逐字一致的同一 fqbn。
- 指令协议、注释保持简洁；避免在注释里写死具体数值（参数调整时容易忘改）。
- **串口日志卫生**：手动指令只在类型切换时打一行「收到手动指令」，避免摇杆高频帧刷屏并阻塞控制时序；IDF 系统日志 `esp_log_level_set("*", ESP_LOG_WARN)` 默认静到 WARN，避免与手动指令 ack 争用同一 UART0。
- 大模型返回的指令必须严格校验后再执行，防止异常 JSON 导致小车误动作。
- 硬件标定值（舵机限位、IK 几何）集中在 `direct_exec.cpp` 顶部宏，改动前确认已实测。
