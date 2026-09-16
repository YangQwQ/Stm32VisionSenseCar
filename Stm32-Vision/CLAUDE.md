# CLAUDE.md

> 本文档基准：仓库 HEAD `5bb3b63`（2026-09-16）。只覆盖已提交内容；未提交改动不收录。

本文件为当前项目（ESP32-S3-CAM / OV3660 摄像头板）的 AI 助手工作指南。

## 项目概述

基于 **ESP32-S3-CAM（N16R8，带 PSRAM）** 的视觉控制板。板子采集画面，交给多模态大模型（可在配网时配置 URL/Key/模型）识别，生成小车与机械臂的控制指令，**经软件 I2C 直驱哪吒扩展板**落地。

### 工作模式

1. **WiFi 直连 AI 模式**：板子经 WiFi 直调多模态 AI 接口，上传画面 → 解析返回控制指令 → **本板直接执行**（`exec::act`）。✅ 已实现（DIRECT：`ai_goal` → `ai_client` 迭代闭环，真机联调中）。AI 侧另维护**画面标定（单应）+ 物体空间记忆 + 车姿态累积**并把结果换算成当前车头局部系喂回模型；是否携带上一帧做运动对比由模型用 `carry_prev` 自行决定。云端持续无有效响应会逐轮退避、超限即中止任务并回报手机。
2. **蓝牙配置模式**：BLE（GATT Server，广播名 VisionS3）接收配置（WiFi 账号密码 / AI 接口等）→ 存 NVS → **在线重建 STA 生效，不整板重启**（BLE 保活，手机无需重连）。✅ 已实现。**生命周期**：手机在 WS 连上后断开本机 BLE 让出射频（WS_ONLY）；广播开关由 `ble::set_transmission` 统一控制——仅当**真在推帧**（UDP 图传 / MJPEG）时停广播让 WiFi 独占射频；WS 指令/状态通道在位时不关广播，保持可发现（手机随时可重连/兜底，`onDisconnect` 尊重该状态）。

> 本板既是视觉/控制大脑，也是执行器：动作由 `direct_exec`（namespace `exec`）经 `nezha_direct`（namespace `nezha`）软件 I2C 直接下发哪吒扩展板。

## 代码库结构

源自 Espressif 官方 `CameraWebServer` 例程，已按功能模块拆分。**源码全部位于 `src/` 下按模块分的子目录**（`Stm32-Vision.ino`、`partitions.csv`、`build_opt.h` 仍在 sketch 根目录）。源码内的本项目头文件 include 一律以 `src/` 为根的相对路径（如 `#include "net/wifi_net.h"`）书写——Arduino 只把 `src/`（不含其子目录）加入 include path，子目录会递归编译但不进搜索路径，因此不要改回无前缀形式：

| 文件 | 作用 |
| --- | --- |
| `Stm32-Vision.ino` | 入口：setup 按 cfg→ble→exec→cam→net→web→ai 初始化；loop 调各模块 update + `exec::update_tick()` |
| `src/cam/camera.h/.cpp` | 摄像头初始化 + 抓帧（JPEG 双缓冲，同步取帧 `cam::grab()`；抓帧模式 `CAMERA_GRAB_WHEN_EMPTY`——图传与 AI 是两个并发取帧方，用 `LATEST` 会与之抢缓冲导致取帧卡死）。**型号唯一配置点**：在 `camera.h` 顶部选 `CAMERA_MODEL_*` 并包含 `camera_pins.h`；画质参数（分辨率/JPEG 质量）与画面回正（`vflip`/`hmirror`，使所见即真实方位）集中在 `camera.cpp` `init()` 顶部，推流帧率在 `net/app_httpd.cpp`（`WS_STREAM_FPS`，图传走 UDP） |
| `src/cam/camera_pins.h` | 各摄像头型号 GPIO 引脚定义（按 `CAMERA_MODEL_*` 分支） |
| `src/cam/camera_index.h` | Web 前端页面（HTML/JS 内嵌数组，源自例程，现基本不用） |
| `src/net/config.h/.cpp` | WiFi / AI 接口配置，NVS 持久化（不再写死 ssid/password） |
| `src/net/wifi_net.h/.cpp` | STA 连接 + 断线重连 + 在线换网 `net::reconnect`（namespace `net`）。⚠️ 勿改回 `network`：会与核心库 `Network.h` 在 Windows 大小写不敏感 FS 上遮蔽冲突 |
| `src/exec/nezha_direct.h/.cpp` | 哪吒扩展板软 I2C 直驱底座：`set_servo(ch,pwm)` / `set_motor(ch,a,b)` / `led(kind,on)`；从机 `0x80`，协议与哪吒硬件一致 |
| `src/exec/direct_exec.h/.cpp` | **执行层**：`act()` 分发 move/stop/arm/arm_pose/light/reset/spin、连续机械臂动作步进 `update_tick()`（兼管定距/定角到点自停）、离散定位走 S 形缓动 `advance_smooth()`（`ARM_SMOOTH_PEAK`，目标值与实际写入值分离）、二连杆 IK `arm_pose()`、本地合成状态 `read_state()`（含末端前/高与爪限位）、持续型判定 `is_continuous()` |
| `src/exec/bivar.h/.cpp` | 机械臂"夹心坐标"双向散点反距离加权(IDW)插值：FK 用 pwm 距、IK 用 x/h 距；散点表 `kArmPts` 在 Calibration.h，`arm_set()` 实现在 Calibration.cpp |
| `src/core/command.h/.cpp` | 统一词表 JSON 分发（与传输解耦、回调应答）；手动指令先 `ai::cancel` 打断 AI 闭环再落地；`apply_network` 在线换网生效；`ai_goal`→`ai::set_goal`、`goto`→`ai::goto_target`、`ai_chat`→`ai::append_chat`；`log` 统一日志指令走 `blog` 开关；`get_state` 回灯/夹爪/AI busy 位图 `params.bits`；`nz_read` 走 `nezha::probe` I2C 在线探测 |
| `src/core/board_log.h/.cpp` | 统一日志模块（namespace `blog`）：`logf(cat,...)` 统一串口调试输出（带 `[类]` 前缀），按 `/log` 开关（exec/ai/all）把 `{type:"log",params:{src,text}}` 入队，由转发任务（栈 8192）经 app_httpd 注册的转发器（WS 广播+BLE status）发手机 |
| `src/ai/ai_client.h/.cpp` | 板载多模态 AI HTTP 调用（DIRECT 直调云端，任务级闭环：move/stop/arm/spin/arm_pose/wait/approach（快速接近），动作最终走 `exec::act`；任务进度以 `tasks` 列表每轮渲染回喂、前几轮思考按 `AI_HIST_N` 环缓存多轮喂回）：HTTPClient + keep-alive TLS 复用与失败重试、PSRAM 缓冲；物体空间记忆 + 车姿态累积，换算到车头局部系后喂回模型（屏幕→地面换算走 `ground_proj`）；默认单帧，仅当上轮 `carry_prev:true` 才附带上一帧做运动对比；WS 文本出口统一过 `sanitize_ws_utf8` 消毒（云端偶发残缺 UTF-8，原样进 TEXT 帧会让手机端以 `1007` 断链）；发往云端的长字符串按 UTF-8 边界截断（截半个中文字节会被判 400）；服务端持续无有效响应则逐轮退避，超限中止任务并回报手机 |
| `src/ai/ground_proj.h/.cpp` | 屏幕→地面坐标换算（namespace `ground`）：实测标定点拟合单应，`ground::screen_to_world(px,py,&x,&y)`；标定点 `kGroundCal` 在 Calibration.h，加测点改那里 |
| `src/net/ping_svc.h/.cpp` | `/ping <目标>` 异步 ICMP echo（esp_ping），结果经 cmd 回复通道回报；无目标仍由 command 就地回 `pong` |
| `src/net/ble.h/.cpp` | BLE GATT Server：配网 + 兜底控制 + status 通知；广播开关随 `set_transmission`（真在推帧即停）（UUID 见下「协议参考」）|
| `src/net/app_httpd.cpp` | HTTP（MJPEG / 拍照 / LED 灯）+ WS（端口 81：文本=指令/状态 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报（默认关，`/log exec on` 开启后约 400ms 一条；状态缓冲须容下含抓手前端的整行，改状态行时同步核对）。注册 `blog` 日志转发器（WS+BLE）。`ws_stream` 任务栈 8192（推流 + 状态上报共用）。人脸检测/识别已停用（宏置 0） |
| `Calibration.h/.cpp` | **手动校准数据集中区**（根目录）：舵机限位/机械臂参数（STEER/REACH/GRIP/LIFT、ARM_*）、小车定距/定角移动时长表（MV_*/SPIN_MSDEG_*/SPIN_MIN_SPEED/ARM_CNT_PER_CM）、屏幕→地面单应标定点 `kGroundCal`、机械臂夹心散点 `kArmPts`；`bivar::arm_set()` 实现在 Calibration.cpp |
| `partitions.csv` | 分区表：app0 约 3.8MB，需选带 3MB+ APP 空间的开发板分区选项 |

## 直驱执行层要点（`exec` / `nezha`）

- **接线**：哪吒 SCL ← GPIO47，SDA ← GPIO14；I2C 速率 ≤200kHz，软 I2C 开漏实现。
- **物理映射**：四轮 M1左后 / M2右后 / M3右前 / M4左前（左轮 `a` 正前、右轮 `b` 正前）；舵机 Servo1 转向 / Servo2 移爪 / Servo3 夹爪 / Servo4 抬落。
- **标定限位**（`Calibration.h`，实测）：转向 150/120/180；移爪中位 200（120..250）；夹爪紧 50 / 松 140；抬落中位 180（115..250）。
- **连续动作**：`lift_up/down`、`reach_forward/backward` 为持续型——`update_tick()` 每拍按 `ARM_STEP_CM`(0.5cm) 沿目标轴步进并保持另一维（reach 保持高度 h、lift 保持 x），经 `arm_pose` 反解联动双舵机下发；到可达域边界/机械限位（末端无实际移动）自动停，收到 `stop(scope="arm")` 或离散动作时清除；`clip/release` 为离散置端，`fold` 收臂折叠回平台位。
- **二连杆 IK**：`arm_pose(x,h)`（x=轴前方 cm，h=地面以上 cm）反解 α/β 后查标定表联动左右两舵机；L1=L2=7.5cm、肩轴离地 9.5cm、可达半径 4..15cm。
- **无里程计（电机无编码器）**：定距/定角不做闭环，一律按**实测标定表的时长近似**到点自停——`move` 带 `distance_cm` 时按 `MV_SPEED_X/Y`（油门→cm/s）与 `MV_COAST_X/Y`（起停余量）换算时长（需 `cm>0` 且油门非零）；`spin` 带 `angle_deg` 时按 `SPIN_MSDEG_X/Y` 换算；`arm` 的 `dist_cm` 按每 cm ≈ `ARM_CNT_PER_CM` 拍近似。不带定距/定角即为持续动作，靠 `stop` 收尾；AI 侧对无 `distance_cm` 的持续 move 另设 `AI_MOVE_CAP_MS` 上限兜底。
- **原地旋转**：`spin` 的 `dir` = `+1` 右转（顺时针，左轮进/右轮退）/ `-1` 左转 / `0` 停（显式写 0 速度）；`dir≠0` 时转速低于 `SPIN_MIN_SPEED` 会被抬高（低于实测拖动线拖不动）。`move` 与 `stop` 都会清掉旋转状态。
- **状态行**：`read_state()` 合成 `小车:<速度> <动作> | 抓手:前Xcm 高Ycm 爪:<合/开/中><限位提示>`，末端位置由正运动学算出。
- **手动指令优先**：`move/stop/arm/drive/spin/servo/motor/arm_pose/reset` 视为手动接管，先 `ai::cancel` 再落地；`arm`/`arm_pose` 只停轮子（`StopMode::Wheels`），其余类型用户指令已覆盖故不补停。手动路径同时清掉上一轮遗留的定距/定角时限，避免旧时限在新指令之后误停。

## 构建要点

- 框架：Arduino（`esp32` 板支持包）。当前板为 **ESP32-S3-CAM（N16R8，带 PSRAM）**，摄像头选型在 `camera.h` 顶部 `CAMERA_MODEL_ESP32S3_EYE`；IDE / 命令行目标一律 `esp32:esp32s3:esp32s3`+`16M flash / opi psram / huge_app`。⚠️ 曾误用经典 `esp32:esp32:esp32cam`（AI-Thinker）目标，编出固件无法用于本板，勿回退。
- 当前在 **esp32 core 3.3.11** 下全量编译链接通过。核心 API 已按 3.x 适配，**勿回退 2.x**：
  - LEDC 引脚式：`ledcAttach(pin, freq, res)` / `ledcWrite(pin, duty)`（勿用 `ledcSetup`/`ledcAttachPin` 等 channel 式调用）
  - BLE：`getValue()` 返回 Arduino `String`；无 `getNotifyProperty`；发射功率枚举为 `ESP_PWR_LVL_P9`
  - WS 无 `httpd_ws_client_iterate` → 用 `httpd_get_client_list` + `httpd_ws_get_fd_info` 过滤 `HTTPD_WS_CLIENT_WEBSOCKET`
- 命令行验证
  当前已核准的 IDE 板子配置的对应 fqbn：
  `arduino-cli compile --fqbn "esp32:esp32s3:esp32s3:FlashSize=16M,FlashMode=dio,PartitionScheme=huge_app,DebugLevel=debug,PSRAM=opi,EraseFlash=none" .`
  （arduino-cli 位于 `D:\Program Files\Arduino IDE\resources\app\lib\backend\resources`，即 IDE 内置同版、缓存目录同源。若某次 IDE 把 DebugLevel 调回 none/其它，命令行同步改回，避免不共享缓存。）
- ⚠️ mbedTLS 握手内存优化依赖自定义核心库：`Arduino15\packages\esp32\tools\esp32s3-libs\3.3.11` 已按 IDF `defconfig` 重编替换，含三处配置：SSLin/out 缓冲 `CONFIG_MBEDTLS_SSL_IN/OUT_CONTENT_LEN=8192`（规避内部堆碎片导致的握手失败 `-32512`/`-17040`）；`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`（mbedTLS 缓冲改走 PSRAM，规避内部堆碎片导致的 AI 请求 `-3`）；lwIP TCP 缓冲 32768/16384。替换时勿用官方同名库覆盖。重编流程与配置见 `.trae/README.md`，链接补丁 `sections.ld` 亦在该目录下，**升级核心版本后需重打**。
  - ⚠️ 该目录 `.trae/` 已列入 `.gitignore`、不在版本库内（属仓库外资料，缺失时找作者）。
  - ⚠️ `ai_client.cpp` 里的 `ai_tls_calloc` / `ai_tls_free` 是同一思路的**预留实现，当前并未接线**：全仓库没有任何 `mbedtls_platform_set_calloc_free` 调用点，两者不会被 mbedTLS 使用。上面那条 PSRAM 效果来自核心库的 `EXTERNAL_MEM_ALLOC` 配置本身；改这两个函数不影响 TLS 分配。
- ⚠️ 实测：即使 fqbn 完全一致，**IDE 验证 ↔ 命令行切换仍常各自全量重编**（esp32 core 整包重编，5–10 分钟）；想省时间就让「主编译入口」固定在一侧，别频繁来回。**改/增/删源文件后首次编译若报多定义或「多个文件 -o」错，删 `C:\Users\Yang\AppData\Local\arduino\sketches\` 下本 sketch 缓存目录再编**。
- 依赖库：**ArduinoJson v7（Benoit Blanchon）**，装在用户 sketchbook `D:\Documents\Arduino\libraries\ArduinoJson`。⚠️ sketch 内 `libraries/ArduinoJson` 子目录 **Arduino 不会自动扫描**，属冗余副本，勿依赖（可删）。
- 分区：**PartitionScheme=huge_app**（3MB APP、无 OTA），已含在上方完整 fqbn 内；真机烧录同此方案。依赖库与编译缓存目录同 IDE（`%LOCALAPPDATA%\arduino\sketches\<sketch哈希>\`）。

## 实现进度（2026-09-16）

目标板为 **ESP32-S3-CAM**（esp32:esp32s3:esp32s3 + 16M flash / opi psram + huge_app）。OV3660 摄像头与图传真机联调正常：

- `camera` ✅ 真机验证：OV3660 识别正常（默认倒置/镜像已在 init 回正），JPEG 抓帧与 UDP 图传工作；图传与 AI 并发取帧用 `CAMERA_GRAB_WHEN_EMPTY`
- `config` / `wifi_net` / `ble` ✅ 真机可用：BLE 配网、图传链路已联调
- `nezha` / `direct_exec` ✅ 真机可用：软件 I2C 直驱四轮（前后/转向/**原地旋转**）、机械臂三舵机、三路灯光；机械臂标定完成
- `command` / `app_httpd`（web_server）✅ 编译通过，词表分发与 WS/UDP 链路接通
- `ai_client`（板载多模态 AI HTTP 调用）✅ 已实现（DIRECT 直调云端，任务级闭环；动作走 `exec`），真机联调中：物体空间记忆、按需携带上一帧、任务列表（`tasks`）、`goto` 指定点导航与 `approach` 快速接近均已接入（屏幕→地面单应换算在 `ground_proj`）

> 待实测校准：机械臂 `dist_cm` 的拍数近似（`ARM_CNT_PER_CM`）、以及 `move`/`spin` 定距定角的时长表（`MV_*` / `SPIN_MSDEG_*`）。

## 协议参考

统一架构与完整协议定义见仓库根 [`../CLAUDE.md`](../CLAUDE.md) 与手机端 [`../Mobile-RemoteCtrl/net/proto/CommandProto.gd`](../Mobile-RemoteCtrl/net/proto/CommandProto.gd)。**该词表由手机端定义，本板逐字对齐解析**（`command.cpp` 的 `type` 分支 / `direct_exec.cpp` 的 params）。

| type | params | 落点 |
| --- | --- | --- |
| `move` | `throttle`(-1..1) `steering`(-1..1)、可选 `distance_cm` | 四轮 PWM（×1000）/ 转向舵 150±30；带 `distance_cm` 按标定表换算时长到点自停 |
| `stop` | `scope` = all/wheels/arm | 停轮 或 清连续机械臂动作 |
| `spin` | `dir` = `+1`右转/`-1`左转/`0`停、`speed`(0..1000)、可选 `angle_deg` | 左右轮反向 PWM（原地旋转）；带 `angle_deg` 到时自停 |
| `arm` | `act` = lift_up/lift_down/reach_forward/reach_backward/clip/release/**fold**、可选 `dist_cm` | 舵机 2/3/4（fold = 收臂折叠回平台位） |
| `light` | `kind` = front/vibe/back，`on` | 哪吒灯命令字节 |
| `reset` | — | 四舵机回中 + 电机 0 |
| `servo` / `motor` / `drive` / `arm_pose` | `servo`:`n`(0..3) `pwm`(50..250，不过标定限位)；`motor`:`n`(1..4) `a` `b`；`drive`:`speed`；`arm_pose`:`x`车头前 cm `h`离地 cm | 调试直驱（绕过上层语义） |
| `stream` | `on`、可选 `udp_port` `src_ip` | 图传开关（UDP 目标由板子据此建立） |
| `log` | `cat`=exec/ai/all，`on` | 统一日志转发开关（默认全关；exec=执行日志+周期状态推送、ai=AI 调试日志、all=板端串口全部输出转发手机；经 `blog` 统一队列 `{type:"log",params:{src,text}}` 上抛）。替代原 `exec_log`/`ai_log` |
| `get_state` | — | 回 `{"type":"state","params":{"bits":…}}` 位图（灯光/夹爪/AI busy，与手机 `Main.gd` 逐位 mirror；重连后同步按钮用） |
| `nz_read` | — | `nezha::probe` 探测哪吒板 I2C 在线/ACK，结果以 status 文本回 |
| `pong` | — | 手机 WS 活体探测的应答（板端静默处理） |
| `config` | `ssid` `password` | NVS + 在线换网 |
| `ping` | 可选 `target` | 无目标回 `pong`；有目标走 `ping_svc` |
| `ai_goal` / `ai_oneshot` / `ai_cancel` | `message`、可选 `annotation` `use_image` | `ai_client` DIRECT 闭环 |
| `ai_chat` | `message` | 任务进行中插话补充（`ai::append_chat`，不打断闭环）；当前无 AI 任务则忽略 |
| `goto` | `x` `y`、可选 `frame`=local(默认)/global | `ai::goto_target`：由板端自行导航到指定坐标 |

BLE UUID / 广播名与手机 `../Mobile-RemoteCtrl/net/ble/BleProfile.gd` **逐字 mirror**：改一侧必须同步另一侧（服务 `0000C0DE-…`，特征 `C0E0`~`C0E6`，广播名 `VisionS3`）。

哪吒 I2C 命令表现只有本板 `nezha_direct.cpp` 一处实现，**无对侧可 mirror**：改字节前须对照哪吒扩展板硬件协议。

⚠️ **板内命名不一致（HEAD 已核）**：`ai_client.cpp` 的系统提示词教 AI 用 `act:"home"`（收臂折叠），而 `validate_cmd` 的 `acts[]` 白名单只有 `fold`、无 `home`——AI 若照提示词输出 `home` 会被判非法并被迫重试。对外词表统一以 `fold` 为准。

## 关联项目

- 手机 App：`../Mobile-RemoteCtrl`（显示画面 / 指令编辑 / 下发 `ai_goal` / BLE 配网）。

## 约定

- 与用户交流使用中文。
- 编译验证：用户未明确要求时，**不主动跑 arduino-cli 编译验证**（esp32 单次 ~80s+ 起步、IDE↔命令行互切会各自全量重编，耗时无谓）；日常编译/烧录验证默认交给用户在 IDE 里做。确需命令行核对时，用「构建要点」里与 IDE 逐字一致的同一 fqbn。
- 指令协议、注释保持简洁；避免在注释里写死具体数值（参数调整时容易忘改）。
- **串口日志卫生**：手动指令只在类型切换时打一行「收到手动指令」，避免摇杆高频帧刷屏并阻塞控制时序；IDF 系统日志 `esp_log_level_set("*", ESP_LOG_WARN)` 默认静到 WARN，避免与手动指令 ack 争用同一 UART0。
- 大模型返回的指令必须严格校验后再执行，防止异常 JSON 导致小车误动作。
- 硬件标定值（舵机限位、IK 几何、移动时长表）集中在 `Calibration.h`，改动前确认已实测。
