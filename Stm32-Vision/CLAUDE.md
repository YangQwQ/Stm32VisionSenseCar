# CLAUDE.md

> 本文档基准：仓库 HEAD `dda6b05`（2026-09-22）。只覆盖已提交内容；未提交改动不收录。

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
| `src/exec/motion_verify.h/.cpp` | **动作后校验**（`mvfy`，namespace 未导出为公共 API）：动作落地后抓帧软解 RGB565、经单应反投影，判"车/臂到底动没动、动到哪"，供死区与"命令发了但没到位"的补救用（阈值/缩放档 `MVFY_*` 在 Calibration.h）。⚠️ 任务栈从 **PSRAM** 分配：内部堆在 AI/TLS/网络初始化后很紧，16384 字节从内部堆 `xTaskCreate` 极易失败（曾实测"任务创建失败"⇒ 全程不采样） |
| `src/core/heap_watch.h/.cpp` | **内部堆水位哨兵**：20ms 采样内部堆与 DMA 块（`MALLOC_CAP_DMA` 最大连续块），跌到危险线即告警（限流 2s），另出周期"最低/当前"汇总。⚠️ 判"小车是不是要挂了"看**DMA 块**而不是总空闲堆——**碎片化**才是真闸门；DMA 块跌破 `4096B` 时 1600B 的 RX 缓冲已在悬崖边（判据与救活流程见「PC 侧工具」的 `doctor`） |
| `src/core/command.h/.cpp` | 统一词表 JSON 分发（与传输解耦、回调应答）；手动指令先 `ai::cancel` 打断 AI 闭环再落地；`apply_network` 在线换网生效；`ai_goal`→`ai::set_goal`、`goto`→`ai::goto_target`、`ai_chat`→`ai::append_chat`；`log` 统一日志指令走 `blog` 开关；`get_state` 回灯/夹爪/AI busy 位图 `params.bits`；`nz_read` 走 `nezha::probe` I2C 在线探测 |
| `src/core/board_log.h/.cpp` | 统一日志模块（namespace `blog`）：`logf(cat,...)` 统一串口调试输出（带 `[类]` 前缀），按 `/log` 开关（exec/ai/all）把 `{type:"log",params:{src,text}}` 入队，由转发任务（栈 8192）经 app_httpd 注册的转发器（WS 广播+BLE status）发手机 |
| `src/ai/ai_client.h/.cpp` | 板载多模态 AI 任务级闭环（DIRECT 直调云端）：可输出的动作见「AI 层要点」的指令词表（`wait`/`approach`/`zoom` 由 worker **本地处理**，其余最终走 `exec::act`）；任务进度以 `tasks` 列表每轮渲染回喂、前几轮思考按 `AI_HIST_N` 环缓存多轮喂回；默认单帧，仅当上轮 `carry_prev:true` 才附带上一帧做运动对比；WS 文本出口统一过 `sanitize_ws_utf8` 消毒（云端偶发残缺 UTF-8，原样进 TEXT 帧会让手机端以 `1007` 断链）；发往云端的长字符串按 UTF-8 边界截断（截半个中文字节会被判 400）；服务端持续无有效响应则逐轮退避，超限中止任务并回报手机。⚠️ 模型回复的**校验与 params 透传**（`validate_cmd`）有一条易踩的不变量，见「AI 层要点」。组包 / 发送 / 记忆三块已拆到下面三个模块 |
| `src/ai/ai_prompt.h/.cpp` | 系统提示词与请求 body 组装：`PsaBuf`（PSRAM 增长缓冲，`heap_caps_realloc`+`MALLOC_CAP_SPIRAM`，worker 亦用它组 body）；`build_body()` 拼「系统提示词（角色+规则+JSON 格式+标定，不含目标）+ 独立 user(目标) 消息」，历史环逐条作为独立消息回喂构成真多轮对话，另喂执行板状态行与"距上次执行"秒数；图预算 ≤2：`carry_prev` 双帧优先（放弃参考图），否则 参考图(首轮)+当前帧 |
| `src/ai/ai_mem.h/.cpp` | 空间记忆 + 车姿态（自 `ai_client` 拆出）：`mem_reset()` 新任务起点（车位置=原点、车头=0°、清物体记忆）；`car_update_pose()` 按定距/定角近似累积位姿（move→平移 / spin→转向）；`mem_observe()`（相对车头角，正=右）与 `mem_observe_xy()`（屏幕归一化像素，经单应解算）入表；`mem_feed()` 生成车头局部系记忆文本喂 AI；`mem_find()` 查目标全局坐标；`mem_tick_stale()` 未观测过期轮数 +1。车姿态 `s_car_x/y/heading` 与 `navigate_to` 共享 |
| `src/ai/ai_http.h/.cpp` | AI TLS 发送层（自 `ai_client` 拆出）：`http_post()` 复用连接 POST 并读响应（keep-alive，成功尽量保留连接供下次复用）；`http_last_status()` 供 worker 做 4xx/429 快速失败判定；`http_stop()` 立即中止在途请求/连接（cancel / set_goal / goto 打断用） |
| `src/ai/ground_proj.h/.cpp` | 屏幕→地面坐标换算（namespace `ground`）：实测标定点拟合单应，`ground::screen_to_world(px,py,&x,&y)`；标定点 `kGroundCal` 在 Calibration.h，加测点改那里 |
| `src/ai/ai_dump.h/.cpp` | **AI 抓帧留档**（debug，供 PC 侧复盘）：把**实际发往云端的那一帧原始 JPEG**连同该轮标注（动作、被闸门拒的原因等）留在 PSRAM，6 槽环形；`seq==0` 表示该槽尚未定案、对 HTTP 不可见。每槽缓冲**按需增长**（8KB 步进，只按实际见过的最大帧分配——预留 6×128KB 会白占近 800KB PSRAM）。HTTP 出口：`/ai_dump`（清单，`?after=N` 长轮询）、`/ai_frame?seq=N`（原始字节）。取回与配日志见「PC 侧工具」的 `car_logcat.py` |
| `src/ai/magnify.h/.cpp` | **放大镜**（namespace `magnify`）：按 AI 给的归一化 `px`/`py` + **绝对倍数** `scale` 裁一小块并放大成新 JPEG 回喂，供"看不清就凑近看"。`crop_to_jpg()` + `last_cost_ms()`/`last_src_px()`（诊断）。解码用**全分辨率**（`JPG_SCALE_NONE`）——**勿退回 1/2**：1/2 解码后放大的是一张已丢掉 3/4 像素的图，看着大了其实全是插值糊出来的。失败时本轮回全幅 |
| `src/net/ping_svc.h/.cpp` | `/ping <目标>` 异步 ICMP echo（esp_ping），结果经 cmd 回复通道回报；无目标仍由 command 就地回 `pong` |
| `src/net/ble.h/.cpp` | BLE GATT Server：配网 + 兜底控制 + status 通知；广播开关随 `set_transmission`（真在推帧即停）（UUID 见下「协议参考」）|
| `src/net/ota.h/.cpp` | OTA 升级：ArduinoOTA + HTTP `POST /update`（板子固定在车上、串口够不着，刷固件只能走这里；双 OTA 槽见下 `partitions.csv`） |
| `src/net/app_httpd.cpp` | HTTP（MJPEG / 拍照 / LED 灯）+ WS（端口 81：文本=指令/状态 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报（默认关，`/log exec on` 开启后约 400ms 一条；状态缓冲须容下含抓手前端的整行，改状态行时同步核对）。注册 `blog` 日志转发器（WS+BLE）。`ws_stream` 任务栈 8192（推流 + 状态上报共用）。人脸检测/识别已停用（宏置 0） |
| `Calibration.h/.cpp` | **手动校准数据集中区**（根目录）：舵机限位/机械臂参数（STEER/REACH/GRIP/LIFT、ARM_*）、小车定距/定角移动时长表（MV_*/SPIN_MSDEG_*/SPIN_MIN_SPEED/MV_START_MS/MV_MIN_PULSE_MS/ARM_CNT_PER_CM）与旋转枢轴偏置 `SPIN_PIVOT_BEHIND_CM`、`mvfy` 的开关与阈值档 `MVFY_*`、屏幕→地面单应标定点 `kGroundCal`、机械臂夹心散点 `kArmPts`；`bivar::arm_set()` 实现在 Calibration.cpp |
| `partitions.csv` | 分区表（sketch 自带，**覆盖** fqbn 的 `huge_app`）：app0/app1 双 OTA 槽各约 3.8MB + `coredump` ⇒ OTA 可用、panic 可落盘 |

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

## AI 层要点（`ai`）

- **AI 指令词表**（模型能输出的 `type`，与手机端词表是**两套**，勿混）：`move` / `stop` / `arm` / `spin` / `arm_pose` / `wait`（空操作）/ `approach`（按记忆自动靠近到约 10cm）/ `zoom`（放大镜）/ `light`（开/关车灯）。其中 `wait`/`approach`/`zoom` 由 worker **本地处理**、到不了 `exec`，因此也**不经过 `fmt_last`**；`fmt_last` 只为 `move`/`arm`/`arm_pose`/`spin`/`light` 单列了分支，其余（`stop`/`reset`）落到 `else` 一律被写成 `"stop"`。
- ⚠️ **`validate_cmd` 是"逐个显式拷贝"白名单 params 键**：把某个 type 加进白名单**不够**——下游 worker / `exec` 读的键只要没在那个块里显式拷一遍，拿到的就**永远是默认值，而且不报错**。已踩三次，全是"执行了却没反应"：
  1. `light` 漏 `kind`/`on` ⇒ `exec::act("light")` 收到空 kind 被拒；
  2. `zoom` 漏 `px`/`py`/`scale`/`reset` ⇒ **放大镜无论 AI 报哪儿都只裁画面正中心**。这条最阴：提示词里有**四处**让 AI"用 zoom 看目标位置"，而合爪的**唯一判据**是"放大图里方块落在两指之间" ⇒ 整段提示词静默失效，AI 反复 zoom 也永远看不到目标。排查"AI 总夹空 / 反复 zoom 空转"**先查这里**；
  3. 模型把**顶层字段塞进 `params`**（实测漏过一次 `done`）⇒ 它以为标了"完成"、程序只在顶层读、任务在该结束的轮次继续空跑（`done`/`carry_prev` 已加两处容错）。
  **改词表的四步**：① 加 type 白名单；② 加 act/kind 类**非法值校验**（让 AI 收到明确拒绝，而不是默默用默认值）；③ 加**透传拷贝**；④ 若该 type 有日志，加 `fmt_last` 分支（否则落进 `else` 被打成 `"stop"`，复盘时把 A 动作误读成"已停车"）。白名单块开头另有一条**兜底告警**：遇到未登记的 params 键会 `logf` 一句，别只靠"现象反推"。
- **同源不变量**：程序喂回 AI 的措辞、闸门判据、程序给出的补救动作、**日志的措辞**必须互相自洽——AI 能看见的只有这些字，四处说法不一致它就会做出与程序预期相反的动作。
- **`zoom` 语义**：`px`/`py` 是**归一化画面坐标**（框半宽 = `0.5/scale`，`scale` 是**相对全幅的绝对倍数**、不累乘，上下限 `AI_ZOOM_MIN/MAX`）；程序用"已发出的框"把 AI 此后在放大图里报的 0~1 坐标自动换算回全幅，所以 AI 在放大图里照常填 0~1。同一块画面被重复索要且车臂未动 ⇒ 判复读，超过 `AI_ZOOM_NOOP_MAX` 次强制回全幅。
- **车灯**：`kind` 分 `front`/`vibe`/`back`——**`back` 是中性白光，唯一适合照亮判色**；`front`/`vibe` 是绿光会把整幅画面染绿，使黄/绿判别直接失效，找方块时不要用。亮环境下开背灯几乎不增亮（只是把色比拉回中性），暗环境才真有增益。⚠️ 测"灯开没开"**别看绝对亮度帧差**（环境光十几秒内就会漂），要看**色比**（`R/G≈0.98` 即背灯签名）。
- **抓取范式**：现行抓取流程与用户给定的目标范式**尚有多处不符**（含 `zoom` 的使用时机、
  丢失目标的尝试次序、`y<7` 后退线）——**改抓取相关提示词或闸门文案前，先读「已知问题」第 1 条**。
- **提示词偏好**：不写死绝对屏幕坐标（定标数值会随摄像头姿态腐烂），优先写方向性 / 差分 / 相对夹爪的判据；词条保持简洁。

## 构建要点

- 框架：Arduino（`esp32` 板支持包）。当前板为 **ESP32-S3-CAM（N16R8，带 PSRAM）**，摄像头选型在 `camera.h` 顶部 `CAMERA_MODEL_ESP32S3_EYE`；IDE / 命令行目标一律 `esp32:esp32:esp32s3`+`16M flash / opi psram / huge_app`（fqbn 是 `vendor:arch:board` 三段，**arch 段是 `esp32` 不是 `esp32s3`**——写成 `esp32:esp32s3:...` 会被 arduino-cli 当成本平台而报 `platform not installed`）。⚠️ 曾误用经典 `esp32:esp32:esp32cam`（AI-Thinker）目标，编出固件无法用于本板，勿回退。
- 当前在 **esp32 core 3.3.11** 下全量编译链接通过。核心 API 已按 3.x 适配，**勿回退 2.x**：
  - LEDC 引脚式：`ledcAttach(pin, freq, res)` / `ledcWrite(pin, duty)`（勿用 `ledcSetup`/`ledcAttachPin` 等 channel 式调用）
  - BLE：`getValue()` 返回 Arduino `String`；无 `getNotifyProperty`；发射功率枚举为 `ESP_PWR_LVL_P9`
  - WS 无 `httpd_ws_client_iterate` → 用 `httpd_get_client_list` + `httpd_ws_get_fd_info` 过滤 `HTTPD_WS_CLIENT_WEBSOCKET`
- 命令行验证
  当前已核准的 IDE 板子配置的对应 fqbn：
  `arduino-cli compile --fqbn "esp32:esp32:esp32s3:FlashSize=16M,FlashMode=dio,PartitionScheme=huge_app,DebugLevel=debug,PSRAM=opi,EraseFlash=none" .`
  （arduino-cli 用 IDE 内置的那份，位于 **Arduino IDE 安装目录**下 `resources\app\lib\backend\resources\`（与 IDE 同版、缓存目录同源）；**该目录不在 PATH**，命令行调用需写全路径。`tools/carctl.py` 不必手填：它按 `ARDUINO_CLI` 环境变量 → PATH → IDE 安装目录（含各盘符 `Program Files`）依次找。若某次 IDE 把 DebugLevel 调回 none/其它，命令行同步改回，避免不共享缓存。）
  - 上面这条 fqbn 已固化进 `tools/carctl.py`，不必手打：`uv run tools/carctl.py build`（只编译）、`build --clean`（全量重编）、`build --flash`（编译→OTA→核对指纹一条龙）。改 fqbn 时两处要同步。
- ⚠️ mbedTLS 握手内存优化依赖自定义核心库：`Arduino15\packages\esp32\tools\esp32s3-libs\3.3.11` 已按 IDF `defconfig` 重编替换，含四处配置：SSLin/out 缓冲 `CONFIG_MBEDTLS_SSL_IN/OUT_CONTENT_LEN=8192`（规避内部堆碎片导致的握手失败 `-32512`/`-17040`）；`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`（mbedTLS 缓冲改走 PSRAM，规避内部堆碎片导致的 AI 请求 `-3`）；lwIP TCP 缓冲 32768/16384；WiFi TX 缓存缓冲 `CONFIG_ESP_WIFI_CACHE_TX_BUFFER_NUM=32`（图传丢帧修复的前提，见提交 `cbc6372`——CACHE 类缓冲落在 PSRAM，不吃内部 RAM）。替换时勿用官方同名库覆盖。**重编流程、定制配置与链接脚本补丁见 [`librebuild/README.md`](librebuild/README.md)**（已入库，随提交 `33b18e7`）。⚠️ 链接脚本补丁不在 `lib/ include/ flags/ ld/` 里而在**变体目录**（本板 `PSRAM=opi`+`FlashMode=dio` ⇒ `dio_opi/sections.ld`），覆盖脚本不管它，**升级核心版本后必须重打**。
  - ⚠️ 判断库配置别读错文件：`esp32s3-libs/3.3.11/sdkconfig` **不是**编库时用的那份。真配置在 WSL lib-builder 的产物里（`~/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs/esp32s3/sdkconfig`，入口 `librebuild/run-idflibs.sh`）。曾据此误判 `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` 没开、差点白跑几小时重编——**它早就是 `=y`**。另：`CONFIG_LOG_MAXIMUM_LEVEL=1` ⇒ 驱动的 `ESP_LOGW/I/D` 编译期即被摘掉，**运行时 `esp_log_level_set` 救不回来**，想看见必须重编库。
  - ⚠️ **但 WiFi 缓冲区"个数"上面两份都不算数**：`WIFI_INIT_CONFIG_DEFAULT()` 是**调用方编译期**展开的宏（`esp_wifi.h`），所以 `*_BUFFER_NUM` 由**生效头文件**决定——本板 `PSRAM=opi`+`FlashMode=dio` ⇒ **`dio_opi/include/sdkconfig.h`**（顶层 `include/` 下没有 `sdkconfig.h`）。库自己那份 config 里的个数对运行时**无效**（WSL 产物里写 `STATIC_TX_BUFFER_NUM=32`，实际生效是 **8**）。该头文件在**变体目录**里，`overlay-libs.sh` 不覆盖 ⇒ 与链接脚本同样是**手工补丁**，已就绪于 `librebuild/patches/sdkconfig.h.dio_opi`。改完必须**重编草图**且建议 `--clean`（它在 74 个文件的 `.d` 依赖里）。验证：`uv run python tools/probe_macro.py`（打印编译期真实宏展开，比"编译通过"强）。
  - 上一条所需的一切（编库脚本 + 定制 `defconfig` + 链接脚本补丁）现均已入库于 `librebuild/`；仓库根的 `.trae/` 是更早的排查草稿目录（已 `.gitignore`，不入库），仅当需要追溯当时怎么试出来的才去看。
  - ⚠️ `ai_http.cpp` 里的 `ai_tls_calloc` / `ai_tls_free` 是同一思路的**预留实现，当前并未接线**：全仓库没有任何 `mbedtls_platform_set_calloc_free` 调用点，两者不会被 mbedTLS 使用。上面那条 PSRAM 效果来自核心库的 `EXTERNAL_MEM_ALLOC` 配置本身；改这两个函数不影响 TLS 分配。
- ⚠️ 实测：即使 fqbn 完全一致，**IDE 验证 ↔ 命令行切换仍常各自全量重编**（esp32 core 整包重编，5–10 分钟）；想省时间就让「主编译入口」固定在一侧，别频繁来回。**改/增/删源文件后首次编译若报多定义或「多个文件 -o」错，删 `%LOCALAPPDATA%\arduino\sketches\<sketch哈希>\` 下本 sketch 缓存目录再编**。
- 依赖库：**ArduinoJson v7（Benoit Blanchon）**，装在**用户 sketchbook** 的 `libraries/ArduinoJson`（sketchbook 位置见 IDE「首选项 → 项目文件夹位置」，Windows 常见为 `Documents\Arduino`；也可从 `arduino-cli config dump` 的 `directories.user` 查）。⚠️ sketch 内 `libraries/ArduinoJson` 子目录 **Arduino 不会自动扫描**，属冗余副本，勿依赖（可删）。
- 分区：fqbn 里的 `PartitionScheme=huge_app`（3MB APP）**实际被 sketch 根目录的 `partitions.csv` 覆盖**（Arduino 优先用 sketch 自带的表），该表是 `nvs / otadata / app0(0x3d0000) / fr / coredump / app1(0x3d0000)` —— 即 **app0 约 3.8MB 且带双 OTA 槽**。所以 OTA 可用（`src/net/ota.*`：ArduinoOTA + HTTP `POST /update`），烧录时不必插拔板子；报错时也**不必**怀疑"分区表没 OTA"。依赖库与编译缓存目录同 IDE（`%LOCALAPPDATA%\arduino\sketches\<sketch哈希>\`）。

## PC 侧工具（`tools/`）

板子固定在车上、串口够不着，**所有诊断只能走网络**。`tools/` 下这几个脚本就是电脑这侧的手，全部经 `uv` 直接跑（依赖写在各文件头的内联元数据里，自动装，无需手动建环境）：

| 脚本 | 定位 | 干什么 |
| --- | --- | --- |
| `tools/carctl.py` | **操作台**（一次性动作，随发随走） | 状态 / 日志 / 发指令 / 抓帧 / ping / 重启 / 体检 / 编译 / OTA / panic 取证 / 压测 |
| `tools/car_logcat.py` | **记录仪**（长时间跑） | 板端日志落盘 `logs/car-<时间戳>.log`，并把 **AI 每轮实际发往云端的那一帧**留档到 `logs/aiframes/<同名目录>/`；断线自愈 |
| `tools/probe_macro.py` | 编译期宏探针 | 打印固件**真实展开**的宏值，用于证实/证伪"某个 sdkconfig 补丁到底进没进固件" |
| `tools/test_car_logcat.py` | 宿主端自测 | 不接板子、不联网，桩板端跑一遍 `car_logcat` 的 CLI（回填环形 / 长轮询 / 旧固件退化 / 归档目录推导等）。**改动抓帧链路后先跑它**，比在车上一趟趟试快得多 |

> ⚠️ `carctl.py` 与 `car_logcat.py` 的少数基础设施（代理剥离、板子指纹、网段扫描、地址缓存）**各有一份拷贝**——刻意不互相 import，好让每个脚本都能单独 `uv run` 起来；改动请顺手同步两个文件。

**地址解析（两者共用）**：`-H` 指定 → `~/.car_logcat/last_host` 缓存 → 扫本机各 `/24` 网段的 81 端口并用板子指纹（`GET /` 回 `302 /stream`）确认；连上后写回缓存。`--no-scan` = 缓存失效时直接报错而不扫网段。
⚠️ 脚本内部已剥离代理环境变量；**手工 `curl` 板端必须加 `--noproxy '*'`**，否则拿回的是代理的错误页（曾据此误判板子死了）。

### `carctl.py` 子命令

公共选项：`-H/--host`、`--no-scan`、`--timeout SEC`（WS 建连 / HTTP 超时，默认 5）、`-v/--verbose`（连保活 pong 也打出来）。

| 子命令 | 干什么 | 常用选项 |
| --- | --- | --- |
| `status` | 板上跑的是哪份固件、OTA 会落到哪个槽、状态位 | — |
| `log` | 实时看板端日志（内部每 30s 重申一次开关，因为板端 `log` 是**全局单选**状态） | `-t/--for SEC`、`--cat {all,ai,exec}`、`--leave-on`（退出不关转发，留给手机看） |
| `cmd <type> [k=v …]` | 按词表发一条指令并收回复；值自动转 int/float/bool/字符串 | `--wait SEC`、`--no-log` |
| `raw '<json>'` | 直接发一段原始 JSON（绕开词表拼装） | `--wait SEC`、`--no-log` |
| `ping [target]` | 无目标=测板子在不在；有目标=**让板子去 ping**（查"WiFi 关联着但不通"） | `--wait SEC`（默认 25）、`--no-log` |
| `reboot` | 远程重启（链路卡死时的解药） | `--wait SEC`、`--wait-up SEC`（等板子回来） |
| `doctor` | **分层体检**：在哪层断的就在那层给解药；WS/HTTP 全灭时自动走 **BLE** 重启救活 | `--no-heal`（只诊断）、`--wait-up SEC` |
| `fw [bin…]` | 读 `.bin` 指纹；不给文件则列出本机编译缓存里的候选 | — |
| `build` | 用与 IDE 逐字一致的 fqbn 编译（见「构建要点」） | `--clean`（全量重编）、`--flash`（编完直接 OTA + 核对指纹）、`--force`、`--wait-up SEC` |
| `ota <bin>` | 推固件 → 等板子回来 → **核对指纹** | `--dry-run`（只报告不上传）、`--force`、`--no-wait`、`--wait-up SEC` |
| `coredump` | 取 panic 现场并解栈（默认按 coredump 回传的 sha 去归档里自动找 `.elf`） | `--erase`（取完清现场）、`--elf PATH` |
| `stress` | **abort 风暴**：AI 在途时反复中止，逼出跨任务竞态（改并发 / 修 panic 前后各跑一次对比） | `--rounds N`、`--gap SEC`、`--abort-ms MS`、`--sample SEC`、`--type {ai_oneshot,ai_goal}`、`--goal TEXT`、`--keep-panic` |
| `frame` | 抓一张（或连拍）画面——手动操作时的眼睛 | `-o/--out PATH`、`-t/--tag TAG`、`-n N`（连拍，看运动用）、`--gap SEC`、`--tries N` |
| `zoomshot` | 板端 `/zoomshot` 裁块放大一张 —— **手动复核"AI 看到的放大图对不对"**，与 AI 的 `zoom` 同一条链路（板端同一实现） | `--px`/`--py`（归一化框心）、`--scale`（倍率，默认 2）、`-o/--out PATH`、`-t/--tag TAG` |
| `step <type> [k=v …]` | **发指令 + 抓帧 + 记状态行三件套**，一起写进 `tools/shots/steps.txt` | `-t/--tag TAG`、`--wait SEC` |

**`step` 是手动复现"夹一次"时最该用的**：一次操作要同时留下「我发了什么 / 板子自报爪子在哪 / 画面实际是什么」，而分开跑三条命令**永远对不上账**（帧比指令晚、状态又是另一时刻的）。合成一条后，帧与状态行取自**同一时刻**、落在**同一个会话文件**里，事后能逐帧对着复盘。它会临时打开 exec 日志转发拿状态行（板端在 WS 断开时自动关掉，不用手动收）。

**`ota` / `build --flash` 的收尾那步最值钱**：写完固件、板子重启回来后，再读一次 `/update`，把**板上正在跑的指纹**与**你刚推的那个 `.bin` 的指纹**摆在一起比。指纹取 `.bin` 偏移 `0xb0` 的 4 字节（esptool 按 `--elf-sha256-offset 0xb0` 打进去的 ELF SHA-256 前 4 字节），换个源就是另一份 `.bin`——所以两边一致就等于"新固件确实在跑"，不一致就是没有。

```bash
uv run tools/carctl.py status                     # 先确认板上跑的是不是你要的那份固件
uv run tools/carctl.py build --flash              # 编译 → OTA → 核对指纹（增量约 90s）
uv run tools/carctl.py log -t 30 --cat ai         # 看 AI 的决策
uv run tools/carctl.py cmd spin dir=1 speed=500   # 发一条词表指令
uv run tools/carctl.py step arm act=low -t low1   # 手动夹取的一步，留可对照的记录
uv run tools/carctl.py frame -t after-lift        # 抓一张看结果
uv run tools/carctl.py zoomshot --scale 3 -t mag3 # 手动放大一块，看 AI 的放大图能不能读
uv run tools/carctl.py doctor                     # 连不上时的第一条命令
```

### `car_logcat.py`

```bash
uv run tools/car_logcat.py                        # 自动找车，日志 + AI 画面留档一起收
uv run tools/car_logcat.py --cat ai -o my.log     # 只要 AI 类别，指定落盘路径
uv run tools/car_logcat.py --no-frames            # 只记日志，不抓帧
```

- 日志走 WS（与手机拿到的一模一样，互不干扰——板端是广播给所有 WS 客户端）。
- **画面留档要两个条件同时满足**，缺一不可：
  1. 板端固件**有 `ai_dump` 模块** —— 记录仪启动时会先探一次 `/ai_dump`，拿不到（404/超时）就直接判定
     "本次只记日志"并**全程不抓帧**（`car_logcat.py:1066`），不会途中重试。该探测**与 `/log` 开关无关**，
     只反映固件版本。
  2. 板端 **`/log ai on` 已生效** —— 留档只在 blog 的 AI 类别开启时进行，关着时 `dump_push` 一次也不
     memcpy，并在关掉的那一刻把已占的 PSRAM **全部还给堆**（`ai_dump.cpp:62-70`）。
     ⇒ 要用 `--cat ai` 抓帧，**先**确认 `/log ai on` 再生效记录仪，别指望它自己重试。
- 环形上限 6 帧（`AI_DUMP_SLOTS`），每槽只按实际见过的最大帧分配、8KB 步进增长。
- **为什么要留画面**：AI 每轮据一帧画面决策，但板端日志只留**决策与坐标**，画面转瞬即弃；于是复盘"它为什么夹空"时只能对着坐标猜它看见了什么——而实测最有价值的那类证据恰恰是"日志说居中、画面里目标却在旁边"。
- 抓帧用**长轮询**（请求挂在板端不返回，一有新定案帧立刻应答），跑在**自己的线程**里与 WS 会话解耦：板子断线重连、甚至 WS 完全连不上时，画面照收。
- 板端活体探测是**全局**的（连续 6s 无任何客户端上行就广播 ping，多轮无应答把**所有**客户端一起踢），所以本脚本自按 3s 周期发 `{"type":"ping"}`——既是自己的保活，也顺带喂饱那个全局 idle 计时器。

### `probe_macro.py`

```bash
uv run python tools/probe_macro.py                            # 默认看 WiFi 缓冲那几个宏
uv run python tools/probe_macro.py -p WIFI_ CONFIG_ESP_WIFI_  # 按前缀过滤
uv run python tools/probe_macro.py -i esp_wifi.h -i lwipopts.h # 换探针头文件
```

从草图缓存的 `compile_commands.json` 里取一条**真实编译命令**的完整参数（`-I`/`-isystem`/`-D` 一个不差），换成 `-E -dM` 只做预处理并打印全部宏定义。"编译通过"完全不能证明某个宏配置生效（头文件没被覆盖、或改错了那一份——变体目录 vs 顶层，见「构建要点」），这个工具给的是**强证据**。

### `tools/shots/`

`frame`/`zoomshot`/`step` 的默认落图处（`tools/shots/*.jpg`）；`steps.txt` 是 `step` 追加的记录文件（动作 / 状态行 / 图名一行一条）。属调试产物，非源码（已在根 `.gitignore` 忽略）。

## 已知问题

> 均为真机实测 / 源码核对所得。**本节目的是交接——只列仍未解决、或需要留意的**；已修复的不再展开
> （要考古看 git 历史与 memory）。

**已修复、不再展开**（随提交 `dda6b05` 入库）：

- **放大镜输出坏图** —— 两个独立缺陷：①RGB565 输出级产噪声；②RGB888 路径整体 R/B 互换（黄方块变青）。
  修法在 `src/ai/magnify.cpp`：解码走 `fmt2rgb888`、放大循环里对调 R/B、编码用 RGB888（3 字节/像素）。
  两处均**已在真机验证**（以同一次运行内的板出全幅帧做配平对照）。
  ⚠️ 留一条方法学教训：**彩度模长对 R/B 互换天然免疫**（R↔B 对调让 Cb/Cr 同时取反、模长不变），
  判颜色正确性必须看**具体像素的通道值**，别只看彩度 / 灰度——当初据此差点漏判。
  该缺陷曾是夹取一线的主要污染源（同时污染"看不看得见"与"算得准不准"）。
- **`motion_verify.cpp` 取帧长度口径不一致** —— 已统一改用 `cam::jpeg_len(fb)`（`fb->len` 是**缓冲容量**，
  会把上一帧残留一起喂进解码器，正是"结构在、颜色毁"坏图的前置条件之一）。

### 1. 抓取流程：范式已定，代码尚有多处不符（**范式由用户给定，代码待按此改**）

#### 1.1 范式（用户 2026-09-22 给定，作为后续改 AI 流程的基准）

⚠️ 关键是**先降爪、后对准**（与"先对准再降爪"相反）；**低姿 `arm low` 本身是对的，不是要改掉的东西**。

1. **远**：标记目标位置（`observe`），用 `approach` 靠近。
2. **判"较近"**：目标落到**屏幕 2/3 高度以下**，**或**标记后**距离 < 15**（cm）——满足其一即进近场流程。
3. **近场**：**先降爪** → **对准** → **小幅前进** → 回到"对准"循环，直到目标进到**两指之间 / 正对位置**。
4. **这时（也才这时）**才用 `zoom` 复检目标位置是否正确——**对位过程中不要靠 zoom 反复重观测**。
5. **高度不合适 ⇒ 略微抬起夹爪**；否则直接试夹（`arm grasp`）并**验证是否夹住**。
6. **目标丢失时的尝试次序（升序，别一上来就环视）**：先做**上一步行为同方向**的操作 → 再**移动机械臂**
   看是不是被自身遮挡 → 最后才是**后退 / 重新寻找**。
   - 降爪后丢失 ⇒ 抬爪查看，或后退；
   - 大步前进后丢失 ⇒ 后退。
7. **纠偏两条**：
   - 局部系 **`y < 7`**（cm）⇒ **后退**（此时物体大致已与夹爪平行、或落在车后）；
   - 目标在夹爪下、**移动时位置不变**（被卡着推走）⇒ **抬臂 + 后退**。

#### 1.2 代码现状（2026-09-22 源码核对）

**主线的顺序已经对了**：`src/ai/ai_prompt.cpp:97-103` 就是「I.靠近 → II.降爪(`arm low`) → III.对位
(小角度 spin + 小步 move 顶进) → VI.夹取 → VII.验证(`wait` 后看是否跟着升起)」，即**低姿先降爪再对准**，
与范式第 3 条一致。范式第 7 条后半（"被卡着推走 ⇒ 抬臂 + 后退"）也已写进 `:100`（"若后退后目标仍跟随夹爪,
需要抬起夹爪并后退避免卡住物体"）。**其余各条均有偏差**：

| 范式 | 代码现状 | 位置 |
|---|---|---|
| 2. 近场判据 = 2/3 高 **或** 距离<15 | 只有"画面 2/3 高及以下"；"接近 15cm"只作 **IV 步确认**的辅助判据，不是近场开关 | `ai_prompt.cpp:98` vs `:101` |
| 3./5. 高度不合适可**略抬爪** | 提示词反向写死「降到这儿以后**高度就不用再管了**, 后面只调角度」；闸门文案同句 | `ai_prompt.cpp:99`、`ai_client.cpp:1087`、`:1540-1541` |
| 4. `zoom` 只在**末端确认**用 | 对位途中就建议「每走一步停下看画面(**拿不准就 zoom 放大**)」 | `ai_prompt.cpp:100`、词表 `:86` |
| 6. 丢失次序：原方向 → 动臂查遮挡 → 后退/环视 | 实际是**环视(spin)优先**，后退与 `fold` 收臂并列推荐 | `ai_prompt.cpp:111`、`:113`；`ai_mem.cpp:319-320` 甚至写「看不见多是被两指挡住(正常, **别后退找**)」 |
| 7. `y < 7` ⇒ 后退 | **全仓库没有任何 y 阈值**；闸门只在 `gfwd < AI_GRASP_FWD_MIN`（= `0`，即已到车后）才拒 | `ai_mem.h:19-20` |

> ✅ **"略抬爪"在程序上是通的**：闸门对 `arm_pose` 的 `h > AI_GRASP_H_CM`(2.5) **无条件放行**
> （`ai_client.cpp:135`、`:1508`），`arm lift_up`/`lift_down` 也从不进闸门——**卡住的只是提示词那句话，
> 不是能力**。改第 3/5 条只需改文案，并把 `arm_pose` 从"一般用不到"（`ai_prompt.cpp:82`）里放出来。
> 已知的位姿锚点：低姿落点 `ARM_LOW_X_CM`=8.0 / `ARM_LOW_H_CM`=1.0（`Calibration.h:41-42`），
> 夹取高度闸门 `AI_GRASP_H_CM`=2.5，`arm_pose` 的 `h` 可达约 1~12（`ai_prompt.cpp:82`）；
> `AI_NEAR_FWD_CM`=14 / `AI_NEAR_BLIND_CM`=12 / `AI_APPROACH_STOP_CM`=10 是近场的一组前距口径。

**⚠️ 同源不变量违规（改代码时要一并清掉）**：

- **残留的旧口径，与范式第 3 条（先降爪）相反**：闸门提醒 `ai_client.cpp:1631`「合爪前用 zoom 确认方块
  已到两指之间, 还在指尖前方就**回悬停位**小步前顶」、`:1641`「…(**悬停位对准后**)再下探」，以及 `:1638`
  注释把"悬停位(h=目标高度+3)"当成新范式的可夹带位——都还在把 AI 往**先悬停对准**指。
  注：`:1534` 的"新范式"指的是"②降爪 在 夹取 之前"，**这条是对的**；错的是上面三处悬停残留。
- **同一流程两套步号**：`ai_client.cpp:1534` 注释写"②降爪 在 **⑤**夹取 之前"、`:1603` 写低姿横向微调是
  "新范式的**第③步**"，而提示词 `ai_prompt.cpp:97-103` 的编号是 I~VII（降爪=**II**、对位=**III**、
  夹取=**VI**）。降爪与对位对得上，夹取差了号——同一件事两处叫不同步号。
- **度数自相矛盾**：闸门最多说"转约 20°"，程序随后在近场却会把这次 spin 砍到
  `AI_SPIN_NEAR_MAX_DEG`=6°（或 `AI_SPIN_TRACK_MAX_DEG`=15°）——程序给出自己**执行不了**的度数，
  量级小、不致跟丢，但会让 AI 困惑。

程序喂给 AI 的字与它自己的判据不自洽，而 **AI 能看见的只有这些字**。

### 2. 推理吃光 token 预算 ⇒ 整轮空跑（中）

日志：`[ai] 空content: choices=1 finish=length content=str(0B) reasoning=30656B 补全/总=8192/14552`。
模型把预算全烧在 `reasoning` 上，`finish_reason=length`、正文 0 字节，该轮白跑。**这是「AI 空 content」
之谜的真实诊断**（此前怀疑的"插话打断"不是根因）。

程序侧已有缓解 + 诊断：请求体带 `"reasoning_effort":"low"`（`ai_prompt.cpp:192`）；空 content 时把
`finish_reason`、用量、content 类型一并打出来区分病因（`ai_client.cpp:700-712`，详见 memory
`ai-empty-content-diagnosis`）。**根因在模型侧**，仍会偶发；方向是给正文留最小预算 / 缩短推理链 / 换非推理档。

### 3. 并发写入者：另一个 AI 助手会同时操作同一块板（中）

本机上**另有一个 AI 助手（Trae SOLO CN，其配置目录在用户主目录下的 `.trae-cn\`）在同一个仓库里操作同一块板**——
它自己起 `tools/car_logcat.py -o logs/<它命名的日志>`（记录仪进程会挂着好几个小时不退），并自己发 `ai_goal`。

**症状（别误判成 AI 违令 / 固件 bug）**：

- 日志里出现**你没发过**的 `[ai] 任务开始 gen=N text=…`（gen 号还会跳号）；
- 明确写着"不要移动小车"的任务里，`[exec]` 的抓手 / 小车状态却在变；
- 你抓的"真值基准帧"与放大帧**内容对不上**（地砖位置、有无方块、机械臂姿态全都不同）。

**后果（重要）**：任何"车/臂全程未动 ⇒ 同场景同光照"的假设**都可能不成立**。所以配平对照**优先用同一次
运行内的板出帧**，别用"事后另抓一张真值帧"。

**怎么发现**：`Get-CimInstance Win32_Process`（或任务管理器）看有没有**不是自己拉起的** `carctl.py` /
`car_logcat.py`；或 `logs/` 下冒出不是你命名的日志。**处置：不要杀对方的进程**（那是另一个会话的活），
先确认谁在跑；要独占测板就跟用户确认后串行化。见 memory `concurrent-ai-on-same-board`。

### 4. 其他（低）

- **内部堆触底**：长任务期间体征出现 `DMA块最低=0k`、`堆最低=5k`（放大镜 / AI 任务常驻 PSRAM 缓冲，
  `ensure()` 按 16KB 对齐增长）。`DMA块最低` 触底会威胁 WiFi 收包，是本板的全板性红线，
  见 memory `car-wedge-wifi-dead-ble-alive`。
- **`tools/shots/` 等调试产物已忽略**：`Stm32-Vision/tools/shots/*`（`frame`/`step` 的落图）、
  `Stm32-Vision/logs/*`、`tools/__pycache__/*` 均在根 `.gitignore` 里。另：根目录 `log.txt` 已不在磁盘上，
  `.gitignore` 里也**没有** `log.txt` / `*.log` 规则。

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

## 关联项目

- 手机 App：`../Mobile-RemoteCtrl`（显示画面 / 指令编辑 / 下发 `ai_goal` / BLE 配网）。

## 约定

- 与用户交流使用中文。
- 编译验证：用户未明确要求时，**不主动跑 arduino-cli 编译验证**（esp32 单次 ~80s+ 起步、IDE↔命令行互切会各自全量重编，耗时无谓）；日常编译/烧录验证默认交给用户在 IDE 里做。确需命令行核对时，用「构建要点」里与 IDE 逐字一致的同一 fqbn。
- 指令协议、注释保持简洁；避免在注释里写死具体数值（参数调整时容易忘改）。
- **串口日志卫生**：手动指令只在类型切换时打一行「收到手动指令」，避免摇杆高频帧刷屏并阻塞控制时序；IDF 系统日志 `esp_log_level_set("*", ESP_LOG_WARN)` 默认静到 WARN，避免与手动指令 ack 争用同一 UART0。
- 大模型返回的指令必须严格校验后再执行，防止异常 JSON 导致小车误动作。
- 硬件标定值（舵机限位、IK 几何、移动时长表）集中在 `Calibration.h`，改动前确认已实测。
