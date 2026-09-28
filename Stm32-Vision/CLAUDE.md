# CLAUDE.md

> 本文档基准：仓库 HEAD `18ef9c1`（2026-09-29）。只覆盖已提交内容；未提交改动不收录。

本文件为当前项目（ESP32-S3-CAM / OV3660 摄像头板）的 AI 助手工作指南。

## 项目概述

基于 **ESP32-S3-CAM（N16R8，带 PSRAM）** 的视觉控制板。板子采集画面，交给多模态大模型（可在配网时配置 URL/Key/模型）识别，生成小车与机械臂的控制指令，**经软件 I2C 直驱哪吒扩展板**落地。

### 工作模式

1. **WiFi 直连 AI 模式**：板子经 WiFi 直调多模态 AI 接口，上传画面 → 解析返回控制指令 → **本板直接执行**（`exec::act`）。✅ 已实现（DIRECT：`ai_goal` → `ai_client` 迭代闭环，真机联调中）。AI 侧另维护**画面标定（单应）+ 物体空间记忆 + 车姿态累积**并把结果换算成当前车头局部系喂回模型；看图由模型的 `look` 工具按需索取（新拍全幅/放大，或按编号回看先前画面）。云端持续无有效响应会逐轮退避、超限即中止任务并回报手机。
2. **蓝牙配置模式**：BLE（GATT Server，广播名 VisionS3）接收配置（WiFi 账号密码 / AI 接口等）→ 存 NVS → **在线重建 STA 生效，不整板重启**（BLE 保活，手机无需重连）。✅ 已实现。**生命周期**：手机在 WS 连上后断开本机 BLE 让出射频（WS_ONLY）；广播开关由 `ble::set_transmission` 统一控制——仅当**真在推帧**（UDP 图传 / MJPEG）时停广播让 WiFi 独占射频；WS 指令/状态通道在位时不关广播，保持可发现（手机随时可重连/兜底，`onDisconnect` 尊重该状态）。

> 本板既是视觉/控制大脑，也是执行器：动作由 `direct_exec`（namespace `exec`）经 `nezha_direct`（namespace `nezha`）软件 I2C 直接下发哪吒扩展板。

## 代码库结构

源自 Espressif 官方 `CameraWebServer` 例程，已按功能模块拆分。**源码全部位于 `src/` 下按模块分的子目录**（`Stm32-Vision.ino`、`partitions.csv`、`build_opt.h` 仍在 sketch 根目录）。源码内的本项目头文件 include 一律以 `src/` 为根的相对路径（如 `#include "net/wifi_net.h"`）书写——Arduino 只把 `src/`（不含其子目录）加入 include path，子目录会递归编译但不进搜索路径，因此不要改回无前缀形式：

| 文件 | 作用 |
| --- | --- |
| `Stm32-Vision.ino` | 入口：setup 按 cfg→ble→exec→cam→net→web→ai 初始化；loop 调各模块 update + `exec::update_tick()` |
| `src/cam/camera.h/.cpp` | 摄像头初始化 + 抓帧（JPEG 双缓冲，同步取帧 `cam::grab()`；抓帧模式 `CAMERA_GRAB_WHEN_EMPTY`——图传与 AI 是两个并发取帧方，用 `LATEST` 会与之抢缓冲导致取帧卡死）。取帧全经 `s_cam_mtx` 串行。**高清真拍** `request_hires()`（放大镜用）：deinit → init(SVGA) → **连续预热 `HI_WARM` 帧挑有效帧** → 存进常驻 PSRAM 快照 `s_snap` → 回 VGA 再预热（`HI_WARM_RECONFIG` = `HI_WARM`+3；**回 VGA 的 deinit/init 失败会重试 3 次**——一次失败就长期回不到 VGA，会让后续所有放大镜取景失败、AI 空转）；期间 `grab()` 返回 **nullptr**（各方须容错），图传停一拍。⚠️ **只支持 SVGA**：SXGA 会卡死驱动。**型号唯一配置点**：在 `camera.h` 顶部选 `CAMERA_MODEL_*`（AI_THINKER 在 S3 上会空指针崩溃，勿用）；画质与画面回正（`vflip`/`hmirror`）在 `make_config(fs)` + `apply_sensor_calib()`（重建后必须重调，否则校准被清零），`jpeg_buffer_size` 已抬到 256KB，推流帧率在 `net/app_httpd.cpp`（`WS_STREAM_FPS`，图传走 UDP） |
| `src/cam/camera_pins.h` | 各摄像头型号 GPIO 引脚定义（按 `CAMERA_MODEL_*` 分支） |
| `src/cam/camera_index.h` | Web 前端页面（HTML/JS 内嵌数组，源自例程，现基本不用） |
| `src/net/config.h/.cpp` | WiFi / AI 接口配置，NVS 持久化 |
| `src/net/wifi_net.h/.cpp` | STA 连接 + 断线重连 + 在线换网 `net::reconnect`（namespace `net`）。⚠️ 勿改回 `network`：会与核心库 `Network.h` 在 Windows 大小写不敏感 FS 上遮蔽冲突 |
| `src/exec/nezha_direct.h/.cpp` | 哪吒扩展板软 I2C 直驱底座：`set_servo(ch,pwm)` / `set_motor(ch,a,b)` / `led(kind,on)`；从机 `0x80`，协议与哪吒硬件一致 |
| `src/exec/direct_exec.h/.cpp` | **执行层**：`act()` 分发 move/stop/arm/arm_pose/light/reset/spin、连续机械臂动作步进 `update_tick()`（兼管定距/定角到点自停）、离散定位走 S 形缓动 `advance_smooth()`（`ARM_SMOOTH_PEAK`）、二连杆 IK `arm_pose()`、固定位 `arm_low()`/`arm_raise()`、本地合成状态 `read_state()`、持续型判定 `is_continuous()`、`arm_moving()`（持续步进 / grasp 待抬臂 / S 形未收敛三类，供 AI 取帧前等停稳） |
| `src/exec/bivar.h/.cpp` | 机械臂"夹心坐标"双向散点反距离加权(IDW)插值：FK 用 pwm 距、IK 用 x/h 距；散点表 `kArmPts` 在 Calibration.h，`arm_set()` 实现在 Calibration.cpp |
| `src/core/heap_watch.h/.cpp` | **内部堆水位哨兵**：20ms 采样内部堆与 DMA 块（`MALLOC_CAP_DMA` 最大连续块），跌到危险线即告警（限流 2s），另出周期"最低/当前"汇总。⚠️ 判"小车是不是要挂了"看**DMA 块**而不是总空闲堆——**碎片化**才是真闸门；DMA 块跌破 `4096B` 时 1600B 的 RX 缓冲已在悬崖边（判据与救活流程见「PC 侧工具」的 `doctor`） |
| `src/core/lock_guard.h` | `ScopedLock`（RAII：构造取锁 / 析构放锁，`held()` 判是否真拿到，禁拷贝），用于需要显式限界的临界区（如 `ai_dump`） |
| `src/core/psram.h` | `ps_dup/ps_free/ps_set/ps_str`：**变长文本按实际长度分配到 PSRAM**——长度在写入那一刻就已知的字段（任务名 / 任务笔记 / 当前目标 / 物体名）一律走它，不写死上限。`ps_set` 先分配后释放（失败保留旧值）；`ps_str` 把空指针当空串 |
| `src/core/utf8.h` | `utf8_clamp_tail(char*)`：**定长缓冲按字节截断后的 UTF-8 边界回退**（末尾是被切一半的多字节字符就整段删掉）。⚠️ 漏了它，残留的孤立引导字节会被各处出口消毒换成 `'?'`；**凡往定长缓冲 snprintf/strncpy 的 AI 可见文本（提示语等）都要跟一句它** |
| `src/core/command.h/.cpp` | 统一词表 JSON 分发（与传输解耦、回调应答）；手动指令先 `ai::cancel` 打断 AI 闭环再落地；`apply_network` 在线换网生效；`ai_goal`→`ai::set_goal`、`goto`→`ai::goto_target`、`ai_chat`→`ai::append_chat`；`log` 统一日志指令走 `blog` 开关；`reboot` 远程重启（链路卡死时的解药，升级中会被拒）；状态位图 `cmd::state_bits()` 是**唯一出口**，`status` / `get_state` / `pong` 三类回执**都**带 `params.bits`（手机据此同步按钮与 AI 运行态）。⚠️ `ping`/`pong`/`get_state` 三类**不打** `type=` 刷屏行——排查"指令到了没有"时这三类查不到凭据 |
| `src/core/board_log.h/.cpp` | 统一日志模块（namespace `blog`）：`logf(cat,...)` 统一串口调试输出（带 `[类]` 前缀），按 `/log` 开关（exec/ai/all）把 `{type:"log",params:{src,text}}` 入队，由转发任务（栈 8192）经 app_httpd 注册的转发器（WS 广播+BLE status）发手机；`forward_text(cat, text)` 发**任意长度**文本（自建 PSRAM 副本 + UTF-8/JSON 转义与非法字节清洗），`logf` 那条 256B 栈缓冲装不下 AI 的 reasoning；`log_line(cat, text)` 收**已格式化好的整行**（省一次 vsnprintf，`logf` 内部走它） |
| `src/ai/ai_client.h/.cpp` | AI 任务**外壳**：worker 任务、代际/槽、对外 API（`init`/`update`/`set_goal`（带 `one_shot` = 只跑一轮决策即收尾）/`goto_target`/`cancel`/`busy`/`append_chat`/`logf`/`set_edited_image`）。worker 起来后调 `round_run_task`。⚠️ 任务主体已拆去 `ai_round.cpp`，而 `update()`/`logf()` 的**实现**在 `ai_result.cpp`——按文件名找会扑空（头文件有注明） |
| `src/ai/ai_round.h/.cpp` | **AI 任务主体**（原 `ai_client.cpp` 的大头）：`round_run_task()` 一轮闭环（取帧 → 组 body → 发送 → 校验 → 落地 → 记账）；跨轮状态全在文件内 `RoundCtx`，**按值建在 worker 的 16KB PSRAM 栈帧上**；任务列表/笔记/目标都是 **PSRAM 变长文本**（`core/psram.h`，无字数与项数上限；`task_list_replace()` 把 `todo` 整表放进单块 PSRAM，分配失败保留原列表并回一句"内存不足"）；收尾在 `round_task_finish` 统一释放。**任务面板上行**：`notify_tasks()` 在任务开始 / `task` 改动列表 / `goal.set` / 转等用户输入 / 收尾时推 `ai_task`（`state: running\|wait\|done\|fail\|abort`）；`round_prepare`（取帧 / 放大镜）、`round_attempt`（组包 → 发送 → 取工具调用）、`dispatch_calls`（**落地次序固定** mem → task → say → car → look → compact → goal）、`build_state_block`（车臂状态 + 目标 + 任务列表 + 笔记，挂本轮**最后一条工具结果**尾部）。历史按**回合**环缓存、`reasoning` 每轮原样回传；图只对**最新一组实景**注入字节（其余按 `ImageN` 编号渲染成占位）；WS 文本出口统一过 `sanitize_ws_utf8` 消毒（云端偶发残缺 UTF-8 原样进 TEXT 帧会让手机端以 `1007` 断链）；发往云端的长字符串按 UTF-8 边界截断（`src/core/utf8.h`；截半个中文字节会被判 400 或显示成 `?`）；服务端持续无有效响应则逐轮退避，超限中止任务并回报手机 |
| `src/ai/ai_result.h/.cpp` | 结果出口：`ai::update()`（消费 worker 回执、推状态）、`ai::logf()`、`enqueue_result()`、`set_edited_image()` / `edited_snapshot()` |
| `src/ai/ai_validate.h/.cpp` | 校验入口：剥代码块 → 解 JSON → 拒旧版扁平 `type` → **遍历工具表**逐项规范化；硬钳制宏 `AI_MOVE_MAX_CM`(40) / `AI_SPIN_MAX_DEG`(180)。⚠️ 词表本身**不在这里**，在 `src/ai/tools/` |
| `src/ai/ai_nav.h/.cpp` | 本地巡航 `navigate_to(gen,tx,ty,stop_cm)`、`settle_wheels(gen,max_ms)` / `settle_arm(...)`（取帧前等车/臂停稳，缺省 `AI_SETTLE_MAX_MS`=700；`AI_ARM_LOW_H_CM`=2.5 判"臂算低姿"）；收敛上限一批 `NAV_*`（到位容差 6cm、单次转角/推进上限、迭代与反转次数上限等，防开环死航打转） |
| `src/ai/tools/` | **AI 工具表的键级定义**：`tool.h` 定义 `ToolSpec{key,doc,group,parse,run,feedback,logfmt}`；`t_observe`(observe/delete) / `t_move`(move) / `t_arm`(arm) / `t_meta`(light + set/finish) / `t_tasks`(note/todo/done) 各提供 `parse_*`，`registry.cpp` 的 `kTools[]` 是唯一有序表（10 键，键名全局唯一）。**表序只决定校验顺序**（`validate_cmd` 按表序跑各 `parse`）；**落地顺序在 `ai_round.cpp` 的 `dispatch_calls`**。现阶段只填了 `parse`，其余留 nullptr 是有意的——给模型的**工具声明**另有 `ai_prompt.cpp::tools_schema()`（7 个工具，改键名/枚举两处必同步）。详见「AI 层要点」 |
| `src/ai/ai_alloc.h` | ArduinoJson 的内存池走 PSRAM（`PsramAllocator` / `g_js_alloc`，定义在 `ai_client.cpp`） |
| `src/ai/ai_prompt.h/.cpp` | 系统提示词与请求 body 组装：系统提示词 = 人设（Caris）+ 规则 + 工作流程 + 画面/夹爪定义（`kSysSrc`，编译期转义成静态字节）；`tools_schema()` = 请求尾部的 7 个工具声明；`build_body(PieceList&, const BodyReq&)` 把「系统提示词（静态段直接引用）+ 历史环逐回合（assistant(tool_calls) / tool 结果 / 用户发言）+ 末尾固定约束」组成**碎片流**（JPEG 边发边 base64，不再整份拼串）；目标/列表/笔记不单独成消息，由 `ai_round` 的 `build_state_block` 每轮现拼挂到最新一条工具结果尾部。⚠️ 提示词（行为判据）与 `tools_schema`（键名/枚举）是一套东西的两半，改一处必改另一处 |
| `src/ai/ai_mem.h/.cpp` | 空间记忆 + 车姿态（自 `ai_client` 拆出）：`mem_reset()` 新任务起点（车位置=原点、车头=0°、清物体记忆）；`car_update_pose()` 按定距/定角近似累积位姿（move→平移 / spin→转向）；`mem_observe_xy()`（屏幕归一化像素，经单应解算后入表；返回 false = 没记进去，调用方须回告 AI）；`mem_feed()` 生成车头局部系记忆文本喂 AI（表 16 条，满了覆盖最旧；**物体名在 PSRAM 上按实际长度分配**，见 `core/psram.h`）；`mem_find()` 查目标全局坐标、`mem_forget()` 按名删除（判据与 store/find 同一套）、`mem_tick_stale()` 未观测过期轮数 +1。车姿态 `s_car_x/y/heading` 与 `navigate_to` 共享 |
| `src/ai/ai_http.h/.cpp` | AI TLS 发送层（自 `ai_client` 拆出）：`http_post(PieceList&)` 复用连接 POST 并读响应（keep-alive）；`http_last_status()` / `http_last_error()` 供 worker 做 4xx/429 快速失败与分诊；`http_stop()` 立即中止在途请求/连接（cancel / set_goal / goto 打断用）；`extract_tool_calls()` 解响应取 `calls[]` + `reasoning_content`（非流式一次收全），无工具调用时把 `finish_reason`/用量打出来 |
| `src/ai/ground_proj.h/.cpp` | 屏幕↔地面坐标换算（namespace `ground`）：实测标定点拟合单应，`screen_to_world(px,py,&x,&y)` 与 `world_to_screen(x,y,&u,&v)`（伴随矩阵解析求逆，标定区外返回 false）；标定点 `kGroundCal` 在 Calibration.h，加测点改那里。⚠️ `init()` 的回验超差**只打日志、不禁用单应** |
| `src/ai/ai_dump.h/.cpp` | **AI 抓帧留档**（debug，供 PC 侧复盘）：把**实际发往云端的那一帧原始 JPEG**连同该轮标注（动作、被判无效的原因等）留在 PSRAM，6 槽环形；`seq==0` 表示该槽尚未定案、对 HTTP 不可见。每槽缓冲**按需增长**（8KB 步进，只按实际见过的最大帧分配——预留 6×128KB 会白占近 800KB PSRAM）。HTTP 出口：`/ai_dump`（清单，`?after=N` 长轮询）、`/ai_frame?seq=N`（原始字节）。取回与配日志见「PC 侧工具」的 `car_logcat.py` |
| `src/ai/magnify.h/.cpp` | **放大镜**（namespace `magnify`）：`crop_center_jpg(...)` —— `cam::request_hires` 高清真拍（SVGA）+ 用 TJpgDec **部分解码**只解出中央 `(0.25,0.25)-(0.75,0.75)` 再重编码（workbuf 与输出全走 PSRAM，规避 `fmt2rgb888` 整幅软解卡死）→ 输出 400×300，**同源像素密度**；外加 `last_cost_ms()` 诊断 |
| `src/net/ping_svc.h/.cpp` | `/ping <目标>` 异步 ICMP echo（esp_ping），结果经 cmd 回复通道回报；无目标仍由 command 就地回 `pong` |
| `src/net/ble.h/.cpp` | BLE GATT Server：配网 + 兜底控制 + status 通知；广播开关随 `set_transmission`（真在推帧即停）（UUID 见下「协议参考」）|
| `src/net/ota.h/.cpp` | OTA 升级：ArduinoOTA + HTTP `POST /update`（板子固定在车上、串口够不着，刷固件只能走这里；双 OTA 槽见下 `partitions.csv`） |
| `src/net/app_httpd.cpp` | HTTP + WS（端口 81：文本=指令/状态 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报（默认关，`/log exec on` 后约 400ms 一条、状态文本相同则跳过、**OTA 期间停推**；状态缓冲须容下含抓手前端的整行）。注册 `blog` 日志转发器（WS+BLE）。`ws_stream` 任务栈 8192（推流 + 状态上报共用）。**端点**：`/`(302→/stream) `/status` `/control` `/capture` `/zoomshot` `/stream`(MJPEG) `/bmp` `/xclk` `/reg` `/greg` `/pll` `/resolution` `/ai_dump` `/ai_frame` `/coredump`；`POST /update` 实现在 `ota.cpp`。⚠️ `/zoomshot` **只认 `out_w`/`out_h`/`quality`**，固定裁中央 `(0.25,0.25)-(0.75,0.75)`（与 AI 的 `zoom` 同框） |
| `Calibration.h/.cpp` | **手动校准数据集中区**（根目录）：舵机限位/机械臂参数（STEER/REACH/GRIP/LIFT、ARM_*）、固定位姿 `ARM_LOW_X/H_CM`(8.0/1.0) 与 `ARM_RAISE_X/H_CM`(9.0/9.0)、`GRASP_LIFT_CM`(7.0)、定距表 `MV_SPEED_X/Y` + `MV_COAST_X/Y`（**4 档**，档位 `{0.15,0.25,0.5,1.0}`）与 `MV_START_MS`/`MV_MIN_PULSE_MS`、定角**查表** `SPIN_TBL_DEG/MS`（10 点插值）、`SPIN_MIN_SPEED`/`SPIN_PIVOT_BEHIND_CM`、屏幕→地面单应标定点 `kGroundCal`、机械臂夹心散点 `kArmPts`；`bivar::arm_set()` 实现在 Calibration.cpp |
| `partitions.csv` | 分区表（sketch 自带，**覆盖** fqbn 的 `huge_app`）：app0/app1 双 OTA 槽各约 3.8MB + `coredump` ⇒ OTA 可用、panic 可落盘 |

## 直驱执行层要点（`exec` / `nezha`）

- **接线**：哪吒 SCL ← GPIO47，SDA ← GPIO14；I2C 速率 ≤200kHz，软 I2C 开漏实现。
- **物理映射**：四轮 M1左后 / M2右后 / M3右前 / M4左前（左轮 `a` 正前、右轮 `b` 正前）；舵机 Servo1 转向 / Servo2 移爪 / Servo3 夹爪 / Servo4 抬落。
- **标定限位**（`Calibration.h`，实测）：转向 146/120/180；移爪中位 200（120..250）；夹爪紧 50 / 松 140；抬落中位 180（115..250）。
- **连续动作**：`lift_up/down`、`reach_forward/backward` 为持续型——`update_tick()` 每拍按 `ARM_STEP_CM`(0.5cm) 沿目标轴步进并保持另一维（reach 保持高度 h、lift 保持 x），经 `arm_pose` 反解联动双舵机下发；到边界自动停的判据**只看被步进的那一轴**，收到 `stop(scope="arm")` 或离散动作时清除。离散动作：`clip/release` 置端、`fold` 收臂折叠回平台位、**`low`** 降到标定低姿夹取位、**`grasp`** 合爪+定量抬升（`GRASP_LIFT_CM`，抬升在 `update_tick` 异步完成）、**`raise`** 抬到标定固定高位（与持续 `lift_up` 的区别：单发目标走 S 形缓动，不会在边界 IDW 两解间来回跳导致舵机抽搐）。
- **状态机 `s_arm_mode`**：六态 `low/raise/clip/grasp/release/fold`；移动臂位（`arm_pose`/持续步进）会把模式清回普通。
- **二连杆 IK**：`arm_pose(x,h)`（x=轴前方 cm，h=地面以上 cm）反解 α/β 后查标定表联动左右两舵机；L1=L2=7.5cm、肩轴离地 9.5cm、可达半径 4..15cm。
- **无里程计（电机无编码器）**：定距/定角不做闭环，一律按**实测标定表的时长近似**到点自停——`move` 带 `distance_cm` 时按 `MV_SPEED_X/Y`（油门→cm/s）与 `MV_COAST_X/Y`（起停余量）换算时长（需 `cm>0` 且油门非零）；`spin` 带 `angle_deg` 时在 `SPIN_TBL_DEG/MS` 上**查表插值**；`arm` 的 `dist_cm` 按每 cm ≈ `ARM_CNT_PER_CM` 拍近似。不带定距/定角即为持续动作，靠 `stop` 收尾。
- ⚠️ **定距必须先降档再写电机**：`pick_throttle_for(a, cm)` 取「起停余量 `c` 小于本次距离」的最低已标定档，且**写进电机的油门与算时长的油门是同一个值**。否则 `plerp` 在首档以下整段钳位，`cm ≤ c(a)` 时脉冲会塌成 `MV_MIN_PULSE_MS` 的满油门一冲——AI 以为在做厘米级微调，实际是一脚油门。请求油门低于最低档（AI 偶发 0.1）时抬到最低档，**表外不外推**。
- ⚠️ **`spin` 的持续旋转没有时限兜底**：`dir≠0` 且不给 `angle_deg`/`ms` 时**什么都不设**，只能靠显式 `stop` / `spin dir=0` 收尾。AI 侧靠 `t_move` 自动补 `AI_SPIN_DEFAULT_DEG`(30°) 兜底，**裸摇杆不加 `angle_deg` 会一直转**。
- **原地旋转**：`spin` 的 `dir` = `+1` 右转（顺时针，左轮进/右轮退）/ `-1` 左转 / `0` 停（显式写 0 速度）；`dir≠0` 时转速低于 `SPIN_MIN_SPEED` 会被抬高（低于实测拖动线拖不动）。另有 `ms` 口（直接指定通电毫秒，**优先于** `angle_deg`，供标定/调试）。`move` 与 `stop` 都会清掉旋转状态。
- **状态行**：`read_state()` 合成 `小车: <停止/前进/后退/原地左转/原地右转> | 夹爪: <clip/release> 姿态: <自由/low/raise/grasp/clip/release/fold> 位置: (<x>,<h>)<已伸最远/已缩最近/已触底/已触顶>`，末端位置由正运动学算出。⚠️ 状态串口径变更后，凡按旧串解析的地方（手机端、`tools/`）都要同步。
- **灯初始状态**：`exec::init()` 主动把三盏灯都写成关并清本地 `s_light_*`——哪吒寄存器会保持灯态，MCU 重启后不同步。
- **手动指令优先**：`move/stop/arm/drive/spin/servo/motor/arm_pose/reset` 视为手动接管，先 `ai::cancel` 再落地；`arm`/`arm_pose` 只停轮子（`StopMode::Wheels`），其余类型用户指令已覆盖故不补停。手动路径同时清掉上一轮遗留的定距/定角时限，避免旧时限在新指令之后误停。

## AI 层要点（`ai`）

- **给模型的工具（7 个，与手机端词表是两套，勿混）**：`car` / `mem` / `task` / `goal` / `look` / `say` / `compact`。声明在 `ai_prompt.cpp::tools_schema()`（请求体尾部的 `"tools":[…]`，放在 messages 之后以保住前缀缓存）。
- **键级校验表**：`src/ai/tools/registry.cpp` 的 `kTools[]`（10 键，键名全局唯一）。⚠️ **表序只决定校验顺序**（`validate_cmd` 按表序跑各 `parse`）；**落地顺序是另一处**：`ai_round.cpp` 的 `dispatch_calls` 固定为 mem → task → say → car → look → compact → goal，`car` 内部再由 `land_car` 排。改任一顺序前先读 `tool.h` 头注与 `dispatch_calls`。
  - 现阶段 `ToolSpec` 只填 `parse`，`doc`/`run`/`feedback`/`logfmt` 留 nullptr 是**有意的**（文案在提示词、落地手写）。所以**加一个键要改四处**：① `t_*.cpp` 写 `parse_*`；② `tool.h` 加声明；③ `registry.cpp` 插一行；④ `ai_prompt.cpp` 的 schema。`tools_selfcheck()` 只打日志**不 assert**（板子在车里，带病上电也比 panic 好救）。
  - ⚠️ `feedback`/`logfmt` 拿到的 `cmdD` 是 **const** `JsonDocument` ⇒ 类型检查只能用 `is<JsonObjectConst>()`/`is<JsonArrayConst>()`；写成 `is<JsonObject>()` **恒 false 且照样编译过**（见 memory `arduinojson-const-variant-is-false`）。另：`parse` 返回的指针**不保证**指向 `err`，允许直接返回静态字面量，调用方只许当只读串用。
- **各工具参数**：
  - `car`：`move{type,value,target}`（`forward`/`backward` → `value`=cm；`spin_left`/`spin_right` → `value`=角度；`approach` → `target`=记忆名，缺省最近目标）· `arm{type,x,h}`（`low`/`raise`/`fold`/`grasp`/`clip`/`release`/`pose`，`x`/`h` 仅 `pose` 用）· `light{kind,on}`（**`on` 必须显式给 bool，缺省直接拒**——防 AI 漏写把灯误关）
  - `mem`：`observe[{name,px,py}]`（px/py 填目标底部中心，放大图也照常填 0~1，程序换算）/ `delete[名字]` / 都不给 = 查询（回小车全局位姿与全部记忆）
  - `task`：`note`（字符串）· `todo[]`（整表重写）· `done[]`（编号，**1-based**，取消标记要重写整表）
  - `goal`：`set`（改目标）· `finish` = `done`|`fail`|`wait`（非法值**明拒**）
  - `look`：`zoom`(bool) · `image[]`（编号回看，最多两张）｜`say`：`text`（必填）｜`compact`：`summary`
- **`approach` 与合爪不能同轮**：`grasp`/`clip` 与 `approach` 同轮会被拦（提示 AI 先 approach、下轮据画面确定是否对齐再夹）；`arm low/fold/raise`、`light` 可与 `approach` 并行。
- **`goal.finish="wait"`**：AI 可主动暂停任务**并等用户输入**（`AI_WAIT_USER_MS`=60s；等待期不发请求，只轮询插话/代际/超时，超时后告知它继续；面板同期转「等你输入」）。
- **⚠️ 程序会补默认值 = 改写模型意图**：`car.move` 缺 `value` 时补 `AI_MOVE_DEFAULT_CM`(8cm)、缺角度补 `AI_SPIN_DEFAULT_DEG`(30°)，补写点在 `parse_move`（`t_move.cpp`，move 的唯一规范化出口）。而"不写"的原意本是**持续动作**（靠后续指令收尾），故这是已知坏味道。
- **同源不变量**：程序喂回 AI 的措辞、程序给出的补救动作、**日志的措辞**必须互相自洽——AI 能看见的只有这些字，说法不一致它就会做出与程序预期相反的动作。
- **每轮上下文**：`build_state_block()` 现拼「车臂状态 + 目标 + 任务列表 + 任务笔记 + 提醒」挂到本轮**最后一条工具结果尾部**（只进请求、不写历史）；历史按**回合**环缓存（一回合 = 1 条 assistant(tool_calls) + N 条 tool 结果），`reasoning_content` 每轮原样回传；`AI_HIST_SOFT_TURNS`(30) 到线后每轮催 `compact`，`AI_HIST_MAX_TURNS`(200) 兜底丢最旧整回合。图只对**最新一组实景**注入字节（先前帧环 3 槽 + 用户参考图，统一 `ImageN` 编号），更早的按编号渲染成占位。
- **空转与死循环**：连续 `AI_IDLE_ROUNDS`(5) 回合只有只读调用 ⇒ 提示它环视搜索；连续几回合重复同一批动作 ⇒ 尾部提示换个角度看目标。
- **请求尾固定约束**：最后一行要求"思考与发言都用简体中文"——模型上轮的英文 reasoning 会原样回喂、形成语言自我强化，贴在末尾（recency 最强）才压得住。
- **`say` 与 `compact`**：`say` 是模型唯一的自然语言出口（推 `ai_result` 给手机，并在历史里复述原话作中文示范，`finish` 前必须先 say）；`compact` 用一段摘要清掉更早的对话（目标/列表/笔记/记忆/位姿保留，可回看的旧图作废）。
- **`look` 的 `zoom` 语义**：`{zoom:true}` 新拍一张**中央放大图** `(0.25,0.25)-(0.75,0.75)`（`AI_ZOOM_DEF`=2）、`false` 新拍全幅。**框与倍数不由 AI 给**：程序按"已发出的框"把 AI 在放大图里报的 0~1 坐标算术换算回全幅（`to_full_x/y`），故 AI 照常填 0~1；框**绝对**不累乘。同块画面车臂未动却反复索要超过 `AI_ZOOM_NOOP_MAX`(2) 次 ⇒ 强制回全幅。
- **车灯**：`kind` 分 `front`（前灯，**白色**）/ `back`（尾灯，**红色**）/ `vibe`（侧面氛围灯，**深蓝色**）——口径与 `ai_prompt.cpp` 给模型的说明一致。**照亮与判断颜色用 `front`**（唯一白光）。⚠️ 测"灯开没开"**别看绝对亮度帧差**（环境光十几秒内就会漂），要看**色比**；增亮多少取决于环境光，别用绝对亮度判灯态。
- **抓取范式**：用户给定的目标范式与代码现状的对照写在「已知问题」第 1 条——**改抓取相关提示词前先读那一节**（现行提示词是围绕 `[左指]` / `[夹爪前端]` 的状态判定式）。
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
⚠️ 但**脚本内部不能把代理一概关掉**：直连与代理两条路**快慢会反过来、且直连会间歇性挂住**（实测直连稳定慢到 10.7s、裸 socket 直连超时，走代理 0.11~0.20s 且内容是板子那份）。故 HTTP 取数**两条路都留**并把选中的记进 `_ROUTE`（首次两条都短试定下来，之后直接用那条，免得每次长轮询都在坏路上白等一个短超时；已定路失败时另一条仍会被试到并改记）。⚠️ **"板子 HTTP 超时"十有八九是客户端选错了路，不是板子抽风**——见 memory `board-http-pc-side-route`。

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
| `warns` | 补 `-Wall` 重编本草图，把编译器告警揪出来（**须先 `build` 一次**，靠复用编译缓存） | `-f/--filter SUBSTR`（只查文件名含此子串的编译单元）、`--only-trunc`（只看 `-Wformat-truncation`）、`--limit N` |
| `build` | 用与 IDE 逐字一致的 fqbn 编译（见「构建要点」） | `--clean`（全量重编）、`--fqbn FQBN`（显式指定，默认自动取 IDE 日志里那串以共用构建缓存）、`--flash`（编完直接 OTA + 核对指纹）、`--force`、`--wait-up SEC` |
| `ota <bin>` | 推固件 → 等板子回来 → **核对指纹** | `--dry-run`（只报告不上传）、`--force`、`--no-wait`、`--wait-up SEC` |
| `coredump` | 取 panic 现场并解栈（默认按 coredump 回传的 sha 去归档里自动找 `.elf`） | `--erase`（取完清现场）、`--elf PATH` |
| `stress` | **abort 风暴**：AI 在途时反复中止，逼出跨任务竞态（改并发 / 修 panic 前后各跑一次对比） | `--rounds N`、`--gap SEC`、`--abort-ms MS`、`--sample SEC`、`--type {ai_oneshot,ai_goal}`、`--goal TEXT`、`--keep-panic` |
| `frame` | 抓一张（或连拍）画面——手动操作时的眼睛 | `-o/--out PATH`、`-t/--tag TAG`、`-n N`（连拍，看运动用）、`--gap SEC`、`--tries N` |
| `zoomshot` | 板端 `/zoomshot` 裁出**中央**放大一张 —— **手动复核"AI 看到的放大图对不对"**，与 AI 的 `zoom` 同一条链路（板端同一实现、同一固定框） | `--out-w`/`--out-h`/`--quality`、`-o/--out PATH`、`-t/--tag TAG`。裁框固定中央 `(0.25,0.25)-(0.75,0.75)`，不接受指定区域/倍数 |
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
uv run tools/carctl.py zoomshot -t mag3           # 手动放大中央一块，看 AI 的放大图能不能读
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
  1. 板端固件**有 `ai_dump` 模块** —— 记录仪启动时会先探一次 `/ai_dump`，但这一问是**三态**（`has_dump`）：
     - `True`（回了 `{"slots"…}` 清单）/ `None`（**超时、连不上**）⇒ 都**照起**归档线程；
     - 只有 `False`（板子**明确**回了非清单应答，典型 404）才判"固件没带这个入口"并**全程不抓帧**（`car_logcat.py:1102`）。

     ⚠️ **`None` 绝不能当"没有"**：一问问不到就锁死会**白白丢掉整场的画面**（实测踩过——刚开 `/log ai on`
     之后那一问板端还一轮没跑完、HTTP 池也常一时忙），归档线程自己会对失败退避重试，且首次清单从环里
     **最旧**那帧起回填，晚起步也不会从中间开始丢。该探测**与 `/log` 开关无关**，只反映固件版本。
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

> 均为真机实测 / 源码核对所得。**本节目的是交接——只列仍未解决、或需要留意的**。

### 1. 抓取流程：范式（用户给定）与代码现状对照

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

#### 1.2 代码现状（`18ef9c1` 核对）

对准/夹取判定已整段重写为**状态式**（`ai_prompt.cpp` 的「物体对准及夹取判定」）：按目标相对 `[左指]` 的位置分档（正上方 / 水平正右方 / 偏左偏右过近），结合 `arm low` 与"目标是否进入 `[夹爪前端]`"决定下一步。范式 3 的"先降爪、后对准"落成"需要夹取且目标未进入 `[夹爪前端]` 时，可先 `arm low` 方便对准"。

- ✅ 范式 3/4 的循环语义保留：微调对准先给接近角度、过头就逐轮砍半反向；`zoom` 只用于检查能否夹取与夹后核对。
- ◑ 范式 2 的"2/3 高度 或 距离<15"不再是显式判据：机器侧只剩 `AI_APPROACH_STOP_CM`(20) 一个停距，其余交给模型看画面。
- ◑ 范式 6/7（丢失次序、`y<7` ⇒ 后退）没有专门判据，靠提示词里"机械臂遮挡 ⇒ 先 `arm low`/`fold`；`arm low` 后目标仍被遮挡 ⇒ 可能太近、适当后退"这类措辞。

位姿锚点：低姿 `ARM_LOW_X/H_CM`=8.0/1.0、固定高位 `ARM_RAISE_X/H_CM`=9.0/9.0、`GRASP_LIFT_CM`=7.0（`Calibration.h`）。

> 抓取流程里**唯一还在拦的机器侧规则**是「`approach` 与合爪不能同轮」（`ai_round.cpp`，防拿过时坐标空夹；降爪 `arm low` 被**放行**）。

### 2. 推理吃光 token 预算 ⇒ 整轮空跑（中）

日志：`[ai] 无工具调用: finish=length content=0B reasoning=30656B 补全/总=8192/14552`。
模型把预算全烧在 `reasoning` 上、`finish_reason=length`，该轮白跑。**根因在模型侧**，仍会偶发；方向是给正文留最小预算 / 缩短推理链 / 换非推理档。

程序侧已有缓解 + 诊断：请求体尾段带 `"reasoning_effort":"low"`（`ai_prompt.cpp` 的 `build_body`）；无工具调用时把 `finish_reason`、用量、content/reasoning 字节数一并打出（`ai_http.cpp::extract_tool_calls`），并在 `ai_round.cpp` 判为"无有效输出"退避重试。
⚠️ 同一判据还覆盖另一种情形：**模型只回自然语言、一个工具都不调**（`ai_round.cpp` 的 `nc<=0` 分支，文本会被丢掉、最后以"云端持续请求失败"中止）——提示词里那句"每轮至少调用一个工具 / 说话用 `say` / `finish` 前必须先 `say`"就是压这个的。

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
| `move` | `throttle`(-1..1) `steering`(-1..1)、可选 `distance_cm` | 四轮 PWM（×1000）/ 转向舵 146±30；带 `distance_cm` 按标定表换算时长到点自停 |
| `stop` | `scope` = all/wheels/arm | 停轮 或 清连续机械臂动作 |
| `spin` | `dir` = `+1`右转/`-1`左转/`0`停、`speed`(0..1000)、可选 `angle_deg`、可选 `ms` | 左右轮反向 PWM（原地旋转）；`angle_deg` 查表插值换算时长到点自停；**`ms` 优先于 `angle_deg`**（显式通电毫秒，跳过角度换算与滑行补偿，标定口用）；**两者都不给 = 持续旋转，不自动停**，只能靠后续指令收尾 |
| `arm` | `act` = lift_up/lift_down/reach_forward/reach_backward/**low**/**clip**/**grasp**/release/**fold**、可选 `dist_cm` | 舵机 2/3/4。`low`=低姿夹取准备位、`grasp`=合爪+定量抬臂一步到位（省一轮往返）、`fold`=收臂折叠回平台位。⚠️ AI 侧另有 `raise`/`pose` 两个 act（`low` 也走这条），**不在这张表里**——它们在 `ai_round.cpp` 直接调 `exec::arm_low/arm_raise/arm_pose`，不经词表 |
| `light` | `kind` = front/vibe/back，`on` | 哪吒灯命令字节 |
| `reset` | — | 四舵机回中 + 电机 0 |
| `servo` / `motor` / `drive` / `arm_pose` | `servo`:`n`(0..3) `pwm`(50..250，不过标定限位)；`motor`:`n`(1..4) `a` `b`；`drive`:`speed`；`arm_pose`:`x`车头前 cm `h`离地 cm | 调试直驱（绕过上层语义） |
| `stream` | `on`、可选 `udp_port` `src_ip` | 图传开关（UDP 目标由板子据此建立） |
| `log` | `cat`=exec/ai/all，`on` | 统一日志转发开关（默认全关；exec=执行日志+周期状态推送、ai=AI 调试日志、all=板端串口全部输出转发手机；经 `blog` 统一队列 `{type:"log",params:{src,text}}` 上抛）|
| `get_state` | — | 回 `{"type":"state","params":{"bits":…}}` 位图（灯光/夹爪/AI busy，与手机 `Main.gd::_apply_state_bits` 逐位 mirror，改一侧必改另一侧；`cmd::state_bits()` 是唯一出口，重连后同步按钮用） |
| `nz_read` | — | `nezha::probe` 探测哪吒板 I2C 在线/ACK，结果以 status 文本回 |
| `pong` | — | 手机 WS 活体探测的应答，**回包带 `bits`**——每次 pong 都是当场现测的板端状态，是"发送按钮没在任务运行时变成中止"那类不同步的兜底来源（手机 `WSCarClient` 把 pong 上抛供 `bits` 同步） |
| `config` | `ssid` `password` | NVS + 在线换网 |
| `reboot` | — | 远程重启板子（链路卡死时的解药；升级中会被拒） |
| `ping` | 可选 `target` | 无目标回 `pong`；有目标走 `ping_svc` |
| `ai_goal` / `ai_oneshot` / `ai_cancel` | `message`、可选 `annotation` `use_image` | `ai_client` DIRECT 闭环 |
| `ai_chat` | `message` | 任务进行中插话补充（`ai::append_chat`，不打断闭环）；当前无 AI 任务则忽略 |
| `goto` | `x` `y`、可选 `frame`=local(默认)/global | `ai::goto_target`：由板端自行导航到指定坐标 |

**板端 → 手机（上行）**：`status`（指令回执，带 `bits`）· `state`（`get_state` 回包，带 `bits`）· `pong`（保活应答，带 `bits`）· `log`（`{src,text}`，按 `/log` 开关转发）· `exec_status`（周期状态文本，默认关）· `mem`（各堆/DMA 水位诊断）· `ai_result`（AI 回执：`reason` / 内嵌 `command` / `done`；`say` 与结束语都走它的 `reason`）· `ai_tool`（每个工具落地的进度行，**不走 `/log` 开关**）· `ai_task`（任务面板快照：`{state,round,goal?,note?,tasks:[{name,done}]}`，供手机端顶部悬浮面板）。

⚠️ 保活/纯查询三件套（`ping` / `pong` / `get_state`）**不打「收到…」那行日志**——手机每几秒就一来一回，打出来只是刷屏，而它们的应答本身就是自描述的状态。

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
