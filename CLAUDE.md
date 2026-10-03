# CLAUDE.md（仓库根 · 总览与索引）

> 本文档基准：仓库 HEAD `f7d7b6b`（2026-10-03）。只覆盖已提交内容；未提交改动不收录。

本仓库是「视觉控制小车」协作工程的**容器仓库**，现收拢两块子工程：

| 目录 | 角色 | 硬件 / 技术栈 | 权威文档 |
|---|---|---|---|
| `Stm32-Vision/` | 视觉/控制大脑板**兼执行器**（采集画面→AI→**直驱**电机/舵机→上报状态） | ESP32-S3-CAM（N16R8 + OV3660）· Arduino（esp32 3.3.x） | [`Stm32-Vision/CLAUDE.md`](Stm32-Vision/CLAUDE.md) |
| `Mobile-RemoteCtrl/` | 手机遥控 App（图传/摇杆/指令/配网，词表源） | Android · Godot 4.7.1 mono + GDScript | [`Mobile-RemoteCtrl/CLAUDE.md`](Mobile-RemoteCtrl/CLAUDE.md) |

**动手改某个子项目前，先读它自己的 `CLAUDE.md`**——每份都是该子工程的权威指南（目录职责、构建入口、调参宏、已知坑），本文件只做跨工程总览与索引，不重复细节。

## 一句话架构与数据流

手机经 BLE 给大脑板配网（广播名 `VisionS3`）；连上后走 WiFi，WebSocket（端口 81）承载指令/状态/消息（文本 JSON），JPEG 图传帧改走 UDP；大脑板把词表指令**就地翻译成哪吒扩展板的 I2C 命令**（舵机 PWM / 电机 PWM / 灯光字节）直接落地，状态由本板 `exec::read_state` 合成后回推——**全链路只有一块可编程板**。

- **大脑板既是视觉大脑也是执行器**：`command` 收到手动指令后直接调 `exec::act`。
- **词表 JSON 只由手机 App 持有**（`net/proto/CommandProto.gd`）；`词表 → 哪吒 I2C 命令` 的翻译在大脑板 `command` + `direct_exec` 一侧。
- **两条视觉链路并存**：① **AI 闭环**（`ai_goal`/`ai_oneshot`，云端每轮决策，看图 ⇄ 单应 ⇄ 车头系记忆）；② **板端本地追踪 + 全闭环自动夹取**（`src/ai/track.*`+`dcf.*` 相关滤波逐帧跟 → `src/ai/grasp.*` 自己降臂/对准/前进/合爪，**纯画面坐标闭环、不经 AI、不用单应**），入口 `auto_grasp`（手机框选或 AI 工具）。后者是"夹个方块"的快路径（每步 ~0.5s，AI 每轮数秒）。
- 当前状态：BLE 配网、WS 指令/状态、UDP 图传、软件 I2C 直驱（舵机/电机/灯，含**原地旋转**与按实测时长近似的**定距/定角**）均已接通；板载 DIRECT AI 已实现并按任务闭环调用 `exec`，其侧维护画面单应标定 + 物体空间记忆 + 车姿态累积，并按需携带上一帧做运动对比；**会话上下文跨任务保留**（任务结束不释放，接新目标即延续；只有手机 `/clear` 才重置）。手机端不持有云端 AI 客户端（DIRECT 不经手机侧）。AI 的可用动作由板端 **`src/ai/tools/` 工具表**统一登记（一工具一文件 + 有序 `registry.cpp`，**表序 = 校验顺序**），含 `move`/`arm`/`light`/`auto_grasp`（本地闭环夹取）/`zoom`（放大镜，**只有一个 bool**，固定裁中央框）等 11 个键；任务进度以 `tasks` 列表回报，另有 `goto`（直移到指定坐标）与 `ai_chat`（任务中插话）两条入口。
- 手机端另有**会话录制与回放**（`state/Recorder.gd`，逻辑时钟只在 AI 运行期前进，空闲等待自动抹掉）、**记忆地图**（画面源可切"图传 ↔ 车头系俯视图"）、**追踪可视化**（十字准星，跟踪期板端不推视频、改显标尺网格 + 跟踪点）。

## 各子工程文件索引（简）

### Stm32-Vision/ — 视觉大脑板兼执行器（Arduino sketch；源码在 `src/` 按模块分子目录，根目录留 `.ino`/`partitions.csv`/`build_opt.h`/`Calibration.*`）

初始化序：cfg→ble→**exec**→cam→net→web→ai→**grasp**。详见其 [CLAUDE.md](Stm32-Vision/CLAUDE.md) 的「代码库结构」表。

| 文件 | 作用 |
|---|---|
| `Stm32-Vision.ino` | 入口（setup 按序初始化并 `grasp::init()`；loop 调各模块 update + `exec::update_tick`；⚠️ setup 末尾抬 loop 任务优先级到 6——它是所有带时限动作的唯一计时器，被同核任务抢会造成定距脉冲拉长/缓动卡顿） |
| `src/cam/camera(.h/.cpp)` `camera_pins.h` | 摄像头初始化 + 取帧（VGA `fb_count=3` + `WHEN_EMPTY`）。**格式常态 = JPEG 直出**，本地追踪期才硬切 **RGB565**（`set_track_mode`，deinit→init、幂等、失败重试 3 次）。三取帧入口要分清：`grab()` 可能拿到队列里压着的旧帧、**「此刻画面」走 `grab_fresh(drop)`**（排空后阻塞等新帧）、`publish_latest`/`latest_copy` 是图传任务发布的最新帧（`/capture` 用它）。`request_hires()` 临时切 **SVGA** 高清抓一帧再切回（放大镜用，全程持 `s_cam_mtx` 串行化；**切回失败会重试 3 次**，否则后续放大镜全废）。⚠️ **psram 直写恒关**（实测丢字节 + 与非 psram 来回切后 VGA 再也取不到帧）；`wait_dma_block()` 等够连续内部 DMA；`reinit_current()` 兜底"init 报 OK 却不出帧"。引脚定义唯一配置点在 `camera.h` 顶部选 `CAMERA_MODEL_*` |
| `src/cam/camera_index.h` | Web 前端页面（源自例程，现基本不用） |
| `src/net/config(.h/.cpp)` | WiFi / AI 接口参数，NVS 持久化 |
| `src/net/wifi_net(.h/.cpp)` | STA 连接 + 断线重连（namespace `net`，勿改回 `network`） |
| `src/exec/direct_exec(.h/.cpp)` | **执行器直驱层**：move/stop/arm/arm_pose/light/reset/spin 落地到哪吒板；机械臂二连杆 IK + 连续动作步进 + `fold` 收臂 + 固定位姿 `arm_low()`/`arm_raise()` 与 `arm_moving()` 查询；离散定位走 S 形缓动（`ARM_SMOOTH_PEAK`）；定距/定角到点自停（`spin` 按实测**查表**插值，`ms` 优先于 `angle_deg`）；本地合成状态文本（含正运动学末端位置） |
| `src/exec/nezha_direct(.h/.cpp)` | 哪吒扩展板软 I2C 驱动（舵机 / 电机 / 灯），协议与硬件一致 |
| `src/exec/bivar(.h/.cpp)` | 机械臂夹心坐标散点反距离加权(IDW) 双向插值（FK/IK）；散点 `kArmPts` 与 `arm_set()` 在根 `Calibration.h/.cpp` |
| `src/core/command(.h/.cpp)` | 统一词表 JSON 分发（与传输解耦）；手动指令先 `ai::cancel` 打断 AI 再落地；`ai_goal`→`ai::set_goal`、`goto`→`ai::goto_target`（直移坐标）、`ai_chat`→`ai::append_chat`（插话）、**`ai_clear`→`ai::session_clear`**（开新会话）、**`auto_grasp`→`grasp::request`**（本地夹取）、`reboot` 远程重启（升级中拒）；统一日志指令 `log`（`blog` 模块）、`get_state` 查询回包、`nz_read` 哪吒 I2C 在线探测、`mem` 堆水位诊断（⚠️ 只回 JSON、别调 `heap_caps_dump_all`）。`cmd::state_bits()` 是状态位图的**唯一出口**（灯光/夹爪/AI busy，与手机 `Main.gd::_apply_state_bits` 逐位 mirror）；**`status` / `get_state` / `pong` 三类回执都带 `bits`**（`pong` 每次现测，手机据此纠偏按钮），且 `ping`/`pong`/`get_state` 三件套**不打「收到…」日志**（手机每几秒一来一回，打出来只是刷屏） |
| `src/core/board_log(.h/.cpp)` | 统一日志模块（namespace `blog`）：所有串口调试统一经 `logf`（带来源标记；另有收已格式化整行的 `log_line`），按 `/log` 开关（exec/ai/all）经队列+转发任务把 `{type:"log",params:{src,text}}` 发手机（WS+BLE）；`forward_text` 发任意长度文本 |
| `src/net/ping_svc(.h/.cpp)` | `/ping <目标>` 异步 ICMP 探测（无目标则就地回 pong） |
| `src/net/ble(.h/.cpp)` | BLE GATT Server：配网 + 兜底控制 + status 通知（广播名 VisionS3） |
| `src/net/app_httpd.cpp` | HTTP（MJPEG/拍照/LED/**追踪自检 `/tracktest`**）+ WS（端口 81 文本 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报 + **`track` 位置 10Hz 上报**（`novid=1` 表示跟踪期不推视频）+ `ai_mem` 之外的本地上行。⚠️ 图传软编与跟踪喂帧都**钉在 core 0**（留在 core1 会与追踪/exec 抢 CPU，实测取图耗时 85ms→925ms） |
| `src/ai/ai_client(.h/.cpp)` | 板载多模态 AI **外壳**（DIRECT 直调云端）：worker 任务 / generation（打断世代）/ 对外接口（含 `session_clear` 开新会话）。⚠️ `update()` 与 `logf()` 的**实现不在这个文件**（在 `ai_result.cpp`）。任务是模块化的：轮次主体在 `ai_round`、回执与日志在 `ai_result`、校验在 `ai_validate`、导航在 `ai_nav`、**本地追踪在 `track`+`dcf`、闭环夹取在 `grasp`**、词表在 `src/ai/tools/` |
| `src/ai/ai_round(.h/.cpp)` | **一轮任务的主体**（自 `ai_client` 拆出）：`round_run_task()`、`round_prepare` / `round_attempt` / `dispatch_calls`（**落地次序固定** mem → task → say → car → **auto_grasp** → look → compact → goal）/ `build_state_block`（车臂状态+目标+任务列表+笔记，挂本轮最后一条工具结果尾部）。⚠️ `RoundCtx` **按值放在 16KB 的 PSRAM 任务栈上**（任务列表/笔记/目标是 PSRAM 变长文本，无项数/字数上限）。**会话上下文 `g_sess_*` 跨任务保留**，只有 `ai_clear` 才重置。落轮的硬约束：`approach` 与合爪/放收**不能同轮**、`auto_grasp` 与 `car` 不能同轮（防拿过时坐标空夹；降爪 `arm low` 放行） |
| `src/ai/ai_validate(.h/.cpp)` | AI 输出校验入口：剥代码块 → 解 JSON → 拒旧版扁平格式 → **按工具表逐项规范化**。⚠️ 词表白名单**不在这里**（在 `src/ai/tools/`）；校验器对每个调用都跑**全表** parse，故每个 parse 都得先自查"本工具有没有被调用"。保留两条硬钳制：`AI_MOVE_MAX_CM`(40) / `AI_SPIN_MAX_DEG`(180) |
| `src/ai/ai_result(.h/.cpp)` | 结果回执与 `ai::update()` / `ai::logf()` 实现、编辑图入库（`ResultItem` 走 PSRAM，否则 AI 任务期间内部 DMA 池会被打崩） |
| `src/ai/ai_nav(.h/.cpp)` | AI 侧导航（`goto` 直移坐标的落地）+ 出帧前等车/臂停稳 |
| **`src/ai/track.h/.cpp`** | **板端本地目标追踪**（namespace `track`）：抽 luma/chroma → 交 `dcf` 逐帧定位；`seed`/`stop` 硬切相机 RGB565⇄JPEG；`hint_motion`/`hint_wide` 把"这一脚动作后目标大致挪到哪"告诉追踪器（比它自己的图像预测可靠）；`light` 轻量刷新只更显示位置。诊断出口分列（`last_capture_ms` 拍摄时刻 / `last_ok` 本帧是否被采纳 / `last_appear` 外观分 / 分段耗时） |
| **`src/ai/dcf.h/.cpp`** | **相关滤波跟踪内核**（MOSSE 思路，自写 radix-2 FFT）：整幅响应图一次 FFT、每帧在线更新、PSR 当置信度、输入窗归一化到 32×32 网格故尺度无关。⚠️ 三道防"锁到自车"闸：种子外观锚定（不达标**不训练滤波器**）、车身 mask、色度加权门（只做否决）。⚠️ **PSR 不达标时位置/尺度/滤波器一律不动**（防死亡螺旋） |
| **`src/ai/track_mask.h`** | 车身 mask 表（按抠图真实轮廓、**只保留 v≥0.63 以下**）；起跟点在车身里则整场关闭 |
| **`src/ai/grasp.h/.cpp`** | **本地自动夹取闭环**（namespace `grasp`）：`arm low` → 转正对准 u → 边前进边对准到 v → 合爪，纯画面闭环。物理判据："**动作后目标必须动**"（相对期望位移）+ 横向漂移预算；合爪自检只出诊断、**不保证夹住** |
| `src/ai/tools/` | **AI 工具表的键级定义**：一工具一文件（`t_move` / `t_arm` / `t_meta` / `t_tasks` / `t_observe` / **`t_grasp`**）+ `registry.cpp` 有序表（**11 键**，键名全局唯一）+ `tool.h`。⚠️ **表序只决定校验顺序**，落地顺序在 `ai_round.cpp` 的 `dispatch_calls`；给模型的 **8 个工具**声明在 `ai_prompt.cpp::tools_schema()`（键名/枚举改一处必改另一处）。现阶段只填 `parse`，其余留 nullptr 是有意的 |
| `src/ai/ai_prompt(.h/.cpp)` | 系统提示词（人设+规则+工作流程+画面/夹爪定义，含"优先使用 auto_grasp"）+ `tools_schema()`（请求尾的 8 工具声明）+ `build_body`（**碎片流**拼请求：静态段直接引用、历史按回合回喂、JPEG 边发边 base64；图只对最新一组实景注入字节，其余按 `ImageN` 渲染成占位；全幅 `detail:low`、放大图 `high`）。⚠️ **别把车自身全局坐标写进提示词**（里程会漂，会误导 AI 推距离） |
| `src/ai/ai_mem(.h/.cpp)` | AI 侧空间记忆 + 车姿态累积（`mem_reset`（**只在开新会话时**）/ `car_update_pose` / `mem_observe_xy` / `mem_feed`（**只喂朝向，不喂全局 x/y**）/ `mem_find` / `mem_forget`（删掉正在跟踪的目标时顺带停跟踪）/ `mem_tick_stale`、`mem_export`（供手机记忆地图） |
| `src/ai/ai_http(.h/.cpp)` | AI TLS 发送层（keep-alive 复用 POST、状态码/错误串供 4xx/429 快速失败与分诊、`http_stop` 中止在途请求供打断用）；`extract_tool_calls()` 解响应取 `calls[]` + `reasoning_content`，无工具调用时打出 `finish_reason`/用量 |
| `src/ai/ai_alloc.h` | ArduinoJson 内存池改用 PSRAM 的分配器（`g_js_alloc`，定义唯一处在 `ai_client.cpp`；`command` / `ble` / `ping_svc` / `app_httpd` 也共用）：避免小分配与 TLS 缓冲交错把内部堆切碎致握手失败 |
| `src/ai/ai_dump(.h/.cpp)` | AI 抓帧留档：把实际发往云端的那帧 JPEG + 该轮标注留在 PSRAM（6 槽环形），HTTP `/ai_dump` 供 PC 侧复盘 |
| `src/ai/magnify(.h/.cpp)` | 放大镜（namespace `magnify`）：`crop_center_jpg` —— 高清真拍（`cam::request_hires` 切 SVGA）+ TJpgDec **中央部分解码**，固定裁 `(0.25,0.25)-(0.75,0.75)` 再重编码 |
| `src/ai/ground_proj(.h/.cpp)` | 屏幕→地面单应换算（namespace `ground`），含反查 `world_to_screen()`，供 AI 用 |
| `src/core/heap_watch(.h/.cpp)` | 内部堆/DMA 块水位哨兵：跌到危险线告警。⚠️ 判"车要挂了"看 **DMA 块**而非总空闲堆——碎片化才是真闸门 |
| `src/net/ota(.h/.cpp)` | OTA：ArduinoOTA + HTTP `POST /update`（板子固定车上、串口够不着，刷固件走这里） |
| `Calibration.h/.cpp` | **手动校准数据集中区**（根目录）：舵机限位/机械臂参数、定距/定角移动时长表、屏幕→地面单应标定点 `kGroundCal`、机械臂夹心散点 `kArmPts`，**+ 本地追踪 `TRACK_*` 与自动夹取 `GRASP_*` 两组调参宏**（注释里大量记着"为什么是这个值"的真机实测，别当噪音删）；`bivar::arm_set()` 实现在 Calibration.cpp |
| `partitions.csv` | 分区表（sketch 自带，**覆盖** fqbn 的 `huge_app`）：app0/app1 双 OTA 槽各约 3.8MB + `coredump`；故 OTA 可用、panic 可落盘 |

> 🛠️ **PC 侧工具**：板子固定在车上、串口够不着，日常诊断/烧录/抓帧都走 [`Stm32-Vision/tools/`](Stm32-Vision/tools/)——`carctl.py`（操作台：状态/日志/发指令/抓帧/**追踪自检**/体检/编译/OTA/panic 取证/压测）、`car_logcat.py`（记录仪：日志落盘 + AI 每轮画面留档）、`probe_macro.py`（编译期宏探针）。**全部经 `uv` 直接跑，不必手打 arduino-cli**；逐条用法见 [`Stm32-Vision/CLAUDE.md`](Stm32-Vision/CLAUDE.md) 的「PC 侧工具」。

### Mobile-RemoteCtrl/ — 手机遥控 App（Godot 工程）

通信：BLE 配网/兜底控制、WiFi WS 指令/状态（文本 JSON）+ UDP 图传（连接策略统一收口 `net/DeviceConn.gd`）、云端 AI（DIRECT 不经手机侧）；`CommandProto` 为唯一命令词表。详见其 [CLAUDE.md](Mobile-RemoteCtrl/CLAUDE.md) 的「目录结构」。

| 路径 | 内容 |
|---|---|
| `Main.tscn/.gd` | App 壳（连接编排、左右滑动切页、**模式与画面源**、「关于」页设置项：自连 / 禁用自动 WS / 原地旋转模式）。控制页 `BodyControl`：`GeneralCtrl`（图传开关 + **模式切换**）/ `ImgEditToolbar`（标注 + **`GraspBtn` 自动夹取**）/ `Video`（+ `TrackOverlay` + `OpMap` 记忆地图）/ `AIModeCtrl`（画面源 / 会话入口 / 标注，仅 AI 接管模式显示）/ `ChatPanel` / `CtrlArea`（摇杆+臂+灯，仅手动模式显示）。**可见性一律经 `_apply_layout()` 收口**（模式/回放/图传/标注/画面源五个条件） |
| `ui/chat/ChatPanel.gd` | 聊天/指令区（消息日志、指令提示、附件列表、指令解析与发送；消息类含「AI」/「AI工具」/「状态」；解析逻辑在 SlashCommands.gd）。底部三页：指令提示 / 图片附件 / **回放控制条**；`save_current_session()` = 归档本趟 + 清场 + 下发 `ai_clear`（`/clear` 与「保存」按钮都走它） |
| `ui/chat/TaskPanel.gd` | AI 任务面板（悬浮在聊天区顶部，数据源 `ai_task`：折叠一行 + 点击展开任务列表/笔记；节点树在 `Main.tscn`，展开区高度脚本自算并封顶到聊天区、超出滚动） |
| `ui/replay/PlaybackBar.gd` + `SessionList.gd` | **会话回放**：进度条 + 上一步/暂停/下一步（锚在「AI 决策点」而非逐条日志）；会话列表 `Session1`=当前、`2..10`=归档（上限 9），带删除。⚠️ 回放是**只读**态：一切下发被 `DeviceConn` 拒发，`SessionBtn` 是唯一出口 |
| `state/Recorder.gd` | autoload **会话录制与回放**：`user://sessions/<id>/{meta.json, events.jsonl, frames/}`；**逻辑时钟只在 AI 运行期前进**（空闲等待在回放里被抹掉）；帧走 `DeviceConn.frame_jpeg`（解码前原始字节） |
| `state/LocalStore.gd` | autoload 本地持久化（last_device / wifi / ai 配置 / 设置项：自连·禁用自动 WS·原地旋转·图传开关·**ai_mode**（手动/AI 接管）/ 输入历史 `input_history`） |
| `state/AppLog.gd` | autoload 本地日志落盘（每次启动截断重写 `user://logs/app.log`；`clear()` 供 `/clear` 截断） |
| `net/proto/CommandProto.gd` | **统一命令词表**（static） |
| `net/DeviceConn.gd` | **统一连接层**：自建并持有 BLE/WS/UDP，收敛状态与重连策略（单一事实源；Main 只订阅其信号）+ send_command 统一出口（⚠️ **回放时一律拒发**）+ 最新帧 current_image + 原始 JPEG 旁路 `frame_jpeg`（录制用） |
| `net/ws/WSCarClient.gd` | WS 传输（端口 81 文本 JSON：指令/状态/`ai_task`/`ai_mem`/`track`；视频已走 UDP）。保活探测的 `pong` 由本类拦截判活，但**仍上抛给上层**——`pong` 带板端当场状态位，是按钮同步最及时的一条来源 |
| `net/ble/BLEClient.gd` + `BleProfile.gd` | BLE GATT 客户端；协议常量表 + BLE 可发类型**黑名单**（现仅 `stream` 图传被拦，其余类型均可走蓝牙兜底） |
| `net/video/UDPVideoClient.gd` | UDP 图传接收（JPEG 分片重组 → 上抛 frame_received；另有解码前的 `frame_jpeg` 旁路） |
| `ui/chat/SlashCommands.gd` | /指令 解析器（文本 → 词表指令/本地动作，纯解析） |
| `ui/bluetooth/ScanPanel.gd` | 蓝牙扫描页（设备列表/刷新动画/空提示，挂 BodyBTScan 节点） |
| `ui/` | 直控面板 `control/DirectControl.gd`（含摇杆映射，用 Godot 内置 `VirtualJoystick`）/ 图传 `video/VideoView.gd` + `GridOverlay.gd`（`/grid` 网格，**无画面时按 VGA 4:3 兜底**）+ `TrackOverlay.gd`（跟踪十字，**跟踪期板端不推视频**）+ `MemoryMap.gd`（记忆地图）/ 图片标注（工具条在 `Main.tscn`，`ImageEditor.gd` + `EditorCanvas.gd` 负责画布与图形，`GraspBtn` 取到区域即下发 `auto_grasp`）/ 通用弹窗 `PopupWindow.gd`（配网 + 确认两页；模型名从服务端拉列表下拉选）及其背景遮罩 `BgDimSharder.gd` |
| `res/` | `icon set.png` **雪碧图**（各 UI 图标用 AtlasTexture 切图，旧的十余张独立 png 已删）+ `bg.jpg` / `icon.png` / 中文字体 |
| `addons/gdble*` | GDBLE 蓝牙运行时（含导出插件） |

## 硬件基线（BOM 速查）

完整清单见根 [`README.md`](README.md) §3。要点：

- **大脑板**：ESP32-S3-CAM（N16R8）+ **OV3660** 摄像头；既跑视觉/网络/AI，也直接输出哪吒 I2C 命令。
- **驱动板**：哪吒（NeZha）扩展板，I2C 从机地址 `0x80`；**大脑板软件 I2C 直连**（SCL=GPIO47 / SDA=GPIO14），无 MCU 中转。
- **执行机构**：4× N20 直流电机（四轮，**无编码器**）+ 4× MG90S 舵机（Servo1 转向 / Servo2 移爪 / Servo3 夹爪 / Servo4 抬落）。
- ⚠️ 因电机无编码器，**直驱下没有里程计**：`move` 的 `distance_cm` 与 `spin` 的 `angle_deg` 不做闭环，按**实测标定表的时长近似**到点自停（粗用，非精确）；`arm` 的 `dist_cm` 按拍数近似。

## 跨子工程同步点（铁律）

改协议/常量前**必须**先读两份子 CLAUDE.md 的「协议参考」/「通信协议速查」，并同步相关侧：

1. **词表 JSON**（type / params 字段）：只由手机 `net/proto/CommandProto.gd` 定义 ↔ 大脑板 `command.cpp` 的 `type` 分支 + `direct_exec.cpp` 的 params 解析，**两侧逐字对应**。改一侧必改另一侧。现行类型：`move`（可选 `distance_cm` 定距，时长近似）/ `stop` / `arm`（act = `lift_up/lift_down/reach_forward/reach_backward/low/clip/grasp/release/fold`）/ `spin`（可选 `angle_deg` 定角、可选 `ms` 直给通电毫秒，**板端 `ms` 优先**）/ `light` / `reset` / `stream` / `log`（`cat=exec|ai|all`、`on`，统一日志转发开关）/ `get_state`（回 `params.bits`）/ `config` / `ping` / `pong`（**带 `bits`**，手机据此同步按钮）/ `ai_goal` / `ai_oneshot` / `ai_cancel` / `ai_chat`（`message`，任务中插话）/ **`ai_clear`**（开新会话：清历史/笔记/物体记忆/车位姿）/ `goto`（`x`/`y`，可选 `frame`=local/global，直移到坐标）/ **`auto_grasp`**（`x`/`y` 目标画面中心 + 可选 `w`/`h`/`name`，**本地**闭环夹取，不经 AI）/ `nz_read`（哪吒 I2C 在线探测）/ `reboot`（远程重启板子）+ 调试直驱 `servo` / `motor` / `drive` / `arm_pose`。**上行**另有 `ai_result` / `ai_tool`（工具进度行）/ `ai_task`（任务面板快照）/ **`ai_mem`**（物体记忆+车姿态 JSON，手机记忆地图的数据源）/ `status` / `state` / `pong` / `log` / `exec_status` / `mem` / **`track`**（本地追踪可视化 `{u,v,conf,st,novid}`，`novid=1` = 跟踪期不推视频）。
   ⚠️ **AI 侧另有词表外的动作写法**：AI 的 `arm` act 支持 `raise`/`pose`/**`place_done`**（`low` 也走这条），由 `ai_round.cpp` 直接调 `exec::arm_low/arm_raise/arm_pose` 或自行编排，**不经词表**——改 `arm` 侧时别把这两者算进"两侧逐字对应"。
2. **BLE UUID / 广播名（VisionS3）**：手机 `net/ble/BleProfile.gd` ↔ 大脑板 `ble.cpp`，逐字 mirror。
3. **哪吒 I2C 命令表**（从机 `0x80`、舵机/电机/灯光 cmd 字节）：现只有大脑板 `nezha_direct.cpp` 一处实现，无对侧；改动须对照哪吒扩展板硬件协议，别单方面改字节。
4. 各子 CLAUDE.md 中还有各自的坑（如大脑板 `namespace net` 勿改回 `network`、esp32 勿回退 2.x / 勿用 esp32cam 目标等），改动前读。

## 仓库外资料（未入库）

- `.trae/`（早期排查草稿 + `documents/` 设计笔记）**被 `.gitignore` 排除、不在版本库内**：本地工作区有，clone 后不会有。**mbedTLS/内核库重编流程已不再依赖它**——编库脚本、定制 `defconfig`、链接脚本补丁均已入库于 [`Stm32-Vision/librebuild/`](Stm32-Vision/librebuild/README.md)（随提交 `33b18e7`）。

## 仓库级约定

- 本仓库不配置顶层构建；编译/烧录入口分散：大脑板走 Arduino IDE/arduino-cli（命令行等价入口已固化进 `Stm32-Vision/tools/carctl.py build`，含自动 OTA 与指纹核对），App 走 Godot headless 导出。**不主动跑编译/烧录验证**（耗时无谓），默认交给用户在其 IDE 中做。
- Git：代码提交由用户操作（全局规则），助手可查看与撤回，若用户要求提交，请在提交消息中注意区分涉及部分，如(Vision/Mobile)，具体可见历史提交。推送时若发现需要先pull，尽量尝试使用git pull --rebase
- 文档维护：每份 `CLAUDE.md` 顶部标注「本文档基准：仓库 HEAD `<短hash>`（日期）」= 该文档对应的代码基准；更新文档前先基于该 hash `git diff` 检查，规则见全局 CLAUDE「CLAUDE.md 维护约定」，Claude.md不主动更新
