# CLAUDE.md

> 本文档基准：仓库 HEAD `f7d7b6b`（2026-10-03）。只覆盖已提交内容；未提交改动不收录。

本文件为当前项目（ESP32-S3-CAM / OV3660 摄像头板）的 AI 助手工作指南。

## 项目概述

基于 **ESP32-S3-CAM（N16R8，带 PSRAM）** 的视觉控制板。板子采集画面，交给多模态大模型（可在配网时配置 URL/Key/模型）识别，生成小车与机械臂的控制指令，**经软件 I2C 直驱哪吒扩展板**落地。

### 工作模式

1. **WiFi 直连 AI 模式**：板子经 WiFi 直调多模态 AI 接口，上传画面 → 解析返回控制指令 → **本板直接执行**（`exec::act`）。✅ 已实现（DIRECT：`ai_goal` → `ai_client` 迭代闭环）。AI 侧另维护**画面标定（单应）+ 物体空间记忆 + 车姿态累积**并把结果换算成当前车头局部系喂回模型；看图由模型的 `look` 工具按需索取（新拍全幅/放大，或按编号回看先前画面）。云端持续无有效响应会逐轮退避、超限即中止任务并回报手机。**会话上下文跨任务保留**（历史环/任务笔记/目标/任务列表/物体记忆都不随任务结束释放，接下发目标即接着跑），只有手机 `/clear`（`ai_clear` → `ai::session_clear`）才真正重置。
2. **蓝牙配置模式**：BLE（GATT Server，广播名 VisionS3）接收配置（WiFi 账号密码 / AI 接口等）→ 存 NVS → **在线重建 STA 生效，不整板重启**（BLE 保活，手机无需重连）。✅ 已实现。**生命周期**：手机在 WS 连上后断开本机 BLE 让出射频（WS_ONLY）；广播开关由 `ble::set_transmission` 统一控制——仅当**真在推帧**（UDP 图传 / MJPEG）时停广播让 WiFi 独占射频；WS 指令/状态通道在位时不关广播，保持可发现（手机随时可重连/兜底，`onDisconnect` 尊重该状态）。

> 本板既是视觉/控制大脑，也是执行器：动作由 `direct_exec`（namespace `exec`）经 `nezha_direct`（namespace `nezha`）软件 I2C 直接下发哪吒扩展板。

3. **本地目标追踪 + 全闭环自动夹取（不经 AI）**：`track`（`dcf` 相关滤波内核）在本地逐帧跟住一个被标记的画面目标，`grasp` worker 据此自己完成「降臂 → 旋转对准 u → 边前进边对准到 v → 合爪」整套动作，**纯画面坐标闭环、不经云端、不用单应/全局坐标**。✅ 已实现，两条入口：词表 `auto_grasp`（手机标注框 / `carctl.py`）与 AI 工具 `auto_grasp`。详见「本地追踪与自动夹取」一节。

## 两条视觉链路的关系

| | AI 闭环（`ai_*`） | 本地追踪（`track`/`grasp`） |
|---|---|---|
| 目标坐标从哪来 | 云端模型看图后报像素 | 板端相关滤波逐帧跟 |
| 谁在动 | 云端每轮决策（一次数秒） | 板端闭环（每步 ~0.5s） |
| 坐标系 | 屏幕像素 ⇄ 单应 ⇄ 车头系全局坐标 | **只有画面归一化坐标 (u,v)** |
| 典型用法 | 语义任务、放置、搜寻、跨目标搬运 | 单个清晰目标的逼近 + 夹取 |

## 代码库结构

源自 Espressif 官方 `CameraWebServer` 例程，已按功能模块拆分。**源码全部位于 `src/` 下按模块分的子目录**（`Stm32-Vision.ino`、`partitions.csv`、`build_opt.h` 仍在 sketch 根目录）。源码内的本项目头文件 include 一律以 `src/` 为根的相对路径（如 `#include "net/wifi_net.h"`）书写——Arduino 只把 `src/`（不含其子目录）加入 include path，子目录会递归编译但不进搜索路径，因此不要改回无前缀形式：

| 文件 | 作用 |
| --- | --- |
| `Stm32-Vision.ino` | 入口：setup 按 cfg→ble→exec→cam→net→web→ai 初始化并 `grasp::init()`；loop 调各模块 update + `exec::update_tick()`。⚠️ setup 末尾 `vTaskPrioritySet(NULL, 6)`：loop 里的 `update_tick` 是**所有带时限动作的唯一计时器**（定距/定角到点停轮、S 形缓动），同在 core 1 的图传(5)/grasp(3) 会抢占它 ⇒ 短脉冲被拉长（实测命令转 8° 实际转出 30~90°）、缓动被补成大跨度。抬到 6 后按 ~10ms 稳定推进 |
| `src/cam/camera.h/.cpp` | 摄像头初始化 + 抓帧（VGA `fb_count=3` + `WHEN_EMPTY`；`s_cam_mtx` 串行化一切重开）。**格式常态 = JPEG 直出**（图传/AI 白嫖 sensor 硬编），`set_track_mode(true)` 才硬切 **RGB565**（`track` 直读 luma/chroma）。三取帧入口要分清：`grab()` 拿到的可能是队列里压着的旧帧（消费者比相机慢时永远压 2 帧）；**「此刻画面」一律走 `grab_fresh(drop)`**（连续 grab+return 排空，下一次 grab 阻塞等新帧）；`publish_latest`/`latest_copy` 是图传任务发布的最新 JPEG 副本（`/capture` 等按需读者用它，不再自己排队）。**高清真拍** `request_hires()`（放大镜用）：deinit → init(SVGA) → 连续预热 `HI_WARM` 帧挑有效帧 → 存进常驻 PSRAM 快照 → 回当前格式再预热（`HI_WARM_RECONFIG` = `HI_WARM`+3；**回 VGA 的 deinit/init 失败会重试 3 次**——一次失败就长期回不到 VGA，会让后续所有放大镜取景失败、AI 空转）；期间 `grab()` 返回 **nullptr**（各方须容错），图传停一拍。⚠️ **只支持 SVGA**：SXGA 会卡死驱动。⚠️ **psram 直写恒关**（`set_psram_for` 无条件 `false`）：实测 RGB565 整帧与 SVGA 都丢字节、且与非 psram 来回切后 VGA 再也取不到帧；代价是常驻约 16KB 内部 DMA，已把 `CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX` 压到 16384 换取。`wait_dma_block(need, max_ms)` 在切 RGB565/高清前等够一块连续内部 DMA。**`reinit_current()`** 兜底"init 报 OK 却不出帧"：连续抓帧失败 2 次就 deinit→init 当前格式。`grab()` 带 1200ms 超时，失败时限频打出内部堆/DMA 水位。**型号唯一配置点**：在 `camera.h` 顶部选 `CAMERA_MODEL_*`（AI_THINKER 在 S3 上会空指针崩溃，勿用）；画质与画面回正（`vflip`/`hmirror`/饱和度）在 `make_config(fs, fmt)` + `apply_sensor_calib()`（重建后必须重调，否则校准被清零），`jpeg_buffer_size` 已抬到 256KB（防高熵 VGA 帧 FB-OVF 硬停图传），推流帧率在 `net/app_httpd.cpp`（`WS_STREAM_FPS`，图传走 UDP） |
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
| `src/core/command.h/.cpp` | 统一词表 JSON 分发（与传输解耦、回调应答）；手动指令先 `ai::cancel` 打断 AI 闭环再落地；`apply_network` 在线换网生效；`ai_goal`→`ai::set_goal`、`goto`→`ai::goto_target`、`ai_chat`→`ai::append_chat`、**`ai_clear`→`ai::session_clear`**、**`auto_grasp`→`grasp::request`**（手动接管：先打断在跑的 AI）；`log` 统一日志指令走 `blog` 开关；`reboot` 远程重启（链路卡死时的解药，升级中会被拒）；状态位图 `cmd::state_bits()` 是**唯一出口**，`status` / `get_state` / `pong` 三类回执**都**带 `params.bits`（手机据此同步按钮与 AI 运行态）。⚠️ `ping`/`pong`/`get_state` 三类**不打** `type=` 刷屏行——排查"指令到了没有"时这三类查不到凭据。⚠️ `mem` 诊断**只回 JSON 摘要、别调 `heap_caps_dump_all()`**（它在 httpd 任务里走 UART 打印会空指针崩溃，实测发 `{"type":"mem"}` 即触发中断狗复位） |
| `src/core/board_log.h/.cpp` | 统一日志模块（namespace `blog`）：`logf(cat,...)` 统一串口调试输出（带 `[类]` 前缀），按 `/log` 开关（exec/ai/all）把 `{type:"log",params:{src,text}}` 入队，由转发任务（栈 8192）经 app_httpd 注册的转发器（WS 广播+BLE status）发手机；`forward_text(cat, text)` 发**任意长度**文本（自建 PSRAM 副本 + UTF-8/JSON 转义与非法字节清洗），`logf` 那条 256B 栈缓冲装不下 AI 的 reasoning；`log_line(cat, text)` 收**已格式化好的整行**（省一次 vsnprintf，`logf` 内部走它） |
| `src/ai/ai_client.h/.cpp` | AI 任务**外壳**：worker 任务、代际/槽、对外 API（`init`/`update`/`set_goal`（带 `one_shot` = 只跑一轮决策即收尾）/`goto_target`/`cancel`/`busy`/**`session_clear`**（开新会话，手机 `/clear` 用）`/append_chat`/`logf`/`set_edited_image`）。worker 起来后调 `round_run_task`；`m_busy` 在**取到槽立刻置位**（不能等进到任务主体才置，否则 `session_clear` 会趁"槽已取走还没置忙"的窗口把会话状态释放掉）。⚠️ 任务主体已拆去 `ai_round.cpp`，而 `update()`/`logf()` 的**实现**在 `ai_result.cpp`——按文件名找会扑空（头文件有注明） |
| `src/ai/ai_round.h/.cpp` | **AI 任务主体**（原 `ai_client.cpp` 的大头）：`round_run_task()` 一轮闭环（取帧 → 组 body → 发送 → 校验 → 落地 → 记账）。跨轮状态在 `RoundCtx`，**按值建在 worker 的 16KB PSRAM 栈帧上**；任务列表/笔记/目标是 **PSRAM 变长文本**（`core/psram.h`，无字数与项数上限）。**会话上下文跨任务保留**：`g_sess_*` 一组全局（历史环 / 任务笔记 / 目标 / 任务列表 / 图片发号器）任务收尾**不释放**，`ai_clear` → `ai::session_clear()` 才真正重置（任务在跑时先登记 `g_sess_clear_pending`，等收尾再生效，避免与 worker 抢历史环）。**面板上行**：`notify_tasks()` 在任务开始 / `task` 改动列表 / `goal.set` / 转等用户输入 / 收尾时推 `ai_task`（`state: running\|wait\|done\|fail\|abort`）；**`notify_mem()`** 在动作落地后推 `ai_mem`（物体记忆 + 车姿态 JSON，手机记忆地图的数据源；记忆只在工具落地时变故每轮一条，勿每轮重推）。`round_prepare`（取帧 / 放大镜 / 本地追踪刷新）、`round_attempt`（组包 → 发送 → 取工具调用）、`dispatch_calls`（**落地次序固定** mem → task → say → car → auto_grasp → look → compact → goal）、`land_car`（含 `place_done` 一段式放置收尾）、`build_state_block`（车臂状态 + 目标 + 任务列表 + 笔记，挂本轮**最后一条工具结果**尾部）。历史按**回合**环缓存、`reasoning` 每轮原样回传；图只对**最新一组实景**注入字节（先前帧环 + 用户参考图，统一 `ImageN` 编号）；WS 文本出口统一过 `sanitize_ws_utf8` 消毒（云端偶发残缺 UTF-8 原样进 TEXT 帧会让手机端以 `1007` 断链）；发往云端的长字符串按 UTF-8 边界截断（`src/core/utf8.h`；截半个中文字节会被判 400 或显示成 `?`）；服务端持续无有效响应则逐轮退避，超限中止任务并回报手机 |
| `src/ai/ai_result.h/.cpp` | 结果出口：`ai::update()`（消费 worker 回执、推状态）、`ai::logf()`、`enqueue_result()`、`set_edited_image()` / `edited_snapshot()`。⚠️ `enqueue_result` 的 `ResultItem` **走 PSRAM**：这条每轮 AI 要发十几~几十条、文本可到 KB 级，落在内部堆会把 DMA 池最大连续块打到 1KB 上下（AI 任务期间「内部DMA块告急」几乎持续不断） |
| `src/ai/ai_validate.h/.cpp` | 校验入口：剥代码块 → 解 JSON → 拒旧版扁平 `type` → **遍历工具表**逐项规范化；硬钳制宏 `AI_MOVE_MAX_CM`(40) / `AI_SPIN_MAX_DEG`(180)。⚠️ 词表本身**不在这里**，在 `src/ai/tools/`；校验器对**每个**工具调用都跑**全表** `parse`，故每个 parse 都得先自查"本工具有没有被调用" |
| `src/ai/ai_nav.h/.cpp` | 本地巡航 `navigate_to(gen,tx,ty,stop_cm)`、`settle_wheels(gen,max_ms)` / `settle_arm(...)`（取帧前等车/臂停稳，缺省 `AI_SETTLE_MAX_MS`=700；`AI_ARM_LOW_H_CM`=2.5 判"臂算低姿"）；收敛上限一批 `NAV_*`（到位容差 6cm、单次转角/推进上限、迭代与反转次数上限等，防开环死航打转） |
| `src/ai/track.h/.cpp` | **板端本地目标追踪**（namespace `track`）：从 RGB565 帧抽 luma（`decode_gray`）+ 饱和度（`decode_chroma`，仅诊断/色度门用），交 `dcf` 逐帧定位；`seed(name,cx,cy,w,h)` / `stop()` 经 `cam::set_track_mode` 在 RGB565⇄JPEG 间硬切（幂等，只在起停各切一次）。控制量先验 `hint_motion(du,dv)` / `hint_wide(...)` 告诉追踪器"这一脚动作后目标大致挪到哪"，**比它自己的图像预测可靠**（后者基于动作前的帧）。`update_from_fb(fb, light)`：正常更新走完整状态机；`light=true` 是"只刷新显示位置"的轻量路径（仅 Tracking 态生效、不推进 Lost、不改模板），图传任务按 `TRACK_REFRESH_MS` 顺带调用。`set_seed_box` / `seed_box` 供手机在切跟踪画面前的最后一帧上画出 AI 标的框。诊断出口齐全：`last_capture_ms`（**帧的拍摄时刻**，夹取等新位置靠它）、`last_ok`（本帧是否被采纳——被拒的帧状态仍是 Tracking、位置是上次的旧值）、`last_appear`（与起跟那帧的 NCC）、`last_phases_us`（取窗/搜索分段耗时）、`last_obj_px`/`last_off_frac` |
| `src/ai/dcf.h/.cpp` | **相关滤波跟踪内核**（namespace `dcf`，MOSSE 思路，自写 radix-2 FFT 不依赖 ESP-DSP）：输入窗重采样归一化到固定 32×32 网格 ⇒ 同一滤波器对任何尺度成立，"尺度搜索"退化为"换几个源窗大小各算一次响应、取 PSR 最高"。每帧**在线更新**滤波器（`ETA`=0.125），PSR（峰值旁瓣比）当置信度。两道防滑机制：① **种子外观锚定**（`APPEAR_MIN`）——与起跟那帧的窗算 NCC，低于门限**不拿去训练滤波器**（专治"锁到随车固定的夹爪上"，那种情况 conf/PSR 反而更高，只有跟初始外观比才看得出）；② **车身 mask**（`track_mask.h`，`carmask::in(u,v)`）——响应图落在车/臂/夹爪的格子上压死，防锁到自车。⚠️ **死亡螺旋**：PSR 不达标时位置/尺度/滤波器**一律不动**（曾只挡训练，一帧坏匹配就把窗缩小、越缩越小）。`radius_norm()` 把当前搜索半径报给夹取闭环限制单步幅度；`appear_at()` 是不动跟踪状态的外观探针 |
| `src/ai/track_mask.h` | 车身 mask 表（`carmask::GRID[48][64]`，由 `armLowMask.png` 生成）。⚠️ **按抠图真实轮廓取、不是矩形**，且**只保留 v≥0.63 以下**：臂架那团与方块的对准路径在画面上重合，压了必丢目标（实测方块在 (0.598,0.438) 被自己人压掉后跟丢且重捕不回来）。早先用 `u≥0.36` 的矩形，种子在 (0.361,0.74) 时被判"种子在车身里"→ 整场关掉保护 → 空夹。**起跟点落在车身里则整场关闭 mask**（目标本就在夹爪上，压死等于直接丢） |
| `src/ai/grasp.h/.cpp` | **本地自动夹取闭环**（namespace `grasp`）：`request(x,y,w,h,name)` / `busy` / `cancel` / `last_result()`，内部 worker 任务。全程只看画面归一化 (u,v)：`arm low` → 原地小角对准 u → 每步前进后重对准，直到 v 进"可夹纵深" → `grasp`（含合后抬臂）。`grasp::set_reacquire(false)`——闭环里**关掉**跟丢后的宽窗重捕（宽搜抓到什么都可能）。`verify_grasp` 出合爪诊断（地面原位 / 夹持位两个外观分），⚠️ **不保证夹住**，"已夹取(可能)"只表示没抓到"方块还在地上"的证据 |
| `src/ai/tools/` | **AI 工具表的键级定义**：`tool.h` 定义 `ToolSpec{key,doc,group,parse,run,feedback,logfmt}`；`t_observe`(observe/delete) / `t_move`(move) / `t_arm`(arm) / `t_meta`(light + set/finish) / `t_tasks`(note/todo/done) / `t_grasp`(**auto_grasp**) 各提供 `parse_*`，`registry.cpp` 的 `kTools[]` 是唯一有序表（**11 键**，键名全局唯一）。**表序只决定校验顺序**（`validate_cmd` 按表序跑各 `parse`）；**落地顺序在 `ai_round.cpp` 的 `dispatch_calls`**。现阶段只填 `parse`，其余留 nullptr 是有意的——给模型的**工具声明**另有 `ai_prompt.cpp::tools_schema()`（8 个工具，改键名/枚举两处必同步）。⚠️ `t_grasp.cpp` 的 parse 必须先判"本工具有没有被调用"再校验 x/y：校验器对每个调用都跑**全表** parse，不先判就会把 mem/task/goal 全误拦成"缺 x/y"。详见「AI 层要点」 |
| `src/ai/ai_alloc.h` | ArduinoJson 的内存池走 PSRAM（`PsramAllocator` / `g_js_alloc`，定义在 `ai_client.cpp`） |
| `src/ai/ai_prompt.h/.cpp` | 系统提示词与请求 body 组装：系统提示词 = 人设（Caris）+ 规则 + 工作流程 + 画面/夹爪定义（`kSysSrc`，编译期转义成静态字节）；`tools_schema()` = 请求尾部的 8 个工具声明；`build_body(PieceList&, const BodyReq&)` 把「系统提示词（静态段直接引用）+ 历史环逐回合（assistant(tool_calls) / tool 结果 / 用户发言）+ 末尾固定约束」组成**碎片流**（JPEG 边发边 base64，不再整份拼串）；目标/列表/笔记不单独成消息，由 `ai_round` 的 `build_state_block` 每轮现拼挂到最新一条工具结果尾部。图像按用途带 `detail`：全幅 `low`（512×512 省 prefill）、放大/精判图 `high`（`ImgRef::hi`）。⚠️ 提示词（行为判据）与 `tools_schema`（键名/枚举）是一套东西的两半，改一处必改另一处 |
| `src/ai/ai_mem.h/.cpp` | 空间记忆 + 车姿态（自 `ai_client` 拆出）：`mem_reset()`（**只在开新会话时调**，任务结束不再重置坐标系）；`car_update_pose()` 按定距/定角近似累积位姿（move→平移 / spin→转向）；`mem_observe_xy()`（屏幕归一化像素，经单应解算后入表；返回 false = 没记进去，调用方须回告 AI）；`mem_feed()` 生成车头局部系记忆文本喂 AI（表 16 条，满了覆盖最旧；**物体名在 PSRAM 上按实际长度分配**，见 `core/psram.h`）——⚠️ **只喂朝向，不再喂车自身全局 x/y**（里程无反馈、撞过就漂，喂给 AI 反而误导它推距离）；`mem_find()` 查目标全局坐标、`mem_forget()` 按名删除（判据与 store/find 同一套；**删掉的正是本地跟踪目标时顺带 `track::stop()`**，否则锁会一直挂着且每轮自动 observe 灌错值）、`mem_tick_stale()` 未观测过期轮数 +1、`mem_export()` 导出 JSON 给手机「画面源 = 记忆」俯视图。车姿态 `s_car_x/y/heading` 与 `navigate_to` 共享 |
| `src/ai/ai_http.h/.cpp` | AI TLS 发送层（自 `ai_client` 拆出）：`http_post(PieceList&)` 复用连接 POST 并读响应（keep-alive）；`http_last_status()` / `http_last_error()` 供 worker 做 4xx/429 快速失败与分诊；`http_stop()` 立即中止在途请求/连接（cancel / set_goal / goto 打断用）；`extract_tool_calls()` 解响应取 `calls[]` + `reasoning_content`（非流式一次收全），无工具调用时把 `finish_reason`/用量打出来 |
| `src/ai/ground_proj.h/.cpp` | 屏幕↔地面坐标换算（namespace `ground`）：实测标定点拟合单应，`screen_to_world(px,py,&x,&y)` 与 `world_to_screen(x,y,&u,&v)`（伴随矩阵解析求逆，标定区外返回 false）；标定点 `kGroundCal` 在 Calibration.h，加测点改那里。⚠️ `init()` 的回验超差**只打日志、不禁用单应** |
| `src/ai/ai_dump.h/.cpp` | **AI 抓帧留档**（debug，供 PC 侧复盘）：把**实际发往云端的那一帧原始 JPEG**连同该轮标注（动作、被判无效的原因等）留在 PSRAM，6 槽环形；`seq==0` 表示该槽尚未定案、对 HTTP 不可见。每槽缓冲**按需增长**（8KB 步进，只按实际见过的最大帧分配——预留 6×128KB 会白占近 800KB PSRAM）。HTTP 出口：`/ai_dump`（清单，`?after=N` 长轮询）、`/ai_frame?seq=N`（原始字节）。取回与配日志见「PC 侧工具」的 `car_logcat.py` |
| `src/ai/magnify.h/.cpp` | **放大镜**（namespace `magnify`）：`crop_center_jpg(...)` —— `cam::request_hires` 高清真拍（SVGA）+ 用 TJpgDec **部分解码**只解出中央 `(0.25,0.25)-(0.75,0.75)` 再重编码（workbuf 与输出全走 PSRAM，规避 `fmt2rgb888` 整幅软解卡死）→ 输出 400×300，**同源像素密度**；外加 `last_cost_ms()` 诊断 |
| `src/net/ping_svc.h/.cpp` | `/ping <目标>` 异步 ICMP echo（esp_ping），结果经 cmd 回复通道回报；无目标仍由 command 就地回 `pong` |
| `src/net/ble.h/.cpp` | BLE GATT Server：配网 + 兜底控制 + status 通知；广播开关随 `set_transmission`（真在推帧即停）（UUID 见下「协议参考」）|
| `src/net/ota.h/.cpp` | OTA 升级：ArduinoOTA + HTTP `POST /update`（板子固定在车上、串口够不着，刷固件只能走这里；双 OTA 槽见下 `partitions.csv`） |
| `src/net/app_httpd.cpp` | HTTP + WS（端口 81：文本=指令/状态 JSON）+ UDP 图传帧推送 + `exec_status` 周期上报（默认关，`/log exec on` 后约 400ms 一条、状态文本相同则跳过、**OTA 期间停推**；状态缓冲须容下含抓手前端的整行）。注册 `blog` 日志转发器（WS+BLE）。`ws_stream` 任务栈 8192（推流 + 状态上报共用）。⚠️ **图传软编 + 跟踪喂帧都钉在 core 0**（`kStreamCore=0`）：留在 core1 会与 `grasp` 搜索、exec 抢 CPU，实测"取图/搜索耗时成倍膨胀"（925ms vs 85ms），闭环一步十几秒。跟踪期 `track::active()` 时**整段不推视频**，改由本任务当**唯一喂帧者**（`track_feed`，先 `grab_fresh(2)` 排空再喂，否则读到的是动作前的旧帧 → 闭环读数滞后一步 → 转头）。`maybe_send_track_pos()` 10Hz 上报跟踪位置（`novid=1` 期间不带视频）与停止时的 `idle`。**端点**：`/`(302→/stream) `/status` `/control` `/capture` `/zoomshot` `/tracktest` `/stream`(MJPEG) `/bmp` `/xclk` `/reg` `/greg` `/pll` `/resolution` `/ai_dump` `/ai_frame` `/coredump`；`POST /update` 实现在 `ota.cpp`。⚠️ `/zoomshot` **只认 `out_w`/`out_h`/`quality`**，固定裁中央 `(0.25,0.25)-(0.75,0.75)`（与 AI 的 `zoom` 同框）。⚠️ `/tracktest` 是**灰度/色度解码自检 + 跟踪自检**两用（`?scale=`/`&chr=1`/`&all=1` 出 BMP；`?u=&v=[&w=&h=][&n=]` 种目标连拍并回逐帧轨迹；`&cont=1` 沿用当前目标；`?stop=1` 停跟踪） |
| `Calibration.h/.cpp` | **手动校准数据集中区**（根目录）：舵机限位/机械臂参数（STEER/REACH/GRIP/LIFT、ARM_*）、固定位姿 `ARM_LOW_X/H_CM`(8.0/1.0) 与 `ARM_RAISE_X/H_CM`(9.0/9.0)、`GRASP_LIFT_CM`(7.0)、定距表 `MV_SPEED_X/Y` + `MV_COAST_X/Y`（**4 档**，档位 `{0.15,0.25,0.5,1.0}`）与 `MV_START_MS`/`MV_MIN_PULSE_MS`、定角**查表** `SPIN_TBL_DEG/MS`（10 点插值）、`SPIN_MIN_SPEED`/`SPIN_PIVOT_BEHIND_CM`、屏幕→地面单应标定点 `kGroundCal`、机械臂夹心散点 `kArmPts`；**本地追踪组** `TRACK_*`（`SCALE`/`SEED_W/H`/`TW_MIN/MAX`/`PSR_KEEP`/`PSR_LO`/`SNAP_*`/`MAGIC_TOL`/`LOST_N`/`SMOOTH_ALPHA`/`REACQ_MAX`/`MIN_STD`/`REFRESH_MS`）；**自动夹取组** `GRASP_*`（对准目标 `U_TGT`/`V_TGT` 与随距离插值的容差、`SPIN_MIN/MAX_DEG`、`DU_PER_DEG_GEO`=1/水平FOV、`MOVE_*`/`BACK_MAX_CM`/`STEER_ALIGN`、`RADIUS_FRAC`、`STUCK_N`、横向漂移预算 `DRIFT_BUDGET`/`DRIFT_ASSUMED`、合爪自检 `HOLD_*`/`EMPTY_MIN`/`VERIFY_BACK_CM`、`MAX_ITERS`/`TIME_MS`/`LOST_MAX`、等稳态 `MOTION_START_MS`/`FRESH_MARGIN_MS`/`FRESH_WAIT_MS`）与 `PLACE_BACK_CM`(5)；`bivar::arm_set()` 实现在 Calibration.cpp。⚠️ 这两组宏的注释里大量记着**为什么是这个值**（真机实测数据），改之前先读，别当噪音删 |
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
- **直行时回正转向舵**：`move` 的 `steering` 为 0 时写回 `STEER_CENTER`。上一次带转向的 `move` 若不回正，之后所有 `move` 都在画弧。
- **状态行**：`read_state()` 合成 `小车: <停止/前进/后退/原地左转/原地右转> | 夹爪: <clip/release> 姿态: <自由/low/raise/grasp/clip/release/fold> 位置: (<x>,<h>)<已伸最远/已缩最近/已触底/已触顶>`，末端位置由正运动学算出。⚠️ 状态串口径变更后，凡按旧串解析的地方（手机端、`tools/`）都要同步。
- **灯初始状态**：`exec::init()` 主动把三盏灯都写成关并清本地 `s_light_*`——哪吒寄存器会保持灯态，MCU 重启后不同步。
- **手动指令优先**：`move/stop/arm/drive/spin/servo/motor/arm_pose/reset` 视为手动接管，先 `ai::cancel` 再落地；`arm`/`arm_pose` 只停轮子（`StopMode::Wheels`），其余类型用户指令已覆盖故不补停。手动路径同时清掉上一轮遗留的定距/定角时限，避免旧时限在新指令之后误停。

## AI 层要点（`ai`）

- **给模型的工具（8 个，与手机端词表是两套，勿混）**：`car` / **`auto_grasp`** / `mem` / `task` / `goal` / `look` / `say` / `compact`。声明在 `ai_prompt.cpp::tools_schema()`（请求体尾部的 `"tools":[…]`，放在 messages 之后以保住前缀缓存）。⚠️ 尾巴上的 `"tool_choice":"auto"` 是风险点：DeepSeek 官方称 V4 思考模式不接受 `tool_choice`，严格时会 400；当前实测能过，若哪天开始 400 第一件事就是摘掉它。
- **键级校验表**：`src/ai/tools/registry.cpp` 的 `kTools[]`（**11 键**，键名全局唯一）。⚠️ **表序只决定校验顺序**（`validate_cmd` 按表序跑各 `parse`）；**落地顺序是另一处**：`ai_round.cpp` 的 `dispatch_calls` 固定为 mem → task → say → car → **auto_grasp** → look → compact → goal，`car` 内部再由 `land_car` 排。改任一顺序前先读 `tool.h` 头注与 `dispatch_calls`。
  - 现阶段 `ToolSpec` 只填 `parse`，`doc`/`group`/`run`/`feedback`/`logfmt` 留 nullptr 是**有意的**（文案在提示词、落地手写）。所以**加一个键要改四处**：① `t_*.cpp` 写 `parse_*`；② `tool.h` 加声明；③ `registry.cpp` 插一行；④ `ai_prompt.cpp` 的 schema。`tools_selfcheck()` 只打日志**不 assert**（板子在车里，带病上电也比 panic 好救）。
  - ⚠️ `feedback`/`logfmt` 拿到的 `cmdD` 是 **const** `JsonDocument` ⇒ 类型检查只能用 `is<JsonObjectConst>()`/`is<JsonArrayConst>()`；写成 `is<JsonObject>()` **恒 false 且照样编译过**（见 memory `arduinojson-const-variant-is-false`）。另：`parse` 返回的指针**不保证**指向 `err`，允许直接返回静态字面量，调用方只许当只读串用。
  - ⚠️ **每个 `parse_*` 都得先自查"本工具有没有被调用"**：校验器对每个工具调用都跑**全表** `parse`，直接校验平铺参数会把 `mem`/`task`/`goal` 任何调用都误拦（实测所有工具回执被缀上 `(-1,-1)` 错误、目标/任务/记忆全没落地）。`t_grasp.cpp` 有现成写法可照抄。
- **各工具参数**：
  - `car`：`move{type,value,target}`（`forward`/`backward` → `value`=cm；`spin_left`/`spin_right` → `value`=角度；`approach` → `target`=记忆名，缺省最近目标）· `arm{type,x,h}`（`low`/`raise`/`fold`/`grasp`/`clip`/`release`/`pose`/**`place_done`**，持续抬落/伸缩已移除；`x`/`h` 仅 `pose` 用）· `light{kind,on}`（**`on` 必须显式给 bool，缺省直接拒**——防 AI 漏写把灯误关）
  - `auto_grasp`：`{x,y,w,h?,name?}`，x/y = 目标在画面上的**中心**归一化（0~1）
  - `mem`：`observe[{name,px,py}]`（px/py 填目标底部中心，放大图也照常填 0~1，程序换算）/ `delete[名字]` / 都不给 = 查询（回车朝向与全部记忆；⚠️ 已不再回车自身全局 x/y）
  - `task`：`note`（字符串）· `todo[]`（整表重写）· `done[]`（编号，**1-based**，取消标记要重写整表）
  - `goal`：`set`（改目标）· `finish` = `done`|`fail`|`wait`（非法值**明拒**）
  - `look`：`zoom`(bool) · `image[]`（编号回看，最多两张）｜`say`：`text`（必填）｜`compact`：`summary`
- **`approach` 与合爪/放收不能同轮**：`grasp`/`clip`/**`place_done`** 与 `approach` 同轮会被拦（approach 会挪车、坐标过时，拿旧坐标合爪必空夹；`place_done` 内含 `release`，靠近途中就松爪会把东西掉在半路）；`arm low/fold/raise`、`light` 可与 `approach` 并行。
- **`auto_grasp` 与 `car` 不能同轮**：`do_grasp` 见到本回合已执行过 `car` 动作就拒（车一挪，模型手上那批画面坐标就过时了）。它同步等本地闭环跑完（`AI_GRASP_WAIT_MS`=50s 兜底）才回执，把 `grasp::last_result()`（`已夹取(可能)` / `确定未夹住(方块仍在地面原位)` / 中止原因）当反馈喂回模型——这是模型判断"要不要再来一次"的依据，比让它自己看图猜可靠。
- **`place_done` 是放置收尾一段式**：`release` → `arm_raise()` → 后退 `PLACE_BACK_CM`(5cm) → `fold`，一个调用完成最终放置。⚠️ 抬臂**只能走 `exec::arm_raise()` 专用入口**——通用 `arm act` 分发里没有 `raise` 分支，拼 `{"act":"raise"}` 会静默空转（实测臂没抬就后退，蹭着盒子走）。
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
- **⚠️ 别把车自身全局坐标喂给 AI**：里程无反馈、撞过就漂，写进提示词会让模型拿它推距离反而误导（已从系统提示与 `mem_feed` 一并移除）。置信优先级现为 `用户发言 > 系统规则 > 每帧图片画面 > 系统坐标提示`。
- **会话上下文跨任务保留**：任务收尾**不释放**历史环/笔记/目标/任务列表/物体记忆/车位姿，接着发新目标即延续上下文；只有 `ai_clear` 才重置。代价是长期不 clear 会一直占着那块 PSRAM（有意取舍）。

## 本地追踪与自动夹取（`track` / `dcf` / `grasp`）

**这一套完全不经云端**：标一个画面目标 → 板端相关滤波逐帧跟 → 自己闭环夹取。AI 只是它的一个调用方。

### 入口

| 入口 | 来源 | 说明 |
|---|---|---|
| 词表 `auto_grasp{x,y,w,h,name}` | 手机标注框 / `carctl.py` | 手动接管，先打断在跑的 AI |
| AI 工具 `auto_grasp` | 云端模型 | 与 `car` 同轮会被拒（坐标过时） |
| `track::seed()` | 手机 `ai_goal` 带 `annotation` 的 `{x,y,w,h}` | 只锁目标、不夹取；`observe` **不再**种跟踪（相机切格式太贵） |
| `/tracktest`、`carctl.py tracktest` | PC 侧 | 不动车、不经 AI 验证追踪器 |

### 追踪器（`track` + `dcf`）

- **算法换代**：原"穷举 ZNCC 模板匹配"已换成 **DCF 相关滤波**（`dcf.cpp`，MOSSE 思路，自写 radix-2 FFT）。理由两条：速度（5 尺度 × 约 2600 位置各做一次匹配 ≈ 230ms/帧 → 整张响应图一次 FFT，32 网格一次相关约 1ms）与鲁棒（ZNCC 是"锁定那帧的模板 + 学一次"，车一转/一逼近就过期，conf 从 0.92 退化到 0.55 后卡死；DCF 每帧在线更新且自带 PSR）。
- **尺度无关**：输入窗重采样归一化到固定 32×32 网格，同一滤波器对任何尺度成立 ⇒ "尺度搜索"退化为"换几个源窗大小各算一次、取 PSR 最高的"。`K_WIN`=6.0 必须罩住"一步动作的位移"（实测转 25° ≈ 0.15 归一化 ≈ 48px，而目标才 ~23px；K_WIN=4 时半径只有 35px，一转就出窗、conf 崩到 0.16）。
- **三道防"锁到自车"闸**（按重要性）：① **种子外观锚定**——与起跟那帧的窗算 NCC，低于 `APPEAR_MIN` **不训练滤波器**；专治"锁到随车固定的夹爪"（那种情况 conf/PSR 反而更高，只有跟初始外观比才看得出已经不是那个物体了）。② **车身 mask**（`track_mask.h`）——响应图落在车/臂/夹爪的格子上压死；⚠️ 只保留 v≥0.63 以下，臂架那团不能压（与方块对准路径重合）。③ **色度加权门**——目标本身有颜色时开启，把"饱和度与目标差太远"的格子的**正向**响应乘 `SAT_PENALTY`（负响应不动，抬高反而可能变成峰）。色度只做**否决**，找目标仍是 luma ⇒ 色度漂了最多退化成原行为。
- ⚠️ **死亡螺旋**：PSR 不达标时**位置、尺度、滤波器一律不动**。曾只挡训练，一帧坏匹配就把窗缩小、越缩越小。
- **跟踪期相机在 RGB565**（常态是 JPEG 直出）：`seed()`/`stop()` 各触发一次 `cam::set_track_mode` 硬切（deinit→init，幂等，失败重试 3 次）。⚠️ 切之前必须 `wait_dma_block()` 等够连续内部 DMA，否则 init 失败会把相机留在 deinit 态。
- **跟踪期不推视频**，改由图传任务当唯一喂帧者（`track_feed`）。理由：相机 ~4~6fps，"抓一帧"本身就要等 ~200ms，夹取循环自己抓每步光这段就 ~600ms；改连续喂后夹取这侧归零，且位置**连续**更新（手机十字真正跟手而非"每步一跳"）。
- **喂帧必须先排空**：`fb_count=3` + `WHEN_EMPTY`，喂帧者比相机慢 ⇒ 队列里一直压着 2 帧旧的，`grab()` 直接返回的是**动作前**的画面。症状：夹取第 N 步读到第 N-1 步动作后的位置 → 以为没转够、再转一脚 → 过头。修法是 `grab_fresh(drop)`（丢到空后 grab 会阻塞等新帧）。
- ⚠️ **色度通道已彻底删除**（用户拍板：地面反光环境下纯色度方案根本用不了）。`decode_chroma` 只作诊断用途（`/tracktest --chr` 出色度 BMP 找真目标坐标），**任何依赖色度找目标的方案都不要再提**。
- **可用距离范围很窄**：1.2m 时目标仅 ~6px、60cm ~12px（模板下限约 20px@320 网格 = 40px VGA）⇒ 锁不住。好用区间约 v 0.25~0.75（≈10~45cm）。超近种子（v>0.7）也是硬边界：近场剧变导致外观分崩，会诚实中止。
- ⚠️ **臂位姿模型会与实物失配**：每次 OTA/重启后臂的位姿模型复位，`arm low` 可能变成**空操作**（模型以为已在 low）。做 mask / 量夹爪位置前先确认臂真的到位。

### 夹取闭环（`grasp`）

- **流程**：`arm low` → 原地小角把物体对到 `GRASP_U_TGT`(0.485) → 每步前进后重对准，直到 v 进"可夹纵深" → `grasp`（合爪 + 抬臂）。全程只看 (u,v)，**不用单应、不用全局坐标**。超时 `GRASP_TIME_MS`=60s / 迭代上限 60 / 后退总里程上限 4 步。
- **闭环里关掉重捕**：`set_reacquire(false)`——跟丢后的宽窗重捕是"抓到什么都可能"。
- **"动作后目标必须动"是唯一的卡死判据**：单步之后 |Δu|/|Δv| 小于**期望位移的 30%** ⇒ 这一脚目标没搭理我；连续 `GRASP_STUCK_N`(2) 次即中止（锁到了随车一起动的东西）。⚠️ 旧的"误差有没有变小"判据已被 0.0001 的蠕动反复清零、**永不触发**，别改回去。
- **合爪前闸**：发过动作却全程零响应 ⇒ 不许合爪。⚠️ 曾因 `saw_resp` 是**粘滞**的（早几帧确实响应过旋转就置 true，之后切成不响应动作的夹爪也不清）而漏挡过一次。
- **横向漂移预算**：一步前进会按实测 `du_per_cm` 把目标横向带偏，带偏量必须留在 u 容差内——否则一步就把目标带出容差，而**贴近时旋转对 u 几乎无效**（实测 4cm 前进带偏 +0.036u，之后连转 5°×5 次 u 纹丝不动）。首次前进还没学到 `du_per_cm` 时用保守值 `GRASP_DRIFT_ASSUMED`，否则"第一次前进等于不设上限"。
- **容差必须大于执行器最小可分辨位移**，且 u 容差**随距离收紧**（远处放宽省时间、近处收紧——最后一段精度该最高）。⚠️ 若最小转角 5° 转不动（脉冲顶不过静摩擦）就退回 8°，靠"容差垫到半步"把落点锁住。
- **等新位置的两根支柱**（都是实测教训，别简化）：
  ① 判"新鲜"用 `track::last_capture_ms()`（**拍摄时刻**），不是结果到达时刻（含解码+搜索延迟，"结果新、画面旧"全放行）；② 决策前等**稳态**——本动作之后连续两帧读数一致（|Δ|<0.012）才放行，最多等 3 帧。滑动样本不决策、不进增益、不动容差（否则 `du_per_deg` 被毒小 → hint 连锁歪 → 恶性循环）。
  - **hint 必须动作后重新喂**：喂早了会被动作中途处理的帧消费掉（`update` 每个采纳帧都把预测改写成图像位移）。
- **单步幅度受搜索半径限幅**：预期位移 ≤ 半径 × `GRASP_RADIUS_FRAC`。半径本身受"输入窗归一化"约束不能独立定，用这个系数等价于"半径由预期位移决定"（实测目标变大时半径跟着小，25° 一步就出窗）。
- **转角→Δu 用几何角速率** `GRASP_DU_PER_DEG_GEO`(=1/水平FOV 65°≈0.0154)，不是靠实测拟合的增益——远场实测 13°→Δu 0.2，与"bearing 变化≈转角"吻合。拟合的 `du_per_deg` 只留给 stuck/响应判据。
- **可夹即收**：v ∈ [V_TGT−V_TOL, V_TGT+CLOSE_V_HI] 且 |eu| ≤ `GRASP_CLOSE_U_MAX` ⇒ 直接合爪，不走"近距纠弧 + 太近后退"（近场纠不动 u，白推；后退会把方块退出爪区，实测退完没夹到）。
- **后退的两个用途**：v 冲过可夹点太近（夹爪会顶开物体）；u 怎么转都不变（多半锁到随车动的东西）时先退一步换视角。用**带转向的后退弧**（转向符号与前进弧相反）能一步同时修 v 和 u。
- ⚠️ **合爪自检不保证夹住**：`verify_grasp` 的两个外观分对比（地面原位 vs 夹持位）真机三验三误判（实际夹住判成空夹），现只打诊断日志不做判定。**要判空夹得靠夹爪端的力/位置反馈，不是画面。** 且"空夹⇒自动重试"的实现里第一步是 `release` ⇒ 误报一次就把已夹住的方块放掉，别加。

### 排查顺序

```bash
uv run tools/carctl.py tracktest --chr --scale 3   # 先确认色度图里目标真的拉得开、坐标是多少
uv run tools/carctl.py tracktest --u <x> --v <y> --w <w> --h <h> --n 3   # 锁不锁得住、耗时多少
uv run tools/carctl.py tracktest --cont -n 10      # 移动目标/相机后再跑，看是否跟住
uv run tools/carctl.py log -t 60 --cat ai         # 看 [track] 的 conf/外观分 与 [grasp] 每步耗时拆解
```

⚠️ **别盲种**：随手给的 (u,v) 很可能落在背景上（色度只 10~17），会得到"锁定后 conf=0.2"而误判成"追踪器坏了"。先 `--chr` 找真目标。
⚠️ **闭环慢先看 `dec`/`trk` 是否成倍膨胀**：膨胀 = 被同核高优先任务饿的 CPU 竞争，不是算法慢。

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
- ⚠️ mbedTLS 握手内存优化依赖自定义核心库：`Arduino15\packages\esp32\tools\esp32s3-libs\3.3.11` 已按 IDF `defconfig` 重编替换，含**五处**配置：SSLin/out 缓冲 `CONFIG_MBEDTLS_SSL_IN/OUT_CONTENT_LEN=8192`（规避内部堆碎片导致的握手失败 `-32512`/`-17040`）；`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`（mbedTLS 缓冲改走 PSRAM，规避内部堆碎片导致的 AI 请求 `-3`）；lwIP TCP 缓冲 32768/16384；WiFi TX 缓存缓冲 `CONFIG_ESP_WIFI_CACHE_TX_BUFFER_NUM=32`（图传丢帧修复的前提，见提交 `cbc6372`——CACHE 类缓冲落在 PSRAM，不吃内部 RAM）；**`CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX=16384`**（默认 32768）——esp32-camera 重开相机要的那块**连续内部 DMA** = 此值/2（VGA RGB565：32768→30720，16384→15360），图传开着时内部池常凑不出 30720 ⇒ 切跟踪失败（`init=-1`，相机留在 deinit 态）；代价是 RGB 多占 ≈16KB 内部 RAM，正好相当于关掉 psram 直写省下的那块。替换时勿用官方同名库覆盖。**重编流程、定制配置与链接脚本补丁见 [`librebuild/README.md`](librebuild/README.md)**（已入库，随提交 `33b18e7`）。⚠️ 链接脚本补丁不在 `lib/ include/ flags/ ld/` 里而在**变体目录**（本板 `PSRAM=opi`+`FlashMode=dio` ⇒ `dio_opi/sections.ld`），覆盖脚本不管它，**升级核心版本后必须重打**。
  - ⚠️ 判断库配置别读错文件：`esp32s3-libs/3.3.11/sdkconfig` **不是**编库时用的那份。真配置在 WSL lib-builder 的产物里（`~/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs/esp32s3/sdkconfig`，入口 `librebuild/run-idflibs.sh`）。曾据此误判 `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` 没开、差点白跑几小时重编——**它早就是 `=y`**。另：`CONFIG_LOG_MAXIMUM_LEVEL=1` ⇒ 驱动的 `ESP_LOGW/I/D` 编译期即被摘掉，**运行时 `esp_log_level_set` 救不回来**，想看见必须重编库。
  - ⚠️ **但 WiFi 缓冲区"个数"上面两份都不算数**：`WIFI_INIT_CONFIG_DEFAULT()` 是**调用方编译期**展开的宏（`esp_wifi.h`），所以 `*_BUFFER_NUM` 由**生效头文件**决定——本板 `PSRAM=opi`+`FlashMode=dio` ⇒ **`dio_opi/include/sdkconfig.h`**（顶层 `include/` 下没有 `sdkconfig.h`）。库自己那份 config 里的个数对运行时**无效**（WSL 产物里写 `STATIC_TX_BUFFER_NUM=32`，实际生效是 **8**）。该头文件在**变体目录**里，`overlay-libs.sh` 不覆盖 ⇒ 与链接脚本同样是**手工补丁**，已就绪于 `librebuild/patches/sdkconfig.h.dio_opi`。改完必须**重编草图**且建议 `--clean`（它在 74 个文件的 `.d` 依赖里）。验证：`uv run python tools/probe_macro.py`（打印编译期真实宏展开，比"编译通过"强）。
  - 上一条所需的一切（编库脚本 + 定制 `defconfig` + 链接脚本补丁）现均已入库于 `librebuild/`；仓库根的 `.trae/` 是更早的排查草稿目录（已 `.gitignore`，不入库），仅当需要追溯当时怎么试出来的才去看。
  - ⚠️ `ai_http.cpp` 里的 `ai_tls_calloc` / `ai_tls_free` 是同一思路的**预留实现，当前并未接线**：全仓库没有任何 `mbedtls_platform_set_calloc_free` 调用点，两者不会被 mbedTLS 使用。上面那条 PSRAM 效果来自核心库的 `EXTERNAL_MEM_ALLOC` 配置本身；改这两个函数不影响 TLS 分配。
  - ⚠️ 编库时 `run-idflibs.sh` 会自动钉两件事，别手工绕过：① 把 `espressif/esp_video` 钉到锁文件里已有的版本（不设上界时组件管理器会去评估新版 manifest 里那条引用了 S3 上不装的 `ESP_VIDEO_USE_CUSTOMIZED_ESP_H264_VERSION` 的 if 规则 ⇒ `kconfig missed_keys` 非空 ⇒ 每轮 `sys.exit(10)`、`FATAL_ERROR: Missing required kconfig option`，3 次重试永远收敛不了）；② 检测到拉下来的 `esp32-camera` **缺 `jpeg_buffer_size` 字段**（旧版）就删掉 lock 那条强制重解析——本工程用这个字段把 VGA JPEG 的 `recv_size` 抬到 256KB 防 FB-OVF，旧版不仅编译报错，默认 `recv_size ≈ 60KB` 还会让高熵帧溢出、硬停图传。
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
| `tracktest` | 追踪器自检（`/tracktest`）：**灰度/色度解码自检**（出 BMP，肉眼确认边缘无错位、按色度峰找真目标坐标）+ **跟踪自检**（种目标连拍，打逐帧 `ok/u/v/conf` 与耗时）—— 不动车、不经 AI，验证追踪器的第一道命令 | `--scale {0,1,2,3}`、`--all`、`--chr`（色度 BMP）、`--u/--v/--w/--h`（给了就进跟踪模式）、`--n`（连拍，上限 20）、`--cont`（沿用当前目标）、`--stop`（停跟踪）、`-o/--out`、`-t/--tag` |
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
uv run tools/carctl.py tracktest --chr --scale 3  # 追踪器自检：出色度图确认目标坐标
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

#### 1.2 代码现状（`f7d7b6b` 核对）

**两条并行的夹取通路**，范式主要落在后者（本地闭环）上：

| | AI 逐步微操 | 本地闭环 `auto_grasp` |
|---|---|---|
| 谁在动 | 云端每轮决策 | 板端 `grasp` worker 每步 |
| 提示词 | 状态判定式（见下） | 不经提示词 |
| 适用 | 目标被遮挡 / 远小 / 需要语义判断 | 目标清晰可见的单点夹取 |

**AI 侧**（`ai_prompt.cpp` 的「物体对准及夹取判定」）已整段重写为**状态式**：按目标相对 `[左指]` 的位置分档（正上方 / 水平正右方 / 偏左偏右过近 / 正下方），结合 `arm low` 与"目标是否进入 `[夹爪前端]`"决定下一步。范式 3 的"先降爪、后对准"落成"需要夹取且目标未进入 `[夹爪前端]` 时，可先 `arm low` 方便对准"。

- ✅ 范式 3/4 的循环语义保留：微调对准先给接近角度、过头就逐轮砍半反向；`zoom` 只用于检查能否夹取与夹后核对。
- ✅ 范式 6/7 里的"目标在夹爪下、**移动时位置不变**（被卡着推走）⇒ **抬臂 + 后退**"已作为 `case 下方` 写进状态判定。
- ◑ 范式 2 的"2/3 高度 或 距离<15"不再是显式判据：机器侧只剩 `AI_APPROACH_STOP_CM`(20) 一个停距，其余交给模型看画面。
- ⚠️ 提示词现在**明确让模型优先用 `auto_grasp`**（"手动对准并不可靠"），`car.grasp`/`clip` 退化为兜底。

**本地侧**：`grasp` 闭环把范式 3/4/7 变成了代码——降爪、旋转对准、边前进边对准、可夹即收；且加了 AI 侧没有的两条物理判据（"动作后目标必须动"、横向漂移预算）。详见「本地追踪与自动夹取」。

位姿锚点：低姿 `ARM_LOW_X/H_CM`=8.0/1.0、固定高位 `ARM_RAISE_X/H_CM`=9.0/9.0、`GRASP_LIFT_CM`=7.0（`Calibration.h`）。

> 抓取流程里**还在拦的机器侧规则**：「`approach` 与合爪/放收不能同轮」+「`auto_grasp` 与 `car` 同轮不能同用」（`ai_round.cpp`，防拿过时坐标空夹；降爪 `arm low` 被**放行**）。

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

### 4. 其他

- **内部堆触底**：长任务期间体征出现 `DMA块最低=0k`、`堆最低=5k`（放大镜 / AI 任务常驻 PSRAM 缓冲，
  `ensure()` 按 16KB 对齐增长）。`DMA块最低` 触底会威胁 WiFi 收包，是本板的全板性红线，
  见 memory `car-wedge-wifi-dead-ble-alive`。
- ⚠️ **`xTaskCreate*` 的 `usStackDepth` 单位是字（4B），不是字节**：写 `16384` 以为有 64KB、实际只 `malloc(16384)` 给了 16KB ⇒ 任务静默溢出 48KB 写进相邻 PSRAM（栈哨兵在声明大小处、查不到）⇒ 表现为**随机故障**。全仓库这类"按字节分配、按字声明"的坑已逐个修掉（`ai_client` / `board_log` / `heap_watch` / `ping_svc` / `app_httpd::ws_stream`），改用 `heap_caps_malloc(N * sizeof(StackType_t), MALLOC_CAP_SPIRAM)`。**新增任务时照此写**。
- ⚠️ **"相机有概率不出帧"= 内部/DMA RAM 被耗尽**，不是随机故障：卡死时体征是 `DMA块(现/低)=29k/低0k`、`堆最低=7k`，且**随运行时间单调退化**（低水位 25k→0），累积到越线才出事。`cam::grab()` 抓帧失败时限频打 `[cam] 抓帧失败! 内部堆=… DMA 最大块=…` 就是这条的诊断入口；`reinit_current()` 是兜底自愈。尚未完全排除的嫌疑：JPEG 软编（内部 malloc）、LwIP/WiFi 缓冲、套接字泄漏、HTTP 响应缓冲。
- **追踪器可用距离窄 / 超近种子是硬边界**：见「本地追踪与自动夹取」。表现为远小目标（<20px）经旋转仍易丢锁，靠"如实中止 + AI 兜底"消化。
- **`tools/shots/` 等调试产物已忽略**：`Stm32-Vision/tools/shots/*`（`frame`/`step`/`tracktest` 的落图）、
  `Stm32-Vision/logs/*`、`tools/__pycache__/*` 均在根 `.gitignore` 里。另：根目录 `log.txt` 已不在磁盘上，
  `.gitignore` 里也**没有** `log.txt` / `*.log` 规则。

## 协议参考

统一架构与完整协议定义见仓库根 [`../CLAUDE.md`](../CLAUDE.md) 与手机端 [`../Mobile-RemoteCtrl/net/proto/CommandProto.gd`](../Mobile-RemoteCtrl/net/proto/CommandProto.gd)。**该词表由手机端定义，本板逐字对齐解析**（`command.cpp` 的 `type` 分支 / `direct_exec.cpp` 的 params）。

| type | params | 落点 |
| --- | --- | --- |
| `move` | `throttle`(-1..1) `steering`(-1..1)、可选 `distance_cm` | 四轮 PWM（×1000）/ 转向舵 146±30；带 `distance_cm` 按标定表换算时长到点自停 |
| `stop` | `scope` = all/wheels/arm | 停轮 或 清连续机械臂动作 |
| `spin` | `dir` = `+1`右转/`-1`左转/`0`停、`speed`(0..1000)、可选 `angle_deg`、可选 `ms` | 左右轮反向 PWM（原地旋转）；`angle_deg` 查表插值换算时长到点自停；**`ms` 优先于 `angle_deg`**（显式通电毫秒，跳过角度换算与滑行补偿，标定口用）；**两者都不给 = 持续旋转，不自动停**，只能靠后续指令收尾 |
| `arm` | `act` = lift_up/lift_down/reach_forward/reach_backward/**low**/**clip**/**grasp**/release/**fold**、可选 `dist_cm` | 舵机 2/3/4。`low`=低姿夹取准备位、`grasp`=合爪+定量抬臂一步到位（省一轮往返）、`fold`=收臂折叠回平台位。⚠️ AI 侧另有 `raise`/`pose`/**`place_done`** 三个 act（`low` 也走这条），**不在这张表里**——它们在 `ai_round.cpp` 直接调 `exec::arm_low/arm_raise/arm_pose` 或自行编排，不经词表 |
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
| `ai_goal` / `ai_oneshot` / `ai_cancel` | `message`、可选 `annotation` `use_image` | `ai_client` DIRECT 闭环。`annotation` 带 `{x,y,w,h}`（手机左上角+宽高）时**额外 `track::seed`** 锁本地跟踪 |
| `ai_chat` | `message` | 任务进行中插话补充（`ai::append_chat`，不打断闭环）；当前无 AI 任务则忽略 |
| `ai_clear` | — | **开新会话**：中止在跑的任务 + 清空历史环/笔记/物体记忆/车位姿（`ai::session_clear`）。会话上下文默认跨任务保留，只有这条真正重置 |
| `auto_grasp` | `x` `y`（目标画面**中心**，0~1）、可选 `w` `h` `name` | **本地自动夹取**（`grasp::request`，纯画面闭环不经 AI）；先打断在跑的 AI；`grasp::busy()` 时回执拒 |
| `goto` | `x` `y`、可选 `frame`=local(默认)/global | `ai::goto_target`：由板端自行导航到指定坐标 |

**板端 → 手机（上行）**：`status`（指令回执，带 `bits`）· `state`（`get_state` 回包，带 `bits`）· `pong`（保活应答，带 `bits`）· `log`（`{src,text}`，按 `/log` 开关转发）· `exec_status`（周期状态文本，默认关）· `mem`（各堆/DMA 水位诊断）· `ai_result`（AI 回执：`reason` / 内嵌 `command` / `done`；`say` 与结束语都走它的 `reason`）· `ai_tool`（每个工具落地的进度行，**不走 `/log` 开关**）· `ai_task`（任务面板快照：`{state,round,goal?,note?,tasks:[{name,done}]}`，供手机端顶部悬浮面板）· `ai_mem`（`{hd,objs:[{name,r,f,stale}]}`，车头系 cm，**手机记忆地图的数据源**；动作落地后每轮推一次）· `track`（本地追踪可视化：`{u,v,conf,st,novid}` 10Hz；`st=seed` 时带 AI 标的框 `{bx,by,bw,bh}`；跟踪结束补 `st=idle`。⚠️ **`novid=1` 表示跟踪期板端不推视频**，手机据此改显"标尺网格 + 跟踪点"占位）。

⚠️ 保活/纯查询三件套（`ping` / `pong` / `get_state`）**不打「收到…」那行日志**——手机每几秒就一来一回，打出来只是刷屏，而它们的应答本身就是自描述的状态。

BLE UUID / 广播名与手机 `../Mobile-RemoteCtrl/net/ble/BleProfile.gd` **逐字 mirror**：改一侧必须同步另一侧（服务 `0000C0DE-…`，特征 `C0E0`~`C0E6`，广播名 `VisionS3`）。

哪吒 I2C 命令表现只有本板 `nezha_direct.cpp` 一处实现，**无对侧可 mirror**：改字节前须对照哪吒扩展板硬件协议。

## 关联项目

- 手机 App：`../Mobile-RemoteCtrl`（图传显示 / 会话回放 / 指令编辑 / 下发 `ai_goal` / 标框触发 `auto_grasp` / BLE 配网）。

## 约定

- 与用户交流使用中文。
- 编译验证：用户未明确要求时，**不主动跑 arduino-cli 编译验证**（esp32 单次 ~80s+ 起步、IDE↔命令行互切会各自全量重编，耗时无谓）；日常编译/烧录验证默认交给用户在 IDE 里做。确需命令行核对时，用「构建要点」里与 IDE 逐字一致的同一 fqbn。
- 指令协议、注释保持简洁；避免在注释里写死具体数值（参数调整时容易忘改）。
- **串口日志卫生**：手动指令只在类型切换时打一行「收到手动指令」，避免摇杆高频帧刷屏并阻塞控制时序；IDF 系统日志 `esp_log_level_set("*", ESP_LOG_WARN)` 默认静到 WARN，避免与手动指令 ack 争用同一 UART0。
- 大模型返回的指令必须严格校验后再执行，防止异常 JSON 导致小车误动作。
- 硬件标定值（舵机限位、IK 几何、移动时长表）集中在 `Calibration.h`，改动前确认已实测。
