# CLAUDE.md

> 本文档基准：仓库 HEAD `f7d7b6b`（2026-10-03）。只覆盖已提交内容；未提交改动不收录。

本项目维护指南，供后续编码助手 / 会话快速对齐上下文。

## 项目性质

- **Godot 4.7.1 mono（mobile，竖屏** **`window/handheld/orientation=1` = Portrait）+ GDScript**，目标平台 Android（project.godot 与 AndroidManifest 均为 portrait）。

- 一台 **ESP32-S3-CAM 视觉/控制大脑板（`../Stm32-Vision`）** 的小车 App。板子经软件 I2C 直驱哪吒扩展板，手机侧只发词表指令、不感知直驱细节。

- 跨子板总览见仓库根 `../CLAUDE.md`。

## 通信架构（链路）

连接策略统一收口在 `net/DeviceConn.gd`（**单一事实源**；Main 只订阅其统一信号，不在别处另写一套）。它自建并持有 BLE / WS / UDP 三个传输并派生统一 `state`；**WS 优先、BLE 兜底**（`BLOCKED_TYPES` 黑名单外的类型均可蓝牙，现仅 `stream` 图传被拦）。信道切换：WS 上连让出 BLE 射频；WS 下连保 BLE / 双断自动重扫 → 发现最近设备自连 →「连上停扫」。`BLEClient` / `WSCarClient` / `UDPVideoClient` 只做纯传输，不做连接策略。

| 通道 | 用途 | 实现状态 |
|---|---|---|
| BLE（GATT） | 配网 + 兜底控制 + status | GDBLE 接通（`BLEClient.gd` → `addons/gdble` + 协议表 `BleProfile.gd`）；板侧 GATT Server VisionS3 已烧录 |
| WiFi WebSocket（端口 81） | 指令 / 状态 / 消息 / `ai_result` / `ai_tool` / `ai_task` / `ai_mem` / `track`（**文本 JSON**） | 真实（`WSCarClient.gd`；图传已不走 WS） |
| WiFi UDP | 图传 JPEG 分片（低延迟；缺片/超时自愈） | 真实（`UDPVideoClient.gd`，分片协议与板侧 `udp_send_frame` 对齐；另有 `frame_jpeg` 旁路信号给录制用） |
| 云端多模态 AI | DIRECT：板子直调云端，手机只下发 `ai_goal` | DIRECT（不经手机侧） |

- **统一命令词表** `CommandProto`：摇杆 / 指令 / 图传三入口共用（DIRECT），固件只解析这一份。现行词表：`move`（可带 `distance_cm` 定距）/ `stop`（scope=all/wheels/arm）/ `arm`（act 含 `low` 低姿夹取准备位、`grasp` 合爪+定量抬臂、`fold` 收臂折叠回平台；可选 `dist_cm` 近似拍数）/ `spin`（原地旋转，可带 `angle_deg` 定角或 `ms` 直给通电毫秒）/ `light` / `reset` / `stream` / `log`（`cat=exec|ai|all`,`on` 统一日志转发）/ `get_state`（回 `bits` 位图）/ `config` / `ping` / `pong`（板端保活应答，**带 `bits`**）/ `ai_goal` / `ai_oneshot` / `ai_cancel` / `ai_chat`（`message` 插话）/ **`ai_clear`**（开新会话）/ `goto`（`x`/`y`，可选 `frame`）/ **`auto_grasp`**（`x`/`y` 中心 + 可选 `w`/`h`/`name`，**本地**自动夹取）/ `nz_read` / `reboot`（远程重启板子），另含调试直驱 `servo / motor / drive / arm_pose`。
- **AI 链路**：DIRECT（手机下发 `ai_goal` 文字/区域目标 → 板子 `ai_client` 执行闭环并回 `ai_result`）。`ai_oneshot` = 只执行一轮决策即收尾。AI 运行中发送键变「中止」：空文本走 `ai_cancel`，非空文本走 `ai_chat` 插话（补充要求、不打断闭环）。
- ⚠️ **会话上下文跨任务保留**：板端任务结束**不释放**历史环/笔记/目标/任务列表/物体记忆/车位姿，接着发新目标即延续。**只有 `/clear`**（→ `ChatPanel.save_current_session()`：归档本趟录制 + 清场 + 下发 `ai_clear`）才真正重置。`/clear` 与会话面板的「保存」按钮**完全等价**。
- **AI 任务面板（`ai_task` 上行）**：板端推 `{state,round,goal?,note?,tasks:[{name,done}]}`（`state` = `running|wait|done|fail|abort`，与 `ai_result` 同一条结果队列）。`Main.gd` 的 `ai_task` 分支 → `ChatPanel.show_task` → 顶部悬浮面板；面板**只渲染最新快照**，不写聊天流、不下发指令。`wait`（AI 在等你回话）与收尾态由面板自己上色，用户点中止 / 掉线时按本地乐观态收尾（`mark_abort_if_live`）。
- **记忆地图（`ai_mem` 上行）**：板端推 `{hd, objs:[{name,r,f,stale}]}`（车头系 cm，右+/前+）。AI 接管模式下画面源可切「图传 ↔ 记忆」，`ui/video/MemoryMap.gd` 画成车头系俯视图（原点=车、车头朝上、半屏 100cm 刻度），位置走补间做出"跟着车动"的观感。⚠️ 板端**不再上报车自身全局 x/y**（里程会漂，喂给 AI 反而误导），这里也用不到。
- **追踪可视化（`track` 上行）**：板端本地追踪时 10Hz 推 `{u,v,conf,st,novid}`，`ui/video/TrackOverlay.gd` 画十字准星（绿=tracking / 黄=locking / 红=lost）。`novid=1` = **跟踪期板端不推视频** ⇒ 藏掉冻结的旧画面、自动开标尺网格、只显示跟踪点（退出时**必须还原用户自己的网格开关**）。`st=seed` 那条带 AI 标的框 `{bx,by,bw,bh}`，画在切跟踪画面前的最后一帧上（标歪/框太大时一眼看得见）。⚠️ `VideoView.set_track_novid` 里 Feed 只是隐藏、texture 保留——网格与十字都要用它算 KEEP_ASPECT 的内容矩形。
- **本地自动夹取（`auto_grasp`）**：手机上**框选目标 → 标注工具条 `GraspBtn`**，取到区域就自动收起画布并下发（板端自己闭环，不调云端）。空标注不收起、留画布由 Main 提示。手机给的是**左上角 + 宽高**，`Main.gd` 换算成中心再发。
- **AI 工具进度（`ai_tool` 上行）**：板端每个工具落地后推 `{text}`（**不走 `/log` 开关**），`Main.gd` 转 `ChatPanel.chat("AI工具", …)`，不开日志也能看见执行轨迹。
- **定距 / 定角**：`move` 的 `distance_cm`、`spin` 的 `angle_deg` 由板端按**时长近似**到点自停（无里程计，靠实测标定表插值），非闭环，供微操与标定粗用；不带则持续动作，靠 `stop` 收尾。`spin` 另有 `ms`（直接给通电毫秒，绕开角度换算与滑行补偿）——**板端 `ms` 优先于 `angle_deg`**，但 `CommandProto.spin` 是 `angle_deg` 优先、`ms` 走 `elif`，故两侧实际只会发其中一个。
- **调试与本地指令**：移动/直驱按族收敛——`/move rotate|spin|spin_ms|fore|back|to|arm`（转向舵三档 / 原地旋转 / **`spin_ms <毫秒>`** 直接指定通电毫秒、绕开角度换算（标定"真实每度 ms"用）/ 定距前进后退 / `to <x> <y> [global]` 移动到指定坐标 → `goto` / 机械臂位姿，`arm` 含 `reset|fold|low`）与 `/drive motor|servo`（单轮电机或 n=0 全车 / 直驱舵机）经词表下发；`/log <exec|ai|all> [on|off]`（旧 `/exec_log`、`/ai_log` 为别名）与 `/nz_read`（哪吒 I2C 探测）同理；`/grid [on|off]` 只在本机图传上叠加标定网格（`ui/video/GridOverlay.gd`，不下发板子，**无画面时按 VGA 4:3 兜底画标尺**——跟踪期板端不推流，标尺不能跟着画面一起消失）；`/clear` = **归档本趟录制 + 清聊天区/任务面板/本地日志 + 开新会话**（`save_current_session()`，见上）；`/append` 打开系统图库选图（与输入框 `AppendImgBtn` 同效，压到 ≤20KB 后进标注编辑器）。
- **重连同步**：WS 连上后主动发一次 `get_state`，板端回 `type:"state"`，`Main.gd._apply_state` → `DirectControl.sync_state` 同步灯光/夹爪按钮（`set_pressed_no_signal`，不回灌指令）。
- **保活即纠偏**：板端每个心跳周期的 `pong` 都带**当场现测的** `bits`，`WSCarClient` 收到后**照旧上抛**（不吞），由 `Main.gd` 的 `pong` 分支 `_apply_state_bits(bits, true)` 同步——`apply_ai=true`，即**连 AI 运行态也一并覆盖**。这是「AI 任务在跑、发送按钮却没变成中止」的兜底：任务由另一侧起停、或漏收了一次 `ai_result` 时，按钮会在一个心跳周期内自行纠正。`pong` 只同步不打印（否则每几秒一行噪声）。
  ⚠️ bit 布局与板端 `command.cpp::state_bits()` **逐位 mirror，改一侧必改另一侧**：灯 bit0-2（front/vibe/back）、夹爪 bit3、AI 运行态 bit4。

## 目录结构

```
res://
  Main.tscn / Main.gd          # App 壳：连接编排、页面切换（含左右滑动切页）、**模式与画面源**、连接状态。控制页 `BodyControl` 自上而下：`GeneralCtrl`（图传开关 `StreamToggle` + 模式切换 `CtrlModeToggle`）/ `ImgEditToolbar`（矩形/椭圆/撤销/清除/取消/采用/**自动夹取 GraspBtn**）/ `Video`（`Feed` + `NoSignal` + `Overlay`=网格 + **`TrackOverlay`** + **`OpMap`**=记忆地图）/ `AIModeCtrl`（画面源 / 会话入口 `SessionBtn` / 标注 `AnnotateBtn`，**仅 AI 接管模式显示**）/ `ChatPanel` / `CtrlArea`（摇杆 + 机械臂 + 灯光，**仅手动模式显示**）。「关于」页设置项仍是自连 / 禁用自动 WS / 原地旋转模式。**可见性一律经 `_apply_layout()` 收口**（模式 / 回放 / 图传 / 标注 / 画面源五个条件），别在别处各改各的 visible
  state/LocalStore.gd          # autoload 本地持久化（last_device / wifi / ai 配置 / 设置项：自连·禁用自动WS·原地旋转·图传开关·**ai_mode** / `input_history` 输入历史）
  state/AppLog.gd              # autoload 本地日志落盘：每次启动截断重写 user://logs/app.log，聊天区每行统一写入；`clear()` 供 `/clear` 截断
  state/Recorder.gd            # autoload 会话录制与回放（详见下方「会话录制与回放」）
  animation/AnimationManager.gd # autoload 通用动画（淡入+缩放滑入/滑出）
  net/
    DeviceConn.gd              # 统一连接层（单一事实源）：持有 BLE/WS/UDP、send_command 统一出口、最新帧 current_image、原始 JPEG 旁路 `frame_jpeg`
    proto/CommandProto.gd      # 统一命令词表（static）
    ws/WSCarClient.gd          # WS 传输（文本 JSON：指令/状态/ai_result/ai_tool/ai_task/ai_mem/track）
    ble/BLEClient.gd           # BLE GATT 客户端（GDBLE 运行时：扫描/连接/读写/配网/status）
    ble/BleProfile.gd          # 协议常量表（UUID/广播名/BLE 黑名单 BLOCKED_TYPES，与固件 ble.cpp 逐字 mirror）
    video/UDPVideoClient.gd    # UDP 图传接收：JPEG 分片重组 → frame_received（+ `frame_jpeg` 旁路，**解码前**的原始字节，只给录制用）
  addons/
    gdble/                     # GDBLE 插件运行时（*.aar + libgdble.so，编译产物）
    gdble_export/              # 导出插件（Android libraries + manifest 注入）
  ui/
    control/DirectControl.gd   # 直控面板（脚本建树）：机械臂/灯光按钮 + **摇杆映射**（Godot 内置 `VirtualJoystick` 写 `vjoy_*` action，本脚本还原方向向量并量化下发；死区 0.15、低速 0.5/高速 1.0、转向 0.8、原地旋转 800）。`manual_takeover` / `ai_busy_changed` 信号；`apply_state_bits` 与板端 `state_bits` 逐位 mirror
    video/VideoView.gd         # 图传显示（脚本建树）：`set_frame` / `set_track_target` / `set_seed_box` / `set_track_novid` / `show_map` / `show_map_data`；子节点 `Overlay`=GridOverlay、`TrackOverlay`、`OpMap`=MemoryMap
    video/GridOverlay.gd       # `/grid` 标定网格（本地叠加，不下发板子）；无画面时按 VGA 4:3 兜底画
    video/TrackOverlay.gd      # 跟踪目标可视化：十字准星 + 置信度（绿 track / 黄 lock / 红 lost）+ AI 标的种子框
    video/MemoryMap.gd         # 记忆地图：把 `ai_mem` 画成车头系俯视图（原点=车、车头朝上、cm）；只在「画面源=记忆」时显示，位置走补间
    PopupWindow.gd             # **通用弹窗**（节点树在 `Main.tscn` 的 `PopupWindow`，脚本建内部 UI）：`TabContainer` 两页——`WifiCfg`（连接前填 WiFi + AI 接口，**模型名从服务端拉列表后下拉选**）+ `Message`（一句话确认，如删除会话）。`open_provision()` / `open_confirm()`，结果经信号回抛，弹窗不持有业务状态
    BgDimSharder.gd            # 弹窗背景遮罩（节点挂 Main 根 `BGDimSharder`）：mouse_filter=STOP 吃掉下层触控，显隐跟随 PopupWindow 的 `visibility_changed`
    chat/ChatPanel.gd          # 聊天区视图；消息类含「AI」/「AI工具」/「状态」/日志；指令解析收口在 SlashCommands.gd；底部 `BottomPanl` 三页（指令提示 / 图片附件 / **回放控制条**）；回放接口 `begin_replay`/`replay_apply`/`end_replay`/`set_input_enabled`
    chat/SlashCommands.gd      # /指令 解析器（文本 → 词表指令/本地动作，纯解析、无副作用）
    chat/TaskPanel.gd          # AI 任务面板：悬浮在 ChatLog 顶部（节点树 `ChatPanel/ChatLog/TaskPanl`），折叠一行、点击展开；数据源 `ai_task`。⚠️ 展开区高度由脚本按字体实测折行数自算（面板是浮层，不走容器测量），按聊天区高度封顶、超出交给滚动条 —— 板端任务项数不封顶，别改回"一项一行"
    replay/PlaybackBar.gd      # 回放控制条（`ChatPanel/ChatLog/BottomPanl/PlaybackCtrl`：进度条 + 上一步/暂停/下一步），只发 `step_requested`/`toggle_pause_requested`/`seek_requested`
    replay/SessionList.gd      # 会话列表面板（`ChatPanel/ChatLog/SessionPanel`）：`Session1`=当前会话，`Session2..10`=归档（上限 9），每行带删除；由 AI 接管栏 `SessionBtn` 开关
    bluetooth/BTDeviceListItem.tscn+.gd  # 蓝牙设备列表项（图标改用 `icon set.png` 的 AtlasTexture）
    bluetooth/ScanPanel.gd     # 蓝牙扫描页（设备列表/刷新动画/空提示，挂 BodyBTScan 节点）
    editor/ImageEditor.gd + EditorCanvas.gd  # 图传画面上的标注：工具条在 `Main.tscn`（`BodyControl/ImgEditToolbar`），画布 `EditorCanvas` 由 ImageEditor 运行时创建并挂到 `BodyControl/Video`（**无 SubViewport**）；矩形 + 椭圆两种图形；「采用」作为附件随 `ai_goal` 上行，**「自动夹取」`GraspBtn` 则取到区域就收起画布并下发 `auto_grasp`**（不上传图，板端自己取帧）
  res/                         # `icon set.png` 雪碧图（各 UI 图标用 AtlasTexture 切图，旧的十余张独立 png 已删）；`bg.jpg` 背景、`icon.png` 应用图标、`MapleMono-NF-CN-Regular.ttf`
```

## 会话录制与回放（`state/Recorder.gd`，autoload）

- **逻辑时钟**：只在 AI 运行期前进（`set_ai_running` 跟着 `ChatPanel.set_ai_running` 走）——"AI 暂停 + 用户插话"在回放里是一瞬间，空闲等待自动被抹掉。
- **录什么**：聊天行（`chat`）、任务面板快照（`task`）、图传帧（`frame`，限频 ≈10fps 且隔帧存一帧）、**跟踪点（`track`）**——即使 AI 已停也照录 `novid` 翻转，否则回放会卡在跟踪画面上。帧走 `DeviceConn.frame_jpeg`（解码前原始字节），不重新压。
- **落盘**（依次追加，不在内存里堆整趟）：`user://sessions/index.json`（≤9 条，新→旧）+ `<id>/{meta.json, events.jsonl（每行 `{t,k,...}`，k=chat|task|frame|track）, frames/<seq>.jpg}`。当前趟先写 `_cur/`，`/clear` 归档时整体改名。**没跑过 AI 的空趟整趟丢弃**（归档时判断）。
- **回放是只读的**：`is_playing()` 为真时 `DeviceConn.send_command`/`send_image` **一律拒发**（在唯一出口拦，UI 漏禁也发不出去），输入框与发送键禁用，画面源/图传开关/模式切换/标注全部 disabled（回放里是 no-op）。**唯一保持可用的是 `SessionBtn`**——它是回放的唯一出口。单步以「AI 决策点」（`AI工具`/`AI` 的发言）为锚而非逐条日志；**按事件推进而不是按时间**（空闲期时间戳全相同，按时间回退会一步跳回原地）；跳走时 `_drop_decode()` 丢掉在途旧帧解码，否则会残留跳走前的画面。

## 重要约定（务必遵守）

1. **不要用 root 脚本里的** **`%唯一名`** **查找**：本项目环境中 root 脚本的 `%Name`（owner 唯一名）查找会失败。统一用：

   - 场景内取子节点：`@onready var _x: Type = $Path/To/Node`

   - 跨场景实例节点：Main.gd 用 `$路径` 引用，访问其脚本方法用 `.call("方法", args)`（避免对基类 `Control` 做静态成员访问）。
2. **脚本间复用的工具类用** **`preload`，不要依赖** **`class_name`** **全局注册**：headless/CLI 下全局类缓存可能未扫描导致 `Identifier not declared`。已统一用 `const CP := preload("res://net/proto/CommandProto.gd")`。
3. **显式类型注解**：从 `%`/`$`/`Node.method()` 等 Variant 来源赋值时，禁止 `var x := ...` 推断（会触发 warning-as-error）。写成 `var x: String = ...`、`var x: Image = ...`、`var x: float = min(...)`。
4. **`unique_name_in_owner`** **写布尔时用无引号** **`= true`**（写 `="true"` 不会被解析为布尔）。
5. **tscn 不可手写错**：ext\_resource / sub\_resource 块用 `]` 闭合，别写成 `)`；格式 `format=3`。
6. **⚠️ 收起摇杆控制区时务必 `release_all()`**：隐藏正在拖拽的摇杆可能收不到 release，会把车留在"前进"（`_apply_layout` 已做，别绕过它直接改 visible）。
7. **⚠️ 回放是只读态**：`is_playing()` 时一切下发被 `DeviceConn` 拒发。别在回放里"顺手加个下发"——那等于拿历史界面在开车。

## 代码风格（延续既有偏好）

- 注释精简、**不写具体魔法数字**（调参时避免手改注释）；文本颜色用 `theme_override_colors/font_color`。

- 复用组件优先（摇杆 / 动画管理 / 弹窗）；不做超出需求的过度设计。

- 组件独立成场景+脚本，动画统一走 `AnimationManager`。

- **摇杆用 Godot 内置 `VirtualJoystick`**（节点在 `Main.tscn` 的 `BodyControl/CtrlArea/Joystick`），映射逻辑集中在 `DirectControl.gd`（不再是自绘控件 + Main 里的映射）；油门满量程 `_DRIVE_MAX`=1000，直接驱动时油门 → 全车 `drive`，左右 → 转向舵（150±30）。

- **摇杆两套转向行为**（「关于」页 `SpinMode` 设置项切换，经 `LocalStore` 持久化）：关 = 左右推杆发 `servo`（转向舵，需前轮回正）；开 = 发 `spin` 原地旋转（左右推杆按 `_SPIN_SPEED`=800 定速，速度低于实测拖动线会拖不动），松手发 `spin(0)` 停。

## 构建环境

- Android SDK / NDK：装在本机**非默认目录**，具体位置以 Godot「编辑器设置 → 导出 → Android」里的 SDK 路径为准（也可由环境变量 `ANDROID_HOME` 给出）。已装：platforms android-36，build-tools 36.1/37，NDK 30.0.15729638，platform-tools。

- JDK：Java 21，`JAVA_HOME` 指向本机 JDK 21 安装目录（Godot「编辑器设置 → 导出 → Android」里有对应设置项）。

- **导出 = 编译动作**（非“无需编译”）：用 Godot 编辑器 headless 导出 Android debug APK。实测命令：
  `"<Godot 4.7.1 mono 可执行文件>" --headless --path <工程根> --export-debug "Android" <输出.apk>`（preset 名 `Android`；Windows 上是 `Godot_v4.7.1-stable_mono_win64.exe`，其它平台换对应可执行文件）。

- **GDBLE 已集成**（非待办）：Java/AAR 部分已编译就绪于 `addons/gdble/android/*.aar` + 导出插件 `addons/gdble_export`；Rust 源在独立仓库 `gdble`（**与本仓库同级的 `../gdble`**，仓库外，未收录进本容器）。若要改 btleplug/Java 侧需重编 AAR 的 classes.jar 再导出。

- 导出预置：`export_presets.cfg` 中 `gradle_build/use_gradle_build=true`，但实际走的是 **Godot 标准模板导出**（未真正跑 gradle assemble）；`plugins/GDBLE=false`、`plugins/GDBLEBridge=false`（插件经导出插件注入，不勾这两个开关）。`permissions/internet=true` 必开（否则发不出 WS 与拉模型列表）。

- ⚠️ **`project.godot` 的 physics 引擎设成了 `Dummy`**（2d/3d 都是）：本项目没有任何 2D/3D 物理，纯 UI 工程，换回 Jolt 只会白占启动时间。`assembly_name="小车遥控"`。

- **已知坑（MIUI/HyperOS BLE 扫描结果门禁）**：Godot 导出器把 **toggle 勾出来的** `ACCESS_FINE_LOCATION` 固定写成 `android:maxSdkVersion="30"`（API≥31 等于未声明），MIUI 蓝牙栈投递扫描结果前仍检查该权限（日志 `Permission denial: Need ACCESS_FINE_LOCATION...`），不声明+不授予则 onScanResult 永不回调。
  **解**：`export_presets.cfg` 里 `access_fine_location=true` **且** `custom_permissions` 含 `android.permission.ACCESS_FINE_LOCATION`，二者**缺一不可**——Godot 会给 custom 那条逐字写一条**无 cap** 的 FINE，与 toggle 的 capped 条共存，Android≥31 认无 cap 那条 → 运行时权限 → `Main.gd._request_ble_permissions()` 启动弹窗授权即过门禁。只 toggle 或只 custom 都会退回单条 capped（无效）。apktool 后处理与 `adb pm grant` 不再需要。

- **扫描必现 `扫描失败: JNI call failed`（2026-09-06 解）**：该字面量是 jni crate 对 `Error::JniCall(ThreadDetached)` 的 Display = 某线程**未 attach JVM** 就调进 Java。gdble 的 gdble-core 工作线程驱动 btleplug 前须 `attach_current_thread_permanently`（`src/android.rs::attach_core_thread`，core.rs 线程闭包调用）；若 .so 缺这段，`start_scan` 里 `global_jvm().get_env()` 直接 JNI_EDETACHED。**根因：主工程 `addons/gdble/android/gdble-release.aar` 里的 libgdble.so 是缺 attach 的旧编译产物**；worktree 同目录 AAR 含 attach、才是好的——AAR 是二进制品，不同步导致从主工程导出的包必坏。重编 gdble 后要把 `gdble/target/aarch64-linux-android/release/libgdble.so` 换入**主工程** AAR。查 .so 含不含 attach：字节里搜 `attach_current_thread_permanently failed` / `[GDBLE] Failed to attach core thread`。另：`BLEClient._on_error` 在扫描态失败也发 `scan_finished([])`，下拉框不再卡"扫描中…"。
