# CLAUDE.md

> 本文档基准：仓库 HEAD `18ef9c1`（2026-09-29）。只覆盖已提交内容；未提交改动不收录。

本项目维护指南，供后续编码助手 / 会话快速对齐上下文。

## 项目性质

- **Godot 4.7.1 mono（mobile，竖屏** **`window/handheld/orientation=1` = Portrait）+ GDScript**，目标平台 Android（project.godot 与 AndroidManifest 均为 portrait）。

- 一台 **ESP32-S3-CAM 视觉/控制大脑板（`../Stm32-Vision`）** 的小车 App。板子经软件 I2C 直驱哪吒扩展板，手机侧只发词表指令、不感知直驱细节。

- 跨子板总览见仓库根 `../CLAUDE.md`。

## 通信架构（链路）

连接策略统一收口在 `net/DeviceConn.gd`（**单一事实源**；Main 只订阅其统一信号，不在别处另写一套）。它自建并持有 BLE / WS / UDP 三个传输并派生统一 `state`；**WS 优先、BLE 兜底**（`BLOCKED_TYPES` 黑名单外的类型均可蓝牙，现仅 `stream` 图传被拦）。信道切换：WS 上连让出 BLE 射频；WS 下连保 BLE / 双断自动重扫 → 发现最近设备自连 →「连上停扫」。`BLEClient` / `WSCarClient` / `UDPVideoClient` 只做纯传输，不做连接策略。

| 通道 | 用途 | 实现状态 |
|---|---|---|
| BLE（GATT） | 配网 + 兜底控制 + status | GDBLE 接通（`BLEClient.gd` → `addons/gdble` + 协议表 `BleProfile.gd`）；板侧 GATT Server VisionS3 已烧录（联调中） |
| WiFi WebSocket（端口 81） | 指令 / 状态 / 消息 / `ai_result` / `ai_tool` / `ai_task`（**文本 JSON**） | 真实（`WSCarClient.gd`；图传已不走 WS） |
| WiFi UDP | 图传 JPEG 分片（低延迟；缺片/超时自愈） | 真实（`UDPVideoClient.gd`，分片协议与板侧 `udp_send_frame` 对齐） |
| 云端多模态 AI | DIRECT：板子直调云端，手机只下发 `ai_goal` | DIRECT（不经手机侧） |

- **统一命令词表** `CommandProto`：摇杆 / 指令 / 图传三入口共用（DIRECT），固件只解析这一份。现行词表：`move`（可带 `distance_cm` 定距）/ `stop`（scope=all/wheels/arm）/ `arm`（act 含 `low` 低姿夹取准备位、`grasp` 合爪+定量抬臂、`fold` 收臂折叠回平台；可选 `dist_cm` 近似拍数）/ `spin`（原地旋转，可带 `angle_deg` 定角或 `ms` 直给通电毫秒）/ `light` / `reset` / `stream` / `log`（`cat=exec|ai|all`,`on` 统一日志转发）/ `get_state`（回 `bits` 位图）/ `config` / `ping` / `pong`（板端保活应答，**带 `bits`**）/ `ai_goal` / `ai_oneshot` / `ai_cancel` / `ai_chat`（`message` 插话）/ `goto`（`x`/`y`，可选 `frame`）/ `nz_read` / `reboot`（远程重启板子），另含调试直驱 `servo / motor / drive / arm_pose`。
- **AI 链路**：DIRECT（手机下发 `ai_goal` 文字/区域目标 → 板子 `ai_client` 执行闭环并回 `ai_result`）。`ai_oneshot` = 只执行一轮决策即收尾。AI 运行中发送键变「中止」：空文本走 `ai_cancel`，非空文本走 `ai_chat` 插话（补充要求、不打断闭环）。
- **AI 任务面板（`ai_task` 上行）**：板端推 `{state,round,goal?,note?,tasks:[{name,done}]}`（`state` = `running|wait|done|fail|abort`，与 `ai_result` 同一条结果队列）。`Main.gd` 的 `ai_task` 分支 → `ChatPanel.show_task` → 顶部悬浮面板；面板**只渲染最新快照**，不写聊天流、不下发指令。`wait`（AI 在等你回话）与收尾态由面板自己上色，用户点中止 / 掉线时按本地乐观态收尾（`mark_abort_if_live`）。
- **AI 工具进度（`ai_tool` 上行）**：板端每个工具落地后推 `{text}`（**不走 `/log` 开关**），`Main.gd` 转 `ChatPanel.chat("AI工具", …)`，不开日志也能看见执行轨迹。
- **定距 / 定角**：`move` 的 `distance_cm`、`spin` 的 `angle_deg` 由板端按**时长近似**到点自停（无里程计，靠实测标定表插值），非闭环，供微操与标定粗用；不带则持续动作，靠 `stop` 收尾。`spin` 另有 `ms`（直接给通电毫秒，绕开角度换算与滑行补偿）——**板端 `ms` 优先于 `angle_deg`**，但 `CommandProto.spin` 是 `angle_deg` 优先、`ms` 走 `elif`，故两侧实际只会发其中一个。
- **调试与本地指令**：移动/直驱按族收敛——`/move rotate|spin|spin_ms|fore|back|to|arm`（转向舵三档 / 原地旋转 / **`spin_ms <毫秒>`** 直接指定通电毫秒、绕开角度换算（标定"真实每度 ms"用）/ 定距前进后退 / `to <x> <y> [global]` 移动到指定坐标 → `goto` / 机械臂位姿，`arm` 含 `reset|fold|low`）与 `/drive motor|servo`（单轮电机或 n=0 全车 / 直驱舵机）经词表下发；`/log <exec|ai|all> [on|off]`（旧 `/exec_log`、`/ai_log` 为别名）与 `/nz_read`（哪吒 I2C 探测）同理；`/grid [on|off]` 只在本机图传上叠加标定网格（`ui/video/GridOverlay.gd`，不下发板子），配合板端单应标定读 (u,v) 取标定点；`/clear` 除清聊天区外**同时截断重写本地日志文件**（`AppLog.clear()`）。
- **重连同步**：WS 连上后主动发一次 `get_state`，板端回 `type:"state"`，`Main.gd._apply_state` → `DirectControl.sync_state` 同步灯光/夹爪按钮（`set_pressed_no_signal`，不回灌指令）。
- **保活即纠偏**：板端每个心跳周期的 `pong` 都带**当场现测的** `bits`，`WSCarClient` 收到后**照旧上抛**（不吞），由 `Main.gd` 的 `pong` 分支 `_apply_state_bits(bits, true)` 同步——`apply_ai=true`，即**连 AI 运行态也一并覆盖**。这是「AI 任务在跑、发送按钮却没变成中止」的兜底：任务由另一侧起停、或漏收了一次 `ai_result` 时，按钮会在一个心跳周期内自行纠正。`pong` 只同步不打印（否则每几秒一行噪声）。
  ⚠️ bit 布局与板端 `command.cpp::state_bits()` **逐位 mirror，改一侧必改另一侧**：灯 bit0-2（front/vibe/back）、夹爪 bit3、AI 运行态 bit4。

## 目录结构

```
res://
  Main.tscn / Main.gd          # App 壳：连接编排、页面切换（含左右滑动切页）、摇杆映射、连接状态；「关于」页设置项（自连 / 禁用自动 WS / 原地旋转模式）；**图传开关、直控面板开关、标注入口在控制页 `BodyControl/VidControls`（`StreamToggle`/`DirectCtrlToggle`/`AnnotateBtn`），不在「关于」页**；图片标注的**工具条**（`BodyControl/ImgEditToolbar`：矩形/椭圆/撤销/清除/取消/采用）内联于本场景，标注画布由 `ui/editor/ImageEditor.gd` 运行时挂到 `BodyControl/Video` 上
  state/LocalStore.gd          # autoload 本地持久化（last_device / wifi / ai 配置 / 设置项 / `input_history` 输入历史）
  state/AppLog.gd              # autoload 本地日志落盘：每次启动截断重写 user://logs/app.log，聊天区每行统一写入；`clear()` 供 `/clear` 截断
  animation/AnimationManager.gd # autoload 通用动画（淡入+缩放滑入/滑出、上下浮动）
  net/
    DeviceConn.gd              # 统一连接层（单一事实源）：持有 BLE/WS/UDP、send_command 统一出口、最新帧 current_image
    proto/CommandProto.gd      # 统一命令词表（static）
    ws/WSCarClient.gd          # WS 传输（文本 JSON：指令/状态/ai_result/ai_tool/ai_task）
    ble/BLEClient.gd           # BLE GATT 客户端（GDBLE 运行时：扫描/连接/读写/配网/status）
    ble/BleProfile.gd          # 协议常量表（UUID/广播名/BLE 黑名单 BLOCKED_TYPES，与固件 ble.cpp 逐字 mirror）
    video/UDPVideoClient.gd    # UDP 图传接收：JPEG 分片重组 → frame_received
  addons/
    gdble/                     # GDBLE 插件运行时（*.aar + libgdble.so，编译产物）
    gdble_export/              # 导出插件（Android libraries + manifest 注入）
  ui/
    control/Joystick.gd + DirectControl.gd  # 复用虚拟摇杆 / 直控面板（脚本建树，无 tscn）
    video/VideoView.gd         # 图传显示（脚本建树）；子节点 Overlay = GridOverlay.gd（/grid 标定网格）
    BgDimSharder.gd            # 弹窗背景遮罩（节点挂 Main 根 `BGDimSharder`，脚本在本目录）：mouse_filter=STOP 吃掉下层触控（翻页手势/摇杆/按钮），点空白处关闭弹窗；显隐跟随配网弹窗
    chat/ChatPanel.gd          # 聊天区视图；消息类含「AI」（`ai_result` 的 reason）/「AI工具」（`ai_tool` 轨迹）/「状态」/日志；指令解析收口在 SlashCommands.gd
    chat/SlashCommands.gd      # /指令 解析器（文本 → 词表指令/本地动作，纯解析，无副作用）
    chat/TaskPanel.gd          # AI 任务面板：悬浮在 ChatLog 顶部（节点树在 Main.tscn：`ChatPanel/ChatLog/TaskPanl`，`VBox/Head` + `VBox/Body(ScrollContainer)/Inner/{Note,Todos}`），折叠一行、点击展开；数据源 `ai_task`。⚠️ 展开区高度由脚本按字体实测折行数自算（面板是浮层，不走容器测量），按聊天区高度封顶、超出交给滚动条 —— 板端任务项数不封顶，别改回"一项一行"
    bluetooth/BTDeviceListItem.tscn+.gd  # 蓝牙设备列表项
    bluetooth/ScanPanel.gd     # 蓝牙扫描页（设备列表/刷新动画/空提示，挂 BodyBTScan 节点）
    editor/ImageEditor.gd + EditorCanvas.gd  # 图传画面上的标注：工具条在 `Main.tscn`（`BodyControl/ImgEditToolbar`），标注画布 `EditorCanvas` 由 ImageEditor 运行时创建并挂到 `BodyControl/Video`（**无 SubViewport**）；矩形 + 椭圆两种图形，「采用」后作为附件随 `ai_goal`/`ai_oneshot` 上行
    provision/WifiConfigPopup.gd  # 配网弹窗（脚本建树）
```

## 重要约定（务必遵守）

1. **不要用 root 脚本里的** **`%唯一名`** **查找**：本项目环境中 root 脚本的 `%Name`（owner 唯一名）查找会失败。统一用：

   - 场景内取子节点：`@onready var _x: Type = $Path/To/Node`

   - 跨场景实例节点：Main.gd 用 `$路径` 引用，访问其脚本方法用 `.call("方法", args)`（避免对基类 `Control` 做静态成员访问）。
2. **脚本间复用的工具类用** **`preload`，不要依赖** **`class_name`** **全局注册**：headless/CLI 下全局类缓存可能未扫描导致 `Identifier not declared`。已统一用 `const CP := preload("res://net/proto/CommandProto.gd")`。
3. **显式类型注解**：从 `%`/`$`/`Node.method()` 等 Variant 来源赋值时，禁止 `var x := ...` 推断（会触发 warning-as-error）。写成 `var x: String = ...`、`var x: Image = ...`、`var x: float = min(...)`。
4. **`unique_name_in_owner`** **写布尔时用无引号** **`= true`**（写 `="true"` 不会被解析为布尔）。
5. **tscn 不可手写错**：ext\_resource / sub\_resource 块用 `]` 闭合，别写成 `)`；格式 `format=3`。

## 代码风格（延续既有偏好）

- 注释精简、**不写具体魔法数字**（调参时避免手改注释）；文本颜色用 `theme_override_colors/font_color`。

- 复用组件优先（摇杆 / 动画管理）；不做超出需求的过度设计。

- 组件独立成场景+脚本，动画统一走 `AnimationManager`。

- 摇杆油门满量程 `Main.gd` 的 `_DRIVE_MAX`；直接驱动时油门 → 全车 `drive`，左右 → 转向舵。

- **摇杆两套转向行为**（「关于」页 `SpinMode` 设置项切换，经 `LocalStore` 持久化）：关 = 左右推杆发 `servo`（转向舵，需前轮回正）；开 = 发 `spin` 原地旋转（左右推杆按 `_SPIN_SPEED` 定速，速度低于实测拖动线会拖不动），松手发 `spin(0)` 停。

## 构建环境

- Android SDK / NDK：装在本机**非默认目录**，具体位置以 Godot「编辑器设置 → 导出 → Android」里的 SDK 路径为准（也可由环境变量 `ANDROID_HOME` 给出）。已装：platforms android-36，build-tools 36.1/37，NDK 30.0.15729638，platform-tools。

- JDK：Java 21，`JAVA_HOME` 指向本机 JDK 21 安装目录（Godot「编辑器设置 → 导出 → Android」里有对应设置项）。

- **导出 = 编译动作**（非“无需编译”）：用 Godot 编辑器 headless 导出 Android debug APK。实测命令：
  `"<Godot 4.7.1 mono 可执行文件>" --headless --path <工程根> --export-debug "Android" <输出.apk>`（preset 名 `Android`；Windows 上是 `Godot_v4.7.1-stable_mono_win64.exe`，其它平台换对应可执行文件）。

- **GDBLE 已集成**（非待办）：Java/AAR 部分已编译就绪于 `addons/gdble/android/*.aar` + 导出插件 `addons/gdble_export`；Rust 源在独立仓库 `gdble`（**与本仓库同级的 `../gdble`**，仓库外，未收录进本容器）。若要改 btleplug/Java 侧需重编 AAR 的 classes.jar 再导出。

- 导出预置：`export_presets.cfg` 中 `gradle_build/use_gradle_build=true`，但实际走的是 **Godot 标准模板导出**（未真正跑 gradle assemble）；`plugins/GDBLE=false`、`plugins/GDBLEBridge=false`（插件经导出插件注入，不勾这两个开关）。

- **已知坑（MIUI/HyperOS BLE 扫描结果门禁）**：Godot 导出器把 **toggle 勾出来的** `ACCESS_FINE_LOCATION` 固定写成 `android:maxSdkVersion="30"`（API≥31 等于未声明），MIUI 蓝牙栈投递扫描结果前仍检查该权限（日志 `Permission denial: Need ACCESS_FINE_LOCATION...`），不声明+不授予则 onScanResult 永不回调。
  **解**：`export_presets.cfg` 里 `access_fine_location=true` **且** `custom_permissions` 含 `android.permission.ACCESS_FINE_LOCATION`，二者**缺一不可**——Godot 会给 custom 那条逐字写一条**无 cap** 的 FINE，与 toggle 的 capped 条共存，Android≥31 认无 cap 那条 → 运行时权限 → `Main.gd._request_ble_permissions()` 启动弹窗授权即过门禁。只 toggle 或只 custom 都会退回单条 capped（无效）。apktool 后处理与 `adb pm grant` 不再需要。

- **扫描必现 `扫描失败: JNI call failed`（2026-09-06 解）**：该字面量是 jni crate 对 `Error::JniCall(ThreadDetached)` 的 Display = 某线程**未 attach JVM** 就调进 Java。gdble 的 gdble-core 工作线程驱动 btleplug 前须 `attach_current_thread_permanently`（`src/android.rs::attach_core_thread`，core.rs 线程闭包调用）；若 .so 缺这段，`start_scan` 里 `global_jvm().get_env()` 直接 JNI_EDETACHED。**根因：主工程 `addons/gdble/android/gdble-release.aar` 里的 libgdble.so 是缺 attach 的旧编译产物**；worktree 同目录 AAR 含 attach、才是好的——AAR 是二进制品，不同步导致从主工程导出的包必坏。重编 gdble 后要把 `gdble/target/aarch64-linux-android/release/libgdble.so` 换入**主工程** AAR。查 .so 含不含 attach：字节里搜 `attach_current_thread_permanently failed` / `[GDBLE] Failed to attach core thread`。另：`BLEClient._on_error` 在扫描态失败也发 `scan_finished([])`，下拉框不再卡"扫描中…"。
