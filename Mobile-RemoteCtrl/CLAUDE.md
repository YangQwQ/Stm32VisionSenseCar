# CLAUDE.md

> 本文档基准：仓库 HEAD `9917330`（2026-09-10）。只覆盖已提交内容；未提交改动不收录。

本项目维护指南，供后续编码助手 / 会话快速对齐上下文。

## 项目性质

- **Godot 4.7.1 mono（mobile，竖屏** **`window/handheld/orientation=1` = Portrait）+ GDScript**，目标平台 Android。（旧文档误写“横向”；project.godot 与 AndroidManifest 均为 portrait。）

- 一台 **ESP32-S3-CAM 视觉/控制大脑板（`../Stm32-Vision`）** 的小车 App。⚠️ **原 STM32 执行板（`../Stm32-Executor`）已裁撤**：板子经软件 I2C 直驱哪吒扩展板，手机侧不感知该差异（词表结构不变）。

- 跨子板总览见仓库根 `../CLAUDE.md`。

## 通信架构（链路）

连接策略统一收口在 `net/DeviceConn.gd`（**单一事实源**；Main / AppState 只订阅其统一信号，不在别处另写一套）。它自建并持有 BLE / WS / UDP 三个传输并派生统一 `state`；**WS 优先、BLE 兜底**（`FALLBACK_TYPES` 白名单）。信道切换：WS 上连让出 BLE 射频；WS 下连保 BLE / 双断自动重扫 → 发现最近设备自连 →「连上停扫」。`BLEClient` / `WSCarClient` / `UDPVideoClient` 只做纯传输，不做连接策略。

| 通道 | 用途 | 实现状态 |
|---|---|---|
| BLE（GATT） | 配网 + 兜底控制 + status | GDBLE 接通（`BLEClient.gd` → `addons/gdble` + 协议表 `BleProfile.gd`）；板侧 GATT Server VisionS3 已烧录（联调中） |
| WiFi WebSocket（端口 81） | 指令 / 状态 / 消息 / `ai_result`（**文本 JSON**） | 真实（`WSCarClient.gd`；图传已不走 WS） |
| WiFi UDP | 图传 JPEG 分片（低延迟；缺片/超时自愈） | 真实（`UDPVideoClient.gd`，分片协议与板侧 `udp_send_frame` 对齐） |
| 云端多模态 AI | DIRECT：板子直调云端，手机只下发 `ai_goal` | 桩（`AIClient.gd`；不经手机侧） |

- **统一命令词表** `CommandProto`：摇杆 / 指令 / 图传三入口共用（DIRECT），固件只解析这一份。现行词表：`move / stop / arm / light / reset / stream / exec_log / config / ping / ai_goal / ai_oneshot / ai_cancel`，另含调试直驱 `servo / motor / drive / arm_pose`。（`snapshot`、`exec_forward` 已移除。）
- **AI 链路**：DIRECT（手机下发 `ai_goal` 文字/区域目标 → 板子 `ai_client` 执行闭环并回 `ai_result`）；手机中转（RELAY）已移除。`ai_oneshot` = 只执行一轮决策即收尾。

## 目录结构

```
res://
  Main.tscn / Main.gd          # App 壳：连接编排、摇杆映射、图传开关、连接状态、布局
  ui/chat/ChatPanel.gd         # 聊天/指令区：消息日志、指令提示、附件列表、指令解析与发送
  state/AppState.gd            # autoload 全局状态（连接引用 + send_command 统一出口：WS 优先、BLE 兜底）
  animation/AnimationManager.gd # autoload 通用动画（淡入+缩放滑入/滑出、上下浮动）
  net/
    DeviceConn.gd              # 统一连接层：持有 BLE/WS/UDP，收敛状态与重连策略（单一事实源）
    proto/CommandProto.gd      # 统一命令词表（static）
    ws/WSCarClient.gd          # WS 传输（文本 JSON：指令/状态/ai_result）
    ble/BLEClient.gd           # BLE GATT 客户端（GDBLE 运行时：扫描/连接/读写/配网/status）
    ble/BleProfile.gd          # 协议常量表（UUID/广播名/兜底白名单，与固件 ble.cpp 逐字 mirror）
    video/UDPVideoClient.gd    # UDP 图传接收：JPEG 分片重组 → frame_received
    ai/AIClient.gd             # 云端 AI 桩（DIRECT 不经手机侧，未接）
  addons/
    gdble/                     # GDBLE 插件运行时（*.aar + libgdble.so，编译产物）
    gdble_export/              # 导出插件（Android libraries + manifest 注入）
  ui/
    control/Joystick.gd + DirectControl.gd  # 复用虚拟摇杆 / 直控面板（脚本建树，无 tscn）
    video/VideoView.gd         # 图传显示（脚本建树）
    chat/ChatPanel.gd          # 聊天区视图（脚本建树）
    bluetooth/BTDeviceListItem.tscn+.gd  # 蓝牙设备列表项
    editor/ImageEditor.tscn+.gd + EditorCanvas.gd  # 图片标注（框/箭头/文字）
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

## 构建环境

- Android SDK / NDK：`D:\AndroidSDK`（platforms android-36，build-tools 36.1/37，NDK 30.0.15729638，platform-tools）。

- JDK：Java 21，`JAVA_HOME=D:\Soft\JAVA\jdk-21`。

- **导出 = 编译动作**（非“无需编译”）：用 Godot 编辑器 headless 导出 Android debug APK。实测命令：
  `"D:/PortableApp/Godot/Godot_v4.7.1-stable_mono_win64.exe" --headless --path <工程根> --export-debug "Android" <输出.apk>`（preset 名 `Android`）。

- **GDBLE 已集成**（非待办）：Java/AAR 部分已编译就绪于 `addons/gdble/android/*.aar` + 导出插件 `addons/gdble_export`；Rust 源在独立仓库 `D:\Downloads\Git\gdble`（仓库外，未收录进本容器）。若要改 btleplug/Java 侧需重编 AAR 的 classes.jar 再导出。

- 导出预置：`export_presets.cfg` 中 `gradle_build/use_gradle_build=true`，但实际走的是 **Godot 标准模板导出**（未真正跑 gradle assemble）；`plugins/GDBLE=false`、`plugins/GDBLEBridge=false`（插件经导出插件注入，不勾这两个开关）。

- **已知坑（MIUI/HyperOS BLE 扫描结果门禁）**：Godot 导出器把 **toggle 勾出来的** `ACCESS_FINE_LOCATION` 固定写成 `android:maxSdkVersion="30"`（API≥31 等于未声明），MIUI 蓝牙栈投递扫描结果前仍检查该权限（日志 `Permission denial: Need ACCESS_FINE_LOCATION...`），不声明+不授予则 onScanResult 永不回调。
  **已解（2026-09-06，双声明，无需 apktool/pm grant）**：`export_presets.cfg` 里 `access_fine_location=true` **且** `custom_permissions` 含 `android.permission.ACCESS_FINE_LOCATION`，二者**缺一不可**——Godot 会给 custom 那条逐字写一条**无 cap** 的 FINE，与 toggle 的 capped 条共存，Android≥31 认无 cap 那条 → 运行时权限 → `Main.gd._request_ble_permissions()` 启动弹窗授权即过门禁。只 toggle 或只 custom 都会退回单条 capped（无效）。apktool 后处理与 `adb pm grant` 不再需要。

- **已知坑（扫描必现 `扫描失败: JNI call failed`，2026-09-06 已解）**：该字面量是 jni crate 对 `Error::JniCall(ThreadDetached)` 的 Display = 某线程**未 attach JVM** 就调进 Java。gdble 的 gdble-core 工作线程驱动 btleplug 前须 `attach_current_thread_permanently`（`src/android.rs::attach_core_thread`，core.rs 线程闭包调用）；若 .so 缺这段，`start_scan` 里 `global_jvm().get_env()` 直接 JNI_EDETACHED。**根因：主工程 `addons/gdble/android/gdble-release.aar` 里的 libgdble.so 是旧编译产物（缺 attach）**；worktree 同目录 AAR（09-05 20:36）含 attach、才是好的——AAR 是二进制品，不同步导致从主工程导出的包必坏。重编 gdble 后要把 `gdble/target/aarch64-linux-android/release/libgdble.so` 换入**主工程** AAR。查 .so 含不含 attach：字节里搜 `attach_current_thread_permanently failed` / `[GDBLE] Failed to attach core thread`。另：`BLEClient._on_error` 在扫描态失败也发 `scan_finished([])`，下拉框不再卡"扫描中…"。

## 后续待办（不在当前阶段）

- BLE（已解决，2026-09-05）：真机“刷新恒 0 设备”根因不是 gdble 扫描——btleplug Java `onScanResult` 正常大量回调、gdble 返回 25+ 周边设备，是 `BLEClient.gd:_labels` 对 `"name": null` 的设备字典做 `var name: String = d.get("name","")` 赋值，取到 Nil 触发运行时错误中断函数，`address` 兜底永远走不到、结果恒 `[]`。已改为显式判 null（name 为 null 时回退 address）。配网 GATT 两侧代码已接（见下）。

- **遗留命名**：板侧本地直驱状态（`exec_status`）在 `Main.gd` / `ChatPanel.gd` 中仍以 `"执行板"` 作为消息来源标签显示；执行板已裁撤，该标签属历史命名，如需改为「状态」需同步两处。
