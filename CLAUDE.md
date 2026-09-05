# CLAUDE.md

本项目维护指南，供后续编码助手 / 会话快速对齐上下文。

## 项目性质

- **Godot 4.7.1 mono（mobile，竖屏** **`window/handheld/orientation=1` = Portrait）+ GDScript**，目标平台 Android。（旧文档误写“横向”；project.godot 与 AndroidManifest 均为 portrait。）

- 一条 **STM32（电机控制）+ 独立 ESP32（摄像头 / WiFi / BLE / 云端 AI 客户端）** 的小车，经 UART 串接。

- 阶段：**UI + WiFi WebSocket 真实 + BLE 全链路代码就绪（GDBLE 扫描/连接/GATT 读写 + 配网下发，真机扫描验证过）+ 板侧 GATT/指令/UART 模块已写（固件未编译）+ 云端 AI 仍为桩（DIRECT）**。真机联调未做（设备未到）。进度见文末「双侧进度」。

- 完整架构设计见 `.trae/documents/ctrl-app-architecture-and-ui-plan.md`。

## 通信架构（三通道）

| 通道             | 用途                     | 实现状态                 |
| -------------- | ---------------------- | -------------------- |
| BLE            | 一次性配网 + 兜底控制           | GDBLE 接通（`BLEClient.gd` → `addons/gdble` + 协议表 `BleProfile.gd`）；扫描→发现→列表真机验证过；连接→GATT 读写→配网（SSID/PASS/CMD 特征）已实现；板侧 GATT Server VisionS3 已写（固件未编译联调） |
| WiFi WebSocket | 图传 JPEG 帧 / 指令 / AI 消息 | 真实（`WSCarClient.gd`） |
| 云端多模态 AI       | 中转 / 小车直连              | 桩（`AIClient.gd`）     |

- **统一命令词表** `CommandProto`：摇杆 / 聊天 / 图传三入口共用（DIRECT），固件只解析这一份。词表：`move / stop / arm / snapshot / stream / config / ping / ai_goal`。

- AI 链路：**DIRECT**（手机下发 ai_goal 文字/区域目标 → 板子执行并回 status）。手机中转（RELAY/审批卡）已移除。云端调用两侧均为桩。

## 目录结构

```
res://
  Main.tscn / Main.gd          # App 壳：统一聊天入口（/ 指令 + 纯文本 ai_goal）、连接状态、布局
  state/AppState.gd            # autoload 全局状态（连接引用 + send_command 统一出口：WS 优先、BLE 兜底）
  animation/AnimationManager.gd # autoload 通用动画（淡入+缩放滑入/滑出、上下浮动）
  net/
    proto/CommandProto.gd      # 统一命令词表（static）
    ws/WSCarClient.gd          # WebSocket 客户端（真实现）
    ble/BLEClient.gd           # BLE GATT 客户端（GDBLE 运行时：扫描/连接/读写/配网/status）
    ble/BleProfile.gd          # 协议常量表（UUID/广播名/兜底白名单，与固件 ble.cpp 逐字 mirror）
    ai/AIClient.gd             # 云端 AI 桩（未接）
  addons/
    gdble/                     # GDBLE 插件运行时（*.aar + libgdble.so，编译产物）
    gdble_export/              # 导出插件（Android libraries + manifest 注入）
  .trae/documents/             # 架构设计文档
  ui/
    control/Joystick.tscn+.gd  # 复用虚拟摇杆
    video/VideoView.tscn+.gd   # 图传显示
    editor/ImageEditor.tscn+.gd + EditorCanvas.gd  # 图片标注（框/箭头/文字）
    provision/WifiConfigPopup.tscn+.gd  # 配网弹窗
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

## 构建环境

- Android SDK / NDK：`D:\AndroidSDK`（platforms android-36，build-tools 36.1/37，NDK 30.0.15729638，platform-tools）。

- JDK：Java 21，`JAVA_HOME=D:\Soft\JAVA\jdk-21`。

- **导出 = 编译动作**（非“无需编译”）：用 Godot 编辑器 headless 导出 Android debug APK。实测命令：
  `"D:/PortableApp/Godot/Godot_v4.7.1-stable_mono_win64.exe" --headless --path <工程根> --export-debug "Android" <输出.apk>`（preset 名 `Android`）。

- **GDBLE 已集成**（非待办）：Java/AAR 部分已编译就绪于 `addons/gdble/android/*.aar` + 导出插件 `addons/gdble_export`；Rust 源在独立仓库 `D:\Downloads\Git\gdble`。若要改 btleplug/Java 侧需重编 AAR 的 classes.jar 再导出。

- 导出预置：`export_presets.cfg` 中 `gradle_build/use_gradle_build=true`，但实际走的是 **Godot 标准模板导出**（未真正跑 gradle assemble）；`plugins/GDBLE=false`、`plugins/GDBLEBridge=false`（插件经导出插件注入，不勾这两个开关）。

- **已知坑（MIUI/HyperOS BLE 扫描结果门禁）**：Godot 导出器把 **toggle 勾出来的** `ACCESS_FINE_LOCATION` 固定写成 `android:maxSdkVersion="30"`（API≥31 等于未声明），MIUI 蓝牙栈投递扫描结果前仍检查该权限（日志 `Permission denial: Need ACCESS_FINE_LOCATION...`），不声明+不授予则 onScanResult 永不回调。
  **已解（2026-09-06，双声明，无需 apktool/pm grant）**：`export_presets.cfg` 里 `access_fine_location=true` **且** `custom_permissions` 含 `android.permission.ACCESS_FINE_LOCATION`，二者**缺一不可**——Godot 会给 custom 那条逐字写一条**无 cap** 的 FINE，与 toggle 的 capped 条共存，Android≥31 认无 cap 那条 → 运行时权限 → `Main.gd._request_ble_permissions()` 启动弹窗授权即过门禁。只 toggle 或只 custom 都会退回单条 capped（无效）。apktool 后处理与 `adb pm grant` 不再需要。

- **已知坑（扫描必现 `扫描失败: JNI call failed`，2026-09-06 已解）**：该字面量是 jni crate 对 `Error::JniCall(ThreadDetached)` 的 Display = 某线程**未 attach JVM** 就调进 Java。gdble 的 gdble-core 工作线程驱动 btleplug 前须 `attach_current_thread_permanently`（`src/android.rs::attach_core_thread`，core.rs 线程闭包调用）；若 .so 缺这段，`start_scan` 里 `global_jvm().get_env()` 直接 JNI_EDETACHED。**根因：主工程 `addons/gdble/android/gdble-release.aar` 里的 libgdble.so 是旧编译产物（缺 attach）**；worktree 同目录 AAR（09-05 20:36）含 attach、才是好的——AAR 是二进制品，不同步导致从主工程导出的包必坏。重编 gdble 后要把 `gdble/target/aarch64-linux-android/release/libgdble.so` 换入**主工程** AAR。查 .so 含不含 attach：字节里搜 `attach_current_thread_permanently failed` / `[GDBLE] Failed to attach core thread`。另：`BLEClient._on_error` 在扫描态失败也发 `scan_finished([])`，下拉框不再卡"扫描中…"。

## 后续待办（不在当前阶段）

- BLE（已解决，2026-09-05）：真机“刷新恒 0 设备”根因不是 gdble 扫描——btleplug Java `onScanResult` 正常大量回调、gdble 返回 25+ 周边设备，是 `BLEClient.gd:_labels` 对 `"name": null` 的设备字典做 `var name: String = d.get("name","")` 赋值，取到 Nil 触发运行时错误中断函数，`address` 兜底永远走不到、结果恒 `[]`。已改为显式判 null（name 为 null 时回退 address）。配网 GATT 两侧代码已接（见下）。

- 接入真实云端多模态 API（配置 Key）。

- 小车固件侧：ESP32（摄像头 + WS 服务器 + AI 直连 + UART）与 STM32（词表解析 + 电机控制）联调。

## 双侧进度（Phase B，2026-09-06；代码就绪、未编译/真机验证）

手机侧（本工程，改动已从 worktree 同步回主目录 `D:\Downloads\Git\Ctrl-App`）：
- BLE 配网闭环：`BLEClient.provision()` 写 SSID/PASS 特征 → 板落 NVS 重启 → BLE status 报 `ip` → `AppState` 自动 `ws.connect_car_ip()`。代码全就绪。
- 统一聊天入口（`Main._handle_slash`）：`/` 前缀=指令（`/ping /snapshot /stream [on|off] /stop [wheels|arm] /config /goal`），纯文本=AI 目标（DIRECT `ai_goal`，可带框选区域）；发送统一 `AppState.send_command`（WS 优先，BLE 兜底走 `BleProfile.FALLBACK_TYPES` 白名单）。
- RELAY / 独立 AILog / 审批卡已删；摇杆 / `DirectControl` 爪控 / 图传开关均走 `send_command`。
- 校验：Godot headless 项目级编译零脚本错误 + 主场景实例化通过（2026-09-05）。

板子侧 `D:\Downloads\Git\Stm32-Vision`（同 Phase B 改，未 commit/未编译）：
- `uart` / `command` / `ble` 三组模块已写：UART 帧+CRC16+词表→帧（uart.cpp）；统一词表分发、传输无关回调应答（command.cpp）；BLE GATT Server VisionS3——cmd 写队列、wifi/ai 配置写特征、status 读+通知（ble.cpp）。`app_httpd.cpp` WS 文本→command + `set_ws_connected`；`.ino` setup/loop 已接线。
- UUID/广播名与 `net/ble/BleProfile.gd` 逐字 mirror；词表 type 与 `CommandProto` 一致。ai_goal 板侧为桩回复；执行板 UART 联调待做。

联调遗留：① BLE 配网后板重启，手机需**重连一次 BLE** 收 ip 才能自动连 WS；② arm 词表 `duration_ms` 板侧 uart 按 `dist_cm` 判定（UI 现均发 0，语义一致，非 0 定时版未实现）。

