# CLAUDE.md

本项目维护指南，供后续编码助手 / 会话快速对齐上下文。

## 项目性质

- **Godot 4.7.1 mono（mobile，竖屏** **`window/handheld/orientation=1` = Portrait）+ GDScript**，目标平台 Android。（旧文档误写“横向”；project.godot 与 AndroidManifest 均为 portrait。）

- 一条 **STM32（电机控制）+ 独立 ESP32（摄像头 / WiFi / BLE / 云端 AI 客户端）** 的小车，经 UART 串接。

- 阶段：**UI 骨架 + WiFi WebSocket 真实 + BLE 已接 GDBLE（真机扫描/发现/列表均验证通过，配网仍为桩）+ 云端 AI 仍为桩**。未接真实硬件 / 真实 API。

- 完整架构设计见 `.trae/documents/ctrl-app-architecture-and-ui-plan.md`。

## 通信架构（三通道）

| 通道             | 用途                     | 实现状态                 |
| -------------- | ---------------------- | -------------------- |
| BLE            | 一次性配网 + 兜底控制           | GDBLE 接通（`BLEClient.gd` → `addons/gdble`）；扫描→发现→列表真机验证通过（25+ 周边设备），配网仍为桩 |
| WiFi WebSocket | 图传 JPEG 帧 / 指令 / AI 消息 | 真实（`WSCarClient.gd`） |
| 云端多模态 AI       | 中转 / 小车直连              | 桩（`AIClient.gd`）     |

- **统一命令词表** `CommandProto`：手动摇杆、手机中转审批、小车直连三来源共用，保证小车固件只解析一份协议。词表：`move / stop / snapshot / stream / config / ping`。

- 两条 AI 链路：**手机中转**（可编辑图片后发云端 + 人工审批卡）与**小车直连**（ESP32 直达云端，手机仅观察）。

## 目录结构

```
res://
  Main.tscn / Main.gd          # App 壳：视图状态机 + 布局拼装
  state/AppState.gd            # autoload 全局状态（连接引用、AI 模式、待审批工具）
  animation/AnimationManager.gd # autoload 通用动画（淡入+缩放滑入/滑出、上下浮动）
  net/
    proto/CommandProto.gd      # 统一命令词表（static）
    ws/WSCarClient.gd          # WebSocket 客户端（真实现）
    ble/BLEClient.gd           # BLE（GDBLE 运行时，扫描/发现流程在调）
    ai/AIClient.gd             # 云端 AI 桩
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

- **已知坑（MIUI/HyperOS BLE 扫描结果门禁）**：Godot 导出器把 `ACCESS_FINE_LOCATION` 固定写成 `android:maxSdkVersion="30"`（API≥31 等于未声明），MIUI 蓝牙栈投递扫描结果前仍检查该权限（日志 `Permission denial: Need ACCESS_FINE_LOCATION...`），不声明+不授予则 onScanResult 永不回调。规避 = 导出后 apktool 全解码、去掉该行 maxSdk 再 `zipalign` + `apksigner`（`~/.android/debug.keystore`，口令 android）重新签名；装前先 `adb uninstall`（签名密钥不同会 INSTALL_FAILED_UPDATE_INCOMPATIBLE）。装后 `pm grant` 授予 BLUETOOTH_SCAN / BLUETOOTH_CONNECT / ACCESS_FINE_LOCATION。

## 后续待办（不在当前阶段）

- BLE（已解决，2026-09-05）：真机“刷新恒 0 设备”根因不是 gdble 扫描——btleplug Java `onScanResult` 正常大量回调、gdble 返回 25+ 周边设备，是 `BLEClient.gd:_labels` 对 `"name": null` 的设备字典做 `var name: String = d.get("name","")` 赋值，取到 Nil 触发运行时错误中断函数，`address` 兜底永远走不到、结果恒 `[]`。已改为显式判 null（name 为 null 时回退 address）。配网 GATT 仍需固件侧协议后接入。

- 接入真实云端多模态 API（配置 Key）。

- 小车固件侧：ESP32（摄像头 + WS 服务器 + AI 直连 + UART）与 STM32（词表解析 + 电机控制）联调。

