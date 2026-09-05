# CLAUDE.md

本项目维护指南，供后续编码助手 / 会话快速对齐上下文。

## 项目性质

- **Godot 4.7（mobile，横向** **`orientation=1`）+ GDScript**，目标平台 Android。

- 一条 **STM32（电机控制）+ 独立 ESP32（摄像头 / WiFi / BLE / 云端 AI 客户端）** 的小车，经 UART 串接。

- 阶段：**UI 骨架 + 通信层桩（Stub）**，BLE 与云端 AI 均为桩，未接真实硬件 / 真实 API。

- 完整架构设计见 `.trae/documents/ctrl-app-architecture-and-ui-plan.md`。

## 通信架构（三通道）

| 通道             | 用途                     | 实现状态                 |
| -------------- | ---------------------- | -------------------- |
| BLE            | 一次性配网 + 兜底控制           | 桩（`BLEClient.gd`）    |
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
    ble/BLEClient.gd           # BLE 桩
    ai/AIClient.gd             # 云端 AI 桩
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

- 当前 UI 骨架为纯 GDScript + 内置 WebSocket，**无需编译**。需要编译的工作：

  - **导出 Android APK**（经 Godot 编辑器 Android 导出，需在编辑器设置指向上方 SDK/NDK 路径）。

  - **集成 GDBLE**（Android BLE 插件，需 JDK17+ + SDK/NDK + cargo-ndk，编译 `addons/gdble`）。

- 相关导出预置：`export_presets.cfg` 中 `gradle_build/use_gradle_build` 当前为 `false`（标准模板导出，无需自定义 gradle）；接插件时需改为 true 并配置模板目录。

## 后续待办（不在当前阶段）

- 集成并编译 GDBLE 插件，接通真实 BLE 配网与兜底控制（需 JDK17 + SDK34 + NDK + cargo-ndk）。

- 接入真实云端多模态 API（配置 Key）。

- 小车固件侧：ESP32（摄像头 + WS 服务器 + AI 直连 + UART）与 STM32（词表解析 + 电机控制）联调。

