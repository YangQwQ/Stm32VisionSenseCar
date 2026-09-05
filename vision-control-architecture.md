# Vision Control 系统架构设计（统一参考）

> 本文件是 ESP32-S3 视觉控制板（Stm32-Vision）与手机 App（Ctrl-App）的共同架构参考。
> 开发任一模块前先读此文件，保证协议与职责一致。
> 指令 JSON 规范**沿用手机端 `net/proto/CommandProto.gd`**，板子固件只解析这一份词表。

## 1. 角色划分

| 角色 | 设备 | 职责 |
|---|---|---|
| 指挥 | 手机 App | 显示图传、编辑画面（圈范围）、下发任务目标文本、虚拟摇杆/按键直控、BLE 配网、中转 AI（可选） |
| 大脑 | ESP32-S3 板（本固件） | 图传、WS 服务器、调用多模态 AI（直连链路）、指令解析/校验/队列、UART 转发 |
| 决策 | 多模态 AI（DeepSeek V4 Flash） | 根据"当前画面 + 任务目标"生成单条动作指令 |
| 执行 | 执行板（单板） | 同一块板同时控制小车移动与机械臂；经 UART 接收动作指令并回传状态 |

**原则：AI 是"被动的"**——只有画面没有目标时 AI 不决策；任务目标只能来自手机。

## 2. 通道与链路（已确认决策）

| 通道 | 用途 | 方向 | 备注 |
|---|---|---|---|
| WiFi WebSocket | **运行态：指令 JSON + JPEG 图传帧** | 双向 | `ws://<板IP>:81`，沿用手机端 WSCarClient；一条连接承载文本指令与二进制帧 |
| WiFi HTTP | 浏览器调试 / 状态查询 | 板 → 浏览器 | 例程保留：`/`、`/stream`（MJPEG）、`/status` |
| BLE | 配网 / 兜底控制 / 状态查询 | 双向 | WiFi 不可用时直接控制也走 BLE；词表 JSON 与 WS 相同 |
| UART | 大脑板 ↔ 执行板（单板，同控小车+机械臂） | 双向 | 帧协议见 §5.4，指令表见 §5.5 |

**已确认决策：**
- 运行态指令默认走 **WebSocket**；BLE 仅配网与 WiFi 不可用时的兜底（词表不变，通道不同）。
- 手机离线（WiFi/BLE 均未连）时，板子**待机**：不调用 AI，只保持图传，等指令。
- AI 每次**只返回单条动作**（词表命令），板子逐条执行。
- 涉及移动的指令都有"**持续移动**"与"**指定距离/角度**"两个版本；持续指令只有收到 `stop` 才停止；持续指令支持速度参数。

## 3. 工作流程

### 3.1 手机中转链路（RELAY，沿用手机端现状）
```
手机编辑图+文本 → 手机调 AI（AIClient.ask）→ 返回"词表命令+reason" → 人工审批卡
→ WS 发板子 → command 校验 → UART 帧 → 执行板
```

### 3.2 板子直连链路（DIRECT）
```
手机 WS 发 ai_goal（目标文本+标注）→ 板子抓当前帧（camera 自采，不回传手机画面）
→ ai_client 调 AI → 返回"词表命令+reason" → command 校验 → UART 帧 → 执行板
```
新 `ai_goal` 到来时，**取消当前 AI 任务**（未执行的 AI 指令清空），开始新目标。

### 3.3 手动控制（抢占）
```
手机摇杆/按键 → move/stop/arm（高优先）→ 清空 AI 队列剩余 → 立即执行
```
手动控制**总是打断** AI 正在进行的操作。

### 3.4 配网
```
BLE：手机下发 WiFi/AI 配置 → NVS 存储 → 重启 → 连 WiFi → 板子经 BLE 上报本机 IP → 手机连 WS
```

## 4. 板子固件模块

| 模块 | 文件 | 职责 | 关键点 |
|---|---|---|---|
| main | `main.ino` | 初始化 + 状态机调度 | 精简例程 setup，不阻塞主循环 |
| camera | `camera.cpp/.h` | OV2640 初始化、抓帧 | JPEG；`CAMERA_GRAB_LATEST` + fb_count=2；提供"取一帧"同步接口给 ai_client 与 WS 推帧 |
| ai_client | `ai_client.cpp/.h` | 调用多模态 AI（DIRECT 链路） | HTTP 上传 JPEG+ai_goal 文本/标注；解析"词表命令+reason"；超时/重试；异步任务，不阻塞图传 |
| command | `command.cpp/.h` | 解析 CommandProto 词表、校验、队列 | 词表与手机端 `CommandProto.gd` 一致；手动指令立即执行（清空队列）；AI 指令入队逐条执行；新 ai_goal 取消旧 AI 任务 |
| uart | `uart.cpp/.h` | 词表 → UART 帧翻译、CRC16、收发 | 映射表见 §5.3；帧协议见 §5.4；执行板状态帧解析（§5.5） |
| ble | `ble.cpp/.h` | 配网 / 兜底控制 / 状态上报 | 特征见 §5.6；配网完成后保持轻量连接供兜底 |
| config | `config.cpp/.h` | NVS 参数持久化 | 配置项见 §7；提供默认值回退 |
| network | `network.cpp/.h` | WiFi STA/AP 管理 | STA 为主；失联重连（IP 变化后 BLE 重报）；`setSleep(false)` |
| web_server | `app_httpd.cpp`（改造） | MJPEG 流（浏览器）+ **WS 服务器** | 例程只标记了 `is_websocket`，**WS 帧收发（httpd_ws_recv/send_frame）需自行实现**：文本=指令 JSON，二进制=JPEG 帧（板→手机） |

**依赖方向**（模块间调用关系）：
```
web_server(WS) ─┐
ai_client(直连) ─┼─▶ command ─▶ uart ─▶ 执行板
手机BLE ──▶ config ─▶ network / ai_client（配置生效）
camera ─▶ ai_client / web_server（供帧）
```

## 5. 通信协议

### 5.1 WebSocket 消息（板子 WS 服务器，端口 81）

| 消息类型 | 方向 | 内容 |
|---|---|---|
| 文本 | 手机 → 板 | 指令 JSON（§5.2 词表） |
| 文本 | 板 → 手机 | 状态/应答 JSON（ping 回包、指令执行结果、执行板状态），结构：`{ "type": "status", "params": {...}, "id": <对应指令id> }` |
| 二进制 | 板 → 手机 | JPEG 帧（图传），每帧一张裸 JPEG（手机端 `load_jpg_from_buffer` 直接解码） |

**图片交换协议**：
- 图传流：`stream {on:true}` 后，板子按设定帧率周期推 JPEG 帧；`stream {on:false}` 停止。
- 单帧请求：`snapshot {quality}` → 板子抓一帧，以单张二进制 JPEG 返回。
- 板子不接收手机回传图片（DIRECT 用板子自采帧，RELAY 手机直接调 AI）。

### 5.2 指令 JSON（统一词表，**与手机端 CommandProto.gd 一致，向后兼容**）

```json
// 移动（摇杆直控；throttle/steering 归一化 -1.0 ~ 1.0）
{ "type": "move", "params": { "throttle": 0.6, "steering": -0.2 }, "id": 1 }
// 可选精确动作（AI 表达时用，缺省=持续移动）
{ "type": "move", "params": { "throttle": 0.5, "steering": 0, "distance_cm": 30 }, "id": 2 }
{ "type": "move", "params": { "throttle": 0, "steering": 0.8, "angle_deg": 90 }, "id": 3 }

// 停止（scope: all / wheels / arm）
{ "type": "stop", "params": { "scope": "all" }, "id": 4 }

// 机械臂（act: lift_up / lift_down / clip / release / reach_forward / reach_backward）
{ "type": "arm", "params": { "act": "lift_up", "duration_ms": 0 }, "id": 5 }
// 可选精确动作：dist_cm 指定距离（缺省=持续移动）
{ "type": "arm", "params": { "act": "reach_forward", "dist_cm": 15, "duration_ms": 0 }, "id": 6 }

// 截图 / 图传开关
{ "type": "snapshot", "params": { "quality": 82 }, "id": 7 }
{ "type": "stream",  "params": { "on": true }, "id": 8 }

// 配网（BLE 或 WS 均可发送同结构）
{ "type": "config", "params": { "ssid": "wifi", "password": "pass" }, "id": 9 }

// 心跳
{ "type": "ping", "params": {}, "id": 10 }

// AI 目标（DIRECT 链路，手机 → 板子；手机端需补 CommandProto.ai_goal）
{ "type": "ai_goal", "params": { "message": "把红色的球推到左前方", "annotation": { "x": 120, "y": 80, "w": 40, "h": 60, "label": "球" } }, "id": 11 }
```

**字段约定：**
- `id`：发送方自增序号，板子可据此去重/应答（应答帧回填同一 id）。
- `move` 可选 `distance_cm`（throttle 方向决定前进/后退）、`angle_deg`（steering 方向决定左/右）；**省略即持续移动**，直到 `stop`。
- `arm` 可选 `dist_cm`（lift/reach 的距离，方向由 act 决定）；`duration_ms=0` 表示持续到 `stop`。
- **AI 返回**（RELAY 或 DIRECT 链路的 AI 输出）＝ 词表命令（move/stop/arm 等）＋ `reason` 人类可读解释字段：
  ```json
  { "type": "move", "params": { "throttle": 0.4, "steering": 0.3 }, "id": 9, "reason": "左前方有障碍物，建议左转绕行" }
  ```
- `ai_goal.annotation`：手机圈选的近似区域（坐标相对手机画面），**视为近似意图**，AI 以语义理解为主，不依赖精确像素；AI 实际使用板子自采的当前帧。

### 5.3 词表 → UART 映射（板子翻译层）

| 词表（手机 / AI → 板子） | UART（板子 → 执行板） |
|---|---|
| `move` throttle≠0，无 distance_cm | 小车 CMD 0x01 持续移动（dir 由符号定，speed=|throttle|） |
| `move` throttle≠0，有 distance_cm | 小车 CMD 0x02 移动指定距离 |
| `move` steering≠0，无 angle_deg | 小车 CMD 0x03 持续转动（dir 由符号定） |
| `move` steering≠0，有 angle_deg | 小车 CMD 0x04 转动指定角度 |
| `move` throttle 与 steering 同时非零 | 移动与转动指令组合发出（执行板能力支持则同帧，否则按执行板约定） |
| `stop {scope}` | 停止帧（scope: 0=all / 1=wheels / 2=arm） |
| `arm {act: lift_up/lift_down}`，无 dist_cm | 机械臂 CMD 0x01 持续升降 |
| `arm {act: lift_up/lift_down}`，有 dist_cm | 机械臂 CMD 0x02 升降指定距离 |
| `arm {act: reach_forward/reach_backward}`，无 dist_cm | 机械臂 CMD 0x03 持续移爪 |
| `arm {act: reach_forward/reach_backward}`，有 dist_cm | 机械臂 CMD 0x04 移爪指定距离 |
| `arm {act: clip/release}` | 机械臂 CMD 0x05 夹取/松夹 |
| （板子定时 / 手机查询） | CMD 0x06 状态查询 |

### 5.4 UART 帧协议（板 ↔ 执行板）

```
AA 55 LEN DEV CMD [PAYLOAD] CRC16
```
- `AA 55`：帧头。
- `LEN`：1 字节，DEV+CMD+PAYLOAD 的字节数。
- `DEV`：1 字节，0x01=小车子系统，0x02=机械臂子系统（同一执行板内区分）。
- `CMD`：1 字节，指令码（见 §5.5）。
- `PAYLOAD`：0~N 字节，多字节整数一律小端。
- `CRC16`：2 字节，帧头之后到 PAYLOAD 末尾的 CRC16（低位在前）。
- 校验失败丢弃并记日志；波特率默认 115200（可配，NVS）。

### 5.5 UART 指令表（设计稿，联调以执行板固件为准）

**小车子系统（DEV=0x01，同属执行板）**

| CMD | 名称 | PAYLOAD | 说明 |
|---|---|---|---|
| 0x01 | 持续移动 | dir(1) speed(1) | dir: 0x01 前进 / 0x02 后退；持续到停止帧 |
| 0x02 | 移动指定距离 | dir(1) dist_cm(2) speed(1) | 到距自动停 |
| 0x03 | 持续转动 | dir(1) speed(1) | dir: 0x01 左转 / 0x02 右转 |
| 0x04 | 转动指定角度 | dir(1) angle_deg(2) speed(1) | 到位自动停 |
| 0x05 | 停止 | scope(1) | 0x00 all / 0x01 wheels / 0x02 arm |
| 0x06 | 状态查询 | — | 板→车，车回状态帧（§5.5 状态帧） |

**机械臂子系统（DEV=0x02，同属执行板）**

| CMD | 名称 | PAYLOAD | 说明 |
|---|---|---|---|
| 0x01 | 持续升降爪 | dir(1) speed(1) | dir: 0x01 抬 / 0x02 落 |
| 0x02 | 升降指定距离 | dir(1) dist_cm(2) speed(1) | |
| 0x03 | 持续前后移爪 | dir(1) speed(1) | dir: 0x01 前伸 / 0x02 后缩 |
| 0x04 | 前后移爪指定距离 | dir(1) dist_cm(2) speed(1) | |
| 0x05 | 夹取/松夹 | act(1) | 0x01 夹取 / 0x02 松夹 |
| 0x06 | 状态查询 | — | 板→臂，臂回状态帧 |

**状态帧（执行板 → 板子，响应查询或周期上报）**

| 设备 | CMD | PAYLOAD | 说明 |
|---|---|---|---|
| 小车 | 0x0A | state(1) speed(1) param(2) flag(1) | state: 0 停止 / 1 移动中 / 2 转动中；param: 累计距离 cm 或角度；flag: 故障位（bit0 打滑/堵转等，语义以执行板为准） |
| 机械臂 | 0x0A | state(1) grip(1) param(2) flag(1) | state: 0 空闲 / 1 升降中 / 2 移爪中；grip: 0 松开 / 1 夹住；param: 累计距离 |

**通用约定：**
- 持续类指令（0x01/0x03 与机械臂 0x01/0x03）保持执行，直到收到停止帧（scope 含该设备）。
- 所有持续/指定距离指令都带 speed；speed 档位与距离/角度精度由执行板固件标定（本架构不写死数值）。

### 5.6 BLE GATT（配网 + 兜底控制）

| 特征 | 方向 | 内容 |
|---|---|---|
| `wifi_ssid` / `wifi_pass` | 写 | WiFi 配置 |
| `ai_url` / `ai_key` / `ai_model` | 写 | AI 接口配置（DIRECT 链路用） |
| `cmd` | 写 | 词表 JSON（config / stream / move / stop / arm / ping），兜底控制复用同一词表 |
| `status` | 读/通知 | 板子状态 JSON：`{ "ip": "...", "wifi_ssid": "...", "ws": true/false, "ai_busy": true/false }`；配网后上报，手机据此连 WS |

## 6. 优先级与抢占

| 优先级 | 来源 | 行为 |
|---|---|---|
| 高 | 手动控制（move/stop/arm 直控） | 清空 AI 队列，立即执行 |
| 低 | AI 指令（词表命令 + reason） | 入队逐条执行；手动指令到来即被清空 |
| — | 新 `ai_goal` | 取消当前 AI 任务（清空未执行 AI 指令），开始新目标 |

AI 每次只返回一条动作 → 抢占 = 清空队列，无需复杂中断逻辑。

## 7. 配置项（NVS）

| 键 | 默认 | 说明 |
|---|---|---|
| `wifi_ssid` / `wifi_pass` | 空 | BLE 或词表 config 写入 |
| `ai_url` / `ai_key` / `ai_model` | 空/DeepSeek V4 Flash | AI 接口（DIRECT 链路用） |
| `uart_baud` | 115200 | UART 波特率 |
| 摄像头参数（帧尺寸/质量） | QVGA / JPEG q12 | 可按需调整 |
| 图传帧率 / snapshot 质量 | 依例程 | 可按需调整 |

## 8. 开放项（待硬件/协议确认）

| 项 | 现状 | 备注 |
|---|---|---|
| WS 帧收发 | 需自行实现 | 例程仅 `is_websocket` 标记；用 `httpd_ws_recv_frame` / `httpd_ws_send_frame` 实现文本+二进制 |
| 执行板硬件/接线 | 未定 | 单执行板 + 单 UART；DEV 仅区分板内小车/机械臂子系统，帧格式不变 |
| 执行板指令 ID / speed 档位 | §5.5 为设计稿 | 指令码与速度/距离精度以执行板固件联调为准 |
| 状态帧 flag / 故障语义 | 待定 | 以执行板固件为准 |
| `ai_goal` 词条 | 手机端待补 | 需在 `CommandProto.gd` 增加 `ai_goal`（DIRECT 链路目标下发） |

## 9. 开发顺序建议

1. `config` + `network`：NVS 配置、WiFi 重连（基础设施）
2. `camera` + `web_server`：HTTP MJPEG 图传跑通（验证硬件），再补 WS 文本/帧收发
3. `command` + `uart`：词表解析 + UART 指令表，用串口调试工具验证与执行板闭环
4. `ai_client`：接 AI API，打通 DIRECT 链路（ai_goal → 动作）
5. `ble`：配网 + 兜底控制通道
6. `Ctrl-App`：补 `ai_goal` 词条，RELAY/DIRECT 两条链路对接 WS