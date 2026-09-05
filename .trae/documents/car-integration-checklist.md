# 小车执行板 联调前检查清单

> 目标：设备到货后，按本清单完成**编译验证 → 烧录 → 上电自测 → 上下位机联调**，逐项打勾确认无遗留后，再进入视觉/决策联调。
> 对应设计文档：`car-execution-firmware-design.md`；执行板固件：`例程 p1\闭环 (视觉) 阿克曼小车+机械臂\`。

---

## 0. 背景与进度速览

| 项 | 状态 | 说明 |
| --- | --- | --- |
| 协议层宿主单测（UartFrame / Dispatch） | ✅ 已完成 | 24/24 + 10/10 通过，见 §2 |
| 上下位机帧测试辅助脚本 | ✅ 已就绪 | `tools\uart_test.py`，见 §4 |
| 硬件编译 / 上电自测 | ⏳ 设备到货后执行 | 本章 §3、§5 清单 |
| 视觉 / 决策联调 | ⏳ 设备到货后执行 | 顶层 `vision-control-architecture.md` |

---

## 1. 前置环境确认（编译用）

- [ ] 已安装 Keil MDK，并装有 `Keil.STM32F1xx_DFP` 器件包（工程用 AC5 编译器）。
- [ ] 已安装 Python3。串口联调脚本需要 `pyserial`：`pip install pyserial`。
- [ ] 宿主单测需要 MinGW gcc（默认路径 `C:\mingw64\bin\gcc.exe`，可改脚本内 `$gcc`）。

---

## 2. PC 端宿主单测（不依赖硬件，必须在烧录前跑一遍）

进入工程目录，执行：

```
powershell -ExecutionPolicy Bypass -File "tools\host_test\run_host_tests.ps1"
```

预期输出末尾为 `ALL EJECTED TESTS PASSED`。

- 单测内容：
  - `test_uartframe`（24 项）：CRC-16/MODBUS 标准向量 `0x4B37`、帧组包/解析、坏 CRC 被拒、前置垃圾+断帧重同步、`AA..AA55` 容错、LEN 越界被拒、多帧串接（帧队列）。
  - `test_dispatch`（10 项）：小车/机械臂各指令按 `DEV+CMD` 正确映射到执行器，含 Relay 到位自动停+上报、夹持状态置位等。
- 若失败：协议层逻辑未通过，**禁止**上板联调。

---

## 3. Keil 编译验证（固件可编译性）

- [ ] 用 Keil 打开 `project.uvprojx`。
- [ ] 全量编译（F7），应 `0 Error(s), 0 Warning(s)`。
- [ ] 关注 `.\Objects\` 下生成 `project.hex`（工程已开启生成 HEX）。
- 若出现编译告警：优先检查新增头文件包含路径（`Control / Hardware` 目录）。

---

## 4. 上下位机联调脚本（设备到货前可先 selfcheck）

无需硬件的 CRC 与帧封装自检：

```
python tools\uart_test.py selfcheck
```

预期输出：`selfcheck OK: CRC=0x4B37, frame roundtrip OK, corrupt-frame rejected.`

设备到货后的常用指令（`--port COMx` 替换为实际串口号）：

```
# 小车
python tools\uart_test.py --port COM5 car-move-fwd 200      # 前移
python tools\uart_test.py --port COM5 car-stop-all           # 急停
python tools\uart_test.py --port COM5 car-move-dist 30 200   # 前进 30cm
python tools\uart_test.py --port COM5 car-turn-angle 30 150  # 左转 30°
python tools\uart_test.py --port COM5 car-query              # 查询小车状态（回状态帧）
# 机械臂
python tools\uart_test.py --port COM5 arm-lift 1
python tools\uart_test.py --port COM5 arm-reach 1
python tools\uart_test.py --port COM5 arm-grip 1
python tools\uart_test.py --port COM5 arm-query
```

> 波特率固定 `115200`，与 `Usart1_Init()` 一致；USB-TTL 需要把板端 TX/RX 交叉连接。

---

## 5. 上电自测清单（设备到货后逐项打勾）

> 安全：上电前确认供电电压、电机/舵机接线极性。做运动项自测时**架空小车**（车轮离地）或限速，避免伤人或损坏。

### 5.1 板级指示灯 / 基本外设
- [ ] 上电后 `LED1` 以 1s 周期闪烁（`main.c`：每 5ms 节拍、每 1s `LED1_Turn()`）→ 确认 5ms 调度正常。
- [ ] 若板载 OLED，显示初始化正常（可选；本工程以串口与 LED 为主要观测口）。

### 5.2 串口接收链路（帧状态机）
- [ ] USB-TTL 连接 `USART1`，任一串口助手以 115200 打开。
- [ ] 发送一帧正常指令（可用 `uart_test.py car-query`），观察小车是否有动作或返回。
- [ ] 连续快速发送多帧（≥8 帧）→ 验证帧队列不丢帧（对应 §2 的"多帧串接"测试）。

### 5.3 小车（阿克曼）子系统
| 指令 | 期望现象 |
| --- | --- |
| `car-move-fwd/back` | 后轮按要求方向转动 |
| `car-stop-all` | 车轮停止、机械臂停止，状态复位 |
| `car-move-dist 30 200` | 前进 30cm 后自动停下并上报"完成"（Relay 距离到位） |
| `car-turn-angle 30 150` | 前轮舵机偏转并整车转 30° 后停下（Relay 角度到位） |
| `car-query` | 回发状态帧，`car_state/car_speed` 正确 |

- [ ] 阿克曼舵机回中正常（上电 `Arc_Servo_Reset`）。
- [ ] 编码器采样方向与 `Odom` 累加方向一致（前进为正）。

### 5.4 机械臂子系统
- [ ] 上电机械臂复位到初始位（Servo2/3/4）。
- [ ] `arm-lift` / `arm-reach` 持续动作、`arm-grip 1/0` 夹爪开合。
- [ ] `arm-query` 回发 `arm_state / arm_grip` 正确。
- [ ] 指定距离升降/移爪到位后状态自动复位。

### 5.5 周期状态上报
- [ ] 观察每 100ms 是否稳定回发小车+机械臂两条状态帧（`DEV=0x01/0x02, CMD=0x0A`），CRC 合法。

---

## 6. 联调遗留问题记录

| 日期 | 问题描述 | 根因/定位 | 解决状态 |
| --- | --- | --- | --- |
|  |  |  |  |
|  |  |  |  |

---

## 附：关键文件定位

| 模块 | 路径 |
| --- | --- |
| 帧协议 | `Control\UartFrame.c/.h` |
| 指令分发 | `Control\Dispatch.c/.h` |
| 状态管理 | `Control\Status.c/.h` |
| 阿克曼驱动 | `Control\AckermannDrive.c/.h`（+`Odom/Relay`） |
| 机械臂动作 | `Control\RobotArmAct.c/.h` |
| 串口驱动 | `Hardware\Usart.c/.h`（115200 + RXNE 中断） |
| 主调度 | `User\main.c`（5ms 节拍） |
| 宿主单测 | `tools\host_test\`（+`run_host_tests.ps1`） |
| 串口辅助脚本 | `tools\uart_test.py` |