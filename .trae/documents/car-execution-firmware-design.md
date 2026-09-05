# 小车执行板固件设计方案（STM32 阿克曼小车 + 机械臂）

> 依据总设计文档 `vision-control-architecture.md`（下称《架构》）§5.4/§5.5 的 UART 帧协议与指令表，设计"小车执行板"固件。
> 范围：**STM32F103C8T6 执行板固件**（架构中"执行"角色），接收大脑板（ESP32-S3）经 UART 下发的动作指令，驱动阿克曼底盘 + 机械臂，并回传状态帧。
> 本设计在《例程 p1\开环 (普通电机)\3. PS2遥控阿克曼小车+机械臂.zip》基础上改造，将"PS2 遥控输入"替换为"UART 指令输入"。

---

## 一、目标与范围

- 在 STM32F103C8T6（Keil MDK + 标准外设库工程）上实现《架构》§5.4 UART 帧协议接收端、§5.5 指令表执行端、状态帧上报端。
- 运动构型：**阿克曼**（后轮驱动 + 前轮阿克曼舵机转向，Servo1 打角）。
- 距离/角度到位停止：**编码器闭环**（NeZha 编码器采样 + 里程计推算）。
- 机械臂：复用 RobotArm（Servo2 前伸/后缩、Servo3 夹爪、Servo4 抬/落）。
- 明确各模块文件、接口、数据流、状态机，供后续直接编码实现。

### 本项目不做（属其他模块）
- 大脑板固件（ESP32-S3：camera / ai_client / command / web_server / ble / config / network）。
- 手机 App、AI 决策链路（《架构》§3 REC穿/DIRECT 是在大脑板/手机侧）。

---

## 二、现状分析（已核对）

| 参照 | 结论 |
|---|---|
| `vision-control-architecture.md` | §5.4 帧格式 `AA 55 LEN DEV CMD [PAYLOAD] CRC16`；§5.5 小车/机械臂指令表与状态帧；§5.3 词表→UART 映射；波特率默认 115200 |
| 例程 #3（阿克曼+机械臂） | Keil/SPL 工程，含 `Hardware/`、`NeZha/`、`Library/`、`Start/`、`System/`、`User/`；USART1(PA9/PA10) 现例程 baud=9600；Board_Timer=TIM2 5ms；「User/main.c」用 PS2 直控 |
| `NeZha.h / NeZha.c` | I2C(软) 总线操作：电机 PWM(0-1000)、4 路编码器读（20ms 增量语义）、4 路舵机 PWM(50-250)；Servo1=阿克曼转向舵机 |
| `Vehicle_Chassis.c` | 阿克曼底盘基础动作：Forward/Backward/TurnLeft/TurnRight（四轮）、akerman 舵机 Servo1（arc_servo {120,150,190}） |
| `RobotArm.c` | 机械臂 Servo2/3/4：前伸/后缩/抬/落/夹/松（PWM 限幅步进） |
| `Board_Timer.c` | TIM2 5ms 时基，例程在 5ms/10ms/25ms/250ms 分频做调度 |
| 原理图 | STM32F103C8T6 核心板；执行板与大脑板接线按 USART1（PA9→大脑板 RX，PA10→大脑板 TX）|

**目标工程**：`e:\桌面\car\例程 p1\闭环 (视觉) 阿克曼小车+机械臂\`（复制 #3 例程改造生成）。保留 `Hardware/`、`NeZha/`、`Library/`、`Start/`、`System/`，替换 `User/main.c`，新增 `Control/` 协议与执行层。

---

## 三、模块划分与分工（核心）

新增 **`Control/` 协议执行层**（本设计的重点），复用底层 `Hardware/` 与 `NeZha/`。

| 模块 | 文件 | 职责 | 依赖 |
|---|---|---|---|
| 帧收发 | `Control/UartFrame.c/.h` | 接收：USART1 中断取字节→§5.4 定长/变长帧状态机、CRC16 校验（错帧丢弃加分帧计数）；发送：编码帧写入发送缓冲 | `Hardware/Usart.c`、`Library/` |
| 指令译码分发 | `Control/Dispatch.c/.h` | 按 `DEV`(0x01 小车 / 0x02 机械臂)`CMD` 分发到对应执行器；维护"持续动作"状态；SV**stop(scope)** 语义；新指令覆盖旧持续指令 | `UartFrame`、`AckermannDrive`、`RobotArmAct` |
| 阿克曼运动控制 | `Control/AckermannDrive.c/.h` | 直行/转向/回中的映射：`dir,speed → NeZha_Motor_SetPwm + Arc_Servo(舵机打角)`；回中；安全限幅 | `NeZha.h`、`Vehicle_Chassis.h`、`Vehicle_Chassis.c` |
| 里程计（编码器闭环） | `Control/Odom.c/.h` | 按 20ms 采样 4 路 NeZha 编码器，累加换算距离/车身 yaw；提供 `GetDist()/GetYawDiff()`；归零 | `NeZha.h` |
| 到位判定器 | `Control/Relay.c/.h`（Target） | 对"指定距离/角度"指令管理目标值，周期性比对里程计→到位自动转 stop | `Odom`、`AckermannDrive` |
| 机械臂动作 | `Control/RobotArmAct.c/.h` | 在 RobotArm 基础上封装持续/指定距离升降、移爪、夹取/松开；指定距离用舵机步进计数+标定换算 | `RobotArm.h/.c` |
| 状态上报 | `Control/Status.c/.h` | 构造 §5.5 状态帧（小车/机械臂 0x0A state/speed/param/flag），应答 CMD 0x06 或周期上报 | `AckermannDrive`、`RobotArmAct`、`Odom` |
| 主调度（入口） | `User/main.c` | 初始化；主循环 5ms 时基：收包处理、运动/臂刷新、里程计采样、到位判定、周期状态上报 | 全部 |
| 底层改造 | `Hardware/Usart.c/.h` | 波特率改 115200；新增"1 字节队列 + RXNE 中断回调"接口，供 UartFrame 拉取 | `Library/stm32f10x_usart` |

**模块依赖图**
```
main(5ms 调度)
  ├─ UartFrame ← Usart1(RXNE 中断)
  ├─ Dispatch ─┬→ AckermannDrive ─→ NeZha(电机) / Servo(舵机打角)
  │            ├→ RobotArmAct ─→ NeZha(舵机2/3/4)
  │            ├→ Relay(到位判定) ─→ Odom ─→ NeZha(编码器)
  │            └→ Status(状态帧) ─→ UartFrame(发送)
```

---

## 四、通信协议（执行板侧实现要点）

### 4.1 接收帧格式（来自大脑板）
```
AA 55  LEN  DEV  CMD  [PAYLOAD]  CRC16(2B, 低在前)
```
- `LEN` = DEV+CMD+PAYLOAD 字节数。
- `CRC16`：对 `LEN..PAYLOAD` 计算，**本执行板采用 CRC-16/MODBUS（poly 0x8005 反相，初值 0xFFFF）**，低位在前。→ 记入《架构》§5.4 的"开放项"：与大脑板确认，本侧为已定实现。
- 校验失败：丢弃、`err_frame_cnt++`（可经状态帧上报）。

**接收状态机**（UartFrame）：
```
IDLE --取到 AA--> SAW_AA --取到 55--> LEN(取LEN) --> DEV(取) --> CMD(取) --> PAYLOAD(依LEN收满) --> CRC_L --> CRC_H --> 校验 --> 交Dispatch
任何字节不符/超长 → 回 IDLE（丢弃）
```

### 4.2 发送帧（状态帧，执行板→大脑板）
```
AA 55  LEN  DEV  0x0A  state speed param(2B,低在前) flag   CRC16
// 小车:     DEV=0x01  state(0停/1移/2转)  speed  param=累计距离cm或角度  flag(bit0 打滑/堵转)
// 机械臂:   DEV=0x02  state(0空闲/1升降/2移爪)  grip(0松/1夹)  param=累计距离  flag
```

---

## 五、阿克曼运动控制映射

（速度 `speed` 按例程 0~1000 → `NeZha_Motor_SetPwm`；驱动前后由 `dir` 决定）

| 逻辑 | 指令 | 底盘动作 | 到位方式 |
|---|---|---|---|
| 直行 | 0x01(dir,speed) | **舵机回中** + Forward/Backward(dir) | 持续至 `stop` |
| 直行指定距离 | 0x02(dir,dist_cm,speed) | 同上；Odom 累计达 `dist_cm` | 编码器闭环自动停 |
| 转向/持续转动 | 0x03(dir,speed) | 舵机打角左/右极限(arc_servo.left/right) + 驱动 4 轮 | 持续至 `stop` |
| 转向指定角度 | 0x04(dir,angle_deg,speed) | 舵机按 `angle_deg` 打角 + 驱动；Odom 推算车身 yaw | 编码器闭环自动停 |
| 停止 | 0x05(scope) | scope: all/1=wheels/0=全停 | 立即 |

**阿克曼转向实现（决策）**：
- `0x03 持续转动`：`left→Arc_ServoPwm_Set(120)`、`right→Set(190)`，同时以 speed 前进/驱动四轮，直到收到 stop。
- `0x04 转动指定角度`：`direction 决定打角方向`，`angle_deg 经标定系数→目标舵机 PWM`（相对中值 150 偏移）；同时驱动。到停止由里程计（yaw≈`∫(驱动距离/轴距·tan(舵角))`）判定。
- 舵机无位置反馈，`angle_deg` 的物理对应依赖标定常量 `SERVO_CNT_PER_DEG`、`STEER_YAW_SCALE`（联调标定项）。
- 直行与转向混用：`throttle+steering 同时非零` 时大脑板会组合下发，本板按"先直行后转向"的两帧执行（验收时确认大脑板不发混合帧）。

**速度档**：`speed∈[0,1000]`（对应例程 `PS2_LSPEED=1000`）。执行板不定义档位，直接透传。

---

## 六、里程计/编码器闭环（Odom + Relay）

- 采样周期 20ms（NeZha 编码器为 20ms 增量语义），在 main 5ms tick 的每第 4 拍读取 `NeZha_Encoder1..4_Read()`。
- `ENC_CNT_PER_CM`：每厘米编码器计数（轮径/减速比/PPR 综合），**标定待实测**，默认给经验值占位。
- `GetDist()`：`Σ 驱动轮增量/ENC_CNT_PER_CM`；`GetYawDiff()`：`Σ (Δdist/轴距·tan(舵角))`（阿克曼简化模型），用于 0x04 到位。
- `Relay`（到位判定器）持有目标 {type, value}；每采样拍比较，`|cur|≥|target|` → 调 `AckermannDrive_Stop(scope)` 并回传"已完成"状态。

---

## 七、机械臂动作（RobotArmAct）

| 指令 | 动作 | 实现 |
|---|---|---|
| 0x01 持续升降(dir,speed) | 抬/落（Servo4） | `RobotArm_RaiseHand/DropHand` 步进，持续至 stop |
| 0x02 升降指定距离(dir,dist_cm,speed) | 抬/落指定距离 | PWM 步进计数，标定 `CNT_PER_CM` 换算，到位停 |
| 0x03 持续前后移爪(dir,speed) | 前伸/后缩（Servo2） | `Stretch/ShrinkHand` 步进 |
| 0x04 前后移爪指定距离(dir,dist_cm,speed) | 前后指定距离 | PWM 步进计数到位停 |
| 0x05 夹取/松开(act) | 夹/松（Servo3） | `ShakeHand/LetHand` 单次动作 |
| 0x01..0x04 的 stop(scope=arm) | 停 | 停止步进 |

注：机械臂无编码器，`dist_cm` 用"步进计数×标定"估算，精度依赖联调标定（同《架构》§8 说明）。

---

## 八、主循环调度（main）与状态机

复刻例程 #3 的 5ms 时基（Board_Timer），分频调度：

| 周期 | 任务 |
|---|---|
| 5ms | 收包处理（UartFrame 尽量新事件）；Dispatch 译码触发执行 |
| 20ms | Odom 采样 4 路编码器 → 累计；Relay 到位判定 |
| 25ms | 刷新 4 轮电机 PWM（AckermannDrive 输出）；机械臂步进刷新 |
| 50ms | 状态帧周期上报（供大脑板查询）|
| 5ms | UartFrame 发送缓冲刷新（TXE） |

**主状态**：`IDLE`（无持续指令）→ `DRIVE_FWD` / `DRIVE_TURN` / `ARM_MOVE` / `MIX_CAR_ARM`；任何新指令覆盖旧持续指令（等价于手动抢占）。stop 使回 IDLE。

**初始化序列**（main）：
```c
__disable_irq(); NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
Usart1_Init();            // 115200 + RXNE 中断
NeZha_Init(); NeZha_Motor_Init();
NeZha_Encoder1..4_Init();
RobotArm_Init(); Vehicle_Chassis_Init();   // 含 Arc_Servo 回中
Board_Timer_Init();
UartFrame_Reset(); Odom_Reset();
__enable_irq();
loop: 按上表调度
```

---

## 九、复用/改造文件清单

| 文件 | 处理 |
|---|---|
| `User/main.c` | 重写：初始化 + 主循环调度（替代 PS2 逻辑）|
| `Hardware/Usart.c/.h` | 改造：波特率 115200；新增字节队列 + RXNE 中断回调入口 |
| `Hardware/Usart.c` 原 PS2 | 移除 PS2 相关分支（ps2.c 可保留不参与）|
| `Hardware/Vehicle_Chassis.c/.h`、`NeZha/*`、`Hardware/RobotArm.c/.h`、`Hardware/Board_Timer.c/.h`、`System/Delay`、`Library/*`、`Start/*` | 原样复用 |
| 新增 `Control/UartFrame.c/.h` | 帧收发 + CRC16 |
| 新增 `Control/Dispatch.c/.h` | 译码分发 + 持续动作状态 |
| 新增 `Control/AckermannDrive.c/.h` | 阿克曼运动 |
| 新增 `Control/Odom.c/.h` | 编码器里程计 |
| 新增 `Control/Relay.c/.h` | 到位判定（距离/角度）|
| 新增 `Control/RobotArmAct.c/.h` | 机械臂动作封装 |
| 新增 `Control/Status.c/.h` | 状态帧构造 |
| `project.uvprojx` | 将 Control 全部 .c 加入工程编译 |

---

## 十、假设与决策（明确记录）

1. **范围**：仅 STM32 执行板固件（用户确认），不含 ESP32-S3 大脑板与手机端。
2. **构型**：阿克曼（用户确认）。0x01/0x03 直行/转向；0x02/0x04 指定距离/角度编码器闭环。
3. **编码器闭环**（用户确认）：距离用驱动轮编码器（可靠）；阿克曼车身转角用"里程计 yaw 简化模型"（需标定），`angle_deg` 同时控制舵机打角，若标定困难可退化为"舵机到位即视为完成"。
4. **CRC16**：采用 CRC-16/MODBUS(poly 0x8005, init 0xFFFF)，低位在前；与大脑板联调时确认。
5. **波特率**：固定 115200（宏 `UART_BAUD` 可改），与《架构》§5.4 默认一致。
6. **速度**：speed 0~1000 透传，执行板不定义档位（与 §5.5"速度档位以执行板固件标定为准"一致，速度换算留标定）。
7. **机械臂 dist_cm**：舵机步进计数估算，无编码器，精度联调标定。
8. §5.5 指令码以本执行板为实现基准，联调时与大脑板核对（《架构》§8 "指令 ID / speed 档位 / flag 语义"为联调标定项）。

---

## 十一、验证步骤

**A. 独立单元验证（连不上大脑板时）**
1. Keil MDK 编译，#3 工程改造后在 `Objects/project.hex` 出固件，下载到 STM32F103C8T6。
2. 用串口调试工具（USB-TTL）接 USART1：手动发 §5.4 帧：
   - `AA 55 08 01 01 01 64 00 00 <CRC>` → 小车前进
   - `AA 55 08 01 05 00 xx xx xx <CRC>`（.无关字段填 0）→ 停止
   - 发 `AA 55 03 01 06 00 00 <CRC>`（状态查询 LEN=3: DEV+CMD+CLD…严格按 LEN 语义：`AA 55 LEN=2?`）→ 收到状态帧 0x0A，校验打印 hex。
   - 发错误 CRC → 丢弃且 err 计数 +1（可从状态帧观察到）。
3. 距离/角度闭环：设 `dist_cm` 观察轮胎转 `ENC_CNT_PER_CM` 后自动停；`angle_deg` 观察 yaw 到位停。
4. 机械臂 0x05 夹/松、0x01..0x04 步进动作目测验证。

**B. 联调验证（接入大脑板 ESP32-S3）**
5. 大脑板 WS 收到手机 `move/arm` 词表 → 转 UART 帧 → 执行板动作，确认一一对应（对照《架构》§5.3 映射表）。
6. 周期性状态帧：大脑板 `/status` 或 BLE `status` 读到车/臂 state、speed、param、flag。
7. 往返时延：指令→到位上报时间满足场景要求。

**C. 交付物**
- 可编译完整 Keil 工程（含全部新增/复用源文件）。
- Code Review 核对 CRC16、帧状态机边界（超长帧、断帧重启）。

---

## 十二、开发顺序

1. **UartFrame**（帧收发 + CRC16）——写一个 PC 端 Python 脚本发帧验证 CRC 与状态机。
2. **Usart 改造 + main 骨架**（115200、RXNE 中断、5ms 调度）。
3. **Dispatch + AckermannDrive**：先把 0x01/0x03/0x05 跑通（不含闭环）。
4. **Odom + Relay**：编码器采样、0x02 距离闭环、0x04 角度闭环。
5. **RobotArmAct**：机械臂指令。
6. **Status**：状态帧上报。
7. 联调标定（dist_cm / angle_deg / flag 语义）并回填标定常量。