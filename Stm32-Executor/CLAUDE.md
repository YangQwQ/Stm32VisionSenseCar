# CLAUDE.md（小车执行板固件）

> 本项目是「视觉控制小车」协作工程里的 **执行板**（STM32F103C8T6，Keil MDK + 标准外设库）。
> 它带 **阿克曼底盘**（后轮驱动 + 前轮舵机转向）和 **机械臂**，经 **USART1** 接上游「视觉/控制大脑板」，只做"执行"：
> 收到大脑板下发的动作指令帧 → 驱动底盘/机械臂 → 回传状态帧。**不解析手机词表、不调 AI**。
>
> 整个协作体系的角色划分见仓库根 [`../CLAUDE.md`](../CLAUDE.md)（本仓库总纲）；详细架构与协议见 `../vision-control-architecture.md`（尚未并入本仓库，缺失期间以本文件「通信协议速查」+ 仓库根总纲为准）。

## 一句话架构

大脑板（`../Stm32-Vision`，ESP32-S3）按 `AA 55 LEN DEV CMD [PAYLOAD] CRC16` 组帧，
经 UART 发给本板 → `Control/` 层译码执行 → 周期回传 `0x0A` 状态帧。**协议（§5.4 帧 / §5.5 指令表与状态字段）以本执行板为最终实现基准**，改指令码/状态语义须与大脑板、架构文档同步。

## 代码结构

源自「PS2 遥控阿克曼小车+机械臂」示例程改造：把 PS2 直控替换为 UART 指令输入，业务逻辑集中在新增的 `Control/` 层。

| 目录 | 内容 | 改动面 |
|---|---|---|
| `Control/` | **协议+执行层**（本仓库核心）：UartFrame(帧/CRC)、Dispatch(译码分发)、AckermannDrive(阿克曼运动)、Odom(里程计)、Relay(到位判定)、RobotArmAct(机械臂动作)、Status(状态帧) | 日常改动集中地 |
| `Hardware/` | 板级驱动：Usart(USART1+环形缓冲)、Board_Timer(TIM2 5ms)、LED、Vehicle_Chassis(底盘动作+阿克曼舵机)、RobotArm(机械臂 3 舵机动作)、ps2(PS2 手柄，自测用) | 少动 |
| `NeZha/` | 哪吒扩展板软 I2C 驱动：4 电机 PWM(0~1000)、4 编码器(20ms 增量)、4 舵机 PWM、灯带 | 底层 |
| `User/` | `main.c`（5ms 主调度）+ 中断/配置头 | main 见下节 |
| `Library/` `Start/` `System/` | STM32F10x 标准外设库 + CMSIS/启动 + Delay/sys | 不动 |
| `RTE/` `Objects/` `Listings/` `DebugConfig/` | Keil 生成物（已 gitignore） | 不动 |
| `stlink-1.8.0-win32/` | 内嵌 st-flash 烧录工具（随仓库提交，供命令行烧录） | 不动 |
| `project.uvprojx` | Keil 工程唯一入口（AC5，已含全部源文件） | 加文件时同步 |

## Control 层模块职责与数据流

```
main(5ms 节拍)
  ├─ UartFrame ← Usart1(RXNE 中断，环形缓冲)
  ├─ Dispatch ──┬─→ AckermannDrive ─→ Vehicle_Chassis / NeZha(电机) + Arc_Servo(转向舵机)
  │             ├─→ RobotArmAct ───→ RobotArm ─→ NeZha(Servo2/3/4)
  │             ├─→ Relay(到位判定) ─→ Odom ─→ NeZha(编码器)
  │             └─→ Status(状态帧) ─→ UartFrame_Send
```

| 模块 | 职责 |
|---|---|
| `UartFrame` | §5.4 帧状态机 + CRC-16/MODBUS 校验（错帧丢弃 + 计数）；`UartFrame_Send` 编码发送。`UART_FRAME_MAX_LEN=32` |
| `Dispatch` | 按 DEV+CMD 分发；新指令**覆盖**旧的持续指令（无需显式 stop 即抢占）；stop(scope) 分 wheels/arm/all |
| `AckermannDrive` | 直行=舵机回中+四轮驱动；持续转向=打满角（左 120/右 190 PWM）；`SteerTo` 按角度线性映射舵机 PWM；**内嵌速度闭环**（运动状态机 + 增量 PID，见下） |
| `Odom` | 20ms 采样 4 路编码器均值 → 累计距离(mm) 与 yaw(0.1°)；yaw 用阿克曼简化模型 `Δψ≈Δs/轴距·tan(舵角)` |
| `Relay` | 定距/定角指令的目标判定：记录起点，达 `|Δ|≥目标` 即停用并回报完成 |
| `RobotArmAct` | 机械臂步进状态机（持续/定距升降、移爪、夹爪）；每 25ms `_Step` 走一步，定距按步进计数判到位 |
| `Status` | 合成小车/机械臂 `0x0A` 状态帧；**数据未变化则不上报**（去重） |

机械臂 3 个舵机分工（`Hardware/RobotArm.c`，NeZha Servo2/3/4）：**Servo2=前后移爪、Servo3=夹爪开合、Servo4=抬/落**。

## 主循环调度（`User/main.c`，TIM2 5ms 时基）

| 周期 | 任务 |
|---|---|
| 5ms | `UartFrame_Feed` 收包 + `Dispatch_Process` 译码分发 |
| 20ms | `Odom_Sample` 编码器采样 → 累计；`AckermannDrive_Regulate` 速度闭环（实测轮速→PID→刷新 PWM）；`Dispatch_Periodic` 到位判定 + 状态刷新（到达即停并置完成标志） |
| 25ms | `RobotArmAct_Step` 机械臂步进 |
| 100ms | `Status_SendCar` / `Status_SendArm` 周期状态上报（数据无变化自动跳过） |
| 1s | `LED1_Turn` 心跳指示灯（确认 5ms 调度活着） |

## 通信协议速查

权威定义见架构文档 §5.4/§5.5；此处为 `Control/` 内实际实现摘要，**改这层时以两侧同步为前提**。

**帧格式**：`AA 55 LEN DEV CMD [PAYLOAD] CRC16(2B 低在前)`；`LEN`=DEV+CMD+PAYLOAD 字节数，上限 32。
CRC-16/MODBUS（反射 poly `0xA001`，初值 `0xFFFF`，低位在前）——**与大脑板联调已定为该实现，勿擅改**。

`DEV`：`0x01` 小车　`0x02` 机械臂　｜　`CMD 0x0A` = 状态帧，`0x06` = 查询（即时回状态帧）

| 子系统 | CMD | 含义 | PAYLOAD |
|---|---|---|---|
| 小车 0x01 | `0x01` | 持续移动 | dir(1 前/2 后) speed |
| | `0x02` | 直行指定距离 | dir dist_cm(u16 低前) speed |
| | `0x03` | 持续转向 | dir(1 左/2 右) speed |
| | `0x04` | 转向指定角度 | dir angle_deg(u16 低前) speed |
| | `0x05` | 停止 | scope(0 全 / 1 轮 / 2 臂) |
| 机械臂 0x02 | `0x01`/`0x02` | 持续/定距升降 | dir(1 抬/2 落) [dist_cm(u16)] speed |
| | `0x03`/`0x04` | 持续/定距移爪 | dir(1 前伸/2 后缩) [dist_cm(u16)] speed |
| | `0x05` | 夹爪动作 | act(1 夹 / 2 松) |

**状态帧 `0x0A` PAYLOAD=5B**：
- 小车：`state(0停/1移/2转) speed param(2B低前=累计距离cm) flag`
- 机械臂：`state(0空闲/1升降/2移爪) grip(0松/1夹) 0 0 flag`

flag 位：`car_flag` bit0=打滑/堵转、bit1=定距/定角完成；`arm_flag` bit0=定距完成、bit1=夹爪动作完成、bit2=是否夹到东西（占位待标定）。
完成标志（`g_status.car_done/arm_done/arm_grip_done`）：新指令清 0，到达目标后置 1，随状态帧上报。

## 值域与换算（易踩坑）

- **速度两域不一致（d24cd33 已修）**：帧内 speed 是**单字节 0~255**，而驱动层（AckermannDrive / Vehicle_Chassis / NeZha 电机）按 **0~1000** 吃 PWM。`Dispatch::CarPwm(sp)=sp*1000/255` 负责归一到驱动域。机械臂速度在 `RobotArmAct::ArmStepPerTick` 里按 `speed/300+1` 折算每拍 1~5 步。**改任一处都要保持两域换算一致**。
- **小车轮速已闭环**：`speed`(驱动域 0~1000) 现在是**目标轮速指令域**，不再直接是 PWM。命令入口（`AckermannDrive_Go`）先按目标 PWM 起步保证响应，此后每个 20ms `AckermannDrive_Regulate` 用增量 PID 修正实际 PWM。反馈把四轮均值编码器增量经 `Odom_SpeedUnits()` 按 `SPD_FULLSCALE_ENC` 换算回同一 0~1000 域比对。`SPD_LOOP_EN=0` 可整体退回开环直通做 A/B。PID 只正向驱动、**无主动刹车**（降到低速靠惯性滑行，正常）。
- 距离：帧内 `dist_cm`(cm) → Relay 目标 `*10` 后按内部 mm 比对里程计；状态帧 param 用整厘米（`Odom_GetDistCm`）。
- 角度：帧内 `angle_deg` 直接打舵机（舵机 PWM 相对中值 150 线性偏移）并按 `*10`(0.1°) 交给 Relay 判到位。
- 转向舵机量程：中 150 / 左 120 / 右 190（PWM）；`AckermannDrive_SteerTo` 在量程内 CLAMP。

## 标定常量（占位经验值，真机联调需实测回填）

以下常量都是"联调标定项"，改数值**只动头文件顶部宏**，别动逻辑：

| 常量 | 位置 | 含义 |
|---|---|---|
| `ENC_CNT_PER_CM` | `Control/Odom.h` | 每厘米编码器计数（轮径/减速比/PPR 综合），当前 100 |
| `SPD_FULLSCALE_ENC` | `Control/Odom.h` | 速度闭环反馈比例：指令 1000 对应的四轮均值编码器增量/20ms。标定 = `ENC_CNT_PER_CM` × 满 PWM 轮速(cm/s) × 0.02，当前 150 |
| `SPD_LOOP_EN` / `SPD_KP` / `SPD_KI` / `SPD_KD` / `SPD_INTG_MAX` | `Control/AckermannDrive.h` | 速度闭环开关（0=开环直通）与增量 PID 增益/积分限幅。默认 KP=1.0 KI=0.05 KD=0，联调按实车调 |
| `WHEELBASE_CM` | `Control/AckermannDrive.h` | 轴距，yaw 模型用，当前 15 |
| `STEER_MAX_DEG` | `Control/AckermannDrive.h` | 满打对应最大转向角，当前 30 |
| 转向角→PWM 线性映射 | `Control/AckermannDrive.c` `SteerTo` | 满量程 70 PWM ↔ 60° 的经验映射 |
| `ARM_CNT_PER_CM` / `ARM_GRIP_STEPS` | `Control/RobotArmAct.h` | 机械臂步进/cm（无编码器，纯步进估算） |
| `Odom_ErrFlag()` | `Control/Odom.c` | 打滑/堵转检测占位，现恒 0 |
| `ARM_FLAG_HAS_LOAD` 判定 | 夹爪电流/到位 | 未实现，待标定 |

> 真机运动自测请**架空车轮**或限速，避免伤人/损坏。

## 构建与烧录

- **入口**：Keil MDK 打开 `project.uvprojx`（AC5 + `Keil.STM32F1xx_DFP`）。器件 `STM32F103C8`（64KB Flash/20KB RAM）。
- **编译默认交给用户在 Keil 里做**（无命令行工具链，不主动跑）。
- 全量编译（F7）后产物在 `.\Objects\project.hex`（工程已开启生成 HEX）。
- 命令行烧录（st-link，工具已随仓库 `stlink-1.8.0-win32/`）：
  ```
  .\stlink-1.8.0-win32\bin\st-flash.exe --reset --format ihex write .\Objects\project.hex
  ```

## PS2 直控自测模式

把 `User/main.c` 顶部 `PS2_SELFTEST` 改为 `1`，Keil 全量重编即可切换（默认 `0` 走 UART）。两种模式共用底层初始化与 Odom/状态帧上报，仅"指令来源"不同：

- 小车：十字键 上/下=直行前/后，左/右=持续转向；全松开即停。
- 机械臂：L1/L2=抬/落（Servo4），R1/R2=前伸/后缩移爪（Servo2），按住运行松开停；CROSS/SQUARE=夹/松（Servo3，按一次触发一次）。
- 自测直驱速度 `PS2_CAR_PWM`（驱动域 0~1000 目标轮速）与臂速 `PS2_ARM_SPD`（0~255）就在该宏下方。

**标定参数用法**：自测时 PS2 每 50ms 轮询（`Time%10`），Odom 仍 20ms 采样、0x0A 状态帧仍 100ms 上报——接 USB-TTL 到 USART1 即可边摇手柄边读里程计帧（param=累计距离 cm），据此校准 `ENC_CNT_PER_CM` 等常量。注意 ps2 软时序较慢，轮询刻意放 50ms 级，别挪进 5ms。

**速度闭环联调顺序建议**：① 先设 `AckermannDrive.h` 的 `SPD_LOOP_EN=0`（开环直通）确认电机方向/接线无误；② 满 PWM 跑一段，按 param=距离算轮速填 `SPD_FULLSCALE_ENC`；③ 再设 `SPD_LOOP_EN=1` 调 `SPD_KP/KI`（从 KP≈1.0、KI 很小起步，KP 过大/轮子悬空会震荡尖叫）。

## 约定与已知坑

- 与用户交流用中文；注释精简、**不在注释里写死魔法数值**（数值都收敛到各头文件顶部宏）。
- **本板只认帧指令，不经手词表 JSON**；词表本体归手机 `../Mobile-RemoteCtrl`（本仓库内即原 `Ctrl-App`），`词表→UART 帧`的翻译在大脑板 `../Stm32-Vision/uart` 侧。
- **改动协议（帧格式/CRC/DEV/CMD/状态字段/flag）必须三侧同步**：本 `Control/` 内常量 ↔ `../Stm32-Vision/uart`（+架构文档 §5.4/§5.5）。状态/flag 语义以本板为基准，改完知会大脑板侧。
- 2026-09 清理：删除示例程遗留且当前无调用的 `OLED.c/.h`、`OLED_Font.h`、`compute_pid.c/.h`，以及 `Hardware/` 下与 `NeZha/` 重复的 `NeZha_I2C.h`（实际生效的是 `NeZha/NeZha_I2C.h`，I2C 接口统一在 `NeZha/`）。
- **`ps2.c/.h`（PS2 遥控驱动）接入自测模式**：`User/main.c` 顶部 `PS2_SELFTEST` 宏，`0`=UART 指令主链路（默认），`1`=PS2 手柄直驱自测（不接大脑板时确认电机方向/舵机/编码器参数用）。接线 CS=PA4 / SCK=PA5 / DI=PA6 / DO=PA7（见 `Hardware/ps2.c` `PS2_GPIO_Init`，板子不同改那里）。改宏后需在 Keil 全量重编。
- 本仓库刚做过目录扁平化（源码从 `car_firmware/` 子目录上移到仓库根），路径改动都进了 git rename 记录，查找历史文件用 `git log --follow`。

## 关联项目

| 角色 | 位置 |
|---|---|
| 大脑板（组帧下发本板、解析状态帧） | `../Stm32-Vision/`（读其 `CLAUDE.md`） |
| 手机 App（指挥，词表源） | `../Mobile-RemoteCtrl/`（本仓库内即原 `Ctrl-App`） |
| 架构 + 完整协议 | `../vision-control-architecture.md`（尚未并入本仓库） |
| 仓库根总纲 | `../CLAUDE.md` |
