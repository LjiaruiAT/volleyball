# 🔥 exact_fire — 全向轮底盘 + RobStride 弹射 + VESC 舵轮，竞赛机器人主控

> **STM32F407VGTx @ 168MHz · FreeRTOS 10.x + CMSIS-RTOS2 · CAN 双总线 (CAN1 + CAN2) · 自研力控底盘框架 · 五次多项式轨迹规划 · 模糊 PID**

---

## 📋 功能概览

本项目是一个完整的**三全向轮机器人竞赛主控固件**，集成四大功能模块：底盘运动控制、弹射机构控制、遥控通信、IMU 姿态感知。

| 子系统 | 硬件 | 通信 | 控制方式 |
|--------|------|------|----------|
| 底盘 (3 全向轮) | VESC v6.4 ×3 | CAN2 (1 Mbps) | PID 速度闭环 |
| 弹射机构 | RobStride 电机 ×2 | CAN1 (1 Mbps) | 位置 + 力混合 (MIT 阻抗模式) |
| 扬臂 | DJI M3508 | CAN1 | 位置 + 速度串级 PID |
| 遥控 | 自定义 UART 协议 | UART5 (DMA + 空闲中断) | 事件驱动回调 |
| 裁判系统交互 | 自定义 UART 协议 | UART4 (DMA) | DMA 半双工 |
| USB 雷达 | USB CDC (VCP) | USB FS | 虚拟串口透传 |

---

## 🏗️ 架构分层

```
┌──────────────────────────────────────────────────────────────────┐
│                Task_Init.c（业务逻辑 + 状态机）                     │
│   Remote()  ·  Hit_Task()  ·  Back_Task()  ·  Remote_Analysis() │
├──────────────────────────────────────────────────────────────────┤
│  AutoPilot (五次多项式)  │  ForceChassis (力控底盘)               │
│  step/CubicParam (三次多项式)  │  PID + FuzzyPID + PID2          │
├──────────────────────────────────────────────────────────────────┤
│  motor · VESC · RobStride2 · go_motor · JY61 · usb_receiver     │
├──────────────────────────────────────────────────────────────────┤
│  CANDrive · 485_bus · comm (遥控协议栈) · crc_ccitt · bsp_dwt   │
├──────────────────────────────────────────────────────────────────┤
│  RMLibHead (跨平台适配层) · My_list (通用双向链表)                 │
├──────────────────────────────────────────────────────────────────┤
│              FreeRTOS + CMSIS-RTOS2 + STM32 HAL                   │
└──────────────────────────────────────────────────────────────────┘
```

**数据流向：**
```
遥控器 ──[UART5 DMA]──▶ Remote_Analysis() ──▶ Remote_Control (期望速度 Vx,Vy,Wz)
                                                      │
                                              ┌───────▼────────┐
                                              │  ForceChassis   │
                                              │  逆解 → 轮子指令 │──▶ VESC (CAN2)
                                              └────────────────┘

光电门 GPIO ──▶ Back_Task() ──▶ Cubic_SetTrajectory() ──▶ RobStride MIT 控制 (CAN1)

遥控拨杆 ──▶ Hit_Task() ──▶ BallStateMachine() ──▶ M3508 串级 PID (CAN1)
```

---

## 🧠 核心巧思

### 1. 五次多项式轨迹规划器 (`AutoPilot.c`)

给定起点和终点的 **位置、速度、加速度** 共 6 个边界条件，直接解析求解五次多项式系数：

```
p(t) = a + b·t + c·t² + d·t³ + e·t⁴ + f·t⁵
```

**精妙之处：**

- **参数归一化**：`set_quintic()` 中将速度和加速度分别乘以 `dt` 和 `dt²`，使时间归一化到 [0,1]——避免浮点溢出且系数公式极简，仅用 6 行纯算术完成全部系数的闭式求解。

- **约束迭代时间缩放**：规划后遍历 (x, y, ω) 三个自由度的速度/加速度共 6 个维度，找出**超限比例最大**的维度作为缩放依据，按比例拉伸运动时间后重算曲线——只多花刚好满足最紧约束的时间，不保守也不激进。

- **力控/速度双模式兼容**：`setAcc_cb` 为 `NULL` 时自动跳过力控前馈输出，退化为纯速度/位置控制。同一个框架不加任何条件编译即可同时用于力控底盘和速度底盘。

- **安全取消**：外部通过 `AutoPilotCancleCurrentPlane()` 给信号量，当前轨迹立即中断，自动清零速度和加速度输出并清空任务队列——安全退出不甩尾。

- **链式回调**：每条轨迹执行完毕后调用 `finish_cb`，上层可在回调中链式下发下一条轨迹，实现连续路径拼接。

### 2. 力控底盘数学解耦 (`ForceChassis.c`)

将三个广义力 (Fx, Fy, Mz) 解耦为 N 个轮子的 (速度方向, 速度大小, 力矩)。

**精妙之处：**

- **矩阵预计算**：运动学映射矩阵 `A_vel` 和动力学映射矩阵 `force_A = A_vel^T` 在 `ChassisInit()` 中根据轮子安装位姿 `(x, y, φ)` 一次性算好并存储在 `Chassis_t` 结构体中。运行时纯矩阵乘法——零动态分配，零重复计算。

- **质心偏移的质量矩阵**：
  ```c
  Mq = [ m         0        -m·y_c      ]
       [ 0         m         m·x_c      ]
       [ -m·y_c    m·x_c     I + m(x_c² + y_c²) ]
  ```
  精确刻画了质心不在几何中心时的力-力矩耦合，支持非对称布局机器人。代码在 `ForceChassis.c:109-114` 一目了然。

- **力矩投影避免奇点**：速度幅值 < `dead_zone` 时直接置零力矩指令（`torque_projection = 0.0f`），防止 `atan2f` 在零速度附近不稳定导致舵轮剧烈跳变。

- **QR 分解正解**：从 N 个轮子实测速度反解机器人本体速度时，使用 QR 分解 (`solve_linear_system_qr_f32`) 求解超定方程组，天然处理轮子冗余——即使某个轮子打滑或离线，其余轮子数据仍可给出合理的速度估计。

- **安全检查循环**：每个控制周期遍历所有轮子状态——`WHEEL_IDEL` 自动触发复位，`WHEEL_ERROR` 回调上层处理，健康轮子按位掩码标记。

### 3. 弹射机构：三次多项式 + 状态机 (`Back_Task` + `step.c`)

弹射流程由四状态状态机驱动：`READY → PLAN → FIRE → ALIGN → READY`。

**精妙之处：**

- **计算与执行分离**：`PLAN` 状态中调用 `Cubic_SetTrajectory()` 根据当前位置和目标角度离线算出三次多项式系数，完成后立即切到 `FIRE` 状态执行——执行阶段零延迟，纯查表输出。

- **RobStride MIT 控制模式**：同时下发期望位置、速度、前馈力矩、Kp、Kd 五个参数，电机端做**阻抗控制**，力位混合跟踪——弹射力度可通过前馈力矩精确调节。

- **光电门互锁触发**：四个 GPIO 检测球到位，`trigger_lock` 互锁防止同一颗球重复触发弹射。

- **复位到位精度**：弹射完成后自动回到初始角度，到位阈值 < 0.03 rad，到位后释放锁允许再次触发。

- **最小时间保护**：`if (duration < 0.05f) duration = 0.05f`，防止极小时间导致三次多项式系数爆炸。

### 4. 扬臂：三路并发的球路状态机 (`Hit_Task`)

一个 M3508 电机，三条不同的击球轨迹（far / middle / near），由遥控器右侧拨杆选择。

**精妙之处：**

- **参数化状态机**：`BallStateMachine()` 接收 6 个参数（状态指针、初始化标志、目标角度、归位角度、准备时间、击球时间），将三条球路的逻辑**统一为一个函数**，调用三次即可——DRY 原则贯彻到底。

- **串级 PID**：外环位置环 → 内环速度环，将 M3508 的角度和速度同时锁定在轨迹上。

- **裁判系统通信**：击球前通过 `send_flag()` 经 UART4 向裁判系统发送球路编号（协议格式：`0xAA, cmd, 0x55`），实现比赛规则要求的击球申报。

- **三态互斥**：`init_done_far / init_done_middle / init_done_near` 三态互斥，确保同一时刻只有一条球路在执行。

### 5. RobStride 电机 CAN 协议 (`RobStride2.c`)

**精妙之处：**

- **扭矩编码进扩展帧 ID**：`ExtID = (0x1 << 24) | (torque_u16 << 8) | motor_id`。控制指令编码在 CAN ID 中，8 字节数据区全部留给位置/速度/Kp/Kd——一个扩展帧传完所有控制参数，带宽利用率极高。

- **按型号自动量化**：四种型号 (01~04) 有不同的扭矩/速度量程，`float_to_uint16()` 按型号自动选择范围映射——一套代码通吃全部 RobStride 系列。

- **反馈解码对称**：`(u16 - 32767) * scale / 32767`，收发使用相同的线性映射公式，降低出错概率。

- **扩展 ID 分段路由**：bit24~31 为消息类型（0 = 设备信息, 2 = 反馈数据, 17 = 参数回读, 21 = 错误信息），不同 ID 段走不同解码分支，清晰高效。

### 6. PID 库：三种 PID + 模糊 PID (`PID.c` + `PID.h`)

**精妙之处：**

- **改进标志位组合**：用 `uint8_t` 的每一位表示一种改进策略（积分限幅、微分先行、梯形积分、变速积分、输出滤波……），通过位或自由组合——不需要为每种组合写一个函数。

- **三种 PID 共存**：增量式 + 位置式 + 史密斯预测器共存于同一个结构体，按场景选用。

- **模糊 PID**：7×7 模糊规则表通过**双线性插值**实时调节 Kp/Ki/Kd，适合工况变化大、非线性强的场景。

- **工业级特性**：死区 + 输出限幅 + 积分分离 + 抗积分饱和——标准工业级 PID 的必备特性全部内置。

### 7. 遥控通信协议栈 (`Remote_control/`)

**精妙之处：**

- **命令-回调注册机制**：`register_comm_recv_cb(callback, cmd, user_data)` 按命令字注册回调，收到匹配数据包时自动分发——类似事件总线的设计模式。

- **零拷贝重传**：ACK 模式下数据指针在收到确认前一直有效，重发时直接复用原始指针，不拷贝。

- **双模式发送**：`asyn_comm_send_pack_nak`（非阻塞不等 ACK，适合高频状态上报）和 `comm_send_pack_ack`（阻塞等 ACK + 自动重试，适合关键指令），满足不同实时性需求。

- **键值边沿检测宏**：
  ```c
  #define KEY_RISING_EDGE(cur, last, field) \
      ((cur.field == 1) && (last.field == 0))
  ```
  遥控器 14 个按键用位域结构体承载，一行宏完成上升沿检测——可读性极强，零性能开销。

- **UART DMA 空闲中断**：使用 `HAL_UARTEx_ReceiveToIdle_DMA` 而非定长接收，遥控数据包变长也可正确分包，不丢帧不粘包。

### 8. RMLibHead 跨平台适配层

**精妙之处：**

- **编译时平台检测**：根据 `STM32F405xx` / `STM32H743xx` 等宏自动选择对应 HAL 头文件——同一套代码在 F4/H7 间迁移只需改芯片型号宏。

- **RTOS 透明切换**：定义 `__USE_RTOS` 时临界区自动用 `vPortEnterCritical()`，否则退化为空宏；内存分配自动用 `pvPortMalloc()` 或标准 `malloc()`——裸机和 RTOS 共用一套代码。

### 9. VESC CAN 协议实现 (`VESC.c`)

直接对接 VESC 开源电调的原生 CAN 协议（参考 `vedderb/bldc` 的 `comm_can.c`），支持设定**电流 / 占空比 / RPM / 位置**四种控制模式，同时接收状态帧（转速、电流、功率、温度、故障码）。

### 10. UART 错误恢复 (`HAL_UART_ErrorCallback`)

UART5 接收出错时手动按 STM32F4 参考手册要求的顺序（**先读 SR 再读 DR**）清除 ORE / NE / FE / PE 错误标志，然后重新启动 DMA 接收——防止遥控信号受干扰后通信永久中断。

### 11. USB 雷达透传 (`usb_receiver.c`)

通过 USB CDC (虚拟串口) 接收上位机雷达数据（dx, dy, 距离），解析后存入全局变量 `radar_dx / radar_dy / radar_d`，供 `Remote_Analysis()` 中的避障/导航逻辑使用。

---

## 📁 目录结构

```
exact_fire/
├── Chassis/                  # 核心框架层
│   ├── AutoPilot.c/h         #   五次多项式轨迹规划器
│   ├── ForceChassis.c/h      #   力控底盘解算器 (逆解+正解+QR)
│   └── Action_Config.c/h     #   动作配置（预留扩展）
├── lib/                      # 驱动与算法库
│   ├── RMLibHead.h           #   跨平台适配层 (F4/H7 兼容)
│   ├── PID.c/h               #   PID + 模糊PID + PI (位标志组合)
│   ├── PID_old.c/h           #   经典位置式PID (PID2)
│   ├── CANDrive.c/h          #   CAN 底层收发 (FIFO0/FIFO1 中断)
│   ├── motor.c/h             #   DJI 电机数据解析
│   ├── motorEx.c/h           #   DJI M3508 扩展 (串级PID)
│   ├── VESC.c/h              #   VESC CAN 协议 (电流/占空比/RPM/位置)
│   ├── RobStride2.c/h        #   RobStride 电机 CAN 协议 (MIT模式)
│   ├── go_motor.c/h          #   Unitree GoMotor RS485 协议
│   ├── 485_bus.c/h           #   RS485 总线抽象层
│   ├── bsp_dwt.c/h           #   DWT (Data Watchpoint) 高精度延时
│   ├── crc_ccitt.c/h         #   CRC-CCITT 校验
│   ├── JY61.c/h              #   JY61 姿态传感器 (UART DMA)
│   ├── usb_receiver.c/h      #   USB CDC 雷达数据接收
│   ├── usb_trans.c/h         #   USB 数据传输
│   ├── Vector.h              #   Vector2D / Vector3D 向量运算
│   └── Chassis.h             #   底盘通用定义
├── Mytask/
│   ├── Task_Init.c/h         #   业务逻辑：遥控、击球、弹射、USB雷达
│   └── config.h              #   机械/运动参数宏定义
├── Remote_control/           #   自定义遥控通信协议栈
│   ├── comm.c/h              #     协议栈核心 (ACK/NAK/重传)
│   ├── comm_stm32_hal_middle.c/h  # STM32 HAL 适配层
│   ├── data_poll.c/h         #     数据轮询
│   ├── dataFrame.h           #     数据帧定义 (PackControl/PackTransRemote)
│   ├── hardware.h            #     硬件抽象
│   └── My_list.c/h           #     通用双向链表
├── Matrix/                   #   矩阵运算库
│   ├── matrix.c/h            #     CMSIS-DSP 矩阵运算包装
│   └── svd.c/h               #     SVD/QR 分解 (超定方程组求解)
├── Core/                     #   STM32CubeMX 生成
│   ├── Src/main.c            #     入口：双CAN + 四UART + USB 初始化
│   ├── Src/freertos.c        #     RTOS 入口 + 任务创建
│   ├── Src/can.c             #     CAN1/CAN2 初始化
│   ├── Src/usart.c           #     UART2/4/5/6 初始化
│   ├── Src/dma.c             #     DMA 配置
│   ├── Src/gpio.c            #     GPIO 配置 (光电门等)
│   ├── Src/stm32f4xx_it.c    #     中断服务函数
│   └── Inc/                  #     对应头文件
├── MDK-ARM/                  #   Keil MDK 工程文件 & step 库
├── Middlewares/              #   FreeRTOS 源码
└── Drivers/                  #   CMSIS + STM32F4 HAL 驱动
```

---

## 🔧 硬件配置

| 参数 | 值 | 定义处 | 说明 |
|------|-----|--------|------|
| 主控 | STM32F407VGTx, 168MHz | `main.c` | Cortex-M4F, FPU |
| RTOS | FreeRTOS 10.x, CMSIS-RTOS2 | `freertos.c` | 抢占式调度 |
| CAN1 | 1 Mbps | `can.c` | RobStride 弹射电机 |
| CAN2 | 1 Mbps | `can.c` | VESC 底盘电调 |
| UART4 | 115200 bps, DMA | `usart.c` | 裁判系统 |
| UART5 | 115200 bps, DMA + IDLE | `usart.c` | 遥控器 |
| USB | FS (12 Mbps), CDC VCP | `usb_device.c` | 雷达数据 |
| 全向轮半径 | 0.075 m | `config.h`: `WHEEL_RADIUS` | VESC 速度换算 |
| 底盘轴距 (轮心到中心) | 0.457 m | `config.h`: `LENGTH` | 运动学解算 |
| 减速比 | 19.2:1 | `Task_Init.h`: `GEAR_RATIO` | 电机到轮子 |
| 最大线速度 | 10 m/s | `config.h`: `MAX_ROBOT_VEL` | 场地坐标系 |
| 最大角速度 | 90°/s (π/2 rad/s) | `config.h`: `MAX_ROBOT_OMEGA` | 场地坐标系 |
| 弹射电机 | RobStride_02 (扭矩 ±60 Nm) | `Task_Init.c` | CAN1, ID 0x01/0x02 |
| 扬臂电机 | DJI M3508 | `Task_Init.c` | CAN1, 串级 PID |
| 弹射轨迹时间 | 0.18 s | `Task_Init.c` | 三次多项式 |
| 球路数量 | 3 (远/中/近) | `Task_Init.c` | 遥控右侧拨杆切换 |
| 光电门 | 4 路 GPIO 输入 | `gpio.c` | 球检测触发弹射 |

---

## 🚀 快速开始

### 环境要求

- **IDE**：Keil MDK 5.36+ 或 STM32CubeIDE 1.12+
- **编译器**：ARM Compiler 5 (ArmCC) 或 ARM Compiler 6 (ArmClang)
- **HAL 版本**：STM32F4 HAL 1.24+（UART `ReceiveToIdle` 依赖）
- **硬件工具**：ST-Link/V2 或 J-Link 调试器

### 编译与烧录

```bash
# 1. 克隆仓库
git clone <repo-url>
cd exact_fire

# 2. 用 Keil MDK 打开
start MDK-ARM/exact_fire.uvprojx
# 或导入 STM32CubeIDE

# 3. 确认以下路径在工程配置中正确：
#    - Middlewares/FreeRTOS
#    - Drivers/CMSIS + Drivers/STM32F4xx_HAL_Driver

# 4. 根据实际机械尺寸修改 Mytask/config.h
#    重点关注: WHEEL_RADIUS, LENGTH, GEAR_RATIO, MAX_ROBOT_VEL

# 5. 编译 (F7) → 烧录 (F8) → 上电
```

### 上电流程

1. **VESC 电调**需提前用 VESC Tool 配置好 CAN 通信参数（ID、波特率 1 Mbps、控制模式）
2. RobStride 电机上电后固件自动调用 `RobStrideEnable()` 使能
3. 遥控器上电，UART5 DMA 空闲中断自动开始接收
4. 拨动遥控器**右侧三段开关**选择球路（上=远 / 中=中 / 下=近）
5. 光电门检测到球后自动触发弹射流程

### 调试建议

- 底盘调试时可先设为 `REMOTE` 模式（`chassis_mode = REMOTE`），用遥控器直接控制，验证运动学解算
- 确认 CAN 总线终端电阻 (120Ω) 已正确安装
- 可用逻辑分析仪挂 UART5 检查遥控数据帧格式
- VESC 状态帧 (CAN ID 0x0900+) 通过 CAN2 FIFO1 中断接收

---

## 📊 RTOS 任务一览

| 任务名 | 入口函数 | 栈大小 | 优先级 | 周期 | 功能 |
|--------|----------|--------|--------|------|------|
| `defaultTask` | `StartDefaultTask` | 512 B | Normal | 一次性 | 初始化 USB + 创建子任务 |
| `Remote` | `Remote()` | 400 words | 4 | 事件驱动 | 遥控器数据处理 + 底盘模式切换 |
| `Hit_Task` | `Hit_Task()` | 258 words | 4 | 事件驱动 | 扬臂击球状态机 (三球路) |
| `Back_Task` | `Back_Task()` | 400 words | 4 | 事件驱动 | 弹射状态机 (PLAN→FIRE→ALIGN) |
| `Remote_Analysis_Task` | `Remote_Analysis_Task()` | 400 words | 4 | 事件驱动 | 遥控数据解析 + USB 雷达融合 |
| `chassis_task` | `ChassisCalculateProcess` | 可配 | 可配 | `update_dt_ms` | 力控底盘逆解+正解循环 |
| `autopilot_task` | `AutoPilotProcess` | 可配 | 可配 | `dt_ms` | 五次多项式轨迹执行器 |

---

## 📝 作者

| 模块 | 作者 |
|------|------|
| 底盘框架 (AutoPilot / ForceChassis) | 刘远钊 |
| RobStride / VESC / GoMotor 驱动 | 刘家瑞 |
| 应用层逻辑 (Task_Init / Hit / Back) | 刘家瑞 |
| RMLib 基础库 | Yao (KDRobot) |

---

## ⚠️ 注意事项

- **VESC 电调**需提前用 VESC Tool 配置好 CAN 通信参数（ID、波特率 1 Mbps、控制模式为 PID Speed Control），否则底盘不响应
- **RobStride 电机**上电后需先发送 `RobStrideEnable()` 使能（固件在初始化中自动调用）
- **UART5 的 DMA 接收**使用 `HAL_UARTEx_ReceiveToIdle_DMA`，依赖 STM32 HAL 1.24+ 版本
- **CAN 终端电阻**：CAN1 和 CAN2 总线两端必须各并联 120Ω 终端电阻，否则通信不稳定
- `force_A_data[3][8]` / `A_vel_data[8][3]` 静态数组按最多 4 舵轮设计，扩展轮数需同步修改 `ForceChassis.h` 中的数组维度
- **光电门**使用 GPIO 外部中断触发，注意中断优先级不要高于 CAN 接收中断
- 本项目的 PID 参数均针对特定机械结构调校，更换机械后需重新整定
- 编译时确保 **FPU 已启用**（`__FPU_PRESENT`），否则 `arm_math.h` 的矩阵运算将回退到软浮点，性能急剧下降

---

## 📄 License

本项目为竞赛用途，保留所有权利。第三方库 (FreeRTOS, CMSIS, STM32 HAL) 遵循各自许可证。
