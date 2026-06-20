# 🔥 exact_fire — 全向轮底盘 + RobStride 弹射 + 雷达视觉自主追踪，竞赛机器人主控

> **STM32F407VGTx @ 168MHz · FreeRTOS 10.x + CMSIS-RTOS2 · CAN 双总线 (CAN1 + CAN2) · USB CDC 雷达融合 · 自研力控底盘框架 · 五次多项式轨迹规划 · 模糊 PID**

---

## 📋 功能概览

本项目是一个完整的**三全向轮机器人竞赛主控固件**，在遥控底盘 + 弹射机构的基础上，**本版本核心新增了 USB 雷达视觉自主追踪与自动击球功能**，实现了从「纯遥控」到「雷达引导自主决策」的跨越。

| 子系统 | 硬件 | 通信 | 控制方式 |
|--------|------|------|----------|
| 底盘 (3 全向轮) | VESC v6.4 ×3 | CAN2 (1 Mbps) | PID 速度闭环 |
| 弹射机构 | RobStride 电机 ×2 | CAN1 (1 Mbps) | 位置 + 力混合 (MIT 阻抗模式) |
| 扬臂 | DJI M3508 | CAN1 | 位置 + 速度串级 PID |
| 遥控 | 自定义 UART 协议 | UART5 (DMA + 空闲中断) | 事件驱动回调 |
| 裁判系统交互 | 自定义 UART 协议 | UART4 (DMA) | DMA 半双工 |
| 🔴 **雷达视觉** | **USB CDC (VCP)** | **USB FS (12 Mbps)** | **P 控制器追踪 + 自动球路选择** |

---

## 🏗️ 架构分层

```
┌──────────────────────────────────────────────────────────────────────┐
│                  Task_Init.c（业务逻辑 + 双模状态机）                    │
│   Remote()  ·  Hit_Task()  ·  Back_Task()  ·  Remote_Analysis()     │
│                        ↙ 雷达融合点                    USB 雷达输入 ↗  │
├──────────────────────────────────────────────────────────────────────┤
│  AutoPilot (五次多项式)  │  ForceChassis (力控底盘)                    │
│  step/CubicParam (三次多项式)  │  PID + FuzzyPID + PID2              │
├──────────────────────────────────────────────────────────────────────┤
│  motor · VESC · RobStride2 · go_motor · JY61 · usb_receiver · usb_trans │
├──────────────────────────────────────────────────────────────────────┤
│  CANDrive · 485_bus · comm (遥控协议栈) · crc_ccitt · bsp_dwt         │
├──────────────────────────────────────────────────────────────────────┤
│  RMLibHead (跨平台适配层) · My_list (通用双向链表)                     │
├──────────────────────────────────────────────────────────────────────┤
│              FreeRTOS + CMSIS-RTOS2 + STM32 HAL                       │
└──────────────────────────────────────────────────────────────────────┘
```

**数据流向（含雷达链路）：**
```
🔴 上位机视觉 ──[USB CDC]──▶ usb_trans (DMA IRQ → Queue → Task)
                                        │
                                        ▼
                              usb_receiver (环形缓冲 + 帧同步)
                                        │
                              (x, y) ──▶ g_usb_data_sem
                                        │
                                        ▼
遥控器 ──[UART5 DMA]──▶ Remote_Analysis() ──▶ 双模决策:
                              │                    
                              ├─ radar_d ≤ 1.5m → AUTO 模式
                              │    · P 控制器追踪球 (Ex, Ey)
                              │    · 雷达距离自动选球路 (近/中/远)
                              │    · 触发击球流程
                              │
                              └─ radar_d > 1.5m → REMOTE 模式
                                   · 遥控摇杆控制底盘
                                   · 拨杆手动选球路
                                        │
                              ┌─────────▼────────┐
                              │  Remote() 任务    │
                              │  三全向轮逆解 +   │──▶ VESC (CAN2)
                              │  PID 速度闭环     │
                              └──────────────────┘

光电门 GPIO ──▶ Back_Task() ──▶ Cubic_SetTrajectory() ──▶ RobStride MIT 控制 (CAN1)

球路选择 (AUTO雷达 / REMOTE拨杆) ──▶ Hit_Task() ──▶ BallStateMachine() ──▶ M3508 串级 PID (CAN1)
```

---

## 🔴 雷达视觉自主追踪（本版本核心新增）

这是 exact_fire 从纯遥控到**半自主决策**的关键升级。上位机（Jetson / PC）运行视觉算法检测球的三维位置，通过 USB 虚拟串口将球相对于机器人的 (x, y) 坐标下发给 STM32，固件据此**自主完成追踪、选路、击球**的全流程。

### 硬件链路

```
相机/雷达 → 上位机视觉算法 → USB CDC (VCP) → STM32F407 USB FS → 固件处理
```

USB 使用 STM32F4 内置的 USB FS 控制器 (12 Mbps)，配置为 CDC (虚拟串口) 模式。上位机侧即插即用，无需额外驱动。

### USB 协议栈：三层架构

| 层 | 文件 | 职责 |
|----|------|------|
| HAL 适配层 | `usb_trans.c/h` | USB CDC 接收任务：DMA 中断 → Queue → Task 循环收包，动态调整缓冲区指针防溢出 |
| 帧解析层 | `usb_receiver.c/h` | 环形缓冲 + 帧头对齐 + 浮点解析 + 信号量通知上层 |
| 融合决策层 | `Task_Init.c` → `Remote_Analysis()` | 读取雷达坐标，P 控制器生成底盘期望速度，自动选球路 |

### 数据帧协议

```
┌──────┬──────────┬──────────┬──────┐
│ 0xAA │ float x  │ float y  │ 0xBB │
│  1B  │   4B     │   4B     │  1B  │
└──────┴──────────┴──────────┴──────┘
         Total: 10 bytes / frame
```

- `x`：球在机器人坐标系下的横向偏移（m），正 = 右方
- `y`：球在机器人坐标系下的纵向距离（m），正 = 前方
- 帧头 `0xAA` + 帧尾 `0xBB` 双层校验，防止字节流中的浮点数据被误认为帧边界

### 环形缓冲 + 帧同步算法 (`usb_receiver.c`)

USB CDC 是流式传输，没有包边界，一帧 10 字节可能被拆成多次 USB 包到达。固件实现了完整的字节流解析：

```
RingBuf_Align():
  扫描 ring_buf[0..ring_idx]，找到第一个 0xAA → 丢弃之前的垃圾字节 (memmove)
  
主循环:
  while (ring_idx >= 10):
    1. RingBuf_Align()           // 对齐到帧头
    2. 校验 ring_buf[9] == 0xBB  // 帧尾确认
    3. memcpy 提取 x, y (float)  // 直接二进制拷贝，零开销
    4. xSemaphoreGive()          // 通知 Remote_Analysis
    5. memmove 消费 10 字节       // 滑动窗口前移
```

**巧思：** 帧头校验失败时，`ring_buf[0] = 0x00` 破坏当前假帧头，下一轮 `RingBuf_Align()` 自动跳到下一个 `0xAA`——错误恢复仅需一行代码。

### USB 接收任务的缓冲区滑动窗口 (`usb_trans.c`)

`USB_RecvTask` 是 USB CDC 接收的核心调度任务：

```
while (1):
  USBD_CDC_SetRxBuffer(UserRxBufferFS + buffer_index)  // 动态偏移
  do:
    USBD_CDC_ReceivePacket()     // 触发 USB 硬件接收
    xQueueReceive(kUsbRecvQueue) // 等待 IRQ 通知 (packet size)
    buffer_index += packet_size
    if (buffer_index + 64 > 512):  // 即将溢出
      → 回退指针，回调 Overflow 通知上层
    else:
      → 缓冲区指针后移，下一包紧跟写入
  while (packet_size == 64)      // 64 = Full-speed BULK 最大包长，说明还有数据
  Recv_Finished_Cb(UserRxBufferFS, buffer_index)  // 一次完整帧组收完
```

**巧思：** `while (packet_size == 64)` 循环——USB FS 的 BULK 端点最大包长 64 字节；当收到满 64 字节包时说明上位机还有数据积压，继续收；当收到短包 (< 64) 时说明上位机本次发送完毕，调用回调一次性交给帧解析层。这是 USB CDC 标准的分包边界检测技法。

### 懒初始化 (Lazy Init)

雷达子系统**不在上电时初始化**，而是在 `Remote_Analysis()` 的每次循环中检测 USB 枚举状态：

```c
extern USBD_HandleTypeDef hUsbDeviceFS;
int usb_ready = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);

if (usb_ready && g_usb_data_sem == NULL) {
    UsbReceiver_Init();  // USB 插上后首次自动初始化
}
```

这意味着：
- 无上位机时，雷达相关代码零开销，不占用堆内存
- 上位机热插拔：插上自动初始化，拔掉自动退化为 `radar_d = 999.0f` → REMOTE 模式
- 不需要编译开关即可在「纯遥控版」和「雷达版」之间无缝切换

### 双模决策：AUTO vs REMOTE

`Remote_Analysis()` 是雷达数据和遥控数据的**汇合点**，每个周期执行一次：

```
雷达距离 radar_d = sqrt(x² + y²)

if radar_d ≤ 1.5m:
  ┌─────────────────────────────────────────────┐
  │  AUTO 模式                                    │
  │                                              │
  │  P 控制器追踪:                                │
  │    Ex = Kp × (radar_dx - 0.4f)   // 保持球在  │
  │    Ey = Kp × (radar_dy - 0.0f)   // 前方0.4m  │
  │    Eomega = 0                    // 不旋转     │
  │                                              │
  │  自动球路选择 (雷达测距):                      │
  │    radar_d < 0.3m        → near  (近路)       │
  │    0.3m ≤ radar_d < 0.8m  → middle (中路)     │
  │    radar_d ≥ 0.8m        → far   (远路)       │
  │                                              │
  │  三态互斥 + 触发击球                          │
  └─────────────────────────────────────────────┘
else:
  ┌─────────────────────────────────────────────┐
  │  REMOTE 模式                                  │
  │                                              │
  │  遥控摇杆 → Ex, Ey, Eomega                    │
  │  遥控拨杆 → 手动选球路                         │
  │  超时 200ms 未收到遥控 → 自动停车              │
  └─────────────────────────────────────────────┘
```

**巧思：** P 控制器的目标偏移量 `(radar_dx - 0.4f)` 不是直接冲向球，而是停在球前方 0.4m——这是弹射机构的最佳击球距离。机器人自动停到击球位置，然后自动选择对应球路触发击球，**追球→停车→选路→击球 全流程自主闭环**。

### 雷达数据流总结（端到端延迟分析）

```
USB IRQ (硬件) → Queue (ISR) → USB_RecvTask (任务唤醒)
     ~10μs          ~50μs         ~1ms (RTOS 调度)

→ usb_receiver 环形缓冲解析 → g_usb_data_sem → Remote_Analysis
     ~5μs                         ~0μs            ~1ms (任务周期)

→ Remote() 底盘解算 + PID → VESC CAN 指令下发
     ~50μs                     ~100μs (CAN 1Mbps)

总延迟: < 3ms (不含上位机视觉处理时间)
```

### 雷达子系统文件清单

| 文件 | 行数 | 功能 |
|------|------|------|
| `lib/usb_receiver.c` | 68 | 环形缓冲 + 0xAA/0xBB 帧解析 + 信号量通知 |
| `lib/usb_receiver.h` | 18 | `UsbRecvData_t` 结构体 (x, y) |
| `lib/usb_trans.c` | 64 | USB CDC 接收任务 (Queue + 滑动窗口) |
| `lib/usb_trans.h` | 29 | CDC 回调类型定义 + 缓冲区配置 |
| `Mytask/Task_Init.c` (Remote_Analysis) | ~55 | 懒初始化 + 双模决策 + P 控制器 + 自动选路 |

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

一个 M3508 电机，三条不同的击球轨迹（far / middle / near）。**在 AUTO 模式下球路由雷达距离自动选择，REMOTE 模式下由遥控器拨杆手动选择**，两种模式的球路触发逻辑统一走 `init_done` 三态互斥。

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

### 11. JY61 姿态传感器 (`JY61.c`)

通过 UART DMA 接收 JY61 IMU 的姿态数据（欧拉角/角速度），为底盘提供航向参考，结合遥控摇杆或雷达坐标实现场地坐标系下的运动控制。

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
│   ├── usb_receiver.c/h      #   🔴 USB 雷达帧解析 (环形缓冲+0xAA/0xBB)
│   ├── usb_trans.c/h         #   🔴 USB CDC 接收任务 (Queue+滑动窗口)
│   ├── Vector.h              #   Vector2D / Vector3D 向量运算
│   └── Chassis.h             #   底盘通用定义
├── Mytask/
│   ├── Task_Init.c/h         #   业务逻辑：遥控、击球、弹射、🔴雷达融合
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
| 🔴 USB | FS (12 Mbps), CDC VCP | `usb_device.c` | 雷达数据 |
| 全向轮半径 | 0.075 m | `config.h`: `WHEEL_RADIUS` | VESC 速度换算 |
| 底盘轴距 (轮心到中心) | 0.457 m | `config.h`: `LENGTH` | 运动学解算 |
| 减速比 | 19.2:1 | `Task_Init.h`: `GEAR_RATIO` | 电机到轮子 |
| 最大线速度 | 10 m/s | `config.h`: `MAX_ROBOT_VEL` | 场地坐标系 |
| 最大角速度 | 90°/s (π/2 rad/s) | `config.h`: `MAX_ROBOT_OMEGA` | 场地坐标系 |
| 🔴 P 控制器 Kp | 2.0 | `Task_Init.c` → `Remote_Analysis()` | 雷达追踪增益 |
| 🔴 AUTO 模式触发距离 | ≤ 1.5 m | `Task_Init.c` → `Remote_Analysis()` | 切换到自主追踪 |
| 🔴 P 控制器目标偏移 | (0.4, 0.0) m | `Task_Init.c` → `Remote_Analysis()` | 停在球前方 0.4m |
| 🔴 近路阈值 | < 0.3 m | `Task_Init.c` → `Remote()` | 雷达自动选 near |
| 🔴 中路阈值 | 0.3 ~ 0.8 m | `Task_Init.c` → `Remote()` | 雷达自动选 middle |
| 🔴 远路阈值 | ≥ 0.8 m | `Task_Init.c` → `Remote()` | 雷达自动选 far |
| 弹射电机 | RobStride_04 (扭矩 ±18 Nm) | `Task_Init.c` | CAN1, ID 0x01/0x02 |
| 扬臂电机 | DJI M3508 | `Task_Init.c` | CAN1, 串级 PID |
| 弹射轨迹时间 | 0.18 s | `Task_Init.c` | 三次多项式 |
| 球路数量 | 3 (远/中/近) | `Task_Init.c` | AUTO雷达 / REMOTE拨杆 |
| 光电门 | 4 路 GPIO 输入 | `gpio.c` | 球检测触发弹射 |

---

## 🚀 快速开始

### 环境要求

- **IDE**：Keil MDK 5.36+ 或 STM32CubeIDE 1.12+
- **编译器**：ARM Compiler 5 (ArmCC) 或 ARM Compiler 6 (ArmClang)
- **HAL 版本**：STM32F4 HAL 1.24+（UART `ReceiveToIdle` 依赖）
- **硬件工具**：ST-Link/V2 或 J-Link 调试器
- **雷达上位机**（AUTO 模式需要）：运行视觉算法的 Jetson / PC，通过 USB 连接 STM32 的 USB FS 口

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

1. **VESC 电调**需提前用 VESC Tool 配置好 CAN 通信参数（ID、波特率 1 Mbps、控制模式为 PID Speed Control）
2. RobStride 电机上电后固件自动调用 `RobStrideEnable()` 使能
3. 遥控器上电，UART5 DMA 空闲中断自动开始接收
4. **纯遥控模式**：拨动遥控器右侧三段开关选择球路（上=远 / 中=中 / 下=近），光电门检测到球后自动弹射
5. **雷达自主模式**：将上位机通过 USB 连接 STM32 的 USB FS 口，上位机启动视觉程序后固件自动切换到 AUTO 模式——机器人自主追踪球、自主选球路、自主击球
6. 拔掉 USB 自动退化为纯遥控模式，无需重启

### 调试建议

- **遥控优先调试**：上电默认 REMOTE 模式，先用遥控器验证底盘运动学解算和弹射机构
- **雷达调试**：USB 连接上位机后，可在 `Remote_Analysis()` 中打印 `radar_dx / radar_dy / radar_d` 确认坐标正确
- **P 控制器调参**：修改 `Kp = 2.0f` 和偏移量 `0.4f` 适配不同机器人的动态特性
- **球路阈值调参**：修改 `0.3f / 0.8f` 三个阈值适配不同场地的弹射距离需求
- 确认 CAN 总线终端电阻 (120Ω) 已正确安装
- 可用逻辑分析仪挂 UART5 检查遥控数据帧格式
- VESC 状态帧 (CAN ID 0x0900+) 通过 CAN2 FIFO1 中断接收

---

## 📊 RTOS 任务一览

| 任务名 | 入口函数 | 栈大小 | 优先级 | 周期 | 功能 |
|--------|----------|--------|--------|------|------|
| `defaultTask` | `StartDefaultTask` | 512 B | Normal | 一次性 | 初始化 USB 设备 + 创建子任务 |
| `Remote` | `Remote()` | 400 words | 4 | 2 ms | 三全向轮逆解 + PID 闭环 + 模式切换 + 球路触发 |
| `Hit_Task` | `Hit_Task()` | 258 words | 4 | 事件驱动 | 扬臂击球状态机 (三球路) |
| `Back_Task` | `Back_Task()` | 400 words | 4 | 事件驱动 | 弹射状态机 (PLAN→FIRE→ALIGN) |
| `Remote_Analysis_Task` | `Remote_Analysis_Task()` | 400 words | 4 | 10 ms | 🔴 雷达融合 + 双模决策 + 遥控数据解析 |
| `usb_cdc_recv` | `USB_RecvTask()` | 128 words | 4 | 事件驱动 | 🔴 USB CDC 数据接收 (Queue + 滑动窗口) |
| `chassis_task` | `ChassisCalculateProcess` | 可配 | 可配 | `update_dt_ms` | 力控底盘逆解+正解循环 |
| `autopilot_task` | `AutoPilotProcess` | 可配 | 可配 | `dt_ms` | 五次多项式轨迹执行器 |

---

## 📝 作者

| 模块 | 作者 |
|------|------|
| 底盘框架 (AutoPilot / ForceChassis) | 刘远钊 |
| RobStride / VESC / GoMotor 驱动 | 刘家瑞 |
| 应用层逻辑 (Task_Init / Hit / Back) | 刘家瑞 |
| 🔴 USB 雷达子系统 (usb_receiver / usb_trans / 双模融合) | 刘家瑞 |
| RMLib 基础库 | Yao (KDRobot) |

---

## ⚠️ 注意事项

- **VESC 电调**需提前用 VESC Tool 配置好 CAN 通信参数（ID、波特率 1 Mbps、控制模式为 PID Speed Control），否则底盘不响应
- **RobStride 电机**上电后需先发送 `RobStrideEnable()` 使能（固件在 `Back_Task` 中自动调用）
- **UART5 的 DMA 接收**使用 `HAL_UARTEx_ReceiveToIdle_DMA`，依赖 STM32 HAL 1.24+ 版本
- **CAN 终端电阻**：CAN1 和 CAN2 总线两端必须各并联 120Ω 终端电阻，否则通信不稳定
- `force_A_data[3][8]` / `A_vel_data[8][3]` 静态数组按最多 4 舵轮设计，扩展轮数需同步修改 `ForceChassis.h` 中的数组维度
- **光电门**使用 GPIO 外部中断触发，注意中断优先级不要高于 CAN 接收中断
- 🔴 **USB 雷达上位机**需按协议发送 `0xAA + float x + float y + 0xBB` 格式的 10 字节帧，坐标单位为米
- 🔴 **USB 线缆**需支持数据通信（非仅充电线），连接 STM32 开发板的 USB FS (Device) 口而非 USB HS 口
- 🔴 **雷达坐标系**：x 正 = 右，y 正 = 前，原点 = 机器人中心——上位机需做坐标系对齐
- 本项目的 PID 参数均针对特定机械结构调校，更换机械后需重新整定
- 编译时确保 **FPU 已启用**（`__FPU_PRESENT`），否则 `arm_math.h` 的矩阵运算将回退到软浮点，性能急剧下降
- 编译时确保 **USB Device 中间件**已启用（STM32CubeMX 中勾选 `USB_OTG_FS` → `Device_Only` + `CDC VCP`）

---

## 📄 License

本项目为竞赛用途，保留所有权利。第三方库 (FreeRTOS, CMSIS, STM32 HAL) 遵循各自许可证。
