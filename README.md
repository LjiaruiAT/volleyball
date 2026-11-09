# ROBOCON 排球机器人主控（Volley_ok）

三轮全向轮底盘 + RobStride 弹射 + Unitree GoMotor 击球
**STM32F407VETx · FreeRTOS · 双 CAN · RS485 · 自定义遥控协议**

> 2026 赛季排球机器人主控固件。本 README 描述的是**这个仓库里实际存在的代码**。

---

## 1. 项目背景

ROBOCON 排球赛项的动作链：底盘移动到击球位 → 姿态对准 → 挑球 / 弹射 → 二级机构击球出球。

这台车的机构方案是「三轮全向轮底盘 + 灵足（RobStride）电机弹射 + 宇树（Unitree）GoMotor 二级击球」，三种电机走三种不同协议，控制侧的主要工作是把它们统一到一个状态机里。

---

## 2. 硬件与总线

| 子系统 | 硬件 | 接口 | 说明 |
|---|---|---|---|
| 主控 | STM32F407VETx | — | CubeMX 工程 `Volley_ok.ioc` |
| 实时内核 | FreeRTOS | — | `Core/Src/freertos.c` |
| 底盘 | VESC 驱动 5055 三轮全向 | CAN2 | `MyLib/VESC.c` |
| 弹射 | RobStride（灵足） | CAN1 扩展帧 | `MyLib/RobStride2.c`，MIT 力位混合 |
| 击球 | Unitree GoMotor | RS485 | `MyLib/go_motor.c` + `MyLib/485_bus.c` |
| 遥控 | 自定义协议 | UART + DMA | `Remote_control/` 协议栈 |
| DJI 电机 | M3508 / C620 等 | CAN | `MyLib/motor.c` `motorEx.c` |

---

## 3. 目录结构

```
Volley_ok/
├── Core/            CubeMX 生成：CAN / UART / DMA / GPIO / FreeRTOS
├── Mytask/          业务层：Task_Init.c（状态机与任务）、config.h（参数）
├── lib/             驱动与算法：
│   ├── CANDrive      CAN 底层收发
│   ├── motor/motorEx DJI 电机解析与串级 PID
│   ├── VESC          VESC 原生 CAN 协议
│   ├── RobStride2    RobStride MIT 力位混合协议
│   ├── go_motor      Unitree GoMotor RS485 协议
│   ├── 485_bus       RS485 总线抽象
│   ├── PID_old       位置式 PID
│   ├── RMLibHead     跨平台适配层
│   └── crc_ccitt / mylist / bsp_dwt
├── Remote_control/  自定义遥控协议栈（命令-回调注册、ACK 重传、DMA 空闲中断分包）
└── MDK-ARM/         Keil 工程 + step.c（三次多项式轨迹）
```

---

## 4. 核心实现

### 4.1 三类电机统一接口

底盘（VESC）、弹射（RobStride）、击球（GoMotor）的协议完全不同：

- **VESC**：`ExtId = (packet << 8) | motor_id`，设定电流/占空比/RPM/位置，反馈帧带转速、电流、温度；
- **RobStride**：扭矩编码进扩展帧 ID `(0x1 << 24) | (torque << 8) | motor_id`，数据区放位置/速度/Kp/Kd，实现 MIT 力位混合；**上电必须先发使能帧**，否则电机不响应；
- **GoMotor**：RS485 帧，经 `485_bus.c` 抽象后由 `go_motor.c` 封装。

上层 `Task_Init.c` 只面对"目标位置/目标力"，不关心具体型号。

### 4.2 状态机与多机构互斥

挑球、弹射、击球是并发动作，用状态机而不是顺序 if-else，保证同一时刻只有一个动作在跑，出问题时也能明确定位停在哪一步。

### 4.3 遥控协议栈

`Remote_control/comm.c` 提供 `register_comm_recv_cb(callback, cmd, user_data)` 的命令-回调注册机制，收到匹配命令字自动分发；配合 `HAL_UARTEx_ReceiveToIdle_DMA` 处理变长数据包。

### 4.4 串口错误恢复

`HAL_UART_ErrorCallback` 中按参考手册顺序（先读 SR 再读 DR）清除 ORE/NE/FE/PE 标志后重挂 DMA——不清标志直接重挂会导致接收永久失效。

---

## 5. 个人职责

团队项目。本人负责：

- **电机驱动层**：`RobStride2.c`、`VESC.c`、`go_motor.c`、`485_bus.c` 的协议实现与调试
- **应用层逻辑**：`Task_Init.c` 中的动作状态机与多机构协同
- 整机联调与参数整定

> 底盘力控框架、轨迹规划与机械结构由团队其他成员完成，不在本仓库 claim 范围内。

---

## 6. 快速开始

1. Keil MDK 打开 `MDK-ARM/Volley_ok.uvprojx`（也可用 EIDE 配置，见 `MDK-ARM/.eide/`）
2. 按实际机械尺寸核对 `Mytask/config.h` 中的参数
3. 编译 → 烧录
4. 遥控上电，先用低速档验证底盘与各机构动作方向

---

## 7. 已知限制

- 无单元测试，验证依赖实机联调
- 参数（PID、限幅、零位）与具体机械强相关，换车需重新标定
- 本仓库为 2026 赛季版本；2027 赛季的重构在 `volley_2027`
- 工程内保留了 CubeMX/Keil 生成的 `Drivers/`、`Middlewares/`，因此仓库体积较大

---

## 8. 注意事项

- VESC 需先用 VESC Tool 配好 CAN ID 与波特率
- RobStride 上电必须发使能帧
- 变长遥控包依赖 `ReceiveToIdle`（HAL 1.24+）
- 电气与气动相关 GPIO 映射需与实际接线一致，换车必须重新确认
