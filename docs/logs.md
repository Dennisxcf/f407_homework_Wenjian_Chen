# 调试日志

> 记录约定：每条包含 **目标 / 改动 / 现象或结论**。还没实测的部分统一标 `【待实测】`。
> 说明：本文件部分条目由 AI 协助整理，实测现象、波形和最终参数需要我自己补充。

---

## 1. 2026-10-17　框架从零搭起来（board → bsp → libs → modules → threads）

### 1.1 板级配置（CubeMX 6.15 + X-CUBE-AZRTOS-F4 1.1.0）

| 外设 | 关键参数 | 说明 |
| --- | --- | --- |
| 时钟 | 168MHz（HSE 8M，PLLM=4/PLLN=168/PLLP=2），APB1=42M、APB2=84M | PWM 与 CAN 分频都依赖这个 |
| HAL 时基 | **TIM14** | 必须不是 SysTick，SysTick 要留给 ThreadX |
| ThreadX | tick=1000Hz、`TX_TIMER_PROCESS_IN_ISR`、静态池 4096B | `tx_thread_sleep(1)` = 1ms |
| CAN1 | PSC=3、BS1=10TQ、BS2=3TQ、SJW=1TQ → **1Mbps**，PD1(TX)/PI9(RX) | 与电机通信的硬性要求 |
| CAN2 | 同上 → **1Mbps**，PB13(TX)/PB12(RX) | 见 1.2 第 4 条 |
| TIM1_CH1 | PSC=191、ARR=19999 → 50Hz（20ms） | 舵机 PWM，PE9 |
| TIM10_CH1 | PSC=0、ARR=24999 → 6.72kHz | 加热电阻，PF6；ARR+1=25000 正好对上 `TempPid.maxOut` |
| USART1 | 115200，PA9(TX)/PA10(RX) | 调试口 |
| USART6 | 115200，PC6(TX)/PC7(RX) | 裁判系统 |
| SPI1 | Mode0、8bit、MSB first、1.3125Mbps，PB3/4/5 | BMI088；ACC_CS=PA4，GYRO_CS=PB0 |
| LED | PH10 红 / PH11 绿 / PH12 蓝，**低电平点亮** | alive 线程用它反映状态 |
| NVIC | CAN1_RX0、CAN2_RX0、USART1、USART6、TIM14 | 漏一个就是"代码对了但收不到数据" |

### 1.2 踩过的坑（按出现顺序）

| # | 现象 | 原因 | 处理 |
| --- | --- | --- | --- |
| 1 | 编译 `fatal error: tx_api.h` | CubeMX 中件层选了 **FreeRTOS**，框架用的是 ThreadX API | 换成 X-CUBE-AZRTOS-F4 的 ThreadX |
| 2 | `htim10` undeclared | `BMI088.hpp` 里写死了加热定时器，但 TIM10 没配 | CubeMX 开 TIM10_CH1（PF6） |
| 3 | CAN 发送"成功"但电机不动 | `HAL_CAN_AddTxMessage` **不看 DLC**，无条件读 `aData[0..7]` | 发送前把数据补零成 8 字节，否则 `len<8` 是越界读栈 |
| 4 | CAN2 上电机无响应 | CAN2 波特率被配成 875kbps、一位只有 3Tq | 改成 PSC=3/BS1=10/BS2=3，与 CAN1 一致 |
| 5 | 接收回调永远不执行 | `.ioc` 里只有 `CAN1_RX0_IRQn`，CAN2_RX0 没勾 | 勾上并重新生成 `CAN2_RX0_IRQHandler` |
| 6 | 在 C++ 里实现的 HAL 回调"没生效" | 覆盖 `__weak` 函数必须保持 C 链接 | 写成 `extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(...)` |
| 7 | 加速度计芯片 ID 一直校验失败 | 加速度计 SPI 读数据有 **1 个 dummy 字节**，陀螺仪没有 | `ReadReg` 保持"原样透传"，dummy 由调用者跳过（与 `VerifyAccChipID` 读 2 字节取 `[1]` 的用法一致） |
| 8 | PID 没有积分效果 | 初版把位置式/增量式混写在一条流程里，且每周期把 `iResult` 清零 | 按 `mode` 分支；位置式保留 `iResult`，增量式保留 `result`；抗饱和只加一次积分 |
| 9 | 框架代码根本没参与编译 | `board/CMakeLists.txt` 只有 `enable_language(C ASM)`，源文件列表是空的 | 加 `enable_language(CXX)` + C++17 + GLOB 四个目录 + include 路径 |
| 10 | 误以为 ThreadX 定时器被禁用 | `tx_user.h` 里的 `#define TX_NO_TIMER` 在 `/* */` 注释块内 | 判断宏是否生效要看是否在注释里 |

### 1.3 各层实现要点

- **bsp**
  - `CAN_Transmit`：参数校验 → 邮箱满就返回 → 组帧头 → 补零到 8 字节 → `HAL_CAN_AddTxMessage`
  - 接收回调：循环取空 FIFO0 → 跳过非标准帧 → `StdId 0x201~0x208` 按 `index = StdId-0x201` 交给 `DJIMotorHandler`
  - `USART_Transmit/Receive`：按 mode 分派；无 DMA 时自动降级（DMA 路径留给以后配）
  - PWM：`TIM_GetTimerClock()` 自动区分 APB1/APB2 与预分频，`PWM_SetPeriod` 用 `ARR = clk×T/(PSC+1)-1`
- **libs**：PID 位置式/增量式 + 抗饱和；CRC16/CRC8 与 `Numeric::LimitABS` 现成可用
- **modules**
  - `DJIMotorHandler`：注册（0x201~0x208 校验 + 槽位）、`0x200`/`0x1FF` 大端打包发送、反馈解析（编码器回绕 + 减速比换算）
  - `M2006`：36:1 减速比、`maxCurrent=10000`（C610 ±10A）、位置环输出作为速度环目标、上电松开保护
  - `BMI088`：CS 控制 + SPI 读写、6G/2000dps 量程换算、11 位温度符号扩展、温度环
- **threads**（静态栈，不占 byte pool）
  - `imu`(prio 8, 1kHz)：ID 校验 → Config → 再校验 → 静置标定（4s）→ 1kHz 采集 + 温度环
  - `control`(prio 10, 1kHz)：上电先松开 1s → 按模式跑速度环或位置+速度双环 → 发帧 → 在线检查
  - `alive`(prio 20, 50ms)：用 `tx_semaphore_get` 统计心跳 → 灯效

### 1.4 验证情况

- 编译：14 个 `.cpp`（bsp 4 / libs 3 / modules 3 / threads 4）+ `main.c`、`app_azure_rtos.c` 均 `-fsyntax-only` 通过，0 error 0 warning
- PID 数值验证（把算法原样搬到脚本里跑，作为上板对照表）：

  | 用例 | 期望 | 结果 |
  | --- | --- | --- |
  | 纯 P（kp=1, ref=10） | 10,10,10 | ✅ |
  | 纯 I（ki=1, maxOut=100） | 涨到 100 后卡住（抗饱和） | ✅ |
  | I 饱和后 ref 反向 | 输出能退出饱和 | ✅ 100→90→80→70 |
  | 纯 D（kd=1，ref 10→0） | 10 后 -10 | ✅ |
  | 增量式（kp=ki=1） | 20,30,40,50 | ✅ |
  | 闭环一阶对象（PI） | 单调收敛 | ✅ 60 周期到 98.4 |
- 【待实测】`cmake --preset Debug && cmake --build --preset Debug` 全量编译 + 烧录
- 【待实测】上板灯效：蓝灯慢闪 4s（标定）→ 绿灯常亮
- 【待实测】电机转速/位置跟随；PID 实测参数

### 1.5 遗留与待办

1. PID 增益全是保守起始值（速度环 3000/30/0，位置环 10/0/0），必须实测调。
2. `M2006::AliveCheck` 判定偏严：每 1ms 比较一次 `AliveFlag`，丢一帧就报 OFFLINE，计划加超时计数。
3. `g_imu_data` 多字段无锁，做姿态解算前考虑双缓冲。
4. UART 还没配 DMA（现在走中断接收）；要 DMA + 空闲中断得回 CubeMX 配。
5. `bsp_usart.cpp` 的 `USART1_Init` / `USART6_Init` 仍是空壳，可在里面挂 `USART_Receive`。
6. 选做项未开始：IST8310 磁力计、6/9 轴姿态解算。
7. `board/Middlewares/Third_Party/FreeRTOS` 是残留目录，可删。
