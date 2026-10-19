# 调试日志

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

### 1.2 坑

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

### 1.3 各层实现

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

- ✅ **编译通过**：`cmake --preset Debug` + `cmake --build --preset Debug`，产物 `board/build/Debug/board.elf`（1.6MB，含 `.debug_info`/`.debug_line`，`-O0 -g3`）
- ✅ **ELF 自检**（用 nm/objdump 核对）：
  `main=0x08006F28`、`SystemClock_Config=0x08006F80`、`Error_Handler=0x08007078`、`HardFault_Handler=0x08007CB4`、`Default_Handler=0x08007EE0`、`ImuThreadEntry=0x08009A94`、`ControlThreadEntry=0x0800987C`、`AliveThreadEntry=0x0800973C`、`Threads_Init=0x08009C50`；向量表初始 SP=0x20020000、Reset=0x08007E91
- ✅ **PID 数值验证**（把 `UpdateResult()` 的算法原样搬到脚本里跑，作为上板对照表）：

  | 用例 | 期望 | 结果 |
  | --- | --- | --- |
  | 纯 P（kp=1, ref=10） | 10,10,10 | ✅ |
  | 纯 I（ki=1, maxOut=100） | 涨到 100 后卡住（抗饱和） | ✅ |
  | I 饱和后 ref 反向 | 输出能退出饱和 | ✅ 100→90→80→70 |
  | 纯 D（kd=1，ref 10→0） | 10 后 -10 | ✅ |
  | 增量式（kp=ki=1） | 20,30,40,50 | ✅ |
  | 闭环一阶对象（PI） | 单调收敛 | ✅ 60 周期到 98.4 |
- 【待实测】上板灯效（蓝灯慢闪 4s → 绿灯常亮）
- 【待实测】BMI088 三轴/温度数值、电机转速与位置跟随、PID 实测参数

### 1.5 遗留与待办

1. PID 增益全是保守起始值（速度环 3000/30/0，位置环 10/0/0），必须实测调。
2. `M2006::AliveCheck` 判定偏严：每 1ms 比较一次 `AliveFlag`，丢一帧就报 OFFLINE，计划加超时计数。
3. `g_imu_data` 多字段无锁，做姿态解算前考虑双缓冲。
4. UART 还没配 DMA（现在走中断接收）；要 DMA + 空闲中断得回 CubeMX 配。
5. `bsp_usart.cpp` 的 `USART1_Init` / `USART6_Init` 仍是空壳，可在里面挂 `USART_Receive`。

---

## 2. 2026-10-18　上板调试（阶段一：先把构建和调试器跑通）

目标：让 `board/` 工程能编译、能下载、能被 VS Code 调试；逐个定位"为什么断点不进、为什么卡住"。

### 2.1 CMake 报 `Could not find toolchain file`

- **现象**：
  ```
  Could not find toolchain file: D:/PnX workplace/rm27_training_framework_hw/cmake/gcc-arm-none-eabi.cmake
  Call Stack: CMakeLists.txt:28 (project)
  ```
- **原因**：VS Code 的 CMake Tools 默认把**工作区根目录**当 source dir，于是用了根目录那份**旧版残留**的 `CMakePresets.json`（它指向 `<根目录>/cmake/...`，而根目录根本没有 `cmake/` 文件夹）。真正的工程根是 `board/`。
- **处理**：
  - `.vscode/settings.json` 加 `"cmake.sourceDirectory": "${workspaceFolder}/board"` + `"cmake.useCMakePresets": "always"`
  - 删掉 configure 中断留下的 `board/build/` 缓存目录
  - 命令行验证：在 `board/` 下 `cmake --preset Debug` → `cmake --build --preset Debug` ✅
- **结论**：以后所有 CMake 操作都在 `board/` 目录下进行（命令行 `cd board` 或 VS Code 状态栏的 preset）。

### 2.2 Cortex-Debug 报 `Failed to launch OpenOCD GDB Server: Timeout`

- **现象**：Cortex-Debug 启动后卡住，最后 `Failed to launch OpenOCD GDB Server: Timeout`；手动跑同一条 openocd 命令得到
  ```
  Warn : DEPRECATED: auto-selecting transport "swd (dapdirect)"
  Error: read version failed
  ```
- **原因**：配置文件（`interface/stlink.cfg`、`target/stm32f4x.cfg`）能找到，**卡在跟 ST-Link 通信**这一步 —— 即调试器没被正常识别。
- **排查项**：① ST-Link 插好、线是数据线；② **C 板必须独立供电**（ST-Link 只给 SWD 信号，不给板子供电，板子没电就一定读不到）；③ SWCLK/SWDIO/GND 接线；④ 驱动（`STM32_Programmer_CLI.exe -l` 能看到 ST-Link 才算驱动 OK）。
- **结果**：处理后现在能正常连接（`[stm32f4x.cpu] halted due to debug-request ... pc: 0x08007e90` = Reset_Handler，说明链路通了）。【待补充：具体是哪一项导致的、怎么改好的】
- **附带信息**：`Unable to match requested speed 2000 kHz, using 1800 kHz` 是 SWD 速度自适应，无害。

### 2.3 main 里的断点不命中

- **现象**：在 `main()` 里下的两个断点都不触发。
- **日志关键证据**：
  ```
  LiveGDB: Reset_Handler () at startup_stm32f407xx.s:61
  LiveGDB: Program stopped, probably due to a reset and/or halt issued by debugger
  Started live-monitor-gdb session
  [stm32f4x.cpu] halted due to debug-request ... pc: 0x08007e90   ← 又复位回 Reset_Handler
  Temporary breakpoint 3, main () at .../main.c:80
  80  HAL_Init();
  ```
- **原因**：`launch.json` 里开了 `liveWatch`，它会**额外启动一个 live-monitor GDB 会话**去高频轮询变量，在后台不停复位/暂停目标 → 主会话的断点被反复打断，几乎不可能命中。
- **处理**：
  1. 把两条配置里的 `liveWatch` 注释掉；
  2. 两条配置都加 `"breakAfterReset": true`，保证每次启动都复位并停在 `main`；
  3. 重新 F5 → 稳定停在 `main.c:80 HAL_Init()` → Continue 后断点正常命中。
- **结论**：**排障期间不要开 `liveWatch`**。以后要看得等断点稳定后再打开。

### 2.4 SPI 传输超时，BMI088 没通

- **现象**：调试器连接后首次 halt 的位置在 HAL 的 SPI 里：
  ```
  0x080046d2 in SPI_WaitFlagStateUntilTimeout (hspi=0x200012ec <hspi1>, Flag=128, State=RESET, Timeout=50, ...)
  stm32f4xx_hal_spi.c:3515
  ```
- **分析**：`Flag=128` 是 **SPI_FLAG_BSY**，`State=RESET` = 在等"总线空闲"，等到 50ms 超时。说明上一次运行**卡死在一次 SPI 传输里**（`HAL_SPI_Transmit` 返回 `HAL_TIMEOUT`），BMI088 完全没通，与"芯片 ID 校验失败/红灯"对应。
- **结果**：已解决

### 2.5 上电瞬间两片 CS 都是低电平

- **现象**：CubeMX 生成的 `board/Core/Src/gpio.c` 里
  ```
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // 加速度计 CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   // 陀螺仪 CS
  ```
  两个片选**初始都是低电平**，即上电瞬间两片同时被选中。
- **风险**：SPI 是共用 MISO 的总线，两个从机同时驱动 MISO 会冲突，也可能是 BSY 卡死的原因之一。
- **计划**：在 `bsp_Init()` 里（CAN 初始化之前）先把 PA4/PB0 拉高；或在 CubeMX 里把这两个脚的初始电平改成 High。`【待实测】`对照修改前后的现象。

### 2.6 电机不动

- `threads/src/threads_init.cpp` 里默认值：
  ```
  g_motor_target_speed    = 0.0f;
  g_control_mode          = THREADS_CONTROL_RELAX;   // 松开模式，输出 0
  ```
- `controlthread.cpp` 里还有"上电前 1 秒强制松开"保护 → 所以上电后电机**本来就不应该动**。
- **让电机转的两种方式**：
  1. 不改代码：调试器里改 `g_control_mode = 1`（速度环）、`g_motor_target_speed = 0.2f`
  2. 改代码：`threads_init.cpp:14-16` 改初值
- **验证顺序**（计划）：先确认总线通（`motorFeedback.speed_rpm` 是否在变）→ 再开环电流验证电机硬件 → 最后上速度环/双环。
- **结果**：能成功转动

### 2.7 下一步

1. 定位并修好 SPI（2.4 / 2.5），目标是**绿灯常亮**（= IMU 初始化成功）。
2. 打通 CAN：确认 `motorFeedback` 在更新 → 再开环给电流 → 再速度环 → 再双环。
3. 按 README 验收项逐条记录实测数据，把本文档里的 `【待实测】`/`【待补充】` 全部替换掉。
