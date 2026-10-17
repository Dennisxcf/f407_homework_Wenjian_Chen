#ifndef THREADS_HPP
#define THREADS_HPP

/* ===========================================================================
 * [AI] 本文件由 agent 新建（原文件为空）。threads 层的公共接口。
 *      注意：为了让 board/AZURE_RTOS/App/app_azure_rtos.c（C 文件）也能调用
 *      Threads_Init()，本头文件保持 C 兼容（只有 typedef / extern / extern "C"）。
 * =========================================================================== */

#include <stdint.h>
#include "tx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- 可调参数 ----------------------------- */
/* ThreadX 优先级：数字越小优先级越高 */
#define IMU_THREAD_PRIORITY        8
#define CONTROL_THREAD_PRIORITY    10
#define ALIVE_THREAD_PRIORITY      20

/* 线程栈大小（字节）。用静态数组，不占用 tx_byte_pool */
#define IMU_THREAD_STACK_SIZE      2048
#define CONTROL_THREAD_STACK_SIZE  2048
#define ALIVE_THREAD_STACK_SIZE    1024

/* alive 线程的巡检周期（tick，1 tick = 1ms） */
#define ALIVE_THREAD_PERIOD_TICKS  50U

/* 加热目标温度（摄氏度） */
#define IMU_TARGET_TEMPERATURE     40.0f

/* control 线程的控制模式 */
#define THREADS_CONTROL_RELAX      0U /* 松开：输出 0 */
#define THREADS_CONTROL_SPEED      1U /* 速度环 */
#define THREADS_CONTROL_POS        2U /* 位置环 + 速度环串联 */

/* --------------------- IMU 数据（线程间共享） --------------------- */
typedef struct
{
    float acc[3];       /* 加速度，m/s^2 */
    float gyro[3];      /* 角速度，rad/s */
    float temperature;  /* 芯片温度，摄氏度 */
    uint32_t timestamp; /* 采集时刻（tx_time_get()） */
} imu_data_t;

/* --------------------- 线程监控状态 --------------------- */
typedef struct
{
    uint8_t imu_init_ok;   /* 1 = IMU 初始化成功 */
    uint8_t imu_init_err;  /* 1 = IMU 初始化失败 */
    uint8_t imu_data_ok;   /* 1 = 最近一次数据校验通过 */
    uint8_t imu_alive;     /* 1 = 上一巡检周期收到了 imu 心跳 */
    uint8_t control_alive; /* 1 = 上一巡检周期收到了 control 心跳 */
} threads_status_t;

/* --------------------- 共享变量（定义在 threads_init.cpp） --------------------- */
extern imu_data_t g_imu_data;
extern threads_status_t g_threads_status;

extern float g_motor_target_speed;    /* 速度环目标，rad/s（输出轴） */
extern float g_motor_target_position; /* 位置环目标，rad，[-π, π] */
extern uint8_t g_control_mode;        /* THREADS_CONTROL_xxx */

/* 心跳信号量：被监控线程 tx_semaphore_put，alive 线程 tx_semaphore_get */
extern TX_SEMAPHORE g_imu_heartbeat_sem;
extern TX_SEMAPHORE g_control_heartbeat_sem;

/* --------------------- 接口 --------------------- */
/**
 * @brief 创建信号量与三个线程。
 * @note 必须在 tx_application_define() 里调用（初始化上下文，不能 sleep）。
 */
void Threads_Init(void);

/* 内部线程入口（由 Threads_Init 创建，用户代码不用直接调） */
void ImuThreadEntry(ULONG thread_input);
void ControlThreadEntry(ULONG thread_input);
void AliveThreadEntry(ULONG thread_input);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP
