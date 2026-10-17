/* ===========================================================================
 * [AI] 本文件由 agent 新建：threads 层共享变量的定义 + Threads_Init()。
 *      线程栈全部用静态数组（8 字节对齐），不占 tx_byte_pool；
 *      Threads_Init() 必须在 tx_application_define() 里调用（此时不能 sleep）。
 * =========================================================================== */

#include "threads.hpp"

/* --------------------- 共享变量定义 --------------------- */
imu_data_t g_imu_data = {};

threads_status_t g_threads_status = {};

float g_motor_target_speed = 0.0f;
float g_motor_target_position = 0.0f;
uint8_t g_control_mode = THREADS_CONTROL_RELAX;

TX_SEMAPHORE g_imu_heartbeat_sem;
TX_SEMAPHORE g_control_heartbeat_sem;

/* --------------------- 线程对象与静态栈 --------------------- */
static TX_THREAD imu_thread;
static TX_THREAD control_thread;
static TX_THREAD alive_thread;

static uint8_t imu_thread_stack[IMU_THREAD_STACK_SIZE] __attribute__((aligned(8)));
static uint8_t control_thread_stack[CONTROL_THREAD_STACK_SIZE] __attribute__((aligned(8)));
static uint8_t alive_thread_stack[ALIVE_THREAD_STACK_SIZE] __attribute__((aligned(8)));

extern "C" void Threads_Init(void)
{
    /* [AI] 心跳信号量：初始计数 0，被监控线程 put，alive 线程 get */
    tx_semaphore_create(&g_imu_heartbeat_sem, (CHAR *)"imu_beat", 0);
    tx_semaphore_create(&g_control_heartbeat_sem, (CHAR *)"ctrl_beat", 0);

    /* [AI] imu：最高优先级，1kHz 采样 */
    tx_thread_create(&imu_thread, (CHAR *)"imu",
                     ImuThreadEntry, 0,
                     imu_thread_stack, sizeof(imu_thread_stack),
                     IMU_THREAD_PRIORITY, IMU_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    /* [AI] control：1kHz 电机控制 */
    tx_thread_create(&control_thread, (CHAR *)"control",
                     ControlThreadEntry, 0,
                     control_thread_stack, sizeof(control_thread_stack),
                     CONTROL_THREAD_PRIORITY, CONTROL_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    /* [AI] alive：50ms 巡检一次，优先级最低 */
    tx_thread_create(&alive_thread, (CHAR *)"alive",
                     AliveThreadEntry, 0,
                     alive_thread_stack, sizeof(alive_thread_stack),
                     ALIVE_THREAD_PRIORITY, ALIVE_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}
