/* ===========================================================================
 * [AI] 本文件由 agent 实现（原文件为空）：IMU 线程。
 *      流程：芯片 ID 校验 → Config → 静置标定 → 1kHz 循环读数据 + 温度控制
 * =========================================================================== */

#include "threads.hpp"

#include "BMI088.hpp"
#include "bsp_pwm.hpp"

namespace
{
/* [AI] IMU 实例（由 imu 线程独占使用） */
BMI088::cBMI088 imu;

/* [AI] 清空快照，避免下游读到上一帧残留 */
void ClearImuSnapshot(void)
{
    g_imu_data.acc[0] = g_imu_data.acc[1] = g_imu_data.acc[2] = 0.0f;
    g_imu_data.gyro[0] = g_imu_data.gyro[1] = g_imu_data.gyro[2] = 0.0f;
    g_imu_data.temperature = 0.0f;
    g_imu_data.timestamp = 0U;
}
} // namespace

extern "C" void ImuThreadEntry(ULONG thread_input)
{
    (void)thread_input;

    ClearImuSnapshot();
    g_threads_status.imu_init_ok = 0U;
    g_threads_status.imu_init_err = 0U;
    g_threads_status.imu_data_ok = 0U;

    /* ------------------ 1. 芯片 ID 校验 ------------------ */
    /* [AI] imu_error_t 默认 INIT_ERR = true，只有校验全过才在下面清掉 */
    imu.self_test.INIT_ERR = true;

    imu.VerifyAccChipID();
    imu.VerifyGyroChipID();

    /* ------------------ 2. 配置寄存器 ------------------ */
    if (!imu.self_test.INIT_ERR)
    {
        imu.Config();

        /* [AI] 配置完再确认一次芯片还在（软复位后重新读 ID） */
        imu.VerifyAccChipID();
        imu.VerifyGyroChipID();
    }

    imu.self_test.INIT_ERR = (imu.self_test.ACC_CHIP_ID_ERR || imu.self_test.GYRO_CHIP_ID_ERR);
    g_threads_status.imu_init_ok = imu.self_test.INIT_ERR ? 0U : 1U;
    g_threads_status.imu_init_err = imu.self_test.INIT_ERR ? 1U : 0U;

    /* ------------------ 3. 静置标定（约 4 秒，期间不要动板子） ------------------ */
    if (!imu.self_test.INIT_ERR)
    {
        imu.Calibrate();

        imu.VerifyAccData();
        imu.VerifyGyroData();
        g_threads_status.imu_data_ok =
            (imu.self_test.ACC_DATA_ERR || imu.self_test.GYRO_DATA_ERR) ? 0U : 1U;
    }

    /* ------------------ 4. 启动加热 PWM ------------------ */
    /* [AI] bsp 的 PWM_Init() 只启动了 TIM1_CH1（舵机），加热通道要单独启动 */
    PWM_Start(&HEATING_RESISTANCE_TIM, HEATING_RESISTANCE_CHANNEL);

    /* ------------------ 5. 1kHz 采样循环 ------------------ */
    for (;;)
    {
        acc_data_t acc = {};
        gyro_data_t gyro = {};

        imu.ReadAccData(&acc);   /* 内部已经把温度也读进 acc.temperature */
        imu.ReadGyroData(&gyro);

        g_imu_data.acc[0] = acc.x;
        g_imu_data.acc[1] = acc.y;
        g_imu_data.acc[2] = acc.z;

        g_imu_data.gyro[0] = gyro.x;
        g_imu_data.gyro[1] = gyro.y;
        g_imu_data.gyro[2] = gyro.z;

        g_imu_data.temperature = acc.temperature;
        g_imu_data.timestamp = tx_time_get();

        /* [AI] 温度环：维持恒温以抑制陀螺仪温漂 */
        imu.TemperatureControl(IMU_TARGET_TEMPERATURE);

        /* [AI] 心跳：告诉 alive 线程"我还活着" */
        tx_semaphore_put(&g_imu_heartbeat_sem);

        tx_thread_sleep(1); /* 1 tick = 1ms @ TX_TIMER_TICKS_PER_SECOND = 1000 */
    }
}
