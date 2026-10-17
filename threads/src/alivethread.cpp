/* ===========================================================================
 * [AI] 本文件由 agent 实现（原本为空）：alive 线程。
 *      用信号量 tx_semaphore_put / tx_semaphore_get 统计其它线程的心跳，
 *      再用板载 RGB 灯反映状态：
 *        红灯闪(500ms)  → IMU 初始化失败
 *        蓝灯慢闪(500ms)→ IMU 还在初始化/标定
 *        蓝灯快闪(100ms)→ imu 或 control 线程掉线
 *        绿灯常亮       → 一切正常
 * =========================================================================== */

#include "threads.hpp"

#include "main.h"

namespace
{
/* [AI] C 板 RGB 灯：PH10 红 / PH11 绿 / PH12 蓝，低电平点亮 */
constexpr uint16_t kLedRedPin = GPIO_PIN_10;
constexpr uint16_t kLedGreenPin = GPIO_PIN_11;
constexpr uint16_t kLedBluePin = GPIO_PIN_12;

inline void LedSet(uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(GPIOH, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* [AI] 把信号量里的心跳全部取走，返回本周期收到的次数 */
ULONG DrainBeats(TX_SEMAPHORE *semaphore)
{
    ULONG beats = 0U;

    while (tx_semaphore_get(semaphore, TX_NO_WAIT) == TX_SUCCESS)
    {
        beats++;
        if (beats > 10000U) /* 防御：参数异常时不要死循环 */
        {
            break;
        }
    }

    return beats;
}
} // namespace

extern "C" void AliveThreadEntry(ULONG thread_input)
{
    (void)thread_input;

    uint32_t tick = 0U;

    for (;;)
    {
        /* ---------- 1. 统计两个被监控线程的心跳 ---------- */
        const ULONG imu_beats = DrainBeats(&g_imu_heartbeat_sem);
        const ULONG control_beats = DrainBeats(&g_control_heartbeat_sem);

        g_threads_status.imu_alive = (imu_beats > 0U) ? 1U : 0U;
        g_threads_status.control_alive = (control_beats > 0U) ? 1U : 0U;

        /* ---------- 2. 灯效 ---------- */
        tick++;

        if (g_threads_status.imu_init_err)
        {
            /* IMU 初始化失败：红灯 500ms 周期闪 */
            const bool on = ((tick / 10U) % 2U) != 0U;
            LedSet(kLedGreenPin, false);
            LedSet(kLedBluePin, false);
            LedSet(kLedRedPin, on);
        }
        else if (!g_threads_status.imu_init_ok)
        {
            /* 还在初始化/标定：蓝灯 500ms 周期慢闪 */
            const bool on = ((tick / 10U) % 2U) != 0U;
            LedSet(kLedRedPin, false);
            LedSet(kLedGreenPin, false);
            LedSet(kLedBluePin, on);
        }
        else if (!g_threads_status.imu_alive || !g_threads_status.control_alive)
        {
            /* 有线程掉线：蓝灯 100ms 周期快闪 */
            const bool on = ((tick / 2U) % 2U) != 0U;
            LedSet(kLedRedPin, false);
            LedSet(kLedGreenPin, false);
            LedSet(kLedBluePin, on);
        }
        else
        {
            /* 一切正常：绿灯常亮 */
            LedSet(kLedRedPin, false);
            LedSet(kLedBluePin, false);
            LedSet(kLedGreenPin, true);
        }

        /* ---------- 3. 50ms 后继续巡检 ---------- */
        tx_thread_sleep(ALIVE_THREAD_PERIOD_TICKS);
    }
}
