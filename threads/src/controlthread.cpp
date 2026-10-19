/* ===========================================================================
 * [AI] 本文件由 agent 实现（原本为空）：control 线程。
 *      单电机（M2006 @ CAN1 0x201）双环控制演示：
 *      上电先松开 1 秒 → 按 g_control_mode 运行 → 每 1ms 发一帧控制报文
 * =========================================================================== */

#include "threads.hpp"

#include "DJIMotorHandler.hpp"
#include "M2006.hpp"

namespace
{
/* [AI] 被控电机实例（control 线程独占使用） */
M2006 motor;

/* [AI] 上电保护时间：这段时间内强制松开，让电调确认零电流 */
constexpr uint32_t kStartupRelaxTicks = 1000U;
} // namespace

extern "C" void ControlThreadEntry(ULONG thread_input)
{
    (void)thread_input;

    DJIMotorHandler *handler = DJIMotorHandler::Instance();

    /* [AI] 注册电机：CAN1 + 反馈 ID 0x201 */
    handler->registerMotor(&motor, &hcan1, 0x201U);

    /* [AI] 初始状态：松开、目标为 0 */
    motor.controlMode = DJIMotor::RELAX_MODE;
    motor.speedSet = 1.0f;
    motor.positionSet = 0.0f;
    motor.currentSet = 0;

    const uint32_t start_tick = tx_time_get();

    for (;;)
    {
        if ((tx_time_get() - start_tick) < kStartupRelaxTicks)
        {
            motor.controlMode = DJIMotor::RELAX_MODE;
            g_control_mode = THREADS_CONTROL_RELAX;
        }
        else
        {
            switch (g_control_mode)
            {
            case THREADS_CONTROL_SPEED:
                motor.controlMode = DJIMotor::SPD_MODE;
                motor.speedSet = g_motor_target_speed;
                break;

            case THREADS_CONTROL_POS:
                motor.controlMode = DJIMotor::POS_MODE;
                motor.positionSet = g_motor_target_position;
                break;

            case THREADS_CONTROL_RELAX:
            default:
                motor.controlMode = DJIMotor::RELAX_MODE;
                break;
            }
        }

        motor.setOutput();
        handler->sendControlData();
        handler->AllMotorAliveCheck();
        tx_semaphore_put(&g_control_heartbeat_sem);
        tx_thread_sleep(1);
    }
}
