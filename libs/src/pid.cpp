#include "pid.hpp"

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    // TODO: 按 mode 实现位置式或增量式 PID，并限制积分和最终输出。
    // 完成前始终保持零输出。
    switch (mode)
    {
        case PID_POSITION:
        {
            pResult  = kp * err[0];
            dResult  = kd * (err[0] - err[1]);

            // 抗饱和：只有"加上本次积分后不饱和"（或误差已反向、正在退饱和）才真的累加
            float i_next = Numeric::LimitABS(iResult + ki * err[0], maxIOut);
            float u_next = pResult + i_next + dResult;
            if ((u_next - Numeric::LimitABS(u_next, maxOut)) * err[0] <= 0.0f)
                iResult = i_next;

            result = Numeric::LimitABS(pResult + iResult + dResult, maxOut);
            break;
        }
        case PID_DELTA:
        {
            float delta = kp * (err[0] - err[1])
                        + ki * err[0]
                        + kd * (err[0] - 2.0f * err[1] + err[2]);
            result = Numeric::LimitABS(result + delta, maxOut);   // result 必须跨周期累积
            // pResult/iResult/dResult 只做观察用，不再参与 result 计算
            pResult = kp * err[0];
            iResult = Numeric::LimitABS(iResult + ki * err[0], maxIOut);
            dResult = kd * (err[0] - err[1]);
            break;
        }
        default:
            result = 0.0f;
            break;
    }
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
