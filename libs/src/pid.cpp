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
    //位置式
    pResult = iResult = dResult = result = 0.0f;
    pResult  = kp * err[0];
    iResult += ki * err[0];
    iResult  = Numeric::LimitABS(iResult, maxIOut);
    dResult  = kd * (err[0] - err[1]);
    result   = Numeric::LimitABS(pResult + iResult + dResult, maxOut);
    //增量式
    float delta = kp * (err[0] - err[1]) + ki * err[0] + kd * (err[0] - 2.0f * err[1] + err[2]);
    result = Numeric::LimitABS(result + delta, maxOut);
    //检查
    // 只有当"加了这次积分后输出还没饱和"，才真正接受这次积分累加
    float i_next = Numeric::LimitABS(iResult + ki * err[0], maxIOut);
    float u_next = pResult + i_next + dResult;
    if (Numeric::LimitABS(u_next, maxOut) == u_next || (u_next * err[0] < 0.0f))
        iResult = i_next;
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
