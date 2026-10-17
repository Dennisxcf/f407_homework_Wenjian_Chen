#include "M2006.hpp"

/* ===========================================================================
 * [AI] 本文件的 TODO 由 agent 补全，均用 [AI] 标注；TODO 区以外的原有行未改。
 * =========================================================================== */

M2006::M2006()
{
    // TODO: 根据 M2006 手册设置减速比、允许电流和速度/位置环 PID 参数。
    // 基类初始化为松开模式、零电流；完成配置前不允许驱动电机。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    gearBox = GearBox_M2006; // [AI] M2006 减速箱 36:1

    // [AI] M2006 配 C610 电调：CAN 电流指令 ±10000 对应 ±10A
    maxCurrent = 10000; //< 单位是 CAN 报文里的电流值，不是安培

    // [AI] 速度环：输入是输出轴角速度(rad/s)，输出是电流（CAN 单位）
    //      下面的增益只是保守的起始值，必须上车实测调（先 Kp，再加 Ki，最后 Kd）
    speedPid.mode = PID_POSITION;
    speedPid.Tuning(3000.0f, 30.0f, 0.0f);
    speedPid.maxOut = (float)maxCurrent;
    speedPid.maxIOut = (float)maxCurrent * 0.5f;
    speedPid.Clear();

    // [AI] 位置环：输入是输出轴角度(rad, [-π, π])，输出是速度环的目标角速度(rad/s)
    //      maxOut 是允许的最大转速，M2006 输出轴空载约 1.5 rad/s，这里给 3 rad/s 余量
    positionPid.mode = PID_POSITION;
    positionPid.Tuning(10.0f, 0.0f, 0.0f);
    positionPid.maxOut = 3.0f;
    positionPid.maxIOut = 1.0f;
    positionPid.Clear();

    currentSet = 0;
    MotorState = MOTOR_OFFLINE;
}

void M2006::setOutput()
{
    // TODO: 分别实现松开、速度和位置模式；位置模式需要串联位置环与速度环。
    // TODO: 将 PID 输出限制在 maxCurrent 范围内后写入 currentSet。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    switch (controlMode)
    {
    case RELAX_MODE:
        // [AI] 松开模式：输出 0，同时清掉两个环的历史状态，防止切回来时暴冲
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        return;

    case SPD_MODE:
        // [AI] 单速度环
        speedPid.ref = speedSet;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)speedPid.result;
        break;

    case POS_MODE:
    {
        // [AI] 位置环 + 速度环串联：位置环的输出作为速度环的目标
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();

        speedPid.ref = positionPid.result;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)speedPid.result;
        break;
    }

    default:
        currentSet = 0;
        return;
    }

    // [AI] 双重保险：PID 内部已按 maxOut 限幅，这里再按 maxCurrent 兜一次
    if (currentSet > (int16_t)maxCurrent)
    {
        currentSet = (int16_t)maxCurrent;
    }
    else if (currentSet < -(int16_t)maxCurrent)
    {
        currentSet = -(int16_t)maxCurrent;
    }
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // TODO: 比较 AliveFlag 与 Pre_AliveFlag，更新在线状态并保存本次计数。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    // [AI] AliveFlag 由反馈解析（UpdateSensorData）累加；两次检查之间没变说明掉线
    if (AliveFlag == Pre_AliveFlag)
    {
        MotorState = MOTOR_OFFLINE;
    }
    else
    {
        MotorState = MOTOR_ONLINE;
    }

    Pre_AliveFlag = AliveFlag;
    return MotorState;
}
