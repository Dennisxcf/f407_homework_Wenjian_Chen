#include "DJIMotorHandler.hpp"

#include "bsp_can.hpp" // [AI] sendControlData 需要用 CAN_Transmit 发送控制帧

/* ===========================================================================
 * [AI] 本文件的 TODO 由 agent 补全，均用 [AI] 标注；TODO 区以外的原有行未改。
 * =========================================================================== */

namespace
{
// [AI] 单位换算常量
constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kEncoderResolution = 8192.0f; // 编码器取值 0~8191

// [AI] 各减速箱的减速比（电机轴 : 输出轴）
float GearRatioOf(GearBox gearBox)
{
    switch (gearBox)
    {
    case GearBox_M2006:
        return 36.0f;
    case GearBox_M3508:
        return 19.0f;
    case GearBox_XRoll:
        return 1.0f; // TODO: 换成 XRoll 的实际减速比
    case GearBox_None:
    default:
        return 1.0f;
    }
}

// [AI] 把 4 个电机的 currentSet 按大端序打包进 8 字节控制帧。
//      DJI 电机协议：ID 0x200 → 0x201~0x204，ID 0x1FF → 0x205~0x208，
//      每个电机占 2 字节，高字节在前。
void PackCurrentFrame(DJIMotor *const *motors, uint8_t *frame)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        const int16_t current = (motors[i] != nullptr) ? motors[i]->currentSet : 0;
        frame[2 * i] = (uint8_t)((current >> 8) & 0xFF);
        frame[2 * i + 1] = (uint8_t)(current & 0xFF);
    }
}
} // namespace

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    // TODO: 校验 CAN 句柄和 0x201~0x208 ID，再登记到对应总线与槽位。
    // TODO: 按 ID 设置对应的 0x200/0x1FF 报文存在标志。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    if (motor == nullptr || hcan == nullptr)
    {
        return;
    }

    // [AI] 电机反馈 ID 只可能是 0x201~0x208
    if (canId < 0x201U || canId > 0x208U)
    {
        return;
    }

    // [AI] 只接受本工程的两个 CAN 总线句柄
    uint8_t bus = 0U;
    if (hcan == &hcan1)
    {
        bus = 0U;
    }
    else if (hcan == &hcan2)
    {
        bus = 1U;
    }
    else
    {
        return;
    }

    // [AI] 槽位 = ID - 0x201，与接收回调里的 index 约定一致
    const uint8_t index = (uint8_t)(canId - 0x201U);

    DJIMotorList[bus][index] = motor;
    motor->canId = canId;
    motor->hcan = hcan;

    // [AI] 0x201~0x204 归 0x200 报文，0x205~0x208 归 0x1FF 报文
    if (index < 4U)
    {
        if (bus == 0U)
        {
            CAN1_0x200_Exist = true;
        }
        else
        {
            CAN2_0x200_Exist = true;
        }
    }
    else
    {
        if (bus == 0U)
        {
            CAN1_0x1FF_Exist = true;
        }
        else
        {
            CAN2_0x1FF_Exist = true;
        }
    }
}

void DJIMotorHandler::sendControlData()
{
    // TODO: 将各电机 currentSet 按大端序放入正确的 8 字节控制帧。
    // TODO: 仅发送包含已注册电机的报文；未完成前不发送 CAN 电流命令。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------

    // [AI] CAN1：0x201~0x204 → 0x200，0x205~0x208 → 0x1FF
    if (CAN1_0x200_Exist)
    {
        PackCurrentFrame(&DJIMotorList[0][0], can1_send_data_0);
        CAN_Transmit(&hcan1, 0x200U, can1_send_data_0, 8U);
    }
    if (CAN1_0x1FF_Exist)
    {
        PackCurrentFrame(&DJIMotorList[0][4], can1_send_data_1);
        CAN_Transmit(&hcan1, 0x1FFU, can1_send_data_1, 8U);
    }

    // [AI] CAN2 同理
    if (CAN2_0x200_Exist)
    {
        PackCurrentFrame(&DJIMotorList[1][0], can2_send_data_0);
        CAN_Transmit(&hcan2, 0x200U, can2_send_data_0, 8U);
    }
    if (CAN2_0x1FF_Exist)
    {
        PackCurrentFrame(&DJIMotorList[1][4], can2_send_data_1);
        CAN_Transmit(&hcan2, 0x1FFU, can2_send_data_1, 8U);
    }
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    // TODO: 校验总线、反馈数据和索引，再找到对应电机并调用 UpdateSensorData。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    if (hcan == nullptr || rx_data == nullptr)
    {
        return;
    }
    if (index < 0 || index >= 8)
    {
        return;
    }

    // [AI] 由句柄判断总线：CAN1 → 0，CAN2 → 1
    uint8_t bus = 0U;
    if (hcan == &hcan1)
    {
        bus = 0U;
    }
    else if (hcan == &hcan2)
    {
        bus = 1U;
    }
    else
    {
        return;
    }

    DJIMotor *motor = DJIMotorList[bus][(uint8_t)index];
    if (motor == nullptr)
    {
        return; // [AI] 该槽位没有注册电机，直接丢弃
    }

    UpdateSensorData(motor, rx_data);
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    // TODO: 从 CAN 数据解析编码器、转速、电流和温度。
    // TODO: 处理编码器回绕，并按减速比换算输出轴位置和角速度。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    if (motor == nullptr || can_data == nullptr)
    {
        return;
    }

    // [AI] DJI 电机反馈帧（8 字节）：
    //      data[0..1] 编码器(0~8191, 高字节在前) data[2..3] 转速 rpm
    //      data[4..5] 实际电流  data[6] 温度(℃)  data[7] 保留
    const uint16_t ecd = (uint16_t)(((uint16_t)can_data[0] << 8) | can_data[1]);
    const int16_t speed_rpm = (int16_t)(((uint16_t)can_data[2] << 8) | can_data[3]);
    const int16_t current_raw = (int16_t)(((uint16_t)can_data[4] << 8) | can_data[5]);
    const uint8_t temperature = can_data[6];

    DJIMotor::MotorFeedBack &fb = motor->motorFeedback;

    // [AI] 第一帧只用来建立编码器基准，不让位置从 0 跳到当前位置
    const bool first_frame = (motor->AliveFlag == 0U);

    // [AI] 编码器回绕：差值超过半圈就认为跨过了 0/8191，补一个整圈
    int32_t delta = (int32_t)ecd - (int32_t)fb.last_ecd;
    if (delta > (int32_t)(kEncoderResolution / 2.0f))
    {
        delta -= (int32_t)kEncoderResolution;
    }
    else if (delta < (int32_t)(-kEncoderResolution / 2.0f))
    {
        delta += (int32_t)kEncoderResolution;
    }

    fb.last_ecd = (int16_t)ecd;
    fb.ecd = ecd;

    const float ratio = GearRatioOf(motor->gearBox);

    // [AI] 位置：电机轴角度增量 → 输出轴角度增量，并折算到 [-π, π]
    fb.lastPositionFdb = fb.positionFdb;
    if (!first_frame)
    {
        fb.positionFdb += (float)delta * (kTwoPi / kEncoderResolution) / ratio;
    }
    if (fb.positionFdb > kPi)
    {
        fb.positionFdb -= kTwoPi;
    }
    else if (fb.positionFdb < -kPi)
    {
        fb.positionFdb += kTwoPi;
    }

    // [AI] 速度：电机轴 rpm → 输出轴 rad/s
    fb.lastSpeedFdb = fb.speedFdb;
    fb.speed_rpm = speed_rpm;
    fb.speedFdb = (float)speed_rpm * (kTwoPi / 60.0f) / ratio;

    // [AI] 电流换算：C610/±10000 对应 ±10A；若换 C620(±16384↔±20A) 改成 20.0/16384
    fb.currentFdb = (float)current_raw * (10.0f / 10000.0f);

    fb.temperatureFdb = (float)temperature;

    // [AI] 收到反馈就算"存活"一次，供 AliveCheck 判断在线状态
    motor->AliveFlag++;
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    // TODO: 遍历已注册电机并调用 AliveCheck。
    // -----------------------------------------------------------------------
    // [AI] 以下为实现
    // -----------------------------------------------------------------------
    for (uint8_t bus = 0; bus < 2; bus++)
    {
        for (uint8_t index = 0; index < 8; index++)
        {
            DJIMotor *motor = DJIMotorList[bus][index];
            if (motor != nullptr)
            {
                motor->AliveCheck();
            }
        }
    }
}
