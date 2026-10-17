#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include "spi.h" // [AI] BMI088_SPI_HANDLE(hspi1) 的定义在 spi.h 里
#include <cmath>

/* ===========================================================================
 * [AI] 本文件的 TODO 由 agent 补全，均用 [AI] 标注；TODO 区以外的原有行未改。
 *      说明：ReadReg 采用"原样透传"的语义——SPI 收到几个字节就交给调用者几个，
 *      加速度计读数据时那个 dummy 字节由调用者自己跳过（与已有的
 *      VerifyAccChipID() 里"读 2 字节取 pRxData[1]"的用法保持一致）。
 * =========================================================================== */

namespace BMI088
{
    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        // TODO: 温度读取完成后，用 TempPid 计算加热占空比并限制到 [0, 1]。
        // 未完成前保持加热关闭。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        float current_temp = 0.0f;
        ReadAccTemperature(&current_temp);

        // [AI] 温度环：ref 是目标温度，fdb 是当前温度
        TempPid.ref = target_temp;
        TempPid.fdb = current_temp;
        TempPid.UpdateResult();

        // [AI] TempPid.maxOut 是 CCR 量级(25000，对应 TIM10 的 ARR)，换算成 0~1 的占空比
        float duty = 0.0f;
        if (TempPid.maxOut > 0.0f)
        {
            duty = TempPid.result / TempPid.maxOut;
        }
        if (duty < 0.0f)
        {
            duty = 0.0f;
        }
        else if (duty > 1.0f)
        {
            duty = 1.0f;
        }

        // [AI] 注意：加热通道要先用 PWM_Start(&htim10, TIM_CHANNEL_1) 启动一次
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty, HEATING_RESISTANCE_CHANNEL);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData[2]; //< 读取两个字节，第一个是需要丢弃的无效字节，第二个是芯片 ID

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData[1] != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData[1] == ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData;                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyAccData()
    {
        // TODO: 检查加速度数据是否有效，并更新 ACC_DATA_ERR。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        // [AI] 思路：读加速度计的 ERR_REG(0x02)，非 0 说明芯片内部报错。
        //      加速度计读数据要多收 1 个 dummy 字节，所以读 2 字节取 buf[1]。
        uint8_t buf[2] = {0};
        ReadReg(BMI088_CS_ACC, ACC_ERR_REG_ADDR, buf, sizeof(buf));
        self_test.ACC_DATA_ERR = (buf[1] != 0U);

        // [AI] 想要更严的话，可以再对数值做合理性检查：静止时 |a| 应接近 9.8 m/s²。
    }

    void cBMI088::VerifyGyroData()
    {
        // TODO: 检查角速度数据是否有效，并更新 GYRO_DATA_ERR。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        // [AI] 思路：GYRO_INT_STAT_1(0x0A) 的 bit7 是"数据就绪"标志，
        //      为 0 说明陀螺仪没有在出数据（SPI 不通或还没配置好）。
        uint8_t stat = 0;
        ReadReg(BMI088_CS_GYRO, GYRO_INT_STAT_1_ADDR, &stat, 1);
        self_test.GYRO_DATA_ERR = ((stat & GYRO_DRDY_ON) == 0U);
    }

    // [AI] 片选辅助：先两片都拉高，再拉低目标片（SPI 总线共用，另一片必须保持高电平）
    static void CS_Select(enum BMI088_SENSOR cs)
    {
        HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);

        if (cs == BMI088_CS_ACC)
        {
            HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET);
        }
    }

    // [AI] 释放片选
    static void CS_Release(void)
    {
        HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);
    }

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        // TODO: 根据 cs 拉低对应 GPIO 片选；按 SPI 写协议发送地址和数据，
        // 最后释放片选，并处理通信失败与必要的延时。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        if (data == nullptr || len == 0U)
        {
            return;
        }

        CS_Select(cs);

        // [AI] 写协议：地址清 bit7（0x7F）表示写，后面直接跟数据，没有 dummy 字节
        uint8_t tx_addr = (uint8_t)(addr & BMI088_SPI_WRITE_CODE);
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &tx_addr, 1, BMI088_SPI_TIMEOUT_MS);
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, data, len, BMI088_SPI_TIMEOUT_MS);

        CS_Release();
        // [AI] 写寄存器后芯片需要建立时间，具体延时由调用者（Config 里的 tx_thread_sleep）负责
    }

    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        // TODO: 根据 cs 拉低对应 GPIO 片选；按 SPI 读协议发送地址，
        // 丢弃加速度计返回的首个无效字节，读取 len 字节并释放片选。
        // 未实现前清零缓冲区，避免芯片 ID 校验读取未初始化数据。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        if (data == nullptr)
        {
            return;
        }
        if (len == 0U)
        {
            return;
        }

        CS_Select(cs);

        // [AI] 读协议：地址置 bit7（0x80）表示读，然后连续读 len 字节。
        //      dummy 字节问题：加速度计读数据时首位是 dummy，陀螺仪没有；
        //      这里按"原样透传"处理，由调用者决定是否跳过（见文件头的说明）。
        uint8_t tx_addr = (uint8_t)(addr | BMI088_SPI_READ_CODE);
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &tx_addr, 1, BMI088_SPI_TIMEOUT_MS);
        HAL_SPI_Receive(&BMI088_SPI_HANDLE, data, len, BMI088_SPI_TIMEOUT_MS);

        CS_Release();
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(150);

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        // TODO: 按 BMI088 加速度计 SPI 协议丢弃首个无效字节，拼接三轴有符号原始值，
        // 再按配置的量程换算为 m/s²，写入 data。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        if (data == nullptr)
        {
            return;
        }

        // [AI] 读 1 个 dummy + 6 个数据字节；buf[1..6] = X_LSB,X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB
        uint8_t buf[ACC_XYZ_LEN + 1] = {0};
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, sizeof(buf));

        const int16_t raw_x = (int16_t)(((uint16_t)buf[2] << 8) | buf[1]);
        const int16_t raw_y = (int16_t)(((uint16_t)buf[4] << 8) | buf[3]);
        const int16_t raw_z = (int16_t)(((uint16_t)buf[6] << 8) | buf[5]);

        // [AI] Config() 里设的是 6G 量程，IMU_ACCEL_6G_SEN 已含 9.8 → 直接得到 m/s²
        //      如果你改了量程，这里的系数必须跟着换（否则数据会差固定倍数）
        data->x = (float)raw_x * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_X;
        data->y = (float)raw_y * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_Y;
        data->z = (float)raw_z * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_Z;

        // [AI] 顺便把温度也填上，IMU 线程可以直接用整帧数据
        ReadAccTemperature(&data->temperature);
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        // TODO: 读取并拼接陀螺仪三轴原始值，按配置量程换算为 rad/s，
        // 减去 Gyro_offset 后写入 data。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        if (data == nullptr)
        {
            return;
        }

        // [AI] 陀螺仪读没有 dummy 字节，直接 6 字节：X_LSB,X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB
        uint8_t buf[GYRO_XYZ_LEN] = {0};
        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, sizeof(buf));

        const int16_t raw_x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        const int16_t raw_y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        const int16_t raw_z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

        // [AI] Config() 里设的是 2000deg/s 量程，IMU_GYRO_2000_SEN 直接给出 rad/s
        //      Gyro_offset 是 Calibrate() 算出来的零偏（rad/s）
        data->x = (float)raw_x * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = (float)raw_y * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = (float)raw_z * IMU_GYRO_2000_SEN - Gyro_offset[2];
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        // TODO: 丢弃加速度计读取时的首个无效字节，解析 11 位有符号温度并换算为摄氏度。
        // -----------------------------------------------------------------------
        // [AI] 以下为实现
        // -----------------------------------------------------------------------
        if (temp == nullptr)
        {
            return;
        }

        // [AI] buf[0] 是 dummy，buf[1]=TEMP_MSB(0x22) buf[2]=TEMP_LSB(0x23)
        uint8_t buf[TEMP_LEN + 1] = {0};
        ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, buf, sizeof(buf));

        // [AI] 温度是 11 位有符号数（高 11 位），先拼成 16 位再算术右移 5 位完成符号扩展
        const int16_t raw = (int16_t)(((int16_t)buf[1] << 8) | buf[2]) >> 5;

        // [AI] 换算：0.125 ℃/LSB，零点是 23 ℃
        *temp = (float)raw * TEMP_UNIT + TEMP_BIAS;
    }

}
