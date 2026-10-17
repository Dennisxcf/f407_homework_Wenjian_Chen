#include "bsp_can.hpp"

#include "DJIMotorHandler.hpp" // [AI] 新增：接收回调需要分发给电机处理器

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st;                   ///< 定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位

    can_filter_st.FilterBank = 0;                                      ///< 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                 ///< 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                             ///< 使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断

    can_filter_st.SlaveStartFilterBank = 14;                           ///< 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                     ///< 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                             ///< 使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断
}

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        // 如果邮箱已满，可以选择等待或者采取其他操作
        // 例如，可以添加一个超时机制或者直接返回错误状态
        // Monitor::instance()->Log_Messages(Monitor::WARNING, (uint8_t *)"CAN Tx Mailbox is full\r\n");
        return;
    }
    // TODO: 校验标准帧 ID 和 DLC (0~8)，构造帧头并调用 HAL_CAN_AddTxMessage。
    // ---------------------------------------------------------------------
    // [AI] 以下为实现，均在 TODO 区内
    // ---------------------------------------------------------------------
    if (msg == nullptr) // [AI] msg 为空会在 HAL 内部解引用崩溃
    {
        return;
    }
    if (Id > 0x7FFU) // [AI] 只支持标准帧
    {
        return;
    }
    if (len == 0U || len > 8U) // [AI] 经典 CAN 数据段为 1~8 字节
    {
        return;
    }

    CAN_TxHeaderTypeDef tx_Header = {0}; // [AI] 构造帧头
    tx_Header.StdId = Id;
    tx_Header.ExtId = 0;
    tx_Header.IDE = CAN_ID_STD;
    tx_Header.RTR = CAN_RTR_DATA;
    tx_Header.DLC = len;
    tx_Header.TransmitGlobalTime = DISABLE;

    // [AI] HAL_CAN_AddTxMessage 不看 DLC，会无条件读 aData[0]~aData[7]
    //      （stm32f4xx_hal_can.c:1310 起），所以 len < 8 时必须自己补零
    uint8_t data[8] = {0};
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = msg[i];
    }

    // [AI] 第 4 个参数是输出参数（回传用了哪个邮箱：0x01/0x02/0x04），
    //      必须是有效地址，不能传 nullptr；本项目用不到该值
    uint32_t tx_mailbox = 0U;
    if (HAL_CAN_AddTxMessage(hcan, &tx_Header, data, &tx_mailbox) != HAL_OK)
    {
        // [AI] 失败常见原因是邮箱满或外设未 HAL_CAN_Start，调试时可在此打日志
    }
    // if(Id>0x7FF) return;
    // if(len>8||len==0) return;
    // CAN_TxHeaderTypeDef tx_Header={0};
    // tx_Header.StdId = Id;
    // tx_Header.ExtId = 0;
    // tx_Header.IDE = CAN_ID_STD;
    // tx_Header.RTR = CAN_RTR_DATA;
    // tx_Header.DLC = len;
    // tx_Header.TransmitGlobalTime = DISABLE;
    // uint32_t tx_mailbox = 0U;
    // HAL_CAN_AddTxMessage(hcan, &tx_Header, msg, &tx_mailbox);
}

// TODO: 实现 CAN 接收回调，根据总线和标准帧 ID 分发电机反馈。
// ---------------------------------------------------------------------------
// [AI] 以下为实现
// ---------------------------------------------------------------------------
/**
 * @brief CAN FIFO0 接收回调。
 * @param hcan 触发中断的 CAN 句柄（CAN1 或 CAN2）
 * @note 由 HAL_CAN_IRQHandler 在中断上下文调用，覆盖了
 *       stm32f4xx_hal_can.c 中的 __weak 空实现；
 *       必须用 extern "C" 保持 C 链接，否则 C++ 名字修饰会生成另一个符号，
 *       __weak 空实现仍然生效，回调会静默失效。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // [AI] FIFO 里可能积压多帧，一次中断全部取完，避免漏帧
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        // [AI] 本项目只用标准帧，扩展帧 / 遥控帧跳过
        if (rx_header.IDE != CAN_ID_STD)
        {
            continue;
        }

        // [AI] DJI 电机反馈 ID 为 0x201~0x208，对应索引 0~7，
        //      需与 DJIMotorHandler::registerMotor 的槽位约定一致
        if (rx_header.StdId >= 0x201U && rx_header.StdId <= 0x208U)
        {
            uint16_t index = (uint16_t)(rx_header.StdId - 0x201U);

            // [AI] 中断上下文，updateFeedback 内部不要做耗时运算
            DJIMotorHandler::Instance()->updateFeedback(hcan, rx_data, index);
        }
    }
}
