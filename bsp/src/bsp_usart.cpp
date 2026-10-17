#include "bsp_usart.hpp"

void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART6_Init()
{
  // TODO: 配置 USART6 的 DMA 接收、发送及空闲中断，并启动接收。
  // [AI] 目前 board/Core/Src/usart.c 里 CubeMX 只生成了 UART4（huart4），没有 huart6，
  //      直接引用会链接失败，所以这里先留空。
  //      在 CubeMX 里打开 USART6 + 对应 DMA 后，在这里加一行即可：
  //        USART_Receive(&huart6, ref_rx_buf, sizeof(ref_rx_buf));  // buf 必须是全局/静态数组
};

void USART1_Init()
{
  // TODO: 配置 USART1 的 DMA 接收、发送及空闲中断，并启动接收。
  // [AI] 同上：当前没有 huart1，CubeMX 配好 USART1 + DMA 后再在这里挂接收。
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  // TODO: 检查参数，并按 mode 选择阻塞、DMA 或中断发送。
  // ---------------------------------------------------------------------------
  // [AI] 以下为实现
  // ---------------------------------------------------------------------------
  if (huart == nullptr || pData == nullptr || Size == 0U)
  {
    return;
  }

  switch (mode)
  {
  case USART_MODE_BLOCK:
    // [AI] 用有限超时，不用 HAL_MAX_DELAY，避免串口异常时卡死调用线程
    HAL_UART_Transmit(huart, pData, Size, 100U);
    break;

  case USART_MODE_DMA:
    // [AI] 没配 DMA 时降级为阻塞发送，而不是静默失败
    if (huart->hdmatx == nullptr)
    {
      HAL_UART_Transmit(huart, pData, Size, 100U);
      break;
    }
    // [AI] DMA 发送期间 pData 必须保持有效，且同一串口不要并发调用
    HAL_UART_Transmit_DMA(huart, pData, Size);
    break;

  case USART_MODE_IT:
    HAL_UART_Transmit_IT(huart, pData, Size);
    break;

  default:
    break;
  }
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  // TODO: 选择接收方式并启动 HAL UART 接收。
  // ---------------------------------------------------------------------------
  // [AI] 以下为实现
  // ---------------------------------------------------------------------------
  if (huart == nullptr || pData == nullptr || Size == 0U)
  {
    return;
  }

  if (huart->hdmarx != nullptr)
  {
    // [AI] DMA + 空闲中断：一帧收完进入 HAL_UARTEx_RxEventCallback，
    //      回调参数 Size 是本帧实际长度，适合裁判系统这种变长帧
    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);

    // [AI] 关掉半传输中断，否则每收一半就触发一次回调
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else
  {
    // [AI] 没有 DMA 时用中断接收，收满 Size 个字节才触发 HAL_UART_RxCpltCallback
    HAL_UART_Receive_IT(huart, pData, Size);
  }
}

// ---------------------------------------------------------------------------
// [AI] 接收完成后的处理钩子（按需实现，都在中断上下文，不要做耗时运算）：
//   DMA + 空闲中断：void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//   中断接收：      void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//   两者都是 HAL 的 __weak 函数，C++ 里实现时必须写成 extern "C"。
// ---------------------------------------------------------------------------
