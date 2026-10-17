//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

void PWM_Init(void)
{
    // TODO: 根据实际使用的定时器和通道完成 PWM 初始化。
    // [AI] 定时器本身由 CubeMX 的 MX_TIM1_Init() 初始化（main.c 中调用），
    //      当前板级配置只有 TIM1_CH1（PE9，舵机那路）。
    //      这里启动通道并把占空比清 0，避免上电瞬间舵机乱动；
    //      换了通道记得改 TIM_CHANNEL_1。
    PWM_SetDutyRatio(&htim1, 0.0f, TIM_CHANNEL_1);
    PWM_Start(&htim1, TIM_CHANNEL_1);
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 启动指定定时器通道的 PWM，并检查返回状态。
    // [AI] TIM1/TIM8 这类高级定时器的主输出（MOE）由 HAL_TIM_PWM_Start 内部使能，
    //      不需要自己调 __HAL_TIM_MOE_ENABLE。
    if (htim == nullptr)
    {
        return;
    }

    HAL_TIM_PWM_Start(htim, Channel);
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 停止指定定时器通道的 PWM。
    if (htim == nullptr)
    {
        return;
    }

    HAL_TIM_PWM_Stop(htim, Channel);
}

// ---------------------------------------------------------------------------
// [AI] 新增的静态工具：取得定时器的计数时钟频率（Hz）。
//      STM32F4 上 TIM1/8/9/10/11 挂 APB2，其余挂 APB1；
//      APB 预分频系数不为 1 时，定时器时钟 = PCLK × 2。
// ---------------------------------------------------------------------------
static uint32_t TIM_GetTimerClock(TIM_TypeDef *Instance)
{
    uint32_t pclk;
    uint32_t ppre;

    if (Instance == TIM1 || Instance == TIM8 || Instance == TIM9 ||
        Instance == TIM10 || Instance == TIM11)
    {
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    }
    else
    {
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    }

    return (ppre == 0U) ? pclk : (pclk * 2U);
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    // TODO: 根据定时器时钟和预分频值，将秒转换为 ARR 并更新周期。
    // [AI] 计数频率 = 定时器时钟 / (PSC + 1)，周期 = (ARR + 1) / 计数频率
    //      => ARR = 定时器时钟 × 周期 / (PSC + 1) - 1
    //      例：TIM1 时钟 168MHz、PSC = 191（tim.c 里的 192-1）时，
    //      20ms 周期（50Hz 舵机）对应 ARR = 17499。
    if (htim == nullptr || period_s <= 0.0f)
    {
        return;
    }

    uint32_t timer_clk = TIM_GetTimerClock(htim->Instance);
    uint32_t psc = htim->Init.Prescaler + 1U;

    float arr_f = (float)timer_clk * period_s / (float)psc - 1.0f;

    // [AI] 限幅：ARR 至少为 1，且不超过 16 位定时器的上限
    if (arr_f < 1.0f)
    {
        arr_f = 1.0f;
    }
    uint32_t arr = (uint32_t)(arr_f + 0.5f);
    if (arr > 0xFFFFU)
    {
        arr = 0xFFFFU;
    }

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    htim->Init.Period = arr; // [AI] 保持句柄记录与实际寄存器一致

    // [AI] 提醒：tim.c 里 AutoReloadPreload = DISABLE，运行中改 ARR 会立即生效，
    //      可能出现一个不完整的周期；需要平滑改周期请在 CubeMX 里打开 ARR 预装载。
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    // TODO: 将 [0, 1] 占空比换算为比较值，写入指定通道。
    // [AI] 一个周期的计数个数是 ARR + 1，CCR 取 0 ~ ARR+1 表示 0% ~ 100%
    if (htim == nullptr)
    {
        return;
    }

    if (dutyratio < 0.0f)
    {
        dutyratio = 0.0f;
    }
    if (dutyratio > 1.0f)
    {
        dutyratio = 1.0f;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    uint32_t ccr = (uint32_t)(dutyratio * (float)period + 0.5f);
    if (ccr > period)
    {
        ccr = period;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}
