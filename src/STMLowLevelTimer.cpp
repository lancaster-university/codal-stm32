#include "STMLowLevelTimer.h"
#include "CodalDmesg.h"
#include "pwmout_api.h"

using namespace codal;

STMLowLevelTimer* instances[5] = { 0 };

void timer_irq_handler(uint8_t index)
{
    if (instances[index] == NULL)
        return;

    uint16_t channel_bitmsk = 0;

    TIM_HandleTypeDef* timHandle = &instances[index]->TimHandle;

    if (__HAL_TIM_GET_IT_SOURCE(timHandle, TIM_FLAG_CC1) == SET)
    {
        channel_bitmsk |= (1 << 0);
        __HAL_TIM_CLEAR_IT(timHandle, TIM_IT_CC1);
    }

    if (__HAL_TIM_GET_IT_SOURCE(timHandle, TIM_FLAG_CC2) == SET)
    {
        channel_bitmsk |= (1 << 1);
        __HAL_TIM_CLEAR_IT(timHandle, TIM_IT_CC2);
    }

    if (__HAL_TIM_GET_IT_SOURCE(timHandle, TIM_FLAG_CC3) == SET)
    {
        channel_bitmsk |= (1 << 2);
        __HAL_TIM_CLEAR_IT(timHandle, TIM_IT_CC3);
    }

    if (__HAL_TIM_GET_IT_SOURCE(timHandle, TIM_FLAG_CC4) == SET)
    {
        channel_bitmsk |= (1 << 3);
        __HAL_TIM_CLEAR_IT(timHandle, TIM_IT_CC4);
    }

    instances[index]->timer_pointer(channel_bitmsk);
}

extern "C" void TIM1_IRQHandler()
{
    timer_irq_handler(0);
}
extern "C" void TIM2_IRQHandler()
{
    timer_irq_handler(1);
}
extern "C" void TIM3_IRQHandler()
{
    timer_irq_handler(2);
}
extern "C" void TIM4_IRQHandler()
{
    timer_irq_handler(3);
}
extern "C" void TIM5_IRQHandler()
{
    timer_irq_handler(4);
}

STMLowLevelTimer::STMLowLevelTimer(TIM_TypeDef* timer, uint8_t irqn) : LowLevelTimer(4)
{
    enable_tim_clk((uint32_t)timer);

    this->timer_instance = timer;
    this->irqN = irqn;
    memset(&TimHandle, 0, sizeof(TIM_HandleTypeDef));

    disableIRQ(); // otherwise it might hit us while we're initializing

    DMESG("SYS CLK: %d %d", SystemCoreClock, (uint32_t)((SystemCoreClock / 1000000)));

    TimHandle.Instance = this->timer_instance;
    TimHandle.Init.Period = 0xFFFFFFFF;
    TimHandle.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000)-1);
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_OC_Init(&TimHandle);

    // all timers run in at least 16 bit mode, so lets use it as a default.
    setBitMode(BitMode16);

    uint8_t instance_index = 0;

    if (timer == TIM2)
        instance_index = 1;
    if (timer == TIM3)
        instance_index = 2;
    if (timer == TIM4)
        instance_index = 3;
#ifdef TIM5
    if (timer == TIM5)
        instance_index = 4;
#endif

    instances[instance_index] = this;

    setIRQPriority(2);
}

int STMLowLevelTimer::setIRQPriority(int priority)
{
    NVIC_SetPriority((IRQn_Type)this->irqN, priority);
    return DEVICE_OK;
}

int STMLowLevelTimer::enable()
{
    NVIC_ClearPendingIRQ((IRQn_Type)this->irqN);
    enableIRQ();
    HAL_TIM_OC_Start_IT(&TimHandle, TIM_CHANNEL_1);
    status |= STM_LOW_LEVEL_TIMER_STATUS_ENABLED;
    return DEVICE_OK;
}

int STMLowLevelTimer::enableIRQ()
{
    NVIC_EnableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int STMLowLevelTimer::disable()
{
    disableIRQ();
    HAL_TIM_OC_Stop_IT(&TimHandle, TIM_CHANNEL_1);
    // HAL_TIM_Base_Stop_IT(&TimHandle);
    status &= ~STM_LOW_LEVEL_TIMER_STATUS_ENABLED;
    return DEVICE_OK;
}

int STMLowLevelTimer::disableIRQ()
{
    NVIC_DisableIRQ((IRQn_Type)this->irqN);
    return DEVICE_OK;
}

int STMLowLevelTimer::reset()
{
    disableIRQ();
    __HAL_TIM_SET_COUNTER(&TimHandle, 0);
    enableIRQ();
    return DEVICE_OK;
}

int STMLowLevelTimer::setMode(TimerMode t)
{
    // only support timer mode.
    return DEVICE_OK;
}

int STMLowLevelTimer::setCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    uint32_t hal_channel = TIM_CHANNEL_1;
    uint32_t hal_int = TIM_IT_CC1;

    if (channel == 1)
    {
        hal_channel = TIM_CHANNEL_2;
        hal_int = TIM_IT_CC2;
    }

    if (channel == 2)
    {
        hal_channel = TIM_CHANNEL_3;
        hal_int = TIM_IT_CC3;
    }

    if (channel == 3)
    {
        hal_channel = TIM_CHANNEL_4;
        hal_int = TIM_IT_CC4;
    }

    __HAL_TIM_DISABLE_IT(&TimHandle, hal_int);
    __HAL_TIM_CLEAR_IT(&TimHandle, hal_int);
    __HAL_TIM_SET_COMPARE(&TimHandle, hal_channel, value);
    __HAL_TIM_ENABLE_IT(&TimHandle, hal_int);

    return DEVICE_OK;
}

int STMLowLevelTimer::offsetCompare(uint8_t channel, uint32_t value)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    uint32_t hal_channel = TIM_CHANNEL_1;
    uint32_t hal_int = TIM_IT_CC1;

    if (channel == 1)
    {
        hal_channel = TIM_CHANNEL_2;
        hal_int = TIM_IT_CC2;
    }

    if (channel == 2)
    {
        hal_channel = TIM_CHANNEL_3;
        hal_int = TIM_IT_CC3;
    }

    if (channel == 3)
    {
        hal_channel = TIM_CHANNEL_4;
        hal_int = TIM_IT_CC4;
    }

    __HAL_TIM_DISABLE_IT(&TimHandle, hal_int);

    __HAL_TIM_SET_COMPARE(&TimHandle, hal_channel, (uint32_t)(__HAL_TIM_GET_COMPARE(&TimHandle, hal_channel) + value));
    __HAL_TIM_ENABLE_IT(&TimHandle, hal_int);

    return DEVICE_OK;
}

int STMLowLevelTimer::clearCompare(uint8_t channel)
{
    if (channel > getChannelCount())
        return DEVICE_INVALID_PARAMETER;

    uint32_t hal_int = TIM_IT_CC1;

    if (channel == 1)
        hal_int = TIM_IT_CC2;

    if (channel == 2)
        hal_int = TIM_IT_CC3;

    if (channel == 3)
        hal_int = TIM_IT_CC4;

    __HAL_TIM_DISABLE_IT(&TimHandle, hal_int);

    return DEVICE_OK;
}

uint32_t STMLowLevelTimer::captureCounter()
{
    return __HAL_TIM_GET_COUNTER(&TimHandle);
}

int STMLowLevelTimer::setClockSpeed(uint32_t speedKHz)
{
    // // 8000 khz
    // // TODO: Reconfigure clocks if resolution is greater than 8khz
    // if (speedKHz > 8000)
    //     return DEVICE_INVALID_PARAMETER;

    // // clock is 8khz
    // uint32_t clockSpeed = 8000;
    // uint8_t prescaleValue = 0;

    // // snap to the lowest
    // for (prescaleValue = 0; prescaleValue < PRESCALE_VALUE_MAX; prescaleValue++)
    // {
    //     if (speedKHz < (clockSpeed / prescalerDivison[prescaleValue]))
    //         continue;

    //     break;
    // }

    // tc->COUNT8.CTRLA.bit.PRESCALER = prescaleValue;

    return DEVICE_OK;
}

int STMLowLevelTimer::setBitMode(TimerBitMode t)
{
    // disable();
    // if (HAL_TIM_OC_DeInit(&TimHandle) != HAL_OK)
    //     CODAL_ASSERT(0);

    // switch (t)
    // {
    //     case BitMode8:
    //         TimHandle.Init.Period = 0xFF;
    //         break;
    //     case BitMode16:
    //         TimHandle.Init.Period = 0xFFFF;
    //         break;
    //     case BitMode24:
    //         TimHandle.Init.Period = 0xFFFFFF;
    //         break;
    //     case BitMode32:
    //         TimHandle.Init.Period = 0xFFFFFFFF;
    //         break;
    // }

    // // TimHandle.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1);
    // // TimHandle.Init.ClockDivision = 0;
    // // TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    // if (HAL_TIM_OC_Init(&TimHandle) != HAL_OK)
    //     CODAL_ASSERT(0);

    // enable();

    this->bitMode = t;
    return DEVICE_OK;
}