#include "ZTimer.h"
#include "CodalCompat.h"
#include "CodalConfig.h"
#include "codal_target_hal.h"

#include "CodalDmesg.h"

namespace codal
{

void ZTimer::callback(k_timer *tm)
{
    auto t = (ZTimer *)k_timer_user_data_get(tm);
    t->trigger();
}

ZTimer::ZTimer() : codal::Timer()
{
    memset(&TimHandle, 0, sizeof(TimHandle));
    instance = this;
    this->prev = 0;
}

extern "C" void TIM5_IRQHandler()
{
    auto h = &ZTimer::instance->TimHandle;
    if (__HAL_TIM_GET_FLAG(h, TIM_FLAG_CC1) == SET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(h, TIM_IT_CC1) == SET)
        {
            __HAL_TIM_CLEAR_IT(h, TIM_IT_CC1);
            __HAL_TIM_CLEAR_FLAG(h, TIM_FLAG_CC1);
            ZTimer::instance->trigger();
        }
    }
}

void ZTimer::init()
{
    TIM_OC_InitTypeDef sConfig;
    TimHandle.Instance = TIM5;

    TimHandle.Init.Period = 0xFFFFFFFF;
    TimHandle.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1);
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    if (HAL_TIM_OC_Init(&TimHandle) != HAL_OK)
        oops();

    NVIC_EnableIRQ(TIM5_IRQn);
    HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1);

    this->prev = __HAL_TIM_GET_COUNTER(&TimHandle);
}

void ZTimer::triggerIn(CODAL_TIMESTAMP t)
{
    this->syncRequest();
    __HAL_TIM_DISABLE_IT(&TimHandle, TIM_IT_CC1);
    __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_1, (uint32_t)(this->prev + t));
    __HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_CC1);
}

void ZTimer::syncRequest()
{
    int key = irq_lock();
    u32_t curr = __HAL_TIM_GET_COUNTER(&TimHandle);
    u32_t delta = curr - this->prev;
    this->prev = curr;
    this->sync(delta);
    irq_unlock(key);
}
} // namespace codal
