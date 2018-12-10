#include "ZTimer.h"
#include "CodalCompat.h"
#include "CodalConfig.h"
#include "codal_target_hal.h"

#include "CodalDmesg.h"

#ifdef STM32F1
#define CMP_T uint16_t
#else
#define CMP_T uint32_t
#endif

namespace codal
{

ZTimer *ZTimer::instance;

ZTimer::ZTimer() : codal::Timer()
{
    memset(&TimHandle, 0, sizeof(TimHandle));
    instance = this;
    this->prev = 0;
    init();
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
    __HAL_RCC_TIM5_CLK_ENABLE();

    TimHandle.Instance = TIM5;

    TimHandle.Init.Period = (CMP_T)0xFFFFFFFF;
    TimHandle.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1);
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    if (HAL_TIM_OC_Init(&TimHandle) != HAL_OK)
        CODAL_ASSERT(0);

    NVIC_SetPriority(TIM5_IRQn, 1);
    NVIC_EnableIRQ(TIM5_IRQn);
    HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1);

    this->prev = __HAL_TIM_GET_COUNTER(&TimHandle);
}

void ZTimer::triggerIn(CODAL_TIMESTAMP t)
{
    if (t < 20)
        t = 20;

    target_disable_irq();
    __HAL_TIM_DISABLE_IT(&TimHandle, TIM_IT_CC1);
    __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_1,
                          (CMP_T)(__HAL_TIM_GET_COUNTER(&TimHandle) + t));
    __HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_CC1);
    target_enable_irq();
}
extern "C" uint32_t uwTick;
void ZTimer::syncRequest()
{
    target_disable_irq();
    CMP_T curr = __HAL_TIM_GET_COUNTER(&TimHandle);
    CMP_T delta = curr - this->prev;

    // update the hal... - this won't work if this is called more than once per millisecond
    uwTick += delta / 1000;

    this->prev = curr;
    this->sync(delta);
    target_enable_irq();
}

extern "C" void wait_us(uint32_t us)
{
    auto end = ZTimer::instance->getTimeUs() + us;

    while (ZTimer::instance->getTimeUs() < end)
    {
        // busy wait
    }
}

} // namespace codal
