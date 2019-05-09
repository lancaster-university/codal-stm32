#include "LowLevelTimer.h"

#ifndef STM_LOW_LEVEL_TIMER_H
#define STM_LOW_LEVEL_TIMER_H

#define STM_LOW_LEVEL_TIMER_STATUS_ENABLED     0x02

namespace codal
{

class STMLowLevelTimer : public LowLevelTimer
{
    uint8_t irqN;
    TIM_TypeDef* timer_instance;
    public:

    TIM_HandleTypeDef TimHandle;

    STMLowLevelTimer(TIM_TypeDef* timer, uint8_t irqn);

    virtual int setIRQPriority(int) override;

    virtual int enable() override;

    virtual int enableIRQ() override;

    virtual int disable() override;

    virtual int disableIRQ() override;

    virtual int reset() override;

    virtual int setMode(TimerMode t) override;

    virtual int setCompare(uint8_t channel, uint32_t value) override;

    virtual int offsetCompare(uint8_t channel, uint32_t value) override;

    virtual int clearCompare(uint8_t channel) override;

    virtual uint32_t captureCounter() override;

    virtual int setClockSpeed(uint32_t speedKHz) override;

    virtual int setBitMode(TimerBitMode t) override;
};

}

#endif