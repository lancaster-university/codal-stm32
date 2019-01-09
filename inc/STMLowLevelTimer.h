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

    virtual int enable();

    virtual int enableIRQ();

    virtual int disable();

    virtual int disableIRQ();

    virtual int reset();

    virtual int setMode(TimerMode t);

    virtual int setCompare(uint8_t channel, uint32_t value);

    virtual int offsetCompare(uint8_t channel, uint32_t value);

    virtual int clearCompare(uint8_t channel);

    virtual uint32_t captureCounter();

    virtual int setClockSpeed(uint32_t speedKHz);

    virtual int setBitMode(TimerBitMode t);
};

}

#endif