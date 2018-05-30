#ifndef CODAL_Z_TIMER_H
#define CODAL_Z_TIMER_H

#include "codal-core/inc/types/Event.h"
#include "codal-core/inc/driver-models/Timer.h"

namespace codal
{
class ZTimer : public codal::Timer
{
    u32_t prev;
    k_timer timer;
    static void callback(k_timer *tm);
public:
    ZTimer();
    virtual void triggerIn(CODAL_TIMESTAMP t);
    virtual void syncRequest();
};
}

#endif
