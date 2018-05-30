#include "ZTimer.h"
#include "CodalCompat.h"
#include "Timer.h"
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
    this->prev = k_cycle_get_32();
    k_timer_init(&timer, &ZTimer::callback, NULL);
    k_timer_user_data_set(&timer, this);
}

void ZTimer::triggerIn(CODAL_TIMESTAMP t)
{
    t /= 1000;
    if (t <= 1)
        t = 1;
    k_timer_start(&timer, t, 0);
}

void ZTimer::syncRequest()
{
    int key = irq_lock();
    u32_t curr = k_cycle_get_32();
    u32_t delta = curr - this->prev;
    this->prev = curr;
    u32_t elapsed =
        ((u64_t)delta * (u64_t)sys_clock_us_per_tick) / (u64_t)sys_clock_hw_cycles_per_tick;
    this->sync(elapsed);
    irq_unlock(key);
}
}
