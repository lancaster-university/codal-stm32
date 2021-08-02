#include "stm32.h"
#include "codal_target_hal.h"
#include "CodalDmesg.h"
#include "CodalCompat.h"
#include "Timer.h"

static int8_t irq_disabled;
void target_enable_irq()
{
    irq_disabled--;
    if (irq_disabled <= 0) {
        irq_disabled = 0;
        __enable_irq();
    }
}

void target_disable_irq()
{
    __disable_irq();
    irq_disabled++;
}

void target_wait_for_event()
{
    __WFE();
}

void target_wait(uint32_t milliseconds)
{
    HAL_Delay(milliseconds);
}

void target_wait_us(unsigned long us)
{
    codal::system_timer_wait_us(us);
}

int target_seed_random(uint32_t rand)
{
    return codal::seed_random(rand);
}

int target_random(int max)
{
    return codal::random(max);
}

/*
    The unique device identifier is ideally suited:
        * for use as serial numbers (for example USB string serial numbers or other end applications)
        * for use as security keys in order to increase the security of code in Flash memory while using and combining this unique ID with software cryptographic primitives and protocols before programming the internal Flash memory
        * to activate secure boot processes, etc.
    The 96-bit unique device identifier provides a reference number which is unique for any
    device and in any context. These bits can never be altered by the user.
    The 96-bit unique device identifier can also be read in single bytes/half-words/words in different ways and then be concatenated using a custom algorithm.
*/

static uint64_t fnv_1a_64(const uint8_t *data, uint32_t len) {
    uint64_t res = 0xcbf29ce484222325;
    while (len--) {
        res ^= *data++;
        res *= 0x100000001b3;
    }
    return res;
}

#define STM32_UUID ((uint8_t *)0x1FFF7A10)
uint64_t target_get_serial()
{
    static uint64_t serial;
    if (!serial)
        serial = fnv_1a_64(STM32_UUID, 12);
    return serial;
}

void target_reset()
{
#ifdef STM32F4
    PWR->CR |= PWR_CR_DBP;
    RCC->BDCR |= RCC_BDCR_RTCEN;
    RTC->BKP0R = 0x24a22d12; // skip bootloader
#endif
    NVIC_SystemReset();
}

extern "C" void assert_failed(uint8_t* file, uint32_t line)
{
    target_panic(920);
}

__attribute__((weak))
void target_panic(int statusCode)
{
    target_disable_irq();

    DMESG("*** CODAL PANIC : [%d]", statusCode);
    while (1)
    {
    }
}

extern "C" void init_irqs();
void target_init()
{
    HAL_Init();
    init_irqs();
}

/**
 *  Thread Context for an ARM Cortex core.
 *
 * This is probably overkill, but the ARMCC compiler uses a lot register optimisation
 * in its calling conventions, so better safe than sorry!
 */
struct PROCESSOR_TCB
{
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R4;
    uint32_t R5;
    uint32_t R6;
    uint32_t R7;
    uint32_t R8;
    uint32_t R9;
    uint32_t R10;
    uint32_t R11;
    uint32_t R12;
    uint32_t SP;
    uint32_t LR;
    uint32_t stack_base;
};

PROCESSOR_WORD_TYPE fiber_initial_stack_base()
{
    uint32_t mbed_stack_base;

    mbed_stack_base = DEVICE_STACK_BASE;

    return mbed_stack_base;
}

void *tcb_allocate()
{
    return (void *)malloc(sizeof(PROCESSOR_TCB));
}

/**
 * Configures the link register of the given tcb to have the value function.
 *
 * @param tcb The tcb to modify
 * @param function the function the link register should point to.
 */
void tcb_configure_lr(void *tcb, PROCESSOR_WORD_TYPE function)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->LR = function;
}

/**
 * Configures the link register of the given tcb to have the value function.
 *
 * @param tcb The tcb to modify
 * @param function the function the link register should point to.
 */
void tcb_configure_sp(void *tcb, PROCESSOR_WORD_TYPE sp)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->SP = sp;
}

void tcb_configure_stack_base(void *tcb, PROCESSOR_WORD_TYPE stack_base)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->stack_base = stack_base;
}

PROCESSOR_WORD_TYPE tcb_get_stack_base(void *tcb)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    return tcbPointer->stack_base;
}

PROCESSOR_WORD_TYPE get_current_sp()
{
    return __get_MSP();
}

PROCESSOR_WORD_TYPE tcb_get_sp(void *tcb)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    return tcbPointer->SP;
}

void tcb_configure_args(void *tcb, PROCESSOR_WORD_TYPE ep, PROCESSOR_WORD_TYPE cp,
                        PROCESSOR_WORD_TYPE pm)
{
    PROCESSOR_TCB *tcbPointer = (PROCESSOR_TCB *)tcb;
    tcbPointer->R0 = (uint32_t)ep;
    tcbPointer->R1 = (uint32_t)cp;
    tcbPointer->R2 = (uint32_t)pm;
}

extern PROCESSOR_WORD_TYPE __end__;
PROCESSOR_WORD_TYPE codal_heap_start = (PROCESSOR_WORD_TYPE)(&__end__);
