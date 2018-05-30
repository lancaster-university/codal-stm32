#ifndef PLATFORM_INCLUDES
#define PLATFORM_INCLUDES

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "stm32.h"

#define PROCESSOR_WORD_TYPE uint32_t

#define DEVICE_COMPONENT_COUNT 64

#define CODAL_ASSERT(cond)                                                                         \
    if (!(cond))                                                                                   \
    target_panic(909)

#endif
