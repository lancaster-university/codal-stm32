#pragma once

#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_RX 1
#define DMA_TX 2
#define DMA_TIM_CH1 3
#define DMA_TIM_CH2 4
#define DMA_TIM_CH3 5
#define DMA_TIM_CH4 6

#define DMA_FLAG_1BYTE 0
#define DMA_FLAG_2BYTE 1
#define DMA_FLAG_4BYTE 2

#define DMA_FLAG_CIRCULAR 4

#define DMA_FLAG_PRI(n) ((n) << 8)

#define DMA_FLAG_DEFAULT DMA_FLAG_PRI(2)

int dma_init(uint32_t peripheral, uint8_t rxdx, DMA_HandleTypeDef *obj, int flags);

#ifdef __cplusplus
}
#endif
