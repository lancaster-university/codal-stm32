#pragma once

#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_RX 1
#define DMA_TX 2

typedef struct
{
    uint32_t peripheral; // SPI1_BASE etc
    uint8_t rxdx;        // 1 rx, 2 tx
    uint8_t dma;         // 1 or 2
    uint8_t stream;
    uint8_t channel;
} DmaMap;

int dma_init(uint32_t peripheral, uint8_t rxdx, DMA_HandleTypeDef *obj);

extern const DmaMap TheDmaMap[];

#ifdef __cplusplus
}
#endif
