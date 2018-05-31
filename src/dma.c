#include "dma.h"

#define DMA_RX 1
#define DMA_TX 2

#define NUM_STREAMS 8
#define NUM_DMA 2

typedef struct
{
    uint32_t peripheral; // SPI1_BASE etc
    uint8_t rxdx;        // 1 rx, 2 tx
    uint8_t dma;         // 1 or 2
    uint8_t stream;
    uint8_t channel;
} DmaMap;

typedef struct
{
    DMA_Channel_TypeDef *instance;
    uint8_t irqn;
} DmaStream;

const DmaStream streams[] = //
    {{DMA1_Stream0, DMA1_Stream0_IRQn},
     {DMA1_Stream1, DMA1_Stream1_IRQn},
     {DMA1_Stream2, DMA1_Stream2_IRQn},
     {DMA1_Stream3, DMA1_Stream3_IRQn},
     {DMA1_Stream4, DMA1_Stream4_IRQn},
     {DMA1_Stream5, DMA1_Stream5_IRQn},
     {DMA1_Stream6, DMA1_Stream6_IRQn},
     {DMA1_Stream7, DMA1_Stream7_IRQn},
     {DMA2_Stream0, DMA2_Stream0_IRQn},
     {DMA2_Stream1, DMA2_Stream1_IRQn},
     {DMA2_Stream2, DMA2_Stream2_IRQn},
     {DMA2_Stream3, DMA2_Stream3_IRQn},
     {DMA2_Stream4, DMA2_Stream4_IRQn},
     {DMA2_Stream5, DMA2_Stream5_IRQn},
     {DMA2_Stream6, DMA2_Stream6_IRQn},
     {DMA2_Stream7, DMA2_Stream7_IRQn},
     {0, 0}};

const uint32_t channels[] = {
    DMA_CHANNEL_0, DMA_CHANNEL_1, DMA_CHANNEL_2, DMA_CHANNEL_3,
    DMA_CHANNEL_4, DMA_CHANNEL_5, DMA_CHANNEL_6, DMA_CHANNEL_7,
};

MBED_WEAK const DmaMap TheDmaMap[] = //
    {
        // SPI1
        {SPI1_BASE, DMA_RX, 2, 0, 3},
        {SPI1_BASE, DMA_RX, 2, 2, 3},
        {SPI1_BASE, DMA_TX, 2, 3, 3},
        {SPI1_BASE, DMA_TX, 2, 5, 3},

        // SPI2
        {SPI2_BASE, DMA_RX, 1, 3, 0},
        {SPI2_BASE, DMA_TX, 1, 4, 0},

        // SPI3
        {SPI3_BASE, DMA_RX, 1, 0, 0},
        {SPI3_BASE, DMA_RX, 1, 2, 0},
        {SPI3_BASE, DMA_TX, 1, 5, 0},
        {SPI3_BASE, DMA_TX, 1, 7, 0},

        // The end
        {0, 0, 0, 0, 0}};

static DMA_HandleTypeDef *handles[NUM_STREAMS * NUM_DMA];

static void irq_callback(int id)
{
    if (handles[id])
        HAL_DMA_IRQHandler(handles[id]);
}

#define DEFIRQ(nm, id)                                                                             \
    void nm() { irq_callback(id); }

DEFIRQ(DMA1_Stream0_IRQHandler, 0)
DEFIRQ(DMA1_Stream1_IRQHandler, 1)
DEFIRQ(DMA1_Stream2_IRQHandler, 2)
DEFIRQ(DMA1_Stream3_IRQHandler, 3)
DEFIRQ(DMA1_Stream4_IRQHandler, 4)
DEFIRQ(DMA1_Stream5_IRQHandler, 5)
DEFIRQ(DMA1_Stream6_IRQHandler, 6)
DEFIRQ(DMA1_Stream7_IRQHandler, 7)
DEFIRQ(DMA2_Stream0_IRQHandler, NUM_STREAMS + 0)
DEFIRQ(DMA2_Stream1_IRQHandler, NUM_STREAMS + 1)
DEFIRQ(DMA2_Stream2_IRQHandler, NUM_STREAMS + 2)
DEFIRQ(DMA2_Stream3_IRQHandler, NUM_STREAMS + 3)
DEFIRQ(DMA2_Stream4_IRQHandler, NUM_STREAMS + 4)
DEFIRQ(DMA2_Stream5_IRQHandler, NUM_STREAMS + 5)
DEFIRQ(DMA2_Stream6_IRQHandler, NUM_STREAMS + 6)
DEFIRQ(DMA2_Stream7_IRQHandler, NUM_STREAMS + 7)

int dma_init(uint32_t peripheral, uint8_t rxdx, DMA_HandleTypeDef *obj)
{
    memset(obj, 0, *obj);
    
    int id;
    for (auto map = TheDmaMap; map->peripheral; map++)
    {
        if (map->peripheral == peripheral && map->rxdx == rxdx)
        {
            id = map->dma * NUM_STREAMS + map->stream;
            if (handles[id] == NULL)
            {
                handles[id] = obj;
                break;
            }
        }
    }

    if (!map->peripheral)
        return -1;

    obj->Instance = streams[map->stream].instance;

    obj->Init.Channel = channels[map->channel];
    obj->Init.Direction = rxdx == DMA_TX ? DMA_MEMORY_TO_PERIPH : DMA_PERIPH_TO_MEMORY;
    obj->Init.PeriphInc = DMA_PINC_DISABLE;
    obj->Init.MemInc = DMA_MINC_ENABLE;
    obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    obj->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    obj->Init.Mode = DMA_NORMAL;
    obj->Init.Priority = DMA_PRIORITY_MEDIUM;
    obj->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    obj->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    obj->Init.MemBurst = DMA_MBURST_INC4;
    obj->Init.PeriphBurst = DMA_PBURST_INC4;

    if (map->dma == 1)
        __HAL_RCC_DMA1_CLK_ENABLE();
    else if (map->dma == 2)
        __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_DMA_Init(obj);

    // __HAL_LINKDMA(hspi, hdmarx, hdma_rx);

    NVIC_EnableIRQ(streams[map->stream].irqn);
}
