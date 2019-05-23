#include "dma.h"
#include "CodalDmesg.h"

#ifdef STM32F1

#define NUM_STREAMS 7
#define NUM_DMA 2

typedef struct
{
    uint32_t peripheral; // SPI1_BASE etc
    uint8_t rxdx;        // 1 rx, 2 tx
    uint8_t channel;
} DmaMap;

//#define LOG DMESG
#define LOG(...) ((void)0)

typedef struct
{
    DMA_Channel_TypeDef *instance;
    uint8_t irqn;
} DmaStream;

const DmaStream streams[] = //
    {{DMA1_Channel1, DMA1_Channel1_IRQn},
     {DMA1_Channel2, DMA1_Channel2_IRQn},
     {DMA1_Channel3, DMA1_Channel3_IRQn},
     {DMA1_Channel4, DMA1_Channel4_IRQn},
     {DMA1_Channel5, DMA1_Channel5_IRQn},
     {DMA1_Channel6, DMA1_Channel6_IRQn},
     {DMA1_Channel7, DMA1_Channel7_IRQn},
#ifdef DMA2_Channel1
     {DMA2_Channel1, DMA2_Channel1_IRQn},
     {DMA2_Channel2, DMA2_Channel2_IRQn},
     {DMA2_Channel3, DMA2_Channel3_IRQn},
     {DMA2_Channel4, DMA2_Channel4_5_IRQn},
     {DMA2_Channel5, DMA2_Channel4_5_IRQn},
#endif
     {0, 0}};

MBED_WEAK const DmaMap TheDmaMap[] = //
    {{ADC1_BASE, DMA_RX, 1},
     {SPI1_BASE, DMA_RX, 2},
     {SPI1_BASE, DMA_TX, 3},
     {SPI2_BASE, DMA_RX, 4},
     {SPI2_BASE, DMA_TX, 5},


     {USART3_BASE, DMA_TX, 2},
     {USART3_BASE, DMA_RX, 3},
     {USART1_BASE, DMA_TX, 4},
     {USART1_BASE, DMA_RX, 5},
     {USART2_BASE, DMA_TX, 6},
     {USART2_BASE, DMA_RX, 7},

#ifdef DAC_BASE
     {ADC3_BASE, DMA_RX, 15},
     {SPI3_BASE, DMA_RX, 11},
     {SPI3_BASE, DMA_TX, 12},
     {UART4_BASE, DMA_RX, 13},
     {UART4_BASE, DMA_TX, 15},
     {DAC_BASE, DMA_TX, 13},
#endif     

     // The end
     {0, 0, 0}};

static DMA_HandleTypeDef *handles[7 + 5];

#define HANDLE_IDX(id) (id < 10 ? id - 1 : id - 4)
static void irq_callback(int id)
{
    LOG("DMA irq %d", id);
    id = HANDLE_IDX(id);
    if (handles[id])
        HAL_DMA_IRQHandler(handles[id]);
}

#define DEFIRQ(nm, id)                                                                             \
    void nm() { irq_callback(id); }

DEFIRQ(DMA1_Channel1_IRQHandler, 1)
DEFIRQ(DMA1_Channel2_IRQHandler, 2)
DEFIRQ(DMA1_Channel3_IRQHandler, 3)
DEFIRQ(DMA1_Channel4_IRQHandler, 4)
DEFIRQ(DMA1_Channel5_IRQHandler, 5)
DEFIRQ(DMA1_Channel6_IRQHandler, 6)
DEFIRQ(DMA1_Channel7_IRQHandler, 7)
DEFIRQ(DMA2_Channel1_IRQHandler, 11)
DEFIRQ(DMA2_Channel2_IRQHandler, 12)
DEFIRQ(DMA2_Channel3_IRQHandler, 13)

void DMA2_Channel4_5_IRQHandler()
{
    irq_callback(14);
    irq_callback(15);
}

int dma_init(uint32_t peripheral, uint8_t rxdx, DMA_HandleTypeDef *obj, int flags)
{
    memset(obj, 0, sizeof(*obj));

    int id = 0;
    const DmaMap *map;

    for (map = TheDmaMap; map->peripheral; map++)
    {
        if (map->peripheral == peripheral && map->rxdx == rxdx)
        {
            id = HANDLE_IDX(map->channel);
            if (handles[id] == NULL)
            {
                handles[id] = obj;
                break;
            }
        }
    }

    CODAL_ASSERT(map->peripheral);

    obj->Instance = streams[id].instance;

    obj->Init.Direction = rxdx == DMA_RX ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH;
    obj->Init.PeriphInc = DMA_PINC_DISABLE;
    obj->Init.MemInc = DMA_MINC_ENABLE;
    if (flags & DMA_FLAG_2BYTE)
    {
        obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        obj->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    }
    else if (flags & DMA_FLAG_4BYTE)
    {
        obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        obj->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    }
    else
    {
        obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        obj->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    }
    obj->Init.Mode = DMA_NORMAL;
    obj->Init.Priority = rxdx == DMA_RX ? DMA_PRIORITY_HIGH : DMA_PRIORITY_LOW;

    if (map->channel <= 10)
        __HAL_RCC_DMA1_CLK_ENABLE();
#ifdef __HAL_RCC_DMA2_CLK_ENABLE
    else
        __HAL_RCC_DMA2_CLK_ENABLE();
#endif

    int res = HAL_DMA_Init(obj);
    CODAL_ASSERT(res == HAL_OK);

    LOG("DMA init %p irq=%d ch=%d str=%d", obj->Instance, streams[id].irqn, map->channel,
        map->stream);

    NVIC_EnableIRQ(streams[id].irqn);

    return 0;
}

#endif