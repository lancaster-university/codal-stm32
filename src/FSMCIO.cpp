// redefine MCU type as one with the FSMC peripheral
#ifdef STM32F401xE
#undef STM32F401xE
#define STM32F412Rx
#endif

#include "FSMCIO.h"
#include "dma.h"

// define missing pins
#define PD_4 0x34
#define PD_5 0x35

#define ZERO(f) memset(&f, 0, sizeof(f))
#define oops() target_panic(909)

namespace codal
{

static FSMCIO *theInstance;

FSMCIO::FSMCIO(uint32_t flags, PinNumber wr, PinNumber rd)
{
    ZERO(hdma);

    theInstance = this;

    GPIO_InitTypeDef gpio_init_structure;

    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Alternate = GPIO_AF12_FSMC;

    __HAL_RCC_FSMC_CLK_ENABLE();

    int hasRead = rd != 0xff;

    // 100+ pin version
#ifdef PD_5
    if (wr == PD_5)
    {
        if (hasRead && rd != PD_4)
            oops();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_5;
        if (rd == PD_4)
            gpio_init_structure.Pin |= GPIO_PIN_4;
        HAL_GPIO_Init(GPIOD, &gpio_init_structure);

        gpio_init_structure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
        HAL_GPIO_Init(GPIOE, &gpio_init_structure);
    }
    else
#endif
        if (wr == PC_2 || wr == PD_2)
    {
        if (hasRead && rd != PC_5)
            oops();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        gpio_init_structure.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        HAL_GPIO_Init(GPIOA, &gpio_init_structure); // AF12

        gpio_init_structure.Pin = 0;
        if (wr == PC_2)
            gpio_init_structure.Pin |= GPIO_PIN_2;
        if (rd == PC_5)
            gpio_init_structure.Pin |= GPIO_PIN_5;
        if (gpio_init_structure.Pin)
            HAL_GPIO_Init(GPIOC, &gpio_init_structure); // AF12

        gpio_init_structure.Alternate = GPIO_AF10_FMC;
        gpio_init_structure.Pin = GPIO_PIN_6 | GPIO_PIN_11 | GPIO_PIN_12;
        HAL_GPIO_Init(GPIOC, &gpio_init_structure); // AF10

        gpio_init_structure.Pin = GPIO_PIN_14;
        HAL_GPIO_Init(GPIOB, &gpio_init_structure); // AF10

        if (wr == PD_2)
        {
            __HAL_RCC_GPIOD_CLK_ENABLE();
            gpio_init_structure.Pin = GPIO_PIN_2;
            HAL_GPIO_Init(GPIOD, &gpio_init_structure); // AF10
        }
    }
    else
    {
        oops();
    }

    // 4 gives 72ns write cycle, datasheet for ILI9341 says 66ns min
    // 3 gives 60ns
    // everything works down to 1
    uint32_t datast = flags & 0xff;

    FSMC_Bank1->BTCR[0] = FSMC_BCR1_EXTMOD | FSMC_BCR1_WREN | FSMC_BCR1_WFDIS |
                          0x80;                       // 0x80 is reserved bit reset value
    FSMC_Bank1->BTCR[1] = 0x112419;                   // timing for reading
    FSMC_Bank1E->BWTR[0] = 0xff00010 | (datast << 8); // timing for writing

    FSMC_Bank1->BTCR[0] |= FSMC_BCR1_MBKEN;

    dma_init(0, DMA_TX, &hdma, DMA_FLAG_PRI(3));
}

#define FSMC_DATA *((volatile uint8_t *)0x60000000)

void FSMCIO::send(const void *txBuffer, uint32_t txSize)
{
    uint8_t *ptr = (uint8_t *)txBuffer;
    while (txSize--)
        FSMC_DATA = *ptr++;
}

static void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *)
{
    theInstance->doneHandler(theInstance->handlerArg);
}

static void HAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *)
{
    oops();
}

void FSMCIO::startSend(const void *txBuffer, uint32_t txSize, PVoidCallback doneHandler,
                       void *handlerArg)
{
    this->doneHandler = doneHandler;
    this->handlerArg = handlerArg;
    hdma.XferCpltCallback = HAL_SRAM_DMA_XferCpltCallback;
    hdma.XferErrorCallback = HAL_SRAM_DMA_XferErrorCallback;
    HAL_DMA_Start_IT(&hdma, (uint32_t)txBuffer, (uint32_t)&FSMC_DATA, txSize);
}

} // namespace codal
