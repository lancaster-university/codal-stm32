#include "FSMCIO.h"
#include "dma.h"

#define ZERO(f) memset(&f, 0, sizeof(f))

namespace codal
{

static void FMC_BANK1_MspInit(void)
{
    GPIO_InitTypeDef gpio_init_structure;

    /* Enable FSMC clock */
    __HAL_RCC_FSMC_CLK_ENABLE();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Common GPIO configuration */
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_structure.Alternate = GPIO_AF12_FSMC;

    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOD, &gpio_init_structure);

    gpio_init_structure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &gpio_init_structure);
}

static void FMC_BANK1_Init(SRAM_HandleTypeDef &hsram)
{
    FMC_NORSRAM_TimingTypeDef sram_timing;
    FMC_NORSRAM_TimingTypeDef sram_timing_write;

    /*** Configure the SRAM Bank 1 ***/
    /* Configure IPs */
    hsram.Instance = FSMC_Bank1;
    hsram.Extended = FSMC_Bank1E;

    /* Timing for READING */
    sram_timing.AddressSetupTime = 9;
    sram_timing.AddressHoldTime = 1;
    sram_timing.DataSetupTime = 36;
    sram_timing.BusTurnAroundDuration = 1;
    sram_timing.CLKDivision = 2;
    sram_timing.DataLatency = 2;
    sram_timing.AccessMode = FSMC_ACCESS_MODE_A;

    /* Timing for WRITTING*/
    sram_timing_write.AddressSetupTime = 0;
    sram_timing_write.AddressHoldTime = 1;
    // 4 gives 72ns write cycle, datasheet for ILI9341 says 66ns min
    // 3 gives 60ns
    // everything works down to 1
    sram_timing_write.DataSetupTime = 4;
    sram_timing_write.BusTurnAroundDuration = 0;
    sram_timing_write.CLKDivision = 2;
    sram_timing_write.DataLatency = 2;
    sram_timing_write.AccessMode = FSMC_ACCESS_MODE_A;

    hsram.Init.NSBank = FSMC_NORSRAM_BANK1;
    hsram.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    hsram.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
    hsram.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
    hsram.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    hsram.Init.WriteFifo = FSMC_WRITE_FIFO_DISABLE;
    hsram.Init.PageSize = FSMC_PAGE_SIZE_NONE;
    hsram.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;

    /* Initialize the SRAM controller */
    FMC_BANK1_MspInit();
    HAL_SRAM_Init(&hsram, &sram_timing, &sram_timing_write);
}

static FSMCIO *theInstance;

FSMCIO::FSMCIO()
{
    ZERO(hsram);
    ZERO(hdma);

    theInstance = this;

    FMC_BANK1_Init(hsram);
    dma_init(0, DMA_TX, &hdma, 0);
    __HAL_LINKDMA(&hsram, hdma, hdma);
}

#define FSMC_DATA *((volatile uint8_t *)0x60000000)

void FSMCIO::send(const void *txBuffer, uint32_t txSize)
{
    uint8_t *ptr = (uint8_t *)txBuffer;
    while (txSize--)
        FSMC_DATA = *ptr++;
}

static void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    UNUSED(hdma);
    theInstance->doneHandler(theInstance->handlerArg);
}

static void HAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma)
{
    UNUSED(hdma);
    target_panic(909);
}

void FSMCIO::startSend(const void *txBuffer, uint32_t txSize, PVoidCallback doneHandler,
                       void *handlerArg)
{
    this->doneHandler = doneHandler;
    this->handlerArg = handlerArg;
    hsram.hdma->XferCpltCallback = HAL_SRAM_DMA_XferCpltCallback;
    hsram.hdma->XferErrorCallback = HAL_SRAM_DMA_XferErrorCallback;
    HAL_DMA_Start_IT(hsram.hdma, (uint32_t)txBuffer, (uint32_t)&FSMC_DATA, txSize);
}

} // namespace codal