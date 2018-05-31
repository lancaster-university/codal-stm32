/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "CodalConfig.h"
#include "ZSPI.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"
#include "codal-core/inc/driver-models/Timer.h"

namespace codal
{

static ZSPI *instances[4];

#define ZERO(f) memset(&f, 0, sizeof(f))

static inline uint32_t setup(Pin *p, uint32_t prev, const PinMap *map)
{
    if (!p)
        return 0;
    auto pin = p->name;
    uint32_t tmp = pinmap_peripheral(pin, map);
    pin_function(pin, pinmap_function(pin, map));
    CODAL_ASSERT(tmp == instance);
}

/**
 * Constructor.
 */
ZSPI::ZSPI(Pin *mosi, Pin *miso, Pin *sclk) : codal::SPI()
{
    uint32_t instance = setup(sclk, 0, PinMap_SPI_SCLK);
    instance = setup(miso, 0, PinMap_SPI_MISO);
    instance = setup(mosi, 0, PinMap_SPI_MOSI);

    spi.Instance = (SPI_TypeDef *)instance;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 7;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.Mode = SPI_MODE_MASTER;

    if (HAL_SPI_Init(&spi) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    if (HAL_SPI_TransmitReceive_DMA(&spi, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) !=
        HAL_OK)
    {
        /* Transfer error in transmission process */
        Error_Handler();
    }

    while (HAL_SPI_GetState(&spi) != HAL_SPI_STATE_READY)
    {
    }

    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
            instances[i] = this;
    }
}

/** Set the frequency of the SPI interface
 *
 * @param frequency The bus frequency in hertz
 */
int ZSPI::setFrequency(uint32_t frequency)
{
    config.frequency = frequency;
    return DEVICE_OK;
}

/** Set the mode of the SPI interface
 *
 * @param mode Clock polarity and phase mode (0 - 3)
 * @param bits Number of bits per SPI frame (4 - 16)
 *
 * @code
 * mode | POL PHA
 * -----+--------
 *   0  |  0   0
 *   1  |  0   1
 *   2  |  1   0
 *   3  |  1   1
 * @endcode
 */
int ZSPI::setMode(int mode, int bits)
{
    config.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE;

    if (mode & 1)
        config.operation |= SPI_MODE_CPHA;
    if (mode & 2)
        config.operation |= SPI_MODE_CPOL;

    return DEVICE_OK;
}

/**
 * Writes the given byte to the SPI bus.
 *
 * The CPU will wait until the transmission is complete.
 *
 * @param data The data to write.
 * @return Response from the SPI slave or DEVICE_SPI_ERROR if the the write request failed.
 */
int ZSPI::write(int data)
{
    rxCh = 0;
    txCh = data;
    if (transfer(&txCh, 1, &rxCh, 1) < 0)
        return DEVICE_SPI_ERROR;
    return rxCh;
}

void ZSPI::dma_callback(struct device *dev, u32_t id, int error_code)
{
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && instances[i]->dma == dev && instances[i]->dma_chan_id == id)
        {
            k_sem_give(&instances[i]->dma_done);
            break;
        }
    }
}

/**
 * Writes and reads from the SPI bus concurrently. Waits (possibly un-scheduled) for transfer to
 * finish.
 *
 * Either buffer can be NULL.
 */
int ZSPI::transfer(const uint8_t *txBuffer, uint32_t txSize, uint8_t *rxBuffer, uint32_t rxSize)
{
#ifdef CONFIG_SOC_SERIES_STM32F4X
    // if nothing to read, and we already did some writes, try DMA!
    if (!rxBuffer && txBuf.buf)
    {
        auto spi = ((spi_stm32_config *)dev->config->config_info)->spi;

        dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
        dma_cfg.source_data_size = 1;
        dma_cfg.dest_data_size = 1;
        dma_cfg.source_burst_length = 1;
        dma_cfg.dest_burst_length = 1;
        dma_cfg.dma_callback = ZSPI::dma_callback;
        dma_cfg.complete_callback_en = 0; // callback at the end only
        dma_cfg.error_callback_en = 1;    // no error callback
        dma_cfg.block_count = 1;
        dma_cfg.head_block = &dma_block_cfg;

        dma_block_cfg.block_size = txSize;
        dma_block_cfg.source_address = (u32_t)txBuffer;
        dma_block_cfg.dest_address = (u32_t)&spi->DR;

        if (!dma)
        {
            if (spi == SPI1)
            {
                dma = device_get_binding("DMA_2");
                dma_chan_id = 3;
                // chan 3
            }
            else if (spi == SPI2)
            {
                dma = device_get_binding("DMA_1");
                dma_chan_id = 4;
                // chan 0
            }
            else if (spi == SPI3)
            {
                dma = device_get_binding("DMA_1");
                dma_chan_id = 5;
                // chan 0
            }
            else
            {
                CODAL_ASSERT(0);
            }
        }

        // dma_chan_id is reall STM32 stream

        if (dma_config(dma, dma_chan_id, &dma_cfg))
        {
            return DEVICE_SPI_ERROR;
        }

        if (dma_start(dma, dma_chan_id))
        {
            return DEVICE_SPI_ERROR;
        }

        // enable SPI if not done yet
        if (!(spi->CR1 & SPI_CR1_SPE))
            spi->CR1 |= SPI_CR1_SPE;

        // Enable the SPI Error Interrupt Bit
        spi->CR2 |= SPI_CR2_ERRIE;

        // Enable Tx DMA Request
        spi->CR2 |= SPI_CR2_TXDMAEN;

        k_sem_take(&dma_done, K_FOREVER);

        return DEVICE_OK;
    }
#endif
    rxBuf.buf = rxBuffer;
    rxBuf.len = rxSize;
    txBuf.buf = (uint8_t *)txBuffer;
    txBuf.len = txSize;
    if (spi_transceive(dev, &config, txSize ? &txBufSet : NULL, rxSize ? &rxBufSet : NULL) < 0)
        return DEVICE_SPI_ERROR;

    return 0;
}

} // namespace codal
