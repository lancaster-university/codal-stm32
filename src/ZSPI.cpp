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
#include "board_pinmux.h"
#include "codal_target_hal.h"

#include "logging/sys_log.h"

#ifdef CONFIG_SOC_SERIES_STM32F4X
#include "drivers/clock_control/stm32_clock_control.h"
#include "zephyr/drivers/spi/spi_ll_stm32.h"
#endif

namespace codal
{

static ZSPI *instances[4];

#define ZERO(f) memset(&f, 0, sizeof(f))
/**
 * Constructor.
 */
ZSPI::ZSPI(Pin *mosi, Pin *miso, Pin *sclk) : codal::SPI()
{
#ifdef ZSPI_DMA_SUPPORTED
    ZERO(dma_cfg);
    ZERO(dma_block_cfg);
    dma = NULL;
    dma_chan_id = 0;
    k_sem_init(&dma_done, 0, 1);
#endif

    pinmux_setup_spi(mosi ? (int)mosi->name : -1, //
                     miso ? (int)miso->name : -1, //
                     sclk ? (int)sclk->name : -1, //
                     &dev);
    ZERO(config);
    ZERO(rxBuf);
    ZERO(txBuf);
    rxBufSet.buffers = &rxBuf;
    rxBufSet.count = 1;
    txBufSet.buffers = &txBuf;
    txBufSet.count = 1;
    setFrequency(1000000);
    setMode(0, 8);

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
