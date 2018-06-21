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
#include "ZPktSerial.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"
#include "codal-core/inc/driver-models/Timer.h"
#include "MessageBus.h"
#include "Event.h"
#include "CodalFiber.h"

#include "dma.h"
#include "pinmap.h"
#include "PeripheralPins.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define UART_ON (uart.Instance->CR1 & USART_CR1_UE)

#define LOG DMESG
//#define LOG(...) ((void)0)

namespace codal
{

static ZPktSerial *instances[4];

#define ZERO(f) memset(&f, 0, sizeof(f))

static int enable_clock(uint32_t instance)
{
    switch (instance)
    {
    case USART1_BASE:
        __HAL_RCC_USART1_CLK_ENABLE();
        NVIC_EnableIRQ(USART1_IRQn);
        return HAL_RCC_GetPCLK2Freq();
    case USART2_BASE:
        __HAL_RCC_USART2_CLK_ENABLE();
        NVIC_EnableIRQ(USART2_IRQn);
        return HAL_RCC_GetPCLK1Freq();
#ifdef USART6_BASE
    case USART6_BASE:
        __HAL_RCC_USART6_CLK_ENABLE();
        NVIC_EnableIRQ(USART6_IRQn);
        return HAL_RCC_GetPCLK2Freq();
#endif
    default:
        CODAL_ASSERT(0);
        return 0;
    }
    return 0;
}

void ZPktSerial::_complete(uint32_t instance, uint32_t mode)
{
    LOG("USART complete %p m=%d", instance, mode);
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && (uint32_t)instances[i]->uart.Instance == instance)
        {
            if (mode == 0)
                HAL_UART_IRQHandler(&instances[i]->uart);
            else if (mode == CODAL_PKTSERIAL_EVT_DATA_RECEIVED) {
                instances[i]->queue(instances[i]->curr);
                instances[i]->startToListen();
            } else 
                Event(instances[i]->id, mode);
            break;
        }
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hspi)
{
    ZPktSerial::_complete((uint32_t)hspi->Instance, CODAL_PKTSERIAL_EVT_DATA_SENT);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hspi)
{
    ZPktSerial::_complete((uint32_t)hspi->Instance, CODAL_PKTSERIAL_EVT_DATA_RECEIVED);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *hspi)
{
    ZPktSerial::_complete((uint32_t)hspi->Instance, CODAL_PKTSERIAL_EVT_ERROR);
}

#define DEFIRQ(nm, id)                                                                             \
    extern "C" void nm() { ZPktSerial::_complete(id, 0); }

DEFIRQ(USART1_IRQHandler, USART1_BASE)
DEFIRQ(USART2_IRQHandler, USART2_BASE)
#ifdef USART6_BASE
DEFIRQ(USART6_IRQHandler, USART6_BASE)
#endif

void ZPktSerial::stop()
{
    tx->setPull(PullMode::Down);
    tx->getDigitalValue();
    tx->eventOn(DEVICE_PIN_EVENT_NONE);
}

void ZPktSerial::start()
{
    if (uart.Instance) {
        startToListen();
        return;
    }

    uart.Instance = (USART_TypeDef *)pinmap_peripheral(tx->name, PinMap_UART_TX);

    LOG("USART instance %p", uart.Instance);

    enable_clock((uint32_t)uart.Instance);

    dma_init((uint32_t)uart.Instance, DMA_TX, &hdma_tx);
    __HAL_LINKDMA(&uart, hdmatx, hdma_tx);

    dma_init((uint32_t)uart.Instance, DMA_RX, &hdma_rx);
    __HAL_LINKDMA(&uart, hdmarx, hdma_rx);

    uart.Init.BaudRate = 1000000;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.Mode = UART_MODE_RX;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;

    auto res = HAL_HalfDuplex_Init(&uart);
    CODAL_ASSERT(res == HAL_OK);

    EventModel::defaultEventBus->listen(tx->id, DEVICE_PIN_EVT_RISE, this, &ZPktSerial::onTxRise,
                                        MESSAGE_BUS_LISTENER_IMMEDIATE);

    startToListen();
}

void ZPktSerial::enableUart()
{
    auto pin = tx->name;
    pin_function(pin, pinmap_function(pin, PinMap_UART_TX));
    pin_mode(pin, PullNone);
    __HAL_UART_ENABLE(&uart);
}

void ZPktSerial::onTxRise(Event e)
{
    if (UART_ON)
        return;

    enableUart();
    HAL_HalfDuplex_EnableReceiver(&uart);

    uint16_t sz = 0;
    int res;

    res = HAL_UART_Receive(&uart, (uint8_t*)&sz, 2, 3);
    if (res == HAL_OK && 0 < sz && sz < 4096)
    {
        if (curr)
            free(curr);
        curr = PktSerialPkt::alloc(sz);
        res = HAL_UART_Receive_DMA(&uart, (uint8_t*)&curr->crc, sz);
        if (res != HAL_OK)
        {
            free(curr);
            curr = NULL;
        }
    }
}


void ZPktSerial::startToListen()
{
    curr = NULL;
    __HAL_UART_DISABLE(&uart);

    tx->setPull(PullMode::Down);
    tx->getDigitalValue();
    tx->eventOn(DEVICE_PIN_EVENT_ON_EDGE);
}

ZPktSerial::ZPktSerial(int id, Pin &tx) : codal::PktSerial()
{
    this->tx = &tx;
    this->id = id;

    ZERO(uart);
    ZERO(hdma_tx);
    ZERO(hdma_rx);

    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
        {
            instances[i] = this;
            break;
        }
    }
}

int ZPktSerial::send(const PktSerialPkt *pkt)
{
    int res;

    LOG("USART start %p/%d", pkt, pkt->size);

    while (UART_ON)
    {
        int delay = getRandom() % 100 + 100;
        wait_us(delay);
    }

    tx->setDigitalValue(1);
    wait_us(100);

    enableUart();
    HAL_HalfDuplex_EnableTransmitter(&uart);

    res = HAL_UART_Transmit(&uart, (uint8_t*)&pkt->size, 2, 3);
    CODAL_ASSERT(res == HAL_OK);

    wait_us(30);

    fiber_wake_on_event(id, CODAL_PKTSERIAL_EVT_DATA_SENT);

    res = HAL_UART_Transmit_DMA(&uart, (uint8_t *)&pkt->crc, pkt->size);
    CODAL_ASSERT(res == HAL_OK);

    schedule();

    startToListen();

    return 0;
}

} // namespace codal
