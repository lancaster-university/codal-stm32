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

#ifndef CODAL_Z_PKTSERIAL_H
#define CODAL_Z_PKTSERIAL_H

#include "CodalConfig.h"
#include "codal-core/inc/driver-models/PktSerial.h"
#include "Pin.h"

namespace codal
{

class ZPktSerial : public codal::PktSerial
{
protected:
    Pin *tx;
    PktSerialPkt *curr;

    USART_HandleTypeDef uart;
    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;

    void enableUart();
    void startToListen();
    void onTxRise();

public:
    static void _complete(uint32_t instance, uint32_t ev);

    ZPktSerial(int id, codal::Pin &tx);

    virtual void start();
    virtual void stop();
    virtual int send(const PktSerialPkt *pkt);
};
} // namespace codal

#endif
