#ifndef FSMCIO_H
#define FSMCIO_H

#include "SPI.h"
#include "Pin.h"
#include "ScreenIO.h"

namespace codal
{

class FSMCIO : public ScreenIO
{
public:
    DMA_HandleTypeDef hdma;
    PVoidCallback doneHandler;
    void *handlerArg;

    FSMCIO(uint32_t flags, PinNumber wr, PinNumber rd);
    virtual void send(const void *txBuffer, uint32_t txSize);
    virtual void startSend(const void *txBuffer, uint32_t txSize, PVoidCallback doneHandler,
                           void *handlerArg);
};


} // namespace codal

#define CODAL_CREATE_PARALLEL_SCREEN_IO new codal::FSMCIO

#endif
