#ifndef FSMCIO_H
#define FSMCIO_H

#include "ST7735.h"
#include "ZPin.h"

#ifdef FSMC_Bank1

namespace codal
{

class FSMCIO : public ScreenIO
{
public:
    SRAM_HandleTypeDef hsram;
    DMA_HandleTypeDef hdma;
    PVoidCallback doneHandler;
    void *handlerArg;

    FSMCIO(uint32_t flags, PinNumber wr, PinNumber rd);
    virtual void send(const void *txBuffer, uint32_t txSize);
    virtual void startSend(const void *txBuffer, uint32_t txSize, PVoidCallback doneHandler,
                           void *handlerArg);
};

} // namespace codal

#define PARALLEL_SCREEN_IO codal::FSMCIO

#endif

#endif
