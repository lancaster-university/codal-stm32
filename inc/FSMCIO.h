#ifndef FSMCIO_H
#define FSMCIO_H

#include "ST7735.h"


namespace codal {

class FSMCIO : public ScreenIO
{
public:
    SRAM_HandleTypeDef hsram;
    DMA_HandleTypeDef hdma;
    PVoidCallback doneHandler;
    void *handlerArg;

    FSMCIO();
    virtual void send(const void *txBuffer, uint32_t txSize);
    virtual void startSend(const void *txBuffer, uint32_t txSize, PVoidCallback doneHandler,
                           void *handlerArg);
};


}

#endif
