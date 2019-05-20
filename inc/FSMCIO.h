#ifndef FSMCIO_H
#define FSMCIO_H

#include "SPI.h"
#include "Pin.h"
#include "ScreenIO.h"

namespace codal
{

ScreenIO *createParallelScreenIO(uint32_t flags, PinNumber wr, PinNumber rd);

} // namespace codal

#define CODAL_CREATE_PARALLEL_SCREEN_IO codal::createParallelScreenIO

#endif
