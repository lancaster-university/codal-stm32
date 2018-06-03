/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

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
#ifndef CODAL_ZI2C_H
#define CODAL_ZI2C_H

#include "CodalConfig.h"
#include "codal-core/inc/driver-models/I2C.h"
#include "Pin.h"

namespace codal
{
/**
 * Class definition for I2C service
 */
class ZI2C : public codal::I2C
{
protected:
    I2C_HandleTypeDef i2c;
    codal::Pin &sda, &scl;
    bool needsInit;
    void init();

public:
    /**
     * Constructor.
     */
    ZI2C(codal::Pin &sda, codal::Pin &scl);

    /** Set the frequency of the I2C interface
     *
     * @param frequency The bus frequency in hertz
     */
    int setFrequency(uint32_t frequency);

    /**
     * Issues a START condition on the I2C bus
     * @return DEVICE_OK on success, or an error code
     */
    virtual int start();

    /**
     * Issues a STOP condition on the I2C bus
     * @return DEVICE_OK on success, or an error code
     */
    virtual int stop();

    /**
     * Writes the given byte to the I2C bus.
     *
     * The CPU will busy wait until the transmission is complete.
     *
     * @param data The byte to write.
     * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the write request failed.
     */
    virtual int write(uint8_t data);

    /**
     * Reads a single byte from the I2C bus.
     * The CPU will busy wait until the transmission is complete.
     *
     * @return the byte read from the I2C bus, or DEVICE_I2C_ERROR if the the write request failed.
     */
    virtual int read(AcknowledgeType ack = ACK);
};
} // namespace codal

#endif
