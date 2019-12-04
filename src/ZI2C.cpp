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

#include "CodalConfig.h"
#include "CodalCompat.h"
#include "ZI2C.h"
#include "ZPin.h"
#include "ErrorNo.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "CodalDmesg.h"
#include "codal_target_hal.h"

#ifdef STM32F1
#include "stm32f1xx_ll_i2c.h"
#else
#include "stm32f4xx_ll_i2c.h"
#endif

namespace codal
{


void ZI2C::init_internal()
{
    if (!needsInit)
        return;
    needsInit = false;

    if (!i2c.Instance)
    {
        uint32_t inst = 0;
        inst = codal_setup_pin(&sda, inst, PinMap_I2C_SDA);
        inst = codal_setup_pin(&scl, inst, PinMap_I2C_SCL);

        i2c.Instance = (I2C_TypeDef *)inst;

        switch (inst)
        {
        case I2C1_BASE:
            __HAL_RCC_I2C1_CLK_ENABLE();
            break;
        case I2C2_BASE:
            __HAL_RCC_I2C2_CLK_ENABLE();
            break;
#ifdef I2C3_BASE
        case I2C3_BASE:
            __HAL_RCC_I2C3_CLK_ENABLE();
            break;
#endif
        }
    }

    int res = HAL_I2C_Init(&i2c);
    CODAL_ASSERT(res == HAL_OK, DEVICE_HARDWARE_CONFIGURATION_ERROR);
}

ZI2C::ZI2C(codal::Pin &sda, codal::Pin &scl) : codal::I2C(sda, scl), sda(sda), scl(scl)
{
    memset(&i2c, 0, sizeof(i2c));

    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.ClockSpeed = 100000;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    i2c.Init.OwnAddress1 = 0xFE;
    i2c.Init.OwnAddress2 = 0xFE;

    needsInit = true;
    numErrors = 0;
}

int ZI2C::setFrequency(uint32_t frequency)
{
    i2c.Init.ClockSpeed = frequency;
    needsInit = true;
    return DEVICE_OK;
}

#define I2C_TIMEOUT 1000

int ZI2C::write(uint16_t address, uint8_t *data, int len, bool repeated)
{
    if (data == NULL || len <= 0)
        return DEVICE_INVALID_PARAMETER; // Send a start condition

    CODAL_ASSERT(!repeated, DEVICE_I2C_ERROR);

    init_internal();
    // timeout in ms - we use 1s
    auto res = HAL_I2C_Master_Transmit(&i2c, address, data, len, I2C_TIMEOUT);

    if (res == HAL_OK)
        return DEVICE_OK;
    else
        return DEVICE_I2C_ERROR;
}

int ZI2C::read(uint16_t address, uint8_t *data, int len, bool repeated)
{
    if (data == NULL || len <= 0)
        return DEVICE_INVALID_PARAMETER;

    CODAL_ASSERT(!repeated, DEVICE_I2C_ERROR);

    init_internal();
    auto res = HAL_I2C_Master_Receive(&i2c, address, data, len, I2C_TIMEOUT);

    return wrapStatus(res);
}

int ZI2C::readRegister(uint16_t address, uint8_t reg, uint8_t *data, int length, bool repeated)
{
    CODAL_ASSERT(repeated, DEVICE_I2C_ERROR);
    init_internal();
    auto res = HAL_I2C_Mem_Read(&i2c, address, reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT);

    return wrapStatus(res);
}

int ZI2C::setSleep(bool sleepMode)
{
    // Doesn't seem to save any power
    #if 0
    if (sleepMode) {
        HAL_I2C_DeInit(&i2c);
        ((ZPin*)&sda)->disconnect();
        ((ZPin*)&sda)->setDigitalValue(1);
        ((ZPin*)&scl)->disconnect();
        ((ZPin*)&scl)->setDigitalValue(1);
    }
    #endif

    return DEVICE_OK;
}

int ZI2C::reset() {
    DMESG("reset I2C");

    sda.getDigitalValue();
    scl.getDigitalValue();

    for (int i = 0; i < 16 * 16; i++)
    {
        scl.setDigitalValue(1);
        target_wait_us(4);
        scl.setDigitalValue(0);
        target_wait_us(4);
    }

    codal_setup_pin(&sda, 0, PinMap_I2C_SDA);
    codal_setup_pin(&scl, 0, PinMap_I2C_SCL);

    return DEVICE_OK;
}

int ZI2C::wrapStatus(int halStatus)
{
    if (halStatus == HAL_OK) {
        numErrors = 0;
        return DEVICE_OK;
    } else {
        if (numErrors++ > 100) {
            numErrors = 0;
            reset();
        }
        return DEVICE_I2C_ERROR;
    }
}

} // namespace codal
