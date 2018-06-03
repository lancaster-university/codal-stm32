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
#include "ZI2C.h"
#include "ErrorNo.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "stm32f4xx_ll_i2c.h"

namespace codal
{

uint32_t setup_pin(Pin *p, uint32_t prev, const PinMap *map);

void ZI2C::init()
{
    if (!needsInit)
        return;
    needsInit = false;

    if (!i2c.Instance)
    {
        uint32_t inst = 0;
        inst = setup_pin(&sda, inst, PinMap_I2C_SDA);
        inst = setup_pin(&scl, inst, PinMap_I2C_SCL);

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
    CODAL_ASSERT(res == HAL_OK);
}

ZI2C::ZI2C(codal::Pin &sda, codal::Pin &scl) : codal::I2C(sda, scl), sda(sda), scl(scl)
{
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.ClockSpeed = 100000;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    i2c.Init.OwnAddress1 = 0xFE;
    i2c.Init.OwnAddress2 = 0xFE;

    needsInit = true;
}

int ZI2C::setFrequency(uint32_t frequency)
{
    i2c.Init.ClockSpeed = frequency;
    needsInit = true;
    return DEVICE_OK;
}

int ZI2C::start()
{
    init();
    while (LL_I2C_IsActiveFlag_BUSY(i2c.Instance))
        ;
    LL_I2C_DisableBitPOS(i2c.Instance);
    LL_I2C_GenerateStartCondition(i2c.Instance);
    while (!LL_I2C_IsActiveFlag_SB(i2c.Instance))
        ;
    return 0;
}

int ZI2C::stop()
{
    LL_I2C_GenerateStopCondition(i2c.Instance);
    return 0;
}

int ZI2C::write(uint8_t data)
{
    while (!LL_I2C_IsActiveFlag_TXE(i2c.Instance))
        ;
    i2c.Instance->DR = data;
    while (!LL_I2C_IsActiveFlag_BTF(i2c.Instance))
        ;
    return 0;
}

int ZI2C::read(AcknowledgeType ack)
{
    LL_I2C_AcknowledgeNextData(i2c.Instance, ack == ACK ? LL_I2C_ACK : LL_I2C_NACK);
    while (!LL_I2C_IsActiveFlag_RXNE(i2c.Instance))
        ;
    return i2c.Instance->DR;
}

} // namespace codal
