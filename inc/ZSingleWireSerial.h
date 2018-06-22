#ifndef ZSINGLE_WIRE_SERIAL_H
#define ZSINGLE_WIRE_SERIAL_H

#include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "SingleWireSerial.h"
#include "PktSerial.h"

namespace codal
{

    class ZSingleWireSerial : public SingleWireSerial
    {
        // PktSerialPkt *recvQueue[10];

        UART_HandleTypeDef uart;
        DMA_HandleTypeDef hdma_tx;
        DMA_HandleTypeDef hdma_rx;

        protected:
        virtual void configureRxInterrupt(int enable);

        virtual int rawGetc();
        virtual int configureTx(int);

        virtual int configureRx(int);

        public:

        PktSerialPkt* currentBuffer;
        uint32_t currentBufferIndex;

        static void _complete(uint32_t instance, uint32_t ev);

        ZSingleWireSerial(Pin& p);

        virtual int putc(char c);
        virtual int getc();

        // virtual int sendPacket();

        // virtual int getPacket();

        virtual int setBaud(uint32_t baud);

        virtual int setMode(SingleWireMode sw);

        virtual int sendBreak();
    };
}

#endif