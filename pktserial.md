# PktSerial protocol

PktSerial is a network protocol utilizing two or three lines: ground, signal, and possibly power.
Nodes are connected in a network, usually in daisy-chain topology.

It is designed to be inexpensive to implement and extensible.

The protocol uses MCU's UART peripheral. The UART is run at 921600 baud using 3.3V TTL levels.
As of 2018 even the cheapest ($0.40) ARM Cortex-M0 MCU implement such UARTs.
To allow for re-defining the speed easily,
we use bauds as units of time in this document (where 1 bauds is around 1us as per the
bit-rate above).
Data format is 1 start bit, 8 bits of data, no parity and 1 stop bit.
The same line is used for both transmitting and receiving data.

When no one is transmitting, the data pin is configured for input
with the weak internal pull-down. Only nodes capable of independent
operation (i.e., not peripherals like radio dongles or joysticks)
should use pull-downs, other nodes should just use no-pull input.

## Sending

When a node wishes to send a packet:
* if the current node is transmitting, wait for that to finish
* while TRANSMIT flags is set wait randomly between 100 and 200 bauds
* reconfigure data pin for push-pull output and pulls the line high (this can be often done in a single write)
* wait for 32 bauds
* start transmitting header; note that the start bit is 0, and thus will pull the line low
* if header indicated more data to be sent: pull the line high for 32 bauds, and then transmit that data
* release the line (i.e., reconfigure data pin as input with pull down)

## Receiving 

The input line is normally configure to raise an interrupt on going high,
when that is detected:
* set TRANSMIT flag
* set a 2000 baud timer, that will clear the global transmit flag if not cleared
* configure data pin as UART receive
* read the header from UART
* if header indicates so, prepare buffer for additional data, and read it
* inform application layer of the incoming packet

## Notes

### Collision window

There are two possible collision scenarios in node B, after node A has pulled the line high to start
transmission:

* node B pulls the line high before it gets the interrupt from the node A pull-up
* node B checks the TRANSMIT flag, gets interrupt, sets TRANSMIT, exits interrupt, and pulls the line high

The first window is electrical (between one node pulling up
the line, and another node receiving an interrupt), while the second
is more on the software side. Both should be just a few cycles,
and should be minimized as much as possible.

Note that interrupt handling (or interrupt setup) is not part of the
collision window, since the MCU cannot configure the gpio while the
interrupt is being handled.

### Random notes

Both UART reads and writes are typically done using DMA.

Some MCUs (eg. STM32) have a mode where the RX and TX lines of an UART can
be internally connected, wheres for others they have to be physically shorted
externally.

## Header structure

```
struct PktSerialHeader {
  u16 crc16;
  u8 sourceAddr;
  u8 size;
  u8 data[16];
};
```

The `size` field specifies the total size of the packet payload.
If `size > 16` then data over the 16 initial bytes is sent after the break.

## Address assignment

When node is first connected:
* assign itself random source address
* wait ~10ms (random delay)
* send HELLO packet

If a node receives a packet with the source address matching its own, it should
reassign itself a random address and send HELLO packet again.

## Identification packet

All nodes broadcast INFO packets at random intervals between 100ms and 200ms.
INFO and HELLO packets carry the same data.
Upon hearing a HELLO packet, all nodes should send an INFO packet without
waiting for the normal INFO interval to elapse.

The `data` field of INFO and HELLO packet is as follows:

```
struct PktSerialID {
  u32 protocol;
  u32 deviceID;
  u32 version;
  u32 serial;
};
```

Scenario:
* arcade A gets cable, sends HELLO into void
* arcade B connects, sends HELLO, A sends INFO - everyone happy

* A gets cable
* radio connected, sends HELLO, A takes it over
* B connected - everyone happy
* C connected 

Bus-powered, vs battery powered


## TODO

* [ ] check all the delays, if they make sense
* [ ] packet type field
* [ ] maybe just use 1mbit even? check NRF51 datasheet
* [ ] check on UART tolerances
* [ ] check oscillator tolerances
