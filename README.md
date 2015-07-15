# DWM1000 Firmware [STM32 USB device Program]

This is the USB device part of an UWB based indoor positioning program.
~~~
         SPI         USB
DWM1000 <===> STM32 <===> Host Software
~~~

## Installation
Use MDK to build this program.

## serial port communication protocol
// Host to Controller Comm
// +-------+---------------+---------------+---------------+---------------+---------------+---------------+
// | Bytes |               0               |       1       |   Variable    |      N-1      |       N       |
// +-------+---+---+---+---+---+---+---+---+---------------+---------------+---------------+---------------+
// | Bits  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |               |   Variable    |                               |
// +-------+---+---+---+---+---+---+---+---+---------------+---------------+-------------------------------+
// |       | TYPE  |  CMD  |     FRAG      | Packet Length |    Payload    |          FEC(Optional)        |
// +-------+-------+-----------------------+---------------+---------------+-------------------------------+
 1. Frame Type
        00 - RES
        01 - Message
                The payload carries the raw message to sent, see raw_write().
        10 - Distance / Location poll
        11 - Command
                CMD - 00
                        Reboot
                CMD - 01
                        Write Reg
                CMD - 10
                        Read Reg
                CMD - 11
                        Set log level?
 2. Packet Length
        Total length of all Payloads in a sequence in Unsigned Integer 8.
 3. FEC(OPTIONAL)
        CRC16 of the frame.

 Controller to Host Comm
 1. Frame Type
        00 - Set Mac
                The payload carries the mac to be set.
        01 - Message
                The payload carries the raw message to sent, see raw_write().
        10 - Distance / Location poll
        11 - Command
                CMD - 00
                        Reboot
                CMD - 01
                        Write Reg
                CMD - 10
                        Read Reg
                CMD - 11
                        Set log level?
 2. Packet Length
        Total length of all Payloads in a sequence in Unsigned Integer 8.
 3. FEC(OPTIONAL)
        CRC16 of the frame.