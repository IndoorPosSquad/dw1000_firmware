# DWM1000 Firmware [STM32 Program]

This is the STM32 part of an UWB based indoor positioning program.
~~~
         SPI       USB/UART
DWM1000 <===> STM32 <===> Host(PC/Android/MCU)
~~~

## Build
使用Keli MDK进行构建，工程文件位于USER/目录下。

## Config
主要的Config均位于 CONFIG.h 文件中

### 结点类型的配置
配置结点类型需要通过反注释以下几个#define，需要注意的是，当配置为任何
一个RX结点时，除了反注释RX*以外还需要同时反注释RX，例如以下：
~~~
//#define TX

#define RX
//#define RX1
#define RX2
//#define RX3
//#define RX4
~~~

### 天线延时的设置

从距离到时间的换算公式为
延时 = 距离(米) / 4.6917519677e-3
其中4.6917519677e-3为o40位计数器一个周斯内电磁波的传播距离

~~~
#ifdef TX
#define ANTENNA_DELAY 0x000041C6
#endif

#ifdef RX1
#define ANTENNA_DELAY 0x000041C6
#endif

#ifdef RX2
#define ANTENNA_DELAY 0x000000
#endif

#ifdef RX3
#define ANTENNA_DELAY 0x000000
#endif
~~~

### 输出内容的设置
首先可以通过设置DEBUG_LVL的值来控制输出Log的数量，设置为:
- 0: 正常使用时需设置为0
- 1: 会输出关于收发事件的Log
- 2: 会输出关于详细的关于收发事件的Log及状态位

另外还可以通过注释以下两行与否来决定是否输出MPU6050的信息与DW1000的温
度电压信息
~~~
#define USE_MPU6050
#define USE_TEMP_VOLT_SENSOR
~~~

## 与上位机通信的数据包格式
Host to Controller Comm
~~~
+-------+---------------+---------------+---------------+---------------+---------------+---------------+
| Bytes |               0               |       1       |   Variable    |      N-1      |       N       |
+-------+---+---+---+---+---+---+---+---+---------------+---------------+---------------+---------------+
| Bits  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |               |   Variable    |                               |
+-------+---+---+---+---+---+---+---+---+---------------+---------------+-------------------------------+
|       | TYPE  |  CMD  |     FRAG      | Packet Length |    Payload    |          FEC(Optional)        |
+-------+-------+-----------------------+---------------+---------------+-------------------------------+
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

~~~
