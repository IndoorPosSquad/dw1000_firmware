# DWM1000 Firmware [STM32 Program]

## 简介
基于UWB室内定位的下位机驱动。
定位精度：20cm
定位频率：每秒10~30次
使用频率：6.5GHz
使用带宽：500MHz
使用功率：<-35dBm/MHz

基本架构
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

从距离到时间的换算公式为:
延时 = 距离(米) / 4.6917519677e-3

其中4.6917519677e-3为40位计数器一个周斯内电磁波的传播距离

RX与TX的Antenna Delay之和为两者测距时的稳态误差。
例如，TX与RX1的测距稳态误差为158米，则两者天线延时之和为：
158 / 4.691e-3 = 33675
~~~
#ifdef TX
#define ANTENNA_DELAY 16838
#endif

#ifdef RX1
#define ANTENNA_DELAY 16965
#endif

#ifdef RX2
#define ANTENNA_DELAY 16752
#endif

#ifdef RX3
#define ANTENNA_DELAY 11189
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
                Host to Controller(H2C)
                        The payload carries the raw message to sent, see raw_write().
                Controller to Host(C2H)
                        The payload carries the raw message received, see raw_read().
        10 - Distance / Location poll
                Trigger the Location Service.
        11 - Command
                CMD - 00
                        Reboot.
                CMD - 01
                        Write Reg.
                CMD - 10
                        H2C - Read Reg.
                        C2H - Return the Read Reg Result.
                CMD - 11
                        Set log level?
 2. Packet Length
        Total length of all Payloads in a sequence in Unsigned 8 bits Integer.
 3. FEC(OPTIONAL)
        CRC16 of the frame.
~~~

## 无线通信数据包格式
基础格式参照`IEEE 802.15.4a`标准

我们尽量采用长地址
~~~
802.15.4a Frame:
+---------------+--------------+------------+-----------+-----------+----------+-----------+---------+
| Frame Control | Sequence Num | Dest PANID | Dest Addr | Src PANID | Src Addr |  Payload  |   FCS   |
+---------------+--------------+------------+-----------+-----------+----------+-----------+---------+
| 2 Bytes       | 1 Byte       | 2 Bytes    | 8 Bytes   | 2 Bytes   | 8 Bytes  | Var Bytes | 2 Bytes |
+---------------+--------------+------------+-----------+-----------+----------+-----------+---------+

Frame Control:
+------+---+---+---+----------+---------+--------+----------+---+---+---+-----+-----+----+----+-----+----+
| Bits | 0 | 1 | 2 |    3     |    4    |   5    |    6     | 7 | 8 | 9 | 10  | 11  | 12 | 13 | 14  | 15 |
+------+---+---+---+----------+---------+--------+----------+---+---+---+-----+-----+----+----+-----+----+
|      | Frame     | Security | Frame   | ACK    | PANID    | Reserved  | Dest Addr | Frame   | Src Addr |
|      | Type      | Enabled  | Pending | Requst | Compress |           | Mode      | Version | Mode     |
+------+-----------+----------+---------+--------+----------+-----------+-----------+---------+----------+
1. Frame Type
        100 - Location Service (802.15.4a Reserved).
                Then the first byte of Payload is used to identify the LS message type.
                0x00 - LS Req
                        Request for Location Service.
                0x01 - LS ACK
                        ACK for LS Req, and mark the receive time of LS Req(T_Req) and the sent time of LS ACK(T_ACK).
                0x02 - LS Data
                        Return T_ACK - T_Req.
                0x03 - LS Information Return
                        Return the distance data to the ACK node.
                0x04 - LS Forward
                        Forward the Location data to a specific node.
2. Other Fields
        Please reference 802.15.4a.
~~~

## API
~~~
void raw_write(u8* tx_buff, u16* size);
// 原始发送一个dw1000帧
// u8* tx_buff 为发送数据
// u16* size 为数据长度，不要超过126

void raw_read(u8* rx_buff, u16* size);
// 从缓冲区读出一个dw1000帧
// u8* rx_buff 接收数据
// u16* size 接受读取的数据长度

void Location_polling(void);
// 完成一次测距请求
// 锚点的MAC写在函数里，需要改进

void send_LS_ACK(u8 *src, u8 *dst);
// 发送定位应答
// u8 *src 为源地址，一般为本机MAC
// u8 *dst 为目的地地址

void send_LS_DATA(u8 *src, u8 *dst);
// 发送处理时延数据
// u8 *src 为源地址，一般为本机MAC
// u8 *dst 为目的地地址
~~~

## 算法
已知三个锚点ABC的位置，以及未知点X到ABC三点的距离。由于UWB测距精度十分高，我们直接采用所测距离，以ABC三点为底面，X为顶点构成四面体。
由海伦-秦九昭公式可以轻松得到这个四面体的体积，进而再求出X到ABC底面的高度。这样就能得到X的相对位置。

## USB DFU
DFU又是IAP(In Application Programming)。其实现原理非常简单，有一个终端检测程序来判断设备是否应进入DFU服务程序。
~~~
	+------------+
	| Int Dectect|
	+------------+
	|  DFU Mode  | 0x800000
	+------------+
	|  ........  |
	+------------+
	|  DW1K_FW   | 0x801000
	+------------+
	|  ........  |
	+------------+
~~~

### 构建
首先使用MDK构建DFU程序，并刷入，注意修改程序中定义的应用起始地址（这里以`0x801000`为例）。
然后使用MDK构建正常的应用程序，注意在编译前设置好MDK，把应用得起始地址设置与之前的相符（这里以`0x801000`为例）。
然后使用DFU固件转换工具，将生成的二进制文件转换为专用的DFU固件文件，再通过专用程序刷写。
