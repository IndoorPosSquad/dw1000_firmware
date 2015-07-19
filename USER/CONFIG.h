#ifndef __CONFIG_H
#define __CONFIG_H

// 结点类型配置
/*
  NOTE: 如果配置为RX1/2/3/4，均需要同时反注释RX, 如要想配置为RX2，则：
  #define RX
  //#define RX1
  #define RX2
  //#define RX3
  //#define RX4
 */
#define TX

//#define RX
//#define RX1
//#define RX2
//#define RX3
//#define RX4


// debug level
/*
 */
#define DEBUG_LVL 2

// antenna delay
/*
  计算公式 antenna_delay = 距离(米) / 4.6917519677e-3
  其中4.6917519677e-3为40位计数器一个周斯内电磁波的传播距离
 */
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

// 若RAW_OUTPUT为0，则输出解算后的结果
#define RAW_OUTPUT 1

// 若用MPU6050, 则反注释下一行
// #define USE_MPU6050

// 若输出温度电压值则反注释下一行
// #define USE_TEMP_VOLT_SENSOR

#endif
