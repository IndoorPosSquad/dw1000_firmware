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
#define DEBUG_LVL 0

/*
  如果需要调试上位机，反注释 FAKE_SERIAL
  如果需要位置信息，反注释 SOLVE_LOCATION
 */
//#define FAKE_SERIAL
#define SOLVE_LOCATION

// 定位周期配置
#define LOCATION_PERIOD (1)
#define TICK_IN_PERIOD (15) // 5 <= TICK_IN_PERIOD <= 255

// antenna delay
/*
  计算公式 antenna_delay = 距离(米) / 4.6917519677e-3
  其中4.6917519677e-3为40位计数器一个周斯内电磁波的传播距离
  RX与TX的Antenna Delay之和为两者测距时的稳态误差

  例如，TX与RX1的测距稳态误差为158米，则两者天线延时之和为：
  158 / 4.691e-3 = 33675
 */
#ifdef TX
#define ANTENNA_DELAY 16838
#endif

#ifdef RX1
#define ANTENNA_DELAY 17137
#endif

#ifdef RX2
#define ANTENNA_DELAY 16952
#endif

#ifdef RX3
#define ANTENNA_DELAY 11439
#endif

// 若RAW_OUTPUT为0，则输出解算后的结果
#define RAW_OUTPUT 1

// 若用MPU6050, 则反注释下一行
// #define USE_MPU6050

// 若输出温度电压值则反注释下一行
// #define USE_TEMP_VOLT_SENSOR

#endif
