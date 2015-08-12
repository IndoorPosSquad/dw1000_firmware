#ifndef __CONFIG_H__
#define __CONFIG_H__

// 结点类型配置
//#define TX
#define RX

// debug level
/*
 */
#ifdef TX
#define DEBUG_LVL 2
#else
#define DEBUG_LVL 2
#endif

// 如果不定义SOLVE_LOCATION，则输出三个测距结果
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
#define TX_ANTENNA_DELAY 16838

#define RX1_ANTENNA_DELAY 17137
#define RX2_ANTENNA_DELAY 16952
#define RX3_ANTENNA_DELAY 11439


// 若用MPU6050, 则反注释下一行
// #define USE_MPU6050

// 若输出温度电压值则反注释下一行
// #define USE_TEMP_VOLT_SENSOR

/*
  如果需要调试上位机，反注释 FAKE_SERIAL
  如果需要位置信息，反注释 SOLVE_LOCATION
 */
//#define FAKE_SERIAL

#endif
