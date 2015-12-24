#ifndef __CONFIG_H__
#define __CONFIG_H__

// 结点类型配置
#define TX
//#define RX

// 应用类型
//#define LOCATION
#define ETC

// debug level
/*
 */
#ifdef TX
#define DEBUG_LVL 2
#else
#define DEBUG_LVL 2
#endif

// 定位周期配置
#ifdef LOCATION
#define LOCATION_PERIOD (1.0f) // should use 1.0f instead of 1f
#define TICK_IN_PERIOD (20) // 5 <= TICK_IN_PERIOD <= 255
#endif

#ifdef ETC
#define LOCATION_PERIOD (3.0f) // should use 1.0f instead of 1f
#define TICK_IN_PERIOD (3) // find node, range node, upload node
#define UPLOAD_RANGE (100) // in cm
#endif


// 如果不定义SOLVE_LOCATION，则输出三个测距结果
//#define SOLVE_LOCATION
#define GROUND_ANCHOR
//#define A1_X -1.45
//#define A1_Y 2.74
//#define A1_Z 1.82
#define A1_X 0.0
#define A1_Y 0.0
#define A1_Z 0.10

//#define A2_X -1.45
//#define A2_Y -0.90
//#define A2_Z 1.82
#define A2_X 4.80
#define A2_Y 1.82
#define A2_Z 0.10

//#define A3_X 1.92
//#define A3_Y 1.92
//#define A3_Z 2.02
#define A3_X 0.0
#define A3_Y 4.55
#define A3_Z 0.10

#define CALI_POS_X 1.50
#define CALI_POS_Y 0.0
#define CALI_POS_Z 0.40

// antenna delay
/*
  计算公式 antenna_delay = 距离(米) / 4.6917519677e-3
  其中4.6917519677e-3为40位计数器一个周斯内电磁波的传播距离
  RX与TX的Antenna Delay之和为两者测距时的稳态误差

  例如，TX与RX1的测距稳态误差为158米，则两者天线延时之和为：
  158 / 4.691e-3 = 33675
 */
#define TX_ANTENNA_DELAY (16838 - 25)

/* 154.44
   154.62
   154.50 */
#define RX1_ANTENNA_DELAY 16085
#define RX2_ANTENNA_DELAY 16123
#define RX3_ANTENNA_DELAY 16103

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
