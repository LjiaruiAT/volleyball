#ifndef __HARDWARE_H__
#define __HARDWARE_H__


//74LS165引脚（遥控器按键扫描）
#define CLK_PIN             40   //3M
#define SH_LD_PIN           41   //3M
#define QH_PIN              2    //3M

//ADC摇杆输入（摇杆）
#define LEFT_ROCKER_X       ADC1_CHANNEL_6
#define LEFT_ROCKER_Y       ADC1_CHANNEL_5
#define RIGHT_ROCKER_X      ADC1_CHANNEL_3
#define RIGHT_ROCKER_Y      ADC1_CHANNEL_4

//通信链路层串口IO（串口IO口）
#define PIN_NUM_UART_RXD        18
#define PIN_NUM_UART_TXD        17

//电池电量ADC采样输入
#define BATTERY_INPUT_CHANNEL   ADC2_CHANNEL_4

//自持式供电电路使能引脚
#define PIN_NUM_POWER_ENABLE    20

#endif