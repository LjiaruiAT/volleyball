#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "Task_Init.h"
#include <stdbool.h>

#define PI 3.14159265359f
#define MAX_VELOCITY 170.0f	  // 底盘最大速度
#define MAX_OMEGA PI*10	 	 //最大角速度
#define LENGTH 0.45f	 	//整车边长的一半（如果底盘为正方形）但是根据公式逆推我们怀疑这是轮子中心到底盘中心距离
#define WHEEL_RADIUS 0.075f  //轮的半径
#define MODE_t  1		  //等于0为漫反射开关模式，1为摄像头模式




//任务
extern TaskHandle_t Remote_t_handle;
extern TaskHandle_t Remote_Handle;



//陀螺仪缓存区
extern uint8_t position_dma_buff[50];

//任务函数
void Remote(void *pvParameters);

//映射遥感函数
float remote_to_velocity(int16_t *remote_value);
float remote_to_Omega(int16_t *remote_value);

uint8_t GetDriverID(uint16_t std_id);
int32_t RAMP_slf( int32_t final, int32_t now, int32_t ramp );

#endif
