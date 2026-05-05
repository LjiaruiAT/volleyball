#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_
#include "485_bus.h"
#include "crc_ccitt.h"
#include "go_motor.h"
#include "RMLibHead.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"
#include "RobStride2.h"
#include "usart.h"
#include "bsp_dwt.h"
#include "math.h"
#include "PID_old.h"
#include "math.h"
#include "motor.h"
#include "motorEx.h"
#include "dataFrame.h"
#include "comm.h"
#include "comm_stm32_hal_middle.h"
#include "PID.h"
#include "VESC.h"
#include "step.h"
void Task_Init(void);
void send_flag(uint8_t val);


TaskHandle_t Remote_Jy61_Task_Handle;
void Remote_Analysis();
void Remote_update(void *pvParameters);
TaskHandle_t Remote_update_handle;
typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;
  
typedef struct {
    float Ex;
    float Ey;
	
	
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STP,
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;;
#define MAX_ROBOT_OMEGA ANGLE2RAD(30.0f)
extern SemaphoreHandle_t Jy61_semaphore;
extern SemaphoreHandle_t remote_semaphore;
extern SemaphoreHandle_t Remote_semaphore;
extern ChassisMode chassis_mode;
extern Remote_Handle_t Remote_Control;
extern uint8_t usart4_dma_buff[30];
extern uint8_t usart5_dma_buff[60];
void Remote(void *pvParameters);
extern TaskHandle_t Remote_Handle;
typedef struct
{
	PID2 PID;
	int dead_area;
	PID_EREOR_TypeDef PID_ERROR;
	VESC_t steer;
}VESC_INIT;

#define PI 3.14159265359f
#define MAX_VELOCITY 15.0f	  
#define MAX_OMEGA PI*15	 	
#define LENGTH 0.457f	 
#define WHEEL_RADIUS 0.075f  
#define MODE_t  1		 
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define MAX_ROBOT_VEL 5.0f // m/s
extern float Vx;
extern float Vy;
extern float Wz;
extern volatile float v1;
extern volatile float v2;
extern volatile float v3;
extern volatile float wheel_one;
extern volatile float wheel_two;
extern volatile float wheel_three;
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))
extern TaskHandle_t Hit_Task_Handle;
void Hit_Task(void *pvParameters);
extern TaskHandle_t Back_Task_Handle;
void Back_Task(void *pvParameters);
extern CubicParam_t cubic; 
typedef struct 
{
    float exp_tor;
    float exp_pos;
    float exp_vel;
    float exp_kp;
    float exp_kd;
}exp_param;

typedef struct
{
  float expect_torque;
	float expect_angle;
	float expect_omega;
	float kp;
	float kd;
}RobStride_Expect;

typedef struct
{
  float reset_torque;
	float reset_angle;
	float reset_omega;
	float kp;
	float kd;
}RobStride_Reset;

// 状态机
typedef enum {
    READY = 0,//等待
    ALIGN,//复位
    FIRE, //击球
    PLAN  //轨迹开始规划
} IFState;
void Remote_Analysis_Task(void *pvParameters);

void Hit_Task(void *pvParameters);
#endif
