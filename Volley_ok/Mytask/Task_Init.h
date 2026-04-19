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
ChassisMode chassis_mode = REMOTE;
PackControl_t recv_pack;
Remote_Handle_t Remote_Control; //取出遥控器数据
//Chassis_t chassis;
uint8_t recv_buff[20] = {0};
uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[30];
float rocker_filter[4] = {0};

#define MAX_ROBOT_OMEGA ANGLE2RAD(30.0f)
extern SemaphoreHandle_t Jy61_semaphore;
extern SemaphoreHandle_t remote_semaphore;
extern SemaphoreHandle_t Remote_semaphore;
extern ChassisMode chassis_mode;
//--------------------------------底盘控制-------------------------------------------------------------------------
void Remote(void *pvParameters);
TaskHandle_t Remote_Handle;
typedef struct
{
	PID2 PID;
	int dead_area;
	PID_EREOR_TypeDef PID_ERROR;
	VESC_t steer;
}VESC_INIT;

#define PI 3.14159265359f
#define MAX_VELOCITY 10.0f	  // 底盘最大速度
#define MAX_OMEGA PI*10	 	 //最大角速度
#define LENGTH 0.457f	 	//底盘中心到轮子的距离
#define WHEEL_RADIUS 0.075f  //轮的半径
#define MODE_t  1		  //等于0为漫反射开关模式，1为摄像头模式
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define MAX_ROBOT_VEL 5.0f // m/s
float Vx =0;  
float Vy =0;  
float Wz =0;  
volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;
volatile float wheel_one = 0.0f; 
volatile float wheel_two = 0.0f; 
volatile float wheel_three=0.0f; 
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))
//------------------------------------------------------
TaskHandle_t Hit_Task_Handle;
void Hit_Task(void *pvParameters);
TaskHandle_t Back_Task_Handle;
void Back_Task(void *pvParameters);
CubicParam_t cubic; 
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
typedef struct
{
	exp_param exp;
	GO_MotorHandle_t go_volleyball;
	PID2 vel_pid;
	PID2 pos_pid;
}push;
typedef struct {
    uint32_t total;
    uint32_t overrun;
    uint32_t frame;
    uint32_t noise;
    uint32_t parity;
    uint32_t last_error_time;
    uint32_t continuous_errors;
    uint32_t recovery_attempts;
    uint32_t last_recovery_time;
} ErrorStats_t;
void Hit_Task(void *pvParameters);
#endif
