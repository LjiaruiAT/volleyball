#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_
#include "config.h"
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
#include "JY61.h"
#include "step.h"

#define GEAR_RATIO 19.2f

/* 陀螺仪滤波参数 */
#define GYRO_LPF_ALPHA      0.50f   /* 陀螺仪原始值低通系数 (0~1) */
#define SLIP_LPF_ALPHA      0.50f   /* 滑移量滤波系数 */
#define CORR_LPF_ALPHA      0.65f   /* 校正输出滤波系数 */

#define GYRO_DEADZONE       0.50f   /* 陀螺仪死区 (度/秒) */
#define SLIP_DEADZONE       0.30f   /* 滑移死区 (度/秒) */
#define CORR_OUT_DEADZONE   0.25f   /* 校正输出死区 (度/秒) */

#define CORR_OUT_MAX        4.0f    /* 校正输出最大值 (度/秒) */
#define SLIP_THRESHOLD      0.50f   /* 滑移判断阈值 (度/秒) */

/* 轮子抓地力权重 */
#define WHEEL1_GRIP_RATIO   1.0f
#define WHEEL2_GRIP_RATIO   1.0f
#define WHEEL3_GRIP_RATIO   0.85f
void Task_Init(void);
void send_flag(uint8_t val);


TaskHandle_t Remote_Jy61_Task_Handle;
void Remote_Jy61(void *pvParameters);
extern float Wz_correction;
extern float gyro_slip_val;
extern uint8_t slip_flag;
extern PID2 JY61_adjust;
extern uint8_t usart6_dma_buff[30];
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


extern float Vx;
extern float Vy;
extern float Wz;
extern float vision_x2;
extern float vision_y2;
extern float vision_vx;
extern float vision_vy;
extern float vision_vz;
extern volatile float v1;
extern volatile float v2;
extern volatile float v3;
extern volatile float wheel_one;
extern volatile float wheel_two;
extern volatile float wheel_three;
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
void Remote_Analysis_Task(void *pvParameters);
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
  float torque;
	float angle;
	float omega;
	float kp;
	float kd;
}RobStride_Stop;
// ״̬��
typedef enum {	
	PLAN = 0,  //�켣��ʼ�滮
	READY,//�ȴ�
	FIRE, //����
	ALIGN,//��λ
} IFState;
GPIO_PinState GPIOB0_State = GPIO_PIN_RESET;
GPIO_PinState GPIOB1_State = GPIO_PIN_RESET;
void Hit_Task(void *pvParameters);
#endif
