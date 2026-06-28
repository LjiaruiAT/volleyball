#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "CANDrive.h"
#include "semphr.h"
#include "RobStride2.h"
#include "step.h"
#include "usb_receiver.h"
#include "usbd_core.h"
ChassisMode chassis_mode = REMOTE;
Remote_Handle_t Remote_Control;
uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[60];
float Vx = 0;
float Vy = 0;
float Wz = 0;
volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;
volatile float wheel_one = 0.0f;
volatile float wheel_two = 0.0f;
volatile float wheel_three = 0.0f;
TaskHandle_t Remote_Handle;
TaskHandle_t Hit_Task_Handle;
TaskHandle_t Back_Task_Handle;
CubicParam_t cubic;
char init_done_far = 0;
char init_done_middle = 0;
char init_done_near = 0;
char far = 0;
char middle = 0;
char near = 0;
TaskHandle_t Remote_Analysis_Handle;
Pack_TransRemote_t trans_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
extern SemaphoreHandle_t Remote_semaphore;  
RobStride_t R_left;
RobStride_t R_right;
void Task_Init()
{

		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	xTaskCreate(Remote,
         "Remote",
          400,
          NULL,
          4,
          &Remote_Handle); 
	xTaskCreate(Hit_Task,
			 "Hit_Task",
				258,
				NULL,
				4,
				&Hit_Task_Handle); 
    xTaskCreate(Back_Task,
			 "Back_Task",
				400,
				NULL,
				4,
				&Back_Task_Handle); 
						xTaskCreate(Remote_Analysis_Task, "Remote_Analysis_Task", 400, NULL, 4, &Remote_Analysis_Handle);

}
PackControl_t recv_pack;


static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}
/* ---- USB radar globals ---- */
float radar_dx = 0, radar_dy = 0, radar_d = 999.0f;
float remote_Ex = 0, remote_Ey = 0, remote_Eomega = 0;
float vx,vy,vz = 0;
  void Remote_Analysis(void)
  {
      /* 1. USB lazy init */
      extern USBD_HandleTypeDef hUsbDeviceFS;
      int usb_ready = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
      if (usb_ready && g_usb_data_sem == NULL) {
          UsbReceiver_Init();
      }

      /* 2. 雷达数据（非阻塞） */
      if (usb_ready && xSemaphoreTake(g_usb_data_sem, 0) == pdTRUE) {
          radar_dx = g_usb_data.x2;
          radar_dy = g_usb_data.y2;
				  vx       = g_usb_data.vx;
		      vy       = g_usb_data.vy;
				  vz       = g_usb_data.vz;
          radar_d  = sqrtf(radar_dx * radar_dx + radar_dy * radar_dy);
      } else if (!usb_ready) {
          radar_d = 999.0f;
      }

      /* 3. 遥控数据（非阻塞，始终读，不管什么模式） */
      if (xSemaphoreTake(Remote_semaphore, 0) == pdTRUE) {
          Remote_Control.Second = Remote_Control.First;
          Key_Parse(recv_pack.Key, &Remote_Control.First);
          remote_Ex     = recv_pack.rocker[0] / REMOTE_FIGER * MAX_ROBOT_VEL;
          remote_Ey     = recv_pack.rocker[1] / REMOTE_FIGER * MAX_ROBOT_VEL;
          remote_Eomega = recv_pack.rocker[2] / REMOTE_FIGER * MAX_ROBOT_OMEGA;
      }

      /* 4. 二选一输出 */
      if (radar_d <= 1.5f) {
          chassis_mode = AUTO;
          float Kp = 4.0f;
          Remote_Control.Ex     = Kp * (radar_dx - 0.0f);
          Remote_Control.Ey     = Kp * (radar_dy - 0.0f);
          Remote_Control.Eomega = remote_Eomega;
      } else {
          chassis_mode = REMOTE;
          Remote_Control.Ex     = remote_Ex;
          Remote_Control.Ey     = remote_Ey;
          Remote_Control.Eomega = remote_Eomega;
      }

      vTaskDelay(pdMS_TO_TICKS(10));
  }
void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;


VESC_INIT vesc_1 ={
	.steer.motor_id = 0x01,	
	.steer.hcan = &hcan2,
};
VESC_INIT vesc_2 ={
	.steer.motor_id = 0x02,
	.steer.hcan = &hcan2,
};
VESC_INIT vesc_3 ={
	.steer.motor_id = 0x03,
	.steer.hcan = &hcan2,
};
uint8_t tr_buf[3] = {0xAA, 0x00, 0x55};
int a = 1;
void Remote(void *pvParameters)
{
    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
    vesc_1.PID.Kp = 0.5f;
   	vesc_1.PID.Ki =0.002f;
	  vesc_1.PID.Kd = 0.1f;
    vesc_1.PID.limit = 10000.0f;
  	vesc_1.PID.output_limit = 40.0f;
    vesc_2.PID = vesc_1.PID;
	  vesc_3.PID = vesc_1.PID;
	    portTickType xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        v1 = -Remote_Control.Ex*OMNI_WHEEL_FACTOR*MAX_VELOCITY - Remote_Control.Ey*SQRT3_OVER_2*MAX_VELOCITY - LENGTH * Remote_Control.Eomega*MAX_OMEGA;
        v2 = +Remote_Control.Ex*OMNI_WHEEL_FACTOR*MAX_VELOCITY - Remote_Control.Ey*SQRT3_OVER_2*MAX_VELOCITY + LENGTH * Remote_Control.Eomega*MAX_OMEGA;
        v3 =    Remote_Control.Ex*SQRT3_OVER_2*MAX_VELOCITY - Remote_Control.Eomega*MAX_OMEGA*LENGTH;

        wheel_one   = (-(v1 / (2.0f * PI * WHEEL_RADIUS))* GEAR_RATIO);
        wheel_two   = ((v2 / (2.0f * PI * WHEEL_RADIUS))* GEAR_RATIO);
        wheel_three = (-(v3 / (2.0f * PI * WHEEL_RADIUS))* GEAR_RATIO);

        float wheel1_actual = (float)vesc_1.steer.epm / EXCHANGE_WHEEL_CONFIG;
        float wheel2_actual = (float)vesc_2.steer.epm / EXCHANGE_WHEEL_CONFIG;
        float wheel3_actual = (float)vesc_3.steer.epm / EXCHANGE_WHEEL_CONFIG;

        PID_Control2(wheel1_actual, wheel_one, &vesc_1.PID);
        PID_Control2(wheel2_actual, wheel_two, &vesc_2.PID);
        PID_Control2(wheel3_actual, wheel_three, &vesc_3.PID);
        
        VESC_SetCurrent(&vesc_1.steer, vesc_1.PID.pid_out);
        VESC_SetCurrent(&vesc_2.steer, vesc_2.PID.pid_out);
        VESC_SetCurrent(&vesc_3.steer, vesc_3.PID.pid_out);


            if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Right_Switch_Up)&&init_done_middle ==0&&init_done_near == 0)
                {init_done_far = 1;
                far = 1;}
            if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Right_Switch_Down)&&init_done_far==0 && init_done_near ==0)
                { init_done_middle = 1;
                middle = 1;}
            if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Right_Key_Right)&&init_done_middle == 0 && init_done_far == 0)
                {init_done_near = 1;
                near = 1;}
            if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Key_Up)&&init_done_middle == 0 && init_done_far == 0&& init_done_near ==0)
                {send_flag(4);
                }
        
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
void Remote_Analysis_Task(void *pvParameters)
{

	while(1)
	{
		Remote_Analysis();
	}
}
int16_t feel_1 = 0;
int16_t feel_2 = 0;
int16_t feel_3 = 0;
int16_t feel_4 = 0;
RobStride_Expect R_left_expect = {
	.expect_angle = -0.37f,
	.expect_omega = -11.0f,
	.expect_torque = -3.5f,
	.kp = 10.0f,
	.kd = 28.0f

};
RobStride_Expect R_right_expect = {
	.expect_angle = 0.39f,
	.expect_omega = 11.0f,
	.expect_torque = 3.5f,
	.kp = 10.0f,
	.kd = 28.0f 
};

RobStride_Reset R_left_reset = {
	.reset_angle = 0.0f,
	.reset_omega = 0.0f,
	.reset_torque = -2.4f,
	.kp = 10.0f,
	.kd = 1.0f
};
RobStride_Reset R_right_reset = {
	.reset_angle = 0.0f,
	.reset_omega = 0.0f,
	.reset_torque = 2.4f,
	.kp = 10.0f,
	.kd = 1.0f
};

RobStride_Stop R_left_stop = {
	.omega = 0.0f,
	.torque = 0.0f,
	.kp = 3.0f,
	.kd = 1.0f

};
RobStride_Stop R_right_stop = {
	.omega = 0.0f,
	.torque = 0.0f,
	.kp = 3.0f,
	.kd = 1.0f
	
};
RobStride_t R_left;
RobStride_t R_right;

CubicParam_t traj_left;
CubicParam_t traj_right;

TrajectoryState_t traj_left_state;
TrajectoryState_t traj_right_state;
CubicParam_t traj_left;
CubicParam_t traj_right;
static uint8_t ball_back_trigger = 0; // 击球标志
GPIO_PinState key1, key2, key3, key4; // 光电门
float time = 0.05f; // 轨迹规划时间
static uint8_t trigger_lock = 0; // 防止击球机构挡住光电门误触发
float left;
float right;
IFState ALLState = READY;

void Back_Task(void *pvParameters)
{
vTaskDelay(5000);
    RobStrideInit(&R_left, &hcan1, 0x01,RobStride_MotionControl, RobStride_04);
	  RobStrideInit(&R_right, &hcan1, 0x02,RobStride_MotionControl, RobStride_04);
	  vTaskDelay(100);
	  RobStrideSetMode(&R_left, RobStride_MotionControl);
	  RobStrideSetMode(&R_right, RobStride_MotionControl);
	  vTaskDelay(100);
    RobStrideEnable(&R_left);
	  RobStrideEnable(&R_right);
	  vTaskDelay(100);

    RobStrideResetAngle(&R_left);
    RobStrideResetAngle(&R_right);
	  vTaskDelay(200);

	  R_left_reset.reset_angle = R_left.state.rad;
	  R_right_reset.reset_angle = R_right.state.rad;
	
  	static uint8_t prev_keys_none = 1;
	TickType_t last_wake = xTaskGetTickCount();
	for(;;)
	{
		key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		key3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		key4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
 		
		uint8_t current_any = (key1 == GPIO_PIN_SET) || 
													(key2 == GPIO_PIN_SET) || 
													(key3 == GPIO_PIN_SET) || 
													(key4 == GPIO_PIN_SET);
		if (prev_keys_none && current_any)
		{
			ball_back_trigger = 1;
		}
		prev_keys_none = ! current_any;

		
		switch(ALLState)
		{
			case PLAN:
			ALLState = READY;

		break;
			case READY:
			if(trigger_lock == 0)
			{
				
				if(ball_back_trigger == 1 && trigger_lock == 0)
					{
						ALLState = FIRE;
						trigger_lock = 1;
					}
			}
			break;
			
			case FIRE:
			{
					TickType_t fire2_wake = xTaskGetTickCount();
					volatile uint8_t braking = 0;
							
					while(1)
					{
						if(!braking)
						{
							RobStrideMotionControl(&R_left, 0x01,
								R_left_expect.expect_torque,
								R_left_expect.expect_angle,
								R_left_expect.expect_omega,
								R_left_expect.kp,
								R_left_expect.kd);

							RobStrideMotionControl(&R_right, 0x02,
								R_right_expect.expect_torque,
								R_right_expect.expect_angle,
								R_right_expect.expect_omega,
								R_right_expect.kp,
								R_right_expect.kd);

							// 检测是否接近目标位置，触发刹车
							if(R_left.state.rad <= -0.32f && R_right.state.rad >= 0.34f)
							{
								left = R_left.state.rad;
								right = R_right.state.rad;
								
								braking = 1;
							}
						}
						else
						{
							// 停止输出
							RobStrideMotionControl(&R_left, 0x01, R_left_stop.torque, left, 0.0f, R_left_stop.kp, R_left_stop.kd);
							RobStrideMotionControl(&R_right, 0x02, R_right_stop.torque, right, 0.0f, R_right_stop.kp, R_right_stop.kd);

							// 检测电机速度是否接近0，退出
							if(fabs(R_left.state.omega) < 1.0f && fabs(R_right.state.omega) < 1.0f)
							{
								break;
							}
							
						}

						vTaskDelayUntil(&fire2_wake, pdMS_TO_TICKS(2));
					}

							
						vTaskDelay(600);

				last_wake = xTaskGetTickCount();
				ALLState = ALIGN;

				break;
			}
				
			case ALIGN:
			{
					RobStrideMotionControl(&R_left, 0x01,
						R_left_reset.reset_torque,
						R_left_reset.reset_angle,
						R_left_reset.reset_omega,
						R_left_reset.kp,
						R_left_reset.kd);

					RobStrideMotionControl(&R_right, 0x02,
						R_right_reset.reset_torque,
						R_right_reset.reset_angle,
						R_right_reset.reset_omega,
						R_right_reset.kp,
						R_right_reset.kd);

				// 等待机构稳定
				vTaskDelay(80);

				// 机构稳定后重新同步光电门状态
				key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
				key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
				key3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
				key4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
				uint8_t any = (key1 == GPIO_PIN_SET) || (key2 == GPIO_PIN_SET) || (key3 == GPIO_PIN_SET) || (key4 == GPIO_PIN_SET);
				prev_keys_none = !any;

				last_wake = xTaskGetTickCount();
				ALLState = PLAN;
				trigger_lock = 0;
				ball_back_trigger = 0;
				break;
			}
				
			default:
				break;
		}
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}

void send_flag(uint8_t val)
{
    static uint8_t tx_buf[3];

    tx_buf[0] = 0xAA;
    tx_buf[1] = val;
    tx_buf[2] = 0x55;

    if (huart4.gState == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(&huart4, tx_buf, sizeof(tx_buf));
    }
}
float R_up_angle = 0;
float L_up_angle = 0;
int16_t motorCurrentBuf[4] = {0};
Motor3508Ex_t Lift_Motor;
TaskHandle_t Hit_Task_Handle;
float first_angle = 0.0f;
float second_angle = 0.0f;
int rad_init = 0;
typedef enum {
    BALL_IDLE = 0,
	  BALL_R_UP,
    BALL_PREPARE,
    BALL_HIT,
    BALL_RESET
} BallState_t;

int time_prepare_far = 450; 

int time_prepare_middle = 300;

int time_prepare_near = 200;

void BallStateMachine(BallState_t *state, char *init_done, float target_angle, float home_angle, int prepare_time, int hit_time)
{
    static TickType_t state_start = 0;
    TickType_t now = xTaskGetTickCount();

    switch(*state)
    {
        case BALL_IDLE:
            if(*init_done)
            {
                state_start = now;
                *state = BALL_R_UP;
            }
            break;
        case BALL_R_UP:
					if((now - state_start) < pdMS_TO_TICKS(1000))
					{

					}
					else{
						*state = BALL_PREPARE;
						state_start = now;
					}
					
        case BALL_PREPARE:
            if ((now - state_start) < pdMS_TO_TICKS(prepare_time))
            {
                if(init_done_far ==1)
							send_flag(1);
								 if(init_done_middle ==1)
							send_flag(2);
								  if(init_done_near ==1)
							send_flag(3); 
            }
            else
            {
                *state = BALL_HIT;
                state_start = now;
            }
            break;

        case BALL_HIT:
            if ((now - state_start) < pdMS_TO_TICKS(hit_time))
            {
                PID_Control2(Lift_Motor.motor.Angle_DEG, target_angle, &Lift_Motor.pos_pid);
                PID_Control2(Lift_Motor.motor.Speed, Lift_Motor.pos_pid.pid_out, &Lift_Motor.vel_pid);
                motorCurrentBuf[2] = (int16_t)(Lift_Motor.vel_pid.pid_out * 1.5f);
                MotorSend(&hcan1, 0x200, motorCurrentBuf);
            }
            else
            {
                *state = BALL_RESET;
                state_start = now;
            }
            break;

        case BALL_RESET:
            if((now - state_start) < pdMS_TO_TICKS(2500))
            {
                PID_Control2(Lift_Motor.motor.Angle_DEG, home_angle, &Lift_Motor.pos_pid);
                PID_Control2(Lift_Motor.motor.Speed, Lift_Motor.pos_pid.pid_out, &Lift_Motor.vel_pid);
                motorCurrentBuf[2] = (int16_t)Lift_Motor.vel_pid.pid_out;
                MotorSend(&hcan1, 0x200, motorCurrentBuf);
            }
            else
            {
                *init_done = 0;
                *state = BALL_IDLE;
            }
            break;

        default:
            *state = BALL_IDLE;
            break;
    }
}
void Hit_Task(void *pvParameters)
{
    Lift_Motor.ID = 0x203;
    Lift_Motor.hcan = &hcan1;

    // PID 参数初始化
    Lift_Motor.pos_pid.Kp = 22.1f;
    Lift_Motor.pos_pid.Ki = 0.0f;
    Lift_Motor.pos_pid.Kd = 0.0f;
    Lift_Motor.pos_pid.limit = 10000.0f;
    Lift_Motor.pos_pid.output_limit = 9006.3f;

    Lift_Motor.vel_pid.Kp = 8.0f;
    Lift_Motor.vel_pid.Ki = 0.01f;
    Lift_Motor.vel_pid.Kd = 0.0f;
    Lift_Motor.vel_pid.limit = 10000.0f;
    Lift_Motor.vel_pid.output_limit = 16384.0f;

    TickType_t last_wake = xTaskGetTickCount();

    BallState_t far_state = BALL_IDLE;
    BallState_t middle_state = BALL_IDLE;
    BallState_t near_state = BALL_IDLE;

    while(1)
    {
			if (rad_init<100) { 
			first_angle = Lift_Motor.motor.Angle_DEG; 
			second_angle = first_angle + MOTOR_ANGLE_OFFSET_DEG; 
			rad_init++; 
			char flag = 0; 
			flag =1; 
			}
        BallStateMachine(&far_state, &init_done_far, second_angle, first_angle, time_prepare_far, UP_VOLLEY_MOTOR_MS);
        BallStateMachine(&middle_state, &init_done_middle, second_angle, first_angle, time_prepare_middle, UP_VOLLEY_MOTOR_MS);
        BallStateMachine(&near_state, &init_done_near, second_angle, first_angle, time_prepare_near, UP_VOLLEY_MOTOR_MS);
  
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance ==CAN1)
	{


		uint8_t Recv[8] = {0};

    if (hcan->Instance == CAN1)
    {
			
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, Recv);
//if(ID == 0x01) {
    RobStrideRecv_Handle(&R_left, &hcan1, ID, Recv);
//} else if(ID == 0x02) {
    RobStrideRecv_Handle(&R_right, &hcan1, ID, Recv);
//}
//else if(ID == 0x203)
//		{
	int c =		Motor3508Recv(&Lift_Motor,hcan, ID, Recv);
//		}
			}
}
	}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
VESC_ReceiveHandler(&vesc_1.steer, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&vesc_2.steer, &hcan2, ID,Recv);
VESC_ReceiveHandler(&vesc_3.steer, &hcan2, ID,Recv);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART4)
    {
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
   	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
	}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}