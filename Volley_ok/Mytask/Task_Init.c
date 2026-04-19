#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "CANDrive.h"
#include "semphr.h"
#include "RobStride2.h"
#include "step.h"

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
//	


	xTaskCreate(Hit_Task,
			 "Hit_Task",
				2558,
				NULL,
				4,
				&Hit_Task_Handle); 
    xTaskCreate(Back_Task,
			 "Back_Task",
				400,
				NULL,
				4,
				&Back_Task_Handle); 
}
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
void Remote_Analysis()
{
	/* 1. 保存上一帧 */
	Remote_Control.Second = Remote_Control.First;
	/* 2. 解析当前按键 */
	Key_Parse(recv_pack.Key, &Remote_Control.First);
	
	Remote_Control.Ex = recv_pack.rocker[1] / 1977.0f *MAX_ROBOT_VEL;
	Remote_Control.Ey = recv_pack.rocker[0] / 1798.0f *MAX_ROBOT_VEL;
	Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
}
void Rocker_Filter(PackControl_t *data)
{
    float alpha = 0.6f;

    for(int i = 0; i < 4; i++)
    {
        rocker_filter[i] = alpha * data->rocker[i] +
                          (1.0f - alpha) * rocker_filter[i];

        data->rocker[i] = rocker_filter[i];
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    Rocker_Filter(&recv_pack);
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
int a = 0;
void Remote(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
      g_comm_handle = Comm_Init(&huart5);
      RemoteCommInit(NULL);
      register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
    vesc_1.PID.Kp = 0.3f;
   	vesc_1.PID.Ki = 0.0005f;
	vesc_1.PID.Kd = 1.5f;
    vesc_1.PID.limit = 10000.0f;
	vesc_1.PID.output_limit = 40.0f;
    vesc_2.PID.Kp =0.3f;
	vesc_2.PID.Ki = 0.0005f;
  	vesc_2.PID.Kd = 1.5f;
    vesc_2.PID.limit = 10000.0f;
  	vesc_2.PID.output_limit = 40.0f;
    vesc_3.PID.Kp =0.3f;
	vesc_3.PID.Ki = 0.0005f;
  	vesc_3.PID.Kd = 1.5f;
    vesc_3.PID.limit = 100000.0f;
	vesc_3.PID.output_limit = 40.0f;
    for(;;)
    {
			Remote_Analysis();
        v1 = -Remote_Control.Ex*0.5f - Remote_Control.Ey*(sqrt(3.0f)/2.0) + LENGTH * Remote_Control.Eomega;
        v2 = -Remote_Control.Ex*0.5f + Remote_Control.Ey*(sqrt(3.0f)/2.0) + LENGTH * Remote_Control.Eomega;
        v3 =  Remote_Control.Eomega + LENGTH * Wz;

        wheel_one   = -(v1 / (2.0f * PI * WHEEL_RADIUS));
        wheel_two   =  (v2 / (2.0f * PI * WHEEL_RADIUS));
        wheel_three = -(v3 / (2.0f * PI * WHEEL_RADIUS));

        float wheel1_actual = (float)vesc_1.steer.epm / 7.0f / 3.4f;
        float wheel2_actual = (float)vesc_2.steer.epm / 7.0f / 3.4f;
        float wheel3_actual = (float)vesc_3.steer.epm / 7.0f / 3.4f;

        PID_Control2(wheel1_actual, wheel_one, &vesc_1.PID);
        PID_Control2(wheel2_actual, wheel_two, &vesc_2.PID);
        PID_Control2(wheel3_actual, wheel_three, &vesc_3.PID);
        
        VESC_SetCurrent(&vesc_1.steer, vesc_1.PID.pid_out);
        VESC_SetCurrent(&vesc_2.steer, vesc_2.PID.pid_out);
        VESC_SetCurrent(&vesc_3.steer, vesc_3.PID.pid_out);
			if(recv_pack.rocker[0] == 0 && recv_pack.rocker[1] == 0 &&recv_pack.rocker[2] == 0 )
			{
                Remote_Control.Ex = 0;
				Remote_Control.Ey = 0;
				Remote_Control.Eomega = 0;

				//按键状态清零
				memset(&recv_pack.Key, 0, sizeof(uint32_t));
		 	}

        if(KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Switch_Up))
            chassis_mode = REMOTE;
        if(KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Switch_Down))
            chassis_mode = AUTO;
			if(a ==1)
                 {
					send_flag(0x01);
				 }
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}




int16_t feel_1 = 0;
int16_t feel_2 = 0;
int16_t feel_3 = 0;
int16_t feel_4 = 0;
typedef enum {
    BALL_IDLE = 0,
    BALL_PREPARE,
    BALL_HIT_LETF,
    BALL_RESET
} BallState_t;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}
RobStride_Expect R_left_expect = {
	.expect_angle = -0.355f,
	.expect_omega = 0.0f,
	.expect_torque = -3.3f,
	.kp = 280.0f,
	.kd = 8.0f

};
RobStride_Expect R_right_expect = {
	.expect_angle = 0.39f,
	.expect_omega = 0.0f,
	.expect_torque = 3.3f,
	.kp = 280.0f,
	.kd = 8.0f
};

RobStride_Reset R_left_reset = {
	.reset_angle = 0.0f,
	.reset_omega = 0.0f,
	.reset_torque = 0.5f,
	.kp = 10.0f,
	.kd = 1.0f
};
RobStride_Reset R_right_reset = {
	.reset_angle = 0.0f,
	.reset_omega = 0.0f,
	.reset_torque = -0.5f,
	.kp = 10.0f,
	.kd = 1.0f
};

RobStride_t R_left;
RobStride_t R_right;

CubicParam_t traj_left;
CubicParam_t traj_right;

TrajectoryState_t traj_left_state;
TrajectoryState_t traj_right_state;

//RobStride_Expect R_left_expect;
//RobStride_Expect R_right_expect;

uint8_t flag = 0;// 复位标志
static uint8_t ball_back_trigger = 0;// 击球标志
static uint8_t traj_started = 0;// 轨迹规划开始标志
static GPIO_PinState key1, key2, key3, key4;
float time = 0.25f;// 轨迹规划时间

void Back_Task(void *pvParameters)
{
		vTaskDelay(3000);
    RobStrideInit(&R_left, &hcan1, 0x01, RobStride_MotionControl, RobStride_04);
	  RobStrideInit(&R_right, &hcan1, 0x02, RobStride_MotionControl, RobStride_04);
	  vTaskDelay(100);
	  RobStrideSetMode(&R_left, RobStride_MotionControl);
	  RobStrideSetMode(&R_right, RobStride_MotionControl);
	  vTaskDelay(100);
    RobStrideEnable(&R_left);
	  RobStrideEnable(&R_right);
	  vTaskDelay(100);

    RobStrideResetAngle(&R_left);
    RobStrideResetAngle(&R_right);
	
	  R_left_reset.reset_angle = R_left.state.rad;
	  R_right_reset.reset_angle = R_right.state.rad;
	
	TickType_t last_wake = xTaskGetTickCount();
	for(;;)
	{
		key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		key3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		key4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		if(key1 == GPIO_PIN_SET || key2 == GPIO_PIN_SET || key3 == GPIO_PIN_SET || key4 == GPIO_PIN_SET)
			{
				ball_back_trigger = 1;
			}

		if (ball_back_trigger == 1 && traj_started == 0)   // 一次触发
		{
				Cubic_SetTrajectory(
						&traj_left,
						R_left.state.rad,        // 当前真实角度
						R_left.state.omega,      // 当前真实速度
						R_left_expect.expect_angle,          // 目标角度
						0,
						time,                  
						xTaskGetTickCount()
				);

				Cubic_SetTrajectory(
						&traj_right,
						R_right.state.rad,
						R_right.state.omega,
						R_right_expect.expect_angle,
						0,
						time,
						xTaskGetTickCount()
				);
				traj_started = 1;
		}

		if(ball_back_trigger == 1)
		{
		 if (traj_left.is_running || traj_right.is_running)
		 {
			Cubic_GetFullState(&traj_left,  xTaskGetTickCount(), &traj_left_state);
			Cubic_GetFullState(&traj_right, xTaskGetTickCount(), &traj_right_state);
				
			RobStrideMotionControl(&R_left, 0x01, 
			R_left_expect.expect_torque, 
			traj_left_state.pos,
			traj_left_state.vel,
			R_left_expect.kp,
			R_left_expect.kd);

			RobStrideMotionControl(&R_right, 0x02, 
			R_right_expect.expect_torque, 
			traj_right_state.pos,
			traj_right_state.vel,
			R_right_expect.kp,
			R_right_expect.kd);
		 }
			else
			{
			RobStrideMotionControl(&R_left, 0x01,
			0.0f, traj_left.target_pos, 0.0f,
			R_left_expect.kp, R_left_expect.kd);

			RobStrideMotionControl(&R_right, 0x02,
			0.0f, traj_right.target_pos, 0.0f,
			R_right_expect.kp, R_right_expect.kd);
				
			traj_started = 0;
			ball_back_trigger = 0;
			}
		}

		if( flag == 0 && ball_back_trigger == 0)
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
//			RobStrideMotionControl(&R_left, 0x01, 0, R_left_reset.reset_angle, 0, 0, 0);
//			RobStrideMotionControl(&R_right, 0x02, 0, R_right_reset.reset_angle, 0, 0, 0);
			}
			
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
uint8_t buf[8];
    if (hcan->Instance == CAN2)
    {
	uint32_t ID = CAN_Receive_DataFrame(&hcan2, buf);
	RobStrideRecv_Handle(&R_left, &hcan2, ID, buf);
  RobStrideRecv_Handle(&R_right, &hcan2, ID, buf);

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

CubicParam_t cubic;       // 三次多项式参数
TrajectoryState_t state;  // 存储当前位置、速度、加速度
int take = 0;
int ready = 2;

uint32_t error_cnt = 0;
uint32_t last_error_time = 0;
ErrorStats_t error_stats = {0};
QueueHandle_t cdc_recv_semphr;
uint32_t err_timer_cnt = 0;
uint32_t bad_Motor = 0;
int err_check = 0;


exp_param go_volley = {0};
RS485_t rs485bus;
exp_param exp_3508;
QueueHandle_t cdc_recv_semphr;
int16_t can_send_buf[4];
push let_fly=
{
.go_volleyball.motor_id = 0x01,
.go_volleyball.rs485 = &rs485bus
};
TaskHandle_t Hit_Task_Handle;
void Hit_Task(void *pvParameters)
{
	



TickType_t Last_wake_time = xTaskGetTickCount();
for(;;)
	{
		if(ready == 2 &&take == 0)
		{
		GoMotorSend(&let_fly.go_volleyball,
                0,  // 保持你的期望力矩
                0,            // 使用轨迹速度
                0,            // 使用轨迹位置
                0,
                0);
			int ret = GoMotorRecv(&let_fly.go_volleyball);
		}
		if (take == 0 && ready == 0)
		{
			int16_t cur_motor_pos = let_fly.go_volleyball.state.rad; // 获取当前位置
      float cur_pos_rad = cur_motor_pos * 2.0f * M_PI / 32768.0f; // 转成弧度
			// 设置三次多项式轨迹
         Cubic_SetTrajectory(&cubic, 
                    cur_pos_rad, 0.0f,       // 当前位姿和速度
                    let_fly.exp.exp_pos, 0.0f, // 目标位姿和目标速度
                    0.2f, HAL_GetTick());    // 持续时间 0.2s (可调)，当前时间
			
			ready = 1;
		}
		if (take == 1)
		{ 
			
			uint32_t now = HAL_GetTick();

    // 获取三次多项式轨迹状态
    Cubic_GetFullState(&cubic, now, &state);

    // 用 state.pos 和 state.vel 下发电机指令
    GoMotorSend(&let_fly.go_volleyball,
                let_fly.exp.exp_tor,  // 保持你的期望力矩
                state.vel,            // 使用轨迹速度
                state.pos,            // 使用轨迹位置
                let_fly.exp.exp_kp,
                let_fly.exp.exp_kd);
			
		}
//	PID_Control2(rm3508.motor_3508.motor.MchanicalAngle,exp_3508.exp_pos,&rm3508.pos_pid_3508);
//  PID_Control2(rm3508.motor_3508.motor.Speed,rm3508.pos_pid_3508.pid_out,&rm3508.vel_pid_3508);
//  can_send_buf[0]=(int16_t)rm3508.pos_pid_3508.pid_out;
//  MotorSend(&hcan1,0x200,can_send_buf);
//  HAL_Delay(200);//电机松手，排球自由落体，这时宇树电机击球
//	HAL_Delay(5000);
	//GoMotorSend这里宇树电机复位
//	HAL_Delay(1000);
	//MotorSend这里3508电机复位
	GoMotorSend(&let_fly.go_volleyball,
                0,  // 保持你的期望力矩
                0,            // 使用轨迹速度
                0,            // 使用轨迹位置
                0,
                0);
			int ret = GoMotorRecv(&let_fly.go_volleyball);

	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART6)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
        err_timer_cnt=0;   
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint32_t now = HAL_GetTick();       
        if ((now - error_stats.last_error_time) < 10) { // 10ms???δ???
            error_stats.continuous_errors++;
        } else {
            error_stats.continuous_errors = 0; 
        }
        error_stats.last_error_time = now;
        
        HAL_UART_DMAStop(huart);
        
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {

            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); 
            
            if (isrflags & USART_SR_ORE) {
                error_stats.overrun++;
            }
            if (isrflags & USART_SR_NE) {
                error_stats.noise++;
            }
            if (isrflags & USART_SR_FE) {
                error_stats.frame++;
            }
        }
        
        if (isrflags & USART_SR_PE) {

            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            error_stats.parity++;
        }
        error_stats.total++;
        error_cnt = error_stats.total; 
        last_error_time = now;
        RS485RecvIRQ_Handler(&rs485bus, huart, 0);
    }
}







