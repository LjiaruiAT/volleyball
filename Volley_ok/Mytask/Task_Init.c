#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "CANDrive.h"
#include "semphr.h"

void Task_Init()
{
	//遥控器
	//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	//    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
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
//	xTaskCreate(Move_Task,
//				"Move_Task",
//				200, NULL,
//				5,
//				&Move_Task_Handle);//遥控器任务

	xTaskCreate(Hit_Task,
			 "Hit_Task",
				400,
				NULL,
				4,
				&Hit_Task_Handle); 
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

//        if(slip1) vesc_1.PID.pid_out *= 0.5f;
//        if(slip2) vesc_2.PID.pid_out *= 0.5f;
//        if(slip3) vesc_3.PID.pid_out *= 0.5f;
//        vesc_1.PID.pid_out = vesc_1.PID.pid_out*3;
//				vesc_2.PID.pid_out = vesc_2.PID.pid_out*3;
//				vesc_3.PID.pid_out = vesc_3.PID.pid_out*3;
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

        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
//CubicParam_t cubic;       // 三次多项式参数
//TrajectoryState_t state;  // 存储当前位置、速度、加速度
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
Rm3508 rm3508;
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
rm3508.pos_pid_3508.Kp =0.0f;
rm3508.pos_pid_3508.Ki =0.0f;
rm3508.pos_pid_3508.Kd =0.0f;
rm3508.pos_pid_3508.limit =500.0f;
rm3508.pos_pid_3508.output_limit = 10000.0f;

rm3508.vel_pid_3508.Kp =0.0f;
rm3508.vel_pid_3508.Ki =0.0f;
rm3508.vel_pid_3508.Kd =0.0f;
rm3508.vel_pid_3508.limit =500.0f;
rm3508.vel_pid_3508.output_limit = 10000.0f;
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
			
//			uint32_t now = HAL_GetTick();

//    // 获取三次多项式轨迹状态
//    Cubic_GetFullState(&cubic, now, &state);

//    // 用 state.pos 和 state.vel 下发电机指令
//    GoMotorSend(&let_fly.go_volleyball,
//                let_fly.exp.exp_tor,  // 保持你的期望力矩
//                state.vel,            // 使用轨迹速度
//                state.pos,            // 使用轨迹位置
//                let_fly.exp.exp_kp,
//                let_fly.exp.exp_kd);
						Remote_Analysis();
			/* 单次触发 */
//			if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Right_Key_Up))
//			{
//					
//			}
			
			int e = GoMotorRecv(&let_fly.go_volleyball);
			GoMotorSend(&let_fly.go_volleyball,
                let_fly.exp.exp_tor,  // 保持你的期望力矩
               let_fly.exp.exp_vel,            // 使用轨迹速度
                let_fly.exp.exp_pos,            // 使用轨迹位置
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
uint8_t buf[8];
    if (hcan->Instance == CAN1)
    {
        uint32_t id = CAN_Receive_DataFrame(hcan, buf);
        Motor3508Recv(&rm3508.motor_3508, hcan, id, buf);
    }
}
