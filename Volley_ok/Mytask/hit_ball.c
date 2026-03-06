#include "hit_ball.h"
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
        if ((now - error_stats.last_error_time) < 10) { // 10ms�ڶ�δ���
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
uint8_t buf[8];
    if (hcan->Instance == CAN1)
    {
        uint32_t id = CAN_Receive_DataFrame(hcan, buf);
        Motor3508Recv(&rm3508.motor_3508, hcan, id, buf);
    }
}
