#include "hit_ball.h"
// 添加错误统计结构
uint32_t error_cnt = 0;
uint32_t last_error_time = 0;
ErrorStats_t error_stats = {0};
exp_param go_volley = {0};
uint32_t err_timer_cnt = 0;
RS485_t rs485bus;
QueueHandle_t cdc_recv_semphr;
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
		
	GoMotorSend(&let_fly.go_volleyball,let_fly.exp.exp_tor,let_fly.exp.exp_vel,let_fly.exp.exp_pos,let_fly.exp.exp_kp,let_fly.exp.exp_kd);
	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
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
        err_timer_cnt=0;    //每接收一次，就清零
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint32_t now = HAL_GetTick();
        
        // 检查错误频率，但不进行复位
        if ((now - error_stats.last_error_time) < 10) { // 10ms内多次错误
            error_stats.continuous_errors++;
        } else {
            error_stats.continuous_errors = 0; // 重置连续错误计数
        }
        error_stats.last_error_time = now;
        
        // 先停止DMA传输，避免在清除错误标志时产生新的中断或错误
        HAL_UART_DMAStop(huart);
        
        // 重置HAL状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        
        // 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
            // 对于ORE、NE、FE错误，需要先读SR再读DR
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE
            
            // 统计具体的错误类型
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
            // 奇偶校验错误只需读SR即可清除
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            error_stats.parity++;
        }
        
        // 增加总错误计数
        error_stats.total++;
        error_cnt = error_stats.total; // 保持与原变量的兼容性
        
        last_error_time = now;
        
        RS485RecvIRQ_Handler(&rs485bus, huart, 0);
    }
}
