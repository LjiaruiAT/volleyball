#include "hit_ball.h"
// ���Ӵ���ͳ�ƽṹ
uint32_t error_cnt = 0;
uint32_t last_error_time = 0;
ErrorStats_t error_stats = {0};
exp_param go_volley = {0};
uint32_t err_timer_cnt = 0;
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
	PID_Control2(rm3508.motor_3508.MchanicalAngle,exp_3508.exp_pos,&rm3508.pos_pid_3508);
    PID_Control2(rm3508.motor_3508.Speed,rm3508.pos_pid_3508.pid_out,&rm3508.vel_pid_3508);
    can_send_buf[0]=(int16_t)rm3508.pos_pid_3508.pid_out;
    MotorSend(&hcan1,0x200,can_send_buf[0]);
    HAL_Delay(500);
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
        err_timer_cnt=0;    //ÿ����һ�Σ�������
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint32_t now = HAL_GetTick();
        
        // ������Ƶ�ʣ��������и�λ
        if ((now - error_stats.last_error_time) < 10) { // 10ms�ڶ�δ���
            error_stats.continuous_errors++;
        } else {
            error_stats.continuous_errors = 0; // ���������������
        }
        error_stats.last_error_time = now;
        
        // ��ֹͣDMA���䣬��������������־ʱ�����µ��жϻ����
        HAL_UART_DMAStop(huart);
        
        // ����HAL״̬
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // Ȼ����������־ - ����STM32F4�ο��ֲ�Ҫ���˳��
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        
        // ��˳�������ִ����־�������ȶ�SR�ٶ�DR���������
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
            // ����ORE��NE��FE������Ҫ�ȶ�SR�ٶ�DR
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // �����ȡ�����ORE��NE��FE
            
            // ͳ�ƾ���Ĵ�������
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
            // ��żУ�����ֻ���SR�������
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            error_stats.parity++;
        }
        
        // �����ܴ������
        error_stats.total++;
        error_cnt = error_stats.total; // ������ԭ�����ļ�����
        
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
        Motor3508Recv(&rm3508, hcan, id, buf);
}
}

