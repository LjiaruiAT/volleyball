#include "Task_Init.h"
#include <math.h>
#define inv_tor 0.4
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)
float unitree_F = 0;
float unitree_S_FAR = 0;
float unitree_S_MIDDLE = 0;
float unitree_S_NEAR = 0;
float unitree_T = 0;
float real_angle = 0;
static float twenty_to_real_pai(float angle)
{
	float temp = angle / 6.369426;
	return temp;	
}
static float NormalizeAngleRad(float angle)
{
    float value = fmodf(angle, TWO_PI_F);
    if (value < 0.0f)
    {
        value += TWO_PI_F;
    }
    return value;
}

static float GravityCompensatedTorque360(float angle_current,
                                         float angle_down,
                                         float torque_max)
{
    float diff = NormalizeAngleRad(angle_current - angle_down);
    if (diff > PI_F)
    {
        diff -= TWO_PI_F;
    }
    else if (diff < -PI_F)
    {
        diff += TWO_PI_F;
    }

    return torque_max * sinf(diff);
}

typedef struct
{
    float exp_tor;
    float exp_pos;
    float exp_vel;
    float exp_kp;
    float exp_kd;
} exp_param;

typedef struct {
    GO_MotorHandle_t motor; 
    float pos_offset;         
    float inv_motor;         
    float exp_rad;           
    float exp_omega;          
    float exp_torque;         
    float Kp;                 
    float Kd;                
} Joint_t;

typedef enum {
    BALL_FAR_IDLE = 0,       
    BALL_FAR_PREPARE,        
    BALL_FAR_HIT,            
    BALL_FAR_RESET           
} BallState_far_t;
typedef enum {
    BALL_MIDDLE_IDLE = 0,       
    BALL_MIDDLE_PREPARE,        
    BALL_MIDDLE_HIT,            
    BALL_MIDDLE_RESET           
} BallState_middle_t;
typedef enum {
    BALL_NEAR_IDLE = 0,       
    BALL_NEAR_PREPARE,        
    BALL_NEAR_HIT,            
    BALL_NEAR_RESET           
} BallState_near_t;

RS485_t rs485bus;
uint8_t dma1_send_buf[sizeof(GOMotor_SendPack_t)];
uint8_t dma1_recv_buf[sizeof(GOMotor_ReceivePack_t)];
Motor3508Ex_t Lift_Motor;
Joint_t let_fly = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}};
TaskHandle_t Hit_Task_Handle;
float balance_rad = 0;
float unitree_inv_back_far  = 25.00f;
float unitree_inv_back_middle  = 20.00f;
float unitree_inv_back_near  = 20.00f;
float unitree_inv_front = 15.00f;
float balance_inv = 0.0f;
char init_done_far = 0;
char init_done_middle = 0;
char init_done_near = 0;
float max_vel = -200;
int      ret       = 0;
uint32_t error_cnt = 0;
uint32_t success_cnt = 0;

CubicParam_t cubic;
TrajectoryState_t state;
float kkp = 0;
float kkd = 0;
static uint8_t uart7_dma_buf[64];
uint8_t bt_cmd = 0;
float test_angle = 0;
void Task_Init(void)
{
	vTaskDelay(1000);
    RS485Init(&rs485bus, &huart2, NULL, NULL, dma1_send_buf, dma1_recv_buf);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);

    xTaskCreate(Hit_Task,
                "Hit_Task",
                400,
                NULL,
                4,
                &Hit_Task_Handle);
}
float ini_rad = 0;
char init_rad = 0;
void Hit_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    BallState_far_t    ball_far_state    = BALL_FAR_IDLE;
    BallState_middle_t ball_middle_state = BALL_MIDDLE_IDLE;
    BallState_near_t   ball_near_state   = BALL_NEAR_IDLE;
    TickType_t state_start_far    = last_wake;
    TickType_t state_start_middle = last_wake;
    TickType_t state_start_near   = last_wake;

    let_fly.Kp = 4.0f;
    let_fly.Kd = 0.2f;
    let_fly.exp_torque = 0.4f;

    char rad_init_done = 0;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
			if(rad_init_done<=10)
			{
			
			
			GoMotorSend(&let_fly.motor,
                        let_fly.exp_torque,
                        0,
                        0,
                        let_fly.Kp,
                        let_fly.Kd);
            ret = GoMotorRecv(&let_fly.motor);
				            rad_init_done ++;
				 unitree_F = let_fly.motor.state.rad;
            unitree_S_FAR = unitree_F + unitree_inv_back_far;
					unitree_S_MIDDLE =unitree_F +unitree_inv_back_middle;
					unitree_S_NEAR = unitree_F + unitree_inv_back_near;
            unitree_T = unitree_F - unitree_inv_front;
				            ini_rad = twenty_to_real_pai(let_fly.motor.state.rad);
Cubic_SetTrajectory(&cubic,
																			let_fly.motor.state.rad, 0.0f,
																			unitree_S_FAR, 0.0f,
																			2.0f, HAL_GetTick());

			}
        if (init_done_far == 0 && init_done_middle == 0 && init_done_near == 0&&rad_init_done>10)
        {

					
					 if(cubic.is_running)
{
    Cubic_GetFullState(&cubic, HAL_GetTick(), &state);
    GoMotorSend(&let_fly.motor,
                let_fly.exp_torque,
                state.vel,
                state.pos,
                0.2f,
                0.05f);
    ret = GoMotorRecv(&let_fly.motor);
}
else
{
    // 保持在终点
    GoMotorSend(&let_fly.motor,
                let_fly.exp_torque,
                0.0f,
                unitree_S_FAR,
                0.2f,
                0.05f);
    ret = GoMotorRecv(&let_fly.motor);
}
           
        }

        real_angle = twenty_to_real_pai(let_fly.motor.state.rad);
        let_fly.exp_torque = GravityCompensatedTorque360(real_angle, ini_rad, inv_tor);

        switch (ball_far_state)
        {
            case BALL_FAR_IDLE:
                if (init_done_far) { state_start_far = now; ball_far_state = BALL_FAR_PREPARE; }
                break;
            case BALL_FAR_PREPARE:
                if ((now - state_start_far) < pdMS_TO_TICKS(500))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_S_FAR, let_fly.Kp, let_fly.Kd);
                else { ball_far_state = BALL_FAR_HIT; state_start_far = now; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
            case BALL_FAR_HIT:
                if ((now - state_start_far) < pdMS_TO_TICKS(1000))
                {
                    if (let_fly.motor.state.rad > unitree_T)
                        GoMotorSend(&let_fly.motor, let_fly.exp_torque, max_vel, unitree_T, 8, 0.2);
                    else
                        GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_T, 4, 0.1);
                }
                else { ball_far_state = BALL_FAR_RESET; state_start_far = now; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
            case BALL_FAR_RESET:
                if ((now - state_start_far) < pdMS_TO_TICKS(1000))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_F, 3, 0.1f);
                else { init_done_far = 0; ball_far_state = BALL_FAR_IDLE; GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_F, 0, 0); }
                ret = GoMotorRecv(&let_fly.motor);
                break;
        }

        switch (ball_middle_state)
        {
            case BALL_MIDDLE_IDLE:
                if (init_done_middle) { state_start_middle = now; ball_middle_state = BALL_MIDDLE_PREPARE; }
                break;
            case BALL_MIDDLE_PREPARE:
                if ((now - state_start_middle) < pdMS_TO_TICKS(500))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_S_MIDDLE, 4, 0.1);
                else { ball_middle_state = BALL_MIDDLE_HIT; state_start_middle = now; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
            case BALL_MIDDLE_HIT:
                if ((now - state_start_middle) < pdMS_TO_TICKS(1000))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, max_vel, unitree_T, 6, 0);
                else { ball_middle_state = BALL_MIDDLE_RESET; state_start_middle = now; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
            case BALL_MIDDLE_RESET:
                if ((now - state_start_middle) < pdMS_TO_TICKS(1000))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_F, 3, 0.1f);
                else { init_done_middle = 0; ball_middle_state = BALL_MIDDLE_IDLE; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
        }

        switch (ball_near_state)
        {
            case BALL_NEAR_IDLE:
                if (init_done_near) { state_start_near = now; ball_near_state = BALL_NEAR_PREPARE; }
                break;
            case BALL_NEAR_PREPARE:
                if ((now - state_start_near) < pdMS_TO_TICKS(500))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_S_NEAR, 4, 0.1);
                else { ball_near_state = BALL_NEAR_HIT; state_start_near = now; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
            case BALL_NEAR_HIT:
                if ((now - state_start_near) < pdMS_TO_TICKS(1000))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, max_vel, unitree_T, 6, 0);
                else { ball_near_state = BALL_NEAR_RESET; state_start_near = now; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
            case BALL_NEAR_RESET:
                if ((now - state_start_near) < pdMS_TO_TICKS(1000))
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_F, 3, 0.1f);
                else { init_done_near = 0; ball_near_state = BALL_NEAR_IDLE; }
                ret = GoMotorRecv(&let_fly.motor);
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART7)
    {
        if (size >= 3 &&
            uart7_dma_buf[0]        == 0xAA &&
            uart7_dma_buf[size - 1] == 0x55)
        {
            bt_cmd = uart7_dma_buf[1];
            if (bt_cmd == 0x01)
            {
                init_done_far = 1;
            }
						 if (bt_cmd == 0x02)
            {
                init_done_middle = 1;
            }
						 if (bt_cmd == 0x03)
            {
                init_done_near = 1;
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
    if (huart->Instance == USART2)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF |
                              UART_CLEAR_FEF  |
                              UART_CLEAR_NEF  |
                              UART_CLEAR_PEF);
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
        error_cnt++;
    }
    if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}
