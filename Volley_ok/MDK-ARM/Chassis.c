#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"

 //电机驱动
VESC_t steering1={
	.motor_id=0x01,
	.hcan = &hcan2,

  
};
VESC_t steering2={ 
	.motor_id=0x02,
	.hcan = &hcan2,

};
VESC_t steering3={
	.motor_id=0x03,
	.hcan = &hcan2,
         
};
VESC_t steering4={
	.motor_id=0x04,
	.hcan = &hcan2,

};

PID2 vesc1;
PID2 vesc2;
PID2 vesc3;
PID2 vesc4;

uint8_t flag = 0;


float Vx =0;   //前后移动
float Vy =0;   //左右移动
float Wz =0;   //顺逆自转

//该变量的值可能会被程序外的因素（如硬件、其他线程）修改
volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;
volatile float v4 = 0.0f;

volatile float wheel_one = 0.0f;  //前左
volatile float wheel_two = 0.0f;  //前右
volatile float wheel_three=0.0f;  //后右
volatile float wheel_four =0.0f;  //后左

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	  portTickType xLastWakeTime = xTaskGetTickCount();

	vesc1.Kp =0.950f;
	vesc1.Ki = 0.0001f;
	vesc1.Kd = 4.220f;
	vesc1.limit = 100000.0f;
	vesc1.output_limit = 60.0f;
	vesc2.Kp =0.950f;
	vesc2.Ki = 0.0001f;
	vesc2.Kd = 4.220f;
	vesc2.limit = 100000.0f;
	vesc2.output_limit = 60.0f;
	vesc3.Kp =0.950f;
	vesc3.Ki = 0.0001f;
	vesc3.Kd = 4.220f;
	vesc3.limit = 100000.0f;
	vesc3.output_limit = 60.0f;
	vesc4.Kp =0.950f;
	vesc4.Ki = 0.0001f;
	vesc4.Kd = 4.220f;
	vesc4.limit = 100000.0f;
	vesc4.output_limit = 60.0f;

	for(;;)
	{

			Vx = - remote_to_velocity(&Remote_Control.Ex);
			Vy = - remote_to_velocity(&Remote_Control.Ey);
    	    Wz =   remote_to_Omega(&Remote_Control.Eomega);

			v1 = -(sqrt(2.0f)/2.0)*(Vx + Vy + 2*LENGTH*Wz);;
			v2 = (sqrt(2.0f)/2.0)*(Vx - Vy- 2*LENGTH*Wz);
			vTaskDelay(1);
			v3 = -(sqrt(2.0f)/2.0)*(Vx + Vy- 2*LENGTH*Wz);
			v4 = (sqrt(2.0f)/2.0)*(-Vx + Vy-2*LENGTH*Wz);
			
			wheel_one=  (int16_t)((v1 / (2.0f * PI * WHEEL_RADIUS)) );
			wheel_two=  (int16_t)((v2 / (2.0f * PI * WHEEL_RADIUS)) );
			wheel_three=-(int16_t)((v3 /(2.0f * PI * WHEEL_RADIUS)) );
			wheel_four =(int16_t)((v4 / (2.0f * PI * WHEEL_RADIUS)) );
			
			PID_Control2((float)(steering1.epm / 7.0f/(3.4f)), (wheel_one   ), &vesc1);
			PID_Control2((float)(steering2.epm / 7.0f/(3.4f)), (wheel_two   ), &vesc2);
			PID_Control2((float)(steering3.epm / 7.0f/(3.4f)), (wheel_three ), &vesc3);
			PID_Control2((float)(steering4.epm / 7.0f/(3.4f)), (wheel_four  ), &vesc4);

//            VESC_SetCurrent(&steering1, vesc1.pid_out);
//            VESC_SetCurrent(&steering2, vesc2.pid_out);
//	          vTaskDelay(1);
//            VESC_SetCurrent(&steering3, vesc3.pid_out);
//            VESC_SetCurrent(&steering4, vesc4.pid_out);  
			
		    vTaskDelayUntil(&xLastWakeTime,2);
		
	}
}

bool is_remote_active(void)
{
    return abs(Remote_Control.mode) > 10;
}

float remote_to_velocity(int16_t *remote_value) {
    if (*remote_value == 0) 
	{
        return 0.0f;
    }
	int16_t rv=((float)*remote_value / 2047.0f) * MAX_VELOCITY;
    return rv;
}

float remote_to_Omega(int16_t *remote_value) {
    if (*remote_value == 0)
	{
        return 0.0f;
    }
	int16_t ro=((float)*remote_value / 2047.0f) * MAX_OMEGA;
    return ro;
}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	uint8_t Recv[8] = {0};
//	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
//	VESC_ReceiveHandler(&steering1, &hcan2, ID,Recv);
//	VESC_ReceiveHandler(&steering2, &hcan2, ID,Recv);
//	VESC_ReceiveHandler(&steering3, &hcan2, ID,Recv);
//	VESC_ReceiveHandler(&steering4, &hcan2, ID,Recv);
//    }
