#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "semphr.h"
extern SemaphoreHandle_t remote_semaphore;

TaskHandle_t Move_Task_Handle;

ChassisMode chassis_mode = REMOTE;

void Task_Init()
{
	//Ò£¿ØÆ÷
	//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	//    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	
//	xTaskCreate(Remote,
//         "Remote",
//          400,
//          NULL,
//          4,
//          &Remote_Handle); 
//	
//	xTaskCreate(Move_Task,
//				"Move_Task",
//				200, NULL,
//				5,
//				&Move_Task_Handle);//Ò£¿ØÆ÷ÈÎÎñ

	xTaskCreate(Hit_Task,
			 "Hit_Task",
				400,
				NULL,
				4,
				&Hit_Task_Handle); 
}


void Move_Task(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	for(;;)
	{
			if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
			{
					memcpy(&RemoteData, usart4_dma_buff, sizeof(RemoteData));
					Updatakey(&Remote_Control);
					Remote_Control.Ex =-RemoteData.rocker[1];
					Remote_Control.Ey = RemoteData.rocker[0];
					Remote_Control.Eomega = RemoteData.rocker[2];
					Remote_Control.mode = RemoteData.rocker[3];
					Remote_Control.Key_Control = &RemoteData.Key;
			}else{
					Remote_Control.Ex = 0;
					Remote_Control.Ey = 0;
					Remote_Control.Eomega = 0;
					Remote_Control.mode = 0;
					//°´¼ü×´Ì¬ÇåÁã
					memset(&RemoteData.Key, 0, sizeof(hw_key_t));
					Remote_Control.Key_Control = &RemoteData.Key;
			}
	}
}

void Updatakey(Remote_Handle_t * xx) { //Ò£¿ØÆ÷Êý¾Ý¸üÐÂ
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
}

