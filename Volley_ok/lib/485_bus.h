#ifndef __485_BUS_H__
#define __485_BUS_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    UART_HandleTypeDef *huart;   //硬件层声明
    GPIO_TypeDef *crtl_port;
    uint32_t ctrl_pin;

    QueueHandle_t send_semphr;   //通信层声明
    QueueHandle_t recv_semphr;
    uint32_t last_recv_size;
}RS485_t;


void RS485Init(RS485_t *rs_485,UART_HandleTypeDef *huart,GPIO_TypeDef *crtl_port,uint32_t ctrl_pin);
uint32_t RS485Send(RS485_t *rs485,uint8_t *data,uint32_t size,uint32_t time_out);
uint32_t RS485Recv(RS485_t *rs485,uint8_t*data,uint32_t size,uint32_t time_out,uint32_t* recv_size);


//以下两个函数要在中断中使用，分别是发送完成中断和接收空闲中断

uint32_t RS485SendIRQ_Handler(RS485_t *rs485,UART_HandleTypeDef *huart);

uint32_t RS485RecvIRQ_Handler(RS485_t *rs485,UART_HandleTypeDef *huart,uint32_t size);

#ifdef __cplusplus
}
#endif

#endif
