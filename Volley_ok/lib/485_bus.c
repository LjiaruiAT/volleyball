#include "485_bus.h"

void RS485Init(RS485_t *rs_485,UART_HandleTypeDef *huart,GPIO_TypeDef *crtl_port,uint32_t ctrl_pin)
{
    rs_485->crtl_port=crtl_port;
    rs_485->ctrl_pin=ctrl_pin;
    rs_485->huart=huart;
    rs_485->recv_semphr=xSemaphoreCreateBinary();
    rs_485->send_semphr=xSemaphoreCreateBinary();
    HAL_GPIO_WritePin(crtl_port,ctrl_pin,GPIO_PIN_RESET);
}

uint32_t RS485Send(RS485_t *rs485,uint8_t *data,uint32_t size,uint32_t time_out)
{
		xSemaphoreTake(rs485->send_semphr,0);
    HAL_GPIO_WritePin(rs485->crtl_port,rs485->ctrl_pin,GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(rs485->huart,data,size);
    return xSemaphoreTake(rs485->send_semphr,pdMS_TO_TICKS(time_out));
}

uint32_t RS485Recv(RS485_t *rs485,uint8_t*data,uint32_t size,uint32_t time_out,uint32_t* recv_size)
{
		rs485->last_recv_size=0;
    xSemaphoreTake(rs485->recv_semphr,0);
		HAL_GPIO_WritePin(rs485->crtl_port,rs485->ctrl_pin,GPIO_PIN_RESET);     //总线转入读取模式
    //HAL_UART_Receive_DMA(rs485->huart,data,size);
		HAL_UART_DMAStop(rs485->huart);
    __HAL_UART_CLEAR_IDLEFLAG(rs485->huart);
		HAL_UARTEx_ReceiveToIdle_DMA(rs485->huart, data,size);
		__HAL_DMA_DISABLE_IT(rs485->huart->hdmarx, DMA_IT_HT);
		uint32_t ret=xSemaphoreTake(rs485->recv_semphr,pdMS_TO_TICKS(time_out));
    *recv_size=rs485->last_recv_size;
    return ret;
}

//放在发送完成中断中
uint32_t RS485SendIRQ_Handler(RS485_t *rs485,UART_HandleTypeDef *huart)
{
    if(rs485->huart->Instance==huart->Instance)
    {
        BaseType_t temp;
        HAL_GPIO_WritePin(rs485->crtl_port,rs485->ctrl_pin,GPIO_PIN_RESET);     //总线转入读取模式
        xSemaphoreGiveFromISR(rs485->send_semphr,&temp);
        portYIELD_FROM_ISR(temp);
        return 1;
    }
    return 0;
}

//放在接收完成中断中
uint32_t RS485RecvIRQ_Handler(RS485_t *rs485,UART_HandleTypeDef *huart,uint32_t size)
{
    if(rs485->huart->Instance==huart->Instance)
    {
        BaseType_t temp;
        rs485->last_recv_size=size;
        xSemaphoreGiveFromISR(rs485->recv_semphr,&temp);
        portYIELD_FROM_ISR(temp);
        return 1;
    }
    return 0;
}
