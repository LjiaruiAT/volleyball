#ifndef COMM_STM32_HAL_MIDDLE_H
#define COMM_STM32_HAL_MIDDLE_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <string.h>
#include <stdint.h>

// 环形缓冲区大小
#define COMM_RING_BUFFER_SIZE 1024
// 发送队列大小
#define COMM_TX_QUEUE_SIZE 10
// 最大发送数据长度
#define COMM_MAX_TX_SIZE 256

// 发送请求结构体
typedef struct {
    uint8_t data[COMM_MAX_TX_SIZE];                    // 发送数据缓冲区
    uint16_t size;                                     // 数据长度
} CommTxRequest_t;

// 通信句柄结构体
typedef struct {
    UART_HandleTypeDef *huart;                          // HAL库UART句柄
    uint8_t rx_buffer[COMM_RING_BUFFER_SIZE];          // 接收环形缓冲区
    volatile uint16_t rx_head;                          // 接收缓冲区头指针
    volatile uint16_t rx_tail;                          // 接收缓冲区尾指针
    volatile uint16_t rx_count;                         // 接收缓冲区数据计数
    TaskHandle_t tx_task_handle;                       // 发送任务句柄
    QueueHandle_t tx_queue;                            // 发送请求队列
    SemaphoreHandle_t rx_semaphore;                    // 接收信号量（有新数据到达时 give）
    TaskHandle_t tx_wait_task;                         // 当前等待发送完成通知的任务（用于 task notify）
    volatile uint8_t tx_busy;                          // 发送忙标志（与 tx_wait_task 配合）
} CommHandle_t;

/*
 * 全局通信句柄（供兼容宏 uart_read_bytes/uart_write_bytes 使用）
 * 注意：请勿在其他 .c 文件里再次定义同名 static g_comm_handle（会遮蔽这里的全局变量）
 */
extern CommHandle_t* g_comm_handle;

// 核心接口函数
CommHandle_t* Comm_Init(UART_HandleTypeDef *huart);
void Comm_UART_IRQ_Handle(CommHandle_t* comm_handle, UART_HandleTypeDef *huart, uint8_t *src, uint16_t size);
void Comm_UART_TxCplt_IRQ_Handle(CommHandle_t* comm_handle, UART_HandleTypeDef *huart);

// 数据操作函数
uint16_t Comm_Available(CommHandle_t* comm_handle);
uint16_t Comm_Read(CommHandle_t* comm_handle, uint8_t *buffer, uint16_t size);
int Comm_Read_Timeout(CommHandle_t* comm_handle, uint8_t *buffer, uint16_t size, uint32_t timeout_ms);
void Comm_Write(CommHandle_t* comm_handle, const uint8_t *data, uint16_t size);

// 工具函数
uint32_t Comm_GetTickMS(void);

// 兼容ESP32的函数映射宏
#define uart_read_bytes(port, buffer, size, timeout_ms) Comm_Read_Timeout(g_comm_handle, buffer, size, timeout_ms)
#define uart_write_bytes(port, data, size) Comm_Write(g_comm_handle, data, size)

#endif // COMM_STM32_HAL_MIDDLE_H
