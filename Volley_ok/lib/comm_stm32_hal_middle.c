#include "comm_stm32_hal_middle.h"

// 静态通信句柄实例（当前实现仅支持单实例）
static CommHandle_t comm_instance;
CommHandle_t* g_comm_handle = NULL;

// 发送任务函数声明
static void Comm_TxTask(void *pvParameters);

static inline void Comm_RingPushFromISR(CommHandle_t* h, const uint8_t* src, uint16_t size);
static inline uint16_t Comm_RingPop(CommHandle_t* h, uint8_t* dst, uint16_t size);
static inline uint8_t Comm_ReadByte(CommHandle_t* h);

static inline void Comm_RingPushFromISR(CommHandle_t* h, const uint8_t* src, uint16_t size)
{
    if (h == NULL || src == NULL || size == 0) {
        return;
    }

    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    // 若空间不足：一次性丢弃最旧数据，腾出 size 空间
    uint16_t free_space = (uint16_t)(COMM_RING_BUFFER_SIZE - h->rx_count);
    if (size > free_space) {
        uint16_t drop = (uint16_t)(size - free_space);
        if (drop > h->rx_count) {
            drop = h->rx_count;
        }
        h->rx_tail = (uint16_t)((h->rx_tail + drop) % COMM_RING_BUFFER_SIZE);
        h->rx_count = (uint16_t)(h->rx_count - drop);
    }

    // 两段 memcpy 写入（处理 wrap）
    uint16_t first = (uint16_t)(COMM_RING_BUFFER_SIZE - h->rx_head);
    if (first > size) {
        first = size;
    }
    memcpy(&h->rx_buffer[h->rx_head], src, first);
    h->rx_head = (uint16_t)((h->rx_head + first) % COMM_RING_BUFFER_SIZE);

    uint16_t remain = (uint16_t)(size - first);
    if (remain > 0) {
        memcpy(&h->rx_buffer[h->rx_head], &src[first], remain);
        h->rx_head = (uint16_t)((h->rx_head + remain) % COMM_RING_BUFFER_SIZE);
    }

    h->rx_count = (uint16_t)(h->rx_count + size);

    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static inline uint16_t Comm_RingPop(CommHandle_t* h, uint8_t* dst, uint16_t size)
{
    if (h == NULL || dst == NULL || size == 0) {
        return 0;
    }

    uint16_t out = 0;

    taskENTER_CRITICAL();
    uint16_t available = h->rx_count;
    if (available == 0) {
        taskEXIT_CRITICAL();
        return 0;
    }
    if (size > available) {
        size = available;
    }

    // 两段 memcpy 读出（处理 wrap）
    uint16_t first = (uint16_t)(COMM_RING_BUFFER_SIZE - h->rx_tail);
    if (first > size) {
        first = size;
    }
    memcpy(dst, &h->rx_buffer[h->rx_tail], first);
    h->rx_tail = (uint16_t)((h->rx_tail + first) % COMM_RING_BUFFER_SIZE);

    uint16_t remain = (uint16_t)(size - first);
    if (remain > 0) {
        memcpy(&dst[first], &h->rx_buffer[h->rx_tail], remain);
        h->rx_tail = (uint16_t)((h->rx_tail + remain) % COMM_RING_BUFFER_SIZE);
    }

    h->rx_count = (uint16_t)(h->rx_count - size);
    out = size;
    taskEXIT_CRITICAL();

    return out;
}

static inline uint8_t Comm_ReadByte(CommHandle_t* h)
{
    uint8_t b = 0;
    (void)Comm_RingPop(h, &b, 1);
    return b;
}

/**
 * @brief 初始化通信模块
 * @param huart HAL库UART句柄指针
 * @return 通信句柄指针
 */
CommHandle_t* Comm_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return NULL;
    }

    // 初始化通信句柄
    memset(&comm_instance, 0, sizeof(comm_instance));
    comm_instance.huart = huart;

    // 创建FreeRTOS信号量和队列
    comm_instance.rx_semaphore = xSemaphoreCreateBinary();
    comm_instance.tx_queue = xQueueCreate(COMM_TX_QUEUE_SIZE, sizeof(CommTxRequest_t));

    if (comm_instance.rx_semaphore == NULL || comm_instance.tx_queue == NULL) {
        if (comm_instance.rx_semaphore != NULL) {
            vSemaphoreDelete(comm_instance.rx_semaphore);
        }
        if (comm_instance.tx_queue != NULL) {
            vQueueDelete(comm_instance.tx_queue);
        }
        return NULL;
    }

    // 设置全局句柄
    g_comm_handle = &comm_instance;

    // 创建发送任务
    BaseType_t tx_task_result = xTaskCreate(
        Comm_TxTask,                    // 任务函数
        "CommTx",                       // 任务名称
        512,                            // 堆栈大小（字）
        &comm_instance,                 // 任务参数
        6,                              // 任务优先级
        &comm_instance.tx_task_handle   // 任务句柄
    );

    if (tx_task_result != pdPASS) {
        vSemaphoreDelete(comm_instance.rx_semaphore);
        vQueueDelete(comm_instance.tx_queue);
        g_comm_handle = NULL;
        return NULL;
    }

    return &comm_instance;
}

/**
 * @brief UART中断处理函数（在HAL库的接收中断中调用）
 * @param comm_handle 通信句柄
 * @param huart HAL库UART句柄
 * @param src 接收到的数据缓冲区
 * @param size 接收到的数据长度
 */
void Comm_UART_IRQ_Handle(CommHandle_t* comm_handle, UART_HandleTypeDef *huart, uint8_t *src, uint16_t size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (comm_handle == NULL || huart == NULL || src == NULL || size == 0) {
        return;
    }

    // 检查是否是对应的UART
    if (comm_handle->huart != huart) {
        return;
    }

    Comm_RingPushFromISR(comm_handle, src, size);

    // 通知有新数据到达
    if (comm_handle->rx_semaphore != NULL) {
        xSemaphoreGiveFromISR(comm_handle->rx_semaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 获取缓冲区中可读数据的数量
 * @param comm_handle 通信句柄
 * @return 可读数据数量
 */
uint16_t Comm_Available(CommHandle_t* comm_handle)
{
    if (comm_handle == NULL) {
        return 0;
    }

    taskENTER_CRITICAL();
    uint16_t c = comm_handle->rx_count;
    taskEXIT_CRITICAL();
    return c;
}

/**
 * @brief 从缓冲区读取指定数量的数据（带互斥锁保护）
 * @param comm_handle 通信句柄
 * @param buffer 目标缓冲区
 * @param size 要读取的数据数量
 * @return 实际读取的数据数量
 */
uint16_t Comm_Read(CommHandle_t* comm_handle, uint8_t *buffer, uint16_t size)
{
    if (comm_handle == NULL || buffer == NULL || size == 0) {
        return 0;
    }

    return Comm_RingPop(comm_handle, buffer, size);
}

/**
 * @brief 带超时的读取函数（兼容原ESP32函数）
 * @param comm_handle 通信句柄
 * @param buffer 目标缓冲区
 * @param size 要读取的数据数量
 * @param timeout_ms 超时时间（毫秒）
 * @return 实际读取的数据数量，-1表示超时
 */
int Comm_Read_Timeout(CommHandle_t* comm_handle, uint8_t *buffer, uint16_t size, uint32_t timeout_ms)
{
    if (comm_handle == NULL || buffer == NULL || size == 0) {
        return -1;
    }

    const uint32_t start_time = Comm_GetTickMS();
    uint16_t read_count = 0;

    while (read_count < size) {
        // 先尽可能多读（批量），减少临界区进入次数
        uint16_t got = Comm_Read(comm_handle, &buffer[read_count], size - read_count);
        if (got > 0) {
            read_count += got;
            continue;
        }

        // 没有数据可读：需要等待
        TickType_t wait_ticks = portMAX_DELAY;
        if (timeout_ms != portMAX_DELAY) {
            uint32_t elapsed = Comm_GetTickMS() - start_time;
            if (elapsed >= timeout_ms) {
                // 超时：按约定返回 -1
                return -1;
            }
            wait_ticks = pdMS_TO_TICKS(timeout_ms - elapsed);
        }

        if (xSemaphoreTake(comm_handle->rx_semaphore, wait_ticks) != pdTRUE) {
            // 超时：按约定返回 -1
            return -1;
        }
    }

    return (int)read_count;
}

/**
 * @brief 发送数据（使用队列提交方式，立即返回）
 * @param comm_handle 通信句柄
 * @param data 要发送的数据
 * @param size 数据长度
 */
void Comm_Write(CommHandle_t* comm_handle, const uint8_t *data, uint16_t size)
{
    if (comm_handle == NULL || comm_handle->huart == NULL || data == NULL || size == 0) {
        return;
    }
    
    // 检查数据长度是否超过最大限制
    if (size > COMM_MAX_TX_SIZE) {
        return;
    }
    
    // 创建发送请求
    CommTxRequest_t tx_request;
    memcpy(tx_request.data, data, size);
    tx_request.size = size;
    
    // 将发送请求放入队列，如果队列满了则丢弃（非阻塞）
    if (comm_handle->tx_queue != NULL) {
        xQueueSend(comm_handle->tx_queue, &tx_request, 0);
    }
}

/**
 * @brief UART发送完成中断处理函数（在HAL库的发送完成中断中调用）
 * @param comm_handle 通信句柄
 * @param huart HAL库UART句柄
 */
void Comm_UART_TxCplt_IRQ_Handle(CommHandle_t* comm_handle, UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (comm_handle == NULL || huart == NULL) {
        return;
    }

    if (comm_handle->huart != huart) {
        return;
    }

    comm_handle->tx_busy = 0;

    // 用 task notify 代替 tx_complete_semaphore
    if (comm_handle->tx_wait_task != NULL) {
        vTaskNotifyGiveFromISR(comm_handle->tx_wait_task, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 获取系统时间（毫秒）
 * @return 系统时间（毫秒）
 */
uint32_t Comm_GetTickMS(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
 * @brief 发送任务（处理发送队列中的请求）
 * @param pvParameters 任务参数（CommHandle_t指针）
 */
static void Comm_TxTask(void *pvParameters)
{
    CommHandle_t* comm_handle = (CommHandle_t*)pvParameters;
    CommTxRequest_t tx_request;

    if (comm_handle == NULL) {
        vTaskDelete(NULL);
        return;
    }

    // 当前发送任务就是等待发送完成通知的任务
    comm_handle->tx_wait_task = xTaskGetCurrentTaskHandle();

    while (1) {
        if (xQueueReceive(comm_handle->tx_queue, &tx_request, portMAX_DELAY) == pdTRUE) {
            // 等待上一次发送完成
            while (comm_handle->tx_busy) {
                (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }

            comm_handle->tx_busy = 1;

            HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(comm_handle->huart,
                                                            tx_request.data,
                                                            tx_request.size);

            if (status != HAL_OK) {
                comm_handle->tx_busy = 0;
            }
        }
    }
}
