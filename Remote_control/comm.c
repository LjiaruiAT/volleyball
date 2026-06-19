#include "comm.h"
#include "dataFrame.h"
#include "usart.h"
#include "comm_stm32_hal_middle.h"
#include "My_list.h"
#include "data_poll.h"

typedef struct
{
    uint16_t size;
    uint8_t cmd;
    uint8_t *data;
    uint8_t max_retry_cnt;
    uint32_t timeout_ms;
    CommPackSend_Cb finished_cb;
    void *user_data;
} DataTransReq_t;

typedef struct
{
    CommPackRecv_Cb callback;
    void *user_data;
    int id;
    uint8_t cmd;
} PackDealFunc_t;

typedef struct
{
    SemaphoreHandle_t semphr_handle;
    StaticQueue_t queue_data;
} StaticSemphrBlock_t;

typedef struct
{
    uint32_t timeout_ms;
    int64_t send_time_ms;
    uint8_t is_using;
    uint32_t pack_id;
    CommPackSend_Cb finished_cb;
    void *user_data;

    // ACK超时重传信息（零拷贝：data 指向外部缓冲，须确保生命周期覆盖整个ACK等待/重试过程）
    uint16_t size;
    uint8_t cmd;
    uint8_t *data;
    uint8_t retry_cnt;
} AckWaitBlock_t;

// 接收回调链表
static MyList_t *kRecvCbList;
// 回调函数ID
static uint32_t kCallbackId;

typedef enum
{
    COMM_BAD_HEAD = 1,
    COMM_BAD_SUM  = 2,
    COMM_BAD_LEN  = 3,
    COMM_BAD_ACK  = 4,
} CommBadType_t;

// 数据包发送请求队列
static QueueHandle_t send_req_queue_handle;
// 串口发送互斥锁（用于保护 uart_write_bytes 原子性）
static SemaphoreHandle_t uart_tx_mutex;
// 接收回调链表互斥锁（保护 kRecvCbList 的增删与遍历）
static SemaphoreHandle_t recv_cb_list_mutex;
// 数据包ACK确认信号量池
static DataPoll_t kCommSendAckSemphrPoll;
// 包ID，用于区分不同的数据包
static uint32_t g_pack_id = 1;

/* 发送与接收 buffer */
static uint8_t send_buffer[PACK_MAX_SIZE];
static uint8_t recv_buffer[PACK_MAX_SIZE];

// 挂起的待确认包信息
static AckWaitBlock_t send_ack_blocks[8];
static SemaphoreHandle_t send_ack_blocks_mutex;

/* -------------------- 包头定义 -------------------- */


void SendDataPackTask(void *param);
void ReceiveDataPackTask(void *param);
void ACKTimeoutCheckTask(void *param);

TaskHandle_t SendDataPackTask_handle;
TaskHandle_t ReceiveDataPackTask_handle;
TaskHandle_t ACKTimeoutCheckTask_handle;

/* -------------------- 模块初始化 -------------------- */
void RemoteCommInit(BadDataPackCb_t callback)
{
    // 初始化ACK确认包信号量池
    PollInit(&kCommSendAckSemphrPoll, sizeof(StaticSemphrBlock_t), 8);

    kCallbackId = 0;
    kRecvCbList = ListCreate(sizeof(PackDealFunc_t)); // 创建接收回调链表

    send_req_queue_handle = xQueueCreate(8, sizeof(DataTransReq_t));
    uart_tx_mutex = xSemaphoreCreateMutex();
    recv_cb_list_mutex = xSemaphoreCreateMutex();
    send_ack_blocks_mutex = xSemaphoreCreateMutex();

    xTaskCreate(SendDataPackTask, "uartSendTask", 300, NULL, 5, &SendDataPackTask_handle);
    xTaskCreate(ReceiveDataPackTask, "uartRecvTask", 400, callback, 5, &ReceiveDataPackTask_handle);
    xTaskCreate(ACKTimeoutCheckTask, "uartackTask", 400, NULL, 3, &ACKTimeoutCheckTask_handle);
}

/* -------------------- 求和校验 -------------------- */
static uint8_t SumCheck(uint16_t size, uint8_t *src)
{
    uint8_t sum = 0;
    for (int i = 0; i < size; i++)
        sum += src[i];
    return sum;
}

void SendDataPackTask(void *param)
{
    DataTransReq_t req;
    while (1)
    {
        xQueueReceive(send_req_queue_handle, &req, portMAX_DELAY);
        uint32_t pack_id = g_pack_id++;

        if (req.size + PACK_OVERHEAD > PACK_MAX_SIZE) {
            if (req.finished_cb) {
                req.finished_cb(req.user_data, 0);
            }
            continue;
        }

        send_buffer[0] = PACK_HEAD;
        send_buffer[1] = (uint8_t)(req.size + PACK_OVERHEAD);
        send_buffer[2] = req.cmd;
        memcpy(&send_buffer[3], &pack_id, 4);
        memcpy(&send_buffer[7], req.data, req.size);

        send_buffer[7 + req.size] = SumCheck(req.size + 7, send_buffer);

        xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);
        uart_write_bytes(UART_NUM_1, (uint8_t *)send_buffer, req.size + PACK_OVERHEAD);
        xSemaphoreGive(uart_tx_mutex);

        if (!(req.cmd & PACK_NEED_ACK)) // 如果该包不需要进行包确认，那么直接执行发送完成回调
        {
            if (req.finished_cb)
                req.finished_cb(req.user_data, 1);
        }
        else // 挂起ACK等待，由接收完成函数或超时处理函数执行回调
        {
            bool inserted=false;
            xSemaphoreTake(send_ack_blocks_mutex, portMAX_DELAY);
            for (int i = 0; i < 8; i++)
            {
                if (!send_ack_blocks[i].is_using)
                {
                    send_ack_blocks[i].is_using = 1;
                    inserted=true;

                    send_ack_blocks[i].pack_id = pack_id;
                    send_ack_blocks[i].finished_cb = req.finished_cb;
                    send_ack_blocks[i].user_data = req.user_data;
                    send_ack_blocks[i].cmd = req.cmd;
                    send_ack_blocks[i].data = req.data;
                    send_ack_blocks[i].size = req.size;
                    send_ack_blocks[i].retry_cnt = req.max_retry_cnt;
                    send_ack_blocks[i].timeout_ms = req.timeout_ms;
                    send_ack_blocks[i].send_time_ms = Comm_GetTickMS()/1000;
                    break;
                }
            }
            xSemaphoreGive(send_ack_blocks_mutex);
            if(!inserted)   //ACK挂起数组已满，不能等待ACK包，直接执行失败回调
            {
                req.finished_cb(req.user_data, 0);
            }
        }
    }
}

void ReceiveDataPackTask(void *param)
{
    ListIterator_t it;
    InitListIterator(&it, kRecvCbList);
    while (1)
    {
        uint8_t head;
        int ret=uart_read_bytes(UART_NUM_1, &head, 1, 1000);
			if(ret==-1)
				continue;

        /* ---------- ACK 包 ---------- */
        if (head == ACK_HEAD)
        {
            uint32_t ack_id = 0;
            int got = uart_read_bytes(UART_NUM_1, (uint8_t *)&ack_id, 4, pdMS_TO_TICKS(20));
            if (got != 4) {
                if(param)
                    ((BadDataPackCb_t)param)(COMM_BAD_ACK);
                continue;
            }

            xSemaphoreTake(send_ack_blocks_mutex, portMAX_DELAY);
            for (int i = 0; i < 8; i++)
            {
                if (send_ack_blocks[i].is_using && send_ack_blocks[i].pack_id == ack_id)
                {
                    if (send_ack_blocks[i].finished_cb) {
                        send_ack_blocks[i].finished_cb(send_ack_blocks[i].user_data, 1);
                    }
                    send_ack_blocks[i].is_using = 0;
                    break;
                }
            }
            xSemaphoreGive(send_ack_blocks_mutex);
            continue;
        }

        /* ---------- 普通数据包 ---------- */
        if (head != PACK_HEAD)
        {
            if(param)
                ((BadDataPackCb_t)param)(COMM_BAD_HEAD);
            continue;
        }

        recv_buffer[0] = head;

        /* size */
        int size_got = uart_read_bytes(UART_NUM_1, &recv_buffer[1], 1, pdMS_TO_TICKS(20));
        if (size_got != 1) {
            if(param)
                ((BadDataPackCb_t)param)(COMM_BAD_LEN);
            continue;
        }

        uint16_t data_len = recv_buffer[1]; // 整包长度（包含 head/len/.../sum）
        if (data_len > PACK_MAX_SIZE || data_len < PACK_OVERHEAD) {
            if(param)
                ((BadDataPackCb_t)param)(COMM_BAD_LEN);
            continue;
        }

        /* CMD + ID + DATA（不含 head/len；sum 单独读） */
        uint16_t payload_to_read = (uint16_t)(data_len - 2 - 1);
        int payload_got = uart_read_bytes(UART_NUM_1, &recv_buffer[2], payload_to_read, pdMS_TO_TICKS(50));
        if (payload_got != (int)payload_to_read) {
            if(param)
                ((BadDataPackCb_t)param)(COMM_BAD_LEN);
            continue;
        }

        /* 校验：sum 覆盖 [0..data_len-2]（不含 sum 本身） */
        uint8_t sum = SumCheck((uint16_t)(data_len - 1), recv_buffer);
        uint8_t recv_sum = 0;
        int sum_got = uart_read_bytes(UART_NUM_1, &recv_sum, 1, pdMS_TO_TICKS(20));
        if (sum_got != 1) {
            if(param)
                ((BadDataPackCb_t)param)(COMM_BAD_LEN);
            continue;
        }

        if (recv_sum != sum)
        {
            if(param)
                ((BadDataPackCb_t)param)(COMM_BAD_SUM);
            continue;
        }

        // 如果当前数据包需要ACK回复，那么立即回复ACK
        uint8_t cmd = recv_buffer[2];
        if (cmd & PACK_NEED_ACK)
        {
            uint8_t ack[5];
            ack[0]=ACK_HEAD;
            memcpy(ack+1, recv_buffer + 3, 4);

            xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);
            uart_write_bytes(UART_NUM_1, ack, sizeof(ack));
            xSemaphoreGive(uart_tx_mutex);
        }

        xSemaphoreTake(recv_cb_list_mutex, portMAX_DELAY);
        ResetListIterator(&it);
        PackDealFunc_t *obj;
        while ((obj = IteraterGet(&it)) != NULL)
        {
            if (obj->cmd == (cmd & PACK_CMD_MASK))
            {
                obj->callback(recv_buffer + 7, (uint16_t)(data_len - PACK_OVERHEAD), obj->user_data);
                //break;
            }
            IteraterNext(&it);
        }
        xSemaphoreGive(recv_cb_list_mutex);
    }
}

void ACKTimeoutCheckTask(void *param)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        xSemaphoreTake(send_ack_blocks_mutex, portMAX_DELAY);
        int64_t cut_time=Comm_GetTickMS()/ 1000;
        for (int i = 0; i < 8; i++)
        {
            if (send_ack_blocks[i].is_using)
            {
                if (cut_time - send_ack_blocks[i].send_time_ms >= send_ack_blocks[i].timeout_ms) // 如果超时
                {
                    if (send_ack_blocks[i].retry_cnt) // 如果还有重试次数，那么重新把这个任务放到发送队列中（最大重试次数减一）
                    {
                        DataTransReq_t req;
                        req.cmd = send_ack_blocks[i].cmd;
                        req.data = send_ack_blocks[i].data;
                        req.finished_cb = send_ack_blocks[i].finished_cb;
                        req.max_retry_cnt = send_ack_blocks[i].retry_cnt - 1;
                        req.size = send_ack_blocks[i].size;
                        req.timeout_ms = send_ack_blocks[i].timeout_ms;
                        req.user_data = send_ack_blocks[i].user_data;
                        if(xQueueSend(send_req_queue_handle, &req, 0)==pdPASS)
                            send_ack_blocks[i].is_using = 0; // 声明当前块不再使用
                    }
                    else    //超时并且没有重试次数，通知应用层通信失败
                    {
                        send_ack_blocks[i].finished_cb(send_ack_blocks[i].user_data, 0);
                        send_ack_blocks[i].is_using = 0;
                    }
                }
            }
        }
        xSemaphoreGive(send_ack_blocks_mutex);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
    }
}


static uint32_t recv_list_match_cb(void *user, void *dst)
{
    uint32_t target_id = (uint32_t)user;
    if (((PackDealFunc_t *)dst)->id == target_id)
        return 1;
    return 0;
}

// 注册上行数据包接收回调
uint32_t register_comm_recv_cb(CommPackRecv_Cb callback, uint8_t cmd, void *user_data)
{
    kCallbackId = kCallbackId + 1;
    xSemaphoreTake(recv_cb_list_mutex, portMAX_DELAY);
    ListAddElement(kRecvCbList, &(PackDealFunc_t){.callback = callback, .user_data = user_data, .id = kCallbackId, .cmd = cmd});
    xSemaphoreGive(recv_cb_list_mutex);
    return kCallbackId;
}

// 取消注册上行数据包接收回调函数
uint32_t unregister_comm_recv_cb(uint32_t cb_id)
{
    xSemaphoreTake(recv_cb_list_mutex, portMAX_DELAY);
    int index = ListGetIndex(kRecvCbList, (void *)cb_id, recv_list_match_cb);
    if (index == -1) {
        xSemaphoreGive(recv_cb_list_mutex);
        return 0;
    }
    ListDeleteElement(kRecvCbList, index);
    xSemaphoreGive(recv_cb_list_mutex);
    return 1;
}

static void default_send_cb(void *user_data, uint32_t is_success)
{
    xSemaphoreGive(((StaticSemphrBlock_t *)user_data)->semphr_handle);
}

// 下行数据包发送（不确认）
uint32_t asyn_comm_send_pack_nak(uint8_t *src, uint8_t cmd, uint16_t size)
{
    if(!send_req_queue_handle)
        return 0;
    DataTransReq_t req;
    req.cmd = cmd & (~((uint8_t)PACK_NEED_ACK));
    req.data = src;
    req.size = size;
    req.finished_cb = NULL;
    return xQueueSend(send_req_queue_handle, &req, 0);
}

// 下行数据包发送（带确认）
uint32_t comm_send_pack_ack(uint8_t *src, uint8_t cmd, uint16_t size, uint32_t time_out_ms, uint8_t max_retry_num)
{
    if(!send_req_queue_handle)
        return 0;
    DataTransReq_t req;
    req.cmd = cmd | PACK_NEED_ACK;
    req.data = src;
    req.size = size;
    req.max_retry_cnt = max_retry_num;
    req.finished_cb = default_send_cb;
    StaticSemphrBlock_t *block = (StaticSemphrBlock_t *)PollRequireBlock(&kCommSendAckSemphrPoll);
    if (!block)
        return 0;
    block->semphr_handle = xSemaphoreCreateBinaryStatic(&block->queue_data);
    xSemaphoreTake(((StaticSemphrBlock_t *)block)->semphr_handle, 0);
    req.user_data = block;
    xQueueSend(send_req_queue_handle, &req, 1);
    BaseType_t ret=xSemaphoreTake(block->semphr_handle, pdMS_TO_TICKS(time_out_ms * max_retry_num));
    PollFreeBlock(&kCommSendAckSemphrPoll,block);
    return ret;
}

// 异步下行数据包发送（带确认）
uint32_t asyn_comm_send_pack_ack(uint8_t *src, uint8_t cmd, uint16_t size, CommPackSend_Cb send_cb, void *user_data, uint8_t max_retry_num)
{
    if(!send_req_queue_handle)
        return 0;
    DataTransReq_t req;
    req.cmd = cmd | PACK_NEED_ACK;
    req.data = src;
    req.size = size;
    req.max_retry_cnt = max_retry_num;
    req.finished_cb = send_cb;
    req.user_data = user_data;
    return xQueueSend(send_req_queue_handle, &req, 0);
}
