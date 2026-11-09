#include "data_poll.h"


/* 计算单个 block 的跨度 */
inline static uint32_t poll_block_stride(const DataPoll_t *handle)
{
    return sizeof(PollBlock_t) + handle->block_size;
}

/* 初始化静态数据池（仅此处 malloc 一次） */
uint32_t PollInit(DataPoll_t *handle, uint32_t block_size, uint32_t num)
{
    if (!handle || block_size == 0 || num == 0) {
        return 1;
    }
    handle->event_semphr=xSemaphoreCreateBinary();
    handle->using_num=0;
    handle->num        = num;
    handle->block_size = block_size;

    uint32_t total_size = num * (sizeof(PollBlock_t) + block_size);

    handle->pool_mem = (uint8_t *)malloc(total_size);
    if (handle->pool_mem == NULL) {
        return 2;
    }

    memset(handle->pool_mem, 0, total_size);

    handle->poll = (PollBlock_t *)handle->pool_mem;

    return 0;
}

/* 从静态数据池中请求 block */
void* PollRequireBlock(DataPoll_t *handle)
{
    if (!handle || !handle->poll) {
        return NULL;
    }

    uint8_t *base = handle->pool_mem;
    uint32_t stride = poll_block_stride(handle);

    for (uint32_t i = 0; i < handle->num; i++) {
        PollBlock_t *blk = (PollBlock_t *)(base + i * stride);

        if (blk->block_is_used == 0) {
            blk->block_is_used = 1;
            handle->using_num++;
            xSemaphoreGive(handle->event_semphr);
            return blk->data;
        }
    }

    return NULL;  // 没有可用 block
}

/* 归还 block */
uint32_t PollFreeBlock(DataPoll_t *handle, void *ptr)
{
    if (!handle || !ptr) {
        return 1;
    }

    uint8_t *base = handle->pool_mem;
    uint32_t stride = poll_block_stride(handle);

    for (uint32_t i = 0; i < handle->num; i++) {
        PollBlock_t *blk = (PollBlock_t *)(base + i * stride);

        if (blk->data == (uint8_t *)ptr) {
            blk->block_is_used = 0;
            handle->using_num--;
            xSemaphoreGive(handle->event_semphr);
            return 0;
        }
    }

    return 2;  // 指针不属于该内存池
}

uint32_t PollWaitEvent(DataPoll_t *handle,uint32_t timeout_ms)
{
    return xSemaphoreTake(handle->event_semphr,pdMS_TO_TICKS(timeout_ms));
}

uint32_t PollFreeBlockNum(DataPoll_t *handle)
{
    return handle->num-handle->using_num;
}