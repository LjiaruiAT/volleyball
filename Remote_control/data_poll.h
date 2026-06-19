#ifndef __DATA_POLL_H__
#define __DATA_POLL_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

typedef struct{
    uint32_t block_is_used;
    uint8_t data[0];
}PollBlock_t;

typedef struct{
    uint32_t num;           // block 数量
    uint32_t using_num;
    uint32_t block_size;        // 每个 block 占用的字节数
    uint8_t *pool_mem;      // malloc 得到的整块内存
    PollBlock_t *poll;      // block 起始
    SemaphoreHandle_t event_semphr;
}DataPoll_t;



//初始化静态数据池
uint32_t PollInit(DataPoll_t *handle, uint32_t block_size, uint32_t num);

//从静态数据池中请求块
void* PollRequireBlock(DataPoll_t *handle);

//归还块
uint32_t PollFreeBlock(DataPoll_t *handle,void *block);

uint32_t PollFreeBlockNum(DataPoll_t *handle);
uint32_t PollWaitEvent(DataPoll_t *handle,uint32_t timeout_ms);

#endif