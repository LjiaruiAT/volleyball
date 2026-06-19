#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include "stdint.h"

// 动作类型定义
typedef enum {
    ACTION_TYPE_UNINTERRUPTABLE,  // 不可打断动作
    ACTION_TYPE_INTERRUPTABLE     // 可打断动作
} ActionType_t;

// 动作状态定义
typedef enum {
    ACTION_STATE_IDLE,          // 空闲
    ACTION_STATE_EXECUTING,     // 执行中
    ACTION_STATE_COMPLETED,     // 已完成
    ACTION_STATE_INTERRUPTED,   // 已打断
    ACTION_STATE_ERROR          // 错误
} ActionState_t;

// 动作基本结构
typedef struct {
    ActionType_t type;              // 动作类型
    ActionState_t state;            // 动作状态
    uint32_t id;                    // 动作ID
    void (*execute)(void *user_data); // 执行函数
    void (*complete)(void *user_data); // 完成回调
    void *user_data;                // 用户数据
} Action_t;

#endif
