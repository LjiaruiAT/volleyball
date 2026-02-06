#ifndef __GOMOTOR_H__
#define __GOMOTOR_H__

#include <stdio.h>
#include <stdint.h>

#include "485_bus.h"

#include "mylist.h"

#ifdef __cplusplus
extern "C" {
#endif


#pragma pack(1)

typedef struct
{
    uint16_t head;
    uint8_t cmd;
    int16_t torque;
    int16_t velocity;
    int32_t position;
    uint16_t pos_k;
    uint16_t vel_k;
    uint16_t crc;
}GOMotor_SendPack_t;

typedef struct
{
    uint16_t head;
    uint8_t cmd;
    int16_t torque;
    int16_t velocity;
    int32_t position;
    uint8_t temp;
    uint16_t state;
    uint16_t crc;
}GOMotor_ReceivePack_t;

#pragma pack()


typedef struct
{
    RS485_t *rs485;
    uint8_t motor_id;
    GOMotor_SendPack_t send_pack_buffer;       //发送缓冲区
    
    struct{
        float torque;
        float velocity;
        float rad;
        int8_t temp;
        uint8_t error;
        uint8_t mode;
    }state;
}GO_MotorHandle_t;

uint32_t GoMotorSend(GO_MotorHandle_t *motor, float torque, float velocity, float position, float kp, float kd);
uint32_t GoMotorRecv(GO_MotorHandle_t *motor);

int32_t GoMotorRecv_AutoMatch(RS485_t *rs485,MyList_t *motor_list);

#ifdef __cplusplus
}
#endif


#endif
