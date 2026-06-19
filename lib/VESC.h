/**
 * @file vesc.h
 * @author Liu yuanzhao
 * @date 10-Sep-2025
 * @brief VESC电机驱动板控制
 * @note 适用于VESC v6.4 参考https://github.com/vedderb/bldc    中的/comm/comm_can.c第1216行起和第1587行起的内容
 */

#ifndef __VESC_H__
#define __VESC_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM = 3,
    CAN_PACKET_SET_POS = 4,
    CAN_PACKET_FILL_RX_BUFFER = 5,
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
    CAN_PACKET_PROCESS_RX_BUFFER = 7,
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
    CAN_PACKET_STATUS = 9,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    CAN_PACKET_STATUS_2 = 14,
    CAN_PACKET_STATUS_3 = 15,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_PING = 17,
    CAN_PACKET_PONG = 18,
    CAN_PACKET_DETECT_APPLY_ALL_FOC = 19,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES = 20,
    CAN_PACKET_CONF_CURRENT_LIMITS = 21,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS = 22,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN = 23,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN = 24,
    CAN_PACKET_CONF_FOC_ERPMS = 25,
    CAN_PACKET_CONF_STORE_FOC_ERPMS = 26,
    CAN_PACKET_STATUS_5 = 27
}VESC_CAN_CMD_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint32_t motor_id;

    int32_t epm;
    float current;
    float duty_cycle;

    struct{
        float consume_ah;
        float back_ah;
        float consume_wh;
        float back_wh;
    }power;

    struct{
        float mos_temp;
        float motor_temp;
        float input_cur;
        float input_vol;
        int32_t estimate_vel;
        float pid_value;
    }state;
} VESC_t;

uint32_t VESC_SetVoltage(VESC_t *vesc,float percentage);
uint32_t VESC_SetCurrent(VESC_t *vesc,float ampere);
uint32_t VESC_SetBreakCur(VESC_t *vesc,float ampere);
uint32_t VESC_SetRPM(VESC_t *vesc,int32_t rpm);
uint32_t VESC_SetPosition(VESC_t *vesc,int32_t pos);


uint32_t VESC_ReceiveHandler(VESC_t *vesc, CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *buf);

#endif
