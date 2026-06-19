#ifndef _JY61_H_
#define _JY61_H_

#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"


#pragma pack(1)
typedef struct {
    uint8_t head;
    uint8_t ID;
    int16_t X, Y, Z, Temp;
    uint8_t sum;
} Angle_Pack_Typedef;
#pragma pack()

typedef struct {
    float X, Y, Z;
} Vector3D_Typedef;

typedef struct {
  Vector3D_Typedef Acceleration;
  Vector3D_Typedef AngularVelocity;
  struct {
    float Yaw, Pitch, Roll;
    float lastYaw;
    int32_t rand;
    float Multiturn;
  } Angle;
  float Temp;
} JY61_Typedef;

extern JY61_Typedef JY61;
extern uint8_t Gyroscope_Init_count;
extern float Yaw_offset;

void JY61_Receive(JY61_Typedef* Gyro, uint8_t *data, uint8_t len);
#endif
