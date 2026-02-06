#include "Task_Init.h"
#include "PID_old.h"
#include <stdbool.h>
#include "step.h"
#include "main.h"
#define C_OFFSET_Y   0.0645f     // C��ƽ̨�����е�y
#define HALF_P       0.1f    // P1-P2 һ��
#define L1           0.27f    // O1-A = A-P1
#define L2           0.24f    // O2-B = B-P2

//typedef struct 
//{
//    float exp_tor;
//    float exp_rad;
//    float exp_omega;
//    float exp_kp;
//    float exp_kd;
//}exp_param;
//typedef struct {
//    float x;
//    float y;
//} Vec2;
void Hit_Task(void *pvParameters);
extern TaskHandle_t Hit_Task_Handle;

