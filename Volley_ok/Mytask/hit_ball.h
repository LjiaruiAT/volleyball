#include "Task_Init.h"
#include "PID_old.h"
#include <stdbool.h>
#include "step.h"
#include "main.h"
#define C_OFFSET_Y   0.0645f     // C��ƽ̨�����е�y
#define HALF_P       0.1f    // P1-P2 һ��
#define L1           0.27f    // O1-A = A-P1
#define L2           0.24f    // O2-B = B-P2

typedef struct 
{
    float exp_tor;
    float exp_pos;
    float exp_vel;
    float exp_kp;
    float exp_kd;
}exp_param;
typedef struct
{
	exp_param exp;
	GO_MotorHandle_t go_volleyball;
	PID2 vel_pid;
	PID2 pos_pid;
}push;

typedef struct {
    uint32_t total;
    uint32_t overrun;
    uint32_t frame;
    uint32_t noise;
    uint32_t parity;
    uint32_t last_error_time;
    uint32_t continuous_errors;
    uint32_t recovery_attempts;
    uint32_t last_recovery_time;
} ErrorStats_t;
//typedef struct {
//    float x;
//    float y;
//} Vec2;
typedef struct {
    RM3508_TypeDef motor_3508;
    PID2 pos_pid_3508;
    PID2 vel_pid_3508;
} Rm3508;
void Hit_Task(void *pvParameters);

