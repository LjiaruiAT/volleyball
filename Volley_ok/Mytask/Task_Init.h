#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_


#include "step.h"
#include "FDCANDriver.h"
#include "485_bus.h"
#include "usart.h"
#include "freertos.h"
#include "go_motor.h"
#include "PID_old.h"
#include "motorEx.h"
#include "math.h"
void Task_Init(void);
void Hit_Task(void *pvParameters);

#endif
