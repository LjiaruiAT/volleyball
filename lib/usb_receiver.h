#ifndef __USB_RECEIVER_H__
#define __USB_RECEIVER_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    float x;
    float y;
} UsbRecvData_t;

extern UsbRecvData_t g_usb_data;
extern SemaphoreHandle_t g_usb_data_sem;

void UsbReceiver_Init(void);

#endif
