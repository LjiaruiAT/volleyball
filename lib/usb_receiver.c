#include "usb_receiver.h"
#include "usb_trans.h"
#include <string.h>

#define USB_FRAME_HEAD    0xAA
#define USB_FRAME_TAIL    0xBB
#define USB_FRAME_LEN     10

UsbRecvData_t       g_usb_data;
SemaphoreHandle_t   g_usb_data_sem;

static uint8_t  ring_buf[USB_FRAME_LEN * 2];
static uint16_t ring_idx = 0;

static void RingBuf_Align(void)
{
    uint16_t head_pos = ring_idx;
    for (uint16_t i = 0; i < ring_idx; i++) {
        if (ring_buf[i] == USB_FRAME_HEAD) {
            head_pos = i;
            break;
        }
    }
    if (head_pos > 0) {
        memmove(ring_buf, ring_buf + head_pos, ring_idx - head_pos);
        ring_idx -= head_pos;
    }
}

void UsbRecvCallback(uint8_t *src, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        if (ring_idx < sizeof(ring_buf)) {
            ring_buf[ring_idx++] = src[i];
        } else {
            ring_idx = 0;
        }
    }

    while (ring_idx >= USB_FRAME_LEN) {
        RingBuf_Align();
        if (ring_idx < USB_FRAME_LEN) break;

        if (ring_buf[0] != USB_FRAME_HEAD ||
            ring_buf[USB_FRAME_LEN - 1] != USB_FRAME_TAIL) {
            ring_buf[0] = 0x00;
            continue;
        }

        float x, y;
        memcpy(&x, ring_buf + 1, 4);
        memcpy(&y, ring_buf + 5, 4);

        g_usb_data.x = x;
        g_usb_data.y = y;
        xSemaphoreGive(g_usb_data_sem);

        memmove(ring_buf, ring_buf + USB_FRAME_LEN, ring_idx - USB_FRAME_LEN);
        ring_idx -= USB_FRAME_LEN;
    }
}

void UsbReceiver_Init(void)
{
    g_usb_data_sem = xSemaphoreCreateBinary();
    USB_CDC_Init(UsbRecvCallback, NULL, NULL);
}
