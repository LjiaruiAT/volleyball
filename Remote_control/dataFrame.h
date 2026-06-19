#ifndef __DATAFRAME_H__
#define __DATAFRAME_H__

#include <stdint.h>

#define PACK_HEAD 0x5A
#define ACK_HEAD 0xAA

#define PACK_TYPE_MASK  0x80
#define PACK_CMD_MASK   0x0F

#define PACK_TYPE_ACK   0x80
#define PACK_TYPE_NAK   0x00

#define CMD_REMOTE_UPDATE_ROCKER            0x01
#define CMD_REMOTE_UPDATE_VIRTUAL_ITEM      0x02

#define CMD_RECEIVER_MESSAGEBOX             0x03
#define CMD_RECEIVER_UPDATE_VIRTUAL_ITEM    0x04

/******************* 按键位定义*******************/
#define KEY_Right_Switch_Up     (1 << 1)
#define KEY_Right_Switch_Down   (1 << 2)

#define KEY_Right_Key_Left      (1 << 3)
#define KEY_Right_Key_Up        (1 << 4)
#define KEY_Right_Key_Right     (1 << 5)
#define KEY_Right_Key_Down      (1 << 6)

#define KEY_Left_Broadside_Key  (1 << 7)
#define KEY_Right_Broadside_Key (1 << 8)

#define KEY_Left_Switch_Down    (1 << 9)
#define KEY_Left_Switch_Up      (1 << 10)

#define KEY_Left_Key_Down       (1 << 11)
#define KEY_Left_Key_Left       (1 << 12)
#define KEY_Left_Key_Right      (1 << 13)
#define KEY_Left_Key_Up         (1 << 14)

#pragma pack(1)

//遥控器下行数据包，控制信号
#define PACK_CONTROL_CMD    0x01
typedef struct
{
	float rocker[4];
	uint32_t Key;
}PackControl_t;


typedef struct
{
	uint8_t head;
	uint32_t Key_Data;
	uint8_t tail;
} __attribute__((packed))Pack_TransRemote_t;

//遥控器上行数据包，字符串反馈信息
#define PACK_STR_FEEDBACK_CMD    0x02
typedef struct
{
    uint8_t type;
    uint8_t size;
    char* str;
}PackMsg_t;
#pragma pack()




#endif