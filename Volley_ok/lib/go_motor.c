#include "go_motor.h"
#include "crc_ccitt.h"


uint32_t GoMotorSend(GO_MotorHandle_t *motor, float torque, float velocity, float position, float kp, float kd)
{
    motor->send_pack_buffer.head=0xEEFE;
    motor->send_pack_buffer.cmd=(motor->motor_id&0x0F)|0x10;
    motor->send_pack_buffer.torque=(((double)torque)*256.0);
    motor->send_pack_buffer.velocity=(((double)velocity)*256.0/(2.0*3.14159265));
    motor->send_pack_buffer.position=((double)position)*32768.0/(2.0*3.14159265);
    motor->send_pack_buffer.pos_k=((double)kp)*1280.0;
    motor->send_pack_buffer.vel_k=((double)kd)*1280.0;
    motor->send_pack_buffer.crc=crc_ccitt(0, (uint8_t *)(&motor->send_pack_buffer), sizeof(GOMotor_SendPack_t)-2);
    return RS485Send(motor->rs485,(uint8_t*)&motor->send_pack_buffer,sizeof(GOMotor_SendPack_t),2);
}

uint32_t GoMotorRecv(GO_MotorHandle_t *motor)
{
    uint8_t buffer[sizeof(GOMotor_ReceivePack_t)];
    uint32_t size=0;
    RS485Recv(motor->rs485,buffer,sizeof(GOMotor_ReceivePack_t),2,&size);
    if(size!=sizeof(GOMotor_ReceivePack_t))     //数据包长度错误
        return 0;
    if(crc_ccitt(0, buffer, sizeof(GOMotor_ReceivePack_t)-2)!=((GOMotor_ReceivePack_t*)buffer)->crc)     //校验错误
        return 0;
    GOMotor_ReceivePack_t* p_buf=(GOMotor_ReceivePack_t*)buffer;
    if((p_buf->cmd&0x0F)!=motor->motor_id)  //检查电机ID
        return 0;
    motor->state.mode=p_buf->cmd>>4;
    motor->state.torque=p_buf->torque/256.0f;
    motor->state.velocity=p_buf->velocity*2.0*3.14159265359/256.0;
    motor->state.rad=p_buf->position/32768.0*2.0*3.14159265359;
    motor->state.temp=(int8_t)p_buf->temp;
    motor->state.error=p_buf->state&0x07;
    return 1;
}

static uint32_t list_match_cb(void* user,void* dst)
{
    GO_MotorHandle_t *motor=(GO_MotorHandle_t*)dst;
    if(motor->motor_id==(uint8_t)(uintptr_t)user)  //匹配成功，返回1
        return 1;
    return 0;
}

int32_t GoMotorRecv_AutoMatch(RS485_t *rs485,MyList_t *motor_list)
{
    uint8_t buffer[sizeof(GOMotor_ReceivePack_t)];
    uint32_t size=0;
    if(RS485Recv(rs485,buffer,sizeof(GOMotor_ReceivePack_t),1000,&size)!=pdPASS)
        return pdFAIL;
    if(size!=sizeof(GOMotor_ReceivePack_t))     //数据包长度错误
        return 0;
    if(crc_ccitt(0, buffer, sizeof(GOMotor_SendPack_t)-2)!=((GOMotor_ReceivePack_t*)buffer)->crc)     //校验错误
        return 0;
    GOMotor_ReceivePack_t* p_buf=(GOMotor_ReceivePack_t*)buffer;

    GO_MotorHandle_t** p2_motor=ListFind(motor_list,(void*)(p_buf->cmd&0x0F),list_match_cb);
    if(p2_motor==NULL)     //如果没有找到这个电机，那么返回0表示接收失败
        return 0;

    GO_MotorHandle_t* motor=*p2_motor;  //解引用指向电机结构体指针的指针
    
    motor->state.mode=p_buf->cmd>>4;
    motor->state.torque=p_buf->torque/256.0f;
    motor->state.velocity=p_buf->velocity*2.0*3.14159265359/256.0;
    motor->state.rad=p_buf->position/32768.0*2.0*3.14159265359;
    motor->state.temp=(int8_t)p_buf->temp;
    motor->state.error=p_buf->state&0x07;
    return 1;
}