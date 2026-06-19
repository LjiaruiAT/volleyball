#ifndef __FORCE_CHASSIS_H__
#define __FORCE_CHASSIS_H__

#include "arm_math.h"
#include "svd.h"
#include "Vector.h"

typedef enum{
    WHEEL_IDEL,         //轮子不会对指令做出响应
    WHEEL_HEALTH,       //轮子全部功能正常
    WHEEL_RESETING,     //轮子正在复位
    WHEEL_ERROR         //轮子异常，影响正常使用，一般尝试复位指令
}WheelState;


typedef struct Wheel_t Wheel_t;
typedef struct Chassis_t Chassis_t;

//接口回调函数定义
typedef void(*SetWheelTarget_Cb_t)(Wheel_t *_this,float rad,float velocity,float force);   //设置舵轮的速度，力
typedef void(*WheelReset_Cb_t)(Wheel_t *_this);                                      //调用该函数请求舵轮复位
typedef WheelState(*WheelState_Cb_t)(Wheel_t *_this);          //获取舵轮状态
typedef Vector2D(*GetWheelVelocity_Cb_t)(Wheel_t *_this); //获取轮子的速度方向
typedef void(*WheelError_Cb_t)(Chassis_t *_this,Wheel_t *wheel);           //轮子运行异常回调

struct Wheel_t{
    //当前轮子机械安装位置(在底盘中心坐标系的x，y和安装角fai(弧度制))
    Vector3D pos;
    float last_rad;

    //回调接口函数(所有函数必须为非阻塞函数)
    SetWheelTarget_Cb_t set_target_cb;  //设置轮子的目标
    WheelReset_Cb_t reset_cb;           //请求轮子执行复位动作
    WheelState_Cb_t state_cb;           //获取轮子的状态
    GetWheelVelocity_Cb_t get_vel_cb;   //获取轮子速度回调

    //用户数据
    void* user_data;
};

struct Chassis_t{
    //硬件驱动部分
    Wheel_t *wheel[4];   //四个轮子的结构体
    uint32_t wheel_num;
    WheelError_Cb_t wheel_err_cb;

    //数学参数部分
    Vector2D barycenter;    //质心坐标在底盘坐标系下的位置
    float mass;             //质量
    float I;                //机器人绕底盘坐标原点作自旋运动时的转动惯量
    float dead_zone;        //速度死区，防止短时间舵轮期望角度发生剧变

    //输入/输出部分
    Vector3D exp_vel;           //底盘期望
    Vector3D exp_pos;           //底盘期望位置
    Vector3D exp_acc;           //底盘期望加速度
    Vector3D cur_pos;           //底盘实际位置
    Vector3D cur_vel;           //底盘实际速度
    
    //控制周期
    uint32_t update_dt_ms;      

    //数学部分：
    //控制映射矩阵(在初始化底盘时预计算且不会再改变)
    arm_matrix_instance_f32 force_A_mat;
    float force_A_data[3][8];
    //运动学控制部分映射矩阵
    arm_matrix_instance_f32 vel_A_mat;
    float A_vel_data[8][3];
    arm_matrix_instance_f32 Mq_mat;
    float Mq_data[3][3];
};

uint32_t ChassisInit(Chassis_t *chassis, Wheel_t *wheel,uint32_t wheel_num, Vector2D barycenter, float mass, float I, float dead_zone, uint32_t update_dt, uint32_t task_stack_size, uint32_t task_priority); // 初始化数学参数并创建任务

#endif
