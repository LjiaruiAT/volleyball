/**
 * @author  Liu Yuanzhao
 * @date    2025-10-12
 * @details 自动驾驶框架，适用于力控底盘，需要FreeRTOS支持。
 * 需要先调用AutoPilotInit初始化自动驾驶仪环境，使用AutoPilotTrajectoryPlane规划路，返回符合限制条件下的预计运行时间，
 * @note 该框架设置的期望速度，位置，加速度均为场地坐标系下的期望，请自行读取传感器并转为车体坐标系下的期望
 */

#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__


#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "Vector.h"

typedef struct AutoPilotReq_t AutoPilotReq_t;

typedef enum{
    AUTOPILOT_STAGE_IDEL,       //自动驾驶仪断开
    AUTOPILOT_STAGE_RUNNING,    //运动中
    AUTOPILOT_STAGE_FINISH,     //自动驾驶仪到达终点
    AUTOPILOT_STAGE_ERROR       //自动驾驶仪错误
}AutopilotState;

//接口回调函数定义
typedef void(*SetRobotPosCallback_t)(Vector3D pos);
typedef void(*SetRobotVelCallback_t)(Vector3D vel);
typedef void(*SetRobotAccCallback_t)(Vector3D acc);
typedef void(*FinishedCallback_t)(AutopilotState state,AutoPilotReq_t *req,void* user_data);//执行导航结束回调函数

typedef struct{
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
}PathLine_t;        //描述位置-时间的五次多项式曲线

typedef struct{
    float step_dt;             //检查器检查时间步长
    float over_limit_gate;     //最大超限范围
    float iter_rate;           //迭代器迭代速率
    int max_iter_count;        //最大迭代次数
}MathSolver_t;                  //轨迹优化数学求解器参数

struct AutoPilotReq_t{
    Vector3D target_pos;      //目标位置
    Vector3D target_vel;      //速度
    Vector3D target_acc;      //末端加速度
    Vector3D start_pos;       //起点位置
    Vector3D start_vel;       //起点速度
    Vector3D start_acc;       //起点加速度
    
    float runTime;           //在期望时间点到达

    FinishedCallback_t finish_cb;   //回调函数参数
    void* user_data;                //回调函数参数
};                    //运动目标规结构体

typedef struct{
    PathLine_t x_line;  //归一化后的x坐标关于时间的曲线
    PathLine_t y_line;
    PathLine_t z_line;
    float exp_time;    //期望的运动时间

    FinishedCallback_t finish_cb;   //运动完成回调函数参数
    void* user_data;                //回调函数参数
}MoveDest_t;  //运动规划结构体

typedef struct{
    SetRobotPosCallback_t setPos_cb;
    SetRobotVelCallback_t setVel_cb;
    SetRobotAccCallback_t setAcc_cb;
}AutoPilotCallback_t;                   //接口回调函数组

typedef struct{
    AutoPilotCallback_t callBack;   //接口回调函数组
    AutopilotState currentState;   //当前自动驾驶仪状态
    QueueHandle_t runReqQueue;      //运动请求队列
    QueueHandle_t cancleReqSemphore;   //取消当前正在执行的运动信号量
    uint32_t dt_ms;                       //自动驾驶仪更新时间
}AutoPilot_t;

#define ZERO_VECTOR() (Vector3D){.x=0.0f,.y=0.0f,.z=0.0f}

#define PositiveValue(x) ((x)>0?(x):0)

#define AUTOPILOT_DEFAULT_SOLVER_PARAM() (MathSolver_t){.iter_rate=0.1f,.max_iter_count=50,.over_limit_gate=0.05f,.step_dt=0.05f}

/**
 * @brief   初始化自动驾驶仪
 * @param handle 自动驾驶仪句柄
 * @param callbackGroup 设置底盘期望的的回调函数组
 * @param proiroty 自动驾驶仪任务优先级
 * @param require_queue_size 导航任务请求队列长度
 * @param update_dt 导航仪目标更新时间间隔(ms)
 * @return 不返回任何值
 */
void AutoPilotInit(AutoPilot_t *handle, AutoPilotCallback_t *callBackGroup, uint32_t priority, uint32_t require_queue_size, uint32_t update_dt, TaskHandle_t task_handle);    //初始化队列指针，请求数据来源

/**
 * @brief 根据请求的期望位置和时间，规划出x,y,z三条曲线，并在满足加速度，速度限制的条件下尽可能以接近期望运动时间的策略规划曲线
 * @param req 运动请求结构体，包含起点约束，终点约束，完成回调函数及其参数
 * @param dest 规划出的结果，包含三个自由度的运动曲线，完成回调函数及其参数
 * @param vel_limit 运动过程中速度限制
 * @param acc_limit 运动过程中的加速度限制
 * @param omega_limit 运动过程中的自旋角速度限制
 * @param acc_omega_limit 运动过程中的自旋角加速度限制
 * @param run_time 期望的运动时间
 * @param solver 迭代器参数配置
 * @return 实际规划出的应当运动的时间（大于等于run_time）
 */
float AutoPilotTrajectoryPlane(AutoPilotReq_t *req, MoveDest_t *dest, float vel_limit, float acc_limit, float omega_limit, float acc_omega_limit, float run_time,MathSolver_t *solver);   //在已知限制条件下求解五次多项式轨迹曲线，并验证是否满足最大速度和加速度约束，如果失败返回0，成功返回1

/**
 * @brief 取消当前导航仪任务执行，并释放轮子
 * @param handle 导航仪句柄
 * @return 无
 */
inline void AutoPilotCancleCurrentPlane(AutoPilot_t *handle) //取消当前自动驾驶任务，返回0表示没有任务在运行，1表示成功取消
{
    xSemaphoreGive(handle->cancleReqSemphore);
};

/**
 * @brief 发送导航任务到导航仪
 * @param handle 导航仪任务句柄
 * @param dest 路径描述（包含路径曲线，运动时间等内容）
 * @return 无
 */
inline void AutoPilotSendTrajectoryToPilot(AutoPilot_t *handle,MoveDest_t *dest)
{
    xQueueSend(handle->runReqQueue,dest,0);
};

#endif
