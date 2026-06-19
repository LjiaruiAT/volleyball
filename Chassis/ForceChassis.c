#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "matrix.h"

float velocity[8] = {0};

void ChassisCalculateProcess(void *param)
{
    Chassis_t *chassis = (Chassis_t *)param;
    int wheel_num=chassis->wheel_num;

    float force[8];
    float cur_velocity[8];
    float robot_force[3];
    float robot_cur_vel[3];

    //底盘当前速度向量
    arm_matrix_instance_f32 wheel_vel_mat, wheel_cur_vel_mat,robot_vel_mat, wheel_force_mat, robot_force_mat, robot_acc_mat,robot_cur_vel_mat;
    arm_mat_init_f32(&wheel_force_mat, wheel_num*2, 1, force);    //轮子力矩向量
    arm_mat_init_f32(&wheel_vel_mat, wheel_num*2, 1, velocity);   //轮子速度向量
    arm_mat_init_f32(&wheel_cur_vel_mat, wheel_num*2, 1, cur_velocity);   //轮子当前速度向量

    arm_mat_init_f32(&robot_force_mat, 3, 1, robot_force);  //底盘广义力向量
    arm_mat_init_f32(&robot_vel_mat, 3, 1, (float *)(&(chassis->exp_vel)));   //底盘广义速度向量
    arm_mat_init_f32(&robot_acc_mat, 3, 1, (float *)(&(chassis->exp_acc)));   //底盘广义加速度向量

    arm_mat_init_f32(&robot_cur_vel_mat,3,1,robot_cur_vel);

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        // 安全检查部分:
        uint8_t safe_check=0;
        for(int i=0;i<wheel_num;i++)
        {
            WheelState state=chassis->wheel[i]->state_cb(chassis->wheel[i]);
            if(state==WHEEL_HEALTH)   //轮子正常
                safe_check=safe_check|(0x01<<i);
            else if(state==WHEEL_IDEL)  //如果是空闲模式，请求执行复位
                chassis->wheel[i]->reset_cb(chassis->wheel[i]);
            else if(state==WHEEL_ERROR) //轮子出现错误，执行底盘回调通知应用层
                chassis->wheel_err_cb(chassis,chassis->wheel[i]);
        }

        //如果底盘硬件通过了安全检查，那么执行后面的数学解算
        // 逆解部分:
        // TODO:底盘期望速度->轮子期望速度
        arm_mat_mult_f32(&chassis->vel_A_mat, &robot_vel_mat, &wheel_vel_mat);
        
        // TODO:底盘期望加速度->底盘广义力
        arm_mat_mult_f32(&chassis->Mq_mat, &robot_acc_mat, &robot_force_mat); // 根据acc计算力矩前馈值

        // 底盘广义力->4个轮子期望力
        arm_mat_min_norm_solve_f32(&chassis->force_A_mat, &robot_force_mat, &wheel_force_mat, 1e-5);

        for (int i = 0; i < wheel_num; i++)     //设置期望到轮子上
        {
            float torque_projection,exp_dir=chassis->wheel[i]->last_rad,exp_vel;

            exp_vel = sqrtf(velocity[2*i]*velocity[2*i]+velocity[2*i+1]*velocity[2*i+1]);
            if (exp_vel > chassis->dead_zone) // 防止奇点
            {
                exp_dir = atan2f(velocity[2*i+1], velocity[2*i]);
                torque_projection = (velocity[2*i] * force[2*i] + velocity[2*i+1] * force[2*i+1]) / exp_vel;
            }
            else
                torque_projection = 0.0f;

            chassis->wheel[i]->set_target_cb(chassis->wheel[i], exp_dir, exp_vel, torque_projection);
						chassis->wheel[i]->last_rad=exp_dir;
        }

        // 正解部分:
        for (int i = 0; i < wheel_num; i++) //获取轮子当前的速度，填写矩阵b
        {
            Vector2D temp = chassis->wheel[i]->get_vel_cb(chassis->wheel[i]);
            cur_velocity[2 * i] = temp.x;
            cur_velocity[2 * i + 1] = temp.y;
        }

        int ret=solve_linear_system_qr_f32(&chassis->vel_A_mat, &wheel_cur_vel_mat, &robot_cur_vel_mat); //求解机器人速度
        if (ret == 0)   //求解正确，更新数据
        {
            chassis->cur_vel.x=robot_cur_vel[0];
            chassis->cur_vel.y=robot_cur_vel[1];
            chassis->cur_vel.z=robot_cur_vel[2];
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(chassis->update_dt_ms));
    }
}

TaskHandle_t task_handle;

uint32_t ChassisInit(Chassis_t *chassis, Wheel_t *wheel,uint32_t wheel_num, Vector2D barycenter, float mass, float I, float dead_zone, uint32_t update_dt, uint32_t task_stack_size, uint32_t task_priority) // 初始化数学参数并创建任务
{
    if(wheel_num<2)
        return pdFAIL;
    chassis->barycenter = barycenter;
    chassis->mass = mass;
    chassis->I = I;
    chassis->dead_zone = dead_zone;
    chassis->update_dt_ms = update_dt;
    chassis->wheel_num=wheel_num;

    //数学映射矩阵初始化
    arm_mat_init_f32(&chassis->Mq_mat, 3, 3, (float*)chassis->Mq_data); // 构建惯性矩阵
    chassis->Mq_data[1][1] = chassis->Mq_data[0][0] = chassis->mass;
    chassis->Mq_data[0][1] = chassis->Mq_data[1][0] = 0.0f;
    chassis->Mq_data[0][2] = chassis->Mq_data[2][0] = -chassis->mass * chassis->barycenter.y;
    chassis->Mq_data[1][2] = chassis->Mq_data[2][1] = chassis->mass * chassis->barycenter.x;
    chassis->Mq_data[2][2] = chassis->I + chassis->mass * chassis->barycenter.x * chassis->barycenter.x +chassis->mass * chassis->barycenter.y * chassis->barycenter.y;
    
    arm_mat_init_f32(&chassis->vel_A_mat,wheel_num*2,3,(float*)&chassis->A_vel_data);//构建运动学映射矩阵
    for(int i=0;i<wheel_num;i++)
    {
        Vector3D pos=chassis->wheel[i]->pos;

        float c=cosf(pos.z);
        float s=sinf(pos.z);
        chassis->A_vel_data[i*2][0]=c;
        chassis->A_vel_data[i*2][1]=s;
        chassis->A_vel_data[i*2][2]=-pos.y*c+pos.x*s;
        chassis->A_vel_data[i*2+1][0]=-s;
        chassis->A_vel_data[i*2+1][1]=c;
        chassis->A_vel_data[i*2+1][2]=pos.y*s+pos.x*c;
    }

    arm_mat_init_f32(&chassis->force_A_mat,3,wheel_num*2,(float*)&chassis->force_A_data);
    arm_mat_trans_f32(&chassis->vel_A_mat,&chassis->force_A_mat);       //计算动力学映射矩阵

    return xTaskCreate(ChassisCalculateProcess, "chassis_task", task_stack_size, chassis, task_priority, &task_handle);
}
