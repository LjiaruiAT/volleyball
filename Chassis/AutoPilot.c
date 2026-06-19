#include "AutoPilot.h"

static inline void set_quintic(PathLine_t *line, float p0, float v0, float a0, float pT, float vT, float aT,float dt)
{
    v0=v0*dt;       //速度/加速度参数归一化
    a0=a0*dt*dt;
    vT=vT*dt;
    aT=aT*dt*dt;

    line->a = p0;
    line->b = v0;
    line->c = 0.5 * a0;

    line->d = 10 * (pT - p0) - 6 * v0 - 1.5 * a0 - 4 * vT + 0.5 * aT;
    line->e = -15 * (pT - p0) + 8 * v0 + 1.5 * a0 + 7 * vT - aT;
    line->f = 6 * (pT - p0) - 3 * v0 - 0.5 * a0 - 3 * vT + 0.5 * aT;
}

// 一次导数
float line_derivative(const PathLine_t *line, float t)
{
    return line->b + 2.0f * line->c * t + 3.0f * line->d * t * t + 4.0f * line->e * t * t * t + 5.0f * line->f * t * t * t * t;
}

// 二次导数
float line_second_derivative(const PathLine_t *line, float t)
{
    return 2.0f * line->c + 6.0f * line->d * t + 12.0f * line->e * t * t + 20.0f * line->f * t * t * t;
}

static void AutoPilotProcess(void *param)
{
    AutoPilot_t *handle = (AutoPilot_t *)param;
    uint32_t wake_dt = handle->dt_ms;
    MoveDest_t current_dest;

    Vector3D pos, vel, acc;

    handle->callBack.setVel_cb(ZERO_VECTOR()); // 设置初速度和初始加速度为0
    handle->callBack.setAcc_cb(ZERO_VECTOR());
    handle->currentState = AUTOPILOT_STAGE_IDEL; // 当前状态为IDEL模式，释放对轮子的闭环控制

    uint32_t autopilot_exit_state;

    while (1)
    {
        autopilot_exit_state = 0;
        xQueueReceive(handle->runReqQueue, &current_dest, portMAX_DELAY); // 等待新的运动请求
        xSemaphoreTake(handle->cancleReqSemphore, 0);                    // 清空信号量
        handle->currentState = AUTOPILOT_STAGE_RUNNING;
        TickType_t pxPreviousWakeTime = xTaskGetTickCount();
        float step_time = 0.0f;
        while (step_time < current_dest.exp_time) // 等待走完全流程
        {
            float cur_step = step_time / current_dest.exp_time;
            
            // 计算位置
            pos.x = current_dest.x_line.a + current_dest.x_line.b * cur_step + current_dest.x_line.c * powf(cur_step, 2.0f) +
                   current_dest.x_line.d * powf(cur_step, 3.0f) + current_dest.x_line.e * powf(cur_step, 4.0f) +
                   current_dest.x_line.f * powf(cur_step, 5.0f);
            pos.y = current_dest.y_line.a + current_dest.y_line.b * cur_step + current_dest.y_line.c * powf(cur_step, 2.0f) +
                   current_dest.y_line.d * powf(cur_step, 3.0f) + current_dest.y_line.e * powf(cur_step, 4.0f) +
                   current_dest.y_line.f * powf(cur_step, 5.0f);
            pos.z = current_dest.z_line.a + current_dest.z_line.b * cur_step + current_dest.z_line.c * powf(cur_step, 2.0f) +
                   current_dest.z_line.d * powf(cur_step, 3.0f) + current_dest.z_line.e * powf(cur_step, 4.0f) +
                   current_dest.z_line.f * powf(cur_step, 5.0f);
            
            // 计算速度
            vel.x = (current_dest.x_line.b + 2 * current_dest.x_line.c * cur_step + 3 * current_dest.x_line.d * powf(cur_step, 2.0f) +
                    4 * current_dest.x_line.e * powf(cur_step, 3.0f) + 5 * current_dest.x_line.f * powf(cur_step, 4.0f)) /
                   current_dest.exp_time;
            vel.y = (current_dest.y_line.b + 2 * current_dest.y_line.c * cur_step + 3 * current_dest.y_line.d * powf(cur_step, 2.0f) +
                    4 * current_dest.y_line.e * powf(cur_step, 3.0f) + 5 * current_dest.y_line.f * powf(cur_step, 4.0f)) /
                   current_dest.exp_time;
            vel.z = (current_dest.z_line.b + 2 * current_dest.z_line.c * cur_step + 3 * current_dest.z_line.d * powf(cur_step, 2.0f) +
                    4 * current_dest.z_line.e * powf(cur_step, 3.0f) + 5 * current_dest.z_line.f * powf(cur_step, 4.0f)) /
                   current_dest.exp_time;
            
            // 计算加速度
            acc.x = (2 * current_dest.x_line.c + 6 * current_dest.x_line.d * cur_step + 12 * current_dest.x_line.e * powf(cur_step, 2.0f) +
                    20 * current_dest.x_line.f * powf(cur_step, 3.0f)) /
                   (current_dest.exp_time * current_dest.exp_time);
            acc.y = (2 * current_dest.y_line.c + 6 * current_dest.y_line.d * cur_step + 12 * current_dest.y_line.e * powf(cur_step, 2.0f) +
                    20 * current_dest.y_line.f * powf(cur_step, 3.0f)) /
                   (current_dest.exp_time * current_dest.exp_time);
            acc.z = (2 * current_dest.z_line.c + 6 * current_dest.z_line.d * cur_step + 12 * current_dest.z_line.e * powf(cur_step, 2.0f) +
                    20 * current_dest.z_line.f * powf(cur_step, 3.0f)) /
                   (current_dest.exp_time * current_dest.exp_time);
            
            handle->callBack.setPos_cb(pos);
            handle->callBack.setVel_cb(vel);
            if(handle->callBack.setAcc_cb)      //如果加速度回调函数为空，那么说明该底盘没有提供力控接口，故直接执行速度/位置控制
                handle->callBack.setAcc_cb(acc);
            vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(wake_dt));
            step_time = step_time + wake_dt * 0.001f;

            if (xSemaphoreTake(handle->cancleReqSemphore, 0) == pdPASS) // 如果外部请求现在立即中断导航仪并释放轮子，那么跳出迭代，设置异常标志位
            {
                autopilot_exit_state = 1;
                break;
            }
        }
        handle->currentState = AUTOPILOT_STAGE_FINISH;

        if (autopilot_exit_state) // 非正常退出，需要清零机器人速度，加速度。并清空导航任务队列
        {
            handle->currentState = AUTOPILOT_STAGE_ERROR;
            handle->callBack.setVel_cb(ZERO_VECTOR());
            handle->callBack.setAcc_cb(ZERO_VECTOR());
            xQueueReset(handle->runReqQueue);
        }
        if(current_dest.finish_cb)
            current_dest.finish_cb(handle->currentState, NULL, current_dest.user_data);
    }
}

float AutoPilotTrajectoryPlane(AutoPilotReq_t *req, MoveDest_t *dest, float vel_limit, float acc_limit, float omega_limit, float acc_omega_limit, float run_time, MathSolver_t *solver) // 根据期望位置和约束条件计算曲线，如果不满足那么进行数值迭代尽可能找到一个可行的曲线
{
    PathLine_t line[3];
    set_quintic(&line[0], req->start_pos.x, req->start_vel.x, req->start_acc.x, req->target_pos.x, req->target_vel.x, req->target_acc.x,run_time);
    set_quintic(&line[1], req->start_pos.y, req->start_vel.y, req->start_acc.y, req->target_pos.y, req->target_vel.y, req->target_acc.y,run_time);
    set_quintic(&line[2], req->start_pos.z, req->start_vel.z, req->start_acc.z, req->target_pos.z, req->target_vel.z, req->target_acc.z,run_time);
    
    float vel_value[3] = {0.0f, 0.0f, 0.0f};
    float acc_value[3] = {0.0f, 0.0f, 0.0f};
    float max_vel = 0.0f, max_acc = 0.0f, max_omega = 0.0f, max_acc_omega = 0.0f;

    float cur_step = 0.0f;
    while (cur_step < 1.0f) // 进行迭代检查函数最值
    {
        float temp;
        for (int i = 0; i < 3; i++) // 记录x,y,z方向的加速度和速度最大值，检查是否满足最大速度或最大加速度约束
        {
            temp = line_derivative(&line[i], cur_step) / run_time;
            if (temp > vel_value[i])
                vel_value[i] = temp;
            temp = line_second_derivative(&line[i], cur_step) / (run_time * run_time);
            if (temp > acc_value[i])
                acc_value[i] = temp;
        }
        temp = sqrtf(vel_value[0] * vel_value[0] + vel_value[1] * vel_value[1]); // 求解欧几里得速度和加速度
        if (temp > max_vel)
            max_vel = temp;

        temp = sqrtf(acc_value[0] * acc_value[0] + acc_value[1] * acc_value[1]);
        if (temp > max_acc)
            max_acc = temp;

        if (vel_value[2] > max_omega) // 记录角速度最大值
            max_omega = vel_value[2];

        if (acc_value[2] > max_acc_omega) // 记录角加速度最大值
            max_acc_omega = acc_value[2];

        cur_step = cur_step + solver->step_dt;
    }

    if (!(vel_limit > max_vel && acc_limit > max_acc && omega_limit > max_omega && acc_omega_limit > max_acc_omega)) // 不满足全部满足约束时
    {
        int iter = 0;
        float rate[4];
        float max_over_limit_rate=0.0f;
        while (iter < solver->max_iter_count)
        {
            rate[0]=PositiveValue(max_vel-vel_limit)/vel_limit;     //计算超限的比例，并选择超限比例最多的一个进行迭代
            rate[1]=PositiveValue(max_acc-acc_limit)/max_acc;
            rate[2]=PositiveValue(max_omega-omega_limit)/omega_limit;
            rate[3]=PositiveValue(max_acc_omega-acc_omega_limit)/acc_omega_limit;
            for(int i=0;i<4;i++)    //找到超限最多的一个
            {
                if(max_over_limit_rate<rate[i])
                    max_over_limit_rate=rate[i];
            }
            if(max_over_limit_rate<solver->over_limit_gate)     //符合迭代要求，允许返回
                break;
            float scale_rate=(1.0f+max_over_limit_rate)*(1.0f+solver->iter_rate);  //根据超限比例计算时间缩放系数
            
            max_acc=max_acc/scale_rate;     //更新时间缩放后的最大值
            max_vel=max_vel/scale_rate;
            max_omega=max_omega/scale_rate;
            max_acc_omega=max_acc_omega/scale_rate;

            run_time=run_time*scale_rate;   //重新计算运行时间
            iter++;
        }
    }

    dest->x_line=line[0];
    dest->y_line=line[1];
    dest->z_line=line[2];
    dest->exp_time=run_time;
    dest->finish_cb=req->finish_cb;
    dest->user_data=req->user_data;
    return run_time;
}

void AutoPilotInit(AutoPilot_t *handle, AutoPilotCallback_t *callBackGroup, uint32_t priority, uint32_t require_queue_size, uint32_t update_dt, TaskHandle_t task_handle)
{
    handle->dt_ms = update_dt;
    handle->callBack = *callBackGroup;
    handle->currentState = AUTOPILOT_STAGE_IDEL;
    handle->runReqQueue = xQueueCreate(require_queue_size, sizeof(MoveDest_t));
    handle->cancleReqSemphore = xSemaphoreCreateBinary();
    xTaskCreate(AutoPilotProcess, "AutoPilot", 256, handle, priority, &task_handle);
}