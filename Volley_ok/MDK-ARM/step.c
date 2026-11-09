#include "step.h"

void Cubic_SetTrajectory(CubicParam_t *seg,
                         float p0, float v0,
                         float pT, float vT,
                         float duration, uint32_t current_tick)
{
    // 确保最小的时长为0.05秒
    if (duration < 0.05f) duration = 0.05f;

    // 限制目标速度在最大电机速度范围内
    if (vT > M3508_MAX_SPEED_RADS) vT = M3508_MAX_SPEED_RADS;
    if (vT < -M3508_MAX_SPEED_RADS) vT = -M3508_MAX_SPEED_RADS;

    float T = (float)duration;

    // 计算三次多项式的系数
    seg->T = duration;
    seg->start_pos = p0;
    seg->target_pos = pT;
    seg->start_time_ms = current_tick;
    seg->is_running = 1;

    // 三次多项式的系数计算
    seg->a0 = p0;
    seg->a1 = v0;
    seg->a2 = (3.0f * (pT - p0) - (2.0f * v0 + vT) * T) / (T * T);
    seg->a3 = (2.0f * (p0 - pT) + (v0 + vT) * T) / (T * T * T);
}

/**
 * @brief 获取当前轨迹状态（位置、速度、加速度）
 * @param seg 指向结构体的指针
 * @param current_tick 当前系统时间 (ms)
 * @param out 指向输出状态结构体的指针
 */
void Cubic_GetFullState(CubicParam_t* seg, uint32_t current_tick, TrajectoryState_t* out)
{
    if (!seg->is_running || (current_tick < seg->start_time_ms)) {
        out->pos = seg->target_pos;
        out->vel = 0.0f;
        out->acc = 0.0f;
        return;
    }

    float t = (float)(current_tick - seg->start_time_ms) * 0.001f;

    if (t >= seg->T) {
        seg->is_running = 0;
        out->pos = seg->target_pos;
        out->vel = 0.0f;
        out->acc = 0.0f;
        return;
    }

    // 计算位置、速度和加速度
    out->pos = seg->a3 * t * t * t + seg->a2 * t * t + seg->a1 * t + seg->a0;
    out->vel = 3.0f * seg->a3 * t * t + 2.0f * seg->a2 * t + seg->a1;
    out->acc = 6.0f * seg->a3 * t + 2.0f * seg->a2;
}