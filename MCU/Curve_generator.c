#include "Curve_generator.h"

#include <math.h>

/*
 * 通道内部状态数组（索引即通道 id）
 *
 * start : 本次规划的起点值 y0
 * target: 本次规划的终点值 y1
 * out   : 当前输出值 y(t)
 * t     : 当前已运行时间 t
 * T     : 本次规划总时长 T
 * busy  : 规划状态，1=正在过渡，0=空闲/已到目标
 */
static float s_sine_start[CURVE_GEN_MAX_NUM];
static float s_sine_target[CURVE_GEN_MAX_NUM];
static float s_sine_out[CURVE_GEN_MAX_NUM];
static float s_sine_t[CURVE_GEN_MAX_NUM];
static float s_sine_T[CURVE_GEN_MAX_NUM];
static uint8_t s_sine_busy[CURVE_GEN_MAX_NUM];

/* 浮点绝对值（避免依赖 fabsf，减少平台差异） */
static float curvegen_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/* 浮点限幅：把 x 约束在 [min_v, max_v] 内 */
static float curvegen_clampf(float x, float min_v, float max_v)
{
    if (x < min_v)
        return min_v;
    if (x > max_v)
        return max_v;
    return x;
}

/*
 * 复位通道：立即设置到给定值，并清空本次规划状态。
 * 该函数常用于上电初始化、急停恢复、模式切换等场景。
 */
void CurveGen_Sine_Reset(uint8_t id, float value)
{
    if (id >= CURVE_GEN_MAX_NUM)
        return;

    s_sine_start[id] = value;
    s_sine_target[id] = value;
    s_sine_out[id] = value;
    s_sine_t[id] = 0.0f;
    s_sine_T[id] = 0.0f;
    s_sine_busy[id] = 0U;
}

/*
 * 以“固定总时间 T”启动一次规划。
 *
 * 行为要点：
 * 1) 起点不是外部传入值，而是“当前输出 s_sine_out[id]”，
 *    这样可在中途改目标时平滑续接，不会突跳。
 * 2) time_s <= 0 时直接到目标，不进入规划状态（busy=0）。
 */
void CurveGen_Sine_StartByTime(uint8_t id, float target, float time_s)
{
    if (id >= CURVE_GEN_MAX_NUM)
        return;

    if (time_s <= 0.0f)
    {
        s_sine_start[id] = target;
        s_sine_target[id] = target;
        s_sine_out[id] = target;
        s_sine_t[id] = 0.0f;
        s_sine_T[id] = 0.0f;
        s_sine_busy[id] = 0U;
        return;
    }

    s_sine_start[id] = s_sine_out[id];
    s_sine_target[id] = target;
    s_sine_t[id] = 0.0f;
    s_sine_T[id] = time_s;
    s_sine_busy[id] = 1U;
}

/*
 * 以“最大变化率”启动规划。
 *
 * 已知半余弦轨迹：
 *   y(t)=y0 + (1-cos(pi*t/T))/2 * (y1-y0)
 *
 * 其导数：
 *   dy/dt = (pi/(2T)) * sin(pi*t/T) * (y1-y0)
 *
 * 因为 |sin(...)|<=1，故最大斜率：
 *   max|dy/dt| = |y1-y0| * pi / (2T)
 *
 * 令 max|dy/dt| = max_rate，可得：
 *   T = |y1-y0| * pi / (2*max_rate)
 */
void CurveGen_Sine_StartByRate(uint8_t id, float target, float max_rate_per_s)
{
    if (id >= CURVE_GEN_MAX_NUM)
        return;

    if (max_rate_per_s <= 0.0f)
    {
        CurveGen_Sine_StartByTime(id, target, 0.0f);
        return;
    }

    /* 当前输出到目标的差值 */
    float delta = target - s_sine_out[id];

    /* 差值为 0 时无需规划，直接对齐目标 */
    if (curvegen_absf(delta) <= 0.0f)
    {
        CurveGen_Sine_StartByTime(id, target, 0.0f);
        return;
    }

    float time_s = curvegen_absf(delta) * CURVE_GEN_PI_F / (2.0f * max_rate_per_s);

    CurveGen_Sine_StartByTime(id, target, time_s);
}

/*
 * 周期更新：按给定 dt 推进一次曲线。
 *
 * 返回值说明：
 * - busy=0：直接返回上次输出（已到目标或未启动）
 * - busy=1：推进时间并计算新输出
 */
float CurveGen_Sine_Update(uint8_t id, float dt_s)
{
    if (id >= CURVE_GEN_MAX_NUM)
        return 0.0f;

    if (s_sine_busy[id] == 0U)
        return s_sine_out[id];

    /* 仅接受正向时间步长，防止误传负值破坏状态 */
    if (dt_s > 0.0f)
    {
        s_sine_t[id] += dt_s;
    }

    /* 到达/超过终点时间：强制贴合目标并结束 busy */
    if (s_sine_T[id] <= 0.0f || s_sine_t[id] >= s_sine_T[id])
    {
        s_sine_out[id] = s_sine_target[id];
        s_sine_busy[id] = 0U;
        return s_sine_out[id];
    }

    /* 归一化进度 s∈[0,1] */
    float s = s_sine_t[id] / s_sine_T[id];
    s = curvegen_clampf(s, 0.0f, 1.0f);

    float delta = s_sine_target[id] - s_sine_start[id];

    /*
     * 半余弦插值：
     * s=0 -> out=start
     * s=1 -> out=target
     * 两端导数为 0，过渡更柔和。
     */
    s_sine_out[id] = s_sine_start[id] + delta * 0.5f * (1.0f - cosf(CURVE_GEN_PI_F * s));

    return s_sine_out[id];
}

/* 只读获取当前输出，不推进内部时间 */
float CurveGen_Sine_Get(uint8_t id)
{
    if (id >= CURVE_GEN_MAX_NUM)
        return 0.0f;

    return s_sine_out[id];
}

/* 查询通道是否处于规划中 */
uint8_t CurveGen_Sine_IsBusy(uint8_t id)
{
    if (id >= CURVE_GEN_MAX_NUM)
        return 0U;

    return s_sine_busy[id];
}
