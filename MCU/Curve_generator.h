#ifndef CURVE_GENERATOR_H
#define CURVE_GENERATOR_H

#include <stdint.h>

/*======================================================================================*/
/*                               通用曲线生成器(正弦)                                   */
/*======================================================================================*/

/*
 * 模块定位：
 * 1) 本模块用于“目标值平滑过渡”，核心是半余弦插值：
 *      y(t) = y0 + (1 - cos(pi * t / T)) / 2 * (y1 - y0), t∈[0,T]
 * 2) 该曲线在起点/终点速度均为 0（dy/dt=0），可减小指令突变带来的冲击。
 * 3) 典型用途：底盘速度、机械臂角度、云台目标等“需要平滑”的指令层。
 *
 * 设计约束：
 * 1) 使用固定通道池，不分配动态内存，不依赖外部结构体。
 * 2) 每个通道由 id 唯一标识，通道之间互不影响。
 * 3) 本模块不感知绝对时间，只依赖调用者传入 dt_s。
 *
 * 典型调用流程（每个通道）：
 * 1) 上电或急停后：CurveGen_Sine_Reset(id, current_value)
 * 2) 目标变化时：CurveGen_Sine_StartByRate(...) 或 CurveGen_Sine_StartByTime(...)
 * 3) 固定周期任务中：CurveGen_Sine_Update(id, dt_s)
 * 4) 读取当前输出：CurveGen_Sine_Get(id) 或直接使用 Update 返回值
 *
 * 并发说明：
 * - 本模块为“裸数据+函数”风格，未做锁保护。
 * - 若同一通道同时在中断和主循环中访问，需要调用方自行保证互斥。
 */

#ifndef CURVE_GEN_MAX_NUM
#define CURVE_GEN_MAX_NUM 8U
#endif

/* 单精度 PI 常量，避免依赖 M_PI 宏（不同编译环境可见性不一致） */
#define CURVE_GEN_PI_F 3.14159265358979323846f

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  复位通道状态（立即生效）
 * @note   调用后该通道会停止规划，且 start/target/out 全部被设为 value。
 * @param  id: 通道号，范围 [0, CURVE_GEN_MAX_NUM)
 * @param  value: 复位值（单位由上层决定，如 m/s、deg、rpm）
 */
void CurveGen_Sine_Reset(uint8_t id, float value);

/**
 * @brief  以“总时间”方式启动过渡：当前输出 -> target
 * @note   若 time_s <= 0，会直接跳转到 target（不进入 busy 状态）。
 * @param  id: 通道号，范围 [0, CURVE_GEN_MAX_NUM)
 * @param  target: 目标值（单位由上层决定）
 * @param  time_s: 过渡总时间，单位秒(s)
 */
void CurveGen_Sine_StartByTime(uint8_t id, float target, float time_s);

/**
 * @brief  以“最大变化率”方式启动过渡：当前输出 -> target
 * @note   内部会根据当前差值和 max_rate 自动换算过渡时间 T。
 * @note   max_rate_per_s 表示 |d(output)/dt| 的上限（单位/秒）。
 *         例如 output 是速度(m/s)，则该参数等价于加速度上限(m/s^2)。
 * @note   若 max_rate_per_s <= 0，会直接跳转到 target。
 * @param  id: 通道号，范围 [0, CURVE_GEN_MAX_NUM)
 * @param  target: 目标值（单位由上层决定）
 * @param  max_rate_per_s: 最大变化率（单位/秒）
 */
void CurveGen_Sine_StartByRate(uint8_t id, float target, float max_rate_per_s);

/**
 * @brief  周期更新曲线状态，并返回当前输出
 * @note   建议在固定周期任务中调用，且 dt_s 与实际周期尽量一致。
 * @note   若通道当前不 busy，本函数直接返回当前 out，不做额外计算。
 * @param  id: 通道号，范围 [0, CURVE_GEN_MAX_NUM)
 * @param  dt_s: 本次更新步长，单位秒(s)
 * @return 当前输出值（单位同目标值）
 */
float CurveGen_Sine_Update(uint8_t id, float dt_s);

/**
 * @brief  获取通道当前输出值（只读，不推进时间）
 * @param  id: 通道号，范围 [0, CURVE_GEN_MAX_NUM)
 * @return 当前输出值；id 非法时返回 0
 */
float CurveGen_Sine_Get(uint8_t id);

/**
 * @brief  查询通道是否处于“规划进行中”状态
 * @param  id: 通道号，范围 [0, CURVE_GEN_MAX_NUM)
 * @return 1=忙（正在过渡），0=空闲（已到目标或未启动）
 */
uint8_t CurveGen_Sine_IsBusy(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif
