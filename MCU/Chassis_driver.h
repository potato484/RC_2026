#ifndef __CHASSIS_DRIVER_H__
#define __CHASSIS_DRIVER_H__

#include "motor_control.h"

/* 未经特殊说明，则本文件中的单位均为标准国际单位。 */

#ifdef __cplusplus
extern "C"
{
#endif

#define CHASSIS_MOTOR_START_ID 1U

#define CHASSIS_CURVE_VX_ID 0U
#define CHASSIS_CURVE_VY_ID 1U
#define CHASSIS_CURVE_WZ_ID 2U

#define CHASSIS_WHEEL_CIRCUMFERENCE_M 0.282743f
#define CHASSIS_HALF_LENGTH_M         0.280f
#define CHASSIS_HALF_WIDTH_M          0.280f

#define CHASSIS_CMD_EPS_VX 0.001f
#define CHASSIS_CMD_EPS_VY 0.001f
#define CHASSIS_CMD_EPS_WZ 0.001f

#define CHASSIS_MAX_VX_ACC 1.0f
#define CHASSIS_MAX_VY_ACC 1.0f
#define CHASSIS_MAX_WZ_ACC 2.0f

#define CHASSIS_DIR_201 1.0f
#define CHASSIS_DIR_202 1.0f
#define CHASSIS_DIR_203 -1.0f
#define CHASSIS_DIR_204 -1.0f

#define MPS_TO_RPM(mps) ((mps) / CHASSIS_WHEEL_CIRCUMFERENCE_M * REDUCTION_RATIO * 60.0f)

void Chassis_m3508_Init(void);

void Chassis_201_SetTarget(float target_rpm);
void Chassis_202_SetTarget(float target_rpm);
void Chassis_203_SetTarget(float target_rpm);
void Chassis_204_SetTarget(float target_rpm);

void Chassis_Allm3508_SetTarget(float target_rpm_201,
                                float target_rpm_202,
                                float target_rpm_203,
                                float target_rpm_204);

void Chassis_Update_once(int16_t *current, float update_time);
void Chassis_Set_Velocity_MPS(float target_vx_mps, float target_vy_mps, float target_wz_radps);
void Chassis_Set_Speed_MPSandRADPS(float target_speed_mps, float target_speed_radps);

#ifdef __cplusplus
}
#endif

#endif
