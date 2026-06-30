#include "Chassis_driver.h"

#include "Curve_generator.h"
#include "M3508_driver.h"
#include "UART3_Protocol.h"

#include <math.h>
#include <string.h>

#define CHASSIS_CMD_POSE_TARGET   0x0CU
#define CHASSIS_POSE_PAYLOAD_LEN  12U
#define CHASSIS_WHEEL_COUNT       4U

PIDf_ParamTypeDef g_chassis_m3508_speedloop_param = {0};

M3508_SpeedLoop_t g_chassis_m3508_201_speedloop = {0};
M3508_SpeedLoop_t g_chassis_m3508_202_speedloop = {0};
M3508_SpeedLoop_t g_chassis_m3508_203_speedloop = {0};
M3508_SpeedLoop_t g_chassis_m3508_204_speedloop = {0};

static float s_last_cmd_vx = 0.0f;
static float s_last_cmd_vy = 0.0f;
static float s_last_cmd_wz = 0.0f;

static float Chassis_DecodeFloat32LE(const uint8_t *payload);
static void Chassis_EncodeFloat32LE(uint8_t *payload, float value);
static float Chassis_GetRotationArm(void);
static float Chassis_SpeedRpmOutToMps(float speed_rpm_out);
static float Chassis_GetWheelSpeedMpsOrZero(uint8_t feedback_index);
static UART3_Protocol_HandlerResult_t Chassis_PoseTargetHandler(const UART3_Protocol_Frame_t *frame, void *context);

void Chassis_m3508_Init(void)
{
    CurveGen_Sine_Reset(CHASSIS_CURVE_VX_ID, 0.0f);
    CurveGen_Sine_Reset(CHASSIS_CURVE_VY_ID, 0.0f);
    CurveGen_Sine_Reset(CHASSIS_CURVE_WZ_ID, 0.0f);
    CurveGen_Sine_StartByRate(CHASSIS_CURVE_VX_ID, 0.0f, CHASSIS_MAX_VX_ACC);
    CurveGen_Sine_StartByRate(CHASSIS_CURVE_VY_ID, 0.0f, CHASSIS_MAX_VY_ACC);
    CurveGen_Sine_StartByRate(CHASSIS_CURVE_WZ_ID, 0.0f, CHASSIS_MAX_WZ_ACC);

    s_last_cmd_vx = 0.0f;
    s_last_cmd_vy = 0.0f;
    s_last_cmd_wz = 0.0f;

    PIDf_ParamInit(&g_chassis_m3508_speedloop_param);
    M3508_SpeedLoop_InitPID(&g_chassis_m3508_speedloop_param);

    (void)M3508_SpeedLoop_Init(&g_chassis_m3508_201_speedloop, 1U, &hcan1, &g_chassis_m3508_speedloop_param);
    (void)M3508_SpeedLoop_Init(&g_chassis_m3508_202_speedloop, 2U, &hcan1, &g_chassis_m3508_speedloop_param);
    (void)M3508_SpeedLoop_Init(&g_chassis_m3508_203_speedloop, 3U, &hcan1, &g_chassis_m3508_speedloop_param);
    (void)M3508_SpeedLoop_Init(&g_chassis_m3508_204_speedloop, 4U, &hcan1, &g_chassis_m3508_speedloop_param);

    M3508_SpeedLoop_Enable(&g_chassis_m3508_201_speedloop, 1U);
    M3508_SpeedLoop_Enable(&g_chassis_m3508_202_speedloop, 1U);
    M3508_SpeedLoop_Enable(&g_chassis_m3508_203_speedloop, 1U);
    M3508_SpeedLoop_Enable(&g_chassis_m3508_204_speedloop, 1U);

    (void)UART3_Protocol_RegisterHandler(CHASSIS_CMD_POSE_TARGET, Chassis_PoseTargetHandler, NULL);
}

void Chassis_201_SetTarget(float target_rpm)
{
    M3508_SpeedLoop_SetTarget(&g_chassis_m3508_201_speedloop, target_rpm);
}

void Chassis_202_SetTarget(float target_rpm)
{
    M3508_SpeedLoop_SetTarget(&g_chassis_m3508_202_speedloop, target_rpm);
}

void Chassis_203_SetTarget(float target_rpm)
{
    M3508_SpeedLoop_SetTarget(&g_chassis_m3508_203_speedloop, target_rpm);
}

void Chassis_204_SetTarget(float target_rpm)
{
    M3508_SpeedLoop_SetTarget(&g_chassis_m3508_204_speedloop, target_rpm);
}

void Chassis_Allm3508_SetTarget(float target_rpm_201,
                                float target_rpm_202,
                                float target_rpm_203,
                                float target_rpm_204)
{
    Chassis_201_SetTarget(target_rpm_201);
    Chassis_202_SetTarget(target_rpm_202);
    Chassis_203_SetTarget(target_rpm_203);
    Chassis_204_SetTarget(target_rpm_204);
}

void Chassis_Set_Velocity_MPS(float target_vx_mps, float target_vy_mps, float target_wz_radps)
{
    if (fabsf(s_last_cmd_vx - target_vx_mps) > CHASSIS_CMD_EPS_VX)
    {
        CurveGen_Sine_StartByRate(CHASSIS_CURVE_VX_ID, target_vx_mps, CHASSIS_MAX_VX_ACC);
        s_last_cmd_vx = target_vx_mps;
    }

    if (fabsf(s_last_cmd_vy - target_vy_mps) > CHASSIS_CMD_EPS_VY)
    {
        CurveGen_Sine_StartByRate(CHASSIS_CURVE_VY_ID, target_vy_mps, CHASSIS_MAX_VY_ACC);
        s_last_cmd_vy = target_vy_mps;
    }

    if (fabsf(s_last_cmd_wz - target_wz_radps) > CHASSIS_CMD_EPS_WZ)
    {
        CurveGen_Sine_StartByRate(CHASSIS_CURVE_WZ_ID, target_wz_radps, CHASSIS_MAX_WZ_ACC);
        s_last_cmd_wz = target_wz_radps;
    }
}

void Chassis_Set_Speed_MPSandRADPS(float target_speed_mps, float target_speed_radps)
{
    Chassis_Set_Velocity_MPS(target_speed_mps, 0.0f, target_speed_radps);
}

void Chassis_Update_once(int16_t *current, float update_time)
{
    float plan_vx = 0.0f;
    float plan_vy = 0.0f;
    float plan_wz = 0.0f;
    float rotation_arm = 0.0f;
    float v201 = 0.0f;
    float v202 = 0.0f;
    float v203 = 0.0f;
    float v204 = 0.0f;

    if (current == NULL)
    {
        return;
    }

    plan_vx = CurveGen_Sine_Update(CHASSIS_CURVE_VX_ID, update_time);
    plan_vy = CurveGen_Sine_Update(CHASSIS_CURVE_VY_ID, update_time);
    plan_wz = CurveGen_Sine_Update(CHASSIS_CURVE_WZ_ID, update_time);

    /* 稳态检测：曲线规划全部结束且目标速度均为零时，提前清零各电机 PID 积分，
       防止积分滞留导致低速极限环振荡。必须在 PID 执行前清零，否则残余力矩
       会通过 CAN 发出，驱动电机微动。 */
    if (CurveGen_Sine_IsBusy(CHASSIS_CURVE_VX_ID) == 0U &&
        CurveGen_Sine_IsBusy(CHASSIS_CURVE_VY_ID) == 0U &&
        CurveGen_Sine_IsBusy(CHASSIS_CURVE_WZ_ID) == 0U &&
        plan_vx == 0.0f && plan_vy == 0.0f && plan_wz == 0.0f)
    {
        g_chassis_m3508_201_speedloop.pid_container.Sum = 0.0f;
        g_chassis_m3508_202_speedloop.pid_container.Sum = 0.0f;
        g_chassis_m3508_203_speedloop.pid_container.Sum = 0.0f;
        g_chassis_m3508_204_speedloop.pid_container.Sum = 0.0f;
    }

    rotation_arm = Chassis_GetRotationArm();
    v201 = plan_vx - plan_vy - rotation_arm * plan_wz;
    v202 = plan_vx + plan_vy - rotation_arm * plan_wz;
    v203 = plan_vx - plan_vy + rotation_arm * plan_wz;
    v204 = plan_vx + plan_vy + rotation_arm * plan_wz;

    Chassis_Allm3508_SetTarget(CHASSIS_DIR_201 * MPS_TO_RPM(v201),
                               CHASSIS_DIR_202 * MPS_TO_RPM(v202),
                               CHASSIS_DIR_203 * MPS_TO_RPM(v203),
                               CHASSIS_DIR_204 * MPS_TO_RPM(v204));

    current[0] = M3508_SpeedLoop_Control(&g_chassis_m3508_201_speedloop);
    current[1] = M3508_SpeedLoop_Control(&g_chassis_m3508_202_speedloop);
    current[2] = M3508_SpeedLoop_Control(&g_chassis_m3508_203_speedloop);
    current[3] = M3508_SpeedLoop_Control(&g_chassis_m3508_204_speedloop);
}

static float Chassis_DecodeFloat32LE(const uint8_t *payload)
{
    float value = 0.0f;
    uint8_t raw[sizeof(float)] = {0U};

    if (payload == NULL)
    {
        return 0.0f;
    }

    raw[0] = payload[0];
    raw[1] = payload[1];
    raw[2] = payload[2];
    raw[3] = payload[3];
    memcpy(&value, raw, sizeof(value));

    return value;
}

static void Chassis_EncodeFloat32LE(uint8_t *payload, float value)
{
    uint8_t raw[sizeof(float)] = {0U};

    if (payload == NULL)
    {
        return;
    }

    memcpy(raw, &value, sizeof(value));
    payload[0] = raw[0];
    payload[1] = raw[1];
    payload[2] = raw[2];
    payload[3] = raw[3];
}

static float Chassis_GetRotationArm(void)
{
    return CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M;
}

static float Chassis_SpeedRpmOutToMps(float speed_rpm_out)
{
    return speed_rpm_out * CHASSIS_WHEEL_CIRCUMFERENCE_M / 60.0f;
}

static float Chassis_GetWheelSpeedMpsOrZero(uint8_t feedback_index)
{
    m3508_feedback_t feedback = {0};

    if (M3508_GetFeedbackSnapshot(feedback_index, &feedback) == 0U)
    {
        return 0.0f;
    }

    if ((feedback.rx_online == 0U) || (feedback.rx_frame_cnt == 0U))
    {
        return 0.0f;
    }

    return Chassis_SpeedRpmOutToMps(feedback.speed_rpm_out);
}

static UART3_Protocol_HandlerResult_t Chassis_PoseTargetHandler(const UART3_Protocol_Frame_t *frame, void *context)
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;

    (void)context;

    if ((frame == NULL) || (frame->payload_len != CHASSIS_POSE_PAYLOAD_LEN))
    {
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    vx = Chassis_DecodeFloat32LE(&frame->payload[0]);
    vy = Chassis_DecodeFloat32LE(&frame->payload[4]);
    wz = Chassis_DecodeFloat32LE(&frame->payload[8]);

    Chassis_Set_Velocity_MPS(vx, vy, wz);

    return UART3_PROTOCOL_HANDLER_DEFER;
}
