#include "app_planar_arm.h"
#include "planar_arm_2d.h"
#include "UART3_Protocol.h"

#include <math.h>

#define PLANAR_ARM_HEIGHT_ARRIVE_WINDOW_MM 10.0f
#define PLANAR_ARM_HEIGHT_ARRIVE_HOLD_MS 80U
#define PLANAR_ARM_HEIGHT_TARGET_RESET_EPS_MM 0.5f
#define PLANAR_ARM_RUN_TARGET_XY_RESET_EPS_MM 0.01f
#define PLANAR_VEER_HEIGHT_MIN_LIMIT 0.0f
#define PLANAR_ARM_CMD_GRAB_KFS_DOWN 0x02U
#define PLANAR_ARM_CMD_GRAB_KFS_UP 0x03U
#define PLANAR_ARM_CMD_ARM_RAISE 0x04U
#define PLANAR_ARM_CMD_ARM_LOWER 0x05U
#define PLANAR_ARM_CMD_ARM_HIGH_RAISE 0x0DU
#define PLANAR_ARM_CMD_GRAB_KFS_DOWN_EXTEND 0x0EU
#define PLANAR_ARM_CMD_HIGH_CLAMP 0x0FU
#define PLANAR_ARM_FB_ARM_RAISE_DONE 0x02U
#define PLANAR_ARM_FB_ARM_LOWER_DONE 0x03U
#define PLANAR_ARM_FB_ARM_HIGH_RAISE_DONE 0x09U
#define PLANAR_ARM_FB_KFS_DOWN_EXTEND_DONE 0x0AU
#define PLANAR_ARM_FB_HIGH_CLAMP_DONE 0x0BU
#define PLANAR_ARM_FB_ACTION_FAIL 0xFEU
#define PLANAR_ARM_FAIL_BUSY 0x01U
#define PLANAR_ARM_FAIL_INVALID_PAYLOAD 0x02U
#define PLANAR_ARM_FAIL_NOT_INIT 0x03U
#define PLANAR_ARM_FAIL_HAL_ERROR 0x04U
#define PLANAR_ARM_FAIL_INVALID_STATE 0x05U
#define PLANAR_ARM_BASE_RAISE_HEIGHT_MM 50.0f
#define PLANAR_ARM_BASE_LOWER_HEIGHT_MM (-120.0f)
#define PLANAR_ARM_HIGH_RAISE_HEIGHT_MM 250.0f
#define PLANAR_ARM_CLAMP_STEP_COUNT 4U
#define PLANAR_ARM_CLAMP_UP_TIME_OFFSET_MS 500U
#define PLANAR_ARM_CLAMP_DOWN_TIME_OFFSET_MS 1500U

typedef enum
{
    PLANAR_ARM_RUN_IDLE = 0U,
    PLANAR_ARM_RUN_PRE_LIFT_WAIT,
    PLANAR_ARM_RUN_SERVO_WAIT,
    PLANAR_ARM_RUN_POST_LIFT_WAIT,
} planar_arm_run_phase_t;

typedef struct
{
    float x_mm;
    float y_mm;
    float height_mm;
    uint8_t ik_branch;
    uint16_t run_time_ms;
} planar_arm_run_cmd_t;

typedef enum
{
    PLANAR_ARM_PROTOCOL_IDLE = 0U,
    PLANAR_ARM_PROTOCOL_KFS_DOWN,
    PLANAR_ARM_PROTOCOL_KFS_DOWN_EXTEND,
    PLANAR_ARM_PROTOCOL_KFS_UP,
    PLANAR_ARM_PROTOCOL_HIGH_CLAMP,
    PLANAR_ARM_PROTOCOL_BASE_RAISE,
    PLANAR_ARM_PROTOCOL_BASE_LOWER,
    PLANAR_ARM_PROTOCOL_BASE_HIGH_RAISE,
} planar_arm_protocol_action_t;

typedef enum
{
    PLANAR_ARM_CLAMP_CYLINDER_NONE = 0U,
    PLANAR_ARM_CLAMP_CYLINDER_OPEN,
    PLANAR_ARM_CLAMP_CYLINDER_CLOSE,
} planar_arm_clamp_cylinder_action_t;

typedef struct
{
    float arm_height_mm;
    float shoulder_angle_deg;
    uint16_t wait_time_ms;
    arm_posture_t posture;
    planar_arm_clamp_cylinder_action_t cylinder_action;
    uint8_t next_status;
} planar_arm_clamp_step_t;

typedef struct
{
    planar_arm_clamp_step_t step[PLANAR_ARM_CLAMP_STEP_COUNT];
    uint16_t run_time_offset_ms;
} planar_arm_clamp_config_t;

typedef struct
{
    planar_arm_protocol_action_t action;
    planar_arm_clamp_dir_t clamp_dir;
    uint8_t active_cmd;
    uint8_t active_seq;
    uint8_t busy;
    uint8_t last_cmd;
    uint8_t last_seq;
    uint8_t last_valid;
    uint8_t done_feedback_id;
    float target_height_mm;
} planar_arm_protocol_ctrl_t;

static const planar_arm_clamp_config_t s_planar_arm_clamp_config[] = {
    {
        {
            {-120.0f, 25.0f, CLAMP_DOWN_1_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, DOWNCLAMP_LIFT},
            {50.0f, 135.0f, CLAMP_DOWN_2_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, DOWNCLAMP_BACK},
            {50.0f, 135.0f, CLAMP_DOWN_3_TIME, INVT_UPW, PLANAR_ARM_CLAMP_CYLINDER_CLOSE, DOWNCLAMP_RETURE},
            {0.0f, 135.0f, CLAMP_DOWN_4_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, DOWNCLAMP_READY},
        },
        PLANAR_ARM_CLAMP_DOWN_TIME_OFFSET_MS,
    },
    {
        {
            {-50.0f, 60.0f, CLAMP_UP_1_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_OPEN, UPCLAMP_LIFT},
            {50.0f, 135.0f, CLAMP_UP_2_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, UPCLAMP_BACK},
            {50.0f, 135.0f, CLAMP_UP_3_TIME, INVT_UPW, PLANAR_ARM_CLAMP_CYLINDER_CLOSE, UPCLAMP_RETURE},
            {0.0f, 135.0f, CLAMP_UP_4_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, UPCLAMP_READY},
        },
        PLANAR_ARM_CLAMP_UP_TIME_OFFSET_MS,
    },
    {
        {
            {250.0f, 60.0f, CLAMP_HIGH_1_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_OPEN, UPCLAMP_LIFT},
            {50.0f, 135.0f, CLAMP_HIGH_2_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, UPCLAMP_BACK},
            {50.0f, 135.0f, CLAMP_HIGH_3_TIME, INVT_UPW, PLANAR_ARM_CLAMP_CYLINDER_CLOSE, UPCLAMP_RETURE},
            {250.0f, 135.0f, CLAMP_HIGH_4_TIME, FORW_UPW, PLANAR_ARM_CLAMP_CYLINDER_NONE, UPCLAMP_READY},
        },
        PLANAR_ARM_CLAMP_UP_TIME_OFFSET_MS,
    },
};

static float s_height_arrive_last_target_mm = PLANAR_ARM_BASE_HEIGHT_MM;
static uint32_t s_height_arrive_window_tick_ms = 0U;
static uint8_t s_height_arrive_in_window = 0U;
static uint8_t s_height_ok = 0U;
static upclamp_status_t s_upclamp_status = UPCLAMP_EXTEND;
static downclamp_status_t s_downclamp_status = DOWNCLAMP_READY;
static uint8_t s_downclamp_lift_entry_open_done = 0U;
static planar_arm_protocol_ctrl_t s_planar_arm_protocol = {
    PLANAR_ARM_PROTOCOL_IDLE,
    PLANAR_ARM_CLAMP_DIR_DOWN,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    PLANAR_ARM_BASE_LOWER_HEIGHT_MM,
};

arm_target_t planar_arm_target = {0};
uint8_t g_planar_arm_init_flag = 0U;
cylinder_t arm_cylinder;
uint32_t clamp_tick = 0U;

static uint8_t planar_arm_is_height_arrived(float target_height_mm);
static uint8_t planar_veer_safeguard(const arm_target_t *target_info);
static uint8_t planar_arm_is_same_run_cmd(const planar_arm_run_cmd_t *lhs, const planar_arm_run_cmd_t *rhs);
static void planar_arm_clamp_reset_timing(void);
static void planar_arm_upclamp_reset(void);
static void planar_arm_downclamp_reset(downclamp_status_t status);
static uint8_t planar_arm_clamp_is_done(planar_arm_clamp_dir_t dir);
static void planar_arm_clamp_process_internal(arm_target_t *target_info,
                                              planar_arm_clamp_dir_t dir,
                                              uint8_t allow_transition_chain);
static void planar_arm_protocol_reset(void);
static void planar_arm_protocol_finish(void);
static void planar_arm_protocol_send_action_fail(uint8_t seq, uint8_t failed_cmd, uint8_t error_code);
static uint8_t planar_arm_protocol_is_duplicate(const UART3_Protocol_Frame_t *frame);
static UART3_Protocol_HandlerResult_t planar_arm_protocol_handler(const UART3_Protocol_Frame_t *frame, void *context);

static void planar_arm_init_servo_move(float shoulder_angle_deg, float lift_height_mm)
{
    PlanarArm2D_LiftSetTargetHeight(lift_height_mm);

    float x = -PLANAR_ARM_LINK2_MM +
              PLANAR_ARM_LINK1_MM * sinf((shoulder_angle_deg - PLANAR_ARM_SERVO_MID_DEG) *
                                          PLANAR_ARM_PI_F / 180.0f);
    float y = PLANAR_ARM_LINK1_MM * cosf((shoulder_angle_deg - PLANAR_ARM_SERVO_MID_DEG) *
                                          PLANAR_ARM_PI_F / 180.0f);
    (void)PlanarArm2D_MoveToXY(x, y, 0U, 2000U);
}

void planar_arm_init(arm_target_t *target_info)
{
    if (target_info == NULL)
    {
        return;
    }

    PlanarArm2D_Init();
    PlanarArm2D_LiftEnable(1U);
    Cylinder_Init_Instance(&arm_cylinder, Arm_Cylinder_GPIO_Port, Arm_Cylinder_Pin, GPIO_PIN_SET);

    target_info->arm_height = 250.0f;
    target_info->elbows_dir = 1U;
    target_info->ik_branch = 0U;
    target_info->shoulder_angle = 135.0f;
    target_info->last_elbows_dir = 1U;
    target_info->last_ik_branch = 0U;

    s_height_arrive_last_target_mm = PLANAR_ARM_BASE_HEIGHT_MM;
    s_height_arrive_window_tick_ms = 0U;
    s_height_arrive_in_window = 0U;
    s_height_ok = 0U;
    planar_arm_upclamp_reset();
    planar_arm_downclamp_reset(DOWNCLAMP_READY);
    planar_arm_protocol_reset();
    g_planar_arm_init_flag = 1U;

    planar_arm_init_servo_move(target_info->shoulder_angle, target_info->arm_height);

    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_GRAB_KFS_DOWN, planar_arm_protocol_handler, NULL);
    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_GRAB_KFS_UP, planar_arm_protocol_handler, NULL);
    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_ARM_RAISE, planar_arm_protocol_handler, NULL);
    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_ARM_LOWER, planar_arm_protocol_handler, NULL);
    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_ARM_HIGH_RAISE, planar_arm_protocol_handler, NULL);
    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_GRAB_KFS_DOWN_EXTEND, planar_arm_protocol_handler, NULL);
    (void)UART3_Protocol_RegisterHandler(PLANAR_ARM_CMD_HIGH_CLAMP, planar_arm_protocol_handler, NULL);
}

static uint8_t planar_arm_is_height_arrived(float target_height_mm)
{
    float current_height_mm = PlanarArm2D_LiftGetCurrentHeight();
    float height_error_mm = fabsf(current_height_mm - target_height_mm);
    float target_delta_mm = fabsf(target_height_mm - s_height_arrive_last_target_mm);
    uint32_t now_tick_ms = HAL_GetTick();

    if (target_delta_mm > PLANAR_ARM_HEIGHT_TARGET_RESET_EPS_MM)
    {
        s_height_arrive_window_tick_ms = 0U;
        s_height_arrive_in_window = 0U;
    }

    s_height_arrive_last_target_mm = target_height_mm;

    if (height_error_mm > PLANAR_ARM_HEIGHT_ARRIVE_WINDOW_MM)
    {
        s_height_arrive_window_tick_ms = 0U;
        s_height_arrive_in_window = 0U;
        return 0U;
    }

    if (s_height_arrive_in_window == 0U)
    {
        s_height_arrive_in_window = 1U;
        s_height_arrive_window_tick_ms = now_tick_ms;
        return 0U;
    }

    if ((uint32_t)(now_tick_ms - s_height_arrive_window_tick_ms) >= PLANAR_ARM_HEIGHT_ARRIVE_HOLD_MS)
    {
        return 1U;
    }

    return 0U;
}

static uint8_t planar_veer_safeguard(const arm_target_t *target_info)
{
    if (target_info->last_elbows_dir != target_info->elbows_dir || target_info->last_ik_branch != target_info->ik_branch)
    {
        if (PlanarArm2D_LiftGetCurrentHeight() < PLANAR_VEER_HEIGHT_MIN_LIMIT)
        {
            return 0U;
        }

        return 1U;
    }

    return 1U;
}

static uint8_t planar_arm_is_same_run_cmd(const planar_arm_run_cmd_t *lhs, const planar_arm_run_cmd_t *rhs)
{
    if ((lhs == NULL) || (rhs == NULL))
    {
        return 0U;
    }

    if (fabsf(lhs->x_mm - rhs->x_mm) > PLANAR_ARM_RUN_TARGET_XY_RESET_EPS_MM)
    {
        return 0U;
    }

    if (fabsf(lhs->y_mm - rhs->y_mm) > PLANAR_ARM_RUN_TARGET_XY_RESET_EPS_MM)
    {
        return 0U;
    }

    if (fabsf(lhs->height_mm - rhs->height_mm) > PLANAR_ARM_HEIGHT_TARGET_RESET_EPS_MM)
    {
        return 0U;
    }

    if ((lhs->ik_branch != rhs->ik_branch) || (lhs->run_time_ms != rhs->run_time_ms))
    {
        return 0U;
    }

    return 1U;
}

HAL_StatusTypeDef planar_arm_run_target(arm_target_t *target_info, uint16_t run_time, arm_posture_t posture)
{
    float x = 0.0f;
    float y = 0.0f;
    float cur_h = 0.0f;
    float tgt_h = 0.0f;
    float pre_lift_target = PLANAR_ARM_BASE_HEIGHT_MM;
    uint8_t need_post_lift = 0U;
    uint8_t need_pre_lift = 0U;
    HAL_StatusTypeDef ret = HAL_OK;
    planar_arm_run_cmd_t request_cmd = {0};
    static planar_arm_run_phase_t s_run_phase = PLANAR_ARM_RUN_IDLE;
    static planar_arm_run_cmd_t s_active_cmd = {0};
    static float s_pre_lift_target_mm = PLANAR_ARM_BASE_HEIGHT_MM;
    static float s_post_lift_target_mm = PLANAR_ARM_BASE_HEIGHT_MM;
    static uint32_t s_servo_start_tick_ms = 0U;
    static uint16_t s_servo_run_time_ms = 0U;
    static float s_last_servo_x_mm = 0.0f;
    static float s_last_servo_y_mm = 0.0f;
    static uint8_t s_last_servo_ik_branch = 0U;
    static uint16_t s_last_servo_run_time_ms = 0U;
    static uint8_t s_last_servo_valid = 0U;
    static planar_arm_run_cmd_t s_completed_cmd = {0};
    static uint8_t s_completed_cmd_valid = 0U;

    if (g_planar_arm_init_flag == 0U)
    {
        return HAL_ERROR;
    }

    if ((target_info == NULL) || (run_time > 9999U))
    {
        return HAL_ERROR;
    }

    switch (posture)
    {
    case FORW_UPW:
        target_info->elbows_dir = 1U;
        target_info->ik_branch = 0U;
        break;

    case FORW_DOWNW:
        target_info->elbows_dir = 1U;
        target_info->ik_branch = 1U;
        break;

    case INVT_UPW:
        target_info->elbows_dir = 0U;
        target_info->ik_branch = 1U;
        break;

    case INVT_DOWNW:
        target_info->elbows_dir = 0U;
        target_info->ik_branch = 0U;
        break;

    default:
        return HAL_ERROR;
    }

    if (target_info->elbows_dir != 0U)
    {
        x = -PLANAR_ARM_LINK2_MM;
    }
    else
    {
        x = PLANAR_ARM_LINK2_MM;
    }

    x += PLANAR_ARM_LINK1_MM * sinf((target_info->shoulder_angle - PLANAR_ARM_SERVO_MID_DEG) * PLANAR_ARM_PI_F / 180.0f);
    y = PLANAR_ARM_LINK1_MM * cosf((target_info->shoulder_angle - PLANAR_ARM_SERVO_MID_DEG) * PLANAR_ARM_PI_F / 180.0f);

    request_cmd.x_mm = x;
    request_cmd.y_mm = y;
    request_cmd.height_mm = target_info->arm_height;
    request_cmd.ik_branch = target_info->ik_branch;
    request_cmd.run_time_ms = run_time;

    if ((s_run_phase != PLANAR_ARM_RUN_IDLE) &&
        (planar_arm_is_same_run_cmd(&s_active_cmd, &request_cmd) == 0U))
    {
        s_run_phase = PLANAR_ARM_RUN_IDLE;
    }

    cur_h = PlanarArm2D_LiftGetCurrentHeight();
    tgt_h = target_info->arm_height;

    if (cur_h >= PLANAR_VEER_HEIGHT_MIN_LIMIT)
    {
        pre_lift_target = cur_h;
    }
    else
    {
        pre_lift_target = PLANAR_VEER_HEIGHT_MIN_LIMIT;
    }

    if ((tgt_h > pre_lift_target + PLANAR_ARM_HEIGHT_TARGET_RESET_EPS_MM) ||
        (tgt_h < pre_lift_target - PLANAR_ARM_HEIGHT_TARGET_RESET_EPS_MM))
    {
        need_post_lift = 1U;
    }

    if ((pre_lift_target > cur_h + PLANAR_ARM_HEIGHT_ARRIVE_WINDOW_MM) ||
        (pre_lift_target < cur_h - PLANAR_ARM_HEIGHT_ARRIVE_WINDOW_MM))
    {
        need_pre_lift = 1U;
    }

    if (PlanarArm2D_IsReachable(x, y) == 0U)
    {
        s_run_phase = PLANAR_ARM_RUN_IDLE;
        return HAL_ERROR;
    }

    if ((s_run_phase == PLANAR_ARM_RUN_IDLE) &&
        (s_completed_cmd_valid != 0U) &&
        (planar_arm_is_same_run_cmd(&s_completed_cmd, &request_cmd) != 0U) &&
        (fabsf(PlanarArm2D_LiftGetCurrentHeight() - target_info->arm_height) <= PLANAR_ARM_HEIGHT_ARRIVE_WINDOW_MM))
    {
        return HAL_OK;
    }

    if (s_run_phase == PLANAR_ARM_RUN_IDLE)
    {
        s_active_cmd = request_cmd;
        s_pre_lift_target_mm = pre_lift_target;

        if (need_pre_lift != 0U)
        {
            s_run_phase = PLANAR_ARM_RUN_PRE_LIFT_WAIT;
        }
        else
        {
            s_run_phase = PLANAR_ARM_RUN_SERVO_WAIT;
        }
    }

    if (s_run_phase == PLANAR_ARM_RUN_PRE_LIFT_WAIT)
    {
        PlanarArm2D_LiftSetTargetHeight(s_pre_lift_target_mm);
        if ((PlanarArm2D_LiftGetCurrentHeight() < PLANAR_VEER_HEIGHT_MIN_LIMIT) ||
            (planar_arm_is_height_arrived(s_pre_lift_target_mm) == 0U))
        {
            return HAL_BUSY;
        }

        s_run_phase = PLANAR_ARM_RUN_SERVO_WAIT;
    }

    if (s_run_phase == PLANAR_ARM_RUN_POST_LIFT_WAIT)
    {
        if ((uint32_t)(HAL_GetTick() - s_servo_start_tick_ms) < s_servo_run_time_ms)
        {
            return HAL_BUSY;
        }

        PlanarArm2D_LiftSetTargetHeight(s_post_lift_target_mm);
        if (planar_arm_is_height_arrived(s_post_lift_target_mm) == 0U)
        {
            return HAL_BUSY;
        }

        s_completed_cmd = s_active_cmd;
        s_completed_cmd_valid = 1U;
        s_run_phase = PLANAR_ARM_RUN_IDLE;
        return HAL_OK;
    }

    PlanarArm2D_LiftSetTargetHeight(s_pre_lift_target_mm);

    if (planar_veer_safeguard(target_info) == 0U)
    {
        s_run_phase = PLANAR_ARM_RUN_IDLE;
        return HAL_ERROR;
    }

    if ((s_last_servo_valid == 0U) ||
        (fabsf(s_last_servo_x_mm - x) > PLANAR_ARM_RUN_TARGET_XY_RESET_EPS_MM) ||
        (fabsf(s_last_servo_y_mm - y) > PLANAR_ARM_RUN_TARGET_XY_RESET_EPS_MM) ||
        (s_last_servo_ik_branch != target_info->ik_branch) ||
        (s_last_servo_run_time_ms != run_time))
    {
        ret = PlanarArm2D_MoveToXY(x, y, target_info->ik_branch, run_time);
        if (ret != HAL_OK)
        {
            s_run_phase = PLANAR_ARM_RUN_IDLE;
            return ret;
        }

        s_last_servo_x_mm = x;
        s_last_servo_y_mm = y;
        s_last_servo_ik_branch = target_info->ik_branch;
        s_last_servo_run_time_ms = run_time;
        s_last_servo_valid = 1U;
    }

    target_info->last_elbows_dir = target_info->elbows_dir;
    target_info->last_ik_branch = target_info->ik_branch;

    if (need_post_lift != 0U)
    {
        s_post_lift_target_mm = target_info->arm_height;
        s_servo_run_time_ms = run_time;
        s_servo_start_tick_ms = HAL_GetTick();
        s_completed_cmd_valid = 0U;
        s_run_phase = PLANAR_ARM_RUN_POST_LIFT_WAIT;
        return HAL_BUSY;
    }

    s_completed_cmd = s_active_cmd;
    s_completed_cmd_valid = 1U;
    s_run_phase = PLANAR_ARM_RUN_IDLE;
    return HAL_OK;
}

static void planar_arm_clamp_reset_timing(void)
{
    s_height_ok = 0U;
    clamp_tick = 0U;
}

static void planar_arm_upclamp_reset(void)
{
    s_upclamp_status = UPCLAMP_EXTEND;
    planar_arm_clamp_reset_timing();
}

static void planar_arm_downclamp_reset(downclamp_status_t status)
{
    s_downclamp_status = status;
    s_downclamp_lift_entry_open_done = 0U;
    planar_arm_clamp_reset_timing();
}

static uint8_t planar_arm_clamp_is_done(planar_arm_clamp_dir_t dir)
{
    if ((dir == PLANAR_ARM_CLAMP_DIR_UP) || (dir == PLANAR_ARM_CLAMP_DIR_HIGH))
    {
        return (s_upclamp_status == UPCLAMP_READY) ? 1U : 0U;
    }

    return (s_downclamp_status == DOWNCLAMP_READY) ? 1U : 0U;
}

static uint8_t planar_arm_clamp_get_step_index(planar_arm_clamp_dir_t dir, uint8_t *step_index)
{
    if (step_index == NULL)
    {
        return 0U;
    }

    if ((dir == PLANAR_ARM_CLAMP_DIR_UP) || (dir == PLANAR_ARM_CLAMP_DIR_HIGH))
    {
        switch (s_upclamp_status)
        {
        case UPCLAMP_EXTEND:
            *step_index = 0U;
            return 1U;

        case UPCLAMP_LIFT:
            *step_index = 1U;
            return 1U;

        case UPCLAMP_BACK:
            *step_index = 2U;
            return 1U;

        case UPCLAMP_RETURE:
            *step_index = 3U;
            return 1U;

        default:
            return 0U;
        }
    }

    switch (s_downclamp_status)
    {
    case DOWNCLAMP_EXTEND:
        *step_index = 0U;
        return 1U;

    case DOWNCLAMP_LIFT:
        *step_index = 1U;
        return 1U;

    case DOWNCLAMP_BACK:
        *step_index = 2U;
        return 1U;

    case DOWNCLAMP_RETURE:
        *step_index = 3U;
        return 1U;

    default:
        return 0U;
    }
}

static void planar_arm_clamp_set_next_status(planar_arm_clamp_dir_t dir, uint8_t next_status)
{
    if ((dir == PLANAR_ARM_CLAMP_DIR_UP) || (dir == PLANAR_ARM_CLAMP_DIR_HIGH))
    {
        s_upclamp_status = (upclamp_status_t)next_status;
    }
    else
    {
        s_downclamp_status = (downclamp_status_t)next_status;
    }
}

void planar_arm_clamp_process(arm_target_t *target_info, planar_arm_clamp_dir_t dir)
{
    planar_arm_clamp_process_internal(target_info, dir, 1U);
}

static void planar_arm_clamp_process_internal(arm_target_t *target_info,
                                              planar_arm_clamp_dir_t dir,
                                              uint8_t allow_transition_chain)
{
    const planar_arm_clamp_config_t *config = NULL;
    const planar_arm_clamp_step_t *step = NULL;
    uint8_t step_index = 0U;
    uint8_t transitioned = 0U;
    uint16_t run_time_ms = 0U;

    if ((target_info == NULL) ||
        ((uint32_t)dir >= (sizeof(s_planar_arm_clamp_config) / sizeof(s_planar_arm_clamp_config[0]))))
    {
        return;
    }

    config = &s_planar_arm_clamp_config[dir];

    do
    {
        transitioned = 0U;

        if (planar_arm_clamp_get_step_index(dir, &step_index) == 0U)
        {
            return;
        }

        step = &config->step[step_index];
        target_info->arm_height = step->arm_height_mm;
        target_info->shoulder_angle = step->shoulder_angle_deg;

        if (step->wait_time_ms > config->run_time_offset_ms)
        {
            run_time_ms = step->wait_time_ms - config->run_time_offset_ms;
        }
        else
        {
            run_time_ms = 0U;
        }

        /* Data flow: params -> motion -> height check -> timed wait -> cylinder -> next state */
        {
            HAL_StatusTypeDef ret = planar_arm_run_target(target_info, run_time_ms, step->posture);

            if (s_height_ok == 0U)
            {
                float cur_h = PlanarArm2D_LiftGetCurrentHeight();
                if (fabsf(cur_h - target_info->arm_height) <= PLANAR_ARM_HEIGHT_ARRIVE_WINDOW_MM)
                {
                    s_height_ok = 1U;
                    clamp_tick = step->wait_time_ms;
                }
            }

            if ((ret == HAL_OK) && (clamp_tick == 0U) && (s_height_ok != 0U))
            {
                s_height_ok = 0U;

                if (step->cylinder_action == PLANAR_ARM_CLAMP_CYLINDER_OPEN)
                {
                    Cylinder_Open(&arm_cylinder);
                }
                else if (step->cylinder_action == PLANAR_ARM_CLAMP_CYLINDER_CLOSE)
                {
                    Cylinder_Close(&arm_cylinder);
                }

                planar_arm_clamp_set_next_status(dir, step->next_status);
                transitioned = allow_transition_chain;
            }
        }
    } while (transitioned != 0U);
}

static void planar_arm_protocol_reset(void)
{
    s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_IDLE;
    s_planar_arm_protocol.clamp_dir = PLANAR_ARM_CLAMP_DIR_DOWN;
    s_planar_arm_protocol.active_cmd = 0U;
    s_planar_arm_protocol.active_seq = 0U;
    s_planar_arm_protocol.busy = 0U;
    s_planar_arm_protocol.last_cmd = 0U;
    s_planar_arm_protocol.last_seq = 0U;
    s_planar_arm_protocol.last_valid = 0U;
    s_planar_arm_protocol.done_feedback_id = 0U;
    s_planar_arm_protocol.target_height_mm = PLANAR_ARM_BASE_LOWER_HEIGHT_MM;
}

static void planar_arm_protocol_finish(void)
{
    s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_IDLE;
    s_planar_arm_protocol.busy = 0U;
    s_planar_arm_protocol.done_feedback_id = 0U;
}

static void planar_arm_protocol_send_action_fail(uint8_t seq, uint8_t failed_cmd, uint8_t error_code)
{
    uint8_t payload[2] = {failed_cmd, error_code};
    (void)UART3_Protocol_SendFeedback(seq, PLANAR_ARM_FB_ACTION_FAIL, payload, (uint8_t)sizeof(payload));
}

static uint8_t planar_arm_protocol_is_duplicate(const UART3_Protocol_Frame_t *frame)
{
    if ((frame == NULL) || (s_planar_arm_protocol.last_valid == 0U))
    {
        return 0U;
    }

    if ((s_planar_arm_protocol.last_cmd == frame->cmd) &&
        (s_planar_arm_protocol.last_seq == frame->seq))
    {
        return 1U;
    }

    return 0U;
}

static UART3_Protocol_HandlerResult_t planar_arm_protocol_handler(const UART3_Protocol_Frame_t *frame, void *context)
{
    (void)context;

    if (frame == NULL)
    {
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    if (planar_arm_protocol_is_duplicate(frame) != 0U)
    {
        (void)UART3_Protocol_SendAck(frame->seq);
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    if (frame->payload_len != 0U)
    {
        (void)UART3_Protocol_SendAck(frame->seq);
        planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_INVALID_PAYLOAD);
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    if (g_planar_arm_init_flag == 0U)
    {
        (void)UART3_Protocol_SendAck(frame->seq);
        planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_NOT_INIT);
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    if (s_planar_arm_protocol.busy != 0U)
    {
        (void)UART3_Protocol_SendAck(frame->seq);
        planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_BUSY);
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    s_planar_arm_protocol.active_cmd = frame->cmd;
    s_planar_arm_protocol.active_seq = frame->seq;
    s_planar_arm_protocol.last_cmd = frame->cmd;
    s_planar_arm_protocol.last_seq = frame->seq;
    s_planar_arm_protocol.last_valid = 1U;
    s_planar_arm_protocol.done_feedback_id = 0U;
    s_planar_arm_protocol.busy = 1U;

    switch (frame->cmd)
    {
    case PLANAR_ARM_CMD_GRAB_KFS_DOWN:
        if (s_downclamp_status != DOWNCLAMP_LIFT)
        {
            planar_arm_protocol_finish();
            (void)UART3_Protocol_SendAck(frame->seq);
            planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_INVALID_STATE);
            return UART3_PROTOCOL_HANDLER_DEFER;
        }

        planar_arm_clamp_reset_timing();
        s_planar_arm_protocol.clamp_dir = PLANAR_ARM_CLAMP_DIR_DOWN;
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_KFS_DOWN;
        break;

    case PLANAR_ARM_CMD_GRAB_KFS_DOWN_EXTEND:
        planar_arm_downclamp_reset(DOWNCLAMP_EXTEND);
        s_planar_arm_protocol.clamp_dir = PLANAR_ARM_CLAMP_DIR_DOWN;
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_KFS_DOWN_EXTEND;
        break;

    case PLANAR_ARM_CMD_GRAB_KFS_UP:
        planar_arm_upclamp_reset();
        s_planar_arm_protocol.clamp_dir = PLANAR_ARM_CLAMP_DIR_UP;
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_KFS_UP;
        break;

    case PLANAR_ARM_CMD_HIGH_CLAMP:
        planar_arm_upclamp_reset();
        s_planar_arm_protocol.clamp_dir = PLANAR_ARM_CLAMP_DIR_HIGH;
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_HIGH_CLAMP;
        break;

    case PLANAR_ARM_CMD_ARM_RAISE:
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_BASE_RAISE;
        s_planar_arm_protocol.target_height_mm = PLANAR_ARM_BASE_RAISE_HEIGHT_MM;
        s_planar_arm_protocol.done_feedback_id = PLANAR_ARM_FB_ARM_RAISE_DONE;
        if (PlanarArm2D_LiftSetTargetHeight(s_planar_arm_protocol.target_height_mm) != HAL_OK)
        {
            planar_arm_protocol_finish();
            (void)UART3_Protocol_SendAck(frame->seq);
            planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_HAL_ERROR);
            return UART3_PROTOCOL_HANDLER_DEFER;
        }
        break;

    case PLANAR_ARM_CMD_ARM_LOWER:
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_BASE_LOWER;
        s_planar_arm_protocol.target_height_mm = PLANAR_ARM_BASE_LOWER_HEIGHT_MM;
        s_planar_arm_protocol.done_feedback_id = PLANAR_ARM_FB_ARM_LOWER_DONE;
        if (PlanarArm2D_LiftSetTargetHeight(s_planar_arm_protocol.target_height_mm) != HAL_OK)
        {
            planar_arm_protocol_finish();
            (void)UART3_Protocol_SendAck(frame->seq);
            planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_HAL_ERROR);
            return UART3_PROTOCOL_HANDLER_DEFER;
        }
        break;

    case PLANAR_ARM_CMD_ARM_HIGH_RAISE:
        s_planar_arm_protocol.action = PLANAR_ARM_PROTOCOL_BASE_HIGH_RAISE;
        s_planar_arm_protocol.target_height_mm = PLANAR_ARM_HIGH_RAISE_HEIGHT_MM;
        s_planar_arm_protocol.done_feedback_id = PLANAR_ARM_FB_ARM_HIGH_RAISE_DONE;
        if (PlanarArm2D_LiftSetTargetHeight(s_planar_arm_protocol.target_height_mm) != HAL_OK)
        {
            planar_arm_protocol_finish();
            (void)UART3_Protocol_SendAck(frame->seq);
            planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_HAL_ERROR);
            return UART3_PROTOCOL_HANDLER_DEFER;
        }
        break;

    default:
        planar_arm_protocol_finish();
        (void)UART3_Protocol_SendAck(frame->seq);
        planar_arm_protocol_send_action_fail(frame->seq, frame->cmd, PLANAR_ARM_FAIL_INVALID_PAYLOAD);
        return UART3_PROTOCOL_HANDLER_DEFER;
    }

    (void)UART3_Protocol_SendAck(frame->seq);
    return UART3_PROTOCOL_HANDLER_DEFER;
}

void planar_arm_protocol_update(void)
{
    if ((g_planar_arm_init_flag == 0U) || (s_planar_arm_protocol.busy == 0U))
    {
        return;
    }

    switch (s_planar_arm_protocol.action)
    {
    case PLANAR_ARM_PROTOCOL_KFS_DOWN:
        if ((s_downclamp_status == DOWNCLAMP_LIFT) && (s_downclamp_lift_entry_open_done == 0U))
        {
            Cylinder_Open(&arm_cylinder);
            s_downclamp_lift_entry_open_done = 1U;
        }

        planar_arm_clamp_process(&planar_arm_target, PLANAR_ARM_CLAMP_DIR_DOWN);
        if (planar_arm_clamp_is_done(PLANAR_ARM_CLAMP_DIR_DOWN) != 0U)
        {
            planar_arm_protocol_finish();
        }
        break;

    case PLANAR_ARM_PROTOCOL_KFS_UP:
        planar_arm_clamp_process(&planar_arm_target, s_planar_arm_protocol.clamp_dir);
        if (planar_arm_clamp_is_done(s_planar_arm_protocol.clamp_dir) != 0U)
        {
            planar_arm_protocol_finish();
        }
        break;

    case PLANAR_ARM_PROTOCOL_HIGH_CLAMP:
        planar_arm_clamp_process(&planar_arm_target, PLANAR_ARM_CLAMP_DIR_HIGH);
        if (planar_arm_clamp_is_done(PLANAR_ARM_CLAMP_DIR_HIGH) != 0U)
        {
            (void)UART3_Protocol_SendFeedback(s_planar_arm_protocol.active_seq,
                                              PLANAR_ARM_FB_HIGH_CLAMP_DONE,
                                              NULL,
                                              0U);
            planar_arm_protocol_finish();
        }
        break;

    case PLANAR_ARM_PROTOCOL_KFS_DOWN_EXTEND:
        planar_arm_clamp_process_internal(&planar_arm_target, PLANAR_ARM_CLAMP_DIR_DOWN, 0U);
        if (s_downclamp_status == DOWNCLAMP_LIFT)
        {
            (void)UART3_Protocol_SendFeedback(s_planar_arm_protocol.active_seq,
                                              PLANAR_ARM_FB_KFS_DOWN_EXTEND_DONE,
                                              NULL,
                                              0U);
            planar_arm_protocol_finish();
        }
        break;

    case PLANAR_ARM_PROTOCOL_BASE_RAISE:
    case PLANAR_ARM_PROTOCOL_BASE_LOWER:
    case PLANAR_ARM_PROTOCOL_BASE_HIGH_RAISE:
        if (PlanarArm2D_LiftSetTargetHeight(s_planar_arm_protocol.target_height_mm) != HAL_OK)
        {
            planar_arm_protocol_send_action_fail(s_planar_arm_protocol.active_seq,
                                                 s_planar_arm_protocol.active_cmd,
                                                 PLANAR_ARM_FAIL_HAL_ERROR);
            planar_arm_protocol_finish();
            break;
        }

        if (planar_arm_is_height_arrived(s_planar_arm_protocol.target_height_mm) != 0U)
        {
            (void)UART3_Protocol_SendFeedback(s_planar_arm_protocol.active_seq,
                                              s_planar_arm_protocol.done_feedback_id,
                                              NULL,
                                              0U);
            planar_arm_protocol_finish();
        }
        break;

    default:
        planar_arm_protocol_finish();
        break;
    }
}
