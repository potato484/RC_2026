#ifndef __APP_PLANAR_ARM_H__
#define __APP_PLANAR_ARM_H__

#include "main.h"

#define CLAMP_UP_1_TIME 2500u
#define CLAMP_UP_2_TIME 2500u
#define CLAMP_UP_3_TIME 2500u
#define CLAMP_UP_4_TIME 2500u
#define CLAMP_DOWN_1_TIME 2500u
#define CLAMP_DOWN_2_TIME 2500u
#define CLAMP_DOWN_3_TIME 2500u
#define CLAMP_DOWN_4_TIME 2500u
#define CLAMP_HIGH_1_TIME 2500u
#define CLAMP_HIGH_2_TIME 2500u
#define CLAMP_HIGH_3_TIME 2500u
#define CLAMP_HIGH_4_TIME 2500u

typedef enum
{
    FORW_UPW,   // 机械臂正向，并向上肘
    FORW_DOWNW, // 机械臂正向，并向下肘
    INVT_UPW,   // 机械臂反向，并向上肘
    INVT_DOWNW, // 机械臂反向，并向下肘
} arm_posture_t;

typedef enum
{
    UPCLAMP_READY,
    UPCLAMP_EXTEND,
    UPCLAMP_LIFT,
    UPCLAMP_BACK,
    UPCLAMP_RETURE,
} upclamp_status_t;

typedef enum
{
    DOWNCLAMP_READY,
    DOWNCLAMP_EXTEND,
    DOWNCLAMP_LIFT,
    DOWNCLAMP_BACK,
    DOWNCLAMP_RETURE,
} downclamp_status_t;

typedef enum
{
    PLANAR_ARM_CLAMP_DIR_DOWN = 0U,
    PLANAR_ARM_CLAMP_DIR_UP,
    PLANAR_ARM_CLAMP_DIR_HIGH,
} planar_arm_clamp_dir_t;

typedef struct
{
    float shoulder_angle;
    uint8_t elbows_dir; // elbows_dir = 1时，往前伸出，elbows_dir = 0时，往后伸出
    // 当elbows_dir = 1时，ik_branch = 0是末端执行器水平。当elbows_dir = 0时，ik_branch = 1是末端执行器水平
    uint8_t ik_branch;
    uint8_t last_elbows_dir;
    uint8_t last_ik_branch;
    float arm_height;
} arm_target_t;

extern arm_target_t planar_arm_target;
extern uint32_t clamp_tick;

void planar_arm_init(arm_target_t *target_info);
HAL_StatusTypeDef planar_arm_run_target(arm_target_t *target_info, uint16_t run_time, arm_posture_t posture);
void planar_arm_clamp_process(arm_target_t *target_info, planar_arm_clamp_dir_t dir);
void planar_arm_protocol_update(void);

#endif
