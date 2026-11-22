#ifndef FRICTION_TEST_H
#define FRICTION_TEST_H

#include <stdint.h>
#include "chassis_actuators.h"

/* 同侧按前关节、后关节、轮排列，供 EP 请求与回读共用。 */
typedef enum
{
    FRICTION_LEFT_FRONT,
    FRICTION_LEFT_REAR,
    FRICTION_LEFT_WHEEL,
    FRICTION_RIGHT_FRONT,
    FRICTION_RIGHT_REAR,
    FRICTION_RIGHT_WHEEL,
    FRICTION_ACTUATOR_COUNT
} FrictionActuator_e;

typedef enum
{
    FRICTION_WAIT_ZERO,
    FRICTION_ARMED = 2,
    FRICTION_ACTIVE,
    FRICTION_FAULT,
    FRICTION_PREP = 5,
    FRICTION_VIRTUAL_ALIGN,
    FRICTION_VIRTUAL_HOLD,
    FRICTION_VIRTUAL_LENGTH,
    FRICTION_PREP_HOLD,
    FRICTION_THIGHS_HOLD,
    FRICTION_CALF_POSE
} FrictionState_e;

typedef enum
{
    FRICTION_END_NONE,
    FRICTION_END_FEEDBACK = 5,
    FRICTION_END_TRAVEL,
    FRICTION_END_SPEED,
    FRICTION_END_TIMEOUT,
    FRICTION_END_REAR_STALL,
    FRICTION_END_PREP_TIMEOUT,
    FRICTION_END_PREP_DIVERGE,
    FRICTION_END_RELEASED
} FrictionEnd_e;

typedef enum
{
    FRICTION_REQUEST_NONE,
    FRICTION_REQUEST_DISARMED,
    FRICTION_REQUEST_ACCEPTED,
    FRICTION_REQUEST_INVALID,
    FRICTION_REQUEST_BUSY,
    FRICTION_REQUEST_FEEDBACK,
    FRICTION_REQUEST_RESET,
    FRICTION_REQUEST_RESET_BLOCKED,
    FRICTION_REQUEST_PREP_DONE,
    FRICTION_REQUEST_RELEASED
} FrictionRequestResult_e;

typedef struct
{
    uint32_t id;
    uint32_t started_tick;
    uint32_t elapsed_ms;
    FrictionActuator_e actuator;
    FrictionEnd_e reason;
    float command_nm;
    float peak_command_nm;
    float peak_speed_rad_s;
    float peak_travel_rad;
    float peak_peer_travel_rad;
    float peak_peer_hold_nm;
    float end_position_rad;
    float end_feedback_torque_nm;
    uint8_t end_feedback_valid;
    uint8_t peer_hold;
} FrictionTrial_s;

typedef struct
{
    uint32_t id;
    uint32_t elapsed_ms;
    FrictionActuator_e actuator;
    FrictionEnd_e reason;
    float target_alpha_rad;
    float alpha_rad;
    float peak_command_nm;
    float command_nm; /* 大腿预定位或保持的力矩，小腿测定期间也独立可读。 */
    uint8_t holding;
    float min_speed_scale; /* 兼容历史 EP 观察，预定位开始后为 1。 */
    float peak_joint_speed_rad_s; /* 到位前大腿和小腿绝对速度的最大值。 */
} FrictionPrepDebug_s;

typedef struct
{
    uint32_t id;
    FrictionEnd_e reason;
    uint8_t reached;
    float reached_theta_rad[2];
    float command_nm[2][2]; /* 左右，前后；撤力后清零 */
    float friction_ff_nm[2][2]; /* 双侧定腿长及归零时的实际摩擦前馈。 */
    float held_position_rad[2][2]; /* 捕获的大腿和小腿原始角。 */
    float reached_length_m[2];
    float reference_speed_rad_s[2][2]; /* 左右、前后电机的原始坐标计划速度。 */
} FrictionVirtualZeroDebug_s;

typedef struct
{
    uint32_t tick;
    uint32_t trial_id;
    uint16_t request_id;
    FrictionRequestResult_e request_result;
    FrictionState_e state;
    FrictionEnd_e gate_reason;
    FrictionActuator_e actuator;
    uint8_t feedback_valid;
    uint8_t has_peer;
    uint8_t reset_ready;
    float request_nm;
    float command_nm;
    float position_rad;
    float speed_rad_s;
    float peer_position_rad;
    float peer_hold_target_rad;
    float peer_hold_command_nm;
    uint8_t peer_hold_active;
    float feedback_torque_nm; /* 关节反馈；轮按反馈电流码折算 */
    float thigh_alpha_rad[2]; /* [0] 左、[1] 右，车身下垂为零 */
    float leg_beta_rad[2];    /* 五连杆张角，采用普通版的运动学标定 */
    float leg_theta_body_rad[2]; /* 髋到轮心连线相对机体向下的摆角 */
    uint8_t leg_angle_valid_mask;
    uint8_t leg_theta_valid_mask;
    int16_t wheel_request_raw;
    int16_t wheel_current_raw; /* C620 原始方向 */
    FrictionPrepDebug_s prep; /* 预定位独立记录；实时下发仍看 command_nm */
    FrictionTrial_s last;
    FrictionVirtualZeroDebug_s virtual_zero;
    float leg_length_m[2];
    struct
    {
        float target_alpha_rad[2];
        float command_nm[2];
        uint8_t active;
    } thighs_hold;
    struct
    {
        uint16_t id;
        uint8_t reached;
        uint8_t use_feedforward;
        float target_raw_rad[2][2];
        float thigh_feedforward_nm[2];
    } calf_pose;
} FrictionDebug_s;

/* EP 单次对齐写入。高 16 位序号、中 3 位电机、低 12 位有符号。
 * V2 为 2 mN·m/码，V3 为 4 mN·m/码，两者共用序号。
 * bit12 = 1 时保持同侧另一关节。低 16 位 0xD000/0xD001 为左/右大腿归零；
 * 0xD002 双侧先定腿长，再归零虚拟腿角并保持，0xD003 撤掉该动作力矩；
 * V3 的 0xC000/0xC800 加有符号 11 位角度码，指定左/右后摆角并保持，每码 4 mrad；
 * V3 的 0xD005 捕获双侧当前大腿角并保持，0xD004 撤掉当前台架动作与保持；
 * V3 的 0xD006/0xD007 启动小腿测定摆位，分别关闭/开启大腿摩擦前馈；
 * V3 高半字节 0xF 是小腿 600 ms 测定，bit11 选右侧，低 11 位补码为 8 mN·m/码。
 * V3 高半字节 0xB 是右轮脉冲，低 12 位补码为 1 mN·m/码。
 * 0xE000 仅清故障、保持零力。 */
extern volatile uint32_t friction_ep_request_v2;
extern volatile uint32_t friction_ep_request_v3;
extern volatile FrictionDebug_s friction_dbg;

#define FRICTION_CALF_PULSE_MS 600u
#define FRICTION_CALF_SAMPLE_MS 5u
#define FRICTION_WHEEL_PULSE_MS 600u
#define FRICTION_WHEEL_COAST_MS 400u
#define FRICTION_CALF_SAMPLE_COUNT ((FRICTION_WHEEL_PULSE_MS + FRICTION_WHEEL_COAST_MS) / FRICTION_CALF_SAMPLE_MS + 1u)

typedef struct
{
    uint32_t elapsed_ms;
    float position_rad;
    float speed_rad_s;
    float torque_nm;
    float thigh_position_rad;
} FrictionCalfSample_s;

typedef struct
{
    uint16_t id;
    uint16_t count;
    FrictionCalfSample_s samples[FRICTION_CALF_SAMPLE_COUNT];
} FrictionCalfTrace_s;

extern volatile FrictionCalfTrace_s friction_calf_trace;

/* 注册完成后调用一次，Init 会令所有执行器进入停止状态。Update 按 1 kHz 调用，tick 为内核节拍。 */
void FrictionTestInit(const ChassisActuators_s *motors);
void FrictionTestUpdate(uint32_t tick);

#endif
