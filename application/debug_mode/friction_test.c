#include "friction_test.h"

#include <math.h>
#include "FreeRTOS.h"
#include "chassis.h"
#include "power_control.h"

#define DEG_TO_RAD 0.01745329252f
#define REAR_FEEDBACK_GAP_MS 20u
/* 伴随关节使用台架位置控制。 */
#define PEER_HOLD_KP 2.0f              /* N·m/rad */
#define PEER_HOLD_KD 0.08f             /* N·m/(rad/s) */
#define EP_TORQUE_SIGN 0x0800u
#define EP_TORQUE_MASK 0x0FFFu
#define EP_V2_TORQUE_CODES_PER_NM 500.0f
#define EP_V3_TORQUE_CODES_PER_NM 250.0f
#define EP_PEER_HOLD 0x1000u
#define EP_RESET_PAYLOAD 0xE000u
#define EP_PREP_LEFT_PAYLOAD 0xD000u
#define EP_PREP_RIGHT_PAYLOAD 0xD001u
#define EP_VIRTUAL_ZERO_PAYLOAD 0xD002u
#define EP_VIRTUAL_RELEASE_PAYLOAD 0xD003u
#define EP_THIGH_RELEASE_PAYLOAD 0xD004u
#define EP_THIGHS_CAPTURE_PAYLOAD 0xD005u
#define EP_CALF_POSE_PAYLOAD 0xD006u
#define EP_CALF_POSE_FEEDFORWARD_PAYLOAD 0xD007u
#define EP_RIGHT_WHEEL_PULSE_TAG 0xB000u
#define EP_CALF_PULSE_TAG 0xF000u
#define EP_CALF_PULSE_RIGHT 0x0800u
#define EP_CALF_PULSE_VALUE 0x07FFu
#define EP_CALF_PULSE_SIGN 0x0400u
#define EP_CALF_PULSE_CODES_PER_NM 125.0f
#define EP_THIGH_TARGET_MASK 0xF000u
#define EP_THIGH_TARGET_PAYLOAD 0xC000u
#define EP_THIGH_RIGHT 0x0800u
#define EP_THIGH_ANGLE_MASK 0x07FFu
#define EP_THIGH_ANGLE_SIGN 0x0400u
#define EP_THIGH_RAD_PER_CODE 0.004f
#define THIGH_HALF_TURN_RAD 3.14159265f
#define CALF_POSE_ALPHA_RAD (-0.5f * THIGH_HALF_TURN_RAD)
#define CALF_POSE_BETA_RAD (-CALF_POSE_ALPHA_RAD)
#define CALF_POSE_REFERENCE_RAD_S 0.30f
#define CALF_POSE_FF_SPEED_RAD_S 0.05f
#define CALF_POSE_STILL_DELTA_RAD 0.025f
#define CALF_POSE_KP 14.0f
#define CALF_POSE_INTEGRAL_NM 3.5f

/* 预定位用单侧大腿编码角闭环，小腿零力随动；参数待新版实车复核。 */
#define PREP_KP 6.0f
#define PREP_KI 3.0f
#define PREP_KI_LIMIT_NM 1.2f
#define PREP_KD 0.6f
#define PREP_DONE_ALPHA_RAD 0.05f
#define PREP_DONE_SPEED_RAD_S 0.15f
#define PREP_DONE_MS 300u
#define DM_FEEDBACK_ENABLED 1u

/* 先定腿长，再调整虚拟腿角度。 */
#define VIRTUAL_TARGET_LENGTH_M 0.30f
#define VIRTUAL_STILL_MS 500u
#define VIRTUAL_STILL_RANGE_RAD 0.01f
#define VIRTUAL_FRICTION_SPEED_RAD_S 0.15f
#define VIRTUAL_FRICTION_GAIN 1.0f /* 2026-11-22 计划速度版 25% 实车运动平顺，改为全量进行对照。 */
#define VIRTUAL_REFERENCE_PEAK_RAD_S 0.30f
/* 左腿在约 0.39 m 停滞时，原 Kp 与积分限幅只给出约 3.3 N·m。 */
#define VIRTUAL_LEFT_LENGTH_KP 24.0f

typedef struct
{
    DMMotorInstance *joint;
    DMMotorInstance *peer;
    DJIMotorInstance *wheel;
    float wheel_rad_per_deg;
    float wheel_sign;
} FrictionMotor_s;

typedef struct
{
    CANInstance *can;
    void (*decode)(CANInstance *);
    volatile uint32_t frames;
} RearFeedbackSource_s;

typedef struct
{
    float position_rad;
    float speed_rad_s;
    float peer_position_rad;
    float peer_speed_rad_s;
    float torque_nm;
    int16_t current_raw;
    uint32_t peer_frames;
    uint8_t joint_enabled;
    uint8_t valid;
} FrictionFeedback_s;

typedef struct
{
    float target_rad[2];
    float reference_start_rad[2];
    float start_theta_rad;
    float integral_nm[2];
    float hold_bias_nm[2];
    uint32_t rear_last_frame;
    uint32_t rear_last_tick;
} VirtualLegControl_s;

static const FrictionActuator_e front_actuator[2] = {FRICTION_LEFT_FRONT, FRICTION_RIGHT_FRONT};
/* 2026-11-22 台架动摩擦候选，按左/右、前/后，力矩幅值由原始电机坐标下的计划速度赋予方向。 */
static const float joint_friction_nm[2][2] = {{0.89f, 0.94f}, {1.42f, 0.70f}};
static VirtualLegControl_s virtual_leg[2];
static uint32_t virtual_still_tick;
static uint32_t virtual_reference_tick;
static float virtual_reference_duration_s;
static uint8_t virtual_still_running;
static float virtual_min_rad[2][2], virtual_max_rad[2][2];
static FrictionMotor_s motors[FRICTION_ACTUATOR_COUNT];
static FrictionState_e state;
static FrictionEnd_e gate_reason;
static FrictionActuator_e selected = FRICTION_ACTUATOR_COUNT;
static uint16_t seen_request;
static uint32_t observed_request_v2, observed_request_v3;
static float active_torque_nm;
static uint8_t pulse_active;
static uint32_t calf_last_sample_tick;
static uint32_t calf_pose_last_tick, calf_pose_still_tick;
static uint8_t calf_pose_still;
static uint8_t calf_pose_hold;
static float calf_pose_reference[2][2];
static float calf_pose_still_alpha[2];
static RearFeedbackSource_s rear_feedback[2];
static FrictionTrial_s trial;
static float start_position;
static float start_peer_position;
static uint32_t prep_start_tick;
static uint32_t prep_still_since;
static uint8_t prep_still_running;
static float prep_integral_nm;
static float thighs_hold_integral_nm[2];
static uint32_t thighs_hold_rear_frame[2], thighs_hold_rear_tick[2];
static uint8_t prep_keep_hold;
static uint32_t prep_peer_last_frame, prep_peer_last_tick;

volatile uint32_t friction_ep_request_v2;
volatile uint32_t friction_ep_request_v3;
volatile FrictionCalfTrace_s friction_calf_trace;
volatile FrictionDebug_s friction_dbg = {
    .actuator = FRICTION_ACTUATOR_COUNT,
    .last.actuator = FRICTION_ACTUATOR_COUNT,
    .prep.actuator = FRICTION_ACTUATOR_COUNT,
};

static void CountRearFeedback(CANInstance *can)
{
    RearFeedbackSource_s *source = &rear_feedback[can == rear_feedback[0].can ? 0 : 1];
    source->decode(can);
    ++source->frames;
}

static void WatchRearFeedback(unsigned side, DMMotorInstance *motor)
{
    CANInstance *can = motor->motor_can_instace;
    rear_feedback[side].can = can;
    rear_feedback[side].decode = can->can_module_callback;
    can->can_module_callback = CountRearFeedback;
}

static void StopJoint(DMMotorInstance *joint)
{
    DMMotorStop(joint);
    DMMotorSetRef(joint, 0.0f);
}

static void StopMotor(const FrictionMotor_s *motor)
{
    if (motor->joint)
        StopJoint(motor->joint);
    else
    {
        DJIMotorStop(motor->wheel);
        DJIMotorSetRef(motor->wheel, 0.0f);
    }
}

static void ApplyTorque(const FrictionMotor_s *motor, float torque_nm)
{
    if (motor->joint)
    {
        DMMotorSetRef(motor->joint, torque_nm);
        DMMotorEnable(motor->joint);
        friction_dbg.wheel_request_raw = 0;
    }
    else
    {
        /* 转成 CAN 电流字段前限制到 C620 可表示的指令范围，避免窄化溢出。 */
        const float current = motor->wheel_sign * WHEEL_T_PER_RAW * torque_nm;
        const int16_t raw = (int16_t)fminf(16384.0f, fmaxf(-16384.0f, current));
        DJIMotorSetRef(motor->wheel, raw);
        DJIMotorEnable(motor->wheel);
        friction_dbg.wheel_request_raw = raw;
    }
    friction_dbg.command_nm = torque_nm;
}

static uint8_t JointOnline(const DMMotorInstance *joint)
{
    return joint->measure.init_flag && DaemonIsOnline(joint->motor_daemon);
}

static FrictionFeedback_s ReadFeedback(const FrictionMotor_s *motor)
{
    FrictionFeedback_s sample = {0};
    /* 同一快照读驱动字段与在线标志，避免 CAN 中断在位置和速度之间更新。 */
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (motor->joint)
    {
        sample.position_rad = motor->joint->measure.total_position;
        sample.speed_rad_s = motor->joint->measure.velocity;
        sample.torque_nm = motor->joint->measure.torque;
        sample.peer_position_rad = motor->peer->measure.total_position;
        sample.peer_speed_rad_s = motor->peer->measure.velocity;
        sample.valid = JointOnline(motor->joint) && JointOnline(motor->peer);
        sample.joint_enabled = (motor->joint->motor_can_instace->rx_buff[0] >> 4) == DM_FEEDBACK_ENABLED;
        if (motor->peer->motor_can_instace == rear_feedback[0].can)
            sample.peer_frames = rear_feedback[0].frames;
        else if (motor->peer->motor_can_instace == rear_feedback[1].can)
            sample.peer_frames = rear_feedback[1].frames;
    }
    else
    {
        sample.position_rad = motor->wheel->measure.total_angle;
        sample.speed_rad_s = motor->wheel->measure.speed_aps;
        sample.current_raw = motor->wheel->measure.real_current;
        /* Daemon 注册后有初始宽限，feed_cnt 由首帧反馈更新。 */
        sample.valid = motor->wheel->feed_cnt != 0 && DaemonIsOnline(motor->wheel->daemon);
    }
    __set_PRIMASK(irq_state);

    if (motor->wheel)
    {
        const float signed_scale = motor->wheel_rad_per_deg * motor->wheel_sign;
        sample.position_rad *= signed_scale;
        sample.speed_rad_s *= signed_scale;
        sample.torque_nm = motor->wheel_sign * sample.current_raw / WHEEL_T_PER_RAW;
    }
    sample.valid = sample.valid && isfinite(sample.position_rad) && isfinite(sample.speed_rad_s) &&
                   isfinite(sample.torque_nm) && isfinite(sample.peer_position_rad) &&
                   isfinite(sample.peer_speed_rad_s);
    return sample;
}

static void StopPrepOutput(FrictionEnd_e reason, uint32_t tick)
{
    StopMotor(&motors[friction_dbg.prep.actuator]);
    friction_dbg.prep.command_nm = 0.0f;
    friction_dbg.prep.holding = 0;
    friction_dbg.prep.elapsed_ms = (tick - prep_start_tick) * portTICK_PERIOD_MS;
    friction_dbg.prep.reason = reason;
    prep_keep_hold = 0;
}

static void StopThighsHold(void)
{
    for (unsigned side = 0; side < 2; ++side)
    {
        StopMotor(&motors[front_actuator[side]]);
        friction_dbg.thighs_hold.command_nm[side] = 0.0f;
    }
    friction_dbg.thighs_hold.active = 0;
    calf_pose_hold = 0;
}

static void EndTrial(FrictionEnd_e reason, uint32_t tick, const FrictionFeedback_s *sample)
{
    if (state == FRICTION_ACTIVE)
    {
        StopMotor(&motors[selected]);
        if (trial.peer_hold)
            StopJoint(motors[selected].peer);
        trial.elapsed_ms = (tick - trial.started_tick) * portTICK_PERIOD_MS;
        trial.reason = reason;
        trial.end_position_rad = sample->position_rad;
        trial.end_feedback_torque_nm = sample->torque_nm;
        trial.end_feedback_valid = sample->valid;
        friction_dbg.last = trial;
    }
    pulse_active = 0;
    active_torque_nm = 0.0f;
    friction_dbg.command_nm = 0.0f;
    friction_dbg.wheel_request_raw = 0;
    friction_dbg.peer_hold_command_nm = 0.0f;
    friction_dbg.reset_ready = 0;
    if (reason == FRICTION_END_TIMEOUT)
    {
        /* 脉冲到期只撤被测电机，保留双侧大腿目标。 */
        selected = FRICTION_ACTUATOR_COUNT;
        state = friction_dbg.thighs_hold.active ? FRICTION_THIGHS_HOLD : FRICTION_ARMED;
        gate_reason = reason;
        return;
    }
    if (prep_keep_hold)
        StopPrepOutput(reason, tick);
    if (friction_dbg.thighs_hold.active)
    {
        StopThighsHold();
        selected = FRICTION_ACTUATOR_COUNT;
    }
    state = reason == FRICTION_END_RELEASED ? FRICTION_ARMED : FRICTION_FAULT;
    gate_reason = reason;
}

static void StartTrial(const FrictionFeedback_s *sample, uint16_t id, float torque,
                       uint8_t peer_hold, uint32_t tick)
{
    trial = (FrictionTrial_s){
        .id = id, .actuator = selected, .started_tick = tick,
        .command_nm = torque, .peer_hold = peer_hold,
    };
    active_torque_nm = torque;
    start_position = sample->position_rad;
    start_peer_position = sample->peer_position_rad;
    state = FRICTION_ACTIVE;
    gate_reason = FRICTION_END_NONE;
}

static void RecordMotion(const FrictionFeedback_s *sample)
{
    const float travel = fabsf(sample->position_rad - start_position);
    const float peer_travel = fabsf(sample->peer_position_rad - start_peer_position);
    const float speed = fabsf(sample->speed_rad_s);
    trial.peak_travel_rad = fmaxf(trial.peak_travel_rad, travel);
    trial.peak_peer_travel_rad = fmaxf(trial.peak_peer_travel_rad, peer_travel);
    trial.peak_speed_rad_s = fmaxf(trial.peak_speed_rad_s, speed);
}

static FrictionRequestResult_e TryStartRequest(uint32_t word, uint8_t use_v3,
                                               FrictionFeedback_s *sample, uint32_t tick)
{
    const unsigned actuator = (word >> 13) & 7u;
    const uint8_t negative = (word & EP_TORQUE_SIGN) != 0;
    const uint8_t peer_hold = (word & EP_PEER_HOLD) != 0;
    const int32_t torque_code = (int32_t)(word & EP_TORQUE_MASK) - (negative ? 4096 : 0);
    const float codes_per_nm = use_v3 ? EP_V3_TORQUE_CODES_PER_NM : EP_V2_TORQUE_CODES_PER_NM;
    const float torque = (float)torque_code / codes_per_nm;
    if (actuator >= FRICTION_ACTUATOR_COUNT || torque_code == 0 ||
        (peer_hold && !motors[actuator].peer))
        return FRICTION_REQUEST_INVALID;
    if (state == FRICTION_PREP_HOLD &&
        (!use_v3 || peer_hold || actuator != (unsigned)friction_dbg.prep.actuator + 1u))
        return FRICTION_REQUEST_INVALID;

    if (state == FRICTION_THIGHS_HOLD &&
        (!use_v3 || peer_hold || (actuator != FRICTION_LEFT_REAR && actuator != FRICTION_RIGHT_REAR)))
        return FRICTION_REQUEST_INVALID;

    selected = (FrictionActuator_e)actuator;
    *sample = ReadFeedback(&motors[selected]);
    if (!sample->valid || ((prep_keep_hold || friction_dbg.thighs_hold.active) && !sample->joint_enabled))
    {
        EndTrial(FRICTION_END_FEEDBACK, tick, sample);
        return FRICTION_REQUEST_FEEDBACK;
    }
    StartTrial(sample, seen_request, torque, peer_hold, tick);
    return FRICTION_REQUEST_ACCEPTED;
}

static FrictionRequestResult_e StartPulse(FrictionActuator_e actuator, float torque_nm,
                                          FrictionFeedback_s *sample, uint32_t tick)
{
    if (torque_nm == 0.0f)
        return FRICTION_REQUEST_INVALID;

    const FrictionFeedback_s initial = ReadFeedback(&motors[actuator]);
    if (!initial.valid || (motors[actuator].joint && !initial.joint_enabled))
        return FRICTION_REQUEST_FEEDBACK;

    selected = actuator;
    *sample = initial;
    friction_calf_trace.id = seen_request;
    friction_calf_trace.count = 0;
    calf_last_sample_tick = tick;
    pulse_active = 1;
    StartTrial(sample, seen_request, torque_nm, 0, tick);
    return FRICTION_REQUEST_ACCEPTED;
}

static FrictionRequestResult_e StartCalfPulse(uint16_t payload, FrictionFeedback_s *sample, uint32_t tick)
{
    const FrictionActuator_e actuator = (payload & EP_CALF_PULSE_RIGHT) ? FRICTION_RIGHT_REAR : FRICTION_LEFT_REAR;
    const int code = (int)(payload & EP_CALF_PULSE_VALUE) - ((payload & EP_CALF_PULSE_SIGN) ? 2048 : 0);
    return StartPulse(actuator, (float)code / EP_CALF_PULSE_CODES_PER_NM, sample, tick);
}

static uint8_t CapturePulseSample(const FrictionFeedback_s *sample, uint32_t tick)
{
    const uint32_t elapsed_ms = (tick - trial.started_tick) * portTICK_PERIOD_MS;
    const uint16_t index = friction_calf_trace.count;
    const uint32_t duration_ms = selected == FRICTION_RIGHT_WHEEL ?
        FRICTION_WHEEL_PULSE_MS + FRICTION_WHEEL_COAST_MS : FRICTION_CALF_PULSE_MS;
    const uint8_t finished = elapsed_ms >= duration_ms;
    if (index < FRICTION_CALF_SAMPLE_COUNT &&
        (index == 0 || finished || tick - calf_last_sample_tick >= pdMS_TO_TICKS(FRICTION_CALF_SAMPLE_MS)))
    {
        friction_calf_trace.samples[index] = (FrictionCalfSample_s){
            .elapsed_ms = elapsed_ms, .position_rad = sample->position_rad,
            .speed_rad_s = sample->speed_rad_s, .torque_nm = sample->torque_nm,
            .thigh_position_rad = sample->peer_position_rad,
        };
        friction_calf_trace.count = index + 1u;
        calf_last_sample_tick = tick;
    }
    return finished;
}

static void ApplyPeerHold(DMMotorInstance *peer, const FrictionFeedback_s *sample)
{
    /* 用原始电机方向。连续编码器以试验起点为参考，跨角度边界时没有误差跳变。 */
    const float error = start_peer_position - sample->peer_position_rad;
    const float torque = PEER_HOLD_KP * error - PEER_HOLD_KD * sample->peer_speed_rad_s;
    DMMotorSetRef(peer, torque);
    DMMotorEnable(peer);
    friction_dbg.peer_hold_command_nm = torque;
    trial.peak_peer_hold_nm = fmaxf(trial.peak_peer_hold_nm, fabsf(torque));
}

static float PrepAlpha(FrictionActuator_e actuator, const FrictionFeedback_s *sample)
{
    if (actuator == FRICTION_LEFT_FRONT)
    {
        l_leg_Resolve leg;
        l_Alpha_beta_calculate(-sample->position_rad, -sample->peer_position_rad,
                               -sample->speed_rad_s, -sample->peer_speed_rad_s, &leg);
        return leg.l_alpha;
    }
    r_leg_Resolve leg;
    r_Alpha_beta_calculate(sample->position_rad, sample->peer_position_rad,
                           sample->speed_rad_s, sample->peer_speed_rad_s, &leg);
    return leg.r_alpha;
}

static FrictionRequestResult_e StartPrep(FrictionActuator_e actuator, float target_alpha_rad,
                                        uint8_t keep_hold, FrictionFeedback_s *sample, uint32_t tick)
{
    const FrictionFeedback_s initial = ReadFeedback(&motors[actuator]);
    if (!initial.valid || (keep_hold && !initial.joint_enabled))
        return FRICTION_REQUEST_FEEDBACK;

    const float alpha = PrepAlpha(actuator, &initial);
    if (!isfinite(alpha) || (keep_hold && (target_alpha_rad >= 0.0f || target_alpha_rad < -THIGH_HALF_TURN_RAD)))
        return FRICTION_REQUEST_INVALID;

    selected = actuator;
    *sample = initial;
    prep_start_tick = tick;
    prep_still_running = 0;
    prep_integral_nm = 0.0f;
    prep_keep_hold = keep_hold;
    prep_peer_last_frame = initial.peer_frames;
    prep_peer_last_tick = tick;
    friction_dbg.prep = (FrictionPrepDebug_s){
        .id = seen_request, .actuator = actuator, .target_alpha_rad = target_alpha_rad, .alpha_rad = alpha,
        .min_speed_scale = 1.0f,
    };
    state = FRICTION_PREP;
    gate_reason = FRICTION_END_NONE;
    return FRICTION_REQUEST_ACCEPTED;
}

static void EndPrep(FrictionEnd_e reason, uint32_t tick)
{
    StopPrepOutput(reason, tick);
    friction_dbg.command_nm = 0.0f;
    friction_dbg.reset_ready = 0;
    gate_reason = reason;
    state = reason == FRICTION_END_NONE || reason == FRICTION_END_RELEASED ? FRICTION_ARMED : FRICTION_FAULT;
    if (reason == FRICTION_END_NONE && seen_request == friction_dbg.prep.id)
        friction_dbg.request_result = FRICTION_REQUEST_PREP_DONE;
}

static uint8_t PrepFeedbackValid(const FrictionFeedback_s *sample, uint32_t tick)
{
    if (!sample->valid)
        return 0;
    if (!prep_keep_hold)
        return 1;
    if (sample->peer_frames != prep_peer_last_frame)
    {
        prep_peer_last_frame = sample->peer_frames;
        prep_peer_last_tick = tick;
    }
    return sample->joint_enabled && tick - prep_peer_last_tick < pdMS_TO_TICKS(REAR_FEEDBACK_GAP_MS);
}

static float ThighAngleError(float alpha, float target)
{
    /* 观测角在 ±π 处回绕，误差也按一圈取值，避免回绕时反向绕一整圈。 */
    return remainderf(alpha - target, 2.0f * THIGH_HALF_TURN_RAD);
}

static float ThighTorque(FrictionActuator_e actuator, float error, float speed_rad_s, float *integral_nm)
{
    *integral_nm = fminf(PREP_KI_LIMIT_NM, fmaxf(-PREP_KI_LIMIT_NM,
        *integral_nm + PREP_KI * error * 0.001f));
    /* 左前原始正力矩减小 alpha，右前镜像相反。 */
    const float sign = actuator == FRICTION_LEFT_FRONT ? 1.0f : -1.0f;
    return sign * (PREP_KP * error + *integral_nm) - PREP_KD * speed_rad_s;
}

static void ApplyPrepTorque(float error, float speed_rad_s)
{
    const FrictionActuator_e actuator = friction_dbg.prep.actuator;
    const float torque = ThighTorque(actuator, error, speed_rad_s, &prep_integral_nm);
    DMMotorSetRef(motors[actuator].joint, torque);
    DMMotorEnable(motors[actuator].joint);
    if (state != FRICTION_ACTIVE)
        friction_dbg.command_nm = torque;
    friction_dbg.prep.command_nm = torque;
    friction_dbg.prep.peak_command_nm = fmaxf(friction_dbg.prep.peak_command_nm, fabsf(torque));
}

static void PrepUpdate(const FrictionFeedback_s *sample, uint32_t tick)
{
    if (!PrepFeedbackValid(sample, tick))
    {
        EndPrep(FRICTION_END_FEEDBACK, tick);
        return;
    }
    const float alpha = PrepAlpha(selected, sample);
    if (!isfinite(alpha))
    {
        EndPrep(FRICTION_END_FEEDBACK, tick);
        return;
    }
    friction_dbg.prep.alpha_rad = alpha;
    const float error = ThighAngleError(alpha, friction_dbg.prep.target_alpha_rad);
    const float joint_speed = fmaxf(fabsf(sample->speed_rad_s), fabsf(sample->peer_speed_rad_s));
    friction_dbg.prep.peak_joint_speed_rad_s = fmaxf(friction_dbg.prep.peak_joint_speed_rad_s, joint_speed);

    const uint8_t done = fabsf(error) <= PREP_DONE_ALPHA_RAD &&
        fabsf(sample->speed_rad_s) <= PREP_DONE_SPEED_RAD_S &&
        fabsf(sample->peer_speed_rad_s) <= PREP_DONE_SPEED_RAD_S;
    if (done && !prep_still_running)
        prep_still_since = tick;
    prep_still_running = done;
    if (done && tick - prep_still_since >= pdMS_TO_TICKS(PREP_DONE_MS))
    {
        if (!prep_keep_hold)
        {
            EndPrep(FRICTION_END_NONE, tick);
            return;
        }
        state = FRICTION_PREP_HOLD;
        friction_dbg.prep.holding = 1;
        friction_dbg.prep.elapsed_ms = (tick - prep_start_tick) * portTICK_PERIOD_MS;
        if (seen_request == friction_dbg.prep.id)
            friction_dbg.request_result = FRICTION_REQUEST_PREP_DONE;
    }
    ApplyPrepTorque(error, sample->speed_rad_s);
}

static FrictionEnd_e UpdatePrepHold(const FrictionFeedback_s *sample, uint32_t tick)
{
    if (!PrepFeedbackValid(sample, tick))
        return FRICTION_END_FEEDBACK;
    const FrictionActuator_e actuator = friction_dbg.prep.actuator;

    const float alpha = PrepAlpha(actuator, sample);
    if (!isfinite(alpha))
        return FRICTION_END_FEEDBACK;
    friction_dbg.prep.alpha_rad = alpha;
    const float error = ThighAngleError(alpha, friction_dbg.prep.target_alpha_rad);
    ApplyPrepTorque(error, sample->speed_rad_s);
    return FRICTION_END_NONE;
}

static uint8_t FeedbackStill(const FrictionFeedback_s *sample, float speed_limit)
{
    return sample->valid && fabsf(sample->speed_rad_s) <= speed_limit &&
           fabsf(sample->peer_speed_rad_s) <= speed_limit;
}

static FrictionRequestResult_e CaptureThighsHold(const FrictionFeedback_s legs[2], uint32_t tick)
{
    if (friction_dbg.leg_angle_valid_mask != 3 || !legs[0].joint_enabled || !legs[1].joint_enabled)
        return FRICTION_REQUEST_FEEDBACK;
    if (!isfinite(friction_dbg.thigh_alpha_rad[0]) || !isfinite(friction_dbg.thigh_alpha_rad[1]))
        return FRICTION_REQUEST_FEEDBACK;
    for (unsigned side = 0; side < 2; ++side)
    {
        friction_dbg.thighs_hold.target_alpha_rad[side] = friction_dbg.thigh_alpha_rad[side];
        thighs_hold_integral_nm[side] = 0.0f;
        thighs_hold_rear_frame[side] = legs[side].peer_frames;
        thighs_hold_rear_tick[side] = tick;
    }
    selected = FRICTION_ACTUATOR_COUNT;
    state = FRICTION_THIGHS_HOLD;
    gate_reason = FRICTION_END_NONE;
    friction_dbg.thighs_hold.active = 1;
    return FRICTION_REQUEST_ACCEPTED;
}

static FrictionEnd_e UpdateThighsHold(const FrictionFeedback_s legs[2], uint32_t tick)
{
    float error[2];
    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionFeedback_s *leg = &legs[side];
        if (leg->peer_frames != thighs_hold_rear_frame[side])
        {
            thighs_hold_rear_frame[side] = leg->peer_frames;
            thighs_hold_rear_tick[side] = tick;
        }
        if (!leg->valid || !leg->joint_enabled ||
            tick - thighs_hold_rear_tick[side] >= pdMS_TO_TICKS(REAR_FEEDBACK_GAP_MS))
            return FRICTION_END_FEEDBACK;
        error[side] = ThighAngleError(friction_dbg.thigh_alpha_rad[side], friction_dbg.thighs_hold.target_alpha_rad[side]);
        if (!isfinite(error[side]))
            return FRICTION_END_FEEDBACK;
    }
    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionActuator_e actuator = front_actuator[side];
        float torque;
        if (calf_pose_hold)
        {
            /* 摆位实测在 1.2 N·m 积分上限附近仍有静差。保持时延续摆位控制和原积分。 */
            float *integral = &thighs_hold_integral_nm[side];
            *integral = fminf(CALF_POSE_INTEGRAL_NM, fmaxf(-CALF_POSE_INTEGRAL_NM,
                *integral + PREP_KI * error[side] * 0.001f));
            const float direction = side == 0 ? 1.0f : -1.0f;
            torque = direction * (CALF_POSE_KP * error[side] + *integral) - PREP_KD * legs[side].speed_rad_s;
        }
        else
            torque = ThighTorque(actuator, error[side], legs[side].speed_rad_s, &thighs_hold_integral_nm[side]);
        DMMotorSetRef(motors[actuator].joint, torque);
        DMMotorEnable(motors[actuator].joint);
        friction_dbg.thighs_hold.command_nm[side] = torque;
    }
    return FRICTION_END_NONE;
}

static void StartVirtualReference(uint32_t tick)
{
    float max_travel_rad = 0.0f;
    for (unsigned side = 0; side < 2; ++side)
        for (unsigned joint = 0; joint < 2; ++joint)
            max_travel_rad = fmaxf(max_travel_rad,
                fabsf(virtual_leg[side].target_rad[joint] - virtual_leg[side].reference_start_rad[joint]));

    /* 五次曲线的归一化峰速为 1.875，四路共用时长以保持左右及前后相位一致。 */
    virtual_reference_duration_s = 1.875f * max_travel_rad / VIRTUAL_REFERENCE_PEAK_RAD_S;
    virtual_reference_tick = tick;
    virtual_still_running = 0;
}

static FrictionRequestResult_e StartVirtualZero(const FrictionFeedback_s legs[2], uint32_t tick)
{
    if (friction_dbg.leg_theta_valid_mask != 3)
        return FRICTION_REQUEST_FEEDBACK;
    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionFeedback_s *sample = &legs[side];
        const float thigh_m = side == 0 ? L_LEG_A : R_LEG_A;
        const float shin_m = side == 0 ? L_LEG_B : R_LEG_B;
        const float length_m = VIRTUAL_TARGET_LENGTH_M;
        /* 用普通版连杆尺寸做逆解。关节反向等量走，保持 θ = α + β/2。 */
        const float half_beta = acosf((length_m * length_m + thigh_m * thigh_m - shin_m * shin_m) /
                                      (2.0f * length_m * thigh_m));
        const float raw_direction = side == 0 ? -1.0f : 1.0f;
        const float delta = raw_direction * (half_beta - 0.5f * friction_dbg.leg_beta_rad[side]);
        if (!isfinite(delta))
            return FRICTION_REQUEST_INVALID;
        virtual_leg[side] = (VirtualLegControl_s){
            .target_rad = {sample->position_rad - delta, sample->peer_position_rad + delta},
            .reference_start_rad = {sample->position_rad, sample->peer_position_rad},
            .start_theta_rad = friction_dbg.leg_theta_body_rad[side],
            .rear_last_frame = sample->peer_frames, .rear_last_tick = tick,
        };
    }
    friction_dbg.virtual_zero = (FrictionVirtualZeroDebug_s){.id = seen_request};
    selected = FRICTION_ACTUATOR_COUNT;
    active_torque_nm = 0.0f;
    friction_dbg.command_nm = 0.0f;
    gate_reason = FRICTION_END_NONE;
    StartVirtualReference(tick);
    state = FRICTION_VIRTUAL_LENGTH;
    return FRICTION_REQUEST_ACCEPTED;
}

static FrictionRequestResult_e StartCalfPose(const FrictionFeedback_s legs[2], uint8_t use_ff, uint32_t tick)
{
    if (friction_dbg.leg_angle_valid_mask != 3)
        return FRICTION_REQUEST_FEEDBACK;

    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionActuator_e front = front_actuator[side];
        const FrictionFeedback_s rear = ReadFeedback(&motors[front + 1]);
        if (!legs[side].joint_enabled || !rear.valid || !rear.joint_enabled)
            return FRICTION_REQUEST_FEEDBACK;
        if (!FeedbackStill(&legs[side], PREP_DONE_SPEED_RAD_S))
            return FRICTION_REQUEST_BUSY;
        if (!isfinite(friction_dbg.thigh_alpha_rad[side]) || !isfinite(friction_dbg.leg_beta_rad[side]))
            return FRICTION_REQUEST_INVALID;
    }
    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionFeedback_s *leg = &legs[side];
        const float sign = side == 0 ? -1.0f : 1.0f;
        const float alpha = friction_dbg.thigh_alpha_rad[side];
        const float beta = friction_dbg.leg_beta_rad[side];
        /* 后侧主动杆角度为 alpha + beta。按原始电机方向换算两根主动杆的位置。 */
        virtual_leg[side] = (VirtualLegControl_s){
            .target_rad = {
                leg->position_rad + sign * ThighAngleError(CALF_POSE_ALPHA_RAD, alpha),
                leg->peer_position_rad + sign * ThighAngleError(CALF_POSE_ALPHA_RAD + CALF_POSE_BETA_RAD, alpha + beta),
            },
            .rear_last_frame = leg->peer_frames,
            .rear_last_tick = tick,
        };
        calf_pose_reference[side][0] = leg->position_rad;
        calf_pose_reference[side][1] = leg->peer_position_rad;
        friction_dbg.calf_pose.target_raw_rad[side][0] = virtual_leg[side].target_rad[0];
        friction_dbg.calf_pose.target_raw_rad[side][1] = virtual_leg[side].target_rad[1];
    }

    selected = FRICTION_ACTUATOR_COUNT;
    calf_pose_last_tick = tick;
    calf_pose_still = 0;
    friction_dbg.calf_pose.id = seen_request;
    friction_dbg.calf_pose.reached = 0;
    friction_dbg.calf_pose.use_feedforward = use_ff;
    friction_dbg.virtual_zero = (FrictionVirtualZeroDebug_s){0};
    state = FRICTION_CALF_POSE;
    gate_reason = FRICTION_END_NONE;
    return FRICTION_REQUEST_ACCEPTED;
}

static void EndVirtualZero(FrictionEnd_e reason)
{
    for (unsigned side = 0; side < 2; ++side)
    {
        StopMotor(&motors[front_actuator[side]]);
        StopJoint(motors[front_actuator[side]].peer);
        for (unsigned joint = 0; joint < 2; ++joint)
        {
            friction_dbg.virtual_zero.command_nm[side][joint] = 0.0f;
            friction_dbg.virtual_zero.friction_ff_nm[side][joint] = 0.0f;
            friction_dbg.virtual_zero.reference_speed_rad_s[side][joint] = 0.0f;
        }
    }
    friction_dbg.virtual_zero.reason = reason;
    friction_dbg.calf_pose.thigh_feedforward_nm[0] = 0.0f;
    friction_dbg.calf_pose.thigh_feedforward_nm[1] = 0.0f;
    friction_dbg.reset_ready = 0;
    gate_reason = reason;
    state = reason == FRICTION_END_RELEASED ? FRICTION_ARMED : FRICTION_FAULT;
}

static FrictionEnd_e VirtualLegCommand(unsigned side, const FrictionFeedback_s *sample, uint32_t tick,
                                       float progress, float progress_rate, float torque[2])
{
    VirtualLegControl_s *leg = &virtual_leg[side];
    if (sample->peer_frames != leg->rear_last_frame)
    {
        leg->rear_last_frame = sample->peer_frames;
        leg->rear_last_tick = tick;
    }
    if (tick - leg->rear_last_tick >= pdMS_TO_TICKS(REAR_FEEDBACK_GAP_MS))
        return FRICTION_END_FEEDBACK;
    const float position[2] = {sample->position_rad, sample->peer_position_rad};
    const float speed[2] = {sample->speed_rad_s, sample->peer_speed_rad_s};
    const uint8_t setting_length = state == FRICTION_VIRTUAL_LENGTH;

    const float kp = side == 0 && setting_length ? VIRTUAL_LEFT_LENGTH_KP : PREP_KP;
    for (unsigned joint = 0; joint < 2; ++joint)
    {
        const float travel = leg->target_rad[joint] - leg->reference_start_rad[joint];
        const float reference_rad = progress == 1.0f ? leg->target_rad[joint] :
            leg->reference_start_rad[joint] + travel * progress;
        const float reference_speed = travel * progress_rate;
        const float error = reference_rad - position[joint];
        const float integral = leg->integral_nm[joint] + PREP_KI * error * 0.001f;
        leg->integral_nm[joint] = fminf(PREP_KI_LIMIT_NM, fmaxf(-PREP_KI_LIMIT_NM, integral));
        const float ff = VIRTUAL_FRICTION_GAIN * joint_friction_nm[side][joint] *
            tanhf(reference_speed / VIRTUAL_FRICTION_SPEED_RAD_S);
        friction_dbg.virtual_zero.reference_speed_rad_s[side][joint] = reference_speed;
        friction_dbg.virtual_zero.friction_ff_nm[side][joint] = ff;
        torque[joint] = kp * error + leg->hold_bias_nm[joint] + leg->integral_nm[joint] +
            PREP_KD * (reference_speed - speed[joint]) + ff;
    }
    return FRICTION_END_NONE;
}

static uint8_t VirtualAnglesStill(const FrictionFeedback_s legs[2], uint32_t tick)
{
    uint8_t reset_window = !virtual_still_running;
    for (unsigned side = 0; side < 2; ++side)
    {
        const float position[2] = {legs[side].position_rad, legs[side].peer_position_rad};
        for (unsigned joint = 0; joint < 2; ++joint)
        {
            virtual_min_rad[side][joint] = fminf(virtual_min_rad[side][joint], position[joint]);
            virtual_max_rad[side][joint] = fmaxf(virtual_max_rad[side][joint], position[joint]);
            if (virtual_max_rad[side][joint] - virtual_min_rad[side][joint] > VIRTUAL_STILL_RANGE_RAD)
                reset_window = 1;
        }
    }
    if (reset_window)
    {
        for (unsigned side = 0; side < 2; ++side)
        {
            virtual_min_rad[side][0] = virtual_max_rad[side][0] = legs[side].position_rad;
            virtual_min_rad[side][1] = virtual_max_rad[side][1] = legs[side].peer_position_rad;
        }
        virtual_still_tick = tick;
        virtual_still_running = 1;
    }
    return tick - virtual_still_tick >= pdMS_TO_TICKS(VIRTUAL_STILL_MS);
}

static void UpdateVirtualZero(const FrictionFeedback_s legs[2], uint32_t tick)
{
    if (friction_dbg.leg_theta_valid_mask != 3)
    {
        EndVirtualZero(FRICTION_END_FEEDBACK);
        return;
    }
    const float elapsed_s = (tick - virtual_reference_tick) * portTICK_PERIOD_MS * 0.001f;
    const uint8_t tracking = state != FRICTION_VIRTUAL_HOLD && elapsed_s < virtual_reference_duration_s;
    float progress = 1.0f;
    float progress_rate = 0.0f;
    if (tracking)
    {
        const float u = elapsed_s / virtual_reference_duration_s;
        const float remaining = 1.0f - u;
        progress = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
        progress_rate = 30.0f * u * u * remaining * remaining / virtual_reference_duration_s;
    }

    float torque[2][2];
    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionEnd_e reason = VirtualLegCommand(side, &legs[side], tick, progress,
                                                       progress_rate, torque[side]);
        if (reason != FRICTION_END_NONE)
        {
            EndVirtualZero(reason);
            return;
        }
    }
    if (!tracking && state != FRICTION_VIRTUAL_HOLD && VirtualAnglesStill(legs, tick))
    {
        if (state == FRICTION_VIRTUAL_LENGTH)
        {
            for (unsigned side = 0; side < 2; ++side)
            {
                VirtualLegControl_s *leg = &virtual_leg[side];
                const float delta = (side == 0 ? 1.0f : -1.0f) * leg->start_theta_rad;
                for (unsigned joint = 0; joint < 2; ++joint)
                {
                    leg->reference_start_rad[joint] = leg->target_rad[joint];
                    leg->target_rad[joint] += delta;
                }
            }
            StartVirtualReference(tick);
            state = FRICTION_VIRTUAL_ALIGN;
        }
        else
        {
            for (unsigned side = 0; side < 2; ++side)
            {
                VirtualLegControl_s *leg = &virtual_leg[side];
                const float position[2] = {legs[side].position_rad, legs[side].peer_position_rad};
                const float speed[2] = {legs[side].speed_rad_s, legs[side].peer_speed_rad_s};
                for (unsigned joint = 0; joint < 2; ++joint)
                {
                    /* 冻结当前电机角，保留静态支撑分量，避免进入保持时撤力。 */
                    leg->target_rad[joint] = position[joint];
                    leg->reference_start_rad[joint] = position[joint];
                    leg->hold_bias_nm[joint] = torque[side][joint] + PREP_KD * speed[joint];
                    leg->integral_nm[joint] = 0.0f;
                    friction_dbg.virtual_zero.held_position_rad[side][joint] = position[joint];
                }
                friction_dbg.virtual_zero.reached_theta_rad[side] = friction_dbg.leg_theta_body_rad[side];
                friction_dbg.virtual_zero.reached_length_m[side] = friction_dbg.leg_length_m[side];
            }
            friction_dbg.virtual_zero.reached = 1;
            state = FRICTION_VIRTUAL_HOLD;
        }
    }
    for (unsigned side = 0; side < 2; ++side)
        for (unsigned joint = 0; joint < 2; ++joint)
        {
            DMMotorInstance *motor = motors[front_actuator[side] + joint].joint;
            DMMotorSetRef(motor, torque[side][joint]);
            DMMotorEnable(motor);
            friction_dbg.virtual_zero.command_nm[side][joint] = torque[side][joint];
        }
}

static void UpdateCalfPose(const FrictionFeedback_s legs[2], uint32_t tick)
{
    /* 速度方向只从摆位轨迹取，静止保持和小腿力矩测定时前馈为零。 */
    const float dt_s = (tick - calf_pose_last_tick) * portTICK_PERIOD_MS * 0.001f;
    calf_pose_last_tick = tick;
    const float reference_step = CALF_POSE_REFERENCE_RAD_S * dt_s;
    uint8_t reference_complete = 1;

    for (unsigned side = 0; side < 2; ++side)
    {
        const FrictionFeedback_s *sample = &legs[side];
        VirtualLegControl_s *leg = &virtual_leg[side];
        if (!sample->valid || !sample->joint_enabled)
        {
            EndVirtualZero(FRICTION_END_FEEDBACK);
            return;
        }
        if (sample->peer_frames != leg->rear_last_frame)
        {
            leg->rear_last_frame = sample->peer_frames;
            leg->rear_last_tick = tick;
        }
        if (tick - leg->rear_last_tick >= pdMS_TO_TICKS(REAR_FEEDBACK_GAP_MS))
        {
            EndVirtualZero(FRICTION_END_FEEDBACK);
            return;
        }
        const float position[2] = {sample->position_rad, sample->peer_position_rad};
        const float speed[2] = {sample->speed_rad_s, sample->peer_speed_rad_s};
        for (unsigned joint = 0; joint < 2; ++joint)
        {
            const float delta = leg->target_rad[joint] - calf_pose_reference[side][joint];
            const float step = fminf(reference_step, fmaxf(-reference_step, delta));
            calf_pose_reference[side][joint] += step;
            const float error = calf_pose_reference[side][joint] - position[joint];
            leg->integral_nm[joint] = fminf(CALF_POSE_INTEGRAL_NM, fmaxf(-CALF_POSE_INTEGRAL_NM,
                leg->integral_nm[joint] + PREP_KI * error * dt_s));
            reference_complete = reference_complete && fabsf(delta) <= reference_step;

            const float ff = joint == 0 && friction_dbg.calf_pose.use_feedforward &&
                dt_s > 0.0f && fabsf(step) >= CALF_POSE_FF_SPEED_RAD_S * dt_s ?
                copysignf(joint_friction_nm[side][0], step) : 0.0f;
            const float torque = CALF_POSE_KP * error + leg->integral_nm[joint] - PREP_KD * speed[joint] + ff;
            DMMotorInstance *motor = motors[front_actuator[side] + joint].joint;
            DMMotorSetRef(motor, torque);
            DMMotorEnable(motor);
            friction_dbg.virtual_zero.command_nm[side][joint] = torque;
            if (joint == 0)
                friction_dbg.calf_pose.thigh_feedforward_nm[side] = ff;
        }
    }

    /* 指令走完后只看两侧大腿角是否稳定，直接捕获实测角作为小腿测定支撑。 */
    if (!reference_complete)
        return;

    if (!calf_pose_still ||
        fabsf(ThighAngleError(friction_dbg.thigh_alpha_rad[0], calf_pose_still_alpha[0])) > CALF_POSE_STILL_DELTA_RAD ||
        fabsf(ThighAngleError(friction_dbg.thigh_alpha_rad[1], calf_pose_still_alpha[1])) > CALF_POSE_STILL_DELTA_RAD)
    {
        calf_pose_still_alpha[0] = friction_dbg.thigh_alpha_rad[0];
        calf_pose_still_alpha[1] = friction_dbg.thigh_alpha_rad[1];
        calf_pose_still_tick = tick;
        calf_pose_still = 1;
        return;
    }
    if (tick - calf_pose_still_tick < pdMS_TO_TICKS(PREP_DONE_MS))
        return;

    for (unsigned side = 0; side < 2; ++side)
    {
        const float sign = side == 0 ? -1.0f : 1.0f;
        StopJoint(motors[front_actuator[side]].peer);
        /* 捕获已停稳的实际角，将切换前的比例出力带入积分，减小保持切换时的力矩跳变。 */
        friction_dbg.thighs_hold.target_alpha_rad[side] = friction_dbg.thigh_alpha_rad[side];
        friction_dbg.thighs_hold.command_nm[side] = friction_dbg.virtual_zero.command_nm[side][0];
        const float effort = friction_dbg.thighs_hold.command_nm[side] + PREP_KD * legs[side].speed_rad_s;
        thighs_hold_integral_nm[side] = fminf(CALF_POSE_INTEGRAL_NM,
            fmaxf(-CALF_POSE_INTEGRAL_NM, -sign * effort));
        thighs_hold_rear_frame[side] = legs[side].peer_frames;
        thighs_hold_rear_tick[side] = tick;
        friction_dbg.virtual_zero.command_nm[side][0] = 0.0f;
        friction_dbg.virtual_zero.command_nm[side][1] = 0.0f;
        friction_dbg.calf_pose.thigh_feedforward_nm[side] = 0.0f;
    }
    friction_dbg.thighs_hold.active = 1;
    friction_dbg.calf_pose.reached = 1;
    calf_pose_hold = 1;
    state = FRICTION_THIGHS_HOLD;
}

static void HandleFault(uint16_t payload, uint8_t new_request, uint8_t feedback_valid)
{
    friction_dbg.reset_ready = feedback_valid;
    if (!new_request || payload != EP_RESET_PAYLOAD)
        return;
    if (!feedback_valid)
    {
        friction_dbg.request_result = FRICTION_REQUEST_RESET_BLOCKED;
        return;
    }

    /* 复位只回到零力待试，保留上一笔摘要并消费本次序号。 */
    state = FRICTION_ARMED;
    gate_reason = FRICTION_END_NONE;
    friction_dbg.reset_ready = 0;
    friction_dbg.request_result = FRICTION_REQUEST_RESET;
}

static uint16_t ObserveRequest(uint32_t word, uint32_t *observed)
{
    /* 两路共用序号。仅处理实际变化的字，避免另一入口的旧字在序号绕回时重放。 */
    if (word == *observed)
        return 0;
    *observed = word;
    const uint16_t id = (uint16_t)(word >> 16);
    const uint16_t advance = (uint16_t)(id - seen_request);
    return id != 0 && advance < 32768u ? advance : 0;
}

static void UpdateOutput(FrictionFeedback_s *sample, const FrictionFeedback_s legs[2], uint32_t tick)
{
    const uint32_t word_v2 = friction_ep_request_v2;
    const uint32_t word_v3 = friction_ep_request_v3;
    const uint16_t advance_v2 = ObserveRequest(word_v2, &observed_request_v2);
    const uint16_t advance_v3 = ObserveRequest(word_v3, &observed_request_v3);
    /* 同拍写入时只接受更新的序号。序号相同优先 V3，两笔均不留队。 */
    const uint8_t use_v3 = advance_v3 >= advance_v2;
    const uint32_t word = use_v3 ? word_v3 : word_v2;
    const uint16_t payload = (uint16_t)word;
    const uint16_t id = (uint16_t)(word >> 16);
    const uint8_t new_request = advance_v2 != 0 || advance_v3 != 0;
    if (new_request)
    {
        /* 先消费序号，启动、故障或试验中收到的力矩请求不能排队。 */
        seen_request = id;
        friction_dbg.request_id = id;
        friction_dbg.request_result = FRICTION_REQUEST_DISARMED;
    }

    if (state == FRICTION_FAULT)
    {
        const uint8_t feedback_valid = selected < FRICTION_ACTUATOR_COUNT ? sample->valid :
            legs[0].valid && legs[1].valid;
        HandleFault(payload, new_request, feedback_valid);
        return;
    }
    if (state == FRICTION_WAIT_ZERO)
    {
        state = FRICTION_ARMED;
        gate_reason = FRICTION_END_NONE;
        return;
    }
    if (new_request && use_v3 && payload == EP_THIGH_RELEASE_PAYLOAD)
    {
        if (state == FRICTION_ACTIVE || state == FRICTION_THIGHS_HOLD)
            EndTrial(FRICTION_END_RELEASED, tick, sample);
        else if (state == FRICTION_PREP || state == FRICTION_PREP_HOLD)
            EndPrep(FRICTION_END_RELEASED, tick);
        else if (state == FRICTION_VIRTUAL_LENGTH || state == FRICTION_VIRTUAL_ALIGN ||
                 state == FRICTION_VIRTUAL_HOLD || state == FRICTION_CALF_POSE)
            EndVirtualZero(FRICTION_END_RELEASED);
        friction_dbg.request_result = FRICTION_REQUEST_RELEASED;
        return;
    }
    if (state == FRICTION_CALF_POSE)
    {
        if (new_request)
            friction_dbg.request_result = FRICTION_REQUEST_BUSY;
        UpdateCalfPose(legs, tick);
        return;
    }
    if (friction_dbg.thighs_hold.active)
    {
        const FrictionEnd_e reason = UpdateThighsHold(legs, tick);
        if (reason != FRICTION_END_NONE)
        {
            if (reason == FRICTION_END_FEEDBACK)
                sample->valid = 0;
            EndTrial(reason, tick, sample);
            return;
        }
    }
    if (state == FRICTION_PREP)
    {
        if (new_request)
            friction_dbg.request_result = FRICTION_REQUEST_BUSY;
        PrepUpdate(sample, tick);
        return;
    }
    if (state == FRICTION_VIRTUAL_LENGTH || state == FRICTION_VIRTUAL_ALIGN || state == FRICTION_VIRTUAL_HOLD)
    {
        if (new_request && payload == EP_VIRTUAL_RELEASE_PAYLOAD)
        {
            EndVirtualZero(FRICTION_END_RELEASED);
            friction_dbg.request_result = FRICTION_REQUEST_RELEASED;
            return;
        }
        if (new_request)
            friction_dbg.request_result = FRICTION_REQUEST_BUSY;
        UpdateVirtualZero(legs, tick);
        return;
    }
    if (prep_keep_hold)
    {
        const FrictionActuator_e actuator = friction_dbg.prep.actuator;
        const FrictionFeedback_s *front = &legs[actuator == FRICTION_LEFT_FRONT ? 0 : 1];
        if (state == FRICTION_PREP_HOLD)
        {
            selected = actuator;
            *sample = *front;
        }
        const FrictionEnd_e reason = UpdatePrepHold(front, tick);
        if (reason != FRICTION_END_NONE)
        {
            if (reason == FRICTION_END_FEEDBACK)
                sample->valid = 0;
            if (state == FRICTION_ACTIVE)
                EndTrial(reason, tick, sample);
            else
                EndPrep(reason, tick);
            return;
        }
    }
    if (new_request)
    {
        if (state == FRICTION_ACTIVE)
            friction_dbg.request_result = FRICTION_REQUEST_BUSY;
        else if (use_v3 && (payload & 0xF000u) == EP_RIGHT_WHEEL_PULSE_TAG)
        {
            const int code = (int)(payload & EP_TORQUE_MASK) - ((payload & EP_TORQUE_SIGN) ? 4096 : 0);
            friction_dbg.request_result = (state == FRICTION_ARMED || state == FRICTION_THIGHS_HOLD) ?
                StartPulse(FRICTION_RIGHT_WHEEL, (float)code * 0.001f, sample, tick) : FRICTION_REQUEST_INVALID;
        }
        else if (use_v3 && (payload & EP_CALF_PULSE_TAG) == EP_CALF_PULSE_TAG)
            friction_dbg.request_result = state == FRICTION_THIGHS_HOLD ?
                StartCalfPulse(payload, sample, tick) : FRICTION_REQUEST_INVALID;
        else if (state == FRICTION_PREP_HOLD || state == FRICTION_THIGHS_HOLD)
            friction_dbg.request_result = TryStartRequest(word, use_v3, sample, tick);
        else if (use_v3 && (payload == EP_CALF_POSE_PAYLOAD || payload == EP_CALF_POSE_FEEDFORWARD_PAYLOAD))
            friction_dbg.request_result = StartCalfPose(legs, payload == EP_CALF_POSE_FEEDFORWARD_PAYLOAD, tick);
        else if (use_v3 && payload == EP_THIGHS_CAPTURE_PAYLOAD)
            friction_dbg.request_result = CaptureThighsHold(legs, tick);
        else if (payload == EP_PREP_LEFT_PAYLOAD)
            friction_dbg.request_result = StartPrep(FRICTION_LEFT_FRONT, 0.0f, 0, sample, tick);
        else if (payload == EP_PREP_RIGHT_PAYLOAD)
            friction_dbg.request_result = StartPrep(FRICTION_RIGHT_FRONT, 0.0f, 0, sample, tick);
        else if (use_v3 && (payload & EP_THIGH_TARGET_MASK) == EP_THIGH_TARGET_PAYLOAD)
        {
            const FrictionActuator_e actuator = (payload & EP_THIGH_RIGHT) ? FRICTION_RIGHT_FRONT : FRICTION_LEFT_FRONT;
            const int angle_code = (int)(payload & EP_THIGH_ANGLE_MASK) - ((payload & EP_THIGH_ANGLE_SIGN) ? 2048 : 0);
            friction_dbg.request_result = StartPrep(actuator, (float)angle_code * EP_THIGH_RAD_PER_CODE, 1, sample, tick);
        }
        else if (payload == EP_VIRTUAL_ZERO_PAYLOAD)
            friction_dbg.request_result = StartVirtualZero(legs, tick);
        else
            friction_dbg.request_result = TryStartRequest(word, use_v3, sample, tick);
    }
    if (state != FRICTION_ACTIVE)
        return;
    if (!sample->valid || (motors[selected].joint &&
        (prep_keep_hold || friction_dbg.thighs_hold.active) && !sample->joint_enabled))
    {
        sample->valid = 0;
        EndTrial(FRICTION_END_FEEDBACK, tick, sample);
        return;
    }

    RecordMotion(sample);
    if (pulse_active && CapturePulseSample(sample, tick))
    {
        EndTrial(FRICTION_END_TIMEOUT, tick, sample);
        return;
    }
    if (pulse_active && selected == FRICTION_RIGHT_WHEEL &&
        tick - trial.started_tick >= pdMS_TO_TICKS(FRICTION_WHEEL_PULSE_MS))
    {
        StopMotor(&motors[selected]);
        friction_dbg.command_nm = 0.0f;
        friction_dbg.wheel_request_raw = 0;
        return;
    }
    if (trial.peer_hold)
        ApplyPeerHold(motors[selected].peer, sample);
    ApplyTorque(&motors[selected], active_torque_nm);
    trial.peak_command_nm = fabsf(active_torque_nm);
}

void FrictionTestInit(const ChassisActuators_s *registered)
{
    motors[FRICTION_LEFT_FRONT] = (FrictionMotor_s){
        .joint = registered->l_front_8009_motor, .peer = registered->l_behind_8009_motor,
    };
    motors[FRICTION_LEFT_REAR] = (FrictionMotor_s){
        .joint = registered->l_behind_8009_motor, .peer = registered->l_front_8009_motor,
    };
    motors[FRICTION_LEFT_WHEEL] = (FrictionMotor_s){
        .wheel = registered->l_wheel_motor, .wheel_rad_per_deg = DEG_TO_RAD / L_REDUCTION_RATIO,
        .wheel_sign = l_wheel_direction,
    };
    motors[FRICTION_RIGHT_FRONT] = (FrictionMotor_s){
        .joint = registered->r_front_8009_motor, .peer = registered->r_behind_8009_motor,
    };
    motors[FRICTION_RIGHT_REAR] = (FrictionMotor_s){
        .joint = registered->r_behind_8009_motor, .peer = registered->r_front_8009_motor,
    };
    motors[FRICTION_RIGHT_WHEEL] = (FrictionMotor_s){
        .wheel = registered->r_wheel_motor, .wheel_rad_per_deg = DEG_TO_RAD / R_REDUCTION_RATIO,
        .wheel_sign = 1.0f,
    };

    for (unsigned i = 0; i < FRICTION_ACTUATOR_COUNT; ++i)
    {
        if (motors[i].joint)
            DMMotorSetCtrlMode(motors[i].joint, DM_CTRL_TORQUE);
        StopMotor(&motors[i]);
    }
    /* 台架应用包装后电机的现有解码回调，普通固件驱动和 CAN 协议共用。 */
    WatchRearFeedback(0, registered->l_behind_8009_motor);
    WatchRearFeedback(1, registered->r_behind_8009_motor);
    SetPowerLimit(1000000.0f);
}

static void ObserveLegAngles(const FrictionFeedback_s legs[2])
{
    const FrictionFeedback_s left = legs[0];
    const FrictionFeedback_s right = legs[1];
    friction_dbg.leg_angle_valid_mask = left.valid | (right.valid << 1);
    uint8_t theta_valid = 0;
    if (left.valid)
    {
        l_leg_Resolve leg;
        l_Alpha_beta_calculate(-left.position_rad, -left.peer_position_rad,
                               -left.speed_rad_s, -left.peer_speed_rad_s, &leg);
        friction_dbg.thigh_alpha_rad[0] = leg.l_alpha;
        friction_dbg.leg_beta_rad[0] = leg.l_beta;
        const float theta = l_leg_theta_calculate(&leg, 0, 1);
        const float length = l_leg_theta_calculate(&leg, 1, 0);
        if (isfinite(theta) && isfinite(length))
        {
            friction_dbg.leg_theta_body_rad[0] = theta;
            friction_dbg.leg_length_m[0] = length;
            theta_valid |= 1u;
        }
    }
    if (right.valid)
    {
        r_leg_Resolve leg;
        r_Alpha_beta_calculate(right.position_rad, right.peer_position_rad,
                               right.speed_rad_s, right.peer_speed_rad_s, &leg);
        friction_dbg.thigh_alpha_rad[1] = leg.r_alpha;
        friction_dbg.leg_beta_rad[1] = leg.r_beta;
        const float theta = r_leg_theta_calculate(&leg, 0, 1);
        const float length = r_leg_theta_calculate(&leg, 1, 0);
        if (isfinite(theta) && isfinite(length))
        {
            friction_dbg.leg_theta_body_rad[1] = theta;
            friction_dbg.leg_length_m[1] = length;
            theta_valid |= 2u;
        }
    }
    friction_dbg.leg_theta_valid_mask = theta_valid;
}

void FrictionTestUpdate(uint32_t tick)
{
    const FrictionFeedback_s legs[2] = {
        ReadFeedback(&motors[FRICTION_LEFT_FRONT]),
        ReadFeedback(&motors[FRICTION_RIGHT_FRONT]),
    };
    FrictionFeedback_s sample = {0};
    if (selected < FRICTION_ACTUATOR_COUNT)
        sample = ReadFeedback(&motors[selected]);

    ObserveLegAngles(legs);
    UpdateOutput(&sample, legs, tick);

    friction_dbg.tick = tick;
    friction_dbg.trial_id = trial.id;
    friction_dbg.state = state;
    friction_dbg.gate_reason = gate_reason;
    friction_dbg.actuator = selected;
    friction_dbg.feedback_valid = sample.valid;
    friction_dbg.has_peer = selected < FRICTION_ACTUATOR_COUNT && motors[selected].peer != NULL;
    friction_dbg.request_nm = active_torque_nm;
    friction_dbg.position_rad = sample.position_rad;
    friction_dbg.speed_rad_s = sample.speed_rad_s;
    friction_dbg.peer_position_rad = sample.peer_position_rad;
    friction_dbg.peer_hold_active = state == FRICTION_ACTIVE && trial.peer_hold;
    friction_dbg.peer_hold_target_rad = friction_dbg.peer_hold_active ? start_peer_position : 0.0f;
    friction_dbg.feedback_torque_nm = sample.torque_nm;
    friction_dbg.wheel_current_raw = sample.current_raw;
}
