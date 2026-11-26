#include "chassis_step_leg.h"
#include "chassis_utils.h" /* rec_wrap_pi, ChaseReferenceSpeed */
#include "grf_observer.h"  /* GRF_AIR_LEFT/RIGHT */
#include "motor_def.h"     /* LIMIT_MIN_MAX */
#include <math.h>

/* ===== 恢复模块参数引用 =====
 * 台阶动作与自起共用摆角追赶/收腿参数；拆文件后经 extern 引用,
 * 定义当前在 chassis.c（后续归入 chassis_recovery.c），符号名不变。 */
extern volatile float rec_theta_end;
extern volatile float rec_l_low;
extern volatile float rec_retract_rate;
extern volatile float rec_chase_push;
extern volatile float rec_done_zero_rate;

/* ===== 状态量定义 ===== */
volatile StepLegMotion_s step_leg_motion[2];

/* ===== 对外接口 ===== */

void StepLegMotion_Reset(void)
{
    step_leg_motion[0] = (StepLegMotion_s){0};
    step_leg_motion[1] = (StepLegMotion_s){0};
}

uint8_t StepLegMotion_ActiveMask(void)
{
    return ((step_leg_motion[0].phase != STEP_LEG_IDLE) ? GRF_AIR_LEFT : 0) |
           ((step_leg_motion[1].phase != STEP_LEG_IDLE) ? GRF_AIR_RIGHT : 0);
}

LegFrictionPlan_s StepLegMotion_Update(volatile StepLegMotion_s *leg, uint8_t hit,
                                       float theta_body, float measured_length)
{
    LegFrictionPlan_s plan = {0};
    const float rear_angle = -0.5f * PI;
    if (leg->phase == STEP_LEG_IDLE)
    {
        if (!hit)
            return plan;
        leg->phase = STEP_LEG_BACKSWING;
    }
    /* 命中后先锁轮；等反馈有效才从实际腿长开始生成动作参考。 */
    if (!isfinite(theta_body) || !isfinite(measured_length) || measured_length <= 0.0f)
    {
        if (leg->phase == STEP_LEG_READY)
            leg->phase = STEP_LEG_RETURN;
        return plan;
    }
    if (leg->length_target <= 0.0f)
        leg->length_target = measured_length;

    const float previous_length = leg->length_target;
    const float rear_error = rec_wrap_pi(rear_angle - theta_body);
    const float low_error = rec_wrap_pi(rec_theta_end - theta_body);
    const uint8_t rear_reached = fabsf(rear_error) <= STEP_LEG_REAR_ALIGN_EPS;
    const uint8_t length_reached = fabsf(measured_length - rec_l_low) <= STEP_LEG_LENGTH_TOL;

    if (leg->phase == STEP_LEG_BACKSWING && rear_reached)
        leg->phase = STEP_LEG_TUCK;

    if (leg->phase == STEP_LEG_TUCK)
    {
        /* 锁住后摆角先收腿，长度目标到位且实际收短后才让轮胎向下回摆。 */
        const float length = leg->length_target;
        if (length > rec_l_low)
            leg->length_target = fmaxf(rec_l_low, length - rec_retract_rate);
        else
            leg->length_target = fminf(rec_l_low, length + rec_retract_rate);

        if (rear_reached && length_reached && leg->length_target == rec_l_low)
            leg->phase = STEP_LEG_RETURN;
    }

    /* 先到的一侧保持准备姿态，每拍复核，双侧交接只用同一拍的到位结果。 */
    if (leg->phase == STEP_LEG_RETURN || leg->phase == STEP_LEG_READY)
    {
        const uint8_t pose_ready =
            measured_length >= STEP_LEG_READY_LEN_MIN &&
            measured_length <= rec_l_low + STEP_LEG_LENGTH_TOL &&
            fabsf(low_error) <= STEP_LEG_READY_ANGLE_TOL;

        leg->phase = pose_ready ? STEP_LEG_READY : STEP_LEG_RETURN;
    }

    /* 复用自起的摆角超前量，限定大角度摆动的误差。回摆段腿长保持收短。 */
    float error = leg->phase <= STEP_LEG_TUCK ? rear_error : low_error;
    LIMIT_MIN_MAX(error, -rec_chase_push, rec_chase_push);
    leg->theta_body_target = theta_body + error;
    plan.length_step = leg->length_target - previous_length;
    if (leg->phase == STEP_LEG_BACKSWING)
        plan.theta_speed = ChaseReferenceSpeed(rear_error, STEP_LEG_REAR_ALIGN_EPS, rec_chase_push, rec_done_zero_rate);
    else if (leg->phase == STEP_LEG_RETURN)
        plan.theta_speed = ChaseReferenceSpeed(low_error, STEP_LEG_READY_ANGLE_TOL, rec_chase_push, rec_done_zero_rate);
    return plan;
}
