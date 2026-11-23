#include "chassis_friction.h"
#include "motor_def.h" /* LIMIT_MIN_MAX */
#include <math.h>

/* ===== 可调参数定义 ===== */
/* 2026-11-22 台架等效运动摩擦候选，N·m。带载运行待验证。
 * 关节顺序与 chassis_dbg 一致（右大腿、右小腿、左大腿、左小腿），轮为左、右。 */
const float joint_friction_nm[4] = {1.42f, 0.70f, 0.89f, 0.94f};
const float wheel_friction_nm[2] = {0.04f, 0.10f};

/* ===== 对外接口 ===== */

void JointReferenceSpeed(LegFrictionPlan_s plan, float length, float thigh, float shin,
                         float motor_sign, float speed[2])
{
    float half_beta_speed = 0.0f;
    if (plan.length_step != 0.0f)
    {
        const float before = length - plan.length_step;
        if (!isfinite(length) || !isfinite(before) || before <= 0.0f || length <= 0.0f)
            return;

        float cos_before = (before * before + thigh * thigh - shin * shin) / (2.0f * before * thigh);
        float cos_now = (length * length + thigh * thigh - shin * shin) / (2.0f * length * thigh);
        if (fabsf(cos_before) > 1.0f || fabsf(cos_now) > 1.0f)
            return;

        half_beta_speed = (acosf(cos_now) - acosf(cos_before)) / CHASSIS_TICK_S;
    }
    if (!isfinite(plan.theta_speed))
        return;

    speed[0] = motor_sign * (plan.theta_speed - half_beta_speed);
    speed[1] = motor_sign * (plan.theta_speed + half_beta_speed);
}

float MotorTorqueWithFriction(float base, float speed, float friction, float smoothing,
                              float limit, uint8_t blocked, volatile float *base_dbg,
                              volatile float *ff_dbg)
{
    if (blocked || !isfinite(base))
    {
        *base_dbg = *ff_dbg = 0.0f;
        return 0.0f;
    }

    *base_dbg = base;
    float limited_base = base;
    LIMIT_MIN_MAX(limited_base, -limit, limit);
    if (!isfinite(speed) || speed == 0.0f)
    {
        *ff_dbg = 0.0f;
        return limited_base;
    }

    float command = base + friction * tanhf(speed / smoothing);
    LIMIT_MIN_MAX(command, -limit, limit);
    *ff_dbg = command - limited_base;
    return command;
}
