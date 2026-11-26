#include "chassis_utils.h"
#include <math.h>

/* ===== 对外接口 ===== */

float LegVelEst(float L, LegVelState_s *st, uint8_t hold_between_samples)
{
    if (!st->started)
    {
        st->last_L = L;
        st->started = 1;
        return 0.0f;
    }
    st->dt_acc += 0.001f;
    if (L != st->last_L)
    {
        st->raw = (L - st->last_L) / st->dt_acc;
        st->last_L = L;
        st->dt_acc = 0.0f;
    }
    else if (!hold_between_samples || st->dt_acc > 0.005f)
    {
        /* 跳跃的阻尼与 5 ms 保持配套，正常路径按开关选择估计方式。 */
        st->raw = 0.0f;
    }
    st->filt += LEG_VEL_LPF * (st->raw - st->filt);
    return st->filt;
}

float ChaseReferenceSpeed(float remaining, float tolerance, float ahead, float rate)
{
    if (!isfinite(remaining) || !isfinite(ahead) || ahead <= 0.0f)
        return 0.0f;

    const float travel = fmaxf(0.0f, fabsf(remaining) - tolerance);
    return copysignf(rate * fminf(1.0f, travel / ahead), remaining);
}
