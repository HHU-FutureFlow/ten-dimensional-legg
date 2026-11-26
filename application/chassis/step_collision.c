#include "step_collision.h"
#include "general_def.h"
#include <math.h>
#include <string.h>

/* 2026-11-18 平地 EP 复核已排除倒转加快样本。短窗和幅度仍是试跑初值，
 * 需要逐侧撞阶和正常行驶数据确认（UNTESTED）。 */
#define STEP_WINDOW_TICKS   10u
#define STEP_CONFIRM_TICKS   3u
#define STEP_SPEED_DROP_MIN  0.05f
#define STEP_REAR_ANGLE_MIN  0.01f

typedef struct
{
    StepCollisionSample_s history[STEP_WINDOW_TICKS];
    uint8_t head;
    uint8_t filled;
    uint8_t confirm_count;
} StepCollisionSide_s;

static StepCollisionSide_s sides[2];
volatile StepCollisionDbg_s step_collision_dbg;

/* 和 GRF 一样检查 binary32 指数位，避免 -Ofast 消去 isfinite。 */
static uint8_t finite_feedback(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static void reset_side(uint8_t side)
{
    sides[side].head = 0;
    sides[side].filled = 0;
    sides[side].confirm_count = 0;
    step_collision_dbg.speed_drop[side] = 0.0f;
    step_collision_dbg.rearward_angle[side] = 0.0f;
    step_collision_dbg.confirm_count[side] = 0;
}

void StepCollision_Reset(void)
{
    reset_side(0);
    reset_side(1);
    step_collision_dbg.enabled = 0;
    step_collision_dbg.history_ready_mask = 0;
    step_collision_dbg.candidate_mask = 0;
    step_collision_dbg.hit_mask = 0;
    step_collision_dbg.reversal_mask = 0;
}

void StepCollision_Update(uint8_t enabled, uint8_t forward_drive,
                          uint8_t air_mask, const StepCollisionSample_s sample[2])
{
    ++step_collision_dbg.tick;

    if (!enabled)
    {
        if (step_collision_dbg.enabled)
            StepCollision_Reset();
        return;
    }

    step_collision_dbg.enabled = 1;
    step_collision_dbg.history_ready_mask = 0;
    step_collision_dbg.candidate_mask = 0;
    step_collision_dbg.reversal_mask = 0;

    for (uint8_t i = 0; i < 2; ++i)
    {
        const uint8_t bit = (uint8_t)(1u << i);
        StepCollisionSide_s *side = &sides[i];
        const StepCollisionSample_s now = sample[i];

        if ((air_mask & bit) || !finite_feedback(now.wheel_speed) ||
            !finite_feedback(now.theta))
        {
            reset_side(i);
            continue;
        }

        const StepCollisionSample_s before = side->history[side->head];
        side->history[side->head] = now;
        side->head = (side->head + 1u) % STEP_WINDOW_TICKS;

        if (side->filled < STEP_WINDOW_TICKS)
        {
            ++side->filled;
            continue;
        }

        step_collision_dbg.history_ready_mask |= bit;

        const float speed_drop = fabsf(before.wheel_speed) -
                                 fabsf(now.wheel_speed);
        float rearward = before.theta - now.theta;
        rearward -= 2.0f * PI * roundf(rearward / (2.0f * PI));

        step_collision_dbg.speed_drop[i] = speed_drop;
        step_collision_dbg.rearward_angle[i] = rearward;

        if ((before.wheel_speed > 0.0f && now.wheel_speed < 0.0f) ||
            (before.wheel_speed < 0.0f && now.wheel_speed > 0.0f))
        {
            step_collision_dbg.reversal_mask |= bit;
        }

        const uint8_t candidate =
            forward_drive &&
            before.wheel_speed > 0.0f && now.wheel_speed >= 0.0f &&
            speed_drop > STEP_SPEED_DROP_MIN && rearward > STEP_REAR_ANGLE_MIN;

        if (candidate)
        {
            step_collision_dbg.candidate_mask |= bit;
            if (side->confirm_count < STEP_CONFIRM_TICKS)
                ++side->confirm_count;

            if (side->confirm_count == STEP_CONFIRM_TICKS &&
                !(step_collision_dbg.hit_mask & bit))
            {
                step_collision_dbg.last_hit_tick[i] = step_collision_dbg.tick;
                step_collision_dbg.hit_mask |= bit;
            }
        }
        else
        {
            side->confirm_count = 0;
        }

        step_collision_dbg.confirm_count[i] = side->confirm_count;
    }
}
