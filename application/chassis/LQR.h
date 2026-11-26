#ifndef _LQR_H
#define _LQR_H

#include <stdint.h>
#include "balance.h"

typedef struct
{
    float position_deadband;
    float wheel_theta_scale;
    float hip_pitch_scale;
} LQR_FeedbackOptions_s;

float LQR_PositionError(float error, float deadband);

/* 两个掩码位序为左、右。受阻侧切轮，只有 step_leg_mask 侧读动作参考，
 * 支撑侧仍使用普通平衡参考，普通空侧保持世界系竖直。 */
void LQR_calc(const ten_balance_observation *observe, const ten_balance_target *balance_targets,
              const ten_balance_target *step_targets, uint8_t wheel_block_mask,
              uint8_t step_leg_mask, const LQR_FeedbackOptions_s *options,
              four_control_data *control_datas);

#endif
