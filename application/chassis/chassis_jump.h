#ifndef CHASSIS_JUMP_H
#define CHASSIS_JUMP_H

#include "robot_def.h"
#include "balance.h"

#define JUMP_LEG_KV_VEL 100.0f /* 和 5 ms 腿速保持配套整定，N/(m/s)。 */

typedef struct
{
    float length[2];
    float velocity[2];
    float stand_length;
    float return_length;
    uint8_t airborne;
    uint8_t can_start;
} ChassisJumpInput_s;

typedef struct
{
    float length_target;
    float length_step; /* 1 ms 内主动规划的腿长增量，m */
    float wheel_theta_scale;
    float hip_pitch_scale;
    uint8_t active;
    uint8_t level_legs;
    uint8_t restart_lift;
} ChassisJumpReference_s;

void ChassisJump_Reset(void);
void ChassisJump_Update(chassis_jump_cmd_e command, const ChassisJumpInput_s *input,
                        ChassisJumpReference_s *ref);
void ChassisJump_Reanchor(const ten_balance_observation *observe, ten_balance_target *target);
void ChassisJump_LegForce(const ChassisJumpInput_s *input, float length_target,
                          const float normal_limits[2], float force[2]);

#endif
