#ifndef STEP_COLLISION_H
#define STEP_COLLISION_H

#include <stdint.h>

typedef struct
{
    float wheel_speed; /* m/s，前进为正 */
    float theta;       /* rad，世界系腿摆角 */
} StepCollisionSample_s;

typedef struct
{
    uint32_t tick;          /* 检测接口调用拍号 */
    uint32_t last_hit_tick[2];
    float speed_drop[2];    /* 轮速幅值降幅，m/s */
    float rearward_angle[2]; /* 后摆角增量，rad */
    uint8_t enabled;
    uint8_t history_ready_mask;
    uint8_t candidate_mask;
    uint8_t hit_mask;       /* bit0 左侧，bit1 右侧 */
    uint8_t reversal_mask;  /* 回看窗口两端轮速反号 */
    uint8_t confirm_count[2];
} StepCollisionDbg_s;

extern volatile StepCollisionDbg_s step_collision_dbg;

/* 底盘 1 kHz 调用。窗口内填历史，正向行驶才累计碰撞确认。
 * air_mask 位序为左、右，只清离地侧的历史，命中保持到关窗。 */
void StepCollision_Update(uint8_t enabled, uint8_t forward_drive,
                          uint8_t air_mask, const StepCollisionSample_s sample[2]);

/* 关闭当前检测窗口，保留最近命中拍号供 EP 查看。 */
void StepCollision_Reset(void);

#endif
