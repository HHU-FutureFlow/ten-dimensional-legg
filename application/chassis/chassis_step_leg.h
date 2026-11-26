#ifndef CHASSIS_STEP_LEG_H
#define CHASSIS_STEP_LEG_H

#include <stdint.h>
#include "chassis_friction.h" /* LegFrictionPlan_s */

/* ===== 常量宏 ===== */
#define STEP_LEG_LENGTH_TOL 0.01f /* m，先确认实测收短再回摆（UNTESTED） */
#define STEP_LEG_REAR_ALIGN_EPS 0.19f /* rad，2026-11-20 右侧停在距正后方 0.183 rad，原 0.10；新值 UNTESTED */
#define STEP_LEG_READY_ANGLE_TOL 0.12f /* rad，2026-11-20 右回摆误差 0.102，原 0.10；新值 UNTESTED */
#define STEP_LEG_READY_LEN_MIN 0.10f /* m，2026-11-20 回摆末端最低约 0.107；新值 UNTESTED */

/* ===== 类型定义 ===== */
typedef enum
{
    STEP_LEG_IDLE,
    STEP_LEG_BACKSWING,
    STEP_LEG_TUCK,
    STEP_LEG_RETURN,
    STEP_LEG_READY
} StepLegPhase_e;

typedef struct
{
    float length_target;     /* m */
    float theta_body_target; /* rad，机体系 */
    uint8_t phase;
} StepLegMotion_s;

/* ===== 状态量（ep 可读） ===== */
/* EP 顺序 [0] 左、[1] 右。READY 保持短腿和准备摆角，等待双侧同拍接管。 */
extern volatile StepLegMotion_s step_leg_motion[2];

/* ===== 对外接口 ===== */
void StepLegMotion_Reset(void);

/* 当前处于非空闲动作期的腿（GRF_AIR_LEFT/RIGHT 位或） */
uint8_t StepLegMotion_ActiveMask(void);

/* 台阶单腿动作状态机：hit 触发后 后摆->收腿->回摆->就绪。
 * 返回本拍腿长增量与参考摆速计划（供摩擦前馈与腿长环使用）。 */
LegFrictionPlan_s StepLegMotion_Update(volatile StepLegMotion_s *leg, uint8_t hit,
                                       float theta_body, float measured_length);

#endif // CHASSIS_STEP_LEG_H
