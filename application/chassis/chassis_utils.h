#ifndef CHASSIS_UTILS_H
#define CHASSIS_UTILS_H

#include <stdint.h>
#include <math.h>
#include "general_def.h" /* PI */

/* ===== 常量宏 ===== */
/* Ldot 一阶 LPF 系数（1kHz 循环），截止约 25 Hz（原 chassis.h 定义, 随 LegVelEst 迁入） */
#define LEG_VEL_LPF (0.15f)

/* ===== 类型定义 ===== */
/* 腿长速度估计状态（P-P 级联的阻尼项用） */
typedef struct
{
    float last_L;      /* 上次变化的腿长读数 */
    float dt_acc;      /* 距上次变化的累计时间 s */
    float raw;         /* 本拍差分原始值 */
    float filt;        /* 低通后的 Ldot */
    uint8_t started;
} LegVelState_s;

/* ===== 内联工具 ===== */

/* 角度差 wrap 到 ±π（统一状态机连续角跟踪用, 2026-11-09） */
static inline float rec_wrap_pi(float a)
{
    return a - 2.0f * PI * roundf(a / (2.0f * PI));
}

/* ===== 对外接口 ===== */

/* Ldot 估计：DM 反馈 500Hz，1kHz 循环里差分隔拍为 0，
 * 必须按"上次变化以来的累计时间"归一，否则均值少一半。 */
float LegVelEst(float L, LegVelState_s *st, uint8_t hold_between_samples);

/* 追赶目标由反馈位置重写。按到终点的剩余角与现有超前量给参考速度，
 * 到位带内取消摆动前馈，PD 实测速度项继续制动。
 * rate 由调用方传入（当前各处统一传 rec_done_zero_rate）。 */
float ChaseReferenceSpeed(float remaining, float tolerance, float ahead, float rate);

#endif // CHASSIS_UTILS_H
