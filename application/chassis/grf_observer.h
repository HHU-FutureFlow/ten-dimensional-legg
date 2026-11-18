#ifndef _GRF_OBSERVER_H
#define _GRF_OBSERVER_H

#include <stdint.h>

enum
{
    GRF_AIR_LEFT  = 1u << 0,
    GRF_AIR_RIGHT = 1u << 1,
    GRF_AIR_BOTH  = GRF_AIR_LEFT | GRF_AIR_RIGHT
};

/* ===== GRF 地面反作用力观测器（手册第九章 T1, 2026-11-02）=====
 * 轮端自由体（本工程符号约定: F>0=伸腿, 腿把轮向下压）:
 *   N = m_w*(z̈_w + g) + F_leg,z
 *   接地静态:  N ≈ 机体分摊重 + 轮重（实测整车 11.5kg → 单轮约 56N）
 *   悬空自由落体: z̈→-g, F_leg→0 → N→0
 * F_leg 由 8009 反馈力矩经 VMC 逆关系反推（2026-11-02 静态账验证,
 * 与 Fleg_cmd 吻合 0.2% 以内, 左腿电机坐标镜像已处理）;
 * z̈_w 由 IMU 比力旋转到竖直方向后减 g。
 * 注意: 用机体竖直加速度近似轮心竖直加速度, 腿长快速变化时有误差;
 * 触地/离地门控场景腿长近似不变或自由落体, 该近似可用。
 * 全部输出经一阶 LPF, grf_dbg 供 ep 读取。 */

typedef struct
{
    float F_leg_l;     /* 左腿轴向力（反馈力矩反推, N, LPF 后） */
    float F_leg_r;     /* 右腿轴向力 */
    float Flegz_l;     /* 左腿竖直分量 F*cos(θ)（N, LPF 后） */
    float Flegz_r;
    float zdd_w;       /* 竖直加速度估计（m/s², LPF 后, 竖直向上为正） */
    float F_n_l;       /* 左轮地面反作用力（N, LPF 后） */
    float F_n_r;       /* 右轮地面反作用力 */
    float F_n_l_raw;   /* 未滤波原值（看噪声/调 LPF 用） */
    float F_n_r_raw;
    uint8_t air_state; /* 两侧均离地时为 1，兼容原全局状态观测 */
    uint8_t air_mask;  /* bit0 左轮离地，bit1 右轮离地 */
    uint8_t valid_mask; /* 本拍反力估计有效的侧，位序同 air_mask */
    uint16_t air_count_l; /* 左侧当前转换的连续拍数 */
    uint16_t air_count_r;
    float F_n_avg;     /* 左右滤波估计均值 N，供 EP 对照 */
} GRF_Dbg_s;

extern volatile GRF_Dbg_s grf_dbg;

extern volatile float grf_m_w;        /* 单侧轮+轮轴组件质量 kg（参数表 m_u） */
extern volatile float grf_accel_fc;   /* z̈ LPF 截止频率 Hz */
extern volatile float grf_fleg_fc;    /* 腿力 LPF 截止频率 Hz */

/* ===== 左右独立判定，共用标定参数和自起静默时间 ===== */
extern volatile float grf_air_th_off;      /* 本侧离地门限 N，低于此才累计 */
extern volatile float grf_air_th_on;       /* 本侧承载门限 N，高于此才累计 */
extern volatile uint16_t grf_air_hold_off; /* 离地判定驻留 tick @1kHz */
extern volatile uint16_t grf_air_hold_on;  /* 落地判定驻留 tick @1kHz */
extern volatile uint16_t grf_air_delay;    /* 死延时 ms：上电后检测静默期,期间 air 恒 0 */
/* 0 为双轮支撑，两侧位可独立置位或清除。 */
uint8_t GRF_GetAirMask(void);
uint8_t GRF_Airborne(void);                /* 仅双轮同时离地时为 1 */
void GRF_ResetAirDwell(void);              /* 清当前驻留计数，保留已确认状态 */
void GRF_RestartAirDelay(void);            /* 自起 S4 重启静默并清已确认离地状态 */

void GRF_Init(void);
/* @param r_Tf/r_Tb 右腿前后关节反馈力矩（电机坐标, N·m）
 * @param l_Tf/l_Tb 左腿前后关节反馈力矩（电机坐标, N·m）
 * @param dt        调用周期 s */
void GRF_Update(float r_Tf, float r_Tb, float l_Tf, float l_Tb, float dt);

#endif
