#include "grf_observer.h"
#include "chassis.h"
#include "balance.h"
#include <math.h>
#include <string.h>

/* 腿杆参数与 VMC.c 同源（balance.h）,g(β) 为 VMC.c 同一公式的副本:
 * 改腿杆尺寸时两处要一起改。 */
#define GRF_LEG_A  (0.21f)
#define GRF_LEG_B  (0.25f)
#define GRF_G      (9.81f)

volatile GRF_Dbg_s grf_dbg;
volatile float grf_m_w      = 0.5f;   /* 单侧轮+轮轴组件 kg,参数表 m_u,ep 可调 */
volatile float grf_accel_fc = 35.0f;  /* z̈ 低通截止 Hz,悬空判定要快,压噪靠腿力通道 */
volatile float grf_fleg_fc  = 60.0f;  /* 腿力低通截止 Hz,力矩噪声 ~0.5N,可放宽 */

/* 2026-11-18 EP 单侧提车时，卸载侧约 -21～10 N、支撑侧反力明显为正。
 * 均值判据曾漏掉左侧卸载，也曾在右侧卸载时一起切掉左轮。
 * 门限改为各侧 20/35 N（原均值 30/40 N），100/25 ms 暂沿用。
 * 短时离地和双侧提车的响应需实车复核（UNTESTED）。 */
volatile float grf_air_th_off = 20.0f;
volatile float grf_air_th_on = 35.0f;
volatile uint16_t grf_air_hold_off = 100;
volatile uint16_t grf_air_hold_on = 25;
volatile uint16_t grf_air_delay = 3000; /* 维持自起接管时的静默时间，ms @1kHz */

static uint8_t grf_air_mask;
static uint16_t grf_air_cnt_l, grf_air_cnt_r;
static uint16_t grf_tick;

uint8_t GRF_GetAirMask(void) { return grf_air_mask; }
uint8_t GRF_Airborne(void) { return grf_air_mask == GRF_AIR_BOTH; }

void GRF_ResetAirDwell(void)
{
    grf_air_cnt_l = grf_air_cnt_r = 0;
    grf_dbg.air_count_l = grf_dbg.air_count_r = 0;
}

void GRF_ClearAirState(void)
{
    grf_air_mask = 0;
    grf_dbg.air_state = grf_dbg.air_mask = 0;
    GRF_ResetAirDwell();
}

void GRF_RestartAirDelay(void)
{
    /* 自起暂时趴地、低位的反力与站立标定分布不同。 */
    grf_tick = 0;
    GRF_ClearAirState();
}

static void grf_update_side(uint8_t side, float normal_force, uint16_t *count)
{
    const uint8_t airborne = grf_air_mask & side;
    /* 无效反馈或反力回到迟滞区时，重新计算连续驻留。 */
    if (!(grf_dbg.valid_mask & side) ||
        !(airborne ? normal_force > grf_air_th_on : normal_force < grf_air_th_off))
    {
        *count = 0;
        return;
    }

    const uint16_t hold = airborne ? grf_air_hold_on : grf_air_hold_off;
    if (++(*count) >= hold)
    {
        grf_air_mask ^= side;
        *count = 0;
    }
}

static void grf_air_update(void)
{
    if (grf_tick < grf_air_delay)
    {
        ++grf_tick;
        return;
    }

    grf_update_side(GRF_AIR_LEFT, grf_dbg.F_n_l, &grf_air_cnt_l);
    grf_update_side(GRF_AIR_RIGHT, grf_dbg.F_n_r, &grf_air_cnt_r);
    grf_dbg.air_mask = grf_air_mask;
    grf_dbg.air_state = GRF_Airborne();
    grf_dbg.air_count_l = grf_air_cnt_l;
    grf_dbg.air_count_r = grf_air_cnt_r;
}

/* STM32 的 float 为 IEEE 754 binary32。发布构建的 -Ofast 可消去 isfinite，
 * 此处查指数位，避免无效反馈进入低通后破坏离地判据。 */
static uint8_t grf_finite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

typedef struct { float y; uint8_t started; } GRF_Lpf_s;
static GRF_Lpf_s lpf_fleg_l, lpf_fleg_r, lpf_zdd, lpf_fn_l, lpf_fn_r;

static float grf_lpf_step(GRF_Lpf_s *st, float x, float fc, float dt)
{
    if (!st->started) { st->started = 1; st->y = x; return st->y; }
    float alpha = dt / (dt + 1.0f / (6.2831853f * fc));
    st->y += alpha * (x - st->y);
    return st->y;
}

/* VMC.c 的 g(β)（dl/dβ）,腿轴向力反推用 */
static float grf_leg_g(float beta)
{
    float s = sinf(0.5f * beta), c = cosf(0.5f * beta);
    float as = GRF_LEG_A * s;
    return -0.5f * as
           - (as * as * c) / (2.0f * sqrtf(GRF_LEG_B * GRF_LEG_B - as * as));
}

void GRF_Init(void)
{
    grf_dbg = (GRF_Dbg_s){0};
    lpf_fleg_l = lpf_fleg_r = lpf_zdd = lpf_fn_l = lpf_fn_r = (GRF_Lpf_s){0, 0};
    GRF_RestartAirDelay();
}

void GRF_Update(float r_Tf, float r_Tb, float l_Tf, float l_Tb, float dt)
{
    /* ---- 腿轴向力: VMC 逆关系 F = ±(Tb−Tf)/(2g(β)) ----
     * 右腿: Tf=-gF+Tp/2, Tb=+gF+Tp/2  → F=(Tb−Tf)/(2g)
     * 左腿: 电机坐标整体取负(l_VMC_Calc) → F=(Tf−Tb)/(2g)
     * 2026-11-02 静态账验证: 反推值与 Fleg_cmd 吻合 <0.2%。 */
    float g_r = grf_leg_g(r_leg.r_beta);
    float g_l = grf_leg_g(l_leg.l_beta);
    float F_r = 0.0f, F_l = 0.0f;
    const uint8_t usable_r = grf_finite(g_r) && fabsf(g_r) > 0.01f;
    const uint8_t usable_l = grf_finite(g_l) && fabsf(g_l) > 0.01f;
    if (usable_r) F_r = (r_Tb - r_Tf) / (2.0f * g_r);
    if (usable_l) F_l = (l_Tf - l_Tb) / (2.0f * g_l);

    /* ---- 竖直分量: θ 为摆杆相对竖直倾角（BalanceObserve 已扣 pitch） ---- */
    float Fz_r = F_r * cosf(ten_balance_observe.r_theta);
    float Fz_l = F_l * cosf(ten_balance_observe.l_theta);

    /* ---- 轮竖直加速度: IMU 比力旋转到竖直方向减 g ----
     * 机体系比力 f_b,竖直分量 f_wz = −sin(p)·ax + cos(p)sin(r)·ay + cos(p)cos(r)·az;
     * roll 修正项为小量(|r|<2°),符号按欧拉 ZYX 惯例,误差可忽略。 */
    float p = chassis_IMU_data->Pitch * 0.01745329252f;
    float r = chassis_IMU_data->Roll   * 0.01745329252f;
    float az_b = chassis_IMU_data->Accel[2];
    float ay_b = chassis_IMU_data->Accel[1];
    float ax_b = chassis_IMU_data->Accel[0];
    float zdd = -sinf(p) * ax_b + cosf(p) * sinf(r) * ay_b
                + cosf(p) * cosf(r) * az_b - GRF_G;

    /* ---- 观测器本体 N = m_w*(z̈+g) + F_leg,z ---- */
    float m = grf_m_w;
    float N_r = m * (zdd + GRF_G) + Fz_r;
    float N_l = m * (zdd + GRF_G) + Fz_l;

    grf_dbg.F_n_l_raw = N_l;
    grf_dbg.F_n_r_raw = N_r;
    uint8_t valid_mask = 0;
    if (usable_l && grf_finite(N_l))
    {
        grf_dbg.F_leg_l = grf_lpf_step(&lpf_fleg_l, F_l, grf_fleg_fc, dt);
        grf_dbg.Flegz_l = grf_dbg.F_leg_l * cosf(ten_balance_observe.l_theta);
        grf_dbg.F_n_l = grf_lpf_step(&lpf_fn_l, N_l, grf_fleg_fc, dt);
        valid_mask |= GRF_AIR_LEFT;
    }
    if (usable_r && grf_finite(N_r))
    {
        grf_dbg.F_leg_r = grf_lpf_step(&lpf_fleg_r, F_r, grf_fleg_fc, dt);
        grf_dbg.Flegz_r = grf_dbg.F_leg_r * cosf(ten_balance_observe.r_theta);
        grf_dbg.F_n_r = grf_lpf_step(&lpf_fn_r, N_r, grf_fleg_fc, dt);
        valid_mask |= GRF_AIR_RIGHT;
    }
    if (grf_finite(zdd))
        grf_dbg.zdd_w = grf_lpf_step(&lpf_zdd, zdd, grf_accel_fc, dt);

    grf_dbg.valid_mask = valid_mask;
    grf_dbg.F_n_avg = 0.5f * (grf_dbg.F_n_l + grf_dbg.F_n_r);
    grf_air_update();
}
