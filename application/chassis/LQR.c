#include "LQR.h"
#include "grf_observer.h"
#include "general_def.h"
#include "motor_def.h"
#include <math.h>

/* ===== 分腿长 K 矩阵组（2026-11-22 定版：sigma_th 0.12 + 原始 s/ds）=====
 * 由 lqr_v2/run_all.py 在 6 个腿长工作点生成（动力学互证通过），
 * 运行时按实测腿长分带硬切（仿南航金城分模式增益组思路）。
 * 本版参数: sigma_th 0.30 -> 0.12, sigma_s 0.28 / sigma_ds 0.80 保持原值。
 * 实测站立腿摆角手掰不回正（θ 恢复力矩 3.1 N·m 推不过承重髋静摩擦死区），
 * θ 惩罚 ×5.4 后剪刀差收敛到肉眼难辨，前后移存在但幅度可接受、无明显抖动。
 *
 * 同日演进记录（实测后废弃，留档防重蹈）:
 * - v2 (sigma_s 0.16/0.45): 治前后移加大 s/ds 惩罚。实测修正机体偏移时
 *   抖动太厉害——强位置环在轮端摩擦死区上打粘滑极限环（攒力-突破-过冲）。
 * - v3 (sigma_s 0.22/0.35): v2 与本版的折中。EP 实测 s 仍以 ~8mm/s 漂移
 *   （6s 内 -0.244 -> -0.307），ds 波动 ±0.1 m/s，l_Tw 在 ±0.9 N·m 频繁换向。
 * - 根因（EP 2026-11-22 晚实读）: 两腿 θ 静差 -3°/-6.3° 被承重髋静摩擦
 *   锁死（Tp 恒定 -1.7~-3.1 N·m 推不动），Tw 行 θ 列把静差转成 -0.80 N·m
 *   轮力矩常值，吃掉 Tw 的 s 项（0.287m×2.26=+0.65 N·m）。死区是饱和
 *   非线性，K 增益路线已到头（增益 ×2.4 仅使锁死误差 14° -> 3.3°，力矩
 *   始终 2~3 N·m）。后续方向: θ 通道抗饱和积分破死区（待评审），或
 *   FRICTION_TANH_W0 0.15 -> 0.30 治抖（不治漂）。
 * 列序: s | ds | yaw | dyaw | lθ | ldθ | rθ | rdθ | φ | dφ
 * 行序: [0]=l_Tw [1]=r_Tw [2]=l_Tp [3]=r_Tp ; 控制律 T = -K(x-x_ref), rate = 1.0
 * 带宽 5cm, 中心即下标对应腿长; 切换带 1cm 迟滞防抖。 */
float K_l015[4][10] =
{
    {   -1.770247f,   -2.289659f,   -0.982345f,   -0.432335f,   +6.286473f,   +0.865263f,   +3.057573f,   +0.571487f,   +2.152596f,   +0.373776f },
    {   -1.770247f,   -2.289659f,   +0.982345f,   +0.432335f,   +3.057573f,   +0.571487f,   +6.286473f,   +0.865263f,   +2.152596f,   +0.373776f },
    {   -4.431988f,   -5.176010f,   +2.765977f,   +1.163157f,  +24.032628f,   +2.146900f,   -6.760329f,   +0.127799f,  -20.170419f,   -1.004202f },
    {   -4.431988f,   -5.176010f,   -2.765977f,   -1.163157f,   -6.760329f,   +0.127799f,  +24.032628f,   +2.146900f,  -20.170419f,   -1.004202f },
};

float K_l020[4][10] =
{
    {   -1.807534f,   -2.274538f,   -0.929929f,   -0.415729f,   +6.710076f,   +0.923138f,   +2.978887f,   +0.556586f,   +1.971606f,   +0.336006f },
    {   -1.807534f,   -2.274538f,   +0.929929f,   +0.415729f,   +2.978887f,   +0.556586f,   +6.710076f,   +0.923138f,   +1.971606f,   +0.336006f },
    {   -4.061508f,   -4.627250f,   +3.186889f,   +1.344404f,  +22.915269f,   +2.043637f,   -6.885131f,   +0.086183f,  -20.605545f,   -1.091127f },
    {   -4.061508f,   -4.627250f,   -3.186889f,   -1.344404f,   -6.885131f,   +0.086183f,  +22.915269f,   +2.043637f,  -20.605545f,   -1.091127f },
};

float K_l025[4][10] =
{
    {   -1.840169f,   -2.266951f,   -0.874086f,   -0.397008f,   +7.096275f,   +0.983856f,   +2.916642f,   +0.548612f,   +1.795397f,   +0.302012f },
    {   -1.840169f,   -2.266951f,   +0.874086f,   +0.397008f,   +2.916642f,   +0.548612f,   +7.096275f,   +0.983856f,   +1.795397f,   +0.302012f },
    {   -3.700046f,   -4.135066f,   +3.560076f,   +1.511289f,  +21.772703f,   +1.938653f,   -6.980675f,   +0.060110f,  -20.985109f,   -1.162482f },
    {   -3.700046f,   -4.135066f,   -3.560076f,   -1.511289f,   -6.980675f,   +0.060110f,  +21.772703f,   +1.938653f,  -20.985109f,   -1.162482f },
};

float K_l030[4][10] =
{
    {   -1.867811f,   -2.266134f,   -0.817249f,   -0.377104f,   +7.443020f,   +1.048204f,   +2.871858f,   +0.548288f,   +1.629578f,   +0.272513f },
    {   -1.867811f,   -2.266134f,   +0.817249f,   +0.377104f,   +2.871858f,   +0.548288f,   +7.443020f,   +1.048204f,   +1.629578f,   +0.272513f },
    {   -3.359378f,   -3.701292f,   +3.882798f,   +1.659932f,  +20.649700f,   +1.833112f,   -7.047097f,   +0.050009f,  -21.304674f,   -1.219316f },
    {   -3.359378f,   -3.701292f,   -3.882798f,   -1.659932f,   -7.047097f,   +0.050009f,  +20.649700f,   +1.833112f,  -21.304674f,   -1.219316f },
};

float K_l035[4][10] =
{
    {   -1.890670f,   -2.269704f,   -0.761075f,   -0.356646f,   +7.752989f,   +1.115251f,   +2.843503f,   +0.554704f,   +1.477477f,   +0.247100f },
    {   -1.890670f,   -2.269704f,   +0.761075f,   +0.356646f,   +2.843503f,   +0.554704f,   +7.752989f,   +1.115251f,   +1.477477f,   +0.247100f },
    {   -3.046543f,   -3.321985f,   +4.158206f,   +1.789076f,  +19.580502f,   +1.726348f,   -7.087344f,   +0.054656f,  -21.567089f,   -1.264193f },
    {   -3.046543f,   -3.321985f,   -4.158206f,   -1.789076f,   -7.087344f,   +0.054656f,  +19.580502f,   +1.726348f,  -21.567089f,   -1.264193f },
};

float K_l040[4][10] =
{
    {   -1.909305f,   -2.275952f,   -0.706615f,   -0.336084f,   +8.030625f,   +1.184029f,   +2.829842f,   +0.566852f,   +1.340325f,   +0.225194f },
    {   -1.909305f,   -2.275952f,   +0.706615f,   +0.336084f,   +2.829842f,   +0.566852f,   +8.030625f,   +1.184029f,   +1.340325f,   +0.225194f },
    {   -2.764212f,   -2.991667f,   +4.391573f,   +1.898521f,  +18.585406f,   +1.617569f,   -7.106478f,   +0.072861f,  -21.779387f,   -1.299606f },
    {   -2.764212f,   -2.991667f,   -4.391573f,   -1.898521f,   -7.106478f,   +0.072861f,  +18.585406f,   +1.617569f,  -21.779387f,   -1.299606f },
};

/* ===== 分带选择: 实测腿长 -> K 组, 1cm 迟滞防抖 =====
 * 带 k 中心 = 0.15 + 0.05k, 相邻带边界 = 0.175 + 0.05k。
 * 上电默认 0.30 带; 恢复流程从低腿长(折叠≈0.11)起步时, 首几次调用会逐带降到 l015。 */
static float (*const K_sets[6])[10] = { K_l015, K_l020, K_l025, K_l030, K_l035, K_l040 };
static uint8_t lqr_k_band = 3;      /* ep 可读: 当前 K 组下标 0..5 */

float rate = 1.0f;

/* 悬空摆矩行仅用本侧世界系摆角、角速度。原地面 K 的角增益约 10，
 * 11-15 提车时曾推不过髋静摩擦。用原悬空 PD 的 30/2 和 8 N·m
 * 限幅作为首版状态反馈初值，保持双侧提车的回正强度（切换效果待实测）。 */
volatile float air_kp_theta = 30.0f; /* N·m/rad */
volatile float air_kd_theta = 2.0f;  /* N·m·s/rad */
volatile float air_tp_max = 8.0f;    /* N·m */

float LQR_PositionError(float error, float deadband)
{
    if (fabsf(error) <= deadband)
        return 0.0f;
    return error - copysignf(deadband, error);
}

void LQR_calc(const ten_balance_observation *observe, const ten_balance_target *balance_targets,
              const ten_balance_target *step_targets, uint8_t wheel_block_mask,
              uint8_t step_leg_mask, const LQR_FeedbackOptions_s *options,
              four_control_data *control_datas)
{
    float x[10];
    float u[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    /* 双腿实测平均腿长选带(双腿由恢复/平衡保持一致, 单腿卡住时取平均仍是合理工作点) */
    float L_avg = 0.5f * (observe->l_leg_length + observe->r_leg_length);
    while (lqr_k_band < 5 &&
           L_avg > (0.175f + 0.05f * lqr_k_band) + 0.01f)
        lqr_k_band++;
    while (lqr_k_band > 0 &&
           L_avg < (0.175f + 0.05f * (lqr_k_band - 1)) - 0.01f)
        lqr_k_band--;
    float (*K)[10] = K_sets[lqr_k_band];

    x[0] = LQR_PositionError(observe->s - balance_targets->s_target,
                             options->position_deadband);
    x[1] = observe->ds - balance_targets->ds_target;
    x[2] = observe->yaw - balance_targets->yaw_target;
    x[3] = observe->dyaw - balance_targets->dyaw_target;
    x[4] = observe->l_theta - balance_targets->l_theta_target;
    x[5] = observe->l_dtheta - balance_targets->l_dtheta_target;
    x[6] = observe->r_theta - balance_targets->r_theta_target;
    x[7] = observe->r_dtheta - balance_targets->r_dtheta_target;
    x[8] = observe->phi - balance_targets->phi_target;
    x[9] = observe->dphi - balance_targets->dphi_target;

    /* 单侧空转也会污染平均里程和差动航向，支撑侧暂只用腿与机身姿态反馈。 */
    const int first_col = wheel_block_mask ? 4 : 0;
    for (int i = 0; i < 4; i++)
    {
        const uint8_t side = (i % 2 == 0) ? GRF_AIR_LEFT : GRF_AIR_RIGHT;
        if (wheel_block_mask & side)
        {
            if (i >= 2)
            {
                /* 摆动侧走专用参考，普通空侧仍以世界系竖直为目标。 */
                float theta = (i == 2) ? observe->l_theta : observe->r_theta;
                float dtheta = (i == 2) ? observe->l_dtheta : observe->r_dtheta;
                if (step_leg_mask & side)
                {
                    theta -= (i == 2) ? step_targets->l_theta_target : step_targets->r_theta_target;
                    dtheta -= (i == 2) ? step_targets->l_dtheta_target : step_targets->r_dtheta_target;
                }
                theta -= 2.0f * PI * roundf(theta / (2.0f * PI));
                u[i] = air_kp_theta * theta + air_kd_theta * dtheta;
                const float tp_max = air_tp_max;
                LIMIT_MIN_MAX(u[i], -tp_max, tp_max);
            }
            continue; /* 空侧 Tw 保持零，Tp 走悬空增益行 */
        }

        for (int j = first_col; j < 10; j++)
        {
            float scale = 1.0f;
            if (i < 2 && j >= 4 && j < 8)
                scale = options->wheel_theta_scale;
            else if (i >= 2 && j >= 8)
                scale = options->hip_pitch_scale;
            u[i] += rate * K[i][j] * x[j] * scale;
        }
    }

    control_datas->l_Tw = -u[0];
    control_datas->r_Tw = -u[1];
    control_datas->l_Tp = -u[2];
    control_datas->r_Tp = -u[3];
}
