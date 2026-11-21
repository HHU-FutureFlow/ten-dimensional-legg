#ifndef _LQR_H
#define _LQR_H

#include "chassis.h"
#include "balance.h"

void LQR_calc(ten_balance_observation *observe,ten_balance_target *balance_targets,four_control_data *control_datas);
void LQR_SetAir(uint8_t air);

/* ===== 可在线整定的 LQR 旋钮（均为 volatile, EmberProbe 可直接改）===== */
extern volatile float lqr_s_deadband;        /* m, 位置误差死区, 0 = 关闭 */
extern volatile float lqr_theta_gain_scale;  /* Tp 行摆角列增益倍数, 1.0 = 原 K */
extern volatile float lqr_tw_theta_gain_scale; /* Tw 行摆角列保留比例, 蹲下期置 0 */
extern volatile float lqr_tp_phi_scale;       /* Tp 行 φ|dφ 列缩放, PUSH 期放松对机身的执着 */
extern volatile float lqr_air_att_gain;      /* 悬空姿态通道: 0=旧行为, +1=设计, -1=反极性 */
extern volatile float lqr_gnd_decouple;      /* 地面解耦: 0=旧行为(phi 混在腿角里), 1=完全扣除 */

#define LQR_EN

#endif
