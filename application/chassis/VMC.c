#include "VMC.h"
#include "balance.h"
#include "chassis.h"
#include "math.h"

/* g = dl/dβ。由 F*δl + Tp*δΘ = Tf*δθf + Tb*δθb 可得：
 * Tf = -g*F + Tp/2，Tb = +g*F + Tp/2。
 * Tleg 是虚拟腿长力 F（单位 N），不是关节力矩。 */
void VMC_Calc(four_control_data *control_datas, r_leg_motor_torque *leg_motor_torques, r_leg_Resolve *leg)
{
    float g = -0.5f * R_LEG_A * sin(0.5f * leg->r_beta)
              - (R_LEG_A * R_LEG_A * sin(0.5f * leg->r_beta) * cos(0.5f * leg->r_beta))
                / (2.0f * sqrt(R_LEG_B * R_LEG_B
                               - R_LEG_A * R_LEG_A * sin(0.5f * leg->r_beta) * sin(0.5f * leg->r_beta)));

    float F = control_datas->r_Tleg;
    leg_motor_torques->r_Tf = -g * F + 0.5f * control_datas->r_Tp;
    leg_motor_torques->r_Tb = +g * F + 0.5f * control_datas->r_Tp;
}

void l_VMC_Calc(four_control_data *control_datas, l_leg_motor_torque *leg_motor_torques, l_leg_Resolve *leg)
{
    float g = -0.5f * L_LEG_A * sin(0.5f * leg->l_beta)
              - (L_LEG_A * L_LEG_A * sin(0.5f * leg->l_beta) * cos(0.5f * leg->l_beta))
                / (2.0f * sqrt(L_LEG_B * L_LEG_B
                               - L_LEG_A * L_LEG_A * sin(0.5f * leg->l_beta) * sin(0.5f * leg->l_beta)));

    float F = control_datas->l_Tleg;
    float Tf = -g * F + 0.5f * control_datas->l_Tp;
    float Tb = +g * F + 0.5f * control_datas->l_Tp;

    /* 左腿反馈在进入运动学前取过负，这里把力矩换回原电机坐标。
     * 因此 ChassisTask 下发 l_Tf/l_Tb 时不能再加负号。 */
    leg_motor_torques->l_Tf = -Tf;
    leg_motor_torques->l_Tb = -Tb;
}
