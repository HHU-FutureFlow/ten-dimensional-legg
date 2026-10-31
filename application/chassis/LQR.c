#include "LQR.h"

/* 完整 10 维 LQR，工作点 l=0.30 m，控制周期 1 ms。
 * 由 lqr_v2/run_all.py 生成，坐标、输入符号与固件一致，不要再翻转任何列。
 *
 * 列序: s | ds | yaw | dyaw | lθ | ldθ | rθ | rdθ | φ | dφ
 * 行序: [0]=l_Tw(左轮) [1]=r_Tw(右轮) [2]=l_Tp(左腿) [3]=r_Tp(右腿)
 * 控制律: T = -K(x-x_ref)，对应下方 control_datas = -u。
 *
 * s/ds：车前进为正；yaw/dyaw：左转为正。
 * θ/dθ：轮心在髋前为正；φ/dφ：车头上抬为正，即 -Pitch/-Gyro[0]。
 * Tw>0：轮子前滚；Tp>0：轮子相对机体向前摆。
 */
float K[4][10] =
{
    {   -1.915674f,   -1.940664f,   -0.725556f,   -0.342231f,   +4.818297f,   +0.888503f,   +2.709488f,   +0.494015f,   +1.317706f,   +0.239200f },
    {   -1.915674f,   -1.940664f,   +0.725556f,   +0.342231f,   +2.709488f,   +0.494015f,   +4.818297f,   +0.888503f,   +1.317706f,   +0.239200f },
    {   -2.715748f,   -2.415919f,   +4.310990f,   +1.812488f,   +9.697371f,   +1.234845f,   -2.915019f,   +0.048127f,  -21.809681f,   -1.297665f },
    {   -2.715748f,   -2.415919f,   -4.310990f,   -1.812488f,   -2.915019f,   +0.048127f,   +9.697371f,   +1.234845f,  -21.809681f,   -1.297665f }
};

float rate = 1.0f;

void LQR_calc(ten_balance_observation *observe, ten_balance_target *balance_targets, four_control_data *control_datas)
{
    float x[10];
    float u[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    x[0] = observe->s - balance_targets->s_target;
    x[1] = observe->ds - balance_targets->ds_target;
    x[2] = observe->yaw - balance_targets->yaw_target;
    x[3] = observe->dyaw - balance_targets->dyaw_target;
    x[4] = observe->l_theta - balance_targets->l_theta_target;
    x[5] = observe->l_dtheta - balance_targets->l_dtheta_target;
    x[6] = observe->r_theta - balance_targets->r_theta_target;
    x[7] = observe->r_dtheta - balance_targets->r_dtheta_target;
    x[8] = observe->phi - balance_targets->phi_target;
    x[9] = observe->dphi - balance_targets->dphi_target;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 10; j++)
            u[i] += rate * K[i][j] * x[j];
    }

    control_datas->l_Tw = -u[0];
    control_datas->r_Tw = -u[1];
    control_datas->l_Tp = -u[2];
    control_datas->r_Tp = -u[3];
}
