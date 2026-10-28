#include "LQR.h"




/* 列序: s | ds | yaw | dyaw | lθ | ldθ | rθ | rdθ | φ | dφ
 * 行序: [0]=l_Tw(左轮) [1]=r_Tw(右轮) [2]=l_Tp(左腿) [3]=r_Tp(右腿)
 * 板凳模型: 保留轮子两行, 腿部两行清零 */
float K[4][10] =
{
    {   4.042174f,  20.240416f, -137.789048f,  -6.202160f, -16.701273f, -0.877369f,  -5.017493f, -0.017654f,  +44.356862f,  +6.399397f },
    {   2.802098f,  27.349857f,  103.030310f,   3.852446f,  -5.513473f, -0.095412f, -21.040766f, -0.783158f,  +51.320052f,  +6.305144f },
    {   0.0f,        0.0f,         0.0f,        0.0f,       0.0f,       0.0f,       0.0f,       0.0f,        0.0f,        0.0f        },
    {   0.0f,        0.0f,         0.0f,        0.0f,       0.0f,       0.0f,       0.0f,       0.0f,        0.0f,        0.0f        }
};


// float K[4][10] =
// {
//     {  4.042174f,  20.240416f, -137.789048f, -6.202160f, -16.701273f, -0.877369f,  -5.017493f, -0.017654f,  -44.356862f,  -6.399397f },
//     {  2.802098f,  27.349857f,  103.030310f,  3.852446f,  -5.513473f, -0.095412f, -21.040766f, -0.783158f,  -51.320052f,  -6.305144f },
//     {  3.555269f, -35.952177f, -929.901337f, -29.292768f, 133.423882f,  5.064401f, -29.160967f, -0.818225f,  -94.202648f, -33.618322f },
//     { -19.424088f, -17.610095f, 1055.445938f, 32.794986f, -42.803231f, -1.247324f, 109.567419f,  3.972082f, -145.568572f, -42.591322f }
// };
float rate = 0.1;
float x[10];
float u[4];

void LQR_calc(ten_balance_observation *observe,ten_balance_target *balance_targets,four_control_data *control_datas)
{
    float x[10];
    float u[4];

    x[0]=observe->s-balance_targets->s_target;
    x[1]=observe->ds-balance_targets->ds_target;
    x[2]=observe->yaw-balance_targets->yaw_target;
    x[3]=observe->dyaw-balance_targets->dyaw_target;
    x[4]=observe->l_theta-balance_targets->l_theta_target;
    x[5]=observe->l_dtheta-balance_targets->l_dtheta_target;
    x[6]=observe->r_theta-balance_targets->r_theta_target;
    x[7]=observe->r_dtheta-balance_targets->r_dtheta_target;
    x[8]=observe->phi-balance_targets->phi_target;
    x[9]=observe->dphi-balance_targets->dphi_target;




    for (int i = 0; i < 4; i++) {
        u[i] = 0.0;
        for (int j = 0; j < 10; j++) {
            u[i] += rate * K[i][j] * x[j];
        }
    }

    control_datas->l_Tw=-u[0];
    control_datas->r_Tw=-u[1];
    control_datas->l_Tp=-u[2];
    control_datas->r_Tp=-u[3];


}




