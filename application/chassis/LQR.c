#include "LQR.h"



float K[4][10]=
{
    {  },
    {   },
    {   },
    {   }

};

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
    x[8]=observe->l_leg_length-balance_targets->l_leg_length_target;
    x[9]=observe->r_leg_length-balance_targets->r_leg_length_target;




    for (int i = 0; i < 4; i++) {
        u[i] = 0.0;
        for (int j = 0; j < 10; j++) {
            u[i] += K[i][j] * x[j];
        }
    }

    control_datas->l_Tw=-u[0];
    control_datas->r_Tw=-u[1];
    control_datas->l_Tp=-u[2];
    control_datas->r_Tp=-u[3];


}




