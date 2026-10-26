#include "VMC.h"
#include "balance.h"
#include "chassis.h"
#include "math.h"






void VMC_Calc(four_control_data *control_datas,r_leg_motor_torque *leg_motor_torques,r_leg_Resolve *leg)
{
    
    float g=-0.5f*R_LEG_A*sin(0.5f*leg->r_beta)-(R_LEG_A*R_LEG_A*sin(0.5f*leg->r_beta)*cos(0.5f*leg->r_beta))/(2.0f*sqrt(R_LEG_B*R_LEG_B-R_LEG_A*R_LEG_A*sin(0.5f*leg->r_beta)*sin(0.5f*leg->r_beta)));
    
    float Tf= g*control_datas->r_Tleg + 0.5f*control_datas->r_Tp;
    float Tb= -g*control_datas->r_Tleg + 0.5f*control_datas->r_Tp;

    leg_motor_torques->r_Tf=Tf;
    leg_motor_torques->r_Tb=Tb;

    
  
}

void l_VMC_Calc(four_control_data *control_datas,l_leg_motor_torque *leg_motor_torques,l_leg_Resolve *leg)
{
    
    float g=-0.5f*L_LEG_A*sin(0.5f*leg->l_beta)-(L_LEG_A*L_LEG_A*sin(0.5f*leg->l_beta)*cos(0.5f*leg->l_beta))/(2.0f*sqrt(L_LEG_B*L_LEG_B-L_LEG_A*L_LEG_A*sin(0.5f*leg->l_beta)*sin(0.5f*leg->l_beta)));
    
    float Tf= g*control_datas->l_Tleg + 0.5f*control_datas->l_Tp;
    float Tb= -g*control_datas->l_Tleg + 0.5f*control_datas->l_Tp;

    leg_motor_torques->l_Tf=-Tf;
    leg_motor_torques->l_Tb=-Tb;

    
  
}
