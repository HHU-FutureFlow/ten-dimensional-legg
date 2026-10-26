#include "balance.h"
#include "chassis.h"
#include "math.h"


void  r_Alpha_beta_calculate(float r_motor_big_leg,float r_motor_small_leg,float r_motor_big_speed,float r_motor_small_speed,r_leg_Resolve *r_leg)
{
    r_leg->r_alpha = (r_motor_big_leg -0.136822f);
    r_leg->r_beta = -(r_motor_big_leg - r_motor_small_leg) + ((31.17*2)*PI/180.0f)-0.019f;
    r_leg->r_dalpha = r_motor_big_speed;
    r_leg->r_dbeta =r_motor_big_speed - r_motor_small_speed;

}

float r_leg_theta_calculate(r_leg_Resolve *r_leg,int r_L,int r_theta)
{
    r_Point2D r_c,r_b,r_p,r_d,r_pd,r_bc;
    r_c.r_x=-R_LEG_A*sin(-r_leg->r_alpha);
    r_c.r_z=-R_LEG_A*cos(-r_leg->r_alpha);
    r_b.r_x= R_LEG_A*sin(r_leg->r_beta+r_leg->r_alpha);
    r_b.r_z=-R_LEG_A*cos(r_leg->r_beta+r_leg->r_alpha);
    r_bc.r_x = r_c.r_x-r_b.r_x;
    r_bc.r_z = r_c.r_z-r_b.r_z;
    r_p.r_x=(r_b.r_x+r_c.r_x)/2;
    r_p.r_z=(r_b.r_z+r_c.r_z)/2;
    float r_BC = sqrt(pow(r_b.r_x - r_c.r_x, 2) + pow(r_b.r_z - r_c.r_z, 2));
    float r_PD = sqrt(pow(R_LEG_B, 2) - pow((r_BC/2), 2));
    float r_k = r_PD/r_BC;
    r_pd.r_x = -r_k * r_bc.r_z ;
    r_pd.r_z = r_k * r_bc.r_x;
    r_d.r_x=r_pd.r_x+r_p.r_x;
    r_d.r_z=r_pd.r_z+r_p.r_z;//�ֶ�
    float r_l =  sqrt(pow(r_d.r_x, 2) + pow(r_d.r_z, 2));//�ȳ�
    float  r_Theta = atan2(r_d.r_x,-r_d.r_z);//�ڽ�

    if((r_L==1)  &&  (r_theta==0))
    {
        return r_l;
    }
    else if((r_L==0) && (r_theta==1))
    {
       return r_Theta;
    }

}

float r_leg_theta_rate_calculate(r_leg_Resolve *r_leg)
{
    return r_leg->r_dalpha - 0.5f * r_leg->r_dbeta;
}

void r_wheel_control(float r_wheel_angle_aps,float r_wheel_w_aps,r_Wheel_data *r_wheel)
{
    float r_wheel_angle_rad=r_wheel_angle_aps* PI/ 180.0f;
    float r_wheel_w_rad=r_wheel_w_aps* PI/ 180.0f;
    float r_x=r_wheel_angle_rad/R_REDUCTION_RATIO*R_HALF_WHEEL_BASE;
    float r_dx=r_wheel_w_rad/R_REDUCTION_RATIO*R_HALF_WHEEL_BASE;

    r_wheel->r_x=r_x;
    r_wheel->r_dx=r_dx;

}

void BalanceObserve(ten_balance_observation *ten_observe)
{

    ten_observe->s =  (r_wheel.r_x+l_wheel.l_x)/2.0f;
    ten_observe->ds = (r_wheel.r_dx+l_wheel.l_dx)/2.0f;

    ten_observe->yaw = chassis_IMU_data->Yaw* 0.01745329252f;//正负须规定
    ten_observe->dyaw =chassis_IMU_data->Gyro[2];
    
    ten_observe->phi = -chassis_IMU_data->Pitch* 0.01745329252f;
    ten_observe->dphi =-chassis_IMU_data->Gyro[0];

    ten_observe->l_theta =l_leg_theta_calculate(&l_leg, 0, 1)-chassis_IMU_data->Pitch* 0.01745329252f;
    ten_observe->l_dtheta =l_leg_theta_rate_calculate(&l_leg)-chassis_IMU_data->Gyro[0];

    ten_observe->r_theta =r_leg_theta_calculate(&r_leg, 0, 1)-chassis_IMU_data->Pitch* 0.01745329252f;
    ten_observe->r_dtheta =r_leg_theta_rate_calculate(&r_leg)-chassis_IMU_data->Gyro[0];


    ten_observe->l_leg_length = l_leg_theta_calculate(&l_leg, 1, 0);
    ten_observe->r_leg_length = r_leg_theta_calculate(&r_leg, 1, 0);

   
}




void  l_Alpha_beta_calculate(float l_motor_big_leg,float l_motor_small_leg,float l_motor_big_speed,float l_motor_small_speed,l_leg_Resolve *l_leg)
{
    l_leg->l_alpha = (l_motor_big_leg +2.82f);
    l_leg->l_beta = -(l_motor_big_leg - l_motor_small_leg)-0.77f + ((31.17*2)*PI/180.0f);
    l_leg->l_dalpha = l_motor_big_speed;
    l_leg->l_dbeta =l_motor_big_speed - l_motor_small_speed;

}

float l_leg_theta_calculate(l_leg_Resolve *l_leg,int l_L,int l_theta)
{
    l_Point2D l_c,l_b,l_p,l_d,l_pd,l_bc;
    l_c.l_x=-L_LEG_A*sin(-l_leg->l_alpha);
    l_c.l_z=-L_LEG_A*cos(-l_leg->l_alpha);
    l_b.l_x= L_LEG_A*sin(l_leg->l_beta+l_leg->l_alpha);
    l_b.l_z=-L_LEG_A*cos(l_leg->l_beta+l_leg->l_alpha);
    l_bc.l_x = l_c.l_x-l_b.l_x;
    l_bc.l_z = l_c.l_z-l_b.l_z;
    l_p.l_x=(l_b.l_x+l_c.l_x)/2;
    l_p.l_z=(l_b.l_z+l_c.l_z)/2;
    float l_BC = sqrt(pow(l_b.l_x - l_c.l_x, 2) + pow(l_b.l_z - l_c.l_z, 2));
    float l_PD = sqrt(pow(L_LEG_B, 2) - pow((l_BC/2), 2));
    float l_k = l_PD/l_BC;
    l_pd.l_x = -l_k * l_bc.l_z ;
    l_pd.l_z = l_k * l_bc.l_x;
    l_d.l_x=l_pd.l_x+l_p.l_x;
    l_d.l_z=l_pd.l_z+l_p.l_z;//�ֶ�
    float l_l =  sqrt(pow(l_d.l_x, 2) + pow(l_d.l_z, 2));//�ȳ�
    float  l_Theta = atan2(l_d.l_x,-l_d.l_z);//�ڽ�

    if((l_L==1)  &&  (l_theta==0))
    {
        return l_l;
    }
    else if((l_L==0) && (l_theta==1))
    {
       return l_Theta;
    }

}

float l_leg_theta_rate_calculate(l_leg_Resolve *l_leg)
{
    return l_leg->l_dalpha - 0.5f * l_leg->l_dbeta;
}

void l_wheel_control(float l_wheel_angle_aps,float l_wheel_w_aps,l_Wheel_data *l_wheel)
{
    float l_wheel_angle_rad=l_wheel_angle_aps* PI/ 180.0f;
    float l_wheel_w_rad=l_wheel_w_aps* PI/ 180.0f;
    float l_x=l_wheel_angle_rad/L_REDUCTION_RATIO*L_HALF_WHEEL_BASE;
    float l_dx=l_wheel_w_rad/L_REDUCTION_RATIO*L_HALF_WHEEL_BASE;

    l_wheel->l_x=l_x;
    l_wheel->l_dx=l_dx;

}



