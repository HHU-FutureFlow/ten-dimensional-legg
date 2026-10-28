#ifndef BALANCE_H
#define BALANCE_H
#pragma once




//零点标定（2026-10-28 新标定，标定结果.docx）：
//α = 大腿电机读数 - 大腿居中(竖直)零点：右 0.62886 / 左 3.49184（固件坐标，左腿已取反）
//β = -(两编码器差值) + 腿最长姿态偏置：右 1.57548 / 左 6.60945
//  （腿最长 = 轮心位于髋正下方；对称位形 β = -2α，由旧标定数据验证到 0.01 rad）


#define R_LEG_A (0.21f)
#define R_LEG_B (0.25f)
#define R_HALF_WHEEL_BASE (0.06f)
#define R_REDUCTION_RATIO (15.8f)



typedef struct
{
    //LQR用

    float s;//      ⽔平位移
    float ds;//     ⽔平速度

    float yaw;//机体偏航⻆
    float dyaw;//机体偏航⻆速度

    float l_theta;//该腿摆杆相对竖直倾⻆
    float l_dtheta;//该腿摆杆⻆速度

    float r_theta;//该腿摆杆相对竖直倾⻆
    float r_dtheta;//该腿摆杆⻆速度

    float phi;//机体俯仰⻆
    float dphi;//机体俯仰⻆速度



    //PID用
    float l_leg_length;//该腿摆杆长度
    float r_leg_length;


} ten_balance_observation; //需要观测的状态



typedef struct {
   //LQR用
    float s_target;//      ⽔平位移
    float ds_target;//     ⽔平速度

    float yaw_target;//机体偏航⻆
    float dyaw_target;//机体偏航⻆速度

    float l_theta_target;//该腿摆杆相对竖直倾⻆
    float l_dtheta_target;//该腿摆杆⻆速度

    float r_theta_target;//该腿摆杆相对竖直倾⻆
    float r_dtheta_target;//该腿摆杆⻆速度

    float phi_target;//机体俯仰⻆
    float dphi_target;//机体俯仰⻆速度


    //PID用
    float l_leg_length_target;//该腿摆杆长度
    float r_leg_length_target;//该腿摆杆长度

} ten_balance_target; //目标值

typedef struct
{
    float r_alpha;
    float r_beta;
    float r_dalpha;
    float r_dbeta;

} r_leg_Resolve;//解算要用的变量

typedef struct {
    float r_x;
    float r_dx;
} r_Wheel_data; 

typedef struct {
    float r_x;
    float r_z;
} r_Point2D; //二维坐标系


typedef struct 
{
    float l_Tw;
    float r_Tw;

    float l_Tp;
    float r_Tp;

    float l_Tleg;
    float r_Tleg;

} four_control_data; // Tw/Tp 单位为 N·m；Tleg 为虚拟腿长力，单位 N

typedef struct {
    float r_Tf;
    float r_Tb;
} r_leg_motor_torque; //腿部电机力矩





float r_leg_theta_calculate(r_leg_Resolve *r_leg,int r_L,int r_theta);
void BalanceObserve(ten_balance_observation *ten_observe);
void  r_Alpha_beta_calculate(float r_motor_big_leg,float r_motor_small_leg,float r_motor_big_speed,float r_motor_small_speed,r_leg_Resolve *r_leg);
float r_leg_theta_rate_calculate(r_leg_Resolve *r_leg);
void r_wheel_control(float r_wheel_angle,float r_wheel_w,r_Wheel_data *r_wheel);

#define L_LEG_A (0.21f)
#define L_LEG_B (0.25f)
#define L_HALF_WHEEL_BASE (0.06f)
#define L_REDUCTION_RATIO (15.8f)




typedef struct
{
    float l_alpha;
    float l_beta;
    float l_dalpha;
    float l_dbeta;

} l_leg_Resolve;//解算要用的变量

typedef struct {
    float l_x;
    float l_dx;
} l_Wheel_data; 

typedef struct {
    float l_x;
    float l_z;
} l_Point2D; //二维坐标系




typedef struct {
    float l_Tf;
    float l_Tb;
} l_leg_motor_torque; //腿部电机力矩





float l_leg_theta_calculate(l_leg_Resolve *l_leg,int l_L,int l_theta);

void  l_Alpha_beta_calculate(float l_motor_big_leg,float l_motor_small_leg,float l_motor_big_speed,float l_motor_small_speed,l_leg_Resolve *l_leg);
float l_leg_theta_rate_calculate(l_leg_Resolve *l_leg);
void l_wheel_control(float l_wheel_angle,float l_wheel_w,l_Wheel_data *l_wheel);

#endif 
