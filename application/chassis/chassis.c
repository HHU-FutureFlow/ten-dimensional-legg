#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "n630.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
# include "dmmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "LQR.h"
#include "controller.h"
#include "VMC.h"
#include <math.h>


static DMMotorInstance *r_front_8009_motor,*r_behind_8009_motor;
static DMMotorInstance *l_front_8009_motor, *l_behind_8009_motor;
static DJIMotorInstance *l_wheel_motor, *r_wheel_motor;
static PIDInstance l_leg_length_pid;
static PIDInstance r_leg_length_pid;

static PIDInstance l_theta_pid;
static PIDInstance r_theta_pid;

attitude_t *chassis_IMU_data;
r_leg_Resolve r_leg;
r_Wheel_data r_wheel;
ten_balance_observation ten_balance_observe;


four_control_data four_control_datas;
r_leg_motor_torque r_leg_motor_torques;


l_leg_Resolve l_leg;
l_Wheel_data l_wheel;



l_leg_motor_torque l_leg_motor_torques;


ten_balance_target ten_targets ;



/* Set raw joint angles measured at the mirrored right calibration pose.
 * Raw reference on right: front = rear = 0.357252 rad.
 * RAM only; fill both offsets in the debugger.
 */
volatile float l_front_zero = 0.0f, l_behind_zero = 0.0f;

volatile float l_wheel_direction = -1.0f;

volatile Chassis_Dbg_s chassis_dbg; // 仅占RAM,供调试器观测

/* 刷新电机反馈调试量,符号与ChassisTask中的控制量保持一致 */
static void ChassisDbgUpdate()
{
    const float d2r = 0.01745329252f;

    chassis_dbg.leg_pos[0] = r_front_8009_motor->measure.position;
    chassis_dbg.leg_pos[1] = r_behind_8009_motor->measure.position;
    chassis_dbg.leg_pos[2] = -l_front_8009_motor->measure.position;
    chassis_dbg.leg_pos[3] = -l_behind_8009_motor->measure.position;

    chassis_dbg.leg_vel[0] = r_front_8009_motor->measure.velocity;
    chassis_dbg.leg_vel[1] = r_behind_8009_motor->measure.velocity;
    chassis_dbg.leg_vel[2] = -l_front_8009_motor->measure.velocity;
    chassis_dbg.leg_vel[3] = -l_behind_8009_motor->measure.velocity;

    chassis_dbg.wheel_pos[0] = -l_wheel_motor->measure.total_angle * d2r;
    chassis_dbg.wheel_pos[1] = r_wheel_motor->measure.total_angle * d2r;
    chassis_dbg.wheel_vel[0] = -l_wheel_motor->measure.speed_aps * d2r;
    chassis_dbg.wheel_vel[1] = r_wheel_motor->measure.speed_aps * d2r;
}

/* 把喂进 LQR 的状态拷一份出来,方便在 Ozone 里和 T_lw 对着看 */
static void ChassisDbgLqr(const ten_balance_observation *o)
{
    chassis_dbg.lqr_x[0] = o->s;        chassis_dbg.lqr_x[1] = o->ds;
    chassis_dbg.lqr_x[2] = o->yaw;      chassis_dbg.lqr_x[3] = o->dyaw;
    chassis_dbg.lqr_x[4] = o->l_theta;  chassis_dbg.lqr_x[5] = o->l_dtheta;
    chassis_dbg.lqr_x[6] = o->r_theta;  chassis_dbg.lqr_x[7] = o->r_dtheta;
    chassis_dbg.lqr_x[8] = o->phi;      chassis_dbg.lqr_x[9] = o->dphi;
}

void ChassisInit()
{
    chassis_IMU_data = INS_Init();

    Motor_Init_Config_s r_config = {
        .can_init_config = {
            .can_handle = &hcan1,            
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    r_config.can_init_config.tx_id = 2;
    r_config.can_init_config.rx_id = 2;
    r_front_8009_motor = DMMotorInit(&r_config);

    r_config.can_init_config.tx_id = 3;
    r_config.can_init_config.rx_id = 3;
    r_behind_8009_motor = DMMotorInit(&r_config);

    Motor_Init_Config_s l_config =
    {
        .can_init_config = {
            .can_handle = &hcan1
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
    };

    l_config.can_init_config.tx_id = 5;
    l_config.can_init_config.rx_id = 5;
    l_front_8009_motor = DMMotorInit(&l_config);

    l_config.can_init_config.tx_id = 4;
    l_config.can_init_config.rx_id = 4;
    l_behind_8009_motor = DMMotorInit(&l_config);


    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 5.0f,
                .Ki = 1.5f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3f,
            },
        },
        .controller_setting_init_config = {

            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

        },

        .motor_type = M3508,

    };

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    l_wheel_motor = PowerControlInit(&chassis_motor_config);



    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    r_wheel_motor = PowerControlInit(&chassis_motor_config);




    static PID_Init_Config_s r_leg_length_pid_config = 

    {
        .Kp = 90.0f,  
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 9000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 0.0f,
    };


    PIDInit(&r_leg_length_pid, &r_leg_length_pid_config);
    PID_Init_Config_s l_leg_length_pid_config = r_leg_length_pid_config;
    PIDInit(&l_leg_length_pid, &l_leg_length_pid_config);



    

    

}



void ChassisTask()
{
    ChassisDbgUpdate();
    /* 使能帧只在 DMMotorInit/DMMotorEnable 里发一次。原来在这里每 5ms 发两帧
     * DM_CMD_MOTOR_MODE，既占 CAN 带宽又打断 MIT 帧时序，已移除。 */
    SetPowerLimit(1000000.0f);   /* @Todo 接裁判系统后换成真实底盘功率 */
    float r_theta_f=r_front_8009_motor->measure.position;
    float r_theta_b=r_behind_8009_motor->measure.position;
    float r_speed_f=r_front_8009_motor->measure.velocity;
    float r_speed_b=r_behind_8009_motor->measure.velocity;
    float r_wheel_angle=r_wheel_motor->measure.total_angle;
    float r_wheel_w=r_wheel_motor->measure.speed_aps;


    float l_theta_f= -l_front_8009_motor->measure.position;
    float l_theta_b=-l_behind_8009_motor->measure.position;
    float l_speed_f=-l_front_8009_motor->measure.velocity;
    float l_speed_b=-l_behind_8009_motor->measure.velocity;
    float l_wheel_angle= -l_wheel_motor->measure.total_angle;
    float l_wheel_w= -l_wheel_motor->measure.speed_aps;

    ten_targets.s_target=0.0f;
    ten_targets.ds_target=0.0f;
    ten_targets.yaw_target=0.0f;
    ten_targets.dyaw_target=0.0f;
    ten_targets.l_theta_target=0.000f;
    ten_targets.l_dtheta_target=0;
    ten_targets.r_theta_target=0.000f;
    ten_targets.r_dtheta_target=0;
   
    ten_targets.phi_target=0;
    ten_targets.dphi_target=0;

    ten_targets.l_leg_length_target = 0.4f;
    ten_targets.r_leg_length_target = 0.4f;

    r_Alpha_beta_calculate(r_theta_f,r_theta_b,r_speed_f,r_speed_b,&r_leg);
    l_Alpha_beta_calculate(l_theta_f,l_theta_b,l_speed_f,l_speed_b,&l_leg);
    r_wheel_control(r_wheel_angle,r_wheel_w,&r_wheel);
    l_wheel_control(l_wheel_angle,l_wheel_w,&l_wheel);
    BalanceObserve(&ten_balance_observe);
    ChassisDbgLqr(&ten_balance_observe);
    LQR_calc(&ten_balance_observe,&ten_targets,&four_control_datas);
    chassis_dbg.lqr_u[0] = four_control_datas.l_Tw;
    chassis_dbg.lqr_u[1] = four_control_datas.r_Tw;
    chassis_dbg.lqr_u[2] = four_control_datas.l_Tp;
    chassis_dbg.lqr_u[3] = four_control_datas.r_Tp;
    four_control_datas.r_Tleg= PIDCalculate(&r_leg_length_pid,ten_balance_observe.r_leg_length,ten_targets.r_leg_length_target);
    four_control_datas.l_Tleg= PIDCalculate(&l_leg_length_pid,ten_balance_observe.l_leg_length,ten_targets.l_leg_length_target);
    VMC_Calc(&four_control_datas,&r_leg_motor_torques,&r_leg);
    l_VMC_Calc(&four_control_datas,&l_leg_motor_torques,&l_leg);
    
    


    

    // DMMotorSetRef(r_front_8009_motor,0.01f );
    // DMMotorSetRef(r_behind_8009_motor,0.01f);
    // DJIMotorSetRef(r_wheel_motor,3318.833*r_control_datas.r_Tw);

    // DMMotorStop(r_front_8009_motor);
    // DMMotorStop(r_behind_8009_motor);
    // DMMotorStop(l_front_8009_motor);
    // DMMotorStop(l_behind_8009_motor);
    // DJIMotorStop(l_wheel_motor);
    // DJIMotorStop(r_wheel_motor); 

    /* Mirror torque as well as position; driver direction remains NORMAL. */
    // DMMotorSetRef(l_front_8009_motor,
    //     isfinite(l_leg_motor_torques.l_Tf) ? -l_leg_motor_torques.l_Tf : 0.0f);
    // DMMotorSetRef(l_behind_8009_motor,
    //     isfinite(l_leg_motor_torques.l_Tb) ? -l_leg_motor_torques.l_Tb : 0.0f);
    // DJIMotorSetRef(l_wheel_motor,l_wheel_direction*3318.833*l_control_datas.l_Tw);



    DMMotorSetCtrlMode(r_front_8009_motor,  DM_CTRL_POSITION);
    DMMotorSetCtrlMode(r_behind_8009_motor, DM_CTRL_POSITION);
    DMMotorSetCtrlMode(l_front_8009_motor,  DM_CTRL_POSITION);
    DMMotorSetCtrlMode(l_behind_8009_motor, DM_CTRL_POSITION);


    DMMotorSetRef(r_front_8009_motor,  -0.03529f);
    DMMotorSetRef(r_behind_8009_motor, -0.13294f);
    DMMotorSetRef(l_front_8009_motor,   -2.77733f);
    DMMotorSetRef(l_behind_8009_motor,  2.31765);

    // DMMotorSetRef(r_front_8009_motor,r_leg_motor_torques.r_Tf );



    // DMMotorSetRef(r_front_8009_motor,r_leg_motor_torques.r_Tf );
    // DMMotorSetRef(r_behind_8009_motor,r_leg_motor_torques.r_Tb );
    // DMMotorSetRef(l_front_8009_motor,l_leg_motor_torques.l_Tf );
    // DMMotorSetRef(l_behind_8009_motor,l_leg_motor_torques.l_Tb );

    /* 轮力矩下发：先按 3508 的真实上限钳位，再换算成电流码。
     * 限幅必须放在换算之前 —— 见 chassis.h 里 WHEEL_T_MAX 的注释。 */
    float r_Tw = four_control_datas.r_Tw;
    float l_Tw = four_control_datas.l_Tw;
    LIMIT_MIN_MAX(r_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
    LIMIT_MIN_MAX(l_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);

    chassis_dbg.Tw_cmd[0] = r_Tw;
    chassis_dbg.Tw_cmd[1] = l_Tw;

    DJIMotorSetRef(r_wheel_motor, WHEEL_T_PER_RAW * r_Tw);
    DJIMotorSetRef(l_wheel_motor, l_wheel_direction * WHEEL_T_PER_RAW * l_Tw);

    



}
