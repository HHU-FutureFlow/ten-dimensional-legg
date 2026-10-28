#ifndef r_CHASSIS_H
#define r_CHASSIS_H
#include "ins_task.h"
#include "balance.h"

extern attitude_t *chassis_IMU_data;
extern r_leg_Resolve r_leg;
extern l_leg_Resolve l_leg;
extern r_Wheel_data r_wheel;
extern l_Wheel_data l_wheel;

extern four_control_data four_control_datas;
extern ten_balance_target ten_targets;
extern ten_balance_observation ten_balance_observe;



extern l_leg_motor_torque l_leg_motor_torques;


/* Joint reference positions in radians; adjust in debugger at calibration pose. */
extern volatile float l_front_zero, l_behind_zero;
/* +1/-1 converts raw left wheel direction to forward; confirm on hardware. */
extern volatile float l_wheel_direction;

/* 电机反馈量调试映射:在 Ozone/VarScope 全局符号里搜 chassis_dbg 即可
 * leg_*  : [0]r_front [1]r_behind [2]l_front [3]l_behind
 * wheel_*: [0]l_wheel [1]r_wheel
 * 位置单位 rad,速度单位 rad/s,符号与 ChassisTask 内的控制量一致 */
typedef struct
{
    float leg_pos[4];
    float leg_vel[4];
    float wheel_pos[2];
    float wheel_vel[2];
} Chassis_Dbg_s;
extern volatile Chassis_Dbg_s chassis_dbg;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // r_CHASSIS_H