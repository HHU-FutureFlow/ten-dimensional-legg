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
extern volatile float l_front_raw_position, l_behind_raw_position;
extern volatile unsigned int l_zero_ready;
/* +1/-1 converts raw left wheel direction to forward; confirm on hardware. */
extern volatile float l_wheel_direction;

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