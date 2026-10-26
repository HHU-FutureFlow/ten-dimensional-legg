#ifndef _VMC_H
#define _VMC_H

#include "chassis.h"
#include "balance.h"

void VMC_Calc(four_control_data *control_datas,r_leg_motor_torque *leg_motor_torques,r_leg_Resolve *leg);
void l_VMC_Calc(four_control_data *control_datas,l_leg_motor_torque *leg_motor_torques,l_leg_Resolve *leg);


#endif