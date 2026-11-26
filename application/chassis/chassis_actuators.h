#ifndef CHASSIS_ACTUATORS_H
#define CHASSIS_ACTUATORS_H

#include "dji_motor.h"
#include "dmmotor.h"

typedef struct
{
    DMMotorInstance *r_front_8009_motor;
    DMMotorInstance *r_behind_8009_motor;
    DMMotorInstance *l_front_8009_motor;
    DMMotorInstance *l_behind_8009_motor;
    DJIMotorInstance *l_wheel_motor;
    DJIMotorInstance *r_wheel_motor;
} ChassisActuators_s;

/* 在任务启动前注册一次。调用方负责在启动任务前设置自己的输出初态。 */
void ChassisActuatorsInit(ChassisActuators_s *motors);

#endif
