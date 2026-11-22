#include "debug_mode.h"

#include "cmsis_os.h"
#include "friction_test.h"
#include "robot_def.h"

#if !defined(ONE_BOARD) || CHASSIS_USE_N630_VESC
#error "Bench firmware requires ONE_BOARD and C620 wheel drives"
#endif

void DebugModeInit(void)
{
    ChassisActuators_s motors;
    ChassisActuatorsInit(&motors);
    FrictionTestInit(&motors);
}

void DebugModeTask(void)
{
    FrictionTestUpdate(osKernelSysTick());
}
