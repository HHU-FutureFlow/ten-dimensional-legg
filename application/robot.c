#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "shoot.h"
#include "robot_cmd.h"
#endif


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    // GimbalInit();
    // ShootInit();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
    GimbalInit();
    // ShootInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    // GimbalTask();
    // ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    /* ChassisTask 已挪到独立的 1 kHz 任务(StartCHASSISTASK),不在这里调用,
     * 否则会跑两遍。
     * GimbalTask/ShootTask 必须与 GimbalInit/ShootInit 配套:轮腿没有云台和
     * 发射机构,ShootInit 已注释,而 ShootTask 会对未初始化的 loader /
     * shoot_can_comm 空指针读写 → BusFault → ROBOT 任务死,遥控链路不通。
     * (对照 develop 双板工程:那边每个被调用的任务都有配套 Init。) */
    // GimbalTask();
    // ShootTask();
#endif

}
