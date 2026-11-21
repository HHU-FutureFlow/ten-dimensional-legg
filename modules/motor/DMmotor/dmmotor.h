#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-54.0f)
#define DM_T_MAX   54.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;     // 反馈 position 越过 ±DM_P_MAX 的次数（12.5 -> -12.5 记 +1）
    float total_position;    // 多圈展开后的连续位置 = position + total_round * (DM_P_MAX - DM_P_MIN)
    uint8_t init_flag;       // 收到第一帧反馈后置 1；之前 DMMotorTask 不出力
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;




typedef enum 
{
    DM_CTRL_TORQUE = 0,   // 纯力矩
    DM_CTRL_POSITION,     // 位置保持（MIT阻抗位置）
} DMMotorCtrlMode_e;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    DMMotorCtrlMode_e ctrl_mode;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;











DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor);
void DMMotorSetCtrlMode(DMMotorInstance *motor, DMMotorCtrlMode_e mode);

/**
 * @brief 位置模式下，把一个"绝对目标角"换算成离当前位置最近的那个等价指令（差值按 2π 归一化）。
 *        用于标定常数可能和当前上电落在不同圈的情况，保证电机只走短的那一边，不会多转一整圈。
 * @param target_abs 目标角，和 measure.total_position 同一参考系（原始方向，未做左右取负）
 * @param cmd        输出：可直接 DMMotorSetRef 的位置指令（电机 ±DM_P_MAX 参考系）
 * @return 1 = 可用；0 = 还没收到反馈 / 反馈已跨过 ±12.5 / 指令超出 ±12.5，此时不要下发位置指令
 */
uint8_t DMMotorNearestPosRef(DMMotorInstance *motor, float target_abs, float *cmd);
#endif // !DMMOTOR
