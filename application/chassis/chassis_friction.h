#ifndef CHASSIS_FRICTION_H
#define CHASSIS_FRICTION_H

#include <stdint.h>

/* ===== 常量宏 ===== */
#define FRICTION_TANH_W0       (0.15f) /* rad/s, 电机轴, tanh 摩擦模型过渡速度 */
#define WHEEL_FRICTION_TANH_V0 (0.03f) /* m/s, 轮缘 */
#define CHASSIS_TICK_S         (0.001f) /* 底盘任务周期 s, 1kHz */

/* ===== 类型定义 ===== */
typedef struct
{
    float length_step; /* 本拍计划的腿长增量，m */
    float theta_speed; /* 本拍主动参考摆速，rad/s */
} LegFrictionPlan_s;

/* ===== 可调参数 =====
 * 2026-11-22 台架等效运动摩擦候选，N·m。带载运行待验证。
 * 关节顺序与 chassis_dbg 一致（右大腿、右小腿、左大腿、左小腿），轮为左、右。 */
extern const float joint_friction_nm[4];
extern const float wheel_friction_nm[2];

/* ===== 对外接口 ===== */

/* 虚拟腿长的逆解与台架一致。关节在机体系为 θ∓β/2，
 * 左腿按 VMC 输出约定换回电机轴坐标（motor_sign: 右腿 1.0f, 左腿 -1.0f）。
 * 重锚和静止拍的长度增量为零。 */
void JointReferenceSpeed(LegFrictionPlan_s plan, float length, float thigh, float shin,
                         float motor_sign, float speed[2]);

/* 力矩合成：基础反馈力矩 + tanh 摩擦前馈，统一限幅后返回。
 * blocked/非有限输入清零调试量并返回 0；base_dbg/ff_dbg 供 chassis_dbg 展示。 */
float MotorTorqueWithFriction(float base, float speed, float friction, float smoothing,
                              float limit, uint8_t blocked, volatile float *base_dbg,
                              volatile float *ff_dbg);

#endif // CHASSIS_FRICTION_H
