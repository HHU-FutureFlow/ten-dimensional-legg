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

/* 轮力矩 -> C620 电流码 的换算系数。
 * 由实测反推：819.2(码/A) / kt_out，kt_out = 0.0156(转子) * 15.8(自改减速箱) = 0.2469 N·m/A
 * 注意这是"输出端力矩"，不是电流。 */
#define WHEEL_T_PER_RAW   (3318.833f)
/* 3508 + C620 的电流码上限是 ±16384(=±20A)，换算回来就是 ±4.94 N·m。
 * dji_motor.c 里 LIMIT_MIN_MAX(set,-16384,16384) 只对 GM6020 生效，
 * M3508 是直接 (int16_t) 强转、没有限幅，所以必须在这里先钳住，
 * 否则 3318.833*8 = 26550 会溢出成负数、电流乱跳。 */
#define WHEEL_T_MAX       (4.90f)

/* 每条腿的重力前馈（2026-10-30 ep 实测修正）：
 * 站立稳态 Fleg_cmd ≈ 45~57 N/腿 → 整车 ≈ 10 kg，每腿承重 ≈ 50 N。
 * 旧值 29 N（按 ~6 kg 估）偏小，P 环把缺的 ~20 N 顶成 ~3 cm 稳态缩短
 * （实测两腿稳在 0.27 m 而非目标 0.30，偏离 LQR 设计工作点）。
 * 与腿长 PID 输出相加后作为 VMC 的虚拟腿长力 F。 */
#define LEG_GRAVITY_FF    (50.0f)
#define LEG_PID_OUT_MAX   (150.0f) /* 腿长反馈分量的全量限幅，单位 N。 */

/* 腿长 P-P 级联（手册 §8.1）：F = FF + Kp*(Ldes-L) - Kv*Ldot
 * volatile 全局: 支持 ep 在线整定,无需重编译烧录。
 * Kp_pos: 位置刚度 N/m；Kv_vel: 轴向阻尼 N/(m/s)。
 * 2026-11-01 加强: 800/100 -> 1600/200,压腿长下垂差(6.5N 扰动 8mm->4mm)。
 * LEG_VEL_LPF(Ldot 低通系数)已迁 chassis_utils.h(随 LegVelEst)。 */
extern volatile float leg_kp_pos;   /* N/m, 原 800 */
extern volatile float leg_kv_vel;   /* N/(m/s), 原 100 */

/* Roll 补偿（手册 §8.2，符号按本机 IMU 实测定）：
 * ΔF = ROLL_KP*(-roll) - ROLL_KD*gyro[1]，右腿 += ΔF，左腿 -= ΔF。
 * 实测 roll=-3.9°(右倾) 时 ΔF>0 -> 右腿加力扶正。
 * 若上车后 roll 发散，把 ROLL_COMP_DIR 翻成 -1.0f。
 * 2026-11-01 加强: 100/10 -> 400/20,1.3° 时补偿力 2.3N->9N。
 * volatile 全局: 支持 ep 在线整定。 */
#define ROLL_COMP_DIR     (1.0f)
extern volatile float roll_kp;      /* N/rad, 原 100 */
extern volatile float roll_kd;      /* N/(rad/s), 原 10 */

/* 陀螺Z零偏（2026-11-23 恢复 27ea99e,IMU yaw 回归主源）:
 * 零力态两段式自动标定——前 2s 固定窗口均值,之后 EMA(τ≈10s) 跟踪温漂;
 * |Gyro[2]|<0.05 rad/s 门限拒绝搬车污染。从 yaw/dyaw 中扣除,ep 可读写。
 * 静止漂移实测 -2.5°/min(QEKF 强制 GyroBias[2]=0,Z 轴不可观测)。 */
extern volatile float gyro_z_bias;        /* rad/s,预期 -0.0007 量级 */
extern volatile uint8_t gyro_z_calib_stage; /* 0=未开始 1=快采中 2=EMA跟踪 */

/* 电机反馈量调试映射:在 EP 全局符号里搜 chassis_dbg 即可
 * leg_*  : [0]r_front [1]r_behind [2]l_front [3]l_behind
 * wheel_*: [0]l_wheel [1]r_wheel
 * 位置单位 rad,速度单位 rad/s,符号与 ChassisTask 内的控制量一致
 *
 * 2026-11-23 瘦身: 源量 EP 直读可达的镜像字段已删, 按字段路径读取:
 *   LQR 输入状态  -> ten_balance_observe（原 lqr_x）
 *   LQR 输出力矩  -> four_control_datas.l_Tw/r_Tw/l_Tp/r_Tp（原 lqr_u）
 *   虚拟腿长力    -> four_control_datas.l_Tleg/r_Tleg（原 Fleg_cmd）
 *   IMU roll      -> chassis_IMU_data->Roll（原 Roll 字段）
 *   状态机段/武装 -> prone_rec_state / prot_armed / prot_enable（原 prot 位包）
 * 排查"往一个方向跑不停"时看：
 *   phi 变负 + four_control_datas.l_Tw 变正  -> 俯仰环方向对
 *   车体回正了但 l_Tw 不换号                 -> s/ds 环没起作用
 *   l_Tw 在 ±4.9 之间几毫秒来回跳            -> 延迟振荡（控制频率不够） */
typedef struct
{
    float leg_pos[4];
    float leg_vel[4];
    float wheel_pos[2];
    float wheel_vel[2];

    float Ldot[2];        /* [0]左 [1]右，腿长速度估计（m/s），低通后 */
    float joint_T_cmd[4];
    float fric_ff[4];     /* [0]r_f [1]r_b [2]l_f [3]l_b，限幅后实际生效的关节摩擦前馈(N·m) */
    float wheel_ff[2];    /* [0]左轮 [1]右轮，限幅后实际生效的轮端摩擦前馈(N·m) */
    float Tw_cmd[2];
    float dFroll;         /* roll 补偿力差 ΔF（N），右腿为正 */
    /* ===== T3/T4 翻倒保护（2026-11-05, ep 观察）===== */
    float theta_body[2];  /* [0]左 [1]右, 机体系摆角 = l_theta + Pitch·rad（纯编码器量） */
    float joint_T_base[4];    /* VMC 后电机轴基础力矩，N·m */
    float wheel_T_base[2];    /* 轮端基础力矩，N·m，前进为正 */
    float joint_ref_speed[4]; /* 摩擦前馈使用的电机轴参考速度，rad/s */
    float wheel_ref_speed[2]; /* 摩擦前馈使用的轮缘参考速度，m/s */
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