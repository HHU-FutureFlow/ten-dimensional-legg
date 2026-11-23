#include "chassis.h"
#include "chassis_actuators.h"
#include "chassis_jump.h"
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
#include "grf_observer.h"
#include "step_collision.h"
#include "chassis_friction.h"
#include "chassis_utils.h"
#include "chassis_step_leg.h"
#include "chassis_recovery.h"
#include "stm32f4xx_hal.h"
#include <math.h>


static DMMotorInstance *r_front_8009_motor,*r_behind_8009_motor;
static DMMotorInstance *l_front_8009_motor, *l_behind_8009_motor;
static DJIMotorInstance *l_wheel_motor, *r_wheel_motor;

/* 控制管线拍内计划(腿长增量/摆角参考速度): 恢复状态机(chassis_recovery.c)与主流程共享写入 */
volatile LegFrictionPlan_s plan_l = {0}, plan_r = {0};

/* cmd层指令订阅: chassis_cmd 话题,robot_cmd 200Hz发布 */
static Subscriber_t *chassis_cmd_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
#define RC_DRIVE_ACTIVE_VX 0.02f /* m/s, |vx|低于此视为松杆→定点 */
#define STEP_COLLISION_WINDOW_MS 15000u /* SW1 单次检测窗口，修改源码后重新编译 */

/* ===== 模型一致 yaw（2026-10-31 剪刀腿根因修复）=====
 * model_lagrange.py 的偏航状态 ψ = [Δ轮位移 + l·(θ_l−θ_r)]/(2Rl)，是"差动里程
 * 计偏航"（原地搓地旋转模型不可见）。固件此前喂 IMU 世界航向，语义分裂：
 * LQR 以为在修"腿别剪开"，实际被要求锁世界航向 → 无限剪刀循环。
 * 固件轮编码器值含 R·θ 项（s_code = x_wheel + R·θ，见 lqr_v2/design.py），展开：
 *   ψ  = (r_x − l_x)/(2Rl) + (R+l)/(2Rl)·(θ_l − θ_r)
 *   ψ̇ = (r_dx − l_dx)/(2Rl) + (R+l)/(2Rl)·(dθ_l − dθ_r)
 * 静止时不漂（无积分无陀螺），代价：站立不再锁世界航向，比赛需要时另加 IMU 外环。 */
#define TRACK_HALF_WIDTH  (0.24f)                 /* 半轮距 m,= params.py R_L */
#define WHEEL_RADIUS      (0.06f)                 /* 驱动轮半径 m,= params.py R_W */
#define YAW_COEF_WHEEL    (1.0f / (2.0f * TRACK_HALF_WIDTH))                            /* 2.083, 与腿长无关(design.py Cq 推导) */
/* YAW_COEF_LEG 已删除: 腿角项系数 (R+l)/(2R_L) 含腿长, 改为 ChassisTask 内
 * 用 rec_l_rise 现算联动, 防止改站立腿长时忘同步偏航模型(2026-11-05)。 */

/* ===== 腿长刚度 + Roll 补偿（2026-11-01 加强,原为宏 800/100/100/10）=====
 * 加强依据: 右倾1.3°的静态偏移 = 腿长下垂差(6.5N/800≈8mm→1.0°) + roll补偿过弱
 * (KP=100 时 1.3° 仅出力 2.3N)。KP 翻倍 + roll KP 4 倍后残余预计 0.2~0.3°。
 * volatile 支持 ep 在线整定; 腿抖→Kp 减回 1200 或 Kv 单独加到 250;
 * 低频晃动→roll_kp 降到 250。 */
volatile float leg_kp_pos = 1600.0f;    /* N/m, 腿长位置刚度 */
volatile float leg_kv_vel = 200.0f;     /* N/(m/s), 腿长轴向阻尼 */
volatile float roll_kp    = 400.0f;     /* N/rad, roll 补偿刚度 */
volatile float roll_kd    = 20.0f;      /* N/(rad/s), roll 补偿阻尼 */

/* ===== 恢复状态机/翻倒保护参数与实现见 chassis_recovery.c/h =====
 * 主流程引用其共享量: prone_rec_state / rec2_ramp / cur_l,r / rec_tht_l,r /
 * lift_anchor, lift_top / rec6_anchor / prot_armed / prot_enable 及 rec 前缀参数。 */

/* ===== 遥控三挡腿长(2026-11-15) =====
 * SW2 三挡由 cmd 层解析下发(leg_length_target: 0.15/0.25/0.35, 0=不使用→恒 rec_l_rise),
 * 此处斜坡跟踪(leg_switch_rate); 态5 起立斜坡顶点仍固定 rec_l_rise(起立位形只用
 * 验证过的 0.25), 双腿到顶沿重锚 ramp 防阶跃, 之后平衡/行驶/锁定的维持目标由挡位
 * 接管。态1~4/6/7 恢复流程不接。LQR K 带按实测腿长自动切换(0.15~0.40 全覆盖)。 */
volatile float leg_len_cmd_ramp = 0.25f;   /* m, 挡位腿长斜坡当前值(ep 可读) */
volatile float leg_switch_rate  = 0.0002f; /* m/tick = 5cm/s @1kHz, 挡位切换斜率 */
/* 挡位斜坡本拍增量: 挡位块每拍置 0 后按斜坡计算, 态5 到顶沿接管时读(recovery extern) */
float switch_length_step = 0.0f;
volatile float dbg_len_l = 0.25f, dbg_len_r = 0.25f; /* ep 读: 状态机当前腿长目标 */

/* 腿长速度估计状态实例（类型与估计器见 chassis_utils.h） */
static LegVelState_s l_legvel, r_legvel;
#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_LEG_VEL_HOLD
static LegVelState_s l_held_legvel, r_held_legvel;
#endif

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

    chassis_dbg.leg_pos[0] = r_front_8009_motor->measure.total_position;
    chassis_dbg.leg_pos[1] = r_behind_8009_motor->measure.total_position;
    chassis_dbg.leg_pos[2] = -l_front_8009_motor->measure.total_position;
    chassis_dbg.leg_pos[3] = -l_behind_8009_motor->measure.total_position;

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

void ChassisActuatorsInit(ChassisActuators_s *motors)
{
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
    motors->r_front_8009_motor = DMMotorInit(&r_config);

    r_config.can_init_config.tx_id = 3;
    r_config.can_init_config.rx_id = 3;
    motors->r_behind_8009_motor = DMMotorInit(&r_config);

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
    motors->l_front_8009_motor = DMMotorInit(&l_config);

    l_config.can_init_config.tx_id = 4;
    l_config.can_init_config.rx_id = 4;
    motors->l_behind_8009_motor = DMMotorInit(&l_config);

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

    motors->l_wheel_motor = PowerControlInit(&chassis_motor_config);


    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    motors->r_wheel_motor = PowerControlInit(&chassis_motor_config);
}

void ChassisInit()
{
    chassis_IMU_data = INS_Init();
    GRF_Init(); /* GRF 地面反作用力观测器（手册第九章 T1） */

    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    ChassisActuators_s motors;
    ChassisActuatorsInit(&motors);
    r_front_8009_motor = motors.r_front_8009_motor;
    r_behind_8009_motor = motors.r_behind_8009_motor;
    l_front_8009_motor = motors.l_front_8009_motor;
    l_behind_8009_motor = motors.l_behind_8009_motor;
    l_wheel_motor = motors.l_wheel_motor;
    r_wheel_motor = motors.r_wheel_motor;
}

static uint8_t ChassisProtectedAirMask(uint8_t step_leg_mask)
{
    /* 动作侧已有自己的轮控门和腿长参考，GRF 只管理仍由正常 LQR 控制的侧。 */
    const uint8_t normal_lqr_wheels = (prone_rec_state == PRS_DONE)
        ? (GRF_AIR_BOTH & ~step_leg_mask)
        : 0;
    return GRF_GetAirMask() & normal_lqr_wheels;
}

#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_THETA_LEVEL
/* 双腿受力拉开摆角差时，按支撑力和腿长增加差模纠偏刚度。 */
volatile float theta_lvl_k = 15.0f;  /* N·m/rad */
volatile float theta_lvl_db = 0.01f; /* rad */
volatile float theta_lvl_max = 15.0f; /* N·m */
volatile float theta_lvl_scl;
volatile float theta_lvl_dth;
volatile float theta_lvl_dbg;

static void ChassisLevelLegs(uint8_t active, four_control_data *control)
{
    theta_lvl_dbg = 0.0f;
    theta_lvl_dth = ten_balance_observe.l_theta - ten_balance_observe.r_theta;
    const float leg_force = 0.5f * (grf_dbg.F_leg_l + grf_dbg.F_leg_r);
    const float length = 0.5f * (ten_balance_observe.l_leg_length +
                                 ten_balance_observe.r_leg_length);
    /* 估计失效时回到站立增益，避免上一有效腿力放大当前摆矩。 */
    theta_lvl_scl = 1.0f;
    if (grf_dbg.valid_mask == GRF_AIR_BOTH)
        theta_lvl_scl = fminf(8.0f, fmaxf(1.0f, leg_force * length / (50.0f * 0.25f)));
    if (active)
    {
        const float error = LQR_PositionError(theta_lvl_dth, theta_lvl_db);
        const float torque = -theta_lvl_k * theta_lvl_scl * error;
        theta_lvl_dbg = fminf(theta_lvl_max, fmaxf(-theta_lvl_max, torque));
        control->l_Tp += theta_lvl_dbg;
        control->r_Tp -= theta_lvl_dbg;
    }
}
#endif

/* ===== 文件级任务状态（原 ChassisTask 内 static, 提为文件级供分区函数共用; 原名不变）===== */
static uint32_t last_step_request;
static uint32_t step_window_start_ms;
static uint8_t  step_window_active;
#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_POSITION_HOLD
static float position_integral;
#endif
static uint8_t zero_force_last = 0;
static uint8_t last_wheel_block_mask = 0; /* 两侧恢复轮控前重锚里程参考 */
static uint8_t drive_last = 0; /* 行驶模式锁存,零力恢复时置0强制重新锚定 */
static uint8_t standing_last = 0; /* 站立测试模式锁存,进入沿锚定 yaw */
static uint8_t lock_last = 0;     /* 锁定模式锁存,进入沿冻结 s/yaw 目标 */

/* ===== 零力急停沿处理 =====
 * 进入沿: 停电机 + 清各模块锁存/锚定; 恢复沿: 使能电机 + 清模式锁存。
 * 返回 1 = 当前拍处于零力(调用方任务早退, 调试量已清零)。 */
static uint8_t ChassisZeroForceEdge(void)
{
    uint8_t zero_force = (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE);
    if (zero_force != zero_force_last)
    {
        if (zero_force)
        {
            DMMotorStop(r_front_8009_motor);
            DMMotorStop(r_behind_8009_motor);
            DMMotorStop(l_front_8009_motor);
            DMMotorStop(l_behind_8009_motor);
            DJIMotorStop(l_wheel_motor);
            DJIMotorStop(r_wheel_motor);
            GRF_ResetAirDwell(); /* 急停早退，前后样本不能拼成一次连续驻留 */
            step_window_active = 0;
            StepCollision_Reset();
            StepLegMotion_Reset();
#if WHEELLEG_JUMP_ENABLE
            ChassisJump_Reset();
#endif
#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_POSITION_HOLD
            position_integral = 0.0f;
#endif
            last_wheel_block_mask = 0;
            Recovery_NotifyZeroForceEnter();
        }
        else
        {
            DMMotorEnable(r_front_8009_motor);
            DMMotorEnable(r_behind_8009_motor);
            DMMotorEnable(l_front_8009_motor);
            DMMotorEnable(l_behind_8009_motor);
            DJIMotorEnable(l_wheel_motor);
            DJIMotorEnable(r_wheel_motor);
            /* 急停期间车体可能被挪动,强制下一拍重新锚定/冻结目标:
             * 三个模式锁存全清 —— 零力期任务早退, lock/standing 锁存被冻结在
             * 急停前的值, 不清则恢复后进入沿不触发, s/yaw 目标仍是挪车前旧值
             * (drive_last=0 只救得到行驶模式)。 */
            drive_last = 0;
            standing_last = 0;
            lock_last = 0;
            Recovery_NotifyZeroForceExit();
        }
        zero_force_last = zero_force;
    }
    if (zero_force)
    {
        for (unsigned i = 0; i < 4; ++i)
            chassis_dbg.joint_T_base[i] = chassis_dbg.fric_ff[i] =
                chassis_dbg.joint_T_cmd[i] = chassis_dbg.joint_ref_speed[i] = 0.0f;
        for (unsigned i = 0; i < 2; ++i)
            chassis_dbg.wheel_T_base[i] = chassis_dbg.wheel_ff[i] =
                chassis_dbg.Tw_cmd[i] = chassis_dbg.wheel_ref_speed[i] = 0.0f;
        return 1;
    }
    return 0;
}

/* ===== 挡位斜坡 + 模型一致 yaw 重构 + 遥控目标生成 =====
 * 挡位: cmd 腿长目标斜坡跟踪(轮离地暂停换挡);
 * yaw: 用里程计+腿角差重构差动里程计偏航模型(见文件头部说明);
 * 目标: 站立测试全零基线 / 行驶控速+航向积分 / 锁定进入沿冻结位姿;
 * 卷绕保护: yaw_target 与实测 ψ 取同分支防 2π 跳变。 */
static void ChassisCommandTargets(uint8_t protected_air_at_start,
                                  uint8_t *drive_out, float *turn_speed_out)
{
    /* 遥控挡位腿长斜坡跟踪(2026-11-15): 目标<0.1=未配置(保持 rec_l_rise, 兼容
     * NONE 编译/未接遥控), 其余 clamp [0.15,0.40](K 带全覆盖上界, 高于机械底)。
     * 任一轮离地时暂停更换挡位目标，承载恢复后由现有腿长斜坡继续跟挡。 */
    const float previous_switch_length = leg_len_cmd_ramp;
    switch_length_step = 0.0f; /* 文件级(态5 到顶沿读): 每拍置 0 语义同原局部变量 */
    if (!protected_air_at_start)
    {
        float leg_len_cmd = chassis_cmd_recv.leg_length_target;
        if (leg_len_cmd < 0.1f)
            leg_len_cmd_ramp = rec_l_rise;
        else
        {
            LIMIT_MIN_MAX(leg_len_cmd, 0.15f, 0.40f);
            if (leg_len_cmd_ramp < leg_len_cmd) { leg_len_cmd_ramp += leg_switch_rate; if (leg_len_cmd_ramp > leg_len_cmd) leg_len_cmd_ramp = leg_len_cmd; }
            else if (leg_len_cmd_ramp > leg_len_cmd) { leg_len_cmd_ramp -= leg_switch_rate; if (leg_len_cmd_ramp < leg_len_cmd) leg_len_cmd_ramp = leg_len_cmd; }
            switch_length_step = leg_len_cmd_ramp - previous_switch_length;
        }
    }
    /* 变腿长时 yaw 模型系数跟随当前生效目标(上一拍值, 慢变斜坡无妨)联动,
     * 防 2026-11-05"改站立腿长忘同步偏航模型"同坑。 */
    float yaw_coef_leg = (WHEEL_RADIUS + ten_targets.l_leg_length_target) / (2.0f * TRACK_HALF_WIDTH);
    ten_balance_observe.yaw = YAW_COEF_WHEEL * (r_wheel.r_x - l_wheel.l_x)
                            + yaw_coef_leg * (ten_balance_observe.l_theta - ten_balance_observe.r_theta);
    ten_balance_observe.dyaw = YAW_COEF_WHEEL * (r_wheel.r_dx - l_wheel.l_dx)
                             + yaw_coef_leg * (ten_balance_observe.l_dtheta - ten_balance_observe.r_dtheta);
    ChassisDbgLqr(&ten_balance_observe);

    /* 轮使能沿重锚定 s/yaw 已并入态5 进入沿(直接写目标, 不再走请求标志)。 */

    /* ===== 遥控目标生成(cmd层意图 → LQR目标) =====
     * 行驶(CHASSIS_WHEELLEG_DRIVE): vx>阈值→控速(s_target跟随s), 松杆→定点(s_target冻结);
     *                               wz 积分为 yaw_target,松杆冻结→航向锁定。
     * 锁定(CHASSIS_NO_FOLLOW等): 进入瞬间冻结当前 s/yaw 为目标 → 原地定住。
     * 上电首拍锁定即冻结,此时 s≈0,yaw≈0,与站立调试的硬编码基线等价。
     * 恢复流程态1~4/态6: 本块目标随后被状态机覆盖(模式锁存照常更新); 态5 起本块
     * 接管(θ 收零/重锚完成前状态机仍短暂写)。腿长目标全程由状态机置位。 */
    uint8_t drive = (chassis_cmd_recv.chassis_mode == CHASSIS_WHEELLEG_DRIVE);
    float turn_speed = 0.0f;
    if (chassis_cmd_recv.chassis_mode == CHASSIS_STANDING_TEST)
    {
        /* 站立调试基线: 除 yaw 外全零(腿长0.25m=rec_l_rise已在上方设置)。
         * yaw_target 进入沿锚定当前 ψ 后冻结——里程计 ψ 会随推车/弧线漂移累计
         * 差动轮里程,若硬编码 0 会产生常值 yaw 误差 → 常值差动 Tp → 单腿被压死。
         * drive_last 置0,退出本模式后按进入沿重新锚定/冻结。 */
        if (!standing_last)
            ten_targets.yaw_target = ten_balance_observe.yaw;
        ten_targets.s_target        = 0.0f;
        ten_targets.ds_target       = 0.0f;
        ten_targets.dyaw_target     = 0.0f;
        ten_targets.l_theta_target  = 0.0f;
        ten_targets.l_dtheta_target = 0.0f;
        ten_targets.r_theta_target  = 0.0f;
        ten_targets.r_dtheta_target = 0.0f;
        ten_targets.phi_target      = 0.0f;
        ten_targets.dphi_target     = 0.0f;
        drive_last = 0;
    }
    else
    {
    if (drive && !drive_last) /* 行驶进入沿(含急停恢复后): 目标锚定当前位姿 */
    {
        ten_targets.s_target   = ten_balance_observe.s;
        ten_targets.yaw_target = ten_balance_observe.yaw;
    }
    if (drive)
    {
        if (!protected_air_at_start && fabsf(chassis_cmd_recv.vx) > RC_DRIVE_ACTIVE_VX)
        {
            ten_targets.ds_target = chassis_cmd_recv.vx;
            ten_targets.s_target  = ten_balance_observe.s;
        }
        else
        {
            ten_targets.ds_target = 0.0f;
        }
        if (!protected_air_at_start)
        {
            turn_speed = chassis_cmd_recv.wz;
            ten_targets.yaw_target += turn_speed * CHASSIS_TICK_S;
        }
        ten_targets.dyaw_target = 0.0f;
    }
        else
        {
            ten_targets.ds_target = 0.0f;
            ten_targets.dyaw_target = 0.0f;
            /* 锁定进入沿: 冻结当前位姿为目标。
             * 注意不能用 !drive_last 判沿——锁定时 drive 恒 0,drive_last 恒 0,
             * 该条件每拍都真,目标会每拍跟随实测(位置环失效→持续后溜/缓转)。
             * 必须用独立的 lock_last 锁存,真·一次性进入沿。 */
            if (!lock_last)
            {
                ten_targets.s_target   = ten_balance_observe.s;
                ten_targets.yaw_target = ten_balance_observe.yaw;
            }
        }
        drive_last = drive;
    } /* end: 非站立测试模式 */
    standing_last = (chassis_cmd_recv.chassis_mode == CHASSIS_STANDING_TEST);
    lock_last = !drive && !standing_last;

    /* yaw_target 卷绕保护：目标始终与实测 ψ 取同一分支，LQR 的 x[2] 限在 ±π 内，
     * 防止 wz 长时间积分与分支卷绕差 2π 时 Tp 猛跳。 */
    float yaw_ref_err = ten_targets.yaw_target - ten_balance_observe.yaw;
    yaw_ref_err -= 2.0f * PI * roundf(yaw_ref_err / (2.0f * PI));
    ten_targets.yaw_target = ten_balance_observe.yaw + yaw_ref_err;

    *drive_out = drive;
    *turn_speed_out = turn_speed;
}

/* ===== 输出覆盖（2026-11-09 重做, 2026-11-14 增态6）=====
 * PD 段不经 LQR（手册第九章: 自救期强制不经 LQR/VMC 平衡通道）:
 * 态1~4/态6: Tp = rec_pd PD(cur-θ_body 连续目标), Tw 硬清零, 腿长力仍走腿长环。
 * 态7  : Tp = prot PD(追赶式目标), Tw 硬清零。
 * 态5  : LQR 正常输出乘 rec2_ramp 0->1 渐入(斜坡补 v2 缺的 Tp 钳位),
 *        腿长环/roll 补偿不乘(腿长目标经斜坡无跳变)。
 * LQR_calc 无内部状态, 覆盖安全。 */
static void ChassisApplyRecoveryOverride(float l_theta_body, float r_theta_body)
{
    if ((prone_rec_state == PRS_FLIP && prot_armed) ||
        (prone_rec_state >= PRS_EXTEND && prone_rec_state <= PRS_PUSH) ||
        prone_rec_state == PRS_QUICK)
    {
        float l_tb, r_tb, kp, kd, tp_max;
        if (prone_rec_state == PRS_FLIP)
        {   /* 翻身: 追赶式目标(态7 内已写 ten_targets), prot 组增益 */
            l_tb = ten_targets.l_theta_target;
            r_tb = ten_targets.r_theta_target;
            kp = prot_kp_theta; kd = prot_kd_theta; tp_max = prot_tp_max;
        }
        else
        {   /* 伸腿保持/对齐/蹬地/折叠快转: 连续目标角, rec 组增益 */
            l_tb = cur_l;
            r_tb = cur_r;
            kp = rec_pd_kp; kd = rec_pd_kd; tp_max = rec_pd_tp_max;
        }
        /* 阻尼项用机体系角速度(l_dtheta+Gyro[0], balance.c 里 l_dtheta 已减 Gyro[0]):
         * 位置项 θ_body 是体系量, 阻尼必须同坐标系 —— 顶升时 pitch 快速变化,
         * 世界系阻尼会引入额外力矩分量(外部审查实锤)。
         * 位置误差必须 wrap: 目标 cur/追赶目标是连续累计角(可超 ±π), θ_body 是
         * atan2 主值(±π 折返) —— 裸相减在动腿过顶(对齐翻越/折叠蹬地必经)后误差
         * 跳 2π: PD 钳位饱和 + 追逐目标在折返空间符号反转 → 腿持续飞转直到段
         * 超时(设计规范本就要求 wrap(目标−实测), 此前实现遗漏)。 */
        float lt4 = kp * rec_wrap_pi(l_tb - l_theta_body)
                    - kd * (ten_balance_observe.l_dtheta + chassis_IMU_data->Gyro[0]);
        float rt4 = kp * rec_wrap_pi(r_tb - r_theta_body)
                    - kd * (ten_balance_observe.r_dtheta + chassis_IMU_data->Gyro[0]);
        LIMIT_MIN_MAX(lt4, -tp_max, tp_max);
        LIMIT_MIN_MAX(rt4, -tp_max, tp_max);
        four_control_datas.l_Tp = lt4;
        four_control_datas.r_Tp = rt4;
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
    }
    else if (prone_rec_state == PRS_LQR_RAMP)
    {   /* 交接混合: PD 支撑力随 ramp 退出、LQR 输出随 ramp 进入 —— 力连续不塌。
         * 15:3x 实测教训: 纯 ramp(Tp*=乘子)会在交接瞬间撤掉撑住重力矩的 PD 力,
         * LQR 又从 0 起步 → 机体塌。PD 用 cur/end(停在 θ_end) 与态3 同组增益;
         * 误差同样必须 wrap —— 过顶对齐后 cur 带 2π 累计偏移, 裸相减会让
         * "PD 支撑力"变成满钳位飞转力(见上方覆盖块注释)。 */
        float lt4 = rec_pd_kp * rec_wrap_pi(cur_l - l_theta_body)
                    - rec_pd_kd * (ten_balance_observe.l_dtheta + chassis_IMU_data->Gyro[0]);
        float rt4 = rec_pd_kp * rec_wrap_pi(cur_r - r_theta_body)
                    - rec_pd_kd * (ten_balance_observe.r_dtheta + chassis_IMU_data->Gyro[0]);
        LIMIT_MIN_MAX(lt4, -rec_pd_tp_max, rec_pd_tp_max);
        LIMIT_MIN_MAX(rt4, -rec_pd_tp_max, rec_pd_tp_max);
        four_control_datas.l_Tp = lt4 * (1.0f - rec2_ramp)
                                  + four_control_datas.l_Tp * rec2_ramp;
        four_control_datas.r_Tp = rt4 * (1.0f - rec2_ramp)
                                  + four_control_datas.r_Tp * rec2_ramp;
        four_control_datas.l_Tw *= rec2_ramp;
        four_control_datas.r_Tw *= rec2_ramp;
    }
}

/* ===== 电机输出: 关节摩擦前馈叠加 + 轮速合成 ===== */
static void ChassisMotorOutput(uint8_t out_gate, const ten_balance_target *control_targets,
                               uint8_t drive, float turn_speed,
                               uint8_t wheel_block_l, uint8_t wheel_block_r)
{
    /* 基础力矩来自反馈控制，摩擦前馈在电机轴独立叠加。 */
    float joint_speed[4] = {0};
    if (!out_gate)
    {
        JointReferenceSpeed(plan_r, control_targets->r_leg_length_target,
                            R_LEG_A, R_LEG_B, 1.0f, &joint_speed[0]);
        JointReferenceSpeed(plan_l, control_targets->l_leg_length_target,
                            L_LEG_A, L_LEG_B, -1.0f, &joint_speed[2]);
    }

    DMMotorInstance *const joint_motor[4] = {
        r_front_8009_motor, r_behind_8009_motor, l_front_8009_motor, l_behind_8009_motor,
    };
    const float joint_base[4] = {
        r_leg_motor_torques.r_Tf, r_leg_motor_torques.r_Tb,
        l_leg_motor_torques.l_Tf, l_leg_motor_torques.l_Tb,
    };
    for (unsigned i = 0; i < 4; ++i)
    {
        chassis_dbg.joint_ref_speed[i] = joint_speed[i];
        const float command = MotorTorqueWithFriction(joint_base[i], joint_speed[i],
                                                      joint_friction_nm[i], FRICTION_TANH_W0,
                                                      DM_T_MAX, out_gate, &chassis_dbg.joint_T_base[i],
                                                      &chassis_dbg.fric_ff[i]);
        chassis_dbg.joint_T_cmd[i] = command;
        DMMotorSetCtrlMode(joint_motor[i], DM_CTRL_TORQUE);
        DMMotorSetRef(joint_motor[i], command);
    }

    /* 左转为正，右轮参考速度较大。左右轮同用前进为正的坐标合成后限幅。 */
    const uint8_t wheel_gate = (prone_rec_state <= PRS_PUSH ||
                                prone_rec_state == PRS_FLIP ||
                                prone_rec_state == PRS_QUICK);
    const uint8_t wheel_blocked[2] = {wheel_gate || wheel_block_l, wheel_gate || wheel_block_r};
    float wheel_speed[2] = {0};
    if (drive && !wheel_gate)
    {
        const float turn = TRACK_HALF_WIDTH * turn_speed;
        const float scale = prone_rec_state == PRS_LQR_RAMP ? rec2_ramp : 1.0f;
        wheel_speed[0] = scale * (ten_targets.ds_target - turn);
        wheel_speed[1] = scale * (ten_targets.ds_target + turn);
    }

    DJIMotorInstance *const wheel_motor[2] = {l_wheel_motor, r_wheel_motor};
    const float wheel_base[2] = {four_control_datas.l_Tw, four_control_datas.r_Tw};
    const float wheel_direction[2] = {l_wheel_direction, 1.0f};
    for (unsigned i = 0; i < 2; ++i)
    {
        chassis_dbg.wheel_ref_speed[i] = wheel_blocked[i] || !isfinite(wheel_speed[i]) ? 0.0f : wheel_speed[i];
        const float command = MotorTorqueWithFriction(wheel_base[i], wheel_speed[i],
                                                      wheel_friction_nm[i], WHEEL_FRICTION_TANH_V0,
                                                      WHEEL_T_MAX, wheel_blocked[i], &chassis_dbg.wheel_T_base[i],
                                                      &chassis_dbg.wheel_ff[i]);
        chassis_dbg.Tw_cmd[i] = command;
        DJIMotorSetRef(wheel_motor[i], wheel_direction[i] * WHEEL_T_PER_RAW * command);
    }
}

void ChassisTask()
{
    ChassisDbgUpdate();
    /* 使能帧只在 DMMotorInit/DMMotorEnable 里发一次。原来在这里每 5ms 发两帧
     * DM_CMD_MOTOR_MODE，既占 CAN 带宽又打断 MIT 帧时序，已移除。 */
    SetPowerLimit(1000000.0f);   /* @Todo 接裁判系统后换成真实底盘功率 */

    /* ===== 接收 cmd 层指令 + 零力急停路径 =====
     * 边沿触发: Stop/Enable 只置标志,电机输出任务持续按标志发零/正常帧。 */
    SubGetMessage(chassis_cmd_sub, (void *)&chassis_cmd_recv);
    /* 急停、自起期间的请求也要消费，禁止恢复正常后重放旧下拨。 */
    const uint8_t new_step_request =
        chassis_cmd_recv.step_window_request != last_step_request;
    last_step_request = chassis_cmd_recv.step_window_request;
    if (ChassisZeroForceEdge())
        return;

    /* 恢复状态机共享计划量(文件级, 见文件头部): 每拍清零后各段写入 */
    plan_l = plan_r = (LegFrictionPlan_s){0};
    const float prev_len_l = ten_targets.l_leg_length_target;
    const float prev_len_r = ten_targets.r_leg_length_target;

    /* 用多圈展开后的连续位置；β 的整圈歧义在 *_Alpha_beta_calculate 里归一化 */
    float r_theta_f=r_front_8009_motor->measure.total_position;
    float r_theta_b=r_behind_8009_motor->measure.total_position;
    float r_speed_f=r_front_8009_motor->measure.velocity;
    float r_speed_b=r_behind_8009_motor->measure.velocity;
    float r_wheel_angle=r_wheel_motor->measure.total_angle;
    float r_wheel_w=r_wheel_motor->measure.speed_aps;


    float l_theta_f= -l_front_8009_motor->measure.total_position;
    float l_theta_b=-l_behind_8009_motor->measure.total_position;
    float l_speed_f=-l_front_8009_motor->measure.velocity;
    float l_speed_b=-l_behind_8009_motor->measure.velocity;
    float l_wheel_angle= -l_wheel_motor->measure.total_angle;
    float l_wheel_w= -l_wheel_motor->measure.speed_aps;

    r_Alpha_beta_calculate(r_theta_f,r_theta_b,r_speed_f,r_speed_b,&r_leg);
    l_Alpha_beta_calculate(l_theta_f,l_theta_b,l_speed_f,l_speed_b,&l_leg);
    r_wheel_control(r_wheel_angle,r_wheel_w,&r_wheel);
    l_wheel_control(l_wheel_angle,l_wheel_w,&l_wheel);
    BalanceObserve(&ten_balance_observe);

    /* GRF 观测器（T1）: 反馈力矩反推腿力 + IMU 竖直加速度,输出 ep 可读 grf_dbg */
    GRF_Update(r_front_8009_motor->measure.torque, r_behind_8009_motor->measure.torque,
               l_front_8009_motor->measure.torque, l_behind_8009_motor->measure.torque,
               0.001f);

    const uint8_t protected_air_at_start = ChassisProtectedAirMask(StepLegMotion_ActiveMask());

    /* ===== T3 机体系摆角 θ_body（2026-11-05, 函数级作用域）=====
     * balance.c:96 里 l_theta = θ_body − Pitch·rad, 加回来即得"腿相对机体竖直"
     * 的角 = 纯编码器运动学量, 与 IMU 姿态无关, 翻倒/欧拉角折返时依然有效
     * （数学恒等抵消, 非近似）。T3 监控、态4/5、输出闸、dbg 填充共用。 */
    float pitch_rad = chassis_IMU_data->Pitch * 0.01745329252f;
    float roll_rad  = chassis_IMU_data->Roll * 0.01745329252f;  /* S0 判定/dF_roll 共用 */
    float l_theta_body = ten_balance_observe.l_theta + pitch_rad;
    float r_theta_body = ten_balance_observe.r_theta + pitch_rad;

    /* v2 FOLD/DONE 状态机已删除(2026-11-09): 低腿长+大φ误差下 DONE 切换 Tp 无钳位猛甩,
     * 由下方 prone_rec_state 统一状态机替代(态5 LQR 斜坡接管补钳位)。
     * 腿长目标改由各态自写: 态1~4 PD 段按段目标, 态5/6 恒 rec_l_rise。 */

    /* ===== 挡位斜坡 + 模型一致 yaw 重构 + 遥控目标生成 =====
     * 实现(含注释依据)见上方 ChassisCommandTargets; drive/turn_speed 供
     * 台阶开窗判定与轮输出合成共用。 */
    uint8_t drive = 0;
    float turn_speed = 0.0f;
    ChassisCommandTargets(protected_air_at_start, &drive, &turn_speed);

    /* ===== 统一恢复状态机（S0 判定/态7 翻身/态1~5 恢复流程/T3 监控, 实现见 chassis_recovery.c）=====
     * 覆盖上面所有目标生成; 返回 1 = 本拍状态段切换(供下方清恢复段遗留前馈)。
     * 快照语义: S0 判定当拍的切换不计入, 态7/T3 触发的切换计入(与原实现一致)。 */
    const uint8_t rec_stage_changed = Recovery_Update(pitch_rad, roll_rad,
                                                      l_theta_body, r_theta_body,
                                                      chassis_cmd_recv.chassis_mode);

    /* ===== 态1~5: 统一恢复流程与 T3 监控均已移入 chassis_recovery.c ===== */

    /* S5 还包含起立收尾，等升腿、重锚和摆角收零完成后才接收开窗请求。 */
    const uint8_t step_window_allowed =
        prone_rec_state == PRS_DONE && lift_top && rec6_anchor &&
        fabsf(rec_tht_l) <= 0.01f && fabsf(rec_tht_r) <= 0.01f &&
        chassis_cmd_recv.chassis_mode == CHASSIS_WHEELLEG_DRIVE;
    const uint32_t now_ms = HAL_GetTick();

    if (!step_window_allowed ||
        (step_window_active &&
         now_ms - step_window_start_ms >= STEP_COLLISION_WINDOW_MS))
    {
        step_window_active = 0;
    }

    if (new_step_request && step_window_allowed && !step_window_active)
    {
        StepCollision_Reset();
        step_window_start_ms = now_ms;
        step_window_active = 1;
    }

    const StepCollisionSample_s collision_sample[2] =
    {
        {l_wheel.l_dx, ten_balance_observe.l_theta},
        {r_wheel.r_dx, ten_balance_observe.r_theta},
    };
    const uint8_t forward_drive = chassis_cmd_recv.vx > RC_DRIVE_ACTIVE_VX;
    const uint8_t previous_hits = step_collision_dbg.hit_mask;
    StepCollision_Update(step_window_active, forward_drive, GRF_GetAirMask(), collision_sample);
    const uint8_t new_hits = step_collision_dbg.hit_mask & ~previous_hits;

    /* 恢复阶段切换可能重锚控制目标，移除上一个阶段的前馈。 */
    if (rec_stage_changed)
        plan_l = plan_r = (LegFrictionPlan_s){0};

    LegFrictionPlan_s step_plan_l = {0}, step_plan_r = {0};
    /* 窗口只限制新触发。已开始的动作独立收尾，急停和恢复状态优先。 */
    if (prone_rec_state == PRS_DONE)
    {
        step_plan_l = StepLegMotion_Update(&step_leg_motion[0], new_hits & GRF_AIR_LEFT,
                                          l_theta_body, ten_balance_observe.l_leg_length);
        step_plan_r = StepLegMotion_Update(&step_leg_motion[1], new_hits & GRF_AIR_RIGHT,
                                          r_theta_body, ten_balance_observe.r_leg_length);
    }
    else
    {
        StepLegMotion_Reset();
    }

    if (Recovery_StepTakeover())
    {
        step_window_active = 0;
        StepLegMotion_Reset();
    }

    /* 按本拍最终的控制归属，避免动作开始或结束当拍沿用旧门控。 */
    const uint8_t step_leg_mask = StepLegMotion_ActiveMask();
    const uint8_t protected_air_mask = ChassisProtectedAirMask(step_leg_mask);
    if (protected_air_mask & GRF_AIR_LEFT)
    {
        ten_targets.l_leg_length_target = prev_len_l;
        plan_l.length_step = 0.0f;
    }
    if (protected_air_mask & GRF_AIR_RIGHT)
    {
        ten_targets.r_leg_length_target = prev_len_r;
        plan_r.length_step = 0.0f;
    }

    const uint8_t wheel_block_mask = step_leg_mask | protected_air_mask;
    const uint8_t optional_balance = (WHEELLEG_BALANCE_POSITION_HOLD ||
                                      WHEELLEG_BALANCE_LEG_VEL_HOLD ||
                                      WHEELLEG_BALANCE_THETA_LEVEL) &&
                                      prone_rec_state == PRS_DONE && lift_top &&
                                      !wheel_block_mask;
    LQR_FeedbackOptions_s feedback = {
        .position_deadband = WHEELLEG_BALANCE_POSITION_HOLD && optional_balance ? 0.020f : 0.0f,
        .wheel_theta_scale = 1.0f,
        .hip_pitch_scale = 1.0f,
    };
#if WHEELLEG_JUMP_ENABLE
    ChassisJumpInput_s jump_input = {
        .length = {ten_balance_observe.l_leg_length, ten_balance_observe.r_leg_length},
        /* 与原跳跃状态机一致，判相位使用上一拍滤波速度。 */
        .velocity = {l_held_legvel.filt, r_held_legvel.filt},
        .stand_length = rec_l_rise,
        .return_length = lift_top ? leg_len_cmd_ramp : rec_l_rise,
        .airborne = GRF_Airborne(),
        .can_start = lift_top && rec6_anchor &&
                     fabsf(rec_tht_l) <= 0.01f && fabsf(rec_tht_r) <= 0.01f,
    };
    ChassisJumpReference_s jump_ref = {0};
    if (prone_rec_state == PRS_DONE)
    {
        ChassisJump_Update((chassis_jump_cmd_e)chassis_cmd_recv.jump_command,
                            &jump_input, &jump_ref);
        if (jump_ref.restart_lift)
            lift_anchor = lift_top = 0;

        if (jump_ref.length_target > 0.0f)
        {
            /* 跳跃的收腿与落地按相位给参考，左右离地位只决定各轮的输出。 */
            ten_targets.l_leg_length_target = jump_ref.length_target;
            ten_targets.r_leg_length_target = jump_ref.length_target;
            plan_l.length_step = plan_r.length_step = jump_ref.length_step;
        }
        feedback.wheel_theta_scale = jump_ref.wheel_theta_scale;
        feedback.hip_pitch_scale = jump_ref.hip_pitch_scale;
        if (jump_ref.active)
            feedback.position_deadband = 0.020f;
    }
    else
    {
        ChassisJump_Reset();
    }
#endif

    const uint8_t wheel_block_l = wheel_block_mask & GRF_AIR_LEFT;
    const uint8_t wheel_block_r = wheel_block_mask & GRF_AIR_RIGHT;
    if (last_wheel_block_mask && !wheel_block_mask && prone_rec_state == PRS_DONE)
    {
        /* 恢复双轮里程反馈前重锚空转产生的位移和航向误差。 */
        ten_targets.s_target = ten_balance_observe.s;
        ten_targets.yaw_target = ten_balance_observe.yaw;
    }
    last_wheel_block_mask = wheel_block_mask;
#if WHEELLEG_JUMP_ENABLE
    ChassisJump_Reanchor(&ten_balance_observe, &ten_targets);
#endif

    /* 动作目标仅作用于本拍，退出试验后普通平衡不会接到遗留的后摆目标。 */
    ten_balance_target step_targets;
    const ten_balance_target *control_targets = &ten_targets;
    uint8_t step_ref_mask = 0;
    if (step_leg_mask)
    {
        step_targets = ten_targets;
        if ((step_leg_mask & GRF_AIR_LEFT) && step_leg_motion[0].length_target > 0.0f)
        {
            step_ref_mask |= GRF_AIR_LEFT;
            step_targets.l_leg_length_target = step_leg_motion[0].length_target;
            step_targets.l_theta_target = step_leg_motion[0].theta_body_target - pitch_rad;
            step_targets.l_dtheta_target = -chassis_IMU_data->Gyro[0];
            plan_l = step_plan_l;
        }
        if ((step_leg_mask & GRF_AIR_RIGHT) && step_leg_motion[1].length_target > 0.0f)
        {
            step_ref_mask |= GRF_AIR_RIGHT;
            step_targets.r_leg_length_target = step_leg_motion[1].length_target;
            step_targets.r_theta_target = step_leg_motion[1].theta_body_target - pitch_rad;
            step_targets.r_dtheta_target = -chassis_IMU_data->Gyro[0];
            plan_r = step_plan_r;
        }
        control_targets = &step_targets;
    }
    dbg_len_l = control_targets->l_leg_length_target;
    dbg_len_r = control_targets->r_leg_length_target;

    LQR_calc(&ten_balance_observe, &ten_targets, control_targets,
             wheel_block_mask, step_ref_mask, &feedback, &four_control_datas);
    chassis_dbg.lqr_u[0] = four_control_datas.l_Tw;
    chassis_dbg.lqr_u[1] = four_control_datas.r_Tw;
    chassis_dbg.lqr_u[2] = four_control_datas.l_Tp;
    chassis_dbg.lqr_u[3] = four_control_datas.r_Tp;
#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_POSITION_HOLD
    if (feedback.position_deadband > 0.0f && prone_rec_state == PRS_DONE &&
        !step_leg_mask && !wheel_block_mask && !drive)
    {
        /* 和 LQR 共用死区，退出位置保持时清掉旧积分。 */
        const float error = LQR_PositionError(ten_balance_observe.s - ten_targets.s_target,
                                              feedback.position_deadband);
        position_integral += 2.0f * error * 0.001f;
        LIMIT_MIN_MAX(position_integral, -1.0f, 1.0f);
        four_control_datas.l_Tw += position_integral;
        four_control_datas.r_Tw += position_integral;
    }
    else
    {
        position_integral = 0.0f;
    }
#endif

    /* ===== 输出覆盖: PD 段不经 LQR / 态5 斜坡混合（实现见上方 ChassisApplyRecoveryOverride）===== */
    ChassisApplyRecoveryOverride(l_theta_body, r_theta_body);

    /* 虚拟腿长力 = 重力前馈 + 腿长 PID；VMC 内部再按修正后的 J^T 映射。 */
    float l_dot = LegVelEst(ten_balance_observe.l_leg_length, &l_legvel, 0);
    float r_dot = LegVelEst(ten_balance_observe.r_leg_length, &r_legvel, 0);
    const uint8_t use_held_velocity = WHEELLEG_BALANCE_LEG_VEL_HOLD && optional_balance;
#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_LEG_VEL_HOLD
    const float l_held_dot = LegVelEst(ten_balance_observe.l_leg_length, &l_held_legvel, 1);
    const float r_held_dot = LegVelEst(ten_balance_observe.r_leg_length, &r_held_legvel, 1);
    if (use_held_velocity)
    {
        l_dot = l_held_dot;
        r_dot = r_held_dot;
    }
#endif
    const float normal_kv = use_held_velocity ? JUMP_LEG_KV_VEL : leg_kv_vel;
    const float l_length_error = control_targets->l_leg_length_target - ten_balance_observe.l_leg_length;
    const float r_length_error = control_targets->r_leg_length_target - ten_balance_observe.r_leg_length;
    float l_pid_out = leg_kp_pos * l_length_error - normal_kv * l_dot;
    float r_pid_out = leg_kp_pos * r_length_error - normal_kv * r_dot;
    /* 腿长误差门限幅(南航金城 stand_PID MaxOut 思路,防恢复段目标跳变弹车):
     * |目标-实测| 超 rec_push_band 期间 PID 分量钳 ±rec_push_max,
     * 误差进带(正常平衡/行驶,误差毫米级)恢复全量 ±LEG_PID_OUT_MAX。 */
    float l_pid_max = (fabsf(l_length_error) > rec_push_band) ? rec_push_max : LEG_PID_OUT_MAX;
    float r_pid_max = (fabsf(r_length_error) > rec_push_band) ? rec_push_max : LEG_PID_OUT_MAX;
    LIMIT_MIN_MAX(l_pid_out, -l_pid_max, l_pid_max);
    LIMIT_MIN_MAX(r_pid_out, -r_pid_max, r_pid_max);
    /* Roll 补偿（手册 §8.2）：右倾(roll<0)时右腿加力、左腿减力。
     * Roll 单位度 -> rad；Gyro[1] 为 roll 角速度(rad/s)。
     * PD 段(态1~4)与态7 倒地阶段 roll 无意义,补偿关闭;态5 起恢复。 */
    uint8_t rec_pd_stage = (prone_rec_state >= PRS_EXTEND && prone_rec_state <= PRS_PUSH)
                           || prone_rec_state == PRS_FLIP
                           || prone_rec_state == PRS_QUICK;
    float dF_roll = rec_pd_stage ? 0.0f
                               : ROLL_COMP_DIR * (roll_kp * (-roll_rad) - roll_kd * chassis_IMU_data->Gyro[1]);
    /* 重力前馈分阶段: S1(腿长整理)置零 —— 前腿承压/左腿摩擦位形要钳位全额出力,
     * +8N 伸腿向前馈会反向抵消收腿力(实测净拉力仅 22N);
     * S2/S3 用小前馈(倒地/顶升期腿不承全重,见 rec_gravity_ff 注释);
     * 态4 斜坡期随 rec2_ramp 从小前馈渐入 50N —— 直接跳变会在 S3->S4 切换当拍
     * 叠加 42N/腿 伸腿阶跃(外部审查抓出的交接跳变), 态5 起全量。 */
    float grav_ff;
    if (prone_rec_state == PRS_EXTEND)
        grav_ff = 0.0f;
    else if (rec_pd_stage)
        grav_ff = rec_gravity_ff;
    else if (prone_rec_state == PRS_LQR_RAMP)
        grav_ff = rec_gravity_ff + (LEG_GRAVITY_FF - rec_gravity_ff) * rec2_ramp;
    else
        grav_ff = LEG_GRAVITY_FF;
    if (prone_rec_state == PRS_LQR_RAMP)
        dF_roll *= rec2_ramp;   /* roll 补偿同随斜坡渐入 */
    /* 摆动侧用现有悬空腿长控制，保留另一侧的承重前馈。 */
    if (wheel_block_mask)
        dF_roll = 0.0f;
    four_control_datas.l_Tleg = (wheel_block_l ? 0.0f : grav_ff) + l_pid_out - dF_roll;
    four_control_datas.r_Tleg = (wheel_block_r ? 0.0f : grav_ff) + r_pid_out + dF_roll;
#if WHEELLEG_JUMP_ENABLE
    jump_input.velocity[0] = l_held_dot;
    jump_input.velocity[1] = r_held_dot;
    if (jump_ref.active)
    {
        const float limits[2] = {l_pid_max, r_pid_max};
        float jump_force[2];
        ChassisJump_LegForce(&jump_input, jump_ref.length_target, limits, jump_force);
        /* 接触登阶摆动侧仍取该侧动作参考与轴向力。 */
        if (!(step_leg_mask & GRF_AIR_LEFT))
            four_control_datas.l_Tleg = jump_force[0] - dF_roll;
        if (!(step_leg_mask & GRF_AIR_RIGHT))
            four_control_datas.r_Tleg = jump_force[1] + dF_roll;
    }
#endif
    chassis_dbg.Fleg_cmd[0] = four_control_datas.l_Tleg;
    chassis_dbg.Fleg_cmd[1] = four_control_datas.r_Tleg;
#if WHEELLEG_JUMP_ENABLE
    chassis_dbg.Ldot[0] = jump_ref.active ? jump_input.velocity[0] : l_dot;
    chassis_dbg.Ldot[1] = jump_ref.active ? jump_input.velocity[1] : r_dot;
#else
    chassis_dbg.Ldot[0] = l_dot;
    chassis_dbg.Ldot[1] = r_dot;
#endif
    chassis_dbg.Roll = chassis_IMU_data->Roll;
    chassis_dbg.dFroll = dF_roll;
    chassis_dbg.theta_body[0] = l_theta_body;
    chassis_dbg.theta_body[1] = r_theta_body;
    chassis_dbg.prot = (float)(prot_enable | (prot_armed << 1));

    /* 输出闸: ①观测无效期(上电头几拍读数全 0, 腿长环却会以 30N/腿推腿,
     * 折叠位形下该力把腿/轮推进入瞬态, 恢复锚定就锚到运动半途
     * (2026-11-03 实测教训), 判定完成(prone_rec_state 离开 0)前电机不出力,
     * 腿由机械摩擦保持;
     * ②态7 未武装期(翻倒检测触发后, 等操作手切档确认, 全零脱力无害等待)。
     * 标志供关节摩擦前馈共用 —— 脱力等待期前馈也必须零(否则关节有速度时
     * 仍输出 ~0.9N·m, 外部审查抓出的零力语义泄漏)。 */
    uint8_t out_gate = (prone_rec_state == PRS_DETECT ||
                        (prone_rec_state == PRS_FLIP && !prot_armed));
    if (out_gate)
    {
        four_control_datas.l_Tp = 0.0f;
        four_control_datas.r_Tp = 0.0f;
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
        four_control_datas.l_Tleg = 0.0f;
        four_control_datas.r_Tleg = 0.0f;
    }

#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_THETA_LEVEL
    uint8_t level_legs = WHEELLEG_BALANCE_THETA_LEVEL && optional_balance;
#if WHEELLEG_JUMP_ENABLE
    if (jump_ref.active)
        level_legs = jump_ref.level_legs;
#endif
    ChassisLevelLegs(!out_gate && prone_rec_state == PRS_DONE && !step_leg_mask && level_legs,
                     &four_control_datas);
#endif
    VMC_Calc(&four_control_datas, &r_leg_motor_torques, &r_leg);
    l_VMC_Calc(&four_control_datas, &l_leg_motor_torques, &l_leg);

    /* ===== 电机输出: 关节摩擦前馈叠加 + 轮速合成（实现见上方 ChassisMotorOutput）===== */
    ChassisMotorOutput(out_gate, control_targets, drive, turn_speed,
                       wheel_block_l, wheel_block_r);

}
