#include "chassis.h"
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
#include "stm32f4xx_hal.h"
#include <math.h>


static DMMotorInstance *r_front_8009_motor,*r_behind_8009_motor;
static DMMotorInstance *l_front_8009_motor, *l_behind_8009_motor;
static DJIMotorInstance *l_wheel_motor, *r_wheel_motor;

static PIDInstance l_theta_pid;
static PIDInstance r_theta_pid;

/* cmd层指令订阅: chassis_cmd 话题,robot_cmd 200Hz发布 */
static Subscriber_t *chassis_cmd_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
#define RC_DRIVE_ACTIVE_VX 0.02f /* m/s, |vx|低于此视为松杆→定点 */
#define STEP_COLLISION_WINDOW_MS 15000u /* SW1 单次检测窗口，修改源码后重新编译 */
#define STEP_LEG_LENGTH_TOL 0.01f /* m，先确认实测收短再回摆（UNTESTED） */
#define STEP_LEG_REAR_ALIGN_EPS 0.19f /* rad，2026-11-20 右侧停在距正后方 0.183 rad，原 0.10；新值 UNTESTED */
#define STEP_LEG_READY_ANGLE_TOL 0.12f /* rad，2026-11-20 右回摆误差 0.102，原 0.10；新值 UNTESTED */
#define STEP_LEG_READY_LEN_MIN 0.10f /* m，2026-11-20 回摆末端最低约 0.107；新值 UNTESTED */

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

/* ===== 关节库仑摩擦前馈（2026-10-30）=====
 * 实测左髋静摩擦 ≥2 N·m（右腿可随意掰动，左腿小腿需相当力气），死区让 LQR 的
 * 差动 Tp 全被右腿吸收（剪刀只走一条腿）。补偿加在各自关节、电机自身坐标：
 *   T_ff = T_c * tanh(ω / ω0)
 * tanh 平滑过零防 1kHz 抖振。T_c 为 volatile 全局，可在调试器在线调：
 * 左腿 0.5 起调（手测静摩擦大），右腿先 0（过补会变负阻尼、诱发极限环）。
 * 判据：左右腿手掰阻力对称；注意 ω=0 时补偿为 0，静止突破仍靠平衡自振。 */
volatile float l_joint_fric_t = 0.55f;   /* N·m/关节, 左腿 */
volatile float r_joint_fric_t = 0.35f;   /* N·m/关节, 右腿 */
/* 悬空专用关节摩擦前馈(2026-11-15): 悬空时载荷相关摩擦塌缩(差一个量级),
 * 沿用着地 Tc 必过补 → 负阻尼(前日提车摆腿发散)。
 * 默认 0 = 悬空跳过起安全基线, 后续从 0 起步在线标定:
 * 自激摆动 = 过补回退一半, 回正/摆动中途粘滞 = 欠补加量。态7 翻身不切这套
 * (该位形 GRF 未标定 air 易误判, 且翻身摆腿要摩擦前馈突破左髋静摩擦)。 */
volatile float l_joint_fric_t_air = 0.0f; /* N·m/关节, 左腿悬空 */
volatile float r_joint_fric_t_air = 0.0f; /* N·m/关节, 右腿悬空 */
#define FRICTION_TANH_W0  (0.15f)       /* rad/s, tanh 速度平滑尺度 */

/* ===== 轮端库仑摩擦前馈（2026-10-31）=====
 * 手感实测右轮毂电机阻尼大于左轮（自改减速箱装配差异），轮端摩擦同样吃掉
 * 平衡环的小 Tw 修正与推车时的滚动 -> 单腿摆动不对称的轮端成因。
 * 补偿按"前进为正"的轮速（r_wheel.r_dx / l_wheel.l_dx，m/s，左轮符号已翻正）
 * 加在 Tw 上、限幅之前；左轮 l_wheel_direction=-1 的坐标换算在其后，随之落位。
 * T_c volatile 在线调：右 0.10 / 左 0.05 起调（右轮阻尼大故给多），
 * 过大 = 轮速通道负阻尼 -> 平衡低频抖动，出现即减半。tanh(0)=0 静止无自发力。 */
volatile float r_wheel_fric_t = 0.15f;  /* N·m/轮, 右轮 */
volatile float l_wheel_fric_t = 0.05f;  /* N·m/轮, 左轮 */
#define WHEEL_FRICTION_TANH_V0 (0.03f)  /* m/s, tanh 速度平滑尺度(轮缘线速度) */

/* ===== 腿长刚度 + Roll 补偿（2026-11-01 加强,原为宏 800/100/100/10）=====
 * 加强依据: 右倾1.3°的静态偏移 = 腿长下垂差(6.5N/800≈8mm→1.0°) + roll补偿过弱
 * (KP=100 时 1.3° 仅出力 2.3N)。KP 翻倍 + roll KP 4 倍后残余预计 0.2~0.3°。
 * volatile 支持 ep 在线整定; 腿抖→Kp 减回 1200 或 Kv 单独加到 250;
 * 低频晃动→roll_kp 降到 250。 */
volatile float leg_kp_pos = 1600.0f;    /* N/m, 腿长位置刚度 */
volatile float leg_kv_vel = 200.0f;     /* N/(m/s), 腿长轴向阻尼 */
volatile float roll_kp    = 400.0f;     /* N/rad, roll 补偿刚度 */
volatile float roll_kd    = 20.0f;      /* N/(rad/s), roll 补偿阻尼 */

/* ===== 统一起立/自救状态机（2026-11-09 重构, 2026-11-14 低重心改版）=====
 * 流程(定稿): 态0 判定 -> 态1 双腿腿长斜坡整理到 0.25(摆角保持锚定)
 *   -> 态2 单向对齐: 规定方向=机体右侧看逆时针(θ 增), 以"逆时针转到髋正下方
 *      的路程"定锚——路程短者锚定, 路程长者沿正向转到锚角(例: 左腿在前右腿在后
 *      -> 左腿逆时针经上方转 180° 到右腿角)。转向只看体系角(θ_body), 机体滑动无关。
 *   -> 态3 蹬地+同步收腿: 双腿一起沿正向转向髋下后方(θ_end 轮心后置对准重心,
 *      见态3 块内), 轮心进入机体下半圆后腿长随体系角联动收短(拟合关系见态3 块内),
 *      腿角到 θ_end 时
 *      腿长同步到 rec_l_low —— 髋被顶起的高度全程不超过终点蹲姿, 低重心交接。
 *   -> 态4 腿角进阈值直接切 LQR(输出斜坡 0.4s, θ 目标锚定后恒定, 腿长保持蹲姿)
 *   -> 态5 起立+完成: 腿长目标以 rec_lift_rate 斜坡升回 rec_l_rise(闭环下起立),
 *      θ 目标收零, pitch 回正后重锚 s/yaw。
 *   -> 态6 折叠位形反向快转: 态0 判定"双腿轮心已在髋下方带内且腿长已收拢"
 *      (断电常停折叠态, 亦即已删 FOLD->STAND 起立机的工作位形)时, 不走伸腿
 *      顶起长路, 允许沿与规定方向相反(顺时针)的最短路快速转到 θ_end, 腿长
 *      保持 rec_l_low, 到位直接切态4 低位接管。规定方向(fmod 强制正向)对略过
 *      θ_end 的折叠角要绕近整圈, 反向只有 0.1rad 量级; 腿收拢时轮子在身下
 *      近地, 短转=轮沿地滚动, 无顶地/无鼻下穿插。
 *   运行中 |θ_body| 出窗 -> 态7 翻身(仰翻前置, 脱力+切档确认) -> 态1。
 * 设计原则: 每段一个目标硬怼到位, 无卡滞检测/无重试/无静定确认,
 *   仅保留速率斜坡(防弹车)+段超时(安全兜底)+出口驻留(防抖)。 */
#define PRS_DETECT   0   /* 上电位形判定 */
#define PRS_EXTEND   1   /* 腿长整理到站立腿长 */
#define PRS_ALIGN    2   /* 双腿单向对齐(逆时针路程短者锚) */
#define PRS_PUSH     3   /* 蹬地顶升+腿角联动收腿 */
#define PRS_LQR_RAMP 4   /* LQR 输出斜坡接管(蹲姿) */
#define PRS_DONE     5   /* 起立(腿长斜坡升回)+完成, 常规控制 */
#define PRS_QUICK    6   /* 折叠位形反向快转(低位直接切 LQR) */
#define PRS_FLIP     7   /* 翻身(仰翻前置) */
volatile uint8_t prone_rec_state = 0;
/* ---- 段速率/位形目标(核心可调) ---- */
volatile float rec_l_rise      = 0.25f;   /* 站立/行驶腿长 m(唯一真源, 偏航模型系数联动) */
volatile float rec_extend_rate = 0.0002f; /* 态1 腿长目标速率 m/tick = 20cm/s @1kHz */
volatile float rec_chase_align = 0.12f;    /* 态2 对齐追赶超前量 rad(PD恒力=kp*ahead, 0.12→6N·m) */
volatile float rec_chase_push  = 0.25f;    /* 态3 蹬地追赶超前量 rad(0.25→12.5N·m 顶升力) */
volatile float rec_theta_end   = -0.17f;   /* 态3/态6 终点体系角 rad(轮心后置: 轮触点对准重心, 实测重心在髋后≈2.2cm; 换腿长按 θ≈−asin(0.022/L) 重标定, 0.13 腿长实测定稿 −0.17) */
/* ---- 到位/静定判据 ---- */
volatile float rec_eps_len     = 0.03f;   /* 态1 腿长到位带 m */
volatile float rec_align_eps   = 0.10f;   /* 态2/3 摆角到位带 rad(连续目标-连续实测) */
volatile uint16_t rec_hold_ticks = 500;   /* 到位驻留确认 0.5s @1kHz */
/* ---- 摆角 PD(态1~4 覆盖段共用; 翻身态7 用 prot_* 那组) ---- */
volatile float rec_pd_kp     = 50.0f;
volatile float rec_pd_kd     = 2.0f;
volatile float rec_pd_tp_max = 30.0f;     /* N·m */
/* ---- 态1~3 统一超时 -> 清确认位回态7 脱力等切档 ---- */
volatile uint16_t rec_stage_timeout = 8000; /* 8s @1kHz */
/* ---- 态4 LQR 输出斜坡 ---- */
volatile float rec_lqr_ramp_t = 0.4f;     /* 斜坡时长 s */
volatile float rec2_ramp      = 0.0f;     /* ep 可读: 斜坡乘子 0~1 */
volatile float rec_done_zero_rate = 0.5f; /* 态5 腿角目标收零速率 rad/s */
/* 态4 进入沿锚定世界系 θ 目标=当前实测后【恒定】, S4 期间不收零:
 * 142335 CSV 实测: PD 未撤+轮未使能时收零会把腿拉过竖直顶点 -> 必然向后翻倒。
 * 态5(LQR 已接管+轮已使能)才由状态机以 rec_done_zero_rate 斜坡收零 ——
 * 普通遥控模式正常路径不写 θ 目标, 不收零会一直维持锚定非零角(歪着站)。 */
/* ---- 低重心交接/折叠快转（2026-11-14）----
 * 依据: 已删的 FOLD->STAND 起立机(git 0f00739)证明 (轮心在髋下方, L≈0.11)
 * 的低位位形可以稳定起立 —— 低位交接重心低、LQR 捕获带大。 */
volatile float rec_l_low       = 0.13f;    /* 蹲姿交接/收腿下限 m(台架折叠底 0.124~0.13, 不许低于机械底) */
volatile float rec_retract_rate= 0.0005f; /* 态3 联动收腿目标斜率 m/tick = 50cm/s @1kHz(压深入下半圆起步的阶跃) */
volatile float rec_lift_rate   = 0.0002f; /* 态5 起立腿长目标斜率 m/tick = 20cm/s @1kHz(旧STAND 实测值, "快而稳"调此) */
/* ===== 遥控三挡腿长(2026-11-15) =====
 * SW2 三挡由 cmd 层解析下发(leg_length_target: 0.15/0.25/0.35, 0=不使用→恒 rec_l_rise),
 * 此处斜坡跟踪(leg_switch_rate); 态5 起立斜坡顶点仍固定 rec_l_rise(起立位形只用
 * 验证过的 0.25), 双腿到顶沿重锚 ramp 防阶跃, 之后平衡/行驶/锁定的维持目标由挡位
 * 接管。态1~4/6/7 恢复流程不接。LQR K 带按实测腿长自动切换(0.15~0.40 全覆盖)。 */
volatile float leg_len_cmd_ramp = 0.25f;   /* m, 挡位腿长斜坡当前值(ep 可读) */
volatile float leg_switch_rate  = 0.0002f; /* m/tick = 5cm/s @1kHz, 挡位切换斜率 */
volatile float rec_fold_l_th   = 0.13f;    /* 态6 触发: 平均腿长下限 m(断电折叠态 ≈0.11~0.124) */
volatile float rec_fold_th     = 0.85f;   /* 态6 触发: 双腿 |θ_body| 上限 rad(11-14 零力实测折叠上电角 0.70/0.75, 留 0.1 裕量; 上界<π/2 保轮心在下半圆带内, 短转=轮沿地滚) */
volatile float dbg_len_l = 0.25f, dbg_len_r = 0.25f; /* ep 读: 状态机当前腿长目标 */
/* ---- 伸腿防弹(沿用 v2): 腿长误差超带期间 PID 分量钳 ±rec_push_max ---- */
volatile float rec_push_band = 0.02f;     /* 误差门限 m */
volatile float rec_push_max  = 60.0f;     /* 大误差期间 PID 分量限幅 N(60: 前腿承压+左腿摩擦+翻身段共用, 关节力矩约6-9N·m) */
/* ---- PD 段专用重力前馈: 倒地/顶升期腿不承机体全重(车头分担), 小前馈防平衡点抬高 ---- */
volatile float rec_gravity_ff = 0.0f;     /* N/腿 */
/* ---- ep 诊断: 连续目标角/连续实测角(态2/3 对齐进度, 不 wrap 的累计值) ---- */
volatile float dbg_goal_l = 0.0f, dbg_goal_r = 0.0f;
volatile float dbg_cont_l = 0.0f, dbg_cont_r = 0.0f;
/* 态7 确认/武装 + 状态机进入沿标志(文件级: 零力进入沿/恢复沿在 ChassisTask
 * 前段(:390/:408 附近)访问, 局部 static 声明点在其后不可见) */
static uint8_t prot_armed = 0;
static uint8_t prot_seen_zf = 0;
static uint8_t rec6_anchor = 0;       /* 通用进入沿标志(段内共享, 段出口清零) */
static uint8_t lift_anchor = 0;       /* 态5 起立斜坡锚定标志(文件级: 零力进入沿在前段访问) */
static uint8_t lift_top = 0;          /* 态5 起立双腿到顶标志(2026-11-15: 到顶沿重锚挡位斜坡, 与 lift_anchor 同批清零) */

/* ===== T3 翻倒保护 + 翻身段（2026-11-05 SPR 式, 2026-11-09 并入统一状态机为态7）=====
 * 判据与动作全部基于腿摆角 θ_body（纯编码器量, 见监控块注释）,
 * 不用欧拉角判翻倒（EKF Pitch=atan2/Roll=asinf 值域均 ±90°, 翻倒后折返）。
 * 流程: 运行中 |θ_body| 出窗 -> 脱力进态7 -> 操作手"切零力再切出"确认 ->
 *       态7 翻身(伸腿 0.40 造支点 + 追赶式恒速摆腿, SPR 同款) -> 态1 伸腿接统一流程。 */
volatile uint8_t  prot_enable     = 0;      /* 0=运行中监控不跑(默认,先配数) 1=出窗脱力+进态4 */
volatile float    prot_theta_th   = 1.3f;   /* |θ_body| 出窗阈值 rad（SPR 窗口外沿 ~1.2） */
volatile uint16_t prot_hold       = 200;    /* 出窗驻留 tick @1kHz */
volatile float    prot_l_max      = 0.40f;  /* 态4 腿长目标 m: 伸到最长造支点（SPR 同款） */
volatile float    prot_l_speed    = 0.0005f;/* 态4 伸腿速率 m/tick = 50cm/s */
volatile float    prot_step       = 0.3f;   /* 追赶式摆角超前量 rad（SPR 同款, 恒速摆+抗卡滞） */
volatile float    prot_step_far   = 0.6f;   /* 双腿差>prot_diff_th 时落后腿的步长（差速纠偏） */
volatile float    prot_diff_th    = 0.8f;   /* 双腿 θ_body 差超此值触发差速 rad */
volatile float    prot_dir_sign   = 1.0f;   /* 摆腿总方向: 实测方向反了整体改 -1 */
volatile float    prot_kp_theta   = 30.0f;  /* 态4/5 θ_body PD: 追赶式下 Tp≈kp*step 恒推 */
volatile float    prot_kd_theta   = 2.0f;   /* 阻尼防摆飞 */
volatile float    prot_tp_max     = 18.0f;  /* 态7 翻身 Tp 限幅 N·m（要顶得动机体） */
volatile float    prot_pitch_back = -1.0f;  /* Pitch(rad) 小于此值摆腿反向（SPR :443, ±90° 内可靠） */
volatile float    prot_pitch_exit = 0.3f;   /* 态4 出口: |Pitch|<此值 才算机身翻正 */
volatile float    prot_theta_win  = 1.2f;   /* 态7 出口: 双腿 |θ_body|<此值 才算回窗口 */
volatile uint16_t prot_timeout    = 8000;   /* 态7 超时 tick -> 回脱力等重新切档 */
volatile uint8_t  prot_force_arm  = 0;      /* ep 置 1 跳过切档直接武装（调试用, 武装后自动清零） */

/* 腿长速度估计状态（P-P 级联的阻尼项用） */
typedef struct
{
    float last_L;      /* 上次变化的腿长读数 */
    float dt_acc;      /* 距上次变化的累计时间 s */
    float raw;         /* 本拍差分原始值 */
    float filt;        /* 低通后的 Ldot */
    uint8_t started;
} LegVelState_s;
static LegVelState_s l_legvel, r_legvel;
#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_LEG_VEL_HOLD
static LegVelState_s l_held_legvel, r_held_legvel;
#endif

/* Ldot 估计：DM 反馈 500Hz，1kHz 循环里差分隔拍为 0，
 * 必须按"上次变化以来的累计时间"归一，否则均值少一半。 */
static float LegVelEst(float L, LegVelState_s *st, uint8_t hold_between_samples)
{
    if (!st->started)
    {
        st->last_L = L;
        st->started = 1;
        return 0.0f;
    }
    st->dt_acc += 0.001f;
    if (L != st->last_L)
    {
        st->raw = (L - st->last_L) / st->dt_acc;
        st->last_L = L;
        st->dt_acc = 0.0f;
    }
    else if (!hold_between_samples || st->dt_acc > 0.005f)
    {
        /* 跳跃的阻尼与 5 ms 保持配套，正常路径按开关选择估计方式。 */
        st->raw = 0.0f;
    }
    st->filt += LEG_VEL_LPF * (st->raw - st->filt);
    return st->filt;
}

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

void ChassisInit()
{
    chassis_IMU_data = INS_Init();
    GRF_Init(); /* GRF 地面反作用力观测器（手册第九章 T1） */

    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

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
    r_front_8009_motor = DMMotorInit(&r_config);

    r_config.can_init_config.tx_id = 3;
    r_config.can_init_config.rx_id = 3;
    r_behind_8009_motor = DMMotorInit(&r_config);

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
    l_front_8009_motor = DMMotorInit(&l_config);

    l_config.can_init_config.tx_id = 4;
    l_config.can_init_config.rx_id = 4;
    l_behind_8009_motor = DMMotorInit(&l_config);


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

    l_wheel_motor = PowerControlInit(&chassis_motor_config);



    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    r_wheel_motor = PowerControlInit(&chassis_motor_config);




    /* 腿长 P-P 级联（手册 §8.1）取代原单环 PID：
     * F = 重力前馈 + Kp_pos*(Ldes-L) - Kv_vel*Ldot
     * 正方向：F>0 = 伸腿（VMC.c 的 Tf=-gF, Tb=+gF 链路，实车已验证）。 */



    

    

}



/* 角度差 wrap 到 ±π（统一状态机连续角跟踪用, 2026-11-09） */
static float rec_wrap_pi(float a)
{
    return a - 2.0f * PI * roundf(a / (2.0f * PI));
}

typedef enum
{
    STEP_LEG_IDLE,
    STEP_LEG_BACKSWING,
    STEP_LEG_TUCK,
    STEP_LEG_RETURN,
    STEP_LEG_READY
} StepLegPhase_e;

typedef struct
{
    float length_target;     /* m */
    float theta_body_target; /* rad，机体系 */
    uint8_t phase;
} StepLegMotion_s;

/* EP 顺序 [0] 左、[1] 右。READY 保持短腿和准备摆角，等待双侧同拍接管。 */
volatile StepLegMotion_s step_leg_motion[2];

static void StepLegMotion_Reset(void)
{
    step_leg_motion[0] = (StepLegMotion_s){0};
    step_leg_motion[1] = (StepLegMotion_s){0};
}

static uint8_t StepLegMotion_ActiveMask(void)
{
    return ((step_leg_motion[0].phase != STEP_LEG_IDLE) ? GRF_AIR_LEFT : 0) |
           ((step_leg_motion[1].phase != STEP_LEG_IDLE) ? GRF_AIR_RIGHT : 0);
}

static uint8_t ChassisProtectedAirMask(uint8_t step_leg_mask)
{
    /* 动作侧已有自己的轮控门和腿长参考，GRF 只管理仍由正常 LQR 控制的侧。 */
    const uint8_t normal_lqr_wheels = (prone_rec_state == PRS_DONE)
        ? (GRF_AIR_BOTH & ~step_leg_mask)
        : 0;
    return GRF_GetAirMask() & normal_lqr_wheels;
}

static void StepLegMotion_Update(volatile StepLegMotion_s *leg, uint8_t hit,
                                 float theta_body, float measured_length)
{
    const float rear_angle = -0.5f * PI;
    if (leg->phase == STEP_LEG_IDLE)
    {
        if (!hit)
            return;
        leg->phase = STEP_LEG_BACKSWING;
    }
    /* 命中后先锁轮；等反馈有效才从实际腿长开始生成动作参考。 */
    if (!isfinite(theta_body) || !isfinite(measured_length) || measured_length <= 0.0f)
    {
        if (leg->phase == STEP_LEG_READY)
            leg->phase = STEP_LEG_RETURN;
        return;
    }
    if (leg->length_target <= 0.0f)
        leg->length_target = measured_length;

    const float rear_error = rec_wrap_pi(rear_angle - theta_body);
    const float low_error = rec_wrap_pi(rec_theta_end - theta_body);
    const uint8_t rear_reached = fabsf(rear_error) <= STEP_LEG_REAR_ALIGN_EPS;
    const uint8_t length_reached = fabsf(measured_length - rec_l_low) <= STEP_LEG_LENGTH_TOL;

    if (leg->phase == STEP_LEG_BACKSWING && rear_reached)
        leg->phase = STEP_LEG_TUCK;

    if (leg->phase == STEP_LEG_TUCK)
    {
        /* 锁住后摆角先收腿，长度目标到位且实际收短后才让轮胎向下回摆。 */
        const float length = leg->length_target;
        if (length > rec_l_low)
            leg->length_target = fmaxf(rec_l_low, length - rec_retract_rate);
        else
            leg->length_target = fminf(rec_l_low, length + rec_retract_rate);

        if (rear_reached && length_reached && leg->length_target == rec_l_low)
            leg->phase = STEP_LEG_RETURN;
    }

    /* 先到的一侧保持准备姿态，每拍复核，双侧交接只用同一拍的到位结果。 */
    if (leg->phase == STEP_LEG_RETURN || leg->phase == STEP_LEG_READY)
    {
        const uint8_t pose_ready =
            measured_length >= STEP_LEG_READY_LEN_MIN &&
            measured_length <= rec_l_low + STEP_LEG_LENGTH_TOL &&
            fabsf(low_error) <= STEP_LEG_READY_ANGLE_TOL;

        leg->phase = pose_ready ? STEP_LEG_READY : STEP_LEG_RETURN;
    }

    /* 复用自起的摆角超前量，限定大角度摆动的误差。回摆段腿长保持收短。 */
    float error = leg->phase <= STEP_LEG_TUCK ? rear_error : low_error;
    LIMIT_MIN_MAX(error, -rec_chase_push, rec_chase_push);
    leg->theta_body_target = theta_body + error;
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

void ChassisTask()
{
    ChassisDbgUpdate();
    /* 使能帧只在 DMMotorInit/DMMotorEnable 里发一次。原来在这里每 5ms 发两帧
     * DM_CMD_MOTOR_MODE，既占 CAN 带宽又打断 MIT 帧时序，已移除。 */
    SetPowerLimit(1000000.0f);   /* @Todo 接裁判系统后换成真实底盘功率 */

    /* ===== 接收 cmd 层指令 + 零力急停路径 =====
     * 边沿触发: Stop/Enable 只置标志,电机输出任务持续按标志发零/正常帧。 */
    SubGetMessage(chassis_cmd_sub, (void *)&chassis_cmd_recv);
    static uint32_t last_step_request;
    static uint32_t step_window_start_ms;
    static uint8_t step_window_active;
    /* 急停、自起期间的请求也要消费，禁止恢复正常后重放旧下拨。 */
    const uint8_t new_step_request =
        chassis_cmd_recv.step_window_request != last_step_request;
    last_step_request = chassis_cmd_recv.step_window_request;

#if WHEELLEG_JUMP_ENABLE || WHEELLEG_BALANCE_POSITION_HOLD
    static float position_integral;
#endif
    static uint8_t zero_force_last = 0;
    static uint8_t last_wheel_block_mask = 0; /* 两侧恢复轮控前重锚里程参考 */
    static uint8_t drive_last = 0; /* 行驶模式锁存,零力恢复时置0强制重新锚定 */
    static uint8_t standing_last = 0; /* 站立测试模式锁存,进入沿锚定 yaw */
    static uint8_t lock_last = 0;     /* 锁定模式锁存,进入沿冻结 s/yaw 目标 */
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
            /* S7 确认沿: 零力期间任务直接 return, 态7 块内的记录永远执行不到
             * (外部审查实锤"切零力再切出不可达"), 必须在进入沿记录。
             * 仅 S7 等确认时记录 —— 其余时机切零力不预支确认。 */
            if (prone_rec_state == PRS_FLIP)
                prot_seen_zf = 1;
            if (prone_rec_state == PRS_DONE)
            {
                lift_anchor = 0;  /* 零力期腿长可能被人为改动, 恢复后从实测重锚起立斜坡 */
                lift_top = 0;     /* 挡位接管沿同样重走(到顶沿重锚 ramp 防阶跃) */
            }
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
            /* 自起恢复段(态1~4/态6)被急停中断: 位形可能被人为改变, 回 S0 重新判定。
             * 站立态(S5)不回 —— 日常切零力急停再恢复, 无缝接回站立控制不引入断力。
             * S7 不回(切零力正是它的确认动作, seen_zf 留给同拍态7 块消费武装)。 */
            if ((prone_rec_state >= PRS_EXTEND && prone_rec_state <= PRS_LQR_RAMP)
                || prone_rec_state == PRS_QUICK)
            {
                prone_rec_state = PRS_DETECT;
                rec6_anchor = 0;
                prot_armed = 0;
            }
        }
        zero_force_last = zero_force;
    }
    if (zero_force)
        return; /* 停机标志已生效,本拍不再计算控制量 */

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
    static uint16_t prot_cnt = 0;
    /* prot_armed/prot_seen_zf 声明提为文件级(文件头部): 零力进入沿(:390 附近)
     * 要写 seen_zf, 零力恢复沿要清 armed —— 两处都在本声明点之前。 */

    /* v2 FOLD/DONE 状态机已删除(2026-11-09): 低腿长+大φ误差下 DONE 切换 Tp 无钳位猛甩,
     * 由下方 prone_rec_state 统一状态机替代(态5 LQR 斜坡接管补钳位)。
     * 腿长目标改由各态自写: 态1~4 PD 段按段目标, 态5/6 恒 rec_l_rise。 */

    /* 模型一致 yaw：用里程计+腿角差重构 model_lagrange.py 的 ψ（见文件头部说明）。
     * BalanceObserve 里的 IMU yaw/dyaw 在此被整体替换；IMU 仅保留 φ/roll 用途。
     * 腿角项系数 (R+l)/(2R_L) 随当前生效腿长目标联动(遥控三挡切换不丢同步)。 */
    /* 遥控挡位腿长斜坡跟踪(2026-11-15): 目标<0.1=未配置(保持 rec_l_rise, 兼容
     * NONE 编译/未接遥控), 其余 clamp [0.15,0.40](K 带全覆盖上界, 高于机械底)。
     * 任一轮离地时暂停更换挡位目标，承载恢复后由现有腿长斜坡继续跟挡。 */
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
    /* drive 在函数级作用域: 目标生成分支共用 */
    uint8_t drive = (chassis_cmd_recv.chassis_mode == CHASSIS_WHEELLEG_DRIVE);
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
            ten_targets.yaw_target += chassis_cmd_recv.wz * 0.001f; /* ChassisTask 1kHz */
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

    /* ===== 统一恢复状态机（2026-11-09, 覆盖上面所有目标生成）=====
     * 进入(态0 三路位形判定, 见下方判定处): 大 |pitch| 仰翻->态7;
     *   腿收拢或摆角出竖直带(含双腿伸直摊开瘫)->态1 自动流程; 轮着地直立->态6。 */
    static uint16_t prone_valid = 0;
    /* T3 机体系摆角 θ_body / prot_armed 等已在本函数开头(GRF_Update 后)计算,
     * 见该处注释; 此处不再重复赋值。 */
    if (prone_rec_state == PRS_DETECT)
    {
        float ll = ten_balance_observe.l_leg_length;
        float rl = ten_balance_observe.r_leg_length;
        /* 双腿观测都有效且连续 50ms 才判定: 单腿先就绪时平均腿长也能过 0.05,
         * 会把另一腿的垃圾值/瞬态半途锚成起始目标(2026-11-03 实测教训) */
        if (ll > 0.05f && rl > 0.05f)
        {
            if (++prone_valid >= 50)
            {
                prone_valid = 0; /* 判定完成即清: 再次进 S0(零力恢复沿)重新走 50ms 有效期驻留 */
                /* 位形四路判定(2026-11-09 三路, 2026-11-14 增折叠快转): 不能只看
                 * 腿长——双腿伸直摊开(L≈0.25)会被误判站立, LQR 全开趴地甩腿+
                 * 轮子狂转(实测事故)。
                 * 大 |pitch|/|roll|(仰翻/侧翻类, 欧拉角 ±90° 折返值域内可靠) -> 态7
                 *   脱力等切档(翻身需人确认);
                 * 双腿轮心已在髋下方带内且腿长已收拢(断电常停折叠态, 鼻撑/蹲姿
                 *   pitch 0~1.2 均可) -> 态6 反向快转, 低位直接切 LQR;
                 * 腿收拢/摆角出竖直带/pitch 明显倾斜/roll 明显侧倾 -> 态1 自动流程
                 *   (pitch 0.5~1.2 的"鼻撑地腿竖直"位形此前漏进 DONE -> LQR 大φ
                 *    误差猛甩; roll 无门则纯侧躺+腿恰好入带同样误判 DONE);
                 * 其余(轮着地直立) -> 态5 站立基线。
                 * 注: roll 侧翻在 ±90° 内读数可靠, 翻过头由 θ_body/L 兜底。 */
                float L_avg = 0.5f * (ll + rl);
                if (fabsf(pitch_rad) > 1.2f || fabsf(roll_rad) > 1.2f)
                {
                    prone_rec_state = PRS_FLIP;
                }
                else if (L_avg < rec_fold_l_th &&
                         fabsf(l_theta_body) < rec_fold_th &&
                         fabsf(r_theta_body) < rec_fold_th &&
                         fabsf(roll_rad) < 0.6f)
                {   /* 折叠低位位形: 轮心已在髋下方左右+腿长最小, 走态6 快转 */
                    prone_rec_state = PRS_QUICK;
                }
                else if (L_avg < 0.13f ||
                         fabsf(l_theta_body) > 0.6f ||
                         fabsf(r_theta_body) > 0.6f ||
                         fabsf(pitch_rad) > 0.5f ||
                         fabsf(roll_rad) > 0.6f)
                {
                    prone_rec_state = PRS_EXTEND;
                }
                else
                {
                    prone_rec_state = PRS_DONE;
                }
            }
        }
        else
        {
            prone_valid = 0;
        }
    }
    /* ===== 统一状态机共享状态（态7 翻身与态1~5 恢复流程共用; rec6_anchor
     * 已提为文件级——零力恢复沿在函数前段清它）===== */
    static uint16_t rec6_t = 0, rec6_hold = 0;
    static float    ext_l = 0.25f, ext_r = 0.25f;      /* 态1 腿长斜坡值 */
    static float    cont_l = 0.0f, cont_r = 0.0f;      /* 连续实测角(体系, 不 wrap) */
    static float    cur_l = 0.0f, cur_r = 0.0f;        /* 当前摆角目标(连续, 覆盖段 PD 用) */
    static float    end_l = 0.0f, end_r = 0.0f;        /* 当前段终点角 */
    static float    rec_tht_l = 0.0f, rec_tht_r = 0.0f;/* 态4 锚定/态5 收零的世界系 θ 目标 */
    static float    len_cmd_l = 0.25f, len_cmd_r = 0.25f;/* 态3 联动收腿目标(限斜率后) */
    static float    lift_l = 0.25f, lift_r = 0.25f;    /* 态5 起立腿长斜坡值 */

    /* ===== 态7: 翻倒过渡/翻身（2026-11-05 SPR 式, 2026-11-09 编号 4->7）=====
     * 进态即脱力（输出闸清零），等操作手"切零力再切出"的确认沿才武装
     * （检测→脱力无害，动作需人确认——11-03 教训的隔离层; prot_force_arm 供 ep 跳过）。
     * 武装后: 腿长匀速伸到 prot_l_max 造支点 + θ_body 追赶式目标恒速摆
     * （目标=实测∓step 每拍重设: 腿能动则恒速摆, 被压住则目标跟着实测走、永不失配）。
     * 方向按 Pitch 符号（prot_pitch_back 反向, 仅 ±90° 可靠区间）; 双腿差大时差速纠偏。
     * 到位（机身翻正+双腿回窗）驻留 -> 态1 伸腿接统一流程; 超时回脱力等重新切档。 */
    static uint8_t  tilt4_anchor = 0;
    static uint16_t tilt4_t = 0;
    static uint16_t tilt4_hold = 0;
    static float tilt4_l = 0.25f, tilt4_r = 0.25f;
    if (prone_rec_state == PRS_FLIP)
    {
        /* 启动闸门: seen_zf 已在零力进入沿记录(本块内的记录不可达, 零力期任务 return);
         * 切出(非零力)后到达本块 -> seen_zf || ep 强制 -> 武装 */
        if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
        {
            /* 零力期间理论上到不了这里(任务 return), 防御性保留 */
        }
        else if (prot_seen_zf || prot_force_arm)
        {
            prot_armed = 1;
            prot_seen_zf = 0;
            prot_force_arm = 0;
        }
        if (prot_armed)
        {
            if (!tilt4_anchor)
            {
                tilt4_l = ten_balance_observe.l_leg_length;
                tilt4_r = ten_balance_observe.r_leg_length;
                LIMIT_MIN_MAX(tilt4_l, 0.09f, 0.40f);
                LIMIT_MIN_MAX(tilt4_r, 0.09f, 0.40f);
                tilt4_anchor = 1;
                tilt4_t = 0;
                tilt4_hold = 0;
            }
            tilt4_t++;
            if (tilt4_l < prot_l_max) tilt4_l += prot_l_speed;
            if (tilt4_r < prot_l_max) tilt4_r += prot_l_speed;
            ten_targets.l_leg_length_target = tilt4_l;
            ten_targets.r_leg_length_target = tilt4_r;
            /* 追赶式摆角目标（世界系 θ 目标仅占位, LQR 输出随后被覆盖） */
            float dir = prot_dir_sign * ((pitch_rad < prot_pitch_back) ? 1.0f : -1.0f);
            float step_l = prot_step, step_r = prot_step;
            if (fabsf(l_theta_body - r_theta_body) > prot_diff_th)
            {   /* 差速: 摆角落后的腿步长加倍（SPR 固定右腿加倍, 这里按实测动态选） */
                if (fabsf(l_theta_body) > fabsf(r_theta_body)) step_r = prot_step_far;
                else                                           step_l = prot_step_far;
            }
            ten_targets.l_theta_target = l_theta_body + dir * step_l;
            ten_targets.r_theta_target = r_theta_body + dir * step_r;
            if (fabsf(pitch_rad) < prot_pitch_exit &&
                fabsf(l_theta_body) < prot_theta_win &&
                fabsf(r_theta_body) < prot_theta_win)
            {
                if (++tilt4_hold >= rec_hold_ticks)
                {
                    prone_rec_state = PRS_EXTEND;
                    tilt4_anchor = 0;
                }
            }
            else
            {
                tilt4_hold = 0;
            }
            if (tilt4_t > prot_timeout)
            {   /* 超时: 回脱力等重新切档, 防卡死顶死。
                 * seen_zf 必须一并清零, 否则残留的"已见零力"会让操作手
                 * 不做任何确认就自动再武装（闸门失效）。 */
                prot_armed = 0;
                prot_seen_zf = 0;
                tilt4_anchor = 0;
            }
        }
    }

    /* ===== 态1~5: 统一恢复流程（2026-11-09 极简版, 2026-11-14 低重心改版）=====
     * 态1 腿长整理: 双腿目标以 rec_extend_rate 斜坡到 rec_l_rise(0.25),
     *   摆角锚定保持; 出口=双腿长进带驻留。无卡滞检测(硬怼到位为止)。
     * 态2 单向对齐: 规定方向=机体右侧看逆时针(θ 增)。以"逆时针转到髋正下方
     *   (体系角 0)的路程"定锚——路程短者锚定, 路程长者沿正向转到锚角
     *   (例: 左前右后 -> 左腿经上方转 180° 到右腿角)。连续角 wrap 跟踪不丢圈。
     * 态3 蹬地+联动收腿: 两腿一起以追赶式(rec_chase_push 恒超前=恒顶升力矩)
     *   转向髋下后方(体系角 rec_theta_end, 轮心后置对准重心, 交接零冲), 轮心
     *   进入机体下半圆后腿长随体系角联动收短, 到位时同步到蹲姿 rec_l_low(见块内拟合)。
     *   腿角只看体系角, 机体/车头滑动无关。
     * 态4 LQR 斜坡: 腿角进阈值直接切——进入沿重锚定 s/yaw + 世界系 θ 目标=
     *   实测锚定后【恒定】(收零会跨顶翻倒, 142335 实锤); 输出=PD×(1−ramp)+LQR×ramp
     *   混合(纯 ramp 交接会撤掉撑重 PD 力导致塌, 15:3x 实锤)。腿长保持蹲姿。
     * 态5 起立+完成: 腿长目标以 rec_lift_rate 斜坡升回 rec_l_rise(闭环下起立),
     *   θ 目标由锚定值斜坡收零, 轮已使能; pitch 回正后重锚 s/yaw 消回程。
     * 设计原则: 每段一个目标硬怼到位, 无卡滞/重试/静定确认,
     *   仅保留速率斜坡(防弹车)+段超时(安全兜底)+出口驻留(防抖)。 */
    /* (共享 static 变量已声明在态7 块前, 此处直接使用) */

    if ((prone_rec_state >= PRS_EXTEND && prone_rec_state <= PRS_PUSH)
        || prone_rec_state == PRS_QUICK)
    {   /* 连续实测角跟踪: 差值 wrap 到 ±π 后累加, 对齐翻越 ±π 解算折返不丢圈 */
        cont_l += rec_wrap_pi(l_theta_body - cont_l);
        cont_r += rec_wrap_pi(r_theta_body - cont_r);
        dbg_cont_l = cont_l;
        dbg_cont_r = cont_r;
        rec6_t++;
    }

    if (prone_rec_state == PRS_EXTEND)
    {   /* 态1 腿长整理: 双腿目标以 rec_extend_rate 斜坡到 rec_l_rise,
         * 摆角目标锚定当前实测(保持不动, 收腿时不许腿乱摆)。
         * 出口: 双腿长进带驻留 -> 态2。无卡滞检测(硬怼到位为止)。 */
        if (!rec6_anchor)
        {
            cont_l = l_theta_body; cont_r = r_theta_body;
            cur_l = cont_l; cur_r = cont_r;
            end_l = cont_l; end_r = cont_r;
            /* 腿长斜坡起点锚定实测(防阶跃弹车), 每拍向目标推进 */
            ext_l = ten_balance_observe.l_leg_length;
            ext_r = ten_balance_observe.r_leg_length;
            LIMIT_MIN_MAX(ext_l, 0.09f, 0.40f);
            LIMIT_MIN_MAX(ext_r, 0.09f, 0.40f);
            rec6_anchor = 1;
            rec6_t = 0; rec6_hold = 0;
        }
        /* 腿长斜坡: 20cm/s 逼近目标(必要阶梯, 防弹车) */
        if (ext_l < rec_l_rise) ext_l += rec_extend_rate;
        if (ext_l > rec_l_rise) ext_l -= rec_extend_rate;
        if (ext_r < rec_l_rise) ext_r += rec_extend_rate;
        if (ext_r > rec_l_rise) ext_r -= rec_extend_rate;
        ten_targets.l_leg_length_target = ext_l;
        ten_targets.r_leg_length_target = ext_r;
        if (fabsf(ten_balance_observe.l_leg_length - rec_l_rise) < rec_eps_len &&
            fabsf(ten_balance_observe.r_leg_length - rec_l_rise) < rec_eps_len)
        {
            if (++rec6_hold >= rec_hold_ticks)
            {
                rec6_anchor = 0;
                prone_rec_state = PRS_ALIGN;
            }
        }
        else
        {
            rec6_hold = 0;
        }
        if (rec6_t > rec_stage_timeout)
        {
            rec6_anchor = 0;
            prone_rec_state = PRS_FLIP;
            prot_armed = 0;
            prot_seen_zf = 0;
        }
    }
    else if (prone_rec_state == PRS_ALIGN)
    {
        if (!rec6_anchor)
        {
            /* 对齐锚选择(2026-11-09): 规定方向=机体右侧看逆时针(θ 增)。
             * 以"逆时针转到髋正下方(体系角 0)的路程"定锚: 路程短者锚定,
             * 路程长者沿正向转到锚角。例: 左腿在前(+90°)右腿在后(−90°)
             * -> 左腿逆时针路程 270° 右腿 90° -> 锚右腿, 左腿经上方转 180°。 */
            float dist_l = fmodf(-cont_l, 2.0f * PI);
            float dist_r = fmodf(-cont_r, 2.0f * PI);
            if (dist_l < 0.0f) dist_l += 2.0f * PI;
            if (dist_r < 0.0f) dist_r += 2.0f * PI;
            float anchor = (dist_l <= dist_r) ? cont_l : cont_r;
            /* 动腿终点 = 自身连续角 + 正向路程(强制归 [0,2π), 恒逆时针不反向) */
            float dfl = fmodf(anchor - cont_l, 2.0f * PI);
            float dfr = fmodf(anchor - cont_r, 2.0f * PI);
            if (dfl < 0.0f) dfl += 2.0f * PI;
            if (dfr < 0.0f) dfr += 2.0f * PI;
            end_l = cont_l + dfl;
            end_r = cont_r + dfr;
            cur_l = cont_l; cur_r = cont_r;

            rec6_anchor = 1;
            rec6_t = 0; rec6_hold = 0;
        }
        ten_targets.l_leg_length_target = rec_l_rise;
        ten_targets.r_leg_length_target = rec_l_rise;
        if (fabsf(end_l - cont_l) < rec_align_eps &&
            fabsf(end_r - cont_r) < rec_align_eps)
        {
            if (++rec6_hold >= rec_hold_ticks)
            {
                rec6_anchor = 0;
                prone_rec_state = PRS_PUSH;
            }
        }
        else
        {
            rec6_hold = 0;
        }
        if (rec6_t > rec_stage_timeout)
        {
            rec6_anchor = 0;
            prone_rec_state = PRS_FLIP;
            prot_armed = 0;
            prot_seen_zf = 0;
        }
    }
    else if (prone_rec_state == PRS_PUSH)
    {   /* 态3 蹬地+联动收腿(2026-11-14): 两腿(已对齐)一起沿规定方向(逆时针,
         * θ 增)转向髋下后方(体系角 rec_theta_end, 实测 −0.17 定稿)。θ_end 的
         * 物理意义是轮触点对准重心: 实测重心在髋后约 2.2cm, 0.13 腿长对应
         * −0.17, 交接瞬间零偏零冲(11-14 台架: 腿瞬收+切平衡几乎无前后移)。
         * 判据(11-14 三值对照实测): 轮触点比重心偏前(含 θ_end=0) → 交接后仰
         * 掀车头 → 失控狂转; 偏后过多 → 前倾啃地前冲(旧全高 +0.22 即此形态,
         * 啃地是其稳定机制而非缺陷)。腿角只看体系角, 机体滑动无关。
         * 路程强制正向(归 [0,2π)) —— 双正腿位形锚角为正, wrap 最短会负向跨体系0 顶地
         * (外部审查实锤); 锚已略过终点带时不绕圈, 直接钳到位。
         * 腿角/腿长分列控制: 腿长目标随体系角联动收短, 低位交给 LQR。 */
        if (!rec6_anchor)
        {
            float dfl = fmodf(rec_theta_end - rec_wrap_pi(cur_l), 2.0f * PI);
            float dfr = fmodf(rec_theta_end - rec_wrap_pi(cur_r), 2.0f * PI);
            if (dfl < 0.0f) dfl += 2.0f * PI;
            if (dfr < 0.0f) dfr += 2.0f * PI;
            if (dfl > 2.0f * PI - 0.2f) dfl = 0.0f;   /* 已在/略过终点带: 不绕整圈 */
            if (dfr > 2.0f * PI - 0.2f) dfr = 0.0f;
            end_l = cur_l + dfl;
            end_r = cur_r + dfr;
            len_cmd_l = rec_l_rise;   /* 联动收腿斜坡起点=S2 出口目标(连续, 防阶跃) */
            len_cmd_r = rec_l_rise;

            rec6_anchor = 1;
            rec6_t = 0; rec6_hold = 0;
        }
        /* 低重心收腿(2026-11-14): 腿角到 θ_end 时腿长目标同步到 rec_l_low。
         * 拟合关系(顶起最小判据): 轮着地时髋高 h=R+L·cos(θ_body), 令 h 全程
         * 不超过终点蹲姿高 R+rec_l_low·cos(θ_end) —— 解出
         *   L(θ) = min(rec_l_rise, rec_l_low·cos(θ_end)/cos(θ_body))
         * 轮心在机体上半圆(cos≤0)保持 rec_l_rise 固定; 进入下半圆后收到
         * "髋刚好维持终点高度"为止, 任一时刻髋位不高于终点蹲姿 —— 机体被轮子
         * 顶起的量/pitch 变化逐点最小(线性收腿在该判据下非最优)。
         * S3 一进来就深入下半圆的位形(趴地双腿朝后, cos 已大)目标有阶跃,
         * 用 rec_retract_rate 限斜率兜底; 限速滞后不加测量值到位门(防左髋
         * 摩擦滞后卡流程), 由出口驻留 + S4 恒 rec_l_low + S5 从实测重锚收尾。 */
        float cosL = cosf(rec_wrap_pi(cont_l));
        float cosR = cosf(rec_wrap_pi(cont_r));
        float want_l = (cosL > 0.05f) ? rec_l_low * cosf(rec_theta_end) / cosL : rec_l_rise;
        float want_r = (cosR > 0.05f) ? rec_l_low * cosf(rec_theta_end) / cosR : rec_l_rise;
        if (want_l > rec_l_rise) want_l = rec_l_rise;
        if (want_r > rec_l_rise) want_r = rec_l_rise;
        if (want_l < rec_l_low)   want_l = rec_l_low;
        if (want_r < rec_l_low)   want_r = rec_l_low;
        float step_l = want_l - len_cmd_l;
        float step_r = want_r - len_cmd_r;
        if (step_l >  rec_retract_rate)  step_l =  rec_retract_rate;
        if (step_l < -rec_retract_rate)  step_l = -rec_retract_rate;
        if (step_r >  rec_retract_rate)  step_r =  rec_retract_rate;
        if (step_r < -rec_retract_rate)  step_r = -rec_retract_rate;
        len_cmd_l += step_l;
        len_cmd_r += step_r;
        ten_targets.l_leg_length_target = len_cmd_l;
        ten_targets.r_leg_length_target = len_cmd_r;
        if (fabsf(end_l - cont_l) < rec_align_eps &&
            fabsf(end_r - cont_r) < rec_align_eps)
        {
            if (++rec6_hold >= rec_hold_ticks)
            {
                /* 腿角到位 -> 直接切 LQR(无静定确认段);
                 * rec2_ramp 立即清零 —— 防二次自起残留 1.0 在切换当拍输出满 LQR */
                rec2_ramp = 0.0f;
                rec6_anchor = 0;
                prone_rec_state = PRS_LQR_RAMP;
            }
        }
        else
        {
            rec6_hold = 0;
        }
        if (rec6_t > rec_stage_timeout)
        {
            rec6_anchor = 0;
            prone_rec_state = PRS_FLIP;
            prot_armed = 0;
            prot_seen_zf = 0;
        }
    }
    else if (prone_rec_state == PRS_QUICK)
    {   /* 态6 折叠位形反向快转(2026-11-14): 态0 判定"双腿轮心已在髋下方带内
         * (|θ_body|<rec_fold_th)且腿长已收拢(<rec_fold_l_th)"——断电常停折叠态,
         * 亦即已删 FOLD->STAND 起立机(git 0f00739)的工作位形。规定方向(逆时针,
         * fmod 强制正向)对略过 θ_end 的折叠角要绕近整圈, 这里按需求的特例允许
         * 反向(顺时针)/最短路快速转到 θ_end: 断电折叠角通常只比 θ_end 大 0.1rad
         * 量级, 反向一拨即到; 腿收拢时轮子在身下近地, 短转=轮沿地滚动,
         * 无顶地/无鼻下穿插。腿长保持 rec_l_low 不动(不做伸腿顶起), 分裂位形
         * (一+一−)双腿各走最短路在 θ_end 汇合; 到位直接切态4 由 LQR 低位接管,
         * 态5 斜坡起立。鼻撑折叠位 pitch 可达 1.0+, S4 斜坡+低位蹲姿兜住交接。 */
        if (!rec6_anchor)
        {
            cont_l = l_theta_body; cont_r = r_theta_body;
            /* 最短符号路径(与态2/3 强制正向相反, 允许负向=顺时针) */
            end_l = cont_l + rec_wrap_pi(rec_theta_end - rec_wrap_pi(cont_l));
            end_r = cont_r + rec_wrap_pi(rec_theta_end - rec_wrap_pi(cont_r));
            cur_l = cont_l; cur_r = cont_r;

            rec6_anchor = 1;
            rec6_t = 0; rec6_hold = 0;
        }
        ten_targets.l_leg_length_target = rec_l_low;
        ten_targets.r_leg_length_target = rec_l_low;
        if (fabsf(end_l - cont_l) < rec_align_eps &&
            fabsf(end_r - cont_r) < rec_align_eps)
        {
            if (++rec6_hold >= rec_hold_ticks)
            {
                /* 到位直接切 LQR(与态3 出口同款: 清 ramp 防残留满输出) */
                rec2_ramp = 0.0f;
                rec6_anchor = 0;
                prone_rec_state = PRS_LQR_RAMP;
            }
        }
        else
        {
            rec6_hold = 0;
        }
        if (rec6_t > rec_stage_timeout)
        {
            rec6_anchor = 0;
            prone_rec_state = PRS_FLIP;
            prot_armed = 0;
            prot_seen_zf = 0;
        }
    }
    else if (prone_rec_state == PRS_LQR_RAMP)
    {
        if (!rec6_anchor)
        {
            /* 进入沿: 重锚定 s/yaw(清 PD 段轮子自由滚动的里程漂移)
             * + 世界系 θ 目标锚定当前实测后恒定(LQR 摆角误差 0 起步, 不收零——
             * 142335 实测: PD 未撤+LQR 未接管时收零会拉腿跨过竖直顶点, 必然向后翻倒。
             * 锚定值存档供 S5 斜坡收零起点)。 */
            ten_targets.s_target   = ten_balance_observe.s;
            ten_targets.yaw_target = ten_balance_observe.yaw;
            ten_targets.l_theta_target  = ten_balance_observe.l_theta;
            ten_targets.r_theta_target  = ten_balance_observe.r_theta;
            rec_tht_l = ten_balance_observe.l_theta;
            rec_tht_r = ten_balance_observe.r_theta;
            ten_targets.l_dtheta_target = 0.0f;
            ten_targets.r_dtheta_target = 0.0f;
            rec2_ramp = 0.0f;
            /* 重启 GRF 死延时+清悬空锁存: S1/S2 趴地位形锁存的 air=1 不清会
             * 带病进 S5(Tw 清零+Tp 降维站不起来); S4->S5 拉起瞬态与上电垫高
             * 起立同类, 用 3s 静默期盖住交接+收零+拉起全程, 之后保护恢复在线 */
            GRF_RestartAirDelay();
            rec6_anchor = 1;
            rec6_t = 0;
        }
        /* θ 目标每拍重写(=锚定值恒定): STANDING_TEST 编译下目标生成块每拍写 0,
         * 只靠进入沿锚定会被冲掉(NONE 调试编译必踩, 斜坡期等效 θ 目标=0);
         * DRIVE/NO_FOLLOW 不写 θ, 此行等价无害。 */
        ten_targets.l_theta_target = rec_tht_l;
        ten_targets.r_theta_target = rec_tht_r;
        /* 蹲姿保持(2026-11-14): 态3 已联动收拢到 rec_l_low, 斜坡期腿长不动,
         * LQR 从低位平衡起步; 起立留给态5 在闭环下做。 */
        ten_targets.l_leg_length_target = rec_l_low;
        ten_targets.r_leg_length_target = rec_l_low;
        rec6_t++;
        rec2_ramp = (float)rec6_t / (rec_lqr_ramp_t * 1000.0f);
        if (rec2_ramp >= 1.0f)
        {
            rec2_ramp = 1.0f;
            rec6_anchor = 0;
            lift_anchor = 0;    /* 态5 起立斜坡从实测重新锚定 */
            lift_top = 0;      /* 挡位接管沿同批清零 */
            prone_rec_state = PRS_DONE;
        }
    }
    else if (prone_rec_state == PRS_DONE)
    {
        /* S5 低重心起立(2026-11-14): LQR 已接管+轮已使能, 腿长目标从蹲姿以
         * rec_lift_rate 斜坡升回 rec_l_rise —— "相对快但保证稳定"的起立在闭环
         * 下完成; 斜坡起点锚定实测(容忍态3 限速滞后), 到顶恒 rec_l_rise。
         * lift_anchor 在 S4 出口/零力沿/T3 触发处清零, 各重入路径重新锚定。 */
        if (!lift_anchor)
        {
            lift_l = ten_balance_observe.l_leg_length;
            lift_r = ten_balance_observe.r_leg_length;
            LIMIT_MIN_MAX(lift_l, rec_l_low, 0.40f);
            LIMIT_MIN_MAX(lift_r, rec_l_low, 0.40f);
            lift_anchor = 1;
        }
        if (lift_l < rec_l_rise)
        {
            lift_l += rec_lift_rate;
            if (lift_l > rec_l_rise) lift_l = rec_l_rise;
        }
        if (lift_r < rec_l_rise)
        {
            lift_r += rec_lift_rate;
            if (lift_r > rec_l_rise) lift_r = rec_l_rise;
        }
        /* 双腿到顶沿(2026-11-15): 挡位斜坡重锚 rec_l_rise 防阶跃, 之后平衡/
         * 行驶/锁定的维持目标由遥控挡位接管(未配置时 ramp 恒 rec_l_rise,
         * 行为与旧版逐拍恒 rec_l_rise 完全一致)。 */
        if (!lift_top && lift_l >= rec_l_rise && lift_r >= rec_l_rise)
        {
            leg_len_cmd_ramp = rec_l_rise;
            lift_top = 1;
        }
        if (lift_top)
        {
            lift_l = leg_len_cmd_ramp;
            lift_r = leg_len_cmd_ramp;
        }
        ten_targets.l_leg_length_target = lift_l;
        ten_targets.r_leg_length_target = lift_r;
        /* S5 腿角目标从 S4 锚定值斜坡收 0 —— 普通遥控模式正常路径不写 θ 目标,
         * 不接管会一直维持锚定非零角(歪着站, 外部审查实锤)。
         * 此时 LQR 已接管+轮已使能, 小行程(≈0.25rad)闭环收零安全(与 142335
         * 跨顶翻倒场景不同: 那是 PD 未撤+轮未使能时收零)。收到 0 停止写,
         * θ 目标交还正常路径。 */
        if (fabsf(rec_tht_l) > 0.01f || fabsf(rec_tht_r) > 0.01f)
        {
            float st0 = rec_done_zero_rate * 0.001f;
            if (fabsf(rec_tht_l) > st0) rec_tht_l -= (rec_tht_l > 0.0f) ? st0 : -st0; else rec_tht_l = 0.0f;
            if (fabsf(rec_tht_r) > st0) rec_tht_r -= (rec_tht_r > 0.0f) ? st0 : -st0; else rec_tht_r = 0.0f;
            ten_targets.l_theta_target = rec_tht_l;
            ten_targets.r_theta_target = rec_tht_r;
        }
        /* 起立完成重锚 s/yaw: S4 进入沿锚的 s_target 在拉起冲刺之前, 站直后 LQR
         * 会把前冲距离原路倒拉回去(实测"前冲再回冲")。pitch 回正后一次性重锚当前
         * 位姿, 去掉回程。rec6_anchor 复用为单次标志(S4 出口/零力恢复沿/T3 触发
         * 均清零, 各进入路径安全; STANDING_TEST 编译下 s_target 被目标生成块每拍
         * 覆盖, 此重锚自然失效, 无害)。 */
        if (!rec6_anchor && fabsf(ten_balance_observe.phi) < 0.1f)
        {
            ten_targets.s_target   = ten_balance_observe.s;
            ten_targets.yaw_target = ten_balance_observe.yaw;
            rec6_anchor = 1;
        }
    }

    /* 态2/3/6 摆角目标追赶式(prot 态7 同款抗摩擦模式): 目标=实测±恒定超前,
     * PD 误差恒定→力矩恒定→突破静摩擦后维持动摩擦匀速滑, 不粘滑(142335 左腿教训)。
     * 剩余行程<超前量即钳到 end。滑动速度由 kd 负阻尼自稳(滑越快力矩越小)。
     * 态6 复用态3 的追赶超前量(折叠腿负载小, 同力矩转得更快)。 */
    if (prone_rec_state == PRS_ALIGN || prone_rec_state == PRS_PUSH ||
        prone_rec_state == PRS_QUICK)
    {
        float ahead = (prone_rec_state == PRS_ALIGN) ? rec_chase_align : rec_chase_push;
        float dl = end_l - cur_l, dr = end_r - cur_r;
        cur_l = (fabsf(dl) <= ahead) ? end_l
                : cont_l + ((dl > 0.0f) ? ahead : -ahead);
        cur_r = (fabsf(dr) <= ahead) ? end_r
                : cont_r + ((dr > 0.0f) ? ahead : -ahead);
    }
    dbg_goal_l = cur_l;   /* ep 可读: 当前摆角目标(连续角) */
    dbg_goal_r = cur_r;

    /* ===== T3: 运行中翻倒监控（2026-11-05, SPR chassis_task.c:362 同款判据换腿摆角版）=====
     * LQR 斜坡期/站立/行驶（态4/态5）双腿任一 |θ_body| 出窗驻留 -> 脱力进态7。
     * θ_body 是纯编码器量, 不受欧拉角 ±90° 折返影响; 但腿悬空不动时不出窗
     * （云台配重下侧倒难出现, 先接受）。prot_enable=0 时整块旁路（行为同旧版）。
     * 触发时清 rec6_anchor: 恢复链会再进态1, 必须重新锚定防阶跃弹车。 */
    if (prot_enable &&
        (prone_rec_state == PRS_LQR_RAMP || prone_rec_state == PRS_DONE) &&
        ten_balance_observe.l_leg_length > 0.05f &&
        ten_balance_observe.r_leg_length > 0.05f)
    {
        if (fabsf(l_theta_body) > prot_theta_th || fabsf(r_theta_body) > prot_theta_th)
        {
            if (++prot_cnt >= prot_hold)
            {
                prone_rec_state = PRS_FLIP;
                prot_armed = 0;
                prot_seen_zf = 0;
                prot_cnt = 0;
                lift_anchor = 0;    /* 回经态1~5 重入态5 时重新锚定起立斜坡 */
                lift_top = 0;      /* 挡位接管沿同批清零 */
                rec6_anchor = 0;
            }
        }
        else
        {
            prot_cnt = 0;
        }
    }
    else
    {
        prot_cnt = 0;
    }

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

    /* 窗口只限制新触发。已开始的动作独立收尾，急停和恢复状态优先。 */
    if (prone_rec_state == PRS_DONE)
    {
        StepLegMotion_Update(&step_leg_motion[0], new_hits & GRF_AIR_LEFT,
                             l_theta_body, ten_balance_observe.l_leg_length);
        StepLegMotion_Update(&step_leg_motion[1], new_hits & GRF_AIR_RIGHT,
                             r_theta_body, ten_balance_observe.r_leg_length);
    }
    else
    {
        StepLegMotion_Reset();
    }

    if (step_leg_motion[0].phase == STEP_LEG_READY &&
        step_leg_motion[1].phase == STEP_LEG_READY)
    {
        /* 接管当拍保持短腿目标，下一拍交给现有 S5 起立及挡位逻辑。 */
        lift_l = lift_r = rec_l_low;
        lift_anchor = 1;
        lift_top = 0;
        ten_targets.l_leg_length_target = rec_l_low;
        ten_targets.r_leg_length_target = rec_l_low;

        /* 旧离地位可能记的是收腿阶段，清锁存后按正常节拍重新判定。 */
        GRF_ClearAirState();
        step_window_active = 0;
        StepLegMotion_Reset();
    }

    /* 按本拍最终的控制归属，避免动作开始或结束当拍沿用旧门控。 */
    const uint8_t step_leg_mask = StepLegMotion_ActiveMask();
    const uint8_t protected_air_mask = ChassisProtectedAirMask(step_leg_mask);
    if (protected_air_mask & GRF_AIR_LEFT)
        ten_targets.l_leg_length_target = prev_len_l;
    if (protected_air_mask & GRF_AIR_RIGHT)
        ten_targets.r_leg_length_target = prev_len_r;

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
        }
        if ((step_leg_mask & GRF_AIR_RIGHT) && step_leg_motion[1].length_target > 0.0f)
        {
            step_ref_mask |= GRF_AIR_RIGHT;
            step_targets.r_leg_length_target = step_leg_motion[1].length_target;
            step_targets.r_theta_target = step_leg_motion[1].theta_body_target - pitch_rad;
            step_targets.r_dtheta_target = -chassis_IMU_data->Gyro[0];
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

    /* ===== 输出覆盖（2026-11-09 重做, 2026-11-14 增态6）=====
     * PD 段不经 LQR（手册第九章: 自救期强制不经 LQR/VMC 平衡通道）:
     * 态1~4/态6: Tp = rec_pd PD(cur-θ_body 连续目标), Tw 硬清零, 腿长力仍走下方腿长环。
     * 态7  : Tp = prot PD(追赶式目标), Tw 硬清零。
     * 态5  : LQR 正常输出乘 rec2_ramp 0->1 渐入(斜坡补 v2 缺的 Tp 钳位),
     *        腿长环/roll 补偿不乘(腿长目标经斜坡无跳变)。
     * LQR_calc 无内部状态, 覆盖安全。 */
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

    /* 关节摩擦前馈：VMC 输出的 Tf/Tb 已是电机自身坐标，与 measure.velocity 同系，
     * 直接同号叠加即可（左右腿各自的坐标约定在 VMC 内部已处理完）。
     * 脱力等待期(S0/S7 未武装)跳过 —— 零力语义完整。 */
    float ff_rf = 0.0f, ff_rb = 0.0f, ff_lf = 0.0f, ff_lb = 0.0f;
    if (!out_gate)
    {
        /* 悬空选 _air 系数(2026-11-15): 悬空载荷相关摩擦低一个量级, 沿用着地
         * Tc 必过补成负阻尼(前日提车摆腿发散); 默认 0=跳过起安全基线,
         * 从 0 起步在线标定。态7 翻身不切(air 易误判+翻身摆腿要前馈破静摩擦)。 */
        float fric_t_r = wheel_block_r ? r_joint_fric_t_air : r_joint_fric_t;
        float fric_t_l = wheel_block_l ? l_joint_fric_t_air : l_joint_fric_t;
        /* 2026-11-20 右侧后摆停角更靠前，仅该侧后摆试调动摩擦前馈 0→0.10 N·m/关节（UNTESTED）。 */
        if (step_leg_motion[1].phase == STEP_LEG_BACKSWING)
            fric_t_r = 0.10f;
        ff_rf = fric_t_r * tanhf(r_front_8009_motor->measure.velocity  / FRICTION_TANH_W0);
        ff_rb = fric_t_r * tanhf(r_behind_8009_motor->measure.velocity / FRICTION_TANH_W0);
        ff_lf = fric_t_l * tanhf(l_front_8009_motor->measure.velocity  / FRICTION_TANH_W0);
        ff_lb = fric_t_l * tanhf(l_behind_8009_motor->measure.velocity / FRICTION_TANH_W0);
        r_leg_motor_torques.r_Tf += ff_rf;
        r_leg_motor_torques.r_Tb += ff_rb;
        l_leg_motor_torques.l_Tf += ff_lf;
        l_leg_motor_torques.l_Tb += ff_lb;
    }

    chassis_dbg.joint_T_cmd[0] = r_leg_motor_torques.r_Tf;
    chassis_dbg.joint_T_cmd[1] = r_leg_motor_torques.r_Tb;
    chassis_dbg.joint_T_cmd[2] = l_leg_motor_torques.l_Tf;
    chassis_dbg.joint_T_cmd[3] = l_leg_motor_torques.l_Tb;
    chassis_dbg.fric_ff[0] = ff_rf;
    chassis_dbg.fric_ff[1] = ff_rb;
    chassis_dbg.fric_ff[2] = ff_lf;
    chassis_dbg.fric_ff[3] = ff_lb;


    // DMMotorStop(r_front_8009_motor);
    // DMMotorStop(r_behind_8009_motor);
    // DMMotorStop(l_front_8009_motor);
    // DMMotorStop(l_behind_8009_motor);
    // DJIMotorStop(l_wheel_motor);
    // DJIMotorStop(r_wheel_motor);

    DMMotorSetCtrlMode(r_front_8009_motor,  DM_CTRL_TORQUE);
    DMMotorSetCtrlMode(r_behind_8009_motor, DM_CTRL_TORQUE);
    DMMotorSetCtrlMode(l_front_8009_motor,  DM_CTRL_TORQUE);
    DMMotorSetCtrlMode(l_behind_8009_motor, DM_CTRL_TORQUE);

    /* l_VMC_Calc 已换回原电机坐标，左腿下发时不能再加负号。 */
    DMMotorSetRef(r_front_8009_motor,
        isfinite(r_leg_motor_torques.r_Tf) ? r_leg_motor_torques.r_Tf : 0.0f);
    DMMotorSetRef(r_behind_8009_motor,
        isfinite(r_leg_motor_torques.r_Tb) ? r_leg_motor_torques.r_Tb : 0.0f);
    DMMotorSetRef(l_front_8009_motor,
        isfinite(l_leg_motor_torques.l_Tf) ? l_leg_motor_torques.l_Tf : 0.0f);
    DMMotorSetRef(l_behind_8009_motor,
        isfinite(l_leg_motor_torques.l_Tb) ? l_leg_motor_torques.l_Tb : 0.0f);

    /* 轮力矩在换算 C620 电流码前限幅。空轮连同摩擦前馈在最终输出闸清零，
     * 支撑轮每拍照常下发，包含从离地转为承载的确认拍。 */
    const uint8_t wheel_gate = (prone_rec_state <= PRS_PUSH ||
                                prone_rec_state == PRS_FLIP ||
                                prone_rec_state == PRS_QUICK);
    const uint8_t wheel_gate_r = wheel_gate || wheel_block_r;
    const uint8_t wheel_gate_l = wheel_gate || wheel_block_l;
    const float ff_rw = wheel_gate_r ? 0.0f : r_wheel_fric_t * tanhf(r_wheel.r_dx / WHEEL_FRICTION_TANH_V0);
    const float ff_lw = wheel_gate_l ? 0.0f : l_wheel_fric_t * tanhf(l_wheel.l_dx / WHEEL_FRICTION_TANH_V0);
    float r_Tw = wheel_gate_r ? 0.0f : four_control_datas.r_Tw + ff_rw;
    float l_Tw = wheel_gate_l ? 0.0f : four_control_datas.l_Tw + ff_lw;
    LIMIT_MIN_MAX(r_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
    LIMIT_MIN_MAX(l_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
    chassis_dbg.wheel_ff[0] = ff_lw;
    chassis_dbg.wheel_ff[1] = ff_rw;
    chassis_dbg.Tw_cmd[0] = l_Tw;
    chassis_dbg.Tw_cmd[1] = r_Tw;

    /* isfinite 与关节侧同款护栏: LQR 状态异常(NaN)时不让 NaN 电流码下发 */
    DJIMotorSetRef(r_wheel_motor, isfinite(r_Tw) ? WHEEL_T_PER_RAW * r_Tw : 0.0f);
    DJIMotorSetRef(l_wheel_motor, l_wheel_direction * (isfinite(l_Tw) ? WHEEL_T_PER_RAW * l_Tw : 0.0f));

    



}
