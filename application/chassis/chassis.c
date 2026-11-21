#include "chassis.h"
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
#include "remote_control.h"   /* rc_left_x: 跳跃蹲伏输入(CH3) */
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
volatile float l_joint_fric_t = 0.91f;   /* N·m/关节, 左腿 */
volatile float r_joint_fric_t = 0.55f;   /* N·m/关节, 右腿 */
/* 【2026-11-16 合并同事版】悬空专用关节摩擦系数。
 * 悬空时载荷相关摩擦低一个量级, 沿用着地 Tc 会把前馈过补成负阻尼
 * (同事 2026-11-15 实测: 提车摆腿发散)。故默认 0 = 悬空不补,
 * 从 0 起步在线标定。态7 翻身不切(air 易误判 + 翻身摆腿要靠前馈破静摩擦)。 */
volatile float l_joint_fric_t_air = 0.0f; /* N·m/关节, 左腿悬空 */
volatile float r_joint_fric_t_air = 0.0f; /* N·m/关节, 右腿悬空 */
/* 【2026-11-16 修】tanh 饱和尺度 0.15 -> 0.50。
 * 旧值太小: 关节速度实测有 -0.5~-0.9 rad/s 的偏置(腿明明静止), 0.6rad/s 就是
 * ω0 的 4 倍 -> tanh 直接饱和到 ±1 -> 前馈满幅输出 0.55~0.91 N·m 的“凭空力矩”。 */
#define FRICTION_TANH_W0  (0.50f)       /* rad/s, tanh 速度平滑尺度(原0.15) */
/* 【2026-11-16 修】摩擦前馈速度死区。
 * 病根: 关节速度实测有偏置, 腿静止时读到 -0.5~-0.9 rad/s。
 * 后果(实测 13:28:43.341): 左腿前馈 -0.909, 把 VMC 的 +4.896 砍到 +3.987
 *   -> 髋电机有效出力被削 19%; 同时共模分量给机身加了 2.92 N·m 污染。
 * 修法: 死区内完全不补(反正腿也没动), 超出部分只按超出量补。
 * 取值 0.6: 能盖住全部噪声(0.49~0.87), 而真实推车/蹲下(2~5rad/s)不受影响。 */
volatile float joint_fric_v_dead = 0.60f;   /* rad/s, 摩擦前馈速度死区 */
/* 带死区的关节摩擦前馈: 死区内输出 0, 超出部分按 (|v|-死区)/ω0 平滑 */
static float JointFricFF(float v, float Tc)
{
    float av = fabsf(v);
    if (av <= joint_fric_v_dead) return 0.0f;
    return (v > 0.0f ? Tc : -Tc) * tanhf((av - joint_fric_v_dead) / FRICTION_TANH_W0);
}

/* ===== 悬空腿角保持 PD（2026-11-16 合并同事 2026-11-15 版）=====
 * 为什么不能用 LQR 降维后的 Tp: LQR_SetAir 之后 θ 通道增益只剩 ~10 N·m/rad,
 * 而左髋静摩擦 ≥2 N·m -> 死区 = 2/10 = 0.2 rad。空中腿被轮子滚动带偏后
 * 完全没有回正力(同事 2026-11-15 提车实测)。
 * 改用 air_kp_theta PD 直接覆写 Tp: 死区 = 2/30 = 0.067 rad, 小 3 倍。
 * 误差与 LQR 同源配对(l_theta_target − l_theta, wrap_pi 防过顶 2π 跳变),
 * 落地沿 LQR 瞄同一个误差 -> 无缝接管。 */
volatile float air_kp_theta = 30.0f;   /* N·m/rad, 悬空腿角保持刚度 */
volatile float air_kd_theta = 2.0f;    /* N·m·s/rad, 悬空腿角保持阻尼 */
volatile float air_tp_max   = 8.0f;    /* N·m, 悬空 Tp 限幅(顶 ≥2N·m 静摩擦留 4 倍裕量) */

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
/* 【2026-11-14】200 -> 100。
 * LegVelEst 修好零阶保持后 Ldot 不再被 500Hz/1kHz 的脉冲串平均掉一半
 * (原实际阻尼只有 92 而非 200)。阻尼翻倍把左腿推过稳定边界:
 * 站立段左腿 L 在 0.241~0.256 间以约 6Hz 自激, Fleg 在 -106~+203N 来回甩。
 * 降到 100 -> 实际阻尼约 100(比原 92 略强), 站立恢复平稳。 */
volatile float leg_kv_vel = 100.0f;     /* N/(m/s), 腿长轴向阻尼 */
/* 【2026-11-20】左右腿独立增益倍率。
 * 实测 2026-11-19 20:26: 收腿段两腿命令完全相同 (-80N),
 *   左腿速度到 -0.42m/s, 右腿只到 -0.23m/s (差 1.8 倍),
 *   160ms 内左腿收 19mm、右腿只收 6mm —— “同样的力不同响应”。
 * 先前增益两腿共用一组, 无法单独篡位。现在拆成倍率:
 *   实际刚度 = leg_kp_pos × leg_gain_x,  实际阻尼 = leg_kv_vel × leg_gain_x
 * 默认 1.0 = 与旧行为完全一致; 哪条腿响应慢就把它的倍率调大。 */
volatile float leg_gain_l = 1.0f;       /* 左腿增益倍率(1.0=基准) */
volatile float leg_gain_r = 1.0f;       /* 右腿增益倍率(响应慢就加大) */
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
/* ===== 遥控三挡腿长（2026-11-16 合并同事 2026-11-15 版）=====
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
#define LEG_PID_OUT_MAX  (150.0f)          /* 腿长 PID 分量正常限幅 N */
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

/* ===== yaw/位置 附加层（2026-11-13 自旧版移植, 2026-11-14 并入新底）===== */
/* yaw 来源开关:
 * 0 = 里程计+腿角差重构(默认做法, 与 model_lagrange 一致)
 * 1 = 直接用 BalanceObserve 里的 IMU yaw(陀螺积分)
 * 实测 20261113: 重构法每 40ms 跳 3~6°(腿角差噪声被 yaw_coef_leg 放大),
 * 改 IMU 后阶跃从 4.65° 降到 0.872°。 */
volatile uint8_t use_imu_yaw = 1;
/* 位置积分: 只补滚动阻力所需的极小力矩, 与 LQR 的 s 项同号叠加 */
volatile float s_ki = 2.0f;                /* N/(m·s) */
#define S_INT_MAX (1.0f)
static float s_int = 0.0f;
/* s/yaw 一次性重锚斜坡(指数, 系数 0.02 @1kHz -> tau=50ms), 由状态机/跳跃机置 300 */
volatile uint16_t s_yaw_reanchor_ramp = 0;

/* =====================================================================
 * 跳跃系统（2026-11-11 引入, 2026-11-14 并入新自起底）
 * 四相位: SQUAT -> PUSH -> SHRINK -> LAND
 * 手动操作: 左杆(CH3) 下拉到底 -> 蹲伏并保持; 松手回中 -> 起跳。
 * 只在 prone_rec_state == PRS_DONE(站立) 下允许起跳。
 * 腿长目标由本机覆盖(在自起状态机之后执行, 优先级更高),
 * 力前馈在下方 grav_ff 处覆盖。
 * ===================================================================== */
#define JUMP_EN  1
#define JP_IDLE    0
#define JP_SQUAT   1
#define JP_PUSH    2
#define JP_SHRINK  3
#define JP_LAND    4

volatile uint8_t  jump_enable     = 1;
/* 【2026-11-18 调试】强制蹲伏开关 —— 绕开遥控直接测跳跃。
 * 0 = 正常(看左摇杆: rc_left_x < jump_rc_th)
 * 1 = 强制蹲伏请求, 等价于“一直拉着杆”
 * 用途: 遥控链路 failsafe / 接收机没供电 时, 台架验证 SQUAT->PUSH->SHRINK->LAND 全流程。
 * 操作: 置 1 -> 车蹲下并保持; 蹲好了(蹲好标志=1) 再把本值改 0 -> 视为松手 -> 进蹬伸。
 * 注意: 要真跳必须同时把 jump_dry_run 改 0。 */
volatile uint8_t  jump_debug_squat = 0;
/* ===== 调试开关: 1 = 只测下蹲, 不真跳 =====
 * 置 1 时 PUSH/SHRINK 的腿长目标锁在蹲伏位(不再伸到 push_ramp / shrink_ramp),
 * 且 PUSH 的力前馈回到站立水平 LEG_GRAVITY_FF。
 * 效果: 松手后腿不动(停在蹲姿), PUSH 跑到 jump_push_timeout(500ms) 超时 -> why=4
 *   -> 按「异常退出」处理, 直接回 IDLE。全程不蹬地, 也不会切 SHRINK(不会把轮子清掉)。
 * 【2026-11-18】默认值 1 -> 0: 改为默认真跳。台架只测下蹲时临时置 1。
 *   判据看的不是这个开关本身, 而是 ten_targets.l_leg_length_target:
 *   蹬伸期间它应该跟着 jump_push_ramp 涨(0.15->0.32), 若一直锁 0.15 就是干跑没关。 */
volatile uint8_t  jump_dry_run    = 0;
volatile float    jump_squat_len  = 0.15f;   /* 蹲伏腿长 m */
/* 【PUSH 的“目标超前量”】注意这【不是】腿要伸到多长, 而是【腿长目标要多超前】。
 * 原理: 腿长环是 P 控制 F = Kp*(目标 - 实测)。目标必须一直跑在实测前面,
 *   PID 才会持续出正力。目标一旦被实测追上(误差=0), 力立刻掉光。
 * 实测 20261112-222343 血泪教训:
 *   目标设 0.315 -> 实测伸到 0.315 时 Fleg 从 176N 掉到 96N;
 *   目标设 0.364 -> 只剩 19N。腿直接“卸力”, 跳不起来。
 * 所以设 0.42 (机械上限 0.46 之前), 让目标全程领先。
 * 真正的“该收腿了”由【实测腿长】判断, 见 jump_push_max_len。 */
volatile float    jump_launch_len = 0.42f;
/* 【2026-11-18 v3】空中收腿目标 0.20 -> 0.22。
 * 依据(10cm 台阶, 起跳 L=0.36, 轮心 60mm, 质心 355.6mm, v0=1.844m/s):
 *   轮心高 = 质心 - 0.821*L (0.821 = 腿质心系数, 实测推算)
 *   「即时过」阈值: L < (355.6-160)/821 = 0.238
 *   窗口(=轮子高于台阶的时间段)随 L 变小而变长, 但收益递减:
 *     不收(0.36) 66~310ms(244) / 0.30 46~346(300) / 0.24 46~375(329)
 *     0.22 46~384(338) / 0.20 46~391(345)  <- 0.22 之后只多 7ms
 * 而下限由【压缩行程】卡死: 机械底 rec_l_low=0.13。
 *   【2026-11-18 修正】压缩量解析解(kp=1600, kv=100, m=11.75, 前馈 50N):
 *     落平地 1.844m/s -> 峰值压缩 79.6mm @ 95ms, 腿到 0.140 (0.22 时余量只剩 9.6mm!)
 *     落台阶 0.918m/s -> 峰值压缩 53.1mm @ 122ms
 *     轻落地 0.5m/s   -> 峰值压缩 ~40mm   @ 158ms
 *   (之前按 40~60mm 估的 —— 峰值时间算错了, 实际 95ms 不是 147ms, 压缩也大得多)
 * 所以 0.22 太冒险(落平地基本贴底) -> 改 0.24:
 *   L=0.24 -> 落平地压到 0.160, 余量 30mm ✅
 *   代价: 窗口 384 -> 375ms (少 9ms), 收腿 137 -> 127ms (更快)
 *   0.24 仍满足「即时过」阈值 0.238, 实际窗口起点 0.8ms, 等价于即时。 */
volatile float    jump_air_len    = 0.28f;   /* 空中收腿 m
                                              * 【v15 2026-11-19】0.24 -> 0.28。
                                              * 大跳后触地速度 2.3m/s, 缓冲段力上限 150N/腿
                                              * -> 总推力 400N -> a=24.2 -> 刹车距离 109mm。
                                              * 从 0.24 压 109mm 到 0.131, 正好卡在机械底 0.13。
                                              * 收到 0.28 -> 落在 0.171, 留 41mm 余量。
                                              * 代价: 落地转动惯量大一点(但 0.28 仍比 0.44 好很多)。 */
volatile float    jump_ff_push    = 40.0f;   /* PUSH 前段基础力 N/腿 */
/* 【2026-11-18 废弃】LAND 已改为直接用 LEG_GRAVITY_FF(50N), 本参数不再生效。
 * 旧代码 LAND 用它做前馈起点: 70N/腿 x2 = 140N > 整车 115.3N —— 光前馈就能把车顶飞。 */
volatile float    jump_ff_peak    = 50.0f;   /* 【已废弃】落地前馈(现用 LEG_GRAVITY_FF) */
volatile float    jump_pulse_frac = 0.60f;   /* 腿伸过这个比例后切到 peak */
/* SQUAT 斜坡速率 m/s。必须斜坡不能阶跃:
 * 阶跃 0.30->0.15 会让 PID 饱合输出 -150N, Fleg = 50-152 = -102N,
 * 腿变成"往里拽"而不是"往下蹲"。
 * 【2026-11-14】0.5 实际只走 0.33(腿长环滞后 40%), 0.15m 用了 418~439ms;
 * 而斜坡又要再等 jump_squat_hold 才授权 -> 总预算 720~740ms,
 * 用户按住只有 550~700ms -> 13/14 次都被取消。故提到 0.9。
 * 【2026-11-14 二改】0.9 -> 0.6。行程只有 10cm, 按 a=9 算刹距:
 *   0.9m/s -> 4.5cm 加速 + 1.0cm 匀速 + 4.5cm 刹车(匀速只占10%)
 *   0.6m/s -> 2.0cm + 6.0cm + 2.0cm  (匀速占60%)
 * 0.9 时腿从未真正“跟住”参考, 全程在追或被甩, 任何跟踪误差直接变成
 * 冲过去再拽回来 -> 现场看到的“抖”。降到 0.6 给刹车留出行程。 */
volatile float    jump_squat_rate = 0.6f;
/* 蹲下参考速度（阻尼项去耦）:
 * 阻尼原本惩罚腿的绝对速度 -kv*l_dot。可腿本来就在跟着斜坡走,
 * 于是被当成需要剥住的运动 -> 必须靠位置误差抵消 -> 恒定滞后 kv*v/kp。
 * 改成只惩罚偏离参考速度的部分 -kv*(l_dot - vref):
 *   匀速段 l_dot == vref -> 阻尼 = 0 -> 滞后 = 0
 *   到底段 vref = 0   -> 阻尼 = -kv*l_dot -> 全阻尼 -> 照样剥得住
 * vref 自身按 jump_squat_acc 限加速度(防起始拍阻尼 -180N 把车蹬下去)。 */
/* 【2026-11-14 四改】9 -> 5。
 * 加前馈后腿真能做出 9m/s² 了, 但 Fleg = m*(g-a) = 6.1*(9.8-9) = 4.9N,
 * 而“腿力 = 轮子压地力”, 轮压只剩 4.9N(整车 120N) -> 一碰就打滑。
 * 降到 5 -> Fleg = 6.1*(9.8-5) = 29N 轮压, 安全。
 * 代价: 下蹲 229ms -> 282ms, 用户按杆 550ms 完全够。
 * 仿真(基线 m*g): acc=9 最低轮压 -0.2N(离地); acc=5 -> 26.4N。 */
volatile float    jump_squat_acc  = 5.0f;    /* m/s^2, 收腿加减速度 */
volatile float    jump_squat_vref = 0.0f;    /* m/s, 负=收缩 */
/* 【2026-11-14】按剩余距离限速后的当前允许速度, 仅用于观测 */
volatile float    jump_squat_vlim = 0.0f;    /* m/s, 正值=收缩速度上限 */
/* 【2026-11-14 三改】斜坡自身速度, 按 jump_squat_acc 限加速度 —— 这就是“斜率变化”
 * 旧写法斜坡第 1 毫秒就以 jump_squat_rate 走, 等于速度阶跃, 腿追不上就欠账 20mm。 */
volatile float    jump_squat_vramp = 0.0f;   /* m/s, 正值=收腿速度, 0=斜坡静止 */
/* 【2026-11-14 四改】斜坡当前加速度 m/s²(正=收腿加速, 负=刹车)。
 * 用途: 加速度前馈 grav_ff = LEG_MASS_EFF * (9.8 - jump_squat_acc_now)。
 * 为什么必须前馈: 位置环靠“误差”出力 ——
 *   要 Fleg 从 60N 降到 4.9N(加速下降)需要 28mm 滞后;
 *   刹车要 114.7N(减速)需要 40mm 超前, 但刹距只有 20mm -> 刹不住。
 *   仿真: 无前馈冲过 26.4mm, 加前馈(基线 m*g)只冲 0.6mm。 */
volatile float    jump_squat_acc_now = 0.0f; /* m/s², 正=收腿加速 */
/* 【蹲稳判定的腿长速度阈值】m/s。
 * 为什么要这一条: 只判位置不够 —— 腿还在往下冲、只是“恰好穿过 0.15”那一瞬间,
 *   也会被算成“到位”。此时松手起跳 = 从运动状态起跳, 速度不连续、位置也不准。
 * 实测稳定站立时 |腿长速度| 约 0.003~0.03 m/s, 取 0.05 (5cm/s) 留 2 倍余量。
 * 太大(>0.1): 会把“还在冲”误判成静止;  太小(<0.02): 会被噪声卡住进不了 PUSH。 */
volatile float    jump_squat_v0   = 0.05f;   /* m/s, 蹲稳的腿长速度阈值 */
volatile float    jump_squat_band = 0.015f;  /* m, 到位判定带(2026-11-14: 10mm->15mm, 满量后弹回也能落在带内) */
/* 蹲下末段专用积分增益: 全局 ki=500 -> tau=3.2s, 蹲底停 300ms 只能攒 1N,
 * 补不上"前馈 50N 比车重少的那部分"。但又不能改全局 ki(站立/落地靠它),
 * 所以只给蹲下末段单独一个。车重不需要提前知道, 积分自己找平衡点。 */
/* 【2026-11-14 四改】2000 -> 400。加了加速度前馈后静态误差本来就接近 0,
 * 2000 在 0.5s 内就能把 25N 限幅积满 -> 反向 -> 自激。仿真里两者差异很小
 * (0.6mm vs 0.8mm), 降下来只是保险。 */
volatile float    jump_squat_ki   = 400.0f;  /* N/(m·s), 蹲底积分增益 */
volatile float    jump_push_rate  = 1.4f;    /* m/s, 腿长目标爬升速度上限 */
volatile float    jump_push_ramp  = 0.15f;
/* 【2026-11-16 PUSH 规划器 v2】与 SQUAT 同一套结构, 但方向相反。
 * PUSH = 蹬地起跳: Fleg = m*(g + a), 所以前馈是【加】不是减。
 * 旧写法只有一行 `ramp += jump_push_rate*0.001`, 等于速度阶跃 —— 腿从静止
 * 突然被要求 2m/s, 追不上就欠账, P 项看到大误差就直接打满, 力失控。
 * 新写法: vramp 自身按 jump_push_acc 限加速度, 再驱动 ramp。
 * 【与 SQUAT 的关键区别】PUSH 不刹车: 要的就是那一瞬间的高速飞出去。
 *
 * v2 三处优化(2026-11-16):
 * ① 加速度改「规划值」而非「差分值」。旧版 acc_now = (vramp-vold)*1000,
 *    vramp 到顶后 acc_now 立刻变 0 -> grav_ff 从 82N 突降到 50N = 32N 阶跃,
 *    腿会"被抽走"一下。现改为: 加速段恒 = jump_push_acc, 到顶后按
 *    jump_push_acc_down 缓降到 0。
 * ② 新增「实测腿长到位」主动退出。air_state 要走 th_off + hold_off(100ms)
 *    才认, 这 100ms 里车已离地而腿还在按规划空伸 -> 转动惯量变大、可能撞限位。
 * ③ 超时 400 -> 300ms (新行程只需 ~240ms)。
 *
 * 算例(rate=1.4, acc=6, 行程 0.15->0.32 = 0.17m):
 *   加速到 1.4m/s 需 233ms, 行程 0.163m; 剩 7mm 匀速 5ms -> 全程 238ms
 *   理论跳高 = 1.4^2/(2*9.81) = 10.0cm
 *   对应 Fleg = 50 + 5.4*6 = 82.4N/腿 (基线 50 + 加速度项) */
volatile float    jump_push_acc  = 6.0f;     /* m/s², 腿伸展加速度(规划值) */
/* PUSH 专用等效质量【2026-11-16 优化】由实测反推:
 * 站立单轮地面反力 median 57.5N, 减轮自重 m_w*g=4.9N -> 单腿实际承重 52.6N
 * -> M/2 = 52.6/9.81 = 5.36 kg。
 * SQUAT 沿用的 LEG_MASS_EFF=5.1 是从「50N/g」来的, 对 SQUAT 已验证够用
 * (overshoot 0.2mm), 但 PUSH 要精确算推力: 用 5.1 会让实际加速度低 11%。 */
volatile float    jump_push_mass = 5.875f;   /* kg, PUSH 加速度前馈等效质量 = 整车/2
                                              * 【2026-11-17 实测回推】站立 F_n 左右合计 115.3N
                                              * -> M=11.75kg -> M/2=5.875。旧值 5.4 少给 8% 推力。 */
volatile float    jump_push_acc_down = 40.0f;/* m/s³, 到顶后加速度撤除斜率(40ms 撤完 6m/s²) */
/* 【实测腿长安全上限】m。PUSH 期间用【实测腿长】判断“该收腿了”, 不再单纯等 air_state ——
 * 因为 air_state 要 th_off + hold_off(100ms) 才认, 这 100ms 里车已离地而腿还在按规划空伸,
 * 转动惯量变大、还可能撞 0.46 机械限位。
 * 0.36 = 0.46 上限 - 100mm 安全余量。到它就进 SHRINK 收腿。 */
volatile float    jump_push_max_len = 0.43f; /* m, 实测腿长出口
                                              * 【v9 2026-11-19 大跳】0.36 -> 0.44。
                                              * 原 0.36 的问题: 冲量段行程只有 0.36-0.32=40mm,
                                              *   冲量段只能给 2*24.2*0.04=1.94 (m/s)^2 的 dv^2,
                                              *   起跳速度只有 1.84m/s(实测 1.5), h=11.5cm。
                                              * 0.44 -> 冲量段行程 120mm, dv^2=5.82,
                                              *   设计起跳速度 2.80m/s。
                                              * 机械上限 = L_LEG_A+L_LEG_B = 0.46; 留 20mm 余量。
                                              * 警告: 0.46 是运动学奇点(实测 202628 左腿在
                                              *   0.4599 处解算翻分支到 0.0401), 不得再往上加。
                                              *
                                              * 【v17 2026-11-21 仿真定案】0.44 -> 0.43。
                                              * 仿真( data/sim/jump_transition_sim3.py )实测时序:
                                              *   出口腿长 0.44 + 判据延迟 10ms(2.9m/s 下走 29mm)
                                              *   + SHRINK 刹车行程 32mm(60N 上限) = 0.501m > 0.46
                                              *   -> 腿在 air_state 置 1 之前就撞上机械限位,
                                              *      车身动量被地面吃掉(悲观模型跳高归零)。
                                              * 收到 0.43 后三项合计 = 0.43+0.029+0.032 = 0.491,
                                              *   配合判据 10->1 拍(延迟 29mm->2.9mm) = 0.465 仍紧,
                                              *   再把 taper 30->15mm(出口速度+7%) -> 仿真 0.43+0.003+0.031
                                              *   = 0.464 的行程, 但实测出口腿速已降到 2.77m/s,
                                              *   刹车行程 29mm -> 合计 0.462, 不撞限。
                                              * 仿真跳高: 282mm (两极限模型一致), 原 0.44 版本 292/0。 */
volatile float    jump_push_vramp = 0.0f;    /* m/s, 斜坡速度(已限加速度), 正值=伸腿 */
volatile float    jump_push_vref  = 0.0f;    /* m/s, 参考速度, 给阻尼项用 */
volatile float    jump_push_acc_now = 0.0f;  /* m/s², 当前加速度, 给前馈用 */
/* ===== 【2026-11-17 v3】目标跳高驱动的 PUSH 参数（主参数只剩 jump_push_h）=====
 * 旧版要同时手调 jump_push_rate 与 jump_push_acc，且两者必须自洽：
 *   rate=1.4 需要行程 v²/(2a)=1.96/12=163mm，而实际可用 0.36-0.15=210mm。
 *   改任一参数就失效，且没人会每次核对。
 * v3: v_goal = sqrt(2·g·jump_push_h) 由跳高反解，
 *     再用 v_cap = sqrt(2·jump_push_acc·stroke) 夹一次 —— 行程不够就自动降速，
 *     不会再出现“命令速度超出加速度能力”导致腿永远追不上的情况。 */
volatile float    jump_push_h      = 0.30f;  /* m, 【主参数】目标跳高
                                              * 【v9 2026-11-19 大跳】0.13 -> 0.30。
                                              * 它与 jump_push_max_len 是耦合的:
                                              *   v_total = sqrt(2*g*h/eta)  -> 总起跳速度需求
                                              *   dv2_imp = 2*a_imp*(L_max-L_impulse_at)
                                              *   v_base  = sqrt(v_total^2 - dv2_imp) -> 基础段末速
                                              * 取 0.30 配 L_max=0.44 时:
                                              *   v_total=2.80, dv2_imp=5.82, v_base=1.43,
                                              *   a_base=5.98 (正好卡在 jump_push_acc=6.0 上限附近)。
                                              * 再往上加 h, a_base 会被 6.0 卡住, 末速反而不变 ——
                                              * 那时应改 jump_push_acc 或 jump_impulse_ff。 */
/* 【2026-11-18 v4】400 -> 500。规划用时: 基础段 281ms(0->1.209@4.3) + 冲量段 26ms
 * = 307ms。400 只留 93ms 余量(30%); 腿长环跟踪有稳态滞后 4.75mm, 关节摩擦再吃掉
 * 一些力气 —— 保守提到 500, 留 63% 余量。超时退出 = 回 IDLE(不跳了), 宁可晚一点。 */
volatile uint16_t jump_push_timeout = 800;   /* ms, 兜底超时
                                              * 【v11 2026-11-19】500 -> 800。
                                              * 出口 now 要等真离地确认: 行程 295ms +
                                              * GRF 驻留 100ms + 余量, 500ms 会误判超时。 */
volatile float    jump_push_sym_th = 0.060f; /* m, 两腿实测腿长差超过此值→判单腿异常
                                              * 【v15】0.040 -> 0.060。
                                              * 出口从 0.36 抬到 0.44 后蹬伸行程长了 60mm,
                                              * 左右差更容易超 40mm; 而 why=3 的去向是 IDLE
                                              * (中止跳跃) —— 0.42 处触发会把正在以 2m/s
                                              * 上升的车直接打断。 */
/* 【2026-11-18 v4】蹬伸「腿卡住」检测 —— 旧写法是死代码:
 *   `!jump_push_imp && jump_push_ramp > jump_impulse_at + 0.015f`
 * 而基础段 ramp 被 clamp 在 jump_impulse_at(0.320) -> 永远不大于 0.335, 从不触发。
 * 新判据: 只在【冲量段】看「腿长还长不长」——
 *   冲量段目标 = 实测+5mm(每拍现算), 正常情况下 26ms 就走完 40mm;
 *   段内 50ms 累计涨幅不到 2mm => 腿被卡住(顶到台阶/电机堵转)。
 * 【为什么不在基础段检测】跳得矮时 v_base 会被算成 0(v_total² < dv2_imp),
 *   基础段规划本来就不动腿 —— 在基础段检测会误判。 */
volatile float    jump_push_stuck_dl = 0.002f;  /* m, 冲量段内累计涨幅阈值 */
volatile uint16_t jump_push_stuck_ms = 50;      /* ms, 冲量段多久没涨算卡住 */
volatile uint16_t jump_push_stuck_t  = 0;       /* 观测: 已连续没涨的拍数 */
volatile float    jump_push_len_last = 0.0f;    /* 观测: 上次重锚的腿长 */
volatile float    jump_push_vtop   = 0.0f;   /* m/s, 观测: 总起跳速度需求 v_total */
volatile float    jump_push_len_avg = 0.0f;  /* m, 观测: 双腿平均实测腿长 */
volatile float    jump_push_vlaunch = 0.0f;  /* m/s, 观测: 退出瞬间平均腿速(=实测起跳速度) */
volatile uint8_t  jump_push_abort  = 0;      /* 观测: 0=进行中 1=离地 2=行程走完 3=不对称 4=超时 5=腿卡住 */
/* 【2026-11-21】PUSH 出口诊断镜像（float）。
 * u8/u16 经 EmberProbe 采样会偏到 addr+4(summary 记过: jump_push_abort 读成 63),
 * 所以关键判断量全部做 float 镜像, 保证看到的是真值。
 *   dbg_push_abort_f : PUSH 退出原因码(0~5) —— 一看就知道卡在哪条判据
 *   dbg_push_imp_f   : 1=已进冲量段(腿长 >= jump_impulse_at)
 *   dbg_push_stuck_f : 卡住计数(达到 jump_push_stuck_ms 就走 why=5)
 *   dbg_jump_ready_f : 蹲伏已保持够 jump_squat_hold(松手才会进 PUSH)
 *   dbg_push_taper_f : 尾部卸载系数 1->0
 *   dbg_air_grf_f    : GRF 原始悬空标志(chassis 内部 air_grf), 与 PUSH 出口判据同一份 */
volatile float    dbg_push_abort_f = 0.0f;
volatile float    dbg_push_imp_f   = 0.0f;
volatile float    dbg_push_stuck_f = 0.0f;
volatile float    dbg_jump_ready_f = 0.0f;
volatile float    dbg_push_taper_f = 1.0f;
volatile float    dbg_air_grf_f    = 0.0f;
volatile float    dbg_jump_ph_f    = 0.0f;   /* 0=IDLE 1=SQUAT 2=PUSH 3=SHRINK 4=LAND */
/* ===== 【2026-11-17 v4】两段式 PUSH: 基础段(位置控) + 冲量段(力控) =====
 * 台阶目标: 2 级, 每级 高 10cm / 深 30cm; 留 3cm 余量 -> 要跨 13cm。
 * 冲量段 = 最后一段行程内的【恒力】, 把基础段的速度再顶一段:
 *     Δv² = 2(2·F_imp - m·g)/m · Δx      (m=整车, F_imp=单腿)
 * 代入 200N × 40mm: Δv²=1.94 -> 基础段只需 1.206m/s, 冲量后 1.842m/s。
 *
 * 【为什么冲量段必须切力控】若继续追位置目标, 腿一变快误差就缩小,
 *   PD 出力跟着掉 -> 冲量自己消失。所以冲量段把 PD 的目标改成
 *   “实测腿长+5mm” / vref 改成实测速度, 让 PD 归零, 力全由前馈给。
 * 【切换判据用实测, 不用规划】40mm 是按“给多少 Δv”算出来的;
 *   用规划值会差掉 5.4mm 跟踪滞后 -> 实际冲量 45mm, 跳高偏 2cm。
 * 【力 5ms 斜坡】50->200N 一步跳会给结构硬冲击; 5ms 斜坡只损失 3.5% 冲量。 */
volatile float    jump_push_eta    = 0.75f;  /* 效率系数: 理想跳高打 75 折(腿惯量/延迟/能量损失) */
volatile float    jump_impulse_at  = 0.320f; /* m, 冲量段起点【实测】腿长 */
volatile float    jump_impulse_ff  = 200.0f; /* N/腿, 冲量段恒力(=50ff+150 PD 钳位上限) */
volatile uint16_t jump_impulse_ms  = 20;     /* ms, 冲量力 50->200 的爬升时间
                                              * 【v18 2026-11-21】5 -> 20。
                                              * 旧值 5ms = 150N/5ms = 30N/ms 的力阶跃, 相当于
                                              * 给车架一脚。实测 22:46 跳跃: 冲量段开启那一拍力从
                                              * 100 -> 206 N/腿, 直接把 dF_roll 的正反馈回路(增益 1.92)
                                              * 踢起来, 150ms 后 roll 到 17°, 两腿撕开43mm。
                                              * 20ms = 7.5N/ms, 同样的冲量但起始扰动小很多。
                                              * 代价: 冲量段总长 78ms(1.4m/s 走 110mm), 20ms 斜坡
                                              * 占 26%, 理论冲量损失约 3~4% —— 换来不再踢出 roll。 */
/* 【v10 2026-11-19】冲量力【尾部卸载】长度 m。
 * 旧版: 恒力 200N 一直顶到 jump_push_max_len, 出口一拍归零。
 *   两个问题:
 *   ① 200N 的力阶跃 -> 关节力矩阶跃, 结构振铃;
 *   ② 出口那一刻轮子被 N = 2F + m_w·g = 400 + 9.8 = 410N 死压在地上,
 *      腿还能伸(0.44 < 0.46), 所以永远不会离地。
 * 现在: 最后这一段落内把力度(含 50N 基线)线性乘到 0,
 *   出口时腿力≈0 -> N≈9.8N -> 轮子卸载 -> 腿靠惯性继续伸到机械极限。
 * 0.03 = 最后 30mm 卸载。设 0 等价于旧行为(硬阶跃)。
 *
 * 【v17 2026-11-21 仿真定案】0.03 -> 0.015。
 * 卸力段每米吃掉的动能是恒力段的 2.6 倍:
 *   恒力段 dv² = 2*24.2*x ; 卸力段 dv² = 17.6*x (积分 a(u)=37.2(1-u)-9.81)
 * 仿真( push_max 0.42 隔离测试, 只改 taper 长度):
 *   taper 30mm -> 出口腿速 2478mm/s, 跳高 242mm
 *   taper 15mm -> 出口腿速 2600mm/s, 跳高 254mm
 *   taper  5mm -> 出口腿速 2653mm/s, 跳高 271mm
 * 也就是说「卸力越好, 跳得越矮」——它是为让离地判据能置位而付的买路钱,
 * 所以只留到「刚好让 air_state 置 1」就够了。15mm 在 2.7m/s 下是 5.5ms,
 * 足以让 f_avg 掉到 15N 以下并驻留 1 拍。
 * 注意: 设 0 不只是「不卸力」, 而是【PUSH 永远退不出去】——
 *   F=200N 时 N=2F+m_w*g=410N 恒 > th_off(30N), air_state 永不置 1,
 *   只能等 jump_push_timeout(800ms) 走 why=4 退回 IDLE。 */
volatile float    jump_impulse_taper = 0.015f;
volatile float    jump_push_taper_k  = 1.0f; /* 观测: 当前冲量力卸载系数 1->0 */
volatile uint8_t  jump_push_imp    = 0;      /* 0=基础段(位置控) 1=冲量段(力控) */
volatile uint16_t jump_push_imp_t  = 0;      /* 冲量段已运行拍数 */
volatile float    jump_push_imp_k  = 0.0f;   /* 0..1, 冲量力爬升系数 */
volatile float    jump_push_vbase  = 0.0f;   /* m/s, 观测: 基础段末速度目标 */
volatile float    jump_push_abase  = 0.0f;   /* m/s², 观测: 基础段加速度 */
/* ===== 【2026-11-17 v2】SHRINK 收腿段 =====
 * 旧版三个问题:
 *  ① 纯 P 无阻尼: 空中腿折算质量 1.77kg, kp=1600 -> ωn=30rad/s(4.8Hz), ζ=0
 *    -> 0.36->0.20 的阶跃目标在 130ms 驻留里正好振一圈半。
 *  ② 目标是阶跃: 160mm 误差 x kp = 256N, 钳在 150N -> 腿冲过 0.20 再弹回。
 *  ③ 重力前馈 50N: 空中没东西承接, 只把腿往外推, 位置环再拉回来 —— 打架。
 * 改法: 目标改成【1.2m/s 的斜坡】(133ms 走完 160mm) + 加"只罚偏离参考速度"的
 *   阻尼(同 SQUAT/PUSH 思路: 匀速段 l_dot==vref -> 阻尼=0 -> 力归零, 不抽)。
 *   力上限单独给 60N: 2x60=120N < mg=115.3N, 万一脚还在地上也拽不动车。 */
/* 【2026-11-18 v3】改三角形速度曲线(加速 -> 立刻减速), 全程用满加速度。
 * 物理: 空中收腿要推开的是【约化质量】μ = m_l·M/(m_l+M)
 *   = 2.1 x 9.65 / 11.75 = 1.73 kg (不是脚踩地时的 LEG_MASS_EFF=5.1)
 *   力 60N -> a_max = 60/1.73 = 34.7 m/s², 留 15% 余量取 30
 * 行程 dL=140mm, a=30 -> 峰值 v = sqrt(dL·a) = 2.05 m/s, 用时 2√(dL/a) = 137ms
 * 窗口(10cm 台阶): 不收腿 66~310ms -> 收 0.22 后 46~384ms (+39%)
 *
 * 【为什么必须加加速度前馈】只给斜坡不限加速度 = 速度阶跃, 腿只能靠
 *   位置误差去追: 误差 = μ·a/kp, 全程欠账, 实际收腿时间远超 137ms, 窗口全丢。
 *   加前馈 F = -μ·a_now 后误差 -> 0。
 * 【约化质量 μ 的推导】双腿对称同时收, 自由落体参考系(无外力, 动量守恒):
 *   m·v_leg + M·v_body = 0,  v_leg - v_body = v(收腿速度)
 *   m=4.2(两腿), M=7.55(车身) -> v_leg = 0.643v, v_body = -0.357v
 *   T = 0.5·4.2·(0.643v)² + 0.5·7.55·(0.357v)² = 1.349v²
 *   广义力 Q = 2F (两腿各 F), 2F = d/dt(∂T/∂v) = 2.698a  ->  F = 1.35a
 *   (1.73 是「单腿收、另一腿+车身不动」的约化质量, 这里用错会多给 28%)
 *   注: m_leg=2.1kg/腿 是从 m_eff=1.77 反推的, 未实测; 宁可少给(慢一点位置环会补)
 * 【空中不需要重力补偿】(自由落体参考系里腿不受重力), 但【需要惯性力】。 */
/* 【2026-11-19 v4】规划器降速: 30 -> 15 m/s², 2.2 -> 1.4 m/s。
 * 【为什么降 acc 才是关键】三角曲线峰值 = sqrt(acc·行程) = sqrt(30×0.121) = 1.905m/s,
 *   1.905 < rate(2.2) —— rate 从来没碰到过, 真正的限幅是 acc。
 * 【实测教训】224726: 命令已经顶满 100.5N/腿, 腿的加速度只有 17.4m/s²,
 *   等效质量 100/17.4 = 5.75kg, 而模型只用 1.35kg (差 4.3 倍)。
 *   结果: 斜坡 137ms 就到 0.24, 腿才走到 0.288/0.322, 落后 32/51mm。
 * acc 降到 15 后峰值 = sqrt(15×0.121) = 1.35m/s, 正好在腿的能力内(18m/s²);
 *   用时 2·sqrt(0.121/15) = 180ms, 加上起跳残余伸腿速度的刹车时间约 50ms,
 *   共 ~230ms, 仍在落地窗口(46~375ms)内。 */
volatile float    jump_shrink_rate = 1.4f;   /* m/s, 收腿速度上限(三角形曲线封顶, 一般不生效) */
volatile float    jump_shrink_acc  = 15.0f;  /* m/s², 收腿加减速度(真正决定峰值速度的就是它) */
volatile float    jump_shrink_mass = 1.35f;  /* kg, 【双腿同时收】的单腿约化质量 */
volatile float    jump_shrink_ff_max = 60.0f;/* N, 位置环力上限(左腿, 前馈不计入) */
/* 【2026-11-18 v5】右腿单独的收腿力上限。
 * 实测右腿静态比左腿多扛 5.9N、腿角偏 -4.1°、腿长多 1.4mm —— 两腿不对称。
 * 收腿时用同一个 60N, 会出现一边先收到位、另一边卡在原地 -> 落地砸单腿。
 * 调法: 右腿卡住就把这个单独调大(80 -> 100 试), 左腿不动。
 * 等于 jump_shrink_ff_max 时与旧版逐拍一致。 */
volatile float    jump_shrink_ff_max_r = 60.0f;
/* 【2026-11-18 v5】左右腿收腿附加前馈 N (负 = 更用力收, 正 = 顶住)。
 * 用途: 力上限已经顶满(命令饱和到 -60N)右腿还是不动时, 用它单独补。
 * 也可用于抵消两腿静态承重差。默认 0 = 不生效。 */
volatile float    jump_shrink_ff_l = 0.0f;
volatile float    jump_shrink_ff_r = 0.0f;
volatile float    jump_shrink_ramp = 0.36f;  /* m, 当前收腿目标(斜坡, 进入时=实测腿长) */
/* 【2026-11-20】收腿段【左右腿各自】的目标与斜坡速度。
 * 旧版两腿共用一个 jump_shrink_ramp: 一条腿响应慢, 位置误差就一直比别人大,
 * 却永远追不上对方 —— 两腿越拉越开(实测落地时差 230mm)。
 * 现在每腿从自己的实测起点出发、各自按同样的三角速度曲线走,
 * 收益 = “各自走完同样的收缩量”, 而不是“追同一个绝对位置”。
 * jump_shrink_ramp / jump_shrink_vramp 保留, 取两者较大值, 供出口判据与观测用。 */
volatile float    jump_shrink_ramp_l = 0.36f;
volatile float    jump_shrink_ramp_r = 0.36f;
volatile float    jump_shrink_vramp_l = 0.0f;
volatile float    jump_shrink_vramp_r = 0.0f;
volatile float    jump_shrink_acc_l = 0.0f;   /* m/s², 观测: 左腿斜坡加速度 */
volatile float    jump_shrink_acc_r = 0.0f;   /* m/s², 观测: 右腿斜坡加速度 */
volatile float    jump_shrink_vref = 0.0f;   /* m/s, 收腿速度参考(负=收腿) */
/* 【v15 2026-11-19】左右腿【各自】的收腿参考速度。
 * 两腿机械不对称(摩擦/静态承重/腿角偏), 同一个 vref 必然导致
 *   ㈠边先收到位、另一边还在半路(12:48 实测差到 136mm) -> 单腿落地。
 * 现在落后的那条腿 lerr 更大 -> v_catch 更大 -> 阻尼项 -kv(ldot-vref)
 *   给出更大的收腿力 -> 自动追赶, 两条腿同时到位。 */
volatile float    jump_shrink_vref_l = 0.0f;
volatile float    jump_shrink_vref_r = 0.0f;
volatile float    jump_shrink_vramp = 0.0f;  /* m/s, 斜坡速度(正=收腿), 已限加速度 */
volatile float    jump_shrink_acc_now = 0.0f;/* m/s², 斜坡加速度(正=加速收腿) */
/* 【2026-11-18 v4】起立速度 1.0 -> 0.3。
 * 缓冲到底时腿约 0.16~0.18, 离站立 0.25 差 70~90mm。
 * 1.0m/s 起立: 90mm 只要 90ms, 但腿跟不上目标 -> 位置误差堆积,
 *   1600x0.09 = 144N/腿 (+前馈50) = 388N 总 = 3.3g 的顶推力, 会把车顶起来。
 * 0.3m/s: 300ms 走到位, 腿能跟住斜坡 -> 位置误差只剩加速需要的部分 -> 力很小。
 * 代价: 总时长 105(缓冲)+300(起立)+250(稳定) = 655ms, 在 2000ms 硬超时内。 */
volatile float    jump_land_rate  = 0.3f;    /* m/s, 起立伸腿速度 */
volatile float    jump_land_ramp  = 0.20f;
/* 【2026-11-18 v3.3】SHRINK→LAND 的冗余落地判据: 腿长速度阈值 m/s。
 * 主判据还是 GRF 接地(air_grf), 但 GRF 有 30~40N 死区(低于30判空中/高于40判接地,
 *   中间什么都不做状态保持) —— 落地力衰减路径恰好穿过这一段, 一旦卡住就退不出去。
 * 冗余判据: 收腿完成(vramp<0.02) 后腿长速度突然变负 = 腿被地面顶住往下压。
 * 取值依据: 空中收完腿后 l_dot 应在 0 附近(噪声 ±0.1), 落地瞬间会到 -0.5~-1.8。
 *   0.3 给噪声留 3 倍余量, 又能灵敏触发。 */
volatile float    jump_land_vth   = 0.3f;    /* m/s, 落地冗余判据: 腿长速度阈值 */
/* ===== 【2026-11-18 v4】LAND 缓冲段 =====
 * 物理: 落地只有阻尼 + 位置环出力。如果目标跟着实测走(误差恒 0), 位置环永不出力,
 *   阻尼的稳态沉速 = mg/(2kv) = 115.3/200 = 0.576 m/s —— 车会一直沉到撞底。
 *   所以【目标必须钉住】= 进 LAND 时的实测腿长, 压缩越深位置环出力越大。
 * 解析解(kp=1600, kv=100, m=11.75): ωn=16.5rad/s, ζ=0.516
 *   峰值时间 tp = 95ms(落平地) / 122ms(落台阶) / 158ms(轻落地)
 *   所以“速度回升”比固定时间更准 —— 它恰好对应压缩最深处。
 * 主判据: ldot 连续 10ms 高于 -jump_land_v0(=-0.05) = 压缩到底开始回弹
 * 兑底: jump_land_buffer=200ms (覆盖最慢的轻落地 158ms) */
volatile uint16_t jump_land_buffer   = 200;  /* ms, 缓冲段兑底时长 */
volatile float    jump_land_v0       = 0.05f;/* m/s, 缓冲结束: 腿长速度回升阈值 */
volatile uint8_t  jump_land_buf_done = 0;    /* 观测/状态: 缓冲段已完成 */
volatile uint16_t jump_land_buf_t    = 0;    /* 观测: 速度回升已连续拍数 */
volatile uint16_t jump_land_settle   = 0;    /* 观测: 稳定计时 */
/* 【2026-11-19 v6】缓冲段必须“真正触地”才开始。
 * 实测 224726: SHRINK 提前退出 -> LAND 在【空中】开始(腿 0.271/0.290),
 *   而空中腿本来静止(ldot≈0) -> 缓冲判据「腿速≈0 连续10ms」在 22.705 空中就满足,
 *   22.844 真正触地时缓冲段已经“用光”, 力被钳在 60N -> 压穿机械底 12mm。
 * jump_land_touched: 0 = 还没触地(目标给 rec_l_rise, 腿先在空中收好)
 *                    1 = 已触地(目标重锚成落地那一刻的实测腿长 = 缓冲起点)
 * jump_land_gnd_t  : 触地后累计 ms, 缓冲段兑底(按触地时间算, 不按 LAND 总时间) */
volatile uint8_t  jump_land_touched  = 0;
volatile uint16_t jump_land_gnd_t    = 0;
/* 【2026-11-19 v6】缓冲段腿力上限 N(单腿)。
 * 设计阻尼 kv·v = 100×1.844 = 184N/腿, 但通用门限幅在 |误差|>rec_push_band 时
 * 把 PID 钳到 rec_push_max(60N) —— 只剩 1/3, 刹车距离从 67mm 变 182mm,
 * 实测直接压穿机械底(0.118 < rec_l_low 0.13)。
 * 只在【缓冲段】放开; 缓冲完成(腿速归零)后自动收回 60N, 防起立弹车。
 * 150N/腿 = 两条 300N = 2.6g 减速度, 够刹住 1.8m/s 且单腿不过 2×车重。 */
volatile float    jump_land_ff_max   = 150.0f;
/* 【2026-11-18 v4】起立速度参考 m/s (正=伸腿)。
 * 一个变量两用 —— 阻尼永远是 -kv(l_dot - vref):
 *   缓冲段 vref=0   -> 阻尼全量 -kv·l_dot -> 吸能(落地瞬间 184N/腿)
 *   起立段 vref=0.3 -> 匀速段 l_dot==vref -> 阻尼=0 -> 不再拖后腿
 * 不加的后果: 起立时 l_dot=+0.3, 全量阻尼反向出 -30N,
 *   位置环要额外攒 30N/1600 = 19mm 误差 -> 全程滞后 23.5mm, 300ms 拖成 378ms。
 * 到位后自动回 0 -> 阻尼恢复全量 -> 温柔停住。 */
volatile float    jump_land_vref     = 0.0f; /* m/s, 起立速度参考(正=伸腿) */
/* ===== 【2026-11-18 v4.1】IDLE 腿长回归 =====
 * 问题: IDLE + PRS_DONE 时【没有任何代码写 ten_targets.l_leg_length_target】,
 *   它保持上一拍的值。正常真跳没事(LAND 退出时 jump_land_ramp 已经是 rec_l_rise),
 *   但下面三种情况会让腿卡在半路回不来:
 *     ① SQUAT 没蹲好就松手 -> 停在当前 jump_squat_ramp
 *     ② PUSH 异常退出(不对称/超时/卡住) -> 停在 jump_push_ramp
 *     ③ 干跑(jump_dry_run=1, 腿长锁在 0.15) -> 松手后一直蹲着起不来
 *   更要命的是干跑测试完全没法用: 蹲下去就再也站不起来。
 * 修法: 回 IDLE 时置 jump_idle_return=1 并锚定实测, IDLE 期间以 jump_land_rate
 *   (0.3m/s) 平滑拉回 rec_l_rise, 到位自动清标志。
 * 仅在 prone_rec_state==PRS_DONE 时生效 —— 自起流程跑的时候不能抢它的目标。 */
volatile float    jump_idle_ramp   = 0.25f;  /* m, IDLE 腿长回归斜坡当前值 */
volatile uint8_t  jump_idle_return = 0;      /* 1 = 需要把腿长拉回站立 */
/* 【2026-11-18 v3】130 -> 150。收腿三角形曲线全程 137ms, 130 会在腿还没收完时
 * 就允许 air_state 抖动提前切 LAND -> 腿停在半路(0.25 左右)落地。 */
volatile uint16_t jump_shrink_min = 150;     /* SHRINK 最小驻留 ms (>收腿全程) */
/* 【v13 2026-11-19】SHRINK 出口只看腿的三条参数 */
volatile float    jump_shrink_done_vth  = 0.15f; /* m/s, |ldot|<此值 才算收腿停住 */
volatile uint16_t jump_shrink_stuck_ms  = 20;    /* ms, 计划在收却收不动 -> 判落地 */
volatile uint16_t jump_shrink_stuck_t   = 0;     /* 观测: 收不动连续拍数 */
/* ===== 【2026-11-19 v7】收腿前置门: 必须【真离地】才允许收腿 =====
 * 实测 202628(703 行) 抓到的原始问题:
 *   PUSH 在 jump_push_max_len(0.3602) 正常收工(abort=2), 那一刻轮子还压在地上
 *   —— 整段 SHRINK 的 grf_dbg.air_state 全程 = 0, 车【从来没离地】。
 *   而 SHRINK 第一拍 Fleg 就从 +201N 砸到 -80.25N(双腿 -562N ≈ -4.8g):
 *     车身上升速度(轮子在地上时 = 腿长速度)66ms 内从 +1.50 归零,
 *     最高只到腿长 0.378(比站立高 128mm)就落回。
 *   后果: 三角斜坡 177ms 走完到 0.24, 腿还在 0.334/0.358(落后 106mm),
 *        air_grf 恒 0 -> 出口走 !air_grf 分支 -> 带着 90mm 落差错位进 LAND。
 *
 * 物理: 轮子还在地上时给腿负力 = 给车身踩刹车, 是【反跳】。
 *   想起跳, 这一瞬间腿力必须≈0: 让车身带着腿按自己的动量继续上升,
 *   轮子卸载(GRF<30N) -> grf_air_hold_off(100ms) 后 GRF_Airborne() 置 1
 *   -> 这时才收腿。
 *
 * jump_shrink_wait: 等离地的最大时长 ms。超时就认输, 不硬收腿, 直接切 LAND。
 * jump_shrink_airok: 观测: 1 = 已确认真离地(air_grf=1), 收腿正式开跑。
 * jump_shrink_hold : 观测: 1 = 正在零力滑行等离地(此时 Fleg≈0, air 不强制)。 */
volatile uint16_t jump_shrink_wait  = 150;
volatile uint8_t  jump_shrink_airok = 0;
volatile uint8_t  jump_shrink_hold  = 0;
volatile int16_t  jump_rc_th      = -400;    /* 左杆阈值 */
volatile uint16_t jump_squat_hold = 100;     /* 蹲到位后再等多久才授权 ms
                                              * 【2026-11-14】300 -> 100。原 300ms 叠上
                                              * 蹲下冲过头后的回弹恢复(~200ms) 让 SQUAT 相位
                                              * 拖到 762~788ms, 超过用户 550~700ms 的按压时长,
                                              * 现场表现: 腿砸下去、弹两下、就一直不动卡在下面。 */
volatile uint16_t jump_land_hold  = 250;     /* 落地稳定判定 ms */
/* 【2026-11-18 v4.2】松手去抖 ms —— 必须加, 否则遥控丢一帧就误起跳。
 * 时序: SBUS 每帧 14ms; RemoteControlRxCallback 里只要帧头带 
 *   丢帧/failsafe 标志(b[23]&0x0c), 整包遥控数据判离线, 左摇杆值被写成 0。
 * 而 SQUAT 的松手判定是【一拍就生效】: 只要 squat_req=0 且 蹲好标志=1,
 *   立刻进蹬伸 —— 实测现象就是“按住杆不动, 车自己突然蹬腿把腿伸到 0.36”。
 * 取 100ms(约 7 帧 SBUS): 能滤掉单帧/双帧丢失, 又不影响正常松手手感。 */
volatile uint16_t jump_release_db  = 100;    /* ms, 松手去抖时间 */
volatile uint16_t jump_squat_rel_t = 0;      /* 观测: 已连续松手拍数 */
/* 【2026-11-14 六改·已撤销】蹲下期 Tw 行「摆角列」保留系数。
 * ⚠️ 置 0 会导致轮子打滑失控, 保持 1.0。
 * 实测(212808 日志): 蹲下期把 Tw 的 lθ/ldθ/rθ/rdθ 四列置 0 后 ——
 *   lqr_u[0] 冲到 -9.15N·m, Tw_cmd 打满 -4.90, wheel_vel 到 -1004 rad/s
 *   (≈9585RPM, M3508 极限), s 退了 1814mm, 轮子全程打滑空转。
 * 原因: LQR 的摆角列不是「可选辅助」, 它是闭环稳定通道的一部分 ——
 *   腿长变化会产生真实的水平分力, LQR 用轮子提前补偿; 把它拆掉后这个力
 *   没人管 -> 车被推走 -> 轮子空转 -> ds 变大 -> LQR 出更大力矩刹车 ->
 *   轮子打滑转得更快 -> ds 更大 …… 正反馈直到 Tw 打满。
 * 所以「蹲下后退」只能从腿角目标 / 机械几何上治, 不能砍 LQR 通道。 */
volatile float    jump_squat_tw_theta_scale = 1.0f;  /* 蹲下期 Tw 摆角列保留比例, 恒为 1.0 */
/* 【2026-11-16】PUSH 期 Tp 行 φ|dφ 列缩放（治“机身与摆角打架”）。
 * 稳态实测: 右腿歪 -6.6° 时, rθ 项给 +1.161 (想拉回),
 *   但 φ 项 (K[3][8]=-21.6) 立刻抵消 -0.888 (机身仰 -2.35°), 净剩 +0.273,
 *   远小于右腿要对抗的支持力 1.80 -> 回不来。且 Tp 共模又去撬机身 -> 自激。
 * PUSH 只 240ms 且马上离地, 机身歪几度可以接受 —— 这里放松对 φ 的执着,
 * 让髋电机专心伸直 (K[2][4] 腿刚度一个字不改)。
 * 1.0 = 原行为; 在线扫 1.0->0.7->0.5->0.3 找最佳。 */
volatile float    jump_push_tp_phi_scale = 1.00f;
/* ===== 腿角差模拉平（2026-11-17, 取代旧“扶正助推”）=====
 * 【为什么只做差模】l_theta 是世界系腿角 = 腿相对机体角 + φ (balance.c:96)。
 *   共模(两腿同向) Tp 只是把腿相对机体转 +Δ, 机身同时仰 -Δ, 世界系读数不变。
 *   仿真: 共模 +8N·m -> 机身仰 19.4°, 腿角只动 0.01°, 轮子后退 232mm —— 纯亏。
 *   差模(两腿反向) 净力矩为零 -> 机身不动, 腿各转各的:
 *         差模 +1/-1N·m -> 左 +2.689° / 右 -2.689°, 机身 0.000°。
 *   所以“把两腿平均值拉到 0”用 Tp 不可能做到, 只有“把两条腿拉成一样”可行。
 *
 * 【稳态实测 13:29:00.632】l_theta=+3.34°, r_theta=-6.62° -> dθ=+9.96°,
 *   拆到机身坐标系: 相对机体 +5.70°/-4.26° (均值仅 +0.72°, 腿本身几乎是直的),
 *   -1.64° 的世界系均值里有 -2.36° 是机身仰出来的, 不归腿管。
 *
 * 【控制律】dθ = θ_l - θ_r
 *            Td = -k · (|dθ| - db) · sgn(dθ)     软死区, 连续不起振
 *            l_Tp += Td;  r_Tp -= Td            反号 -> 净力矩 0, 绝对不撬机身
 * 【实测标定 2026-11-17 20:11】k=5 站立 57s: dθ 由 +9.96° 降到 +4.33°(标准差仅
 *   0.21°, 完全不抖), lθ +3.34°->+0.11° 基本归零, rθ -6.62°->-4.22°。
 *   等效差模扰动力矩 D≈1.26N·m, LQR 自带差模刚度 G0≈7.2N·m/rad -> k=5 时
 *   总刚度 17.2 仍小于支撑力差模刚度 N·l≈14.3, 故停在 4.33°。
 * 临界点: k_crit = N·l/2 ≈ 7.2。平衡解 dθ = k·db/(k - 7.17):
 *   k=10 -> ~2.0°, k=15 -> ~1.1°, k=20 -> ~0.9° (极点 ζ 分别 0.68/0.61/0.55)
 * 只在 JP_IDLE 生效: 跳跃 SQUAT/PUSH/SHRINK/LAND 期间腿角本就在按规划变。 */
volatile float theta_lvl_k   = 15.0f;   /* N·m/rad, 差模比例增益(站立工况标定值) */
volatile float theta_lvl_db  = 0.01f;   /* rad, 软死区(0.57°), 已拉平就不再出力 */
volatile float theta_lvl_max = 15.0f;   /* N·m, 单腿差模力矩上限 */
volatile float theta_lvl_scl = 1.0f;    /* 倍率, 观测: 按「腿力x腿长」自适应算出的 k 倍率 */
/* 【2026-11-21】自起/腿长目标调试镜像。
 * u8 直读会偏到 addr+4(summary 里 multiple 次实测), 所以全部用 float。
 * dbg_prone_state_f : 自起状态机当前态。跳跃状态机只在 == PRS_DONE(3) 时才进,
 *   其他值下 jump_ph 恒为 IDLE -> 表现为“拉杆没反应 / 不跳”。
 * dbg_leg_tgt_l/r  : 腿长目标。与实测腿长对比即知位置环到底在往哪个方向拉。
 * dbg_out_gate     : 输出闸(1=电机全零)。态0(PRS_DETECT)与态7未武装时为 1。 */
volatile float dbg_prone_state_f = 0.0f;
volatile float dbg_leg_tgt_l     = 0.0f;
volatile float dbg_leg_tgt_r     = 0.0f;
volatile float dbg_out_gate      = 0.0f;
volatile float theta_lvl_dth = 0.0f;    /* rad, 观测: 当前 θ_l - θ_r */
volatile float theta_lvl_dbg = 0.0f;    /* N·m, 观测: 实际施加的差模力矩(左腿 +, 右腿 -) */
volatile uint8_t  jump_ph    = JP_IDLE;
volatile uint16_t jump_t     = 0;
volatile float    jump_squat_ramp = 0.30f;
volatile uint8_t  jump_ready = 0;
static   uint16_t jump_land_total = 0;   /* LAND 段不回零的总计时(防 air 锁死卡相位) */
static uint8_t rec6_anchor = 0;       /* 通用进入沿标志(段内共享, 段出口清零) */
static uint8_t lift_anchor = 0;       /* 态5 起立斜坡锚定标志(文件级: 零力进入沿在前段访问) */
static uint8_t lift_top = 0;          /* 态5 起立双腿到顶标志(2026-11-16 合并: 到顶沿重锚挡位斜坡, 与 lift_anchor 同批清零) */

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

/* ===== T2 GRF 悬空门控（2026-11-03 改手册第九章降维）=====
 * 悬空时 LQR_SetAir(1): K 矩阵轮相关值清零——Tw 整行清零,
 * Tp 跳过 s|ds|yaw|dyaw 列, 腿被 LQR 主动锁在目标位(§10.3)。
 * 输出端 Tw 再硬清一道双保险, 见 ChassisTask 悬空分支。 */

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

/* Ldot 估计：DM 反馈 500Hz，1kHz 循环里差分隔拍为 0，
 * 必须按"上次变化以来的累计时间"归一，否则均值少一半。 */
static float LegVelEst(float L, LegVelState_s *st)
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
    else if (st->dt_acc > LEG_VEL_HOLD_S)
    {
        /* 连续 LEG_VEL_HOLD_S 没有任何变化 -> 腿真的停了, raw 才归零。
         * 【2026-11-14】旧写法这里无条件 raw=0, 而电机反馈 500Hz / 主循环 1kHz,
         * 喂给低通的是 [v,0,v,0...] 脉冲串, 稳态滤波值只剩 0.46v,
         * 阻尼实际只有 92 而不是 200(阻尼比 0.49, 欠阻尼) -> 蹲下冲过 29mm。
         * 新数据未到的拍保持上一拍的值(零阶保持)。 */
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

void ChassisTask()
{
    ChassisDbgUpdate();
    /* 使能帧只在 DMMotorInit/DMMotorEnable 里发一次。原来在这里每 5ms 发两帧
     * DM_CMD_MOTOR_MODE，既占 CAN 带宽又打断 MIT 帧时序，已移除。 */
    SetPowerLimit(1000000.0f);   /* @Todo 接裁判系统后换成真实底盘功率 */

    /* ===== 接收 cmd 层指令 + 零力急停路径 =====
     * 边沿触发: Stop/Enable 只置标志,电机输出任务持续按标志发零/正常帧。 */
    SubGetMessage(chassis_cmd_sub, (void *)&chassis_cmd_recv);
    static uint8_t zero_force_last = 0;
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
    /* 遥控挡位腿长斜坡跟踪(2026-11-16 合并同事 2026-11-15 版): 目标<0.1=未配置
     * (保持 rec_l_rise, 兼容 NONE 编译/未接遥控), 其余 clamp [0.15,0.40]
     * (K 带全覆盖上界, 高于机械底)。放此处保证先于下方状态机取值。
     * ⚠ 跳跃 SQUAT/PUSH 段本块仍跑, 但状态机随后会覆写腿长目标 -> 不影响跳跃。 */
    {
        float leg_len_cmd = chassis_cmd_recv.leg_length_target;
        if (leg_len_cmd < 0.1f)
            leg_len_cmd_ramp = rec_l_rise;
        else
        {
            LIMIT_MIN_MAX(leg_len_cmd, 0.15f, 0.40f);
            if (leg_len_cmd_ramp < leg_len_cmd)
            {   leg_len_cmd_ramp += leg_switch_rate;
                if (leg_len_cmd_ramp > leg_len_cmd) leg_len_cmd_ramp = leg_len_cmd; }
            else if (leg_len_cmd_ramp > leg_len_cmd)
            {   leg_len_cmd_ramp -= leg_switch_rate;
                if (leg_len_cmd_ramp < leg_len_cmd) leg_len_cmd_ramp = leg_len_cmd; }
        }
    }
    /* 变腿长时 yaw 模型系数跟随当前生效目标(上一拍值, 慢变斜坡无妨)联动,
     * 防 2026-11-05"改站立腿长忘同步偏航模型"同坑。 */
    float yaw_coef_leg = (WHEEL_RADIUS + ten_targets.l_leg_length_target) / (2.0f * TRACK_HALF_WIDTH);
    if (!use_imu_yaw)
    {
        ten_balance_observe.yaw = YAW_COEF_WHEEL * (r_wheel.r_x - l_wheel.l_x)
                                + yaw_coef_leg * (ten_balance_observe.l_theta - ten_balance_observe.r_theta);
        ten_balance_observe.dyaw = YAW_COEF_WHEEL * (r_wheel.r_dx - l_wheel.l_dx)
                                 + yaw_coef_leg * (ten_balance_observe.l_dtheta - ten_balance_observe.r_dtheta);
    }
    ChassisDbgLqr(&ten_balance_observe);

    /* 轮使能沿重锚定 s/yaw 已并入态5 进入沿(直接写目标, 不再走请求标志)。 */

    /* ===== 遥控目标生成(cmd层意图 → LQR目标) =====
     * 行驶(CHASSIS_WHEELLEG_DRIVE): vx>阈值→控速(s_target跟随s), 松杆→定点(s_target冻结);
     *                               wz 积分为 yaw_target,松杆冻结→航向锁定。
     * 锁定(CHASSIS_NO_FOLLOW等): 进入瞬间冻结当前 s/yaw 为目标 → 原地定住。
     * 上电首拍锁定即冻结,此时 s≈0,yaw≈0,与站立调试的硬编码基线等价。
     * 恢复流程态1~4/态6: 本块目标随后被状态机覆盖(模式锁存照常更新); 态5 起本块
     * 接管(θ 收零/重锚完成前状态机仍短暂写)。腿长目标全程由状态机置位。 */
    uint8_t air_state = GRF_Airborne(); /* T2 悬空快照,PD 段期间也有效(轮输出端共用) */
    /* 【2026-11-18 v3.2】原始 GRF 判定留一份 —— 收腿段的【出口】只能用这个。
     * 否则下面把 air_state 强置 1 后, 出口的 !air_state 永远不成立 -> 只能靠 800ms 超时。 */
    uint8_t air_grf   = air_state;
    dbg_air_grf_f = (float)air_grf;   /* 【2026-11-21】调试镜像: GRF 原始悬空标志 */
    /* 自起恢复段(态1~4/态6)强制接地: GRF 阈值只在站立位形标定过, 趴/蹲/顶升位形
     * F_n 观测未标定, 15:3x 实测塌回趴地后误判悬空 → Tw 清零+Tp 降维,
     * LQR 弱到站不起来。态5(完成)恢复 —— 运行中悬空保护必须在线;
     * 拉起过程若再遇误判由 T3 出窗监控兜底。 */
    if ((prone_rec_state >= PRS_EXTEND && prone_rec_state <= PRS_LQR_RAMP)
        || prone_rec_state == PRS_QUICK)
        air_state = 0;
    /* 跳跃蹲伏段同样强制接地: 蹲下时 N=mg-ma, 腿力会低于 air_th_off,
     * 实测误判悬空 -> Tw 被清零 700ms(轮子完全没力)。 */
    if (jump_ph == JP_SQUAT) air_state = 0;
    /* 【2026-11-18 v3.2】收腿段强制悬空 —— 必须抢在 GRF 的 100ms 之前。
     * 物理: 切 SHRINK 那一拍腿力从 +200N 断崖到 -100N(300N 阶跃), 轮子瞬间失压,
     *   车在这里就离地了。但 GRF 要连续 100ms F_n<30N 才置 1(grf_air_hold_off=100),
     *   而这 100ms 恰好盖住收腿加速段(0~68ms)。
     * 不强制的后果(三条全中):
     *   ① LQR_SetAir(0) -> 全维, 轮子通道没清 -> 空中空转污染里程
     *   ② air_kp_theta PD 没接管 -> 腿角还在用 LQR 的 Tp, 降维也没做
     *   ③ dF_roll 没清零 -> 无支撑面的纯扰动, 机身被推
     * 只有 why<=2(真离地/行程走完)才会进 SHRINK(PUSH 退出已分流), 所以无条件强制。
     *
     * 【2026-11-19 v7】不能再「无条件」了 —— 实测 202628 证明 why=2(行程走完)时
     *   车【还在地上】。在地上把 air_state 强置 1 的代价:
     *     ① LQR_SetAir(1) -> 轮子通道清零 -> 车在地上却不管轮速
     *     ② air_state 块把 dF_roll 清零 -> 侧倾补偿被砍
     *     ③ air_kp_theta PD 接管腿角 -> 用空中逻辑控一条压在地上的腿
     *   现在改成「只有真离地(air_grf=1)才强制」: 等离地期间 air_state 保持 GRF 原值,
     *   轮子平衡和侧倾补偿照常在线, 等 jump_shrink_hold 段滑行完再接管。 */
    if (jump_ph == JP_SHRINK && jump_shrink_airok) air_state = 1;
    LQR_SetAir(air_state);              /* 悬空降维同步给 LQR(手册ch9: 轮相关通道清零) */
    { /* 目标生成总跑, 态1~4/态6 由状态机随后覆盖; 态5 起归目标生成接管
       * (θ 收零/起立斜坡/重锚完成前状态机仍短暂写) */
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
        if (fabsf(chassis_cmd_recv.vx) > RC_DRIVE_ACTIVE_VX)
        {
            ten_targets.ds_target = chassis_cmd_recv.vx;
            ten_targets.s_target  = ten_balance_observe.s;
        }
        else
        {
            ten_targets.ds_target = 0.0f;
        }
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
        /* 双腿到顶沿(2026-11-16 合并同事 2026-11-15 版): 挡位斜坡重锚 rec_l_rise
         * 防阶跃, 之后平衡/行驶/锁定的维持目标由遥控挡位接管
         * (未配置时 ramp 恒 rec_l_rise, 行为与旧版逐拍恒 rec_l_rise 完全一致)。 */
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
    dbg_len_l = ten_targets.l_leg_length_target;  /* ep 可读: 状态机当前腿长目标 */
    dbg_len_r = ten_targets.r_leg_length_target;

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

    /* ===== 跳跃状态机（2026-11-11, 2026-11-14 并入新自起底）=====
     * 放在自起状态机之后: 自起在 PRS_DONE 下每拍写腿长目标(lift_l),
     * 跳跃必须后写才能覆盖。
     * 回 IDLE 时把 lift_anchor 清零, 让自起态5 的起立斜坡从实测重锚 ——
     * 否则腿长目标会从 0.15 直接阶跃回 rec_l_rise, 轮子瞬时卸载、车身后退
     * (实测每次后退 96~238mm)。 */
#if JUMP_EN
    /* 【v16 2026-11-20】腿长/腿张角保护总开关 —— 只在跳跃期间启用。
     * 放在状态机之前、无条件每拍更新(即使 prone_rec_state 掉出自起也会清)。
     * leg_guard_en 在 balance.c 里消费: 0 = 完全旧行为(自起/站立/行驶不受影响),
     * 1 = 启用腿张角钳位 + 腿长跳变保护。
     * 差一拍生效(BalanceObserve 在状态机之前调用), 无影响。 */
    leg_guard_en = (jump_ph != JP_IDLE) ? 1u : 0u;
    /* 【2026-11-21】PUSH 期抬高 GRF 腿力判据的长度门槛(详见 grf_observer.c 注释)。
     * PUSH 末段的 taper 是主副卸力, 不能当离地信号用。 */
    grf_fleg_use_push_arm = (jump_ph == JP_PUSH) ? 1u : 0u;

    if (jump_enable && prone_rec_state == PRS_DONE)
    {
        /* 【2026-11-18】jump_debug_squat=1 时强制当“一直拉着杆”, 绕开遥控链路。
         * 遥控 SBUS 帧的 b[23]&0x0c(丢帧/failsafe) 会让整个遥控数据判离线,
         * rc_left_x 被每帧写成 0 -> squat_req 永远是 0 -> 蹲伏进不去。 */
        uint8_t squat_req = jump_debug_squat ? 1u : (rc_left_x < jump_rc_th);

        switch (jump_ph)
        {
        case JP_IDLE:
            if (squat_req)
            {
                jump_ph = JP_SQUAT; jump_t = 0; jump_ready = 0;
                /* 斜坡起点 = 当前实测腿长, 避免第一步误差过大 */
                jump_squat_ramp = 0.5f * (ten_balance_observe.l_leg_length
                                        + ten_balance_observe.r_leg_length);
                jump_squat_vref  = 0.0f;   /* 参考速度从 0 起步, 防阻尼阶跃 */
                jump_squat_vramp = 0.0f;   /* 斜坡速度从 0 起步, 才能限加速度爬上 */
                jump_squat_acc_now = 0.0f; /* 加速度前馈从 0 起步 */
                jump_squat_rel_t = 0;      /* 【v4.2】松手去抖计数清零 */
                /* 【v14 2026-11-20】跳过 GRF 上电死延时。
                 * 死延时(grf_air_delay=3000ms)是给"上电垫高/自起"用的,
                 * 跳跃时人已站好, 这 3s 静默期会把整个跳跃窗口吃掉:
                 * 离地判据在 grf_air_update 开头就被 return 掉, 永远不置 air_state。
                 * 实测 2026-11-19 20:26: 收腿起点 8.109, air 到 8.906 才=1 (晚 797ms)。 */
                GRF_JumpArm();
            }
            break;

        case JP_SQUAT:
            {
                /* 【2026-11-14 三改】斜坡也限加速度 -> 真正的 S 形速度曲线
                 *
                 * 二改的问题: 只给参考速度 jump_squat_vref 限了加速度,
                 *   斜坡位置 jump_squat_ramp 第 1 毫秒就以 jump_squat_rate 走,
                 *   等于给腿一个“速度阶跃”。腿从 0 加速到 0.6 要 67ms(按 9m/s²),
                 *   这 67ms 里斜坡跑 40mm、腿只跑 20mm -> 永久落后 20mm。
                 *   实测 12:05:19.542: 斜坡 0.18865 / 腿 0.20855, 落后 19.9mm, 对上了。
                 *   之后位置环让腿超速去补这 20mm, 追上时刚好到 0.15,
                 *   速度还有 0.57m/s, 刹车要 17mm 却没行程 -> 冲过 17.7mm。
                 *
                 * 三改: 让斜坡速度 jump_squat_vramp 自身按 jump_squat_acc 限加速度。
                 *   速度曲线 = 加速(67ms/20mm) + 匀速(95ms/57mm) + 刹车(67ms/20mm)
                 *   腿的加减速能力和斜坡一致 -> 不落后 -> 追上位置提前 ->
                 *   后面全是刹车行程 -> 到达 0.15 时速度接近 0。
                 *
                 * jump_squat_vlim  : 目标速度上限(刹车曲线), 正值, 仅观测
                 * jump_squat_vramp : 斜坡实际速度(已限加速度), 正值, 驱动斜坡
                 * jump_squat_vref  : 速度参考 = -vramp, 给阻尼项用 */
                float remain = jump_squat_ramp - jump_squat_len;   /* m, 离终点还差多少 */
                float vlim   = 0.0f;                                /* m/s, 目标速度上限 */
                float dvmax  = jump_squat_acc * 0.001f;             /* m/s, 每拍允许的速度变化 */
                float vold   = jump_squat_vramp;                    /* m/s, 本拍未更新前的斜坡速度 */

                if (remain > 0.0f)
                {
                    vlim = sqrtf(2.0f * jump_squat_acc * remain);   /* 刹车曲线: 越近越慢 */
                    if (vlim > jump_squat_rate) vlim = jump_squat_rate;
                }

                /* 斜坡速度按 jump_squat_acc 爬升/下降 —— 这一步就是“斜率变化” */
                if      (jump_squat_vramp < vlim - dvmax) jump_squat_vramp += dvmax;
                else if (jump_squat_vramp > vlim + dvmax) jump_squat_vramp -= dvmax;
                else                                      jump_squat_vramp  = vlim;
                if (jump_squat_vramp < 0.0f) jump_squat_vramp = 0.0f;

                /* 【2026-11-14 四改】斜坡当前加速度, 正=收腿加速, 负=刹车。
                 * 供加速度前馈使用(在下方 grav_ff 处)。 */
                jump_squat_acc_now = (jump_squat_vramp - vold) * 1000.0f;  /* m/s² */

                /* 斜坡按限加速度后的速度步进 */
                jump_squat_ramp -= jump_squat_vramp * 0.001f;
                /* 【2026-11-14 修 bug】旧写法无条件钳位, 若起步时腿已低于目标
                 * (如落地后还没升回 0.25 就再按杆), 会把 ramp 硬抬到 0.15
                 * -> “蹲下”变成向上伸腿。只在从上方靠近时才钳。 */
                if (remain > 0.0f && jump_squat_ramp < jump_squat_len)
                    jump_squat_ramp = jump_squat_len;

                jump_squat_vlim = vlim;                /* 观测: 目标速度上限 */
                jump_squat_vref = -jump_squat_vramp;   /* 参考速度, 负=收腿 */
            }
            /* ===== 进 PUSH 的判定（2026-11-16 加“速度为 0”判据）=====
             * 两个条件必须同时满足才算“蹲稳了”:
             *   ① 位置到位【len_ok】: 左右腿【实测腿长】都在【蹲伏目标 jump_squat_len=0.15m】
             *      的【判定带 jump_squat_band=0.015m】以内。
             *   ② 速度静止【still_ok】: 左右腿【腿长速度 chassis_dbg.Ldot】绝对值都小于
             *      【静止阈值 jump_squat_v0=0.05m/s】。
             * 本拍两条都满足 -> 【计时 jump_t】+1; 任一条破 -> 计时清零重来。
             * 连续满足满【jump_squat_hold=100ms】-> 置【jump_ready】允许起跳。 */
            uint8_t len_ok   = (fabsf(ten_balance_observe.l_leg_length - jump_squat_len) < jump_squat_band)
                            && (fabsf(ten_balance_observe.r_leg_length - jump_squat_len) < jump_squat_band);
            uint8_t still_ok = (fabsf(chassis_dbg.Ldot[0]) < jump_squat_v0)
                            && (fabsf(chassis_dbg.Ldot[1]) < jump_squat_v0);
            if (len_ok && still_ok)
            {
                if (jump_t < 0xFFFE) jump_t++;
                if (jump_t >= jump_squat_hold) jump_ready = 1;
            }
            else { jump_t = 0; jump_ready = 0; }

            /* 【v4.2】松手去抖: 必须连续 jump_release_db 毫秒读到松手才算真松手。
             * 不加的后果: 遥控丢一帧(14ms) -> 左摇杆值被清零 -> 蹲好标志已置 1
             * -> 立刻蹬腿 -> 腿瞬间伸到 0.36 但车没起飞 -> 异常退出 -> 腿长卡在高位。
             * 这就是“明明没松手, 车自己站起来了”的根因。 */
            if (squat_req)
                jump_squat_rel_t = 0;
            else if (jump_squat_rel_t < 0xFFFF)
                jump_squat_rel_t++;

            if (squat_req)
            {
                /* 还按着杆: 继续蹲等 */
            }
            else if (jump_squat_rel_t < jump_release_db)
            {
                /* 去抖窗口内: 当作还按着 —— 保持蹲姿, 既不蹬伸也不取消 */
            }
            else if (jump_ready)
            {
                jump_ph = JP_PUSH; jump_t = 0;      /* 蹲好 + 真松手 -> 起跳 */
                jump_push_ramp = jump_squat_len;    /* 斜坡从蹲伏位起步 */
                jump_push_vramp = 0.0f;             /* 速度从 0 起步, 才能限加速度爬上 */
                jump_push_vref  = 0.0f;
                jump_push_acc_now = 0.0f;
                jump_push_imp   = 0;                /* 【v4】基础段起步 */
                jump_push_imp_t = 0;
                jump_push_imp_k = 0.0f;
                jump_push_stuck_t  = 0;             /* 【v4】卡住计时清零 */
                jump_push_len_last = jump_squat_len;/* 【v4】卡住检测基准 */
            }
            else
            {
                jump_ph = JP_IDLE; jump_t = 0;      /* 没蹲好就松手 = 取消 */
                lift_anchor = 0;                    /* 回站立: 腿长从实测重锚 */
                lift_top = 0;                       /* 挡位接管沿同批清零 */
                /* 【v4.1】腿可能已经蹲了一半, 置回归标志让它滑回站立 */
                jump_idle_ramp   = 0.5f * (ten_balance_observe.l_leg_length
                                         + ten_balance_observe.r_leg_length);
                jump_idle_return = 1;
            }
            break;

        case JP_PUSH:
        {
                jump_push_len_avg = 0.5f * (ten_balance_observe.l_leg_length
                                          + ten_balance_observe.r_leg_length);
                float len_dif = fabsf(ten_balance_observe.l_leg_length
                                    - ten_balance_observe.r_leg_length);

                /* ===== 本拍重算两段参数(在线改 jump_push_h 立即生效) ===== */
                float MT       = 2.0f * jump_push_mass;      /* kg, 整车质量 */
                /* 总起跳速度需求(理想跳高 = 目标/效率) */
                float v_total  = sqrtf(2.0f * 9.81f * jump_push_h / jump_push_eta);
                /* 冲量段贡献的 Δv²
                 * 【v10 2026-11-19】考虑尾部卸载后的分段积分:
                 *   恒力段: dv² = 2·a_imp·(dx_imp - dx_tap)
                 *   卸载段: a(u) = a_imp - (F_imp/MT)·u, u 从 0 到 1
                 *           dv² = 2·dx_tap·(a_imp - F_imp/MT)
                 * 必须跟着改, 否则 v_base 会算大 -> 基础段冲过头。 */
                float dx_imp   = jump_push_max_len - jump_impulse_at;
                if (dx_imp < 0.005f) dx_imp = 0.005f;
                float a_imp    = (2.0f * jump_impulse_ff - MT * 9.81f) / MT;
                float dx_tap   = jump_impulse_taper;
                if (dx_tap < 0.0f)            dx_tap = 0.0f;
                if (dx_tap > dx_imp - 0.005f) dx_tap = dx_imp - 0.005f;
                float dv2_imp  = 2.0f * a_imp * (dx_imp - dx_tap)
                               + 2.0f * dx_tap * (a_imp - jump_impulse_ff / MT);
                /* 基础段只需这么多末速度 */
                float vb2      = v_total * v_total - dv2_imp;
                float v_base   = (vb2 > 0.0f) ? sqrtf(vb2) : 0.0f;
                /* 基础段全程匀加速, 正好在冲量点到位 */
                float dx_base  = jump_impulse_at - jump_squat_len;
                if (dx_base < 0.01f) dx_base = 0.01f;
                float a_base   = (v_base * v_base) / (2.0f * dx_base);
                if (a_base > jump_push_acc) a_base = jump_push_acc;

                jump_push_vtop  = v_total;
                jump_push_vbase = v_base;
                jump_push_abase = a_base;

                if (!jump_push_imp)
                {
                    /* ---------- 基础段: 位置控制, 匀加速 ---------- */
                    jump_push_vramp += a_base * 0.001f;
                    if (jump_push_vramp > v_base) jump_push_vramp = v_base;
                    jump_push_ramp += jump_push_vramp * 0.001f;
                    if (jump_push_ramp > jump_impulse_at) jump_push_ramp = jump_impulse_at;
                    /* 【2026-11-18 v4.3】干跑时参考速度/加速度前馈必须归零 ——
                     * 干跑把目标腿长锁在 jump_squat_len(0.15), 但 jump_push_vramp
                     * 还在按规划涨到 1.209。阻尼项 -kv(l_dot - vref) 看到
                     * “规划 1.209 / 实测 0.22” 就出 +99N 正力把腿硬拽长:
                     *   实测: 腿从 0.15 被拽到 0.22, 基础段目标 0.15 vs 实测 0.22,
                     *   位置环 -103N + 阻尼 +99N = +45N (日志 Fleg_cmd 42.28 完全对得上)
                     *   后果: jump_push_len_avg 永远到不了 0.32 -> 切不了冲量段
                     *        -> 500ms 超时退出(原因码 4), 看起来就是“压根没跳起来”。 */
                    if (jump_dry_run)
                    {
                        jump_push_vref    = 0.0f;
                        jump_push_acc_now = 0.0f;
                    }
                    else
                    {
                        jump_push_vref    = jump_push_vramp;
                        jump_push_acc_now = a_base;
                    }
                    jump_push_imp_k   = 0.0f;

                    /* 【实测】到位才切冲量 —— 不用规划值, 否则少 5.4mm 行程 */
                    if (jump_push_len_avg >= jump_impulse_at)
                    {
                        jump_push_imp   = 1;
                        jump_push_imp_t = 0;
                    }
                }
                else
                {
                    /* ---------- 冲量段: 力控制, 50->F_imp 爬升 ---------- */
                    if (jump_push_imp_t < 0xFFFF) jump_push_imp_t++;
                    {
                        float r = (float)jump_push_imp_t
                                / (float)((jump_impulse_ms > 0u) ? jump_impulse_ms : 1u);
                        if (r > 1.0f) r = 1.0f;
                        jump_push_imp_k = r;
                    }
                    /* PD 归零: 目标=实测+5mm 余量, vref=实测速度。力全靠前馈 */
                    jump_push_ramp    = jump_push_len_avg + 0.005f;
                    jump_push_vref    = 0.5f * (chassis_dbg.Ldot[0] + chassis_dbg.Ldot[1]);
                    jump_push_acc_now = 0.0f;
                }

                jump_t++;

                /* 【2026-11-18 v4 卡住检测】只在冲量段生效:
                 * 累计涨满 jump_push_stuck_dl 就重锚计时, 连续 stuck_ms 涨不到就判卡住。
                 * 基础段 v_base 可能为 0(跳得矮时规划本来就不动腿), 所以基础段只重锚不计时。 */
                if (jump_push_imp)
                {
                    if (jump_push_len_avg - jump_push_len_last >= jump_push_stuck_dl)
                    {
                        jump_push_len_last = jump_push_len_avg;
                        jump_push_stuck_t  = 0;
                    }
                    else if (jump_push_stuck_t < jump_push_stuck_ms)
                        jump_push_stuck_t++;
                }
                else
                {
                    jump_push_stuck_t  = 0;
                    jump_push_len_last = jump_push_len_avg;
                }

                /* ===== 退出判据(按优先级, 命中即走) =====
                 * 【2026-11-18 v4 优先级重排】「行程走完」提到最前。
                 * 旧顺序: 3(不对称) -> 2(行程) -> 1(离地) -> 5 -> 4。
                 * 问题: 腿都已经伸到 0.36 了说明起跳已经成立, 这时因为两腿差 40mm
                 * 判「单腿异常」退回 IDLE, 会把轮子通道恢复、腿角交给 LQR ——
                 * 人已经在空中了, 直接翻。
                 * 新顺序: 先认「已经跳出去(2)」和「GRF 确认离地(1)」, 再谈异常。 */
                uint8_t why = 0;
                /* 【v11 2026-11-19】出口必须叠加【真离地】。
                 * 物理: 摆角控制 / 轮速清零 / LQR 降维 这些「空中模式」的东西,
                 *   只能在确认真离地之后才开 —— 否则就是「人在空中、机器以为在地上」,
                 *   轮子追着 ds 空转把里程污染掉。
                 * 时序: 冲量段末段 jump_impulse_taper 已经把腿力卸到 0,
                 *   到 0.44 时 N = 2·0 + m_w·g = 9.8N -> 轮子卸载 ->
                 *   腿靠惯性继续伸到机械极限, 变成刚性杆把轮子顶离地 ->
                 *   100ms 后 air_state=1 -> 这时才切 SHRINK。
                 * 兜底: 一直不离地则由 jump_push_timeout 走 why=4 退回 IDLE(轮子照常控), 不会卡死。 */
                if      (jump_push_len_avg >= jump_push_max_len && air_state) why = 2; /* 蹬满 + 已离地 */
                else if (air_state)                                          why = 1; /* 提前离地 */
                else if (len_dif > jump_push_sym_th)                         why = 3; /* 单腿异常 */
                else if (jump_push_len_avg < jump_push_max_len
                         && jump_push_stuck_t >= jump_push_stuck_ms)         why = 5; /* 腿卡住(仅未蹬满时判) */
                else if (jump_t > jump_push_timeout)                         why = 4; /* 兜底 */
                jump_push_abort = why;
                if (why)
                {
                    /* 退出瞬间的平均腿速 ≈ 起跳速度, 算跳高 h=v²/(2g) 对照规划 */
                    jump_push_vlaunch = 0.5f * (chassis_dbg.Ldot[0]
                                              + chassis_dbg.Ldot[1]);
                    if (why <= 2)
                    {
                        /* 1=真离地 2=行程走完 -> 正常起跳, 进收腿段 */
                        /* 【v2】收腿斜坡从当前实测腿长起步, 避免目标跳变 */
                        jump_shrink_ramp = jump_push_len_avg;
                        /* 【v16 2026-11-20】两腿各自从自己的实测起步 */
                        jump_shrink_ramp_l = ten_balance_observe.l_leg_length;
                        jump_shrink_ramp_r = ten_balance_observe.r_leg_length;
                        jump_shrink_vramp_l = 0.0f;
                        jump_shrink_vramp_r = 0.0f;
                        jump_shrink_vref = 0.0f;
                        jump_shrink_vramp = 0.0f;      /* 【v3】斜坡速度从 0 起步才能限加速度 */
                        jump_shrink_acc_now = 0.0f;    /* 【v3】前馈从 0 起步 */
                        /* 【v7】进收腿段必须先等真离地, 见 jump_shrink_wait 注释 */
                        jump_shrink_airok = 0;
                        jump_shrink_hold  = 0;
                        jump_ph = JP_SHRINK; jump_t = 0;
                    }
                    else
                    {
                        /* 【2026-11-18 v3.2】3=两腿不对称 4=超时 5=腿卡住 -> 异常, 不进收腿段。
                         * 这三种大概率车根本没跳起来(腿没伸到位/被卡), 还在蹬地。
                         * 硬切 SHRINK 会把轮子力矩清零、腿角交给空中 PD、LQR 降维 ——
                         * 一旦真在地上就没有平衡了 -> 摔。
                         * 直接退回 IDLE: 相位清掉, 腿长目标保持 PUSH 最后一拍的规划值
                         * (IDLE 下没人写腿长目标, 会保持上一拍, 位置环自然稳在那里),
                         * 轮子平衡照常在线, 不会翻。
                         * 里程重锚一次: 蹬伸期轮子可能已被带得空转。 */
                        jump_ph = JP_IDLE; jump_t = 0;
                        lift_anchor = 0;             /* 腿长从实测重锚 */
                        lift_top    = 0;             /* 挡位接管沿重走 */
                        s_yaw_reanchor_ramp = 300;
                        /* 【v4.1】腿长停在 PUSH 中途(可能是蹲姿 0.15), 滑回站立 */
                        jump_idle_ramp   = 0.5f * (ten_balance_observe.l_leg_length
                                                 + ten_balance_observe.r_leg_length);
                        jump_idle_return = 1;
                    }
                }
            }
            break;

        case JP_SHRINK:
        {
            jump_t++;

            /* ===== 【2026-11-19 v7】收腿前先等【真离地】=====
             * 详见 jump_shrink_wait 注释。air_grf==0 期间:
             *   腿长目标锁实测均值 (位置环误差 = 0)
             *   vref = max(0, 实测腿速)  (上升时阻尼 = 0, 回落时恢复 -kv·l_dot 全量)
             *   grav_ff = 0              (给 50N 轮子永远卸不掉载, 永远等不到离地)
             *   本拍不判出口; 确认真离地时 jump_t 清零, 150ms 最小驻留从收腿真正开始时算。
             * 兜底: jump_shrink_wait 内没离地 -> 认输, 直接切 LAND(不硬收腿)。 */
            if (air_grf) jump_shrink_airok = 1;
            if (!jump_shrink_airok)
            {
                if (jump_t < jump_shrink_wait)
                {
                    float lavg = 0.5f * (chassis_dbg.Ldot[0] + chassis_dbg.Ldot[1]);
                    jump_shrink_hold    = 1;
                    jump_shrink_ramp    = 0.5f * (ten_balance_observe.l_leg_length
                                                + ten_balance_observe.r_leg_length);
                    /* 【v16】两腿各自锁自己的实测, 各自从 0 速起步 */
                    jump_shrink_ramp_l  = ten_balance_observe.l_leg_length;
                    jump_shrink_ramp_r  = ten_balance_observe.r_leg_length;
                    jump_shrink_vramp   = 0.0f;
                    jump_shrink_vramp_l = 0.0f;
                    jump_shrink_vramp_r = 0.0f;
                    jump_shrink_acc_now = 0.0f;
                    jump_shrink_vref    = (lavg > 0.0f) ? lavg : 0.0f;
                    jump_shrink_vref_l  = jump_shrink_vref;
                    jump_shrink_vref_r  = jump_shrink_vref;
                    break;
                }
                jump_shrink_hold = 0;
                jump_ph = JP_LAND; jump_t = 0; jump_land_total = 0;
                s_yaw_reanchor_ramp = 300;
                jump_land_ramp    = 0.5f * (ten_balance_observe.l_leg_length
                                          + ten_balance_observe.r_leg_length);
                jump_land_buf_done = 0; jump_land_buf_t = 0; jump_land_settle = 0;
                jump_land_touched  = 0; jump_land_gnd_t = 0;
                break;
            }
            if (jump_shrink_hold)
            {   /* 刚确认真离地: 驻留计时从这一拍重新算, 别被滑行段吃掉 150ms */
                jump_shrink_hold = 0;
                jump_t = 0;
            }

            /* 【v16 2026-11-20】三角速度曲线拆成【左右腿各一套】。
             * 旧版两腿追同一个绝对目标: 响应慢的那条永远追不上, 两腿越拉越开
             * (实测收腿末差 21mm -> 落地前撞顶时差 230mm)。
             * 现在两腿从自己的实测起点出发, 各走各的三角曲线, 各收到 jump_air_len。
             * 共享量 jump_shrink_ramp / jump_shrink_vramp 取两腿中的保守值,
             * 只用于出口判据和观测。 */
            float dvmax  = jump_shrink_acc * 0.001f;          /* m/s, 每拍允许的速度变化 */

            /* ---- 左腿 ---- */
            {
                float remain_l = jump_shrink_ramp_l - jump_air_len;
                float vlim_l   = 0.0f;
                float vold_l   = jump_shrink_vramp_l;
                if (remain_l > 0.0f)
                {
                    vlim_l = sqrtf(2.0f * jump_shrink_acc * remain_l);
                    if (vlim_l > jump_shrink_rate) vlim_l = jump_shrink_rate;
                }
                if      (jump_shrink_vramp_l < vlim_l - dvmax) jump_shrink_vramp_l += dvmax;
                else if (jump_shrink_vramp_l > vlim_l + dvmax) jump_shrink_vramp_l -= dvmax;
                else                                           jump_shrink_vramp_l  = vlim_l;
                if (jump_shrink_vramp_l < 0.0f) jump_shrink_vramp_l = 0.0f;

                jump_shrink_acc_l = (jump_shrink_vramp_l - vold_l) * 1000.0f;

                jump_shrink_ramp_l -= jump_shrink_vramp_l * 0.001f;
                if (remain_l > 0.0f && jump_shrink_ramp_l < jump_air_len)
                    jump_shrink_ramp_l = jump_air_len;
            }

            /* ---- 右腿 ---- */
            {
                float remain_r = jump_shrink_ramp_r - jump_air_len;
                float vlim_r   = 0.0f;
                float vold_r   = jump_shrink_vramp_r;
                if (remain_r > 0.0f)
                {
                    vlim_r = sqrtf(2.0f * jump_shrink_acc * remain_r);
                    if (vlim_r > jump_shrink_rate) vlim_r = jump_shrink_rate;
                }
                if      (jump_shrink_vramp_r < vlim_r - dvmax) jump_shrink_vramp_r += dvmax;
                else if (jump_shrink_vramp_r > vlim_r + dvmax) jump_shrink_vramp_r -= dvmax;
                else                                           jump_shrink_vramp_r  = vlim_r;
                if (jump_shrink_vramp_r < 0.0f) jump_shrink_vramp_r = 0.0f;

                jump_shrink_acc_r = (jump_shrink_vramp_r - vold_r) * 1000.0f;

                jump_shrink_ramp_r -= jump_shrink_vramp_r * 0.001f;
                if (remain_r > 0.0f && jump_shrink_ramp_r < jump_air_len)
                    jump_shrink_ramp_r = jump_air_len;
            }

            /* 共享量取保守值(剩余行程更多的腿) -> 出口判据看得到"还有腿没收完" */
            jump_shrink_ramp  = (jump_shrink_ramp_l > jump_shrink_ramp_r)
                              ? jump_shrink_ramp_l : jump_shrink_ramp_r;
            jump_shrink_vramp = (jump_shrink_vramp_l > jump_shrink_vramp_r)
                              ? jump_shrink_vramp_l : jump_shrink_vramp_r;
            /* 惯性前馈用较大加速度(保守: 不让任何一条腿少收) */
            jump_shrink_acc_now = (fabsf(jump_shrink_acc_l) > fabsf(jump_shrink_acc_r))
                                ? jump_shrink_acc_l : jump_shrink_acc_r;

            /* 【2026-11-19 v4】参考速度不能只跟斜坡 —— 斜坡送到 0.24 就归零,
             * 但腿往往还差 30~50mm、速度还有 -1.0m/s。旧写法 vref 归零后:
             *   阻尼 -kv·(ldot-0) 把收腿整个刹停。
             *   实测 t=150: P=-50.9N, 阻尼=+68.9N, 合计只有 +14.4N(方向还是往伸),
             *   腿停在 0.271/0.290 就再也不动了。
             * 现在 vref 取「斜坡速度」和「剩余误差能刹住的速度」中的较大者:
             *   还差 32mm -> sqrt(2·15·0.032) = 0.98m/s -> 阻尼不再误判, 腿继续收。
             *   到位后 lerr≈0 -> v_catch≈0 -> vref=-vramp=0 -> 阻尼恢复全量, 自然刹住。
             * 起步时 lerr≈0 -> v_catch≈0 -> vref=-vramp, 与旧版完全一致。 */
            {
                /* 【v15 2026-11-19】左右腿【各自】算参考速度, 不再用均值。
                 * 落后的那条腿 lerr 更大 -> v_catch 更大 -> vref 更负
                 *   -> 阻尼项 -kv(ldot - vref) 给出更大的收腿力 -> 自动追赶。
                 * 到位后 lerr≈0 -> v_catch≈0 -> vref=-vramp=0 -> 阻尼恢复全量, 自然刹住。 */
                float el = ten_balance_observe.l_leg_length - jump_air_len;
                float er = ten_balance_observe.r_leg_length - jump_air_len;
                float vcl, vcr;
                if (el < 0.0f) el = -el;   /* 绝对剩余量 */
                if (er < 0.0f) er = -er;
                vcl = sqrtf(2.0f * jump_shrink_acc * el);
                if (vcl > jump_shrink_rate) vcl = jump_shrink_rate;
                vcr = sqrtf(2.0f * jump_shrink_acc * er);
                if (vcr > jump_shrink_rate) vcr = jump_shrink_rate;
                /* 【v16 2026-11-20】必须用【各腿自己的】斜坡速度 vramp_l/r。
                 * 旧版这里用共享的 jump_shrink_vramp: 两腿斜坡拆开后,
                 * 共享值取的是较大者 -> 走得快的那条腿会把另一条腿的 vref
                 * 一并拉高 -> 阻尼项误判 -> 慢腿反而被顶住不动。 */
                jump_shrink_vref_l = -(jump_shrink_vramp_l > vcl ? jump_shrink_vramp_l : vcl);
                jump_shrink_vref_r = -(jump_shrink_vramp_r > vcr ? jump_shrink_vramp_r : vcr);
                /* 共用的那个取两者中较小的(保守), 仅给外部看/兼容 */
                jump_shrink_vref = (jump_shrink_vref_l > jump_shrink_vref_r)
                                 ? jump_shrink_vref_l : jump_shrink_vref_r;
            }

            /* 最小驻留: 先让收腿真正跑起来 */
            if (jump_t < jump_shrink_min)
                break;

            /* ===== 【v13 2026-11-19】SHRINK 出口改成【只看腿】, GRF 彻底退出 =====
             * 旧出口两条支路看的都是「会被收腿动作本身污染」的量:
             *   ① !air_grf: 三角曲线【减速段】acc_now = -15 ->
             *      grav_ff = -1.35×(-15) = +20.25N, 再加 PID 上限 +60
             *      -> F_leg 可到 +80N; 空中 N = F_leg > th_on(40N)
             *      -> 25ms 后 air 清 0 -> 误判落地。而 !air_grf 排第一条,
             *      一误判就立刻切 LAND, 腿还在 0.35 左右。
             *      (v13 已把减速段正前馈钳掉, 但 GRF 在收腿段仍不可信)
             *   ② shrink_at_tgt && ldot < -0.3: 收腿本身 ldot 就是 -1.4m/s,
             *      「斜坡走完」+「腿到位」同时成立时必然满足 -> 被当成"被地面顶住"。
             * 新出口三条都与收腿动作解耦:
             *   主判据: 腿【实测】到位(<= air_len+10mm) 且 腿速归零
             *   早落地: 计划以 vramp>0.3 收, 实际腿速接近 0 -> 被地面顶住
             *   兜底  : jump_t > 800ms
             * 斜坡没走完不切(早落地除外), 防收一半就切。 */
            {
                float ldot_avg  = 0.5f * (chassis_dbg.Ldot[0] + chassis_dbg.Ldot[1]);
                uint8_t do_land = 0;

                /* ③ 早落地: 该动却不动 = 被地面顶住。与收腿动作无关, 不受前馈污染。 */
                if (jump_shrink_vramp > 0.30f && ldot_avg > -0.20f)
                {
                    if (jump_shrink_stuck_t < 0xFFFFu) jump_shrink_stuck_t++;
                }
                else jump_shrink_stuck_t = 0;
                if (jump_shrink_stuck_t >= jump_shrink_stuck_ms)
                    do_land = 1;

                /* ② 主判据: 腿到位 + 腿速归零 + GRF 确认接地
                 * 【v14 2026-11-19】只判腿不行 —— 实测收完腿只用 250ms,
                 *   而滞空有 470ms: 从 t=250 到落地这 200ms 里,
                 *   腿长 = 0.24、ldot ≈ 0, 和"站在地上"完全一样,
                 *   光靠腿信号会在【空中】就判落地。
                 * 但这时可以重新信任 GRF: 斜坡已走完(ramp 钉在 0.24, 误差 0,
                 *   v_catch=0/vramp=0 -> vref=0), grav_ff = -1.35×acc_now = 0
                 *   -> F_leg ≈ 0 -> N ≈ 0, 收腿动作对 N 的污染已经消失。
                 * 空中: N≈0 -> air 保持 1 -> 不切;
                 * 触地: 腿要托住车身 -> F_leg≈57N -> N≈62 > th_on(40) -> 25ms -> air=0 -> 切。
                 * 所以腿到位是【必要】条件, GRF 是【确认】条件。 */
                if (!do_land
                    && !(jump_shrink_vramp > 0.02f && jump_t < 300))
                {
                    /* 【v16 2026-11-20】两腿都到位才算完成(旧版用均值, 一条到位
                     * 另一条还差 80mm 时均值也能过 -> 带差落地)。 */
                    uint8_t leg_ok_l = (ten_balance_observe.l_leg_length <= jump_air_len + 0.015f)
                                    && (fabsf(chassis_dbg.Ldot[0]) < jump_shrink_done_vth);
                    uint8_t leg_ok_r = (ten_balance_observe.r_leg_length <= jump_air_len + 0.015f)
                                    && (fabsf(chassis_dbg.Ldot[1]) < jump_shrink_done_vth);
                    uint8_t leg_done = leg_ok_l && leg_ok_r;
                    if ((leg_done && !air_grf) || jump_t > 800)
                        do_land = 1;
                }

                if (do_land)
                {
                    jump_ph = JP_LAND; jump_t = 0;
                    jump_land_total = 0;
                    /* 落地沿即时重锚: 轮子空中空转把 s 污染了 */
                    s_yaw_reanchor_ramp = 300;
                    jump_land_ramp = 0.5f * (ten_balance_observe.l_leg_length
                                           + ten_balance_observe.r_leg_length);
                    /* 【v4】缓冲段状态清零, 目标就是上面锤定的实测腿长(不再动) */
                    jump_land_buf_done = 0;
                    jump_land_buf_t    = 0;
                    jump_land_settle   = 0;
                    jump_land_touched  = 0;   /* 【v6】等真正触地再开始缓冲 */
                    jump_land_gnd_t    = 0;
                }
            }
        }
            break;

        case JP_LAND:
        {
            jump_t++;                    /* 总计时(缓冲+起立), 不因 air 抖动清零 */
            jump_land_total++;           /* 不回零的硬超时计数 */

            float ldot_avg = 0.5f * (chassis_dbg.Ldot[0] + chassis_dbg.Ldot[1]);
            float len_avg  = 0.5f * (ten_balance_observe.l_leg_length
                                   + ten_balance_observe.r_leg_length);

            /* ===== ① 缓冲段: 腿长目标【钉死不动】=====
             * 进 LAND 时 jump_land_ramp 已经锤定成实测腿长, 这里不能更新它 ——
             * 跟着实测走就恒零误差, 位置环不出力, 车以 mg/(2kv)=0.576m/s 一路沉到撞底。
             * 结束判据: 腿长速度回升(连续 10ms > -0.05) = 压缩到底, 或 200ms 兑底。 */
            if (!jump_land_buf_done)
            {
                if (air_state)
                {   /* 【v6】还没真正触地: 不计数, 目标给站立腿长(腿先在空中收好) */
                    jump_land_touched = 0;
                    jump_land_buf_t   = 0;
                    jump_land_gnd_t   = 0;
                    jump_land_ramp    = rec_l_rise;
                }
                else
                {
                    if (!jump_land_touched)
                    {   /* 【v6】落地沿: 以落地那一刻的实测腿长为缓冲起点 */
                        jump_land_touched = 1;
                        jump_land_ramp    = len_avg;
                        jump_land_buf_t   = 0;
                        jump_land_gnd_t   = 0;
                    }
                    if (jump_land_gnd_t < 0xFFFF) jump_land_gnd_t++;

                    if (ldot_avg > -jump_land_v0)
                    {
                        if (jump_land_buf_t < 10) jump_land_buf_t++;
                    }
                    else jump_land_buf_t = 0;

                    if (jump_land_buf_t >= 10 || jump_land_gnd_t >= jump_land_buffer)
                        jump_land_buf_done = 1;
                }
            }

            /* ===== ② 起立段: 目标以 jump_land_rate 斜坡回站立腿长 =====
             * 双向: 落地时腿可能比 rec_l_rise 长也可能短。
             * jump_land_vref 同步给出速度参考, 给阻尼去耦用(见 PID 段)。 */
            jump_land_vref = 0.0f;       /* 默认全阻尼(缓冲段/到位后) */
            if (jump_land_buf_done)
            {
                float lerr = rec_l_rise - jump_land_ramp;
                if (fabsf(lerr) > 0.0005f)
                {
                    float lstep = jump_land_rate * 0.001f;
                    if (lerr > lstep)
                    {   jump_land_ramp += lstep;
                        jump_land_vref  = +jump_land_rate; }   /* 正 = 伸腿 */
                    else if (lerr < -lstep)
                    {   jump_land_ramp -= lstep;
                        jump_land_vref  = -jump_land_rate; }   /* 负 = 收腿 */
                    else
                        jump_land_ramp  = rec_l_rise;          /* 到位: vref 保持 0 */
                }
            }

            /* ===== ③ 稳定判定 =====
             * 旧版只判 air_state, 腿还没伸回 0.25 就可能退出。
             * 新: 接地 + 腿长到位(±10mm) + 缓冲段已完成, 连续 jump_land_hold 才出口。
             * jump_land_settle 在 air_state=1 时清零(又弹起来了) —— 但 jump_t 不清零,
             * 所以旧版「弹起来 -> jump_t 归零 -> 前馈跳回峰值」的正反馈被切断了。 */
            {
                uint8_t len_ok = (fabsf(len_avg - rec_l_rise) < 0.01f);
                if (!air_state && len_ok && jump_land_buf_done)
                {
                    if (jump_land_settle < 0xFFFF) jump_land_settle++;
                    if (jump_land_settle >= jump_land_hold) jump_land_total = 2000;
                }
                else jump_land_settle = 0;
            }

            /* ===== ④ 退出 ===== */
            if (jump_land_total >= 2000)
            {
                jump_ph = JP_IDLE; jump_t = 0;
                s_yaw_reanchor_ramp = 300;
                lift_anchor = 0;
                lift_top = 0;             /* 落地回站立: 挡位接管沿重走 */
                jump_land_buf_done = 0;
                jump_land_buf_t    = 0;
                jump_land_settle   = 0;
                jump_land_touched  = 0;
                jump_land_gnd_t    = 0;
                /* 【v4.1】正常路径 ramp 已是 rec_l_rise, 但硬超时退出时可能没到位 */
                jump_idle_ramp   = 0.5f * (ten_balance_observe.l_leg_length
                                         + ten_balance_observe.r_leg_length);
                jump_idle_return = 1;
            }
        }
            break;

        default:
            jump_ph = JP_IDLE; jump_t = 0;
            jump_idle_ramp   = 0.5f * (ten_balance_observe.l_leg_length
                                     + ten_balance_observe.r_leg_length);
            jump_idle_return = 1;
            break;
        }

        /* 相位 -> 腿长目标覆盖 */
        if (jump_ph != JP_IDLE)
        {
            float jtgt = rec_l_rise;
            uint8_t split_lr = 0;   /* 【v16】1 = 两腿用各自的目标 */
            switch (jump_ph)
            {
            case JP_SQUAT:  jtgt = jump_squat_ramp; break;
            /* 干跑: PUSH/SHRINK 不再伸腿, 锁在蹲伏位 */
            case JP_PUSH:   jtgt = jump_dry_run ? jump_squat_len : jump_push_ramp;  break;
            case JP_SHRINK:
                /* 【v16 2026-11-20】两腿各用各的斜坡。
                 * 旧版两腿同一个 jtgt: 响应慢的腿位置误差一直更大却永远追不上,
                 * 实测落地前两腿差 230mm -> 一条腿撞顶 -> 整段崩。
                 * 现在各自从自己的实测起点走三角曲线, 各收到 jump_air_len。 */
                if (jump_dry_run)
                    jtgt = jump_squat_len;
                else
                {
                    jtgt = jump_shrink_ramp;
                    split_lr = 1;
                }
                break;
            case JP_LAND:   jtgt = jump_land_ramp;  break;
            default:        jtgt = rec_l_rise;      break;
            }
            if (split_lr)
            {
                ten_targets.l_leg_length_target = jump_shrink_ramp_l;
                ten_targets.r_leg_length_target = jump_shrink_ramp_r;
            }
            else
            {
                ten_targets.l_leg_length_target = jtgt;
                ten_targets.r_leg_length_target = jtgt;
            }
        }
        else if (jump_idle_return && prone_rec_state == PRS_DONE)
        {
            /* 【v4.1】跳跃退出后把腿长平滑拉回站立 —— 详见 jump_idle_ramp 注释。
             * 只用 jump_land_rate(0.3m/s), 与 LAND 起立段同一个速率, 手感一致。
             *
             * 【v4.4 2026-11-18】回位目标不能写死 rec_l_rise(0.25)。
             *   SW2 拨上位时挡位 leg_len_cmd_ramp = 0.35, 态5 的 lift_l 跟着它走。
             *   若仍按 0.25 收尾: jump_idle_return 清零那一拍, 态5 立刻把目标写回
             *   0.35 -> 100mm 阶跃弹车(现场「落完地突然窜高一截」就是这个)。
             *   改成取挡位当前值: SW2 中位(0.25)时行为与旧版逐拍一致, 不会变;
             *   上位时以 0.3m/s 平滑爬到 0.35, 无阶跃。
             *   lift_top==0(态5 起立斜坡还没到顶, 此时态5 写的是 lift_l 不是挡位)
             *   时仍用 rec_l_rise, 否则同样会阶跃。 */
            float idle_tgt = lift_top ? leg_len_cmd_ramp : rec_l_rise;
            float lerr = idle_tgt - jump_idle_ramp;
            if (fabsf(lerr) > 0.0005f)
            {
                float lstep = jump_land_rate * 0.001f;
                if      (lerr > lstep)  jump_idle_ramp += lstep;
                else if (lerr < -lstep) jump_idle_ramp -= lstep;
                else                    jump_idle_ramp  = idle_tgt;
            }
            else jump_idle_return = 0;   /* 到位, 交还给正常逻辑 */

            ten_targets.l_leg_length_target = jump_idle_ramp;
            ten_targets.r_leg_length_target = jump_idle_ramp;
        }
    }
    else if (jump_ph != JP_IDLE)
    {
        jump_ph = JP_IDLE; jump_t = 0; jump_ready = 0;   /* 掉出站立态 -> 强制复位 */
        lift_anchor = 0;
        lift_top = 0;                                     /* 挡位接管沿重走 */
    }
#endif

    /* ===== s/yaw 一次性重锚斜坡（2026-11-13 自旧版移植）=====
     * 摔地/跳跃落地时轮子空转会把里程计记脏, 用一个 300ms 指数斜坡把目标拉回实测,
     * 而不是直接赋值(直接赋值会让 Tw 阶跃)。系数 0.02 @1kHz -> tau=50ms。
     * 由状态机/跳跃机置 300, 此处递减。 */
    if (s_yaw_reanchor_ramp > 0)
    {
        s_yaw_reanchor_ramp--;
        ten_targets.s_target   += (ten_balance_observe.s   - ten_targets.s_target)   * 0.02f;
        ten_targets.yaw_target += (ten_balance_observe.yaw - ten_targets.yaw_target) * 0.02f;
    }

    /* 【2026-11-14 六改】蹲下期抑制 LQR 的 Tw 行摆角列。
     * 腿长斜坡跑起来时腿角会被动漂 (实测 lθ +0.092 -> -0.12), 而 Tw 行对
     * lθ 的增益是 +4.05 -> 单这一项就让轮力矩从 -0.365(前进) 翻到 +0.486(后退),
     * 变化 0.85N·m, 把 LQR 自己 ds 项的 0.39N·m 刹车力完全吃掉, 稳态就变成
     * -0.2m/s 持续后退 (蹲 100mm 退 152mm)。Tp 行的腿角反馈完全保留。 */
    lqr_tw_theta_gain_scale = (jump_ph == JP_SQUAT) ? jump_squat_tw_theta_scale : 1.0f;
    /* 【2026-11-16】PUSH 期放松 Tp 行对机身的执着, 让髆电机专心伸直腿。
     * 其余相位恒 1.0 (站立/行驶/蹲下 完全不受影响)。 */
    lqr_tp_phi_scale = (jump_ph == JP_PUSH) ? jump_push_tp_phi_scale : 1.0f;
    /* 【2026-11-20】地面解耦: 跳跃三段把 phi 从腿角里扣掉, 让 K[2][4] 只管腿、
     * K[2][8] 只管机身。详见 LQR.c 的 lqr_gnd_decouple 注释。 */
    lqr_gnd_decouple = (jump_ph == JP_SQUAT || jump_ph == JP_PUSH
                        || jump_ph == JP_LAND) ? 1.0f : 0.0f;

    LQR_calc(&ten_balance_observe,&ten_targets,&four_control_datas);
    chassis_dbg.lqr_u[0] = four_control_datas.l_Tw;
    chassis_dbg.lqr_u[1] = four_control_datas.r_Tw;
    chassis_dbg.lqr_u[2] = four_control_datas.l_Tp;
    chassis_dbg.lqr_u[3] = four_control_datas.r_Tp;

    /* ===== 位置积分（2026-11-13 自旧版移植）=====
     * 作用在 Tw(轮), 与 LQR 的 s 项同号叠加; 只补滚动阻力所需的极小力矩。
     * 与 LQR 的 x[0] 用同一死区规则, 否则积分会在死区内继续累积。 */
    {
        uint8_t drive_now = (chassis_cmd_recv.chassis_mode == CHASSIS_WHEELLEG_DRIVE);
        if (prone_rec_state == PRS_DONE && !air_state && !drive_now)
        {
            float es_i = ten_balance_observe.s - ten_targets.s_target;
            if (fabsf(es_i) <= lqr_s_deadband)
                es_i = 0.0f;
            else
                es_i -= (es_i > 0.0f) ? lqr_s_deadband : -lqr_s_deadband;
            s_int += s_ki * es_i * 0.001f;
            LIMIT_MIN_MAX(s_int, -S_INT_MAX, S_INT_MAX);
        }
        else
        {
            s_int = 0.0f;
        }
        four_control_datas.l_Tw += s_int;
        four_control_datas.r_Tw += s_int;
    }

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

    /* ===== T2: GRF 悬空门控（2026-11-03 手册第九章降维; 2026-11-16 合并同事 11-15 版: Tp 改直接 PD）=====
     * 悬空时 LQR_SetAir 已把 K 矩阵轮相关值清零: Tw=0(止疯转);
     * 此处输出端 Tw 再硬清一道双保险(也消 Tw 反作用对力矩反推的污染)。
     * Tp 不再交给 LQR 降维通道: 降维后 θ 增益仅 ~10 N·m/rad, 对左髋 ≥2 N·m
     * 静摩擦的死区达 0.2 rad —— 提车实测轮子滚动带偏腿后完全没有回正力。
     * 改用 air_kp_theta PD 直接覆写(30 N·m/rad → 死区 0.067 rad), 误差与 LQR
     * 同源配对(l_theta_target − l_theta, wrap_pi 防过顶 2π 跳变), 落地沿 LQR
     * 瞄同一误差无缝接管。态7 翻身(PRS_FLIP)不覆写: 该位形 GRF 未标定 air 易误判,
     * 且翻身摆腿用追赶式 Tp 语义不同, 保持原行为。
     * 腿锁住后 F_leg 卸到 -m_w·g, F_n 干净归 0;
     * 落地沿(F_n 双侧>th_on 驻留)恢复 LQR 并重锚定 s/yaw 清空转里程漂移。 */
    static uint8_t air_reanchor = 0;
    if (air_state)
    {
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
        if (prone_rec_state != PRS_FLIP)
        {
            float l_air_err = rec_wrap_pi(ten_targets.l_theta_target - ten_balance_observe.l_theta);
            float r_air_err = rec_wrap_pi(ten_targets.r_theta_target - ten_balance_observe.r_theta);
            four_control_datas.l_Tp = air_kp_theta * l_air_err
                                    - air_kd_theta * ten_balance_observe.l_dtheta;
            four_control_datas.r_Tp = air_kp_theta * r_air_err
                                    - air_kd_theta * ten_balance_observe.r_dtheta;
            LIMIT_MIN_MAX(four_control_datas.l_Tp, -air_tp_max, air_tp_max);
            LIMIT_MIN_MAX(four_control_datas.r_Tp, -air_tp_max, air_tp_max);
        }
        air_reanchor = 1;
    }
    else if (air_reanchor)
    {   /* 落地沿: 悬空期间轮子自由空转的里程漂移一次性清掉 */
        ten_targets.s_target   = ten_balance_observe.s;
        ten_targets.yaw_target = ten_balance_observe.yaw;
        air_reanchor = 0;
    }
    } /* end: 目标生成块(总跑; 态1~5 随后被状态机覆盖, 态6 归它接管) */
    /* 虚拟腿长力 = 重力前馈 + 腿长 PID；VMC 内部再按修正后的 J^T 映射。 */
    float l_dot = LegVelEst(ten_balance_observe.l_leg_length, &l_legvel);
    float r_dot = LegVelEst(ten_balance_observe.r_leg_length, &r_legvel);
    float l_len_err = ten_targets.l_leg_length_target - ten_balance_observe.l_leg_length;
    float r_len_err = ten_targets.r_leg_length_target - ten_balance_observe.r_leg_length;

    /* ===== 蹲下末段积分（2026-11-14）=====
     * 车重不需要提前知道: 积分自己找到"刚好托住车"的那个力。
     * 只在蹲到底且停稳(|误差|<15mm && 两腿速度<0.05m/s)时累加,
     * 其余时候(含蹲下中段)清零 —— 中段任其累加会一路往负走,
     * Fleg 越压越低(实测最低掉到 3N/-2N/-4N, 轮子抓地力≈0)。
     * ki 用 jump_squat_ki(默认 2000), 不动全局(全局无积分)。 */
    static float l_len_int = 0.0f, r_len_int = 0.0f;
    if (jump_ph == JP_SQUAT)
    {
        if (fabsf(l_len_err) < 0.015f && fabsf(r_len_err) < 0.015f &&
            fabsf(l_dot)     < 0.05f  && fabsf(r_dot)     < 0.05f)
        {
            l_len_int += jump_squat_ki * l_len_err * 0.001f;
            r_len_int += jump_squat_ki * r_len_err * 0.001f;
            LIMIT_MIN_MAX(l_len_int, -25.0f, 25.0f);
            LIMIT_MIN_MAX(r_len_int, -25.0f, 25.0f);
        }
        else
        {
            l_len_int = 0.0f;
            r_len_int = 0.0f;
        }
    }
    else
    {
        l_len_int = 0.0f;
        r_len_int = 0.0f;
    }

    /* 腿长环三段:
     * 普通相位: P + 阻尼。
     * PUSH/SHRINK: 腿以 ~1m/s 高速伸展, kv 会把前馈整个拽回去
     *   (实测 -kv*1.174 抵消约 137N, 把 120N 前馈彻底淹没), 故只留 P。
     * SQUAT: 阻尼改成只惩罚"偏离参考速度的部分" -kv*(l_dot - vref) ——
     *   匀速下蹲时 l_dot == vref -> 阻尼 = 0 -> 不再需要位置误差去抵消它,
     *   稳态滞后从 kv*v/kp(=0.125v, 实测 0.5m/s 时 62.5mm)降到 ~6mm;
     *   到底段 vref 回 0 -> 阻尼恢复全量 -> 照样刹得住。 */
    float l_pid_out, r_pid_out;
    /* 【v16 2026-11-20】左右腿独立增益: 实际刚度 = 基准 × 倍率。
     * 倍率默认 1.0 = 旧行为。右腿响应慢就把 leg_gain_r 加大。 */
    {
        /* 【v16】倍率只在跳跃期间生效(IDLE 强制 1.0) —— 站立/自起/行驶 完全不变 */
        float gl = (jump_ph != JP_IDLE) ? leg_gain_l : 1.0f;
        float gr = (jump_ph != JP_IDLE) ? leg_gain_r : 1.0f;
        float kp_l = leg_kp_pos * gl;
        float kp_r = leg_kp_pos * gr;
        float kv_l = leg_kv_vel * gl;
        float kv_r = leg_kv_vel * gr;
    if (jump_ph == JP_PUSH)
    {
        /* 【2026-11-16】PUSH 恢复阻尼, 但改成只惩罚"偏离参考速度"的部分:
         * 旧写法这里完全没有 kv 项 —— 因为腿要伸到 2m/s, 全量阻尼
         * -kv*l_dot = -100*2 = -200N 会把前馈整个吃掉。
         * 现在有了 jump_push_vref, 匀速伸展时 l_dot==vref -> 阻尼=0,
         * 腿落后/超前才会出力。与 SQUAT 同一个思路, 方向相反。 */
        l_pid_out = kp_l * l_len_err + l_len_int
                  - kv_l * (l_dot - jump_push_vref);
        r_pid_out = kp_r * r_len_err + r_len_int
                  - kv_r * (r_dot - jump_push_vref);
    }
    else if (jump_ph == JP_LAND)
    {
        /* 【2026-11-18 v4】起立段阻尼去耦, 与 SQUAT/PUSH/SHRINK 同一个思路。
         * jump_land_vref 一个变量两用:
         *   缓冲段 = 0   -> -kv·l_dot 全量阻尼 -> 落地 184N/腿 吸能
         *   起立段 = ±0.3 -> 匀速段 l_dot==vref -> 阻尼 0 -> 不拖后腿
         *   到位后 = 0   -> 阻尼恢复全量 -> 温柔停住 */
        l_pid_out = kp_l * l_len_err + l_len_int
                  - kv_l * (l_dot - jump_land_vref);
        r_pid_out = kp_r * r_len_err + r_len_int
                  - kv_r * (r_dot - jump_land_vref);
    }
    else if (jump_ph == JP_SHRINK)
    {
        /* 【2026-11-17 v2】收腿: 加"只罚偏离参考速度"的阻尼(同 SQUAT/PUSH 思路)。
         * 旧版这里完全没 kv 项 -> 空中腿 ωn=30rad/s(4.8Hz) ζ=0 -> 阶跃目标必振。
         * 现在目标是三角形速度曲线, 匀速段 l_dot==vref -> 阻尼=0 -> 力≈kp*误差≈0,
         * 平滑收腿不抽。到位后 vref=0 -> 阻尼恢复全量 -> 自然刹住。 */
        l_pid_out = kp_l * l_len_err + l_len_int
                  - kv_l * (l_dot - jump_shrink_vref_l);
        r_pid_out = kp_r * r_len_err + r_len_int
                  - kv_r * (r_dot - jump_shrink_vref_r);
    }
    else if (jump_ph == JP_SQUAT)
    {
        l_pid_out = kp_l * l_len_err + l_len_int
                  - kv_l * (l_dot - jump_squat_vref);
        r_pid_out = kp_r * r_len_err + r_len_int
                  - kv_r * (r_dot - jump_squat_vref);
    }
    else
    {
        l_pid_out = kp_l * l_len_err + l_len_int - kv_l * l_dot;
        r_pid_out = kp_r * r_len_err + r_len_int - kv_r * r_dot;
    }
    } /* end: 独立增益作用域 */
    /* 腿长误差门限幅(南航金城 stand_PID MaxOut 思路,防恢复段目标跳变弹车):
     * |目标-实测| 超 rec_push_band 期间 PID 分量钳 ±rec_push_max,
     * 误差进带(正常平衡/行驶,误差毫米级)恢复全量 ±LEG_PID_OUT_MAX。 */
    float l_pid_max = (fabsf(l_len_err) > rec_push_band)
                      ? rec_push_max : LEG_PID_OUT_MAX;
    float r_pid_max = (fabsf(r_len_err) > rec_push_band)
                      ? rec_push_max : LEG_PID_OUT_MAX;
    /* 跳跃例外: 腿长目标是阶跃的(0.15->0.42 = 27cm), 远超 rec_push_band,
     * 若仍钳在 rec_push_max(60N) 腿根本伸不动。这三相位直接放开全量。 */
    if (jump_ph == JP_SQUAT || jump_ph == JP_PUSH)
    {
        l_pid_max = LEG_PID_OUT_MAX;
        r_pid_max = LEG_PID_OUT_MAX;
    }
    else if (jump_ph == JP_SHRINK)
    {   /* 【2026-11-17 v2】空腿用不了 150N (2.17kg 下是 69m/s²);
         * 且 2x60=120N < mg=115.3N —— 万一脚还在地上也拽不动车, 安全兜底。
         * 【2026-11-18 v5】左右分开: 两腿不对称(右腿多扛 6N、腿角偏 4°),
         * 同一个上限会一边先收到位、另一边卡住 -> 落地砸单腿。 */
        l_pid_max = jump_shrink_ff_max;
        r_pid_max = jump_shrink_ff_max_r;
    }
    else if (jump_ph == JP_LAND && !jump_land_buf_done)
    {   /* 【2026-11-19 v6】缓冲段单独放开力上限, 详见 jump_land_ff_max 注释。
         * 缓冲完成(腿速归零)后自动退回 rec_push_max(60N), 防起立弹车。 */
        l_pid_max = jump_land_ff_max;
        r_pid_max = jump_land_ff_max;
    }
    LIMIT_MIN_MAX(l_pid_out, -l_pid_max, l_pid_max);
    LIMIT_MIN_MAX(r_pid_out, -r_pid_max, r_pid_max);
    /* Roll 补偿（手册 §8.2）：右倾(roll<0)时右腿加力、左腿减力。
     * Roll 单位度 -> rad；Gyro[1] 为 roll 角速度(rad/s)。
     * PD 段(态1~4)与态7 倒地阶段 roll 无意义,补偿关闭;态5 起恢复。
     *
     * ============================================================
     * 【v18 2026-11-21 致命回路修复】这是一个【增益 >1 的正反馈回路】
     * ============================================================
     *   dF_roll 不是独立的扰动量, 它通过【腿长】绕成一个闭环:
     *     ① 机身侧倾 θ
     *     ② dF_roll = roll_kp·θ            (400 N/rad)
     *     ③ 左腿 -= dF_roll, 右腿 += dF_roll  -> 两腿差模力 2·dF_roll
     *     ④ 腿长差 = 2·dF_roll / leg_kp_pos = 800θ/1600 = 0.5θ (m)
     *     ⑤ 两腿长度差直接把机身拧斜: θ' = 0.5θ / 轮距w
     *   环路增益  G = 2·roll_kp/(leg_kp_pos·w) = 800/(1600×0.26) = 【1.92】
     *   G>1 -> 静态不稳定 -> 任意小偏差指数发散(实测 e 时间常数 53ms)。
     *
     *   实测 2026-11-20 22:46 (跳跃):
     *     PUSH 基础段两腿差全程<=2.5mm (规划器完全对称, 不是规划问题)
     *     冲量段 100->206N 阶跃踢出 1° roll 后:
     *       Roll  -1.0° -> +17°      (150ms)
     *       dF_roll +7 -> -129 N
     *       左腿 273 -> 453mm, 右腿 276 -> 411mm  (差 43mm)
     *       腿角差 0.087 -> 0.855 rad (49°)
     *       -> 平均腿长停滞 -> abort=5(腿卡住) -> 退回 IDLE
     *   实测 2026-11-20 22:39 (自起): 同一个病, dF_roll 60~85N, 两腿撕成 198/420。
     *
     * 【为什么不是“加阻尼”而是“直接关”】
     *   dF_roll 是【位置级】执行器(改腿长->改姿态), 比轮子的差速力矩慢一个量级。
     *   地面上只要有轮子可用, roll 就该交给轮子; 腿的差模只在自起流程(轮子未接管)
     *   才有意义。跳跃三相位里腿的输出已被主任务(蹬地 200N / 收腿 -60N)占满,
     *   再塞 ±129N 差模必然变成腿长差。
     *
     * 【空中(JP_SHRINK)必须关的硬理由】
     *   离地后没有地面反力, 腿上的差模力是纯内力 -> 对机身姿态贡献【严格为零】,
     *   只会把两条腿撕开(实测“超级剪刀脚”)。禁用它损失零控制能力, 只除一个纯扰动源。
     *   这与 theta_lvl 被移出 SHRINK 相位窗口是同一个理由。 */
    uint8_t rec_pd_stage = (prone_rec_state >= PRS_EXTEND && prone_rec_state <= PRS_PUSH)
                           || prone_rec_state == PRS_FLIP
                           || prone_rec_state == PRS_QUICK;
    float dF_roll = rec_pd_stage ? 0.0f
                               : ROLL_COMP_DIR * (roll_kp * (-roll_rad) - roll_kd * chassis_IMU_data->Gyro[1]);
    /* 【v18】① 跳跃三相位禁用: 腿力是主控制量, 差模会直接变成腿长差 */
    if (jump_ph == JP_SQUAT || jump_ph == JP_PUSH || jump_ph == JP_SHRINK)
        dF_roll = 0.0f;
    /* 【v18】② 永久安全网: 任何相位都不许超过腿长环权限(60N)。
     * 即使将来重新启用, 也不会再出现“PID 双满还输给 roll 补偿”的失控。
     * 注: roll_kp=400 时 60N 对应 8.6° 的 roll —— 超过这个角度就该由轮子接管了,
     * 腿再使劲也只会把腿长撕开。 */
    LIMIT_MIN_MAX(dF_roll, -rec_push_max, rec_push_max);
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
    /* 【2026-11-14 四改】蹲下加速度前馈 —— 蹲下段唯一的加速力来源。
     * 物理: Fleg = m*(g - a)。a>0(收腿加速) -> 减力; a<0(刹车) -> 加力。
     * 不加前馈时只能靠位置环的“误差”攒力:
     *   加速段要 Fleg 60->4.9N, 需 28mm 滞后(仿真/实测 19.9mm 均对得上);
     *   刹车段要 Fleg 114.7N, 需 40mm 超前, 但刹车行程只有 20mm
     *   -> 实际只能出 82N -> 减速度 3.6 而非 9 -> 冲过 17.7mm。
     * 【2026-11-14 五改】基线回到 LEG_GRAVITY_FF(50N), 只保留加速度项:
     *   acmd=+5 -> 50-25.5=24.5N ; acmd=0 -> 50N ; acmd=-5 -> 75.5N
     * 204658 实测教训: 四改把基线写成 6.1*9.8=59.8N, 多给 9.8N 把腿顶在 0.157,
     *   差 7mm 蹲不到位 —— overshoot 治好了, 又变成 undershoot。
     *   仿真扫描前馈等效质量 m_ff: 4.6->-3.6mm, 5.1->-0.5mm, 6.1->+4.9mm, 6.6->+8.0mm。
     *   所以 LEG_MASS_EFF = 50/9.8 = 5.1kg, 基线就是站立时的 LEG_GRAVITY_FF。 */
    if (jump_ph == JP_SQUAT)
        grav_ff = LEG_GRAVITY_FF - LEG_MASS_EFF * jump_squat_acc_now;

    /* ===== 跳跃力前馈（2026-11-11, 这是起跳的唯一动力来源, 不是 Tp）=====
     *   PUSH   -> 双段: 前段 jump_ff_push 建立速度, 末段 jump_ff_peak 爆发
     *   SHRINK -> 回到 LEG_GRAVITY_FF, 空中腿不承重, 收腿降转动惯量
     *   LAND   -> 从 peak 线性回落, 吸收落地冲击
     * 物理: 跳高只看离地速度 v=sqrt(2gh), 所需冲量 p=m*v 是硬指标。
     *   跳10cm: v=1.40m/s, p=17.1N·s -> t=0.107s 需 140N/腿。 */
    if (jump_ph == JP_PUSH)
    {
        /* 干跑时不给蹬地力, 保持在站立水平 */
        if (jump_dry_run)
        {
            grav_ff = LEG_GRAVITY_FF;
        }
        else
        {
        /* 【2026-11-16 v2】PUSH 加速度前馈 —— 与 SQUAT 对称, 但【加】不是减。
         * PUSH 是蹬地: Fleg = m*(g + a)。a>0(伸腿加速) -> 加力。
         *   acc_now=0 -> 50N (与站立一致, 不会像旧版那样降到40N把腿变软)
         *   acc_now=6 -> 50 + 5.4*6 = 82.4N/腿
         * 等效质量用 jump_push_mass(5.4) 而非 LEG_MASS_EFF(5.1):
         *   5.1 是「50N/g」, 而实测单腿承重 52.6N -> 5.36kg, 用 5.1 会少给 11% 的力。
         * 旧版是两段阶跃 jump_ff_push(40N) -> jump_ff_peak(70N),
         *   起点40N比站立50N还低 10N, 转 PUSH 那一拍腿会突然变软。 */
        /* 【2026-11-17 v4】两段式力前馈:
         *   基础段: F = 50 + m·a_base            (位置控, a_base 由跳高反解)
         *   冲量段: F = 50 + (F_imp-50)·k        (k 在 jump_impulse_ms 内 0->1, 恒力力控)
         * 冲量段必须走前馈而不是靠 PD: PD 的力 = kp·(目标-实测), 腿一快误差就缩,
         * 力自己就掉了 —— 冲量消失。 */
        /* 【v10 2026-11-19】尾部卸载: 最后 jump_impulse_taper 米内整体乘 taper_k(1->0)。
         * 基线 50N 也一起衰减 —— 只降冲量项不降基线的话出口还剩 50N,
         *   N = 2×50+9.8 = 110N > 30N 门限, 轮子还是卸不掉。
         * 与位置出口配合: 出口那拍力已经≈0, 腿靠惯性继续伸到机械极限,
         *   变成刚性杆把轮子顶离地 —— 这是「放出去」, 不是「拽回来」。 */
        {
            float taper_k = 1.0f;
            if (jump_push_imp && jump_impulse_taper > 0.0005f)
            {
                float remain = jump_push_max_len - jump_push_len_avg;
                if (remain < jump_impulse_taper)
                {
                    taper_k = remain / jump_impulse_taper;
                    if (taper_k < 0.0f) taper_k = 0.0f;
                    if (taper_k > 1.0f) taper_k = 1.0f;
                }
            }
            jump_push_taper_k = taper_k;
            grav_ff = (LEG_GRAVITY_FF
                     + (jump_impulse_ff - LEG_GRAVITY_FF) * jump_push_imp_k) * taper_k
                    + jump_push_mass * jump_push_acc_now;
        }
        }
    }
    else if (jump_ph == JP_SHRINK)
    {
        /* 【2026-11-17 v2】空中没东西承接重力, 50N 前馈只会把腿往外推,
         * 位置环再拉回来 —— 纯属来回打架。收腿不需要重力补偿。 */
        grav_ff = 0.0f;
    }
    else if (jump_ph == JP_LAND)
    {
        /* 【2026-11-18 v4】落地前馈 = 站立重力, 【不加码】。
         * 旧写法 jump_ff_peak(70N) 线性降到 50N —— 70x2=140N > 整车 115.3N,
         * 光前馈就能把车顶离地; 再加上阻尼 184N, 必然弹。
         * 缓冲靠什么: 阻尼(落地瞬间 kv·v = 100x1.844 = 184N/腿) + 位置环(压缩越深出力越大),
         *   前馈只需要抵消重力。 */
        grav_ff = LEG_GRAVITY_FF;
    }
    if (prone_rec_state == PRS_LQR_RAMP)
        dF_roll *= rec2_ramp;   /* roll 补偿同随斜坡渐入 */
    /* 【2026-11-16 合并同事 2026-11-15 版】悬空清零重力前馈 + Roll 补偿。
     * 为什么必须清: 悬空时没有地面承接, LEG_GRAVITY_FF(50N) 变成纯"把腿推长"的
     * 内力, 再叠加 ±dF_roll 会顶满 60N 误差带钳位; 顶进饱和后增量刚度=0
     * -> 单边腿长漂移。同事提车实测: 车体往一边偏时一侧持续伸长、一侧停在 25cm;
     * roll 补偿在没有支撑面时纯粹是扰动源。
     * 悬空维持腿长只需脚端自重级的小力, 腿长 PID(1600N/m) 独立锁定毫米级误差;
     * 落地沿 0->50N 的阶跃发生在 F_n 双侧 > th_on 承力确认【之后】, 由 GRF 直接
     * 消化, 与站立语义一致。态1~4/6 已强制接地不受影响; 态7 翻身期两项本就为 0。
     * 注意: 跳跃 PUSH 段 air_state=0(还在蹬地), 加速度前馈不受影响;
     *       离地进 JP_SHRINK 后清零 —— 正是所需(空中腿不承重)。 */
    if (air_state)
    {
        grav_ff  = 0.0f;
        dF_roll  = 0.0f;
    }
    /* 【2026-11-18 v3.1】收腿惯性前馈 —— 必须【无条件】按相位给, 不能挂在 air_state 里。
     * 时序坑: PUSH 腿长到 0.36 切 SHRINK 那一刻, GRF 离地判定还要连续 100ms
     *   (grf_air_hold_off=100) 才置 1 —— 而收腿加速段只有 0~68ms。
     *   挂在 air_state 里 = 【整个加速段没有前馈】, 腿只能靠位置误差攒力。
     *   减速段(68~100ms)也在 air_state=0 期间, 同样白搭。
     * 安全性: 前馈 1.35x30 = 40N/腿, x2 = 80N < 整车 115.3N ——
     *   即使脚还在地上也推不动车, 只会把腿收短。
     * 符号: a_now>0(收腿加速) -> grav_ff 负(收腿方向); a_now<0(刹车) -> 正(顶住)。 */
    float l_shrink_ff = 0.0f, r_shrink_ff = 0.0f;
    if (jump_ph == JP_SHRINK)
    {
        /* 【v7】等离地滑行段: 前馈必须归零。
         * 这段腿只留位置环(误差=0)和单向阻尼, 总力≈0 —— 车身带腿自由上升,
         * 轮子才卸得掉载。若给 -1.35*acc_now 或 50N 重力前馈, 轮子一直被压在地上,
         * air_grf 永远等不到 1, 就又回到「在地上收腿」的老路。 */
        if (jump_shrink_hold)
        {
            grav_ff = 0.0f;
        }
        else
        {
            grav_ff = -jump_shrink_mass * jump_shrink_acc_now;
            /* 【v13 2026-11-19】只允许负惯性前馈。
             * 三角曲线减速段 acc_now = -15 -> 旧公式给 +20.25N(往伸)。
             * 空中加上 PID 最多能到 +80N, 使 N = F_leg > th_on(40)
             * -> air 被消掉 -> 误判落地。
             * 减速靠位置环+阻尼自己剥就行, 不需要额外顶。 */
            if (grav_ff > 0.0f) grav_ff = 0.0f;
            /* 【2026-11-18 v5】左右腿收腿附加前馈 —— 补偿两腿静态不对称。
             * 默认 0, 不影响旧版行为。负值 = 额外往回收。 */
            l_shrink_ff = jump_shrink_ff_l;
            r_shrink_ff = jump_shrink_ff_r;
        }
    }
    four_control_datas.l_Tleg = grav_ff + l_pid_out - dF_roll + l_shrink_ff;
    four_control_datas.r_Tleg = grav_ff + r_pid_out + dF_roll + r_shrink_ff;
    chassis_dbg.Fleg_cmd[0] = four_control_datas.l_Tleg;
    chassis_dbg.Fleg_cmd[1] = four_control_datas.r_Tleg;
    chassis_dbg.Ldot[0] = l_dot;
    chassis_dbg.Ldot[1] = r_dot;
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
    /* 【2026-11-21】调试镜像刷值(在 out_gate 算完之后, 值才是最终生效的) */
    dbg_prone_state_f = (float)prone_rec_state;
    dbg_out_gate      = (float)out_gate;
    dbg_leg_tgt_l     = ten_targets.l_leg_length_target;
    dbg_leg_tgt_r     = ten_targets.r_leg_length_target;
    {
        float pf = 0.0f;
        switch (jump_ph)
        {
        case JP_SQUAT:  pf = 1.0f; break;
        case JP_PUSH:   pf = 2.0f; break;
        case JP_SHRINK: pf = 3.0f; break;
        case JP_LAND:   pf = 4.0f; break;
        default:        pf = 0.0f; break;
        }
        dbg_jump_ph_f = pf;
    }
    dbg_push_abort_f = (float)jump_push_abort;
    dbg_push_imp_f   = (float)jump_push_imp;
    dbg_push_stuck_f = (float)jump_push_stuck_t;
    dbg_jump_ready_f = (float)jump_ready;
    dbg_push_taper_f = jump_push_taper_k;
    if (out_gate)
    {
        four_control_datas.l_Tp = 0.0f;
        four_control_datas.r_Tp = 0.0f;
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
        four_control_datas.l_Tleg = 0.0f;
        four_control_datas.r_Tleg = 0.0f;
    }

    /* ===== 腿角差模拉平（2026-11-17, 详见 theta_lvl_k 注释）=====
     * 只拉“两条腿的差值”, 绝不碰“两腿的平均值”:
     *   dθ > 0 -> 左腿比右腿更靠前 -> 左腿给负 Tp、右腿给正 Tp, 两腿对向收拢。
     *   两腿等量反向 => 机身净力矩恒为 0, 不会造成 pitch 变化。 */
    theta_lvl_dth = ten_balance_observe.l_theta - ten_balance_observe.r_theta;
    theta_lvl_dbg = 0.0f;
    /* 【2026-11-19 v1】从"只在 JP_IDLE"放开到"蹬地/落地也拉平"。
     * 实测 224726: 跳跃前两腿静态只差 0.084rad, 但
     *   蹬伸段右腿 θ 从 -0.087 跑到 -0.279 (左腿只动 +0.051) -> 差 0.327rad;
     *   落地段继续发散, 1.5s 后差到 1.94rad(111°) -> 车侧翻, 轮子卸载(F_n 掉到 10/25N)。
     *   根因: 这条拉平只在空闲生效, 跳跃全程没人管这个差值, 小偏置被放大成翻车。
     * 为什么不含 SHRINK: 空中两条腿各自由 Tleg 控制, 没有地面约束, 强行拉平只会互相拽。
     * 为什么不含 SQUAT: 蹲伏段腿角本来就在大范围变化, 拉平会干扰。 */
    /* 【2026-11-20 修】① 相位窗口补 SQUAT; ② 增益按支撑刚度 N·l 自适应
     * (站立=1.0倍, PUSH 末段≈5.8倍), 否则 k 低于临界值会发散;
     * ③ 上限 1.5->15 N·m (PUSH 末段侧向力矩实测需要 ≈9.5 N·m)。详见参数定义注释。
     *
     * 【2026-11-21 修】曾把 SHRINK 也放进窗口, 实测直接炸成「超级剪刀脚 + yaw 飞掉」:
     *   差模 Tp 是 ±15N·m 的纯内部力矩, 在地上靠地面反力抵消, 在空中没人抵消。
     *   而它是【叠加】在 air_kp_theta 空中 PD 之后的, 享受不到 air_kd_theta 阻尼,
     *   等于给两条腿各灌一个无阻尼的差模恒力矩 -> VMC 里同一个 Fleg 生成反向关节力矩
     *   -> 一条腿收、一条腿放 -> 剪刀角张开, 收腿动作被完全吃掉。
     *   现已永久排除 SHRINK: 空中两条腿各自由 Tleg 管, 不互相拽。 */
    {
        /* 【v19 2026-11-21 仿真+实测定案】自适应倍率 scl 已【删除】, 恒为 1.0。
         *
         * 旧写法: scl = (N·l)/(50N × 0.25m), 限幅 [1, 8]。
         * 概念错误: N·l 是腿的【轴向】支撑刚度(顶地能力), 而这条环调的是
         *   【摆角 θ】。拿轴向刚度去放大摆角增益没有任何物理依据 ——
         *   而且它是【随蹬地力线性增长】的, 等于"蹬得越狠, 摆角环越激进",
         *   正好在最需要稳的 PUSH 段最激进。
         *
         * 实测 2026-11-20 23:13 (PUSH 段, dF_roll 已禁用, 全程 = 0.0):
         *   theta_lvl_scl : 1.33 -> 2.6 -> 3.3 -> 5.5 -> 6.9   (随 200N 蹬地力爬升)
         *   实际增益 k·scl: 15 × 6.9 = 【103.5 N·m/rad】
         *   theta_lvl_dbg : -1.44 -> -3.20 -> 【+5.87 -> +15.00 打满】
         *     ^ 注意 865 拍【符号翻转】: dth 冲过死区后被反向打满 = 典型过增益振荡
         *   后果: 差模 Tp 打满 ±15N·m -> VMC 生成反向关节力矩 ->
         *         左腿 θ -0.16->-0.60 (后摆), 右腿 θ -0.22->+0.14 (前摆)
         *         -> 剪刀角 0.736 rad(42°), 平均腿长停滞 -> abort=5
         *
         * 各相位的 scl 实际值(按旧公式):
         *   站立 N≈52, l≈0.25  -> scl = 1.04   (本来就≈1, 放大毫无意义)
         *   蹲伏 N≈50, l≈0.15  -> scl = 0.60 -> 被夹成 1.0
         *   蹬伸 N≈200,l≈0.40  -> scl = 6.40   ← 唯一被放大的相位, 且正好放大出事
         * 也就是说这条自适应【只在 PUSH 生效】, 收益为零、风险全占。
         *
         * 【v19】PUSH / SHRINK 直接移出相位窗口 —— 与 dF_roll 同一个理由:
         *   这两个相位腿的输出被主任务(蹬地 200N / 收腿 -60N)占满,
         *   任何差模注入都只会变成腿长差/剪刀角, 不会变成有用的姿态修正。
         *   SQUAT 保留(蹲伏段实测歪 4.6°, 确实需要; 且 scl=1 很温和)。
         *   IDLE / LAND 保留(站立微调, 实测只有 -0.8 N·m)。 */
        theta_lvl_scl = 1.0f;

        if (!out_gate && prone_rec_state == PRS_DONE &&
            (jump_ph == JP_IDLE || jump_ph == JP_SQUAT
             || jump_ph == JP_LAND))
        {
            float adth = fabsf(theta_lvl_dth);
            if (adth > theta_lvl_db)
            {
                float mag = adth - theta_lvl_db;      /* 软死区: 出死区后从 0 连续增长 */
                float td  = -theta_lvl_k * mag;
                if (theta_lvl_dth < 0.0f) td = -td;   /* 差值为负则反向 */
                LIMIT_MIN_MAX(td, -theta_lvl_max, theta_lvl_max);
                four_control_datas.l_Tp += td;
                four_control_datas.r_Tp -= td;
                theta_lvl_dbg = td;
            }
        }
    }

    VMC_Calc(&four_control_datas, &r_leg_motor_torques, &r_leg);
    l_VMC_Calc(&four_control_datas, &l_leg_motor_torques, &l_leg);

    /* 关节摩擦前馈：VMC 输出的 Tf/Tb 已是电机自身坐标，与 measure.velocity 同系，
     * 直接同号叠加即可（左右腿各自的坐标约定在 VMC 内部已处理完）。
     * 脱力等待期(S0/S7 未武装)跳过 —— 零力语义完整。 */
    float ff_rf = 0.0f, ff_rb = 0.0f, ff_lf = 0.0f, ff_lb = 0.0f;
    if (!out_gate)
    {
        /* 【2026-11-16 合并同事 2026-11-15 版】悬空选 _air 系数: 悬空载荷相关
         * 摩擦低一个量级, 沿用着地 Tc 必过补成负阻尼(同事实测: 提车摆腿发散);
         * 默认 0 = 悬空跳过, 从 0 起步在线标定。
         * 态7 翻身不切(air 易误判 + 翻身摆腿要靠前馈破静摩擦)。 */
        uint8_t fric_air = (air_state && prone_rec_state != PRS_FLIP) ? 1u : 0u;
        float fric_t_r = fric_air ? r_joint_fric_t_air : r_joint_fric_t;
        float fric_t_l = fric_air ? l_joint_fric_t_air : l_joint_fric_t;
        ff_rf = JointFricFF(r_front_8009_motor->measure.velocity,  fric_t_r);
        ff_rb = JointFricFF(r_behind_8009_motor->measure.velocity, fric_t_r);
        ff_lf = JointFricFF(l_front_8009_motor->measure.velocity,  fric_t_l);
        ff_lb = JointFricFF(l_behind_8009_motor->measure.velocity, fric_t_l);
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

    /* 轮力矩下发：先按 3508 的真实上限钳位，再换算成电流码。
     * 限幅必须放在换算之前 —— 见 chassis.h 里 WHEEL_T_MAX 的注释。 */
    /* 轮端摩擦前馈：r_dx/l_dx 已是前进为正的轮缘线速度(m/s)，直接取符号；
     * 左轮 l_wheel_direction=-1 的换算在 DMMotorSetRef 处，FF 加在换算前随之落位。
     * PD 段(态1~4)与态7 轮力矩恒 0(轮未触地/不应滚动);态5 斜坡起轮控随 LQR 渐入恢复。 */
    float r_Tw, l_Tw;
    if (air_state || prone_rec_state <= PRS_PUSH || prone_rec_state == PRS_FLIP ||
        prone_rec_state == PRS_QUICK)
    {
        /* 恢复 PD 段/翻身段/折叠快转段, 或 GRF 判悬空(T2): 轮力矩清零, 摩擦前馈一并跳过 */
        r_Tw = 0.0f;
        l_Tw = 0.0f;
    }
    else
    {
        float ff_rw = r_wheel_fric_t * tanhf(r_wheel.r_dx / WHEEL_FRICTION_TANH_V0);
        float ff_lw = l_wheel_fric_t * tanhf(l_wheel.l_dx / WHEEL_FRICTION_TANH_V0);
        r_Tw = four_control_datas.r_Tw + ff_rw;
        l_Tw = four_control_datas.l_Tw + ff_lw;
        LIMIT_MIN_MAX(r_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
        LIMIT_MIN_MAX(l_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
        chassis_dbg.wheel_ff[0] = ff_lw;
        chassis_dbg.wheel_ff[1] = ff_rw;
    }
    chassis_dbg.Tw_cmd[0] = l_Tw;
    chassis_dbg.Tw_cmd[1] = r_Tw;

    /* isfinite 与关节侧同款护栏: LQR 状态异常(NaN)时不让 NaN 电流码下发 */
    DJIMotorSetRef(r_wheel_motor, isfinite(r_Tw) ? WHEEL_T_PER_RAW * r_Tw : 0.0f);
    DJIMotorSetRef(l_wheel_motor, l_wheel_direction * (isfinite(l_Tw) ? WHEEL_T_PER_RAW * l_Tw : 0.0f));

    



}
