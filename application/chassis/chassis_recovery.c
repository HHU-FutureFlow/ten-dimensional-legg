#include "chassis_recovery.h"
#include "chassis.h"         /* ten_balance_observe / ten_targets / chassis_IMU_data */
#include "chassis_friction.h" /* LegFrictionPlan_s / CHASSIS_TICK_S */
#include "chassis_utils.h"   /* rec_wrap_pi / ChaseReferenceSpeed */
#include "chassis_step_leg.h" /* STEP_LEG_READY / step_leg_motion */
#include "grf_observer.h"    /* GRF_RestartAirDelay / GRF_ClearAirState */
#include "robot_def.h"       /* CHASSIS_ZERO_FORCE 等模式枚举 */
#include "motor_def.h"       /* LIMIT_MIN_MAX */
#include <math.h>

/* ===== 模块职责 =====
 * 统一起立/自救状态机 + T3 翻倒保护。直接写共享目标/计划量,
 * 不经 LQR/VMC 平衡通道(手册第九章), 力矩覆盖与电机下发仍由 chassis.c 主流程完成。 */

/* ===== 交叉引用（定义在 chassis.c, 挡位斜坡与控制管线计划） ===== */
extern volatile LegFrictionPlan_s plan_l, plan_r;

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
 *   仅保留速率斜坡(防弹车)+段超时(安全兜底)+出口驻留(防抖)。
 * 历史: v2 FOLD/DONE 状态机已删除(2026-11-09): 低腿长+大φ误差下 DONE 切换
 *   Tp 无钳位猛甩, 由本状态机替代(态5 LQR 斜坡接管补钳位)。
 *   腿长目标改由各态自写: 态1~4 PD 段按段目标, 态5/6 恒 rec_l_rise。 */

/* ===== 可调参数定义（ep 在线整定, 符号名不得改动） ===== */
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
/* ---- 态6 触发带 ---- */
volatile float rec_fold_l_th   = 0.13f;    /* 态6 触发: 平均腿长下限 m(断电折叠态 ≈0.11~0.124) */
volatile float rec_fold_th     = 0.85f;    /* 态6 触发: 双腿 |θ_body| 上限 rad(11-14 零力实测折叠上电角 0.70/0.75, 留 0.1 裕量; 上界<π/2 保轮心在下半圆带内, 短转=轮沿地滚) */
/* ---- 伸腿防弹(沿用 v2): 腿长误差超带期间 PID 分量钳 ±rec_push_max ---- */
volatile float rec_push_band = 0.02f;     /* 误差门限 m */
volatile float rec_push_max  = 60.0f;     /* 大误差期间 PID 分量限幅 N(60: 前腿承压+左腿摩擦+翻身段共用, 关节力矩约6-9N·m) */
/* ---- PD 段专用重力前馈: 倒地/顶升期腿不承机体全重(车头分担), 小前馈防平衡点抬高 ---- */
volatile float rec_gravity_ff = 0.0f;     /* N/腿 */
/* ---- ep 诊断: 连续目标角/连续实测角(态2/3 对齐进度, 不 wrap 的累计值) ---- */
volatile float dbg_goal_l = 0.0f, dbg_goal_r = 0.0f;
volatile float dbg_cont_l = 0.0f, dbg_cont_r = 0.0f;

/* ===== T3 翻倒保护 + 翻身段参数（2026-11-05 SPR 式, 2026-11-09 并入统一状态机为态7）=====
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

/* ===== 文件级状态 ===== */
volatile uint8_t prone_rec_state = 0;
/* 态7 确认/武装 + 状态机进入沿标志: 零力进入沿/恢复沿经 Recovery_Notify* 访问 */
volatile uint8_t prot_armed = 0;
volatile uint8_t prot_seen_zf = 0;
volatile uint8_t rec6_anchor = 0;     /* 通用进入沿标志(段内共享, 段出口清零) */
volatile uint8_t lift_anchor = 0;     /* 态5 起立斜坡锚定标志(零力进入沿/T3/READY 接管访问) */
volatile uint8_t lift_top = 0;        /* 态5 起立双腿到顶标志(2026-11-15: 到顶沿重锚挡位斜坡, 与 lift_anchor 同批清零) */
/* 统一状态机共享状态（态7 翻身与态1~5 恢复流程共用） */
static uint16_t prone_valid = 0;
static uint16_t rec6_t = 0, rec6_hold = 0;
static float    ext_l = 0.25f, ext_r = 0.25f;      /* 态1 腿长斜坡值 */
static float    cont_l = 0.0f, cont_r = 0.0f;      /* 连续实测角(体系, 不 wrap) */
volatile float  cur_l = 0.0f, cur_r = 0.0f;        /* 当前摆角目标(连续, 覆盖段 PD 用; 主流程输出覆盖读) */
static float    end_l = 0.0f, end_r = 0.0f;        /* 当前段终点角 */
volatile float  rec_tht_l = 0.0f, rec_tht_r = 0.0f;/* 态4 锚定/态5 收零的世界系 θ 目标(主流程台阶开窗/跳跃门控读) */
static float    len_cmd_l = 0.25f, len_cmd_r = 0.25f;/* 态3 联动收腿目标(限斜率后) */
static float    lift_l = 0.25f, lift_r = 0.25f;    /* 态5 起立腿长斜坡值 */
/* 态7 翻身段状态 */
static uint8_t  tilt4_anchor = 0;
static uint16_t tilt4_t = 0;
static uint16_t tilt4_hold = 0;
static float tilt4_l = 0.25f, tilt4_r = 0.25f;
/* T3 出窗驻留计数 */
static uint16_t prot_cnt = 0;

/* ===== 对外接口 ===== */

void Recovery_NotifyZeroForceEnter(void)
{
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

void Recovery_NotifyZeroForceExit(void)
{
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

uint8_t Recovery_StepTakeover(void)
{
    if (!(step_leg_motion[0].phase == STEP_LEG_READY &&
          step_leg_motion[1].phase == STEP_LEG_READY))
        return 0;

    /* 接管当拍保持短腿目标，下一拍交给现有 S5 起立及挡位逻辑。 */
    lift_l = lift_r = rec_l_low;
    lift_anchor = 1;
    lift_top = 0;
    ten_targets.l_leg_length_target = rec_l_low;
    ten_targets.r_leg_length_target = rec_l_low;
    plan_l = plan_r = (LegFrictionPlan_s){0};

    /* 旧离地位可能记的是收腿阶段，清锁存后按正常节拍重新判定。 */
    GRF_ClearAirState();
    return 1;
}

uint8_t Recovery_Update(float pitch_rad, float roll_rad,
                        float l_theta_body, float r_theta_body,
                        uint8_t chassis_mode)
{
    /* ===== 态0: 上电位形判定 =====
     * 进入(态0 三路位形判定, 见下方判定处): 大 |pitch| 仰翻->态7;
     *   腿收拢或摆角出竖直带(含双腿伸直摊开瘫)->态1 自动流程; 轮着地直立->态6。 */
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

    /* 段切换快照(原态7 块前位置): S0 判定当拍的切换不计入, 态7/T3 触发的切换计入 */
    const uint8_t entry_stage = prone_rec_state;

    /* ===== 态7: 翻倒过渡/翻身（2026-11-05 SPR 式, 2026-11-09 编号 4->7）=====
     * 进态即脱力（输出闸清零），等操作手"切零力再切出"的确认沿才武装
     * （检测→脱力无害，动作需人确认——11-03 教训的隔离层; prot_force_arm 供 ep 跳过）。
     * 武装后: 腿长匀速伸到 prot_l_max 造支点 + θ_body 追赶式目标恒速摆
     * （目标=实测∓step 每拍重设: 腿能动则恒速摆, 被压住则目标跟着实测走、永不失配）。
     * 方向按 Pitch 符号（prot_pitch_back 反向, 仅 ±90° 可靠区间）; 双腿差大时差速纠偏。
     * 到位（机身翻正+双腿回窗）驻留 -> 态1 伸腿接统一流程; 超时回脱力等重新切档。 */
    if (prone_rec_state == PRS_FLIP)
    {
        /* 启动闸门: seen_zf 已在零力进入沿记录(本块内的记录不可达, 零力期任务 return);
         * 切出(非零力)后到达本块 -> seen_zf || ep 强制 -> 武装 */
        if (chassis_mode == CHASSIS_ZERO_FORCE)
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
            const float before_l = tilt4_l, before_r = tilt4_r;
            if (tilt4_l < prot_l_max) tilt4_l += prot_l_speed;
            if (tilt4_r < prot_l_max) tilt4_r += prot_l_speed;
            ten_targets.l_leg_length_target = tilt4_l;
            ten_targets.r_leg_length_target = tilt4_r;
            plan_l.length_step = tilt4_l - before_l;
            plan_r.length_step = tilt4_r - before_r;
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
                if (prot_step > 0.0f)
                {
                    plan_l.theta_speed = dir * rec_done_zero_rate * step_l / prot_step;
                    plan_r.theta_speed = dir * rec_done_zero_rate * step_r / prot_step;
                }
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
        const float before_l = ext_l, before_r = ext_r;
        if (ext_l < rec_l_rise) ext_l += rec_extend_rate;
        if (ext_l > rec_l_rise) ext_l -= rec_extend_rate;
        if (ext_r < rec_l_rise) ext_r += rec_extend_rate;
        if (ext_r > rec_l_rise) ext_r -= rec_extend_rate;
        ten_targets.l_leg_length_target = ext_l;
        ten_targets.r_leg_length_target = ext_r;
        plan_l.length_step = ext_l - before_l;
        plan_r.length_step = ext_r - before_r;
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
        plan_l.length_step = step_l;
        plan_r.length_step = step_r;
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
        const float before_l = lift_l, before_r = lift_r;
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
        plan_l.length_step = lift_l - before_l;
        plan_r.length_step = lift_r - before_r;
        const uint8_t switching_length = lift_top;
        if (!lift_top && lift_l >= rec_l_rise && lift_r >= rec_l_rise)
        {
            leg_len_cmd_ramp = rec_l_rise;
            if (lift_l != rec_l_rise)
                plan_l.length_step = 0.0f;
            if (lift_r != rec_l_rise)
                plan_r.length_step = 0.0f;
            lift_top = 1;
        }
        if (lift_top)
        {
            lift_l = leg_len_cmd_ramp;
            lift_r = leg_len_cmd_ramp;
            if (switching_length)
                plan_l.length_step = plan_r.length_step = switch_length_step;
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
            const float before_theta_l = rec_tht_l, before_theta_r = rec_tht_r;
            float st0 = rec_done_zero_rate * CHASSIS_TICK_S;
            if (fabsf(rec_tht_l) > st0) rec_tht_l -= (rec_tht_l > 0.0f) ? st0 : -st0; else rec_tht_l = 0.0f;
            if (fabsf(rec_tht_r) > st0) rec_tht_r -= (rec_tht_r > 0.0f) ? st0 : -st0; else rec_tht_r = 0.0f;
            ten_targets.l_theta_target = rec_tht_l;
            ten_targets.r_theta_target = rec_tht_r;
            plan_l.theta_speed = (rec_tht_l - before_theta_l) / CHASSIS_TICK_S;
            plan_r.theta_speed = (rec_tht_r - before_theta_r) / CHASSIS_TICK_S;
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
        plan_l.theta_speed = ChaseReferenceSpeed(end_l - cont_l, rec_align_eps, ahead, rec_done_zero_rate);
        plan_r.theta_speed = ChaseReferenceSpeed(end_r - cont_r, rec_align_eps, ahead, rec_done_zero_rate);
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

    return (prone_rec_state != entry_stage);
}
