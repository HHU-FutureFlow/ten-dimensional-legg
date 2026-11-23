#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H

#include <stdint.h>

/* ===== 常量宏 ===== */
/* 统一起立/自救状态机段编号（详见 chassis_recovery.c 头部流程说明） */
#define PRS_DETECT   0   /* 上电位形判定 */
#define PRS_EXTEND   1   /* 腿长整理到站立腿长 */
#define PRS_ALIGN    2   /* 双腿单向对齐(逆时针路程短者锚) */
#define PRS_PUSH     3   /* 蹬地顶升+腿角联动收腿 */
#define PRS_LQR_RAMP 4   /* LQR 输出斜坡接管(蹲姿) */
#define PRS_DONE     5   /* 起立(腿长斜坡升回)+完成, 常规控制 */
#define PRS_QUICK    6   /* 折叠位形反向快转(低位直接切 LQR) */
#define PRS_FLIP     7   /* 翻身(仰翻前置) */

/* ===== 可调参数（ep 在线整定, 符号名不得改动） =====
 * 此处仅 extern 主流程(chassis.c)需要读写的子集;
 * 全部 rec 前缀与 prot 前缀参数的定义与注释见 chassis_recovery.c。
 * 交叉引用: leg_len_cmd_ramp/switch_length_step 定义在 chassis.c(挡位斜坡)。 */
extern volatile float leg_len_cmd_ramp;  /* m, 挡位腿长斜坡当前值 */
extern float switch_length_step;         /* m, 挡位斜坡本拍增量(挡位块写, 态5 到顶沿读) */

extern volatile uint8_t prone_rec_state; /* 状态机当前段 0~7 */

/* 状态机产出/共享量（输出覆盖/台阶开窗/跳跃门控读取） */
extern volatile float rec2_ramp;         /* 态4 LQR 输出斜坡乘子 0~1 */
extern volatile float cur_l, cur_r;      /* 当前摆角目标(连续角, 覆盖段 PD 用) */
extern volatile float rec_tht_l, rec_tht_r; /* 态4 锚定/态5 收零的世界系 θ 目标 */
extern volatile uint8_t lift_top;        /* 态5 起立双腿到顶标志 */
extern volatile uint8_t lift_anchor;     /* 态5 起立斜坡锚定标志 */
extern volatile uint8_t rec6_anchor;     /* 通用进入沿标志 */
extern volatile uint8_t prot_armed;      /* 态7 已武装 */
extern volatile uint8_t prot_enable;     /* T3 监控使能 */

/* 主流程引用的恢复参数 */
extern volatile float rec_l_rise;        /* 站立/行驶腿长 m(唯一真源, 偏航模型系数联动) */
extern volatile float rec_l_low;         /* 蹲姿交接/收腿下限 m */
extern volatile float rec_pd_kp;         /* 摆角 PD 态1~4 增益组 */
extern volatile float rec_pd_kd;
extern volatile float rec_pd_tp_max;     /* N·m */
extern volatile float prot_kp_theta;     /* 态7 翻身 PD 增益组 */
extern volatile float prot_kd_theta;
extern volatile float prot_tp_max;       /* N·m */
extern volatile float rec_push_band;     /* 腿长误差门限 m(主流程腿长 PID 分段限幅) */
extern volatile float rec_push_max;      /* 大误差期间 PID 分量限幅 N */
extern volatile float rec_gravity_ff;    /* PD 段重力前馈 N/腿 */

/* ===== 对外接口 ===== */

/* 统一恢复状态机每拍执行（S0 判定 -> 态7 翻身 -> 态1~5 恢复流程 -> 追赶目标 -> T3 监控）。
 * 直接写 ten_targets / plan_l / plan_r / rec2_ramp / cur_* 等共享量。
 * 返回 1 = 本拍状态段发生切换（供主流程清恢复段遗留前馈）。
 * 快照点在 S0 判定之后、态7 之前 —— 与原实现一致, S0 判定当拍的切换不计入。 */
uint8_t Recovery_Update(float pitch_rad, float roll_rad,
                        float l_theta_body, float r_theta_body,
                        uint8_t chassis_mode);

/* 零力急停进入沿: S7 确认沿记录 + S5 起立锚清零（原零力进入沿内状态机操作） */
void Recovery_NotifyZeroForceEnter(void);

/* 零力急停恢复沿: 被中断的恢复段(S1~S4/S6)回 S0 重新判定（原零力恢复沿内状态机操作） */
void Recovery_NotifyZeroForceExit(void);

/* 台阶双侧 READY 接管的恢复侧动作: 蹲姿目标写入 + 起立锚置位 + GRF 离地锁存清除。
 * 返回 1 = 接管发生（主流程据此清 step_window 并复位台阶动作） */
uint8_t Recovery_StepTakeover(void);

#endif // CHASSIS_RECOVERY_H
