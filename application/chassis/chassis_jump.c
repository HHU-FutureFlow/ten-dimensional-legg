#include "chassis_jump.h"

#if WHEELLEG_JUMP_ENABLE
#include "chassis.h"
#include <math.h>

/* 动作沿用同事 2026-11-20 源码的速度轨迹和阶段参数。当前工程的 GRF
 * 仅给跳跃提供双侧离地状态，分侧轮电机输出仍由 ChassisTask 负责。 */
enum { JP_IDLE, JP_SQUAT, JP_PUSH, JP_SHRINK, JP_LAND };
#define JUMP_SQUAT_MASS 5.1f

volatile uint8_t jump_dry_run = 0;
volatile float jump_squat_len = 0.15f;
volatile float jump_air_len = 0.28f;
volatile float jump_squat_rate = 0.6f;
volatile float jump_squat_acc = 5.0f;
volatile float jump_squat_vref = 0.0f;
volatile float jump_squat_vlim = 0.0f;
volatile float jump_squat_vramp = 0.0f;
volatile float jump_squat_acc_now = 0.0f;
volatile float jump_squat_v0 = 0.05f;
volatile float jump_squat_band = 0.015f;
volatile float jump_squat_ki = 400.0f;
volatile float jump_push_ramp = 0.15f;
volatile float jump_push_acc = 6.0f;
volatile float jump_push_mass = 5.875f;
volatile float jump_push_max_len = 0.44f;
volatile float jump_push_vramp = 0.0f;
volatile float jump_push_vref = 0.0f;
volatile float jump_push_acc_now = 0.0f;
volatile float jump_push_h = 0.30f;
volatile uint16_t jump_push_timeout = 800;
volatile float jump_push_sym_th = 0.060f;
volatile float jump_push_stuck_dl = 0.002f;
volatile uint16_t jump_push_stuck_ms = 50;
volatile uint16_t jump_push_stuck_t = 0;
volatile float jump_push_len_last = 0.0f;
volatile float jump_push_vtop = 0.0f;
volatile float jump_push_len_avg = 0.0f;
volatile float jump_push_vlaunch = 0.0f;
volatile uint8_t jump_push_abort = 0;
volatile float jump_push_eta = 0.75f;
volatile float jump_impulse_at = 0.320f;
volatile float jump_impulse_ff = 200.0f;
volatile uint16_t jump_impulse_ms = 5;
volatile float jump_impulse_taper = 0.03f;
volatile float jump_push_taper_k = 1.0f;
volatile uint8_t jump_push_imp = 0;
volatile uint16_t jump_push_imp_t = 0;
volatile float jump_push_imp_k = 0.0f;
volatile float jump_push_vbase = 0.0f;
volatile float jump_push_abase = 0.0f;
volatile float jump_shrink_rate = 1.4f;
volatile float jump_shrink_acc = 15.0f;
volatile float jump_shrink_mass = 1.35f;
volatile float jump_shrink_ff_max = 60.0f;
volatile float jump_shrink_ff_max_r = 60.0f;
volatile float jump_shrink_ff_l = 0.0f;
volatile float jump_shrink_ff_r = 0.0f;
volatile float jump_shrink_ramp = 0.36f;
volatile float jump_shrink_vref = 0.0f;
volatile float jump_shrink_vref_l = 0.0f;
volatile float jump_shrink_vref_r = 0.0f;
volatile float jump_shrink_vramp = 0.0f;
volatile float jump_shrink_acc_now = 0.0f;
volatile float jump_land_rate = 0.3f;
volatile float jump_land_ramp = 0.20f;
volatile uint16_t jump_land_buffer = 200;
volatile float jump_land_v0 = 0.05f;
volatile uint8_t jump_land_buf_done = 0;
volatile uint16_t jump_land_buf_t = 0;
volatile uint16_t jump_land_settle = 0;
volatile uint8_t jump_land_touched = 0;
volatile uint16_t jump_land_gnd_t = 0;
volatile float jump_land_ff_max = 150.0f;
volatile float jump_land_vref = 0.0f;
volatile float jump_idle_ramp = 0.25f;
volatile uint8_t jump_idle_return = 0;
volatile uint16_t jump_shrink_min = 150;
volatile float jump_shrink_done_vth = 0.15f;
volatile uint16_t jump_shrink_stuck_ms = 20;
volatile uint16_t jump_shrink_stuck_t = 0;
volatile uint16_t jump_shrink_wait = 150;
volatile uint8_t jump_shrink_airok = 0;
volatile uint8_t jump_shrink_hold = 0;
volatile uint16_t jump_squat_hold = 100;
volatile uint16_t jump_land_hold = 250;
volatile uint16_t jump_release_db = 100;
volatile uint16_t jump_squat_rel_t = 0;
volatile float jump_squat_tw_theta_scale = 1.0f;
volatile float jump_push_tp_phi_scale = 0.30f;
volatile uint8_t jump_ph = JP_IDLE;
volatile uint16_t jump_t = 0;
volatile float jump_squat_ramp = 0.30f;
volatile uint8_t jump_ready = 0;
static uint16_t jump_land_total = 0;
volatile uint16_t s_yaw_reanchor_ramp = 0;

static float squat_integral[2];

void ChassisJump_Reset(void)
{
    jump_ph = JP_IDLE;
    jump_t = 0;
    jump_ready = 0;
    jump_squat_rel_t = 0;
    jump_idle_return = 0;
    jump_shrink_hold = 0;
    jump_shrink_airok = 0;
    jump_land_touched = 0;
    jump_land_buf_done = 0;
    jump_land_buf_t = 0;
    jump_land_gnd_t = 0;
    jump_land_settle = 0;
    jump_land_total = 0;
    s_yaw_reanchor_ramp = 0;
    squat_integral[0] = squat_integral[1] = 0.0f;
}

static void BeginLengthReturn(float measured_length, ChassisJumpReference_s *ref)
{
    jump_ph = JP_IDLE;
    jump_t = 0;
    jump_ready = 0;
    jump_squat_rel_t = 0;
    jump_shrink_airok = 0;
    jump_land_buf_done = 0;
    jump_land_buf_t = 0;
    jump_land_touched = 0;
    jump_land_settle = 0;
    jump_land_gnd_t = 0;
    squat_integral[0] = squat_integral[1] = 0.0f;
    jump_idle_ramp = measured_length;
    jump_idle_return = 1;
    ref->restart_lift = 1;
}

static void BeginLanding(float measured_length)
{
    jump_ph = JP_LAND;
    jump_t = 0;
    jump_land_total = 0;
    s_yaw_reanchor_ramp = 300;
    jump_land_ramp = measured_length;
    jump_land_vref = 0.0f;
    jump_land_buf_done = 0;
    jump_land_buf_t = 0;
    jump_land_settle = 0;
    jump_land_touched = 0;
    jump_land_gnd_t = 0;
}

void ChassisJump_Update(chassis_jump_cmd_e command, const ChassisJumpInput_s *input,
                        ChassisJumpReference_s *ref)
{
    *ref = (ChassisJumpReference_s){0};
    const float length_avg = 0.5f * (input->length[0] + input->length[1]);
    const float velocity_avg = 0.5f * (input->velocity[0] + input->velocity[1]);
    const uint8_t squat_req = command == CHASSIS_JUMP_HELD;
    const uint8_t start_phase = jump_ph;
    float length_step = 0.0f;

    switch (jump_ph)
    {
    case JP_IDLE:
        if (input->can_start && squat_req)
        {
            jump_idle_return = 0;
            jump_ph = JP_SQUAT;
            jump_t = 0;
            jump_ready = 0;

            jump_squat_ramp = length_avg;
            jump_squat_vref  = 0.0f;
            jump_squat_vramp = 0.0f;
            jump_squat_acc_now = 0.0f;
            jump_squat_rel_t = 0;
        }
        break;

    case JP_SQUAT:
    {
        if (command == CHASSIS_JUMP_CANCEL)
        {
            BeginLengthReturn(length_avg, ref);
            break;
        }
        /* 两腿蹲稳后才准许松杆触发，掉线取消在驻留计数之前处理。 */
        {
            float remain = jump_squat_ramp - jump_squat_len;
            float vlim   = 0.0f;
            float dvmax  = jump_squat_acc * 0.001f;
            float vold   = jump_squat_vramp;

            if (remain > 0.0f)
            {
                vlim = sqrtf(2.0f * jump_squat_acc * remain);
                if (vlim > jump_squat_rate) vlim = jump_squat_rate;
            }

            if      (jump_squat_vramp < vlim - dvmax) jump_squat_vramp += dvmax;
            else if (jump_squat_vramp > vlim + dvmax) jump_squat_vramp -= dvmax;
            else                                      jump_squat_vramp  = vlim;
            if (jump_squat_vramp < 0.0f) jump_squat_vramp = 0.0f;

            jump_squat_acc_now = (jump_squat_vramp - vold) * 1000.0f;

            const float previous_length = jump_squat_ramp;
            jump_squat_ramp -= jump_squat_vramp * 0.001f;

            if (remain > 0.0f && jump_squat_ramp < jump_squat_len)
                jump_squat_ramp = jump_squat_len;

            length_step = jump_squat_ramp - previous_length;
            jump_squat_vlim = vlim;
            jump_squat_vref = -jump_squat_vramp;
        }

        uint8_t len_ok   = (fabsf(input->length[0] - jump_squat_len) < jump_squat_band)
                        && (fabsf(input->length[1] - jump_squat_len) < jump_squat_band);
        uint8_t still_ok = (fabsf(input->velocity[0]) < jump_squat_v0)
                        && (fabsf(input->velocity[1]) < jump_squat_v0);
        if (len_ok && still_ok)
        {
            if (jump_t < 0xFFFE) jump_t++;
            if (jump_t >= jump_squat_hold) jump_ready = 1;
        }
        else
        {
            jump_t = 0;
            jump_ready = 0;
        }

        if (squat_req)
            jump_squat_rel_t = 0;
        else if (jump_squat_rel_t < 0xFFFF)
            jump_squat_rel_t++;

        if (squat_req || jump_squat_rel_t < jump_release_db)
            break;

        if (jump_ready)
        {
            jump_ph = JP_PUSH;
            jump_t = 0;
            jump_push_ramp = jump_squat_len;
            jump_push_vramp = 0.0f;
            jump_push_vref  = 0.0f;
            jump_push_acc_now = 0.0f;
            jump_push_imp   = 0;
            jump_push_imp_t = 0;
            jump_push_imp_k = 0.0f;
            jump_push_stuck_t  = 0;
            jump_push_len_last = jump_squat_len;
        }
        else
        {
            BeginLengthReturn(length_avg, ref);
        }
        break;

    }

    case JP_PUSH:
    {
        jump_push_len_avg = length_avg;
        float len_dif = fabsf(input->length[0]
                            - input->length[1]);

        float MT       = 2.0f * jump_push_mass;

        float v_total  = sqrtf(2.0f * 9.81f * jump_push_h / jump_push_eta);

        float dx_imp   = jump_push_max_len - jump_impulse_at;
        if (dx_imp < 0.005f) dx_imp = 0.005f;
        float a_imp    = (2.0f * jump_impulse_ff - MT * 9.81f) / MT;
        float dx_tap   = jump_impulse_taper;
        if (dx_tap < 0.0f)            dx_tap = 0.0f;
        if (dx_tap > dx_imp - 0.005f) dx_tap = dx_imp - 0.005f;
        float dv2_imp  = 2.0f * a_imp * (dx_imp - dx_tap)
                       + 2.0f * dx_tap * (a_imp - jump_impulse_ff / MT);

        float vb2      = v_total * v_total - dv2_imp;
        float v_base   = (vb2 > 0.0f) ? sqrtf(vb2) : 0.0f;

        float dx_base  = jump_impulse_at - jump_squat_len;
        if (dx_base < 0.01f) dx_base = 0.01f;
        float a_base   = (v_base * v_base) / (2.0f * dx_base);
        if (a_base > jump_push_acc) a_base = jump_push_acc;

        jump_push_vtop  = v_total;
        jump_push_vbase = v_base;
        jump_push_abase = a_base;

        if (!jump_push_imp)
        {
            jump_push_vramp += a_base * 0.001f;
            if (jump_push_vramp > v_base) jump_push_vramp = v_base;
            const float previous_length = jump_push_ramp;
            jump_push_ramp += jump_push_vramp * 0.001f;
            if (jump_push_ramp > jump_impulse_at) jump_push_ramp = jump_impulse_at;

            if (jump_dry_run)
            {
                jump_push_vref    = 0.0f;
                jump_push_acc_now = 0.0f;
            }
            else
            {
                jump_push_vref    = jump_push_vramp;
                jump_push_acc_now = a_base;
                length_step = jump_push_ramp - previous_length;
            }
            jump_push_imp_k   = 0.0f;

            if (jump_push_len_avg >= jump_impulse_at)
            {
                jump_push_imp   = 1;
                jump_push_imp_t = 0;
            }
        }
        else
        {
            if (jump_push_imp_t < 0xFFFF) jump_push_imp_t++;
            {
                float r = (float)jump_push_imp_t
                        / (float)((jump_impulse_ms > 0u) ? jump_impulse_ms : 1u);
                if (r > 1.0f) r = 1.0f;
                jump_push_imp_k = r;
            }

            /* 冲量段的位置目标随测量值追赶。摩擦前馈沿既有蹬地速度计划伸腿。 */
            if (!jump_dry_run)
                length_step = v_total * 0.001f;
            jump_push_ramp    = jump_push_len_avg + 0.005f;
            jump_push_vref    = velocity_avg;
            jump_push_acc_now = 0.0f;
        }

        jump_t++;

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

        /* 用主工程的双侧离地快照判相位，轮端继续按侧门控。 */
        uint8_t why = 0;

        if      (jump_push_len_avg >= jump_push_max_len && input->airborne) why = 2;
        else if (input->airborne)                                          why = 1;
        else if (len_dif > jump_push_sym_th)                         why = 3;
        else if (jump_push_len_avg < jump_push_max_len
                 && jump_push_stuck_t >= jump_push_stuck_ms)         why = 5;
        else if (jump_t > jump_push_timeout)                         why = 4;
        jump_push_abort = why;
        if (why)
        {
            jump_push_vlaunch = velocity_avg;
            if (why <= 2)
            {
                jump_shrink_ramp = jump_push_len_avg;
                jump_shrink_vref = 0.0f;
                jump_shrink_vref_l = jump_shrink_vref_r = 0.0f;
                jump_shrink_stuck_t = 0;
                jump_shrink_vramp = 0.0f;
                jump_shrink_acc_now = 0.0f;

                jump_shrink_airok = 0;
                jump_shrink_hold  = 0;
                jump_ph = JP_SHRINK;
                jump_t = 0;
            }
            else
            {
                s_yaw_reanchor_ramp = 300;
                BeginLengthReturn(length_avg, ref);
            }
        }
    }
        break;

    case JP_SHRINK:
    {
        jump_t++;

        if (input->airborne)
            jump_shrink_airok = 1;
        if (!jump_shrink_airok)
        {
            /* 未确认离地时跟随后续实测，避免在地面主动收腿。 */
            if (jump_t < jump_shrink_wait)
            {
                float lavg = velocity_avg;
                jump_shrink_hold    = 1;
                jump_shrink_ramp    = length_avg;
                jump_shrink_vramp   = 0.0f;
                jump_shrink_acc_now = 0.0f;
                jump_shrink_vref    = (lavg > 0.0f) ? lavg : 0.0f;
                jump_shrink_vref_l  = jump_shrink_vref;
                jump_shrink_vref_r  = jump_shrink_vref;
                break;
            }
            jump_shrink_hold = 0;
            BeginLanding(length_avg);
            break;
        }
        if (jump_shrink_hold)
        {
            jump_shrink_hold = 0;
            jump_t = 0;
        }

        /* 规划减速距离决定速度上限，左右腿各自用剩余行程计算追赶参考。 */
        float remain = jump_shrink_ramp - jump_air_len;
        float vlim   = 0.0f;
        float dvmax  = jump_shrink_acc * 0.001f;
        float vold   = jump_shrink_vramp;

        if (remain > 0.0f)
        {
            vlim = sqrtf(2.0f * jump_shrink_acc * remain);
            if (vlim > jump_shrink_rate) vlim = jump_shrink_rate;
        }

        if      (jump_shrink_vramp < vlim - dvmax) jump_shrink_vramp += dvmax;
        else if (jump_shrink_vramp > vlim + dvmax) jump_shrink_vramp -= dvmax;
        else                                       jump_shrink_vramp  = vlim;
        if (jump_shrink_vramp < 0.0f) jump_shrink_vramp = 0.0f;

        jump_shrink_acc_now = (jump_shrink_vramp - vold) * 1000.0f;

        const float previous_length = jump_shrink_ramp;
        jump_shrink_ramp -= jump_shrink_vramp * 0.001f;
        if (remain > 0.0f && jump_shrink_ramp < jump_air_len)
            jump_shrink_ramp = jump_air_len;
        if (!jump_dry_run)
            length_step = jump_shrink_ramp - previous_length;

        {
            float el = input->length[0] - jump_air_len;
            float er = input->length[1] - jump_air_len;
            float vcl, vcr;
            if (el < 0.0f) el = -el;
            if (er < 0.0f) er = -er;
            vcl = sqrtf(2.0f * jump_shrink_acc * el);
            if (vcl > jump_shrink_rate) vcl = jump_shrink_rate;
            vcr = sqrtf(2.0f * jump_shrink_acc * er);
            if (vcr > jump_shrink_rate) vcr = jump_shrink_rate;
            jump_shrink_vref_l = -(jump_shrink_vramp > vcl ? jump_shrink_vramp : vcl);
            jump_shrink_vref_r = -(jump_shrink_vramp > vcr ? jump_shrink_vramp : vcr);

            jump_shrink_vref = (jump_shrink_vref_l > jump_shrink_vref_r)
                             ? jump_shrink_vref_l : jump_shrink_vref_r;
        }

        if (jump_t < jump_shrink_min)
            break;

        {
            float ldot_avg  = velocity_avg;
            float len_avg_s = length_avg;
            uint8_t do_land = 0;

            if (jump_shrink_vramp > 0.30f && ldot_avg > -0.20f)
            {
                if (jump_shrink_stuck_t < 0xFFFFu) jump_shrink_stuck_t++;
            }
            else jump_shrink_stuck_t = 0;
            if (jump_shrink_stuck_t >= jump_shrink_stuck_ms)
                do_land = 1;

            if (!do_land
                && !(jump_shrink_vramp > 0.02f && jump_t < 300))
            {
                uint8_t leg_done = (len_avg_s <= jump_air_len + 0.010f)
                                && (fabsf(ldot_avg) < jump_shrink_done_vth);
                if ((leg_done && !input->airborne) || jump_t > 800)
                    do_land = 1;
            }

            if (do_land)
            {
                BeginLanding(length_avg);
            }
        }
    }
        break;

    case JP_LAND:
    {
        jump_t++;
        jump_land_total++;

        float ldot_avg = velocity_avg;
        float len_avg  = length_avg;

        if (!jump_land_buf_done)
        {
            if (input->airborne)
            {
                jump_land_touched = 0;
                jump_land_buf_t   = 0;
                jump_land_gnd_t   = 0;
                jump_land_ramp    = input->stand_length;
            }
            else
            {
                if (!jump_land_touched)
                {
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

        jump_land_vref = 0.0f;
        if (jump_land_buf_done)
        {
            const float previous_length = jump_land_ramp;
            float lerr = input->stand_length - jump_land_ramp;
            if (fabsf(lerr) > 0.0005f)
            {
                float lstep = jump_land_rate * 0.001f;
                if (lerr > lstep)
                {   jump_land_ramp += lstep;
                    jump_land_vref  = +jump_land_rate; }
                else if (lerr < -lstep)
                {   jump_land_ramp -= lstep;
                    jump_land_vref  = -jump_land_rate; }
                else
                    jump_land_ramp  = input->stand_length;
            }
            length_step = jump_land_ramp - previous_length;
        }

        {
            uint8_t len_ok = (fabsf(len_avg - input->stand_length) < 0.01f);
            if (!input->airborne && len_ok && jump_land_buf_done)
            {
                if (jump_land_settle < 0xFFFF) jump_land_settle++;
                if (jump_land_settle >= jump_land_hold) jump_land_total = 2000;
            }
            else jump_land_settle = 0;
        }

        if (jump_land_total >= 2000)
        {
            s_yaw_reanchor_ramp = 300;
                BeginLengthReturn(length_avg, ref);
        }
    }
        break;

    default:
        BeginLengthReturn(length_avg, ref);
        break;
    }

    /* 切相重锚的参考位置可以跳变，只有相位内的计划运动进入前馈。 */
    ref->length_step = jump_ph == start_phase ? length_step : 0.0f;
    ref->active = jump_ph != JP_IDLE;
    ref->wheel_theta_scale = jump_ph == JP_SQUAT ? jump_squat_tw_theta_scale : 1.0f;
    ref->hip_pitch_scale = jump_ph == JP_PUSH ? jump_push_tp_phi_scale : 1.0f;
    ref->level_legs = jump_ph == JP_SQUAT || jump_ph == JP_PUSH ||
                      jump_ph == JP_LAND || (jump_ph == JP_SHRINK && !input->airborne);

    switch (jump_ph)
    {
    case JP_SQUAT:
        ref->length_target = jump_squat_ramp;
        break;
    case JP_PUSH:
        ref->length_target = jump_dry_run ? jump_squat_len : jump_push_ramp;
        break;
    case JP_SHRINK:
        ref->length_target = jump_dry_run ? jump_squat_len : jump_shrink_ramp;
        break;
    case JP_LAND:
        ref->length_target = jump_land_ramp;
        break;
    default:
        if (jump_idle_return)
        {
            const float goal = ref->restart_lift ? input->stand_length : input->return_length;
            const float error = goal - jump_idle_ramp;
            const float step = jump_land_rate * 0.001f;
            if (fabsf(error) > 0.0005f)
            {
                ref->length_step = fminf(step, fmaxf(-step, error));
                jump_idle_ramp += ref->length_step;
            }
            else
            {
                jump_idle_ramp = goal;
                jump_idle_return = 0;
            }
            ref->length_target = jump_idle_ramp;
        }
        break;
    }
}

void ChassisJump_Reanchor(const ten_balance_observation *observe, ten_balance_target *target)
{
    if (s_yaw_reanchor_ramp)
    {
        --s_yaw_reanchor_ramp;
        target->s_target += (observe->s - target->s_target) * 0.02f;
        target->yaw_target += (observe->yaw - target->yaw_target) * 0.02f;
    }
}

void ChassisJump_LegForce(const ChassisJumpInput_s *input, float length_target,
                          const float normal_limits[2], float force[2])
{
    float vref[2] = {0.0f, 0.0f};
    float limit[2] = {normal_limits[0], normal_limits[1]};
    float extra_ff[2] = {0.0f, 0.0f};
    float feedforward = LEG_GRAVITY_FF;
    float error[2] = {length_target - input->length[0], length_target - input->length[1]};

    switch (jump_ph)
    {
    case JP_SQUAT:
        vref[0] = vref[1] = jump_squat_vref;
        limit[0] = limit[1] = LEG_PID_OUT_MAX;
        feedforward -= JUMP_SQUAT_MASS * jump_squat_acc_now;
        break;
    case JP_PUSH:
        vref[0] = vref[1] = jump_push_vref;
        limit[0] = limit[1] = LEG_PID_OUT_MAX;
        if (!jump_dry_run)
        {
            float taper = 1.0f;
            if (jump_push_imp && jump_impulse_taper > 0.0005f)
                taper = fminf(1.0f, fmaxf(0.0f, (jump_push_max_len - jump_push_len_avg)
                                                    / jump_impulse_taper));
            jump_push_taper_k = taper;
            feedforward = (LEG_GRAVITY_FF + (jump_impulse_ff - LEG_GRAVITY_FF)
                                                * jump_push_imp_k) * taper
                          + jump_push_mass * jump_push_acc_now;
        }
        break;
    case JP_SHRINK:
        vref[0] = jump_shrink_vref_l;
        vref[1] = jump_shrink_vref_r;
        limit[0] = jump_shrink_ff_max;
        limit[1] = jump_shrink_ff_max_r;
        feedforward = 0.0f;
        if (!jump_shrink_hold)
        {
            /* 减速由位置与速度反馈处理，避免正前馈把空中的反力估计抬高。 */
            feedforward = fminf(0.0f, -jump_shrink_mass * jump_shrink_acc_now);
            extra_ff[0] = jump_shrink_ff_l;
            extra_ff[1] = jump_shrink_ff_r;
        }
        break;
    case JP_LAND:
        vref[0] = vref[1] = jump_land_vref;
        if (!jump_land_buf_done)
            limit[0] = limit[1] = jump_land_ff_max;
        if (input->airborne)
            feedforward = 0.0f;
        break;
    default:
        return;
    }

    /* 蹲底积分只在两腿都静稳且接近目标时累积，退出或掉出带立即释放。 */
    const uint8_t integrate = jump_ph == JP_SQUAT &&
        fabsf(error[0]) < 0.015f && fabsf(error[1]) < 0.015f &&
        fabsf(input->velocity[0]) < 0.05f && fabsf(input->velocity[1]) < 0.05f;
    for (uint8_t side = 0; side < 2; ++side)
    {
        if (integrate)
        {
            squat_integral[side] += jump_squat_ki * error[side] * 0.001f;
            squat_integral[side] = fminf(25.0f, fmaxf(-25.0f, squat_integral[side]));
        }
        else
        {
            squat_integral[side] = 0.0f;
        }
        float pid = leg_kp_pos * error[side] + squat_integral[side]
                    - JUMP_LEG_KV_VEL * (input->velocity[side] - vref[side]);
        pid = fminf(limit[side], fmaxf(-limit[side], pid));
        force[side] = feedforward + pid + extra_ff[side];
    }
}

#endif
