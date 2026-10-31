"""
左腿重新标定计算器。读数一律取 Ozone 里 chassis_dbg.leg_pos[2]（左前）/ leg_pos[3]（左后），
它们已经按 ChassisTask 取过负，和 balance.c 里用的是同一套符号，不要再手动取负。

用法：
  python calib_left.py  F_v  F_long  B_long
    F_v    : 左前大腿竖直朝下时 leg_pos[2]
    F_long : 轮心在髋正下方、腿最长时 leg_pos[2]
    B_long : 同一位姿 leg_pos[3]
不带参数运行 = 用 docx 旧数据复算，结果应当复现旧常数（2.71324 / -0.7599），用来证明公式没错。
"""
import sys
import numpy as np
import kin

K0 = kin.K0
args = [float(a) for a in sys.argv[1:4]] if len(sys.argv) >= 4 else [-2.71324, -3.25151, -2.49161]
F_v, F_long, B_long = args
D = F_long - B_long                      # 大腿 - 小腿 差值 = β 修正量
alpha_long = F_long - F_v

def fk(big, small):
    return kin.leg(big - F_v, -(big - small) + K0 + D)

l_long, th_long = fk(F_long, B_long)
print(f"读数: F_v={F_v:+.5f}  F_long={F_long:+.5f}  B_long={B_long:+.5f}")
print(f"→ α 零点 = {F_v:+.5f}   β 修正 D = 大腿-小腿 = {D:+.5f}")
print("\n自检：")
print(f"  腿最长位姿 α = {np.degrees(alpha_long):+.2f}°   (几何应为 -31.17°，偏差 {np.degrees(alpha_long)+31.17:+.2f}°)")
print(f"  腿最长位姿 正解 l = {l_long:.4f} m (应≈0.405)  θ = {np.degrees(th_long):+.2f}° (应≈0)")
bad = [n for n, v in (('F_v', F_v), ('F_long', F_long), ('B_long', B_long)) if abs(v) > 2.5]
print("  " + ("⚠ 读数 %s 绝对值 > 2.5 rad，靠近 ±π，零点没设好，先别改代码" % bad if bad else "读数都远离 ±π ✓"))
if abs(np.degrees(alpha_long) + 31.17) > 3:
    print("  ⚠ α 偏差 > 3°，检查'大腿竖直'是否摆准（或两个位姿之间电机掉过电）")

print("\n—— balance.c  l_Alpha_beta_calculate() 改成：")
print(f"    l_leg->l_alpha = (l_motor_big_leg {-F_v:+.5f}f);")
print(f"    l_leg->l_beta  = -(l_motor_big_leg - l_motor_small_leg) {D:+.5f}f + ((31.17*2)*PI/180.0f);")

beta = kin.beta_for_length(0.30)
alpha = 0.0 - beta / 2
big = alpha + F_v
small = big + beta - K0 - D
print("\n—— chassis.c 板凳锁腿目标（l=0.30 m, θ_rel=0），左腿下发的是原始值 = 取负：")
print(f"    DMMotorSetRef(l_front_8009_motor,  {-big:+.5f}f);")
print(f"    DMMotorSetRef(l_behind_8009_motor, {-small:+.5f}f);")
print(f"    （核对：代回正解 l={fk(big, small)[0]:.4f}, θ={np.degrees(fk(big, small)[1]):+.3f}°）")
