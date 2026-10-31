"""
轮腿 LQR 物理参数（独立重建，不依赖 lqr_sim/）
单位：SI。所有"腿"参数均为单侧一条腿。

来源：
  [表]  d0744b5e...png 参数表（l=0.40 m 时的值）
  [docx] 标定结果.docx
  [用户] 2026-10-28 对话
  [估]  本文件内估算，见注释
"""
import numpy as np

# ---------------- 设计工作点 ----------------
L_LEG   = 0.30          # [用户] 腿长先按 30 cm
TS      = 1e-3          # 控制周期 1 kHz（StartCHASSISTASK osDelay(1)）
G       = 9.81

# ---------------- 轮 ----------------
R_W     = 0.060         # [表] 驱动轮半径
M_W     = 0.500         # [表] 轮旋转组件质量（单侧）
I_W     = 6.95e-4       # [表] 轮绕轮轴惯量
R_L     = 0.24          # [用户] 半轮距
N_GEAR  = 15.8          # 自改减速比（balance.h）
J_ROTOR = 1.2e-5        # [估] 3508 转子惯量，手册未给，按转子尺寸估 1e-5~1.5e-5
                        #      反射到轮端 J*N^2 = 3.0e-3，与 I_w+m_w R^2 同量级，必须计入

# ---------------- 腿（五连杆整体，单侧） ----------------
M_L     = 1.674         # [表]
LEG_A   = 0.21          # 大腿（髋 -> 膝）balance.h
LEG_B   = 0.25          # 小腿（膝 -> 轮心）
LB_04   = 0.1067        # [表] l=0.40 时 腿质心 -> 髋 的轴向距离
LW_04   = 0.2933        # [表] l=0.40 时 腿质心 -> 轮心
IL_04   = 2.57e-2       # [表] l=0.40 时 腿绕自身质心惯量

# ---------------- 机体 ----------------
M_B     = 4.326         # [表] 车架 + 两髋电机 + 髋部 CNC
I_B     = 4.19e-2       # [表] 机体绕质心俯仰惯量
L_C     = 0.00494       # [表] 机体质心在髋轴连线上方的距离（可信度低）
I_BZ    = 0.10          # [估] 机体绕自身质心的偏航惯量，见 estimate_IBZ()

# ---------------- 执行器 ----------------
TW_MAX  = 16384 / 3318.833   # 3508 + C620 轮端峰值 4.94 N·m
TP_MAX  = 20.0               # [估] 单腿等效髋力矩允许值（两台 DM8009 合成）


def fivebar(l):
    """同轴五连杆：已知虚拟腿长 l（腿竖直向下），返回 膝、轮心 坐标（髋为原点，z 向上）。"""
    # 轮心 D=(0,-l)；膝 C/B 到髋距离 A，到 D 距离 B，左右对称
    # 设膝在 (±x, z)：x^2+z^2=A^2, x^2+(z+l)^2=B^2
    z = (LEG_B**2 - LEG_A**2 - l**2) / (2 * l)
    x = np.sqrt(LEG_A**2 - z**2)
    return np.array([x, z]), np.array([-x, z]), np.array([0.0, -l])


def leg_mass_model():
    """
    用 l=0.40 的表值反推五连杆质量分布，再算其他腿长。
    模型：两根大腿各 m_a（均质杆）、两根小腿各 m_b（均质杆）、轮心处集中质量 m_d。
    表里只给了两个条件（总质量、质心位置），三个未知数不定，取 m_d = 0（轮毂电机定子
    算进了 m_w 的平动部分，见 README），由此解出 m_a, m_b。
    惯量：五连杆点质量模型算出的惯量与表值不一致（表值来自 URDF），因此只用模型的
    "随腿长的变化比例"去缩放表值。
    """
    def com_depth(l, ma, mb):
        c, b, d = fivebar(l)
        za = c[1] / 2            # 大腿质心 z（两根对称，x 抵消）
        zb = (c[1] + d[1]) / 2   # 小腿质心 z
        return -(2 * ma * za + 2 * mb * zb) / (2 * ma + 2 * mb)

    # 解 2ma+2mb = M_L,  com_depth(0.4) = LB_04
    c, b, d = fivebar(0.40)
    za, zb = c[1] / 2, (c[1] + d[1]) / 2
    # -(2ma za + 2mb zb) = LB_04 * M_L ; 2ma + 2mb = M_L
    Amat = np.array([[2, 2], [-2 * za, -2 * zb]])
    ma, mb = np.linalg.solve(Amat, [M_L, LB_04 * M_L])

    def inertia_about_com(l):
        c, b, d = fivebar(l)
        lb = com_depth(l, ma, mb)
        com = np.array([0.0, -lb])
        I = 0.0
        for p0, p1, m, L in [((0, 0), c, ma, LEG_A), ((0, 0), b, ma, LEG_A),
                             (c, d, mb, LEG_B), (b, d, mb, LEG_B)]:
            p0 = np.array(p0, float); p1 = np.array(p1, float)
            mid = (p0 + p1) / 2
            I += m * L**2 / 12 + m * np.sum((mid - com)**2)
        return I

    return ma, mb, com_depth, inertia_about_com


def leg_params(l=L_LEG):
    ma, mb, com_depth, Icom = leg_mass_model()
    lb = com_depth(l, ma, mb)
    lw = l - lb
    Il = IL_04 * Icom(l) / Icom(0.40)
    return dict(l=l, lb=lb, lw=lw, Il=Il, ma=ma, mb=mb)


def estimate_IBZ():
    """
    机体偏航惯量（绕机体自身质心）。
    I_B=0.0419 按长方体 m(L^2+H^2)/12，取 H=0.12 → L≈0.31 m。
    宽度方向：髋电机 + CNC 约 2×0.6 kg 位于 y=±0.19 m，其余 3.1 kg 视作宽 0.30 m 的板。
    """
    H = 0.12
    L = np.sqrt(12 * I_B / M_B - H**2)
    m_hip = 1.2
    m_rest = M_B - m_hip
    I = m_rest * (L**2 + 0.30**2) / 12 + m_hip * 0.19**2
    return I, L


def all_params(l=L_LEG):
    p = dict(R=R_W, mw=M_W, Iw=I_W, Rl=R_L, N=N_GEAR, Jr=J_ROTOR,
             ml=M_L, mb=M_B, Ib=I_B, lc=L_C, Ibz=I_BZ, g=G)
    p.update(leg_params(l))
    return p


if __name__ == "__main__":
    ma, mb, cd, Ic = leg_mass_model()
    print(f"五连杆质量拟合: 大腿每根 {ma:.3f} kg, 小腿每根 {mb:.3f} kg")
    for l in [0.20, 0.25, 0.30, 0.35, 0.40]:
        q = leg_params(l)
        print(f"l={l:.2f}  l_b={q['lb']:.4f}  l_w={q['lw']:.4f}  I_l={q['Il']:.5f}")
    I, L = estimate_IBZ()
    print(f"I_bz 估算 = {I:.4f} (机体长度 {L:.3f} m)")
    print(f"转子反射惯量 J*N^2 = {J_ROTOR*N_GEAR**2:.4e}, I_w + m_w R^2 = {I_W+M_W*R_W**2:.4e}")
    print(f"TW_MAX = {TW_MAX:.3f}")
