"""
方法一：拉格朗日法（sympy 符号推导），直接在固件的符号约定下建模。

平面坐标（从车左侧看）：x 向前，z 向上，角度一律逆时针(CCW)为正。
  θ_i : 世界系虚拟腿角，轮心在髋前方为正（= 固件 l_theta/r_theta）
  φ   : 机体俯仰，车头上抬为正（= 固件 phi = -Pitch）
  x_i : 轮心前向位置（真实对地）
  ψ   : 机体偏航，左转为正（= 固件 yaw）

广义坐标 q = [x_l, x_r, θ_l, θ_r, φ]
输入     u = [T_w,l, T_w,r, T_p,l, T_p,r]
  T_w : 轮毂电机输出力矩，驱动轮子"前滚"为正；反作用于小腿（定子），使 θ 增大
  T_p : 髋部等效力矩（VMC 的 Tp），使 θ_rel = θ - φ 增大为正；反作用于机体
"""
import sympy as sp
import numpy as np
from params import all_params

t = sp.symbols('t')
xl, xr, thl, thr, ph = [sp.Function(n)(t) for n in ('xl', 'xr', 'thl', 'thr', 'ph')]
q = [xl, xr, thl, thr, ph]
R, mw, Iw, Rl, N, Jr, ml, mb, Ib, lc, Ibz, g, l, lb, lw, Il, Ilz, Iwz = sp.symbols(
    'R m_w I_w R_l N J_r m_l m_b I_b l_c I_bz g l l_b l_w I_l I_lz I_wz', positive=True)
Twl, Twr, Tpl, Tpr = sp.symbols('T_wl T_wr T_pl T_pr')
u = [Twl, Twr, Tpl, Tpr]

def d(f): return sp.diff(f, t)

T = 0
V = 0
hips = []
for x, th in ((xl, thl), (xr, thr)):
    # 轮：平动 + 自转（前滚=顺时针 → CCW 角速度 = -ẋ/R）
    T += sp.Rational(1, 2) * mw * d(x)**2 + sp.Rational(1, 2) * Iw * (d(x) / R)**2
    # 转子：定子随小腿以 θ̇ 转动，转子-定子 = N(轮-定子)
    w_rotor = d(th) + N * (-d(x) / R - d(th))
    T += sp.Rational(1, 2) * Jr * w_rotor**2
    # 腿质心
    xlc = x - lw * sp.sin(th)
    zlc = R + lw * sp.cos(th)
    T += sp.Rational(1, 2) * ml * (d(xlc)**2 + d(zlc)**2) + sp.Rational(1, 2) * Il * d(th)**2
    V += ml * g * zlc
    hips.append((x - l * sp.sin(th), R + l * sp.cos(th)))

xh = (hips[0][0] + hips[1][0]) / 2
zh = (hips[0][1] + hips[1][1]) / 2
xb = xh - lc * sp.sin(ph)
zb = zh + lc * sp.cos(ph)
T += sp.Rational(1, 2) * mb * (d(xb)**2 + d(zb)**2) + sp.Rational(1, 2) * Ib * d(ph)**2
V += mb * g * zb

# 偏航：机体偏航角由两髋前后差决定（左右轮、腿的偏航平动已含在 x_l, x_r 里）
psi = (hips[1][0] - hips[0][0]) / (2 * Rl)
T += sp.Rational(1, 2) * (Ibz + 2 * Ilz + 2 * Iwz) * d(psi)**2

# 广义力（虚功）
#   轮：相对前滚角 = x/R + θ（轮顺时针 - 定子顺时针(-θ)）
#   髋：θ_rel = θ - φ
Wv = (Twl * (xl / R + thl) + Twr * (xr / R + thr)
      + Tpl * (thl - ph) + Tpr * (thr - ph))
Q = [sp.diff(Wv, qi) for qi in q]

L = T - V
qd = [d(qi) for qi in q]
qdd = [d(d(qi)) for qi in q]
eqs = [sp.diff(sp.diff(L, qdi), t) - sp.diff(L, qi) - Qi for qi, qdi, Qi in zip(q, qd, Q)]

# 在平衡点 q=0, q̇=0 线性化：M q̈ + K q = Bq u
zero = {}
for qi in q:
    zero[qi] = 0
Msym = sp.zeros(5, 5)
Ksym = sp.zeros(5, 5)
Bsym = sp.zeros(5, 4)
for i, e in enumerate(eqs):
    for j in range(5):
        Msym[i, j] = sp.diff(e, qdd[j])
        Ksym[i, j] = sp.diff(e, q[j])
    for j in range(4):
        Bsym[i, j] = -sp.diff(e, u[j])

def at_eq(expr):
    expr = expr.subs({qdi: 0 for qdi in qdd}).subs({qdi: 0 for qdi in qd})
    return sp.simplify(expr.subs(zero))

Msym = Msym.applyfunc(at_eq)
Ksym = Ksym.applyfunc(at_eq)
Bsym = Bsym.applyfunc(at_eq)
SYMS = dict(R=R, mw=mw, Iw=Iw, Rl=Rl, N=N, Jr=Jr, ml=ml, mb=mb, Ib=Ib, lc=lc,
            Ibz=Ibz, g=g, l=l, lb=lb, lw=lw, Il=Il, Ilz=Ilz, Iwz=Iwz)


def numeric(p):
    sub = {SYMS[k]: v for k, v in p.items() if k in SYMS}
    M = np.array(Msym.subs(sub).evalf(), dtype=float)
    K = np.array(Ksym.subs(sub).evalf(), dtype=float)
    B = np.array(Bsym.subs(sub).evalf(), dtype=float)
    return M, K, B


def default_params(l=None):
    p = all_params() if l is None else all_params(l)
    # 腿、轮自身绕竖直轴的偏航惯量（小量）
    c = np.sqrt(0.21**2 - ((0.25**2 - 0.21**2 - p['l']**2) / (2 * p['l']))**2)  # 膝前后伸出量
    p['Ilz'] = (2 * p['ma'] + 2 * p['mb']) * c**2 / 3
    p['Iwz'] = p['Iw'] / 2
    return p


if __name__ == '__main__':
    sp.init_printing()
    print("M ="); sp.pprint(Msym)
    print("K ="); sp.pprint(Ksym)
    print("B ="); sp.pprint(Bsym)
    p = default_params()
    M, K, B = numeric(p)
    np.set_printoptions(precision=5, suppress=True, linewidth=150)
    print(M); print(K); print(B)
