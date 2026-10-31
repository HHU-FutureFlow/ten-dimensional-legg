"""
方法二：牛顿-欧拉法（逐刚体受力，含约束力和减速箱内力），数值求解线性化方程。
与 model_lagrange.py 完全独立（不共享任何动力学表达式），只共享参数和符号约定。

每侧刚体：轮(带行星架) / 转子 / 腿(含定子) ；整车：机体
未知量（13）：
  ẍ_l ẍ_r θ̈_l θ̈_r φ̈ ψ̈ | f_l f_r (地面摩擦) | F_l F_r (轮→腿 水平力) | H_l H_r (腿→机体 水平力)
  以及减速箱内力矩通过转子方程消去（见下）
"""
import numpy as np


def solve_accel(p, q, u):
    """给定线性化状态 q=[xl,xr,thl,thr,ph]（位置量只有角度起作用），输入 u，返回 q̈（5）和 ψ̈。"""
    R, mw, Iw, Rl, N, Jr = p['R'], p['mw'], p['Iw'], p['Rl'], p['N'], p['Jr']
    ml, mb, Ib, lc, Ibz, g = p['ml'], p['mb'], p['Ib'], p['lc'], p['Ibz'], p['g']
    l, lb, lw, Il = p['l'], p['lb'], p['lw'], p['Il']
    th = [q[2], q[3]]
    ph = q[4]
    Tw = [u[0], u[1]]
    Tp = [u[2], u[3]]

    # 未知量索引
    names = ['axl', 'axr', 'atl', 'atr', 'aph', 'aps',
             'fl', 'fr', 'Fl', 'Fr', 'Hl', 'Hr', 'te_l', 'te_r']
    # te_i: 电磁力矩（转子受 +te，定子受 -te），减速箱：轮受 N*te_g，转子受 -te_g ...
    # 为简洁起见引入 te_i 为"转子-定子间电磁力矩"，T_w 已知时由转子方程给出约束
    idx = {n: i for i, n in enumerate(names)}
    n = len(names)
    rows, rhs = [], []

    def eq(coef, b):
        r = np.zeros(n)
        for k, v in coef.items():
            r[idx[k]] += v
        rows.append(r); rhs.append(b)

    ax = ['axl', 'axr']; at = ['atl', 'atr']
    f = ['fl', 'fr']; F = ['Fl', 'Fr']; H = ['Hl', 'Hr']; te = ['te_l', 'te_r']

    # 静态竖直力（零阶）：腿→机体 Vh = mb g/2 ；轮→腿 V = (mb/2 + ml) g
    Vh = mb * g / 2
    Vw = Vh + ml * g

    for i in range(2):
        # 减速箱（理想行星，齿圈=定子，太阳=转子，行星架=轮）：
        #   ω_r - ω_s = N(ω_w - ω_s)，功率守恒 → 轮受 τ_gw = -N τ_gr，定子受 τ_gs = (N-1) τ_gr
        #   转子：Jr ω̇_r = τ_e + τ_gr       （τ_e 电磁力矩，定子受 -τ_e）
        #   T_w 定义 = 无转子惯量时作用在轮上的前滚力矩 = N τ_e
        # CCW 角加速度：轮 ω̇_w = -ẍ/R，定子 ω̇_s = θ̈，转子 ω̇_r = θ̈ + N(-ẍ/R - θ̈)
        # 这里用未知量 te_i 表示 τ_gr（减速箱作用在转子上的力矩）
        tau_e = -Tw[i] / N         # 前滚 = 顺时针，CCW 电磁力矩 = -T_w/N
        # 转子：Jr[θ̈ + N(-ẍ/R - θ̈)] = tau_e + τ_gr
        eq({at[i]: Jr * (1 - N), ax[i]: -Jr * N / R, te[i]: -1.0}, tau_e)
        # 轮转动（CCW）：Iw(-ẍ/R) = τ_gw + 摩擦力矩
        #   摩擦 f(前向) 作用于接地点 (0,-R)：力矩 = (0)(0) - (-R)(f) = +R f
        #   τ_gw = -N τ_gr
        eq({ax[i]: -Iw / R, te[i]: N, f[i]: -R}, 0.0)
        # 轮平动：mw ẍ = f - F
        eq({ax[i]: mw, f[i]: -1.0, F[i]: 1.0}, 0.0)
        # 腿平动：ml ẍ_lc = F - H，ẍ_lc = ẍ - lw θ̈
        eq({ax[i]: ml, at[i]: -ml * lw, F[i]: -1.0, H[i]: 1.0}, 0.0)
        # 腿转动（CCW，绕腿质心）：
        #   Il θ̈ = τ_gs + (-τ_e) + Tp + [轮心处力矩] + [髋处力矩]
        #   轮心相对质心 r1 = ( lw sinθ, -lw cosθ)，受力 (F, Vw)  → r1x*Vw - r1z*F = lw sinθ Vw + lw F
        #   髋相对质心   r2 = (-lb sinθ,  lb cosθ)，受力 (-H, -Vh) → r2x*(-Vh) - r2z*(-H) = lb sinθ Vh + lb H
        #   τ_gs = (N-1) τ_gr
        eq({at[i]: Il, te[i]: -(N - 1), F[i]: -lw, H[i]: -lb},
           -tau_e + Tp[i] + (lw * Vw + lb * Vh) * th[i])
    # 机体平动：mb ẍ_b = H_l + H_r
    #   ẍ_b = (ẍ_hl + ẍ_hr)/2 - lc φ̈，ẍ_h = ẍ - l θ̈
    eq({'axl': mb / 2, 'axr': mb / 2, 'atl': -mb * l / 2, 'atr': -mb * l / 2,
        'aph': -mb * lc, 'Hl': -1.0, 'Hr': -1.0}, 0.0)
    # 机体俯仰（CCW，绕机体质心）：
    #   髋相对质心 r = (lc sinφ, -lc cosφ)，受力 (H, Vh)：r_x Vh - r_z H = lc sinφ Vh + lc H
    eq({'aph': Ib, 'Hl': -lc, 'Hr': -lc}, -(Tp[0] + Tp[1]) + 2 * lc * Vh * ph)
    # 偏航：Ibz ψ̈ = Rl (H_r - H_l)   （右髋受前向力 → 左转）
    eq({'aps': Ibz, 'Hr': -Rl, 'Hl': Rl}, 0.0)
    # 偏航运动学：ψ = (x_hr - x_hl)/(2Rl)
    eq({'aps': 2 * Rl, 'axr': -1.0, 'atr': l, 'axl': 1.0, 'atl': -l}, 0.0)

    A = np.array(rows); b = np.array(rhs)
    assert A.shape[0] == A.shape[1], A.shape
    s = np.linalg.solve(A, b)
    return s[:5], s[5]


def linear_model(p):
    """返回 q̈ = -Minv K q + Minv Bq u 中的 (Aq, Bu)：q̈ = Aq q + Bu u"""
    Aq = np.zeros((5, 5)); Bu = np.zeros((5, 4))
    z5 = np.zeros(5); z4 = np.zeros(4)
    a0, _ = solve_accel(p, z5, z4)
    for j in range(5):
        e = z5.copy(); e[j] = 1.0
        Aq[:, j] = solve_accel(p, e, z4)[0] - a0
    for j in range(4):
        e = z4.copy(); e[j] = 1.0
        Bu[:, j] = solve_accel(p, z5, e)[0] - a0
    return Aq, Bu
