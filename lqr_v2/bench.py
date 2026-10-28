"""
板凳模式：两髋位置环锁死在 θ_rel = θ0（两腿相同），只用轮子两行。
约束 θ_l = θ_r = θ, φ = θ - θ0，在约束后的拉格朗日量上重新推导（髋力矩为约束力，不做虚功）。
降维坐标 r = [x_l, x_r, θ]；固件观测 s = 平均x + Rθ，yaw = (x_r-x_l)/(2R_l)，θ_l=θ_r=θ，φ=θ-θ0。
"""
import numpy as np
import sympy as sp
import model_lagrange as ML
from lqrlib import c2d, dare, max_delay
from params import TS, TW_MAX
import design as DS

t = ML.t
th = sp.Function('th')(t)
th0 = sp.symbols('theta0')
sub = {ML.thl: th, ML.thr: th, ML.ph: th - th0}
Tb = ML.T.subs(sub).doit()
Vb = ML.V.subs(sub).doit()
Wb = ML.Twl * (ML.xl / ML.R + th) + ML.Twr * (ML.xr / ML.R + th)
rq = [ML.xl, ML.xr, th]
Lb = Tb - Vb
eqs = [sp.diff(sp.diff(Lb, sp.diff(c, t)), t) - sp.diff(Lb, c) - sp.diff(Wb, c) for c in rq]
X = sp.symbols('r0:3'); XD = sp.symbols('rd0:3'); XDD = sp.symbols('rdd0:3')
s2 = {}
for c, a, b, e in zip(rq, X, XD, XDD):
    s2[sp.diff(c, t, 2)] = e
for c, a, b, e in zip(rq, X, XD, XDD):
    s2[sp.diff(c, t)] = b
for c, a in zip(rq, X):
    s2[c] = a
eqs = [e.subs(s2) for e in eqs]
par = list(ML.SYMS.values()) + [th0]
fM = sp.lambdify([X, par], sp.Matrix([[sp.diff(e, a) for a in XDD] for e in eqs]), 'numpy')
grav = sp.lambdify([X, par], sp.Matrix([e.subs({a: 0 for a in XDD}).subs({a: 0 for a in XD}).subs(
    {ML.Twl: 0, ML.Twr: 0}) for e in eqs]), 'numpy')
fB = sp.lambdify([X, par], sp.Matrix([[-sp.diff(e, u) for u in (ML.Twl, ML.Twr)] for e in eqs]), 'numpy')


def bench_design(l, theta0, sx=None, su=None):
    p = ML.default_params(l)
    pv = [p[k] for k in ML.SYMS.keys()] + [theta0]
    # 平衡点：dV/dθ = 0
    g = lambda a: float(np.array(grav([0, 0, a], pv), float)[2, 0])
    a = 0.0
    for _ in range(50):
        h = 1e-7; a -= g(a) / ((g(a + h) - g(a - h)) / (2 * h))
    r0 = np.array([0, 0, a])
    M = np.array(fM(r0, pv), float); Bq = np.array(fB(r0, pv), float)
    Kq = np.zeros((3, 3))
    for j in range(3):
        e = np.zeros(3); e[j] = 1e-6
        Kq[:, j] = (np.array(grav(r0 + e, pv), float) - np.array(grav(r0 - e, pv), float)).ravel() / 2e-6
    Aq = -np.linalg.solve(M, Kq); Bu = np.linalg.solve(M, Bq)
    A = np.block([[np.zeros((3, 3)), np.eye(3)], [Aq, np.zeros((3, 3))]])
    B = np.vstack([np.zeros((3, 2)), Bu])
    R, Rl = p['R'], p['Rl']
    Cq = np.array([[0.5, 0.5, R], [-1 / (2 * Rl), 1 / (2 * Rl), 0], [0, 0, 1]])
    P = np.zeros((6, 6))
    for k in range(3):
        P[2 * k, :3] = Cq[k]; P[2 * k + 1, 3:] = Cq[k]
    Ac = P @ A @ np.linalg.inv(P); Bc = P @ B
    sx = dict(DS.SIGMA_X, **(sx or {})); su = dict(DS.SIGMA_U, **(su or {}))
    Q = np.diag(1 / np.array([sx['s'], sx['ds'], sx['yaw'], sx['dyaw'], sx['phi'], sx['dphi']])**2)
    Rw = np.diag(1 / np.array([su['Tw'], su['Tw']])**2)
    Ad, Bd = c2d(Ac, Bc, TS)
    Kb, _, res = dare(Ad, Bd, Q, Rw)
    # 嵌入 4x10：角度增益平分到 θl/θr 两列（锁髋时 θ 的平衡点 ≈ 0，与锁定角无关；φ 的平衡点 = -θ0）
    K10 = np.zeros((4, 10))
    for i in range(2):
        K10[i, 0:4] = Kb[i, 0:4]
        K10[i, 4] = K10[i, 6] = Kb[i, 4] / 2
        K10[i, 5] = K10[i, 7] = Kb[i, 5] / 2
    info = dict(theta_eq=a, phi_eq=a - theta0, ol=np.linalg.eigvals(Ac),
                cl=np.linalg.eigvals(Ac - Bc @ Kb), maxz=np.abs(np.linalg.eigvals(Ad - Bd @ Kb)).max(),
                md=max_delay(Ad, Bd, Kb), res=res, Ad=Ad, Bd=Bd, Kb=Kb)
    # 峰值力矩
    for name, z0 in [('φ5', [0, 0, 0, 0, np.radians(5), 0]), ('ds0.5', [0, 0.5, 0, 0, 0, 0]),
                     ('s0.3', [0.3, 0, 0, 0, 0, 0])]:
        z = np.array(z0, float); pk = 0; pkth = 0
        for k in range(3000):
            u = -Kb @ z; pk = max(pk, np.abs(u).max()); z = Ad @ z + Bd @ u; pkth = max(pkth, abs(z[4]))
        info['pk_' + name] = (pk, np.degrees(pkth))
    return K10, info


def joint_targets(l, theta_rel):
    """虚拟腿 (l, θ_rel) → 四个电机原始位置指令（DMMotorSetRef 位置模式用原始值，左腿取负）。"""
    import kin
    beta = kin.beta_for_length(l)
    alpha = theta_rel - beta / 2
    # 右：alpha = big - 0.67311 ; beta = -(big - small) + K0 + 0.56
    K0 = 2 * 31.17 * np.pi / 180
    rb = alpha + 0.67311; rs = rb + beta - K0 - 0.56
    # 左（代码取负后的值）：alpha = big + 2.71234 ; beta = -(big-small) - 0.7599 + K0
    lb = alpha - 2.71234; ls = lb + beta + 0.7599 - K0
    return dict(r_front=rb, r_behind=rs, l_front_raw=-lb, l_behind_raw=-ls)


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True, linewidth=170)
    for l, t0 in [(0.30, 0.0), (0.353, np.radians(7.2))]:
        K10, I = bench_design(l, t0)
        print(f"\n--- 板凳 l={l} θ_rel={np.degrees(t0):.1f}° ---")
        print(f"平衡点 θ={np.degrees(I['theta_eq']):.3f}°  φ={np.degrees(I['phi_eq']):.3f}°")
        print("开环", np.sort_complex(I['ol'])); print("闭环", np.sort_complex(I['cl']))
        print(f"max|z|={I['maxz']:.5f} 延迟裕度 {I['md']} ms  残差 {I['res']:.1e}")
        print("峰值|Tw| / θmax°:", {k[3:]: tuple(round(x, 2) for x in v) for k, v in I.items() if k.startswith('pk_')})
        print(K10[:2])
