"""用拉格朗日方程的完整非线性形式做时域仿真（RK4, 1 kHz, 含 1 拍指令延迟和力矩限幅）。"""
import numpy as np
import sympy as sp
import model_lagrange as ML
from params import TS, TW_MAX, TP_MAX

_q = ML.q
_qd = [sp.diff(x, ML.t) for x in _q]
_qdd = [sp.diff(x, ML.t, 2) for x in _q]
_Q = sp.symbols('q0:5'); _QD = sp.symbols('qd0:5'); _QDD = sp.symbols('qdd0:5')
_sub = {}
for a, b in zip(_qdd, _QDD): _sub[a] = b
for a, b in zip(_qd, _QD): _sub[a] = b
for a, b in zip(_q, _Q): _sub[a] = b
_eqs = [e.subs(_sub) for e in ML.eqs]   # 顺序替换：先二阶导再一阶导再函数
_Mnl = sp.Matrix([[sp.diff(e, a) for a in _QDD] for e in _eqs])
_rhs = sp.Matrix([-e.subs({a: 0 for a in _QDD}) for e in _eqs])
_par = list(ML.SYMS.values())
_fM = sp.lambdify([_Q, _QD, ML.u, _par], _Mnl, 'numpy')
_fR = sp.lambdify([_Q, _QD, ML.u, _par], _rhs, 'numpy')


def make_f(p):
    pv = [p[k] for k in ML.SYMS.keys()]
    def f(x, u):
        q, qd = x[:5], x[5:]
        M = np.array(_fM(q, qd, u, pv), float)
        r = np.array(_fR(q, qd, u, pv), float).ravel()
        return np.concatenate([qd, np.linalg.solve(M, r)])
    return f


def code_state(x, p):
    R, Rl, l = p['R'], p['Rl'], p['l']
    xl, xr, tl, tr, ph = x[:5]; dxl, dxr, dtl, dtr, dph = x[5:]
    s = (xl + xr) / 2 + R * (tl + tr) / 2
    ds = (dxl + dxr) / 2 + R * (dtl + dtr) / 2
    yaw = ((xr - l * np.sin(tr)) - (xl - l * np.sin(tl))) / (2 * Rl)
    dyaw = ((dxr - l * np.cos(tr) * dtr) - (dxl - l * np.cos(tl) * dtl)) / (2 * Rl)
    return np.array([s, ds, yaw, dyaw, tl, dtl, tr, dtr, ph, dph])


def sim(p, K, x0, T=3.0, delay=1, push=None):
    f = make_f(p)
    n = int(T / TS)
    x = np.array(x0, float)
    lim = np.array([TW_MAX, TW_MAX, TP_MAX, TP_MAX])
    buf = [np.zeros(4)] * delay
    Z = np.zeros((n, 10)); U = np.zeros((n, 4))
    for k in range(n):
        z = code_state(x, p)
        u = np.clip(-K @ z, -lim, lim)
        buf.append(u); ua = buf.pop(0)
        if push: ua = ua + push(k * TS)
        k1 = f(x, ua); k2 = f(x + TS / 2 * k1, ua); k3 = f(x + TS / 2 * k2, ua); k4 = f(x + TS * k3, ua)
        x = x + TS / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Z[k] = z; U[k] = u
        if not np.all(np.isfinite(x)) or abs(x[4]) > 1.5 or abs(x[2]) > 1.5:
            return Z[:k + 1], U[:k + 1], False
    return Z, U, True
