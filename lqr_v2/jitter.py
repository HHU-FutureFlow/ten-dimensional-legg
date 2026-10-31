"""
轮子高频抖动分析：给定 K，评估
 (1) 每个状态通道的噪声 → 轮力矩 RMS 的放大倍数（闭环、离散 Lyapunov）
 (2) 对导数通道加一阶低通后的稳定性 / 延迟裕度
 (3) 最快闭环模态：频率、阻尼比、主要由哪些状态构成
"""
import numpy as np
import model_lagrange as ML, design as DS
from lqrlib import c2d, dare
from params import TS
np.set_printoptions(precision=3, suppress=True, linewidth=160)

K_CODE = np.array([
 [-2.531771,-2.879755,-1.212459,-0.483793, 7.447163,1.289258, 3.324787,0.667027,  2.114482, 0.330359],
 [-2.531771,-2.879755, 1.212459, 0.483793, 3.324787,0.667027, 7.447163,1.289258,  2.114482, 0.330359],
 [-2.401419,-2.487657, 6.905139, 2.503371,11.294617,1.315818,-3.959357,0.017008,-32.616841,-1.841675],
 [-2.401419,-2.487657,-6.905139,-2.503371,-3.959357,0.017008,11.294617,1.315818,-32.616841,-1.841675]])
DER = [1, 3, 5, 7, 9]          # ds dyaw dθl dθr dφ

p = ML.default_params()
Ac, Bc = DS.code_model(p)
Ad, Bd = c2d(Ac, Bc, TS)

def loop(K, fc=None, delay=1):
    """增广：对象 10 + 滤波 5 + 延迟 4*delay。返回 F 和噪声输入矩阵 G（噪声加在测量上）。"""
    n, m, nf = 10, 4, len(DER)
    a = 0.0 if fc is None else np.exp(-2 * np.pi * fc * TS)
    N = n + nf + m * delay
    F = np.zeros((N, N)); G = np.zeros((N, 10))
    F[:n, :n] = Ad
    # 滤波器：y_f[k+1] = a y_f[k] + (1-a) (x_der[k] + v_der)
    for i, j in enumerate(DER):
        F[n + i, n + i] = a; F[n + i, j] = 1 - a; G[n + i, j] = 1 - a
    # 控制器测量：非导数通道直接用 x+v，导数通道用 y_f（fc=None 时 a=0 即一拍延迟，等价于无滤波+1拍）
    Kx = K.copy(); Kf = np.zeros((m, nf))
    if fc is not None:
        for i, j in enumerate(DER):
            Kf[:, i] = K[:, j]; Kx[:, j] = 0
    u_row = n + nf
    # 指令队列
    F[u_row:u_row + m, :n] = -Kx; F[u_row:u_row + m, n:n + nf] = -Kf
    Gu = np.zeros((m, 10)); Gu[:, :] = -Kx
    G[u_row:u_row + m, :] = Gu
    for d in range(1, delay):
        F[u_row + d*m:u_row + (d+1)*m, u_row + (d-1)*m:u_row + d*m] = np.eye(m)
    F[:n, u_row + (delay-1)*m:u_row + delay*m] = Bd
    return F, G, u_row

def lyap_d(F, W, iters=400):
    X = W.copy(); Fk = F.copy()
    for _ in range(iters):                       # 倍增法
        X = X + Fk @ X @ Fk.T; Fk = Fk @ Fk
        if np.abs(Fk).max() < 1e-14: break
    return X

def analyse(K, name):
    print(f"\n===== {name} =====")
    lam, V = np.linalg.eig(Ad - Bd @ K)
    s = np.log(lam) / TS
    i = np.argmax(np.abs(s))
    w = np.abs(s[i]); z = -s[i].real / w
    v = np.abs(V[:, i]); v = v / v.max()
    print(f"最快闭环模态 {w/2/np.pi:.1f} Hz, 阻尼比 {z:.2f}, 主要状态: " +
          ", ".join(f"{DS.STATE_NAMES[j]}={v[j]:.2f}" for j in np.argsort(-v)[:3]))
    # 噪声敏感度：单位方差白噪声加在某个通道 → 轮力矩 RMS
    F, G, ur = loop(K, None, 1)
    print("单位噪声(σ=1) → 轮力矩 Tw RMS / 髋力矩 Tp RMS：")
    for j in DER + [4, 8]:
        X = lyap_d(F, np.outer(G[:, j], G[:, j]))
        tw = np.sqrt(X[ur, ur]); tp = np.sqrt(X[ur + 2, ur + 2])
        print(f"   {DS.STATE_NAMES[j]:<5}  Tw {tw:6.3f}   Tp {tp:6.3f}")
    print("导数通道加一阶低通 → 闭环 max|z| / 延迟裕度(ms)：")
    for fc in [None, 100, 50, 30, 20, 10]:
        md = None
        for d in range(1, 40):
            F, G, ur = loop(K, fc, d)
            if np.abs(np.linalg.eigvals(F)).max() >= 1: md = d - 2; break
        F, G, ur = loop(K, fc, 1)
        print(f"   fc={'无' if fc is None else fc:>4} Hz  max|z|={np.abs(np.linalg.eigvals(F)).max():.5f}  延迟裕度 {md} ms")

analyse(K_CODE, "当前 LQR.c 的 K")

# 软化版：放宽导数状态、加大力矩惩罚
Q, R = DS.weights(sx=dict(dth=6.0, ds=1.2, dphi=3.0, dyaw=3.0), su=dict(Tw=1.0, Tp=4.0))
Ks = dare(Ad, Bd, Q, R)[0]
analyse(Ks, "软化版 K（σ_dθ 3→6, σ_ds 0.8→1.2, σ_dφ 1.5→3, σ_dyaw 2→3, σ_Tw 1.5→1, σ_Tp 6→4）")
print("\n软化版 K =")
for r in Ks: print("    { " + ", ".join(f"{v:+10.6f}f" for v in r) + " },")
D = dict(K=Ks, Ac=Ac, Bc=Bc, Ad=Ad, Bd=Bd, res=0)
DS.report(p, D)
