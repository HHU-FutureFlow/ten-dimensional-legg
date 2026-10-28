"""不依赖 scipy 的 LQR 工具：expm、ZOH 离散化、CARE(Hamilton)、DARE(SDA)。"""
import numpy as np


def expm(M, terms=30):
    nrm = np.abs(M).sum(axis=1).max()
    s = max(0, int(np.ceil(np.log2(nrm))) + 1) if nrm > 0.5 else 0
    X = M / 2**s
    E = np.eye(M.shape[0]); term = np.eye(M.shape[0])
    for k in range(1, terms):
        term = term @ X / k
        E = E + term
    for _ in range(s):
        E = E @ E
    return E


def c2d(A, B, Ts):
    n, m = B.shape
    Mx = np.zeros((n + m, n + m)); Mx[:n, :n] = A; Mx[:n, n:] = B
    E = expm(Mx * Ts)
    return E[:n, :n], E[:n, n:]


def care(A, B, Q, R):
    Ri = np.linalg.inv(R)
    n = A.shape[0]
    H = np.block([[A, -B @ Ri @ B.T], [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    Vs = V[:, w.real < 0]
    assert Vs.shape[1] == n
    P = np.real(Vs[n:] @ np.linalg.inv(Vs[:n]))
    P = (P + P.T) / 2
    K = Ri @ B.T @ P
    res = A.T @ P + P @ A - P @ B @ Ri @ B.T @ P + Q
    return K, P, np.abs(res).max()


def dare(A, B, Q, R, iters=80):
    n = A.shape[0]
    Ak = A.copy(); Gk = B @ np.linalg.solve(R, B.T); Hk = Q.copy()
    I = np.eye(n)
    for _ in range(iters):
        W = np.linalg.inv(I + Gk @ Hk)
        A1 = Ak @ W @ Ak
        G1 = Gk + Ak @ W @ Gk @ Ak.T
        H1 = Hk + Ak.T @ Hk @ W @ Ak
        if np.abs(H1 - Hk).max() < 1e-12 * max(1, np.abs(H1).max()):
            Hk = H1; break
        Ak, Gk, Hk = A1, G1, H1
    P = (Hk + Hk.T) / 2
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    res = A.T @ P @ A - P - A.T @ P @ B @ K + Q
    return K, P, np.abs(res).max()


def delayed_cl(Ad, Bd, K, d):
    """u_k = -K x_k，但作用到对象上延迟 d 个采样。返回增广闭环矩阵。"""
    n, m = Bd.shape
    if d == 0:
        return Ad - Bd @ K
    N = n + d * m
    F = np.zeros((N, N))
    F[:n, :n] = Ad
    F[:n, n + (d - 1) * m: n + d * m] = Bd          # 最老的指令作用到对象
    F[n:n + m, :n] = -K                              # 新指令入队
    for i in range(1, d):
        F[n + i * m: n + (i + 1) * m, n + (i - 1) * m: n + i * m] = np.eye(m)
    return F


def max_delay(Ad, Bd, K, dmax=60):
    for d in range(dmax + 1):
        if np.abs(np.linalg.eigvals(delayed_cl(Ad, Bd, K, d))).max() >= 1.0:
            return d - 1
    return dmax
