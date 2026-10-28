"""两种推导互证：拉格朗日 vs 牛顿-欧拉。偏航用同一 I_z 口径（腿、轮自身偏航惯量置 0）。"""
import numpy as np
import model_lagrange as ML
import model_newton as MN

np.set_printoptions(precision=6, suppress=True, linewidth=160)
worst = 0
for l in [0.20, 0.30, 0.40]:
    for Jr in [0.0, 1.2e-5]:
        p = ML.default_params(l)
        p['Jr'] = Jr
        p['Ilz'] = 0.0; p['Iwz'] = 0.0
        M, K, Bq = ML.numeric(p)
        Aq_L = -np.linalg.solve(M, K); Bu_L = np.linalg.solve(M, Bq)
        Aq_N, Bu_N = MN.linear_model(p)
        e = max(np.abs(Aq_L - Aq_N).max() / np.abs(Aq_L).max(),
                np.abs(Bu_L - Bu_N).max() / np.abs(Bu_L).max())
        worst = max(worst, e)
        print(f"l={l:.2f} Jr={Jr:.1e}  相对误差 {e:.2e}")
print("最大相对误差", worst)
p = ML.default_params(0.30); p['Ilz'] = 0; p['Iwz'] = 0
M, K, Bq = ML.numeric(p)
print("Aq(L)=\n", -np.linalg.solve(M, K)); print("Aq(N)=\n", MN.linear_model(p)[0])
print("Bu(L)=\n", np.linalg.solve(M, Bq)); print("Bu(N)=\n", MN.linear_model(p)[1])
