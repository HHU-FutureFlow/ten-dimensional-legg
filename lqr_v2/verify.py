import numpy as np
import model_lagrange as ML
import design as DS
import nonlinear as NL
from lqrlib import care, max_delay
from params import TS

np.set_printoptions(precision=3, suppress=True, linewidth=170)
p0 = ML.default_params()
D = DS.design(p0)
K = D['K']

print("=== 1. 连续 CARE 与离散 DARE 的 K 对比 ===")
Kc, _, resc = care(D['Ac'], D['Bc'], D['Q'], D['R'])
print(f"max|K_d-K_c|/max|K| = {np.abs(K-Kc).max()/np.abs(K).max():.3%}  (CARE 残差 {resc:.1e})")

print("\n=== 2. 参数失配鲁棒性（K 按标称设计，作用于失配对象） ===")
cases = [('标称', {}),
         ('J_rotor=0', dict(Jr=0.0)), ('J_rotor=3e-5', dict(Jr=3e-5)),
         ('l_c=-0.03 (质心在髋下)', dict(lc=-0.03)), ('l_c=+0.05', dict(lc=0.05)),
         ('I_bz×0.5', dict(Ibz=p0['Ibz']*0.5)), ('I_bz×2', dict(Ibz=p0['Ibz']*2)),
         ('I_l×0.6', dict(Il=p0['Il']*0.6)), ('I_l×1.5', dict(Il=p0['Il']*1.5)),
         ('l_b-0.03', dict(lb=p0['lb']-0.03, lw=p0['lw']+0.03)),
         ('l_b+0.03', dict(lb=p0['lb']+0.03, lw=p0['lw']-0.03)),
         ('m_b+2kg (加云台)', dict(mb=p0['mb']+2.0)), ('I_b×2', dict(Ib=p0['Ib']*2)),
         ('R_l=0.19', dict(Rl=0.19)),
         ('腿长实际0.25', 'L025'), ('腿长实际0.35', 'L035')]
print(f"{'工况':<22}{'max|z|':>9}{'连续maxRe':>11}{'延迟裕度ms':>11}")
for name, mod in cases:
    if mod == 'L025': p = ML.default_params(0.25)
    elif mod == 'L035': p = ML.default_params(0.35)
    else: p = dict(p0, **mod)
    Ac, Bc = DS.code_model(p)
    from lqrlib import c2d
    Ad, Bd = c2d(Ac, Bc, TS)
    mz = np.abs(np.linalg.eigvals(Ad - Bd @ K)).max()
    mr = np.linalg.eigvals(Ac - Bc @ K).real.max()
    md = max_delay(Ad, Bd, K, 40)
    print(f"{name:<22}{mz:9.5f}{mr:11.3f}{md:11d}")

print("\n=== 3. 非线性仿真（完整拉格朗日方程，RK4 1kHz，1拍延迟，力矩限幅） ===")
tests = [('φ 15°', np.r_[0,0,0,0, 0, 0,0,0,0,0], 4, np.radians(15)),
         ('θ 两腿 15°', None, 'th', np.radians(15)),
         ('ds 1.5 m/s 冲击', None, 'v', 1.5),
         ('yaw 1 rad', None, 'yaw', 1.0)]
for name, _, kind, mag in tests:
    x0 = np.zeros(10)
    if kind == 4: x0[4] = mag
    elif kind == 'th': x0[2] = x0[3] = mag
    elif kind == 'v': x0[5] = x0[6] = mag
    elif kind == 'yaw':
        # 纯偏航偏差：右轮前、左轮后
        x0[0] = -mag * p0['Rl']; x0[1] = mag * p0['Rl']
    Z, U, ok = NL.sim(p0, K, x0, T=4.0)
    print(f"{name:<16} {'稳定' if ok else '发散'}  峰值|Tw|={np.abs(U[:,:2]).max():.2f}  峰值|Tp|={np.abs(U[:,2:]).max():.2f}"
          f"  φmax={np.degrees(np.abs(Z[:,8]).max()):.1f}°  θmax={np.degrees(np.abs(Z[:,[4,6]]).max()):.1f}°"
          f"  s范围=[{Z[:,0].min():.2f},{Z[:,0].max():.2f}]  终值max|z|={np.abs(Z[-1]).max():.1e}")
