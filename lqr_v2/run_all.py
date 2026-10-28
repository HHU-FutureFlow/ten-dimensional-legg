"""一键生成：互证 → 完整 K → 板凳 K → C 代码。用法：python run_all.py [腿长m]"""
import sys, io, contextlib
import numpy as np
import model_lagrange as ML
import model_newton as MN
import design as DS
import bench as BN
from params import TS

L = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
tag = f"l{int(round(L*100)):03d}"
np.set_printoptions(precision=4, suppress=True, linewidth=170)

# 互证
p = ML.default_params(L); pc = dict(p, Ilz=0.0, Iwz=0.0)
M, K_, Bq = ML.numeric(pc)
e = max(np.abs(-np.linalg.solve(M, K_) - MN.linear_model(pc)[0]).max(),
        np.abs(np.linalg.solve(M, Bq) - MN.linear_model(pc)[1]).max())
print(f"[互证] 拉格朗日 vs 牛顿-欧拉 最大绝对误差 {e:.1e}")

D = DS.design(p)
print(f"\n===== 完整 4x10 K  (l={L} m, Ts={TS*1e3:.0f} ms) =====")
r = DS.report(p, D)
Kb, I = BN.bench_design(L, 0.0)
print(f"\n===== 板凳 K (髋锁 θ_rel=0) =====  闭环最慢 {I['cl'].real.max():.2f}, 延迟裕度 {I['md']} ms")
jt = BN.joint_targets(L, 0.0)

def cblock(K, name, comment):
    rows = []
    for i in range(4):
        rows.append("    { " + ", ".join(f"{v:+11.6f}f" for v in K[i]) + " }")
    return (f"/* {comment}\n"
            f" * 列序: s | ds | yaw | dyaw | lθ | ldθ | rθ | rdθ | φ | dφ\n"
            f" * 行序: [0]=l_Tw [1]=r_Tw [2]=l_Tp [3]=r_Tp ;  LQR_calc 里 T = -u = -K(x-x_ref)，rate = 1.0\n"
            f" * 由 lqr_v2/run_all.py 生成，固件符号约定，直接粘贴，不要再翻任何列 */\n"
            f"float {name}[4][10] =\n{{\n" + ",\n".join(rows) + "\n};\n")

full = cblock(D['K'], 'K', f"完整 10 维 LQR，腿长 l={L} m，1 kHz。闭环最慢极点 {r['maxRe']:.2f} rad/s，"
              f"可容忍额外延迟 {r['max_delay_ms']:.0f} ms")
bench = cblock(Kb, 'K', f"板凳模式（髋位置环锁死，K[2]/K[3] 全 0），l={L} m。角度增益放在 θl/θr 列（不放 φ 列），"
               f"\n * 这样平衡点与锁定角无关。延迟裕度 {I['md']} ms")
bench += ("\n/* 推荐锁定位姿 l=%.2f m, θ_rel=0 对应的位置指令（DMMotorSetRef 原始值）:\n"
          " *   r_front  %+.5f\n *   r_behind %+.5f\n"
          " *   l_front  %+.5f   <- 与现有代码 -2.77733 同一圈分支；标定时(docx)那一圈对应 %+.5f，\n"
          " *                        绝不能直接用后者：电机会转一整圈去找它\n"
          " *   l_behind %+.5f\n"
          " * 下发前先用 Ozone 看当前读数，确认与目标差值 < 0.5 rad 再切位置模式。 */\n") % (
    L, jt['r_front'], jt['r_behind'], jt['l_front_raw'] - 2 * np.pi, jt['l_front_raw'], jt['l_behind_raw'])
open(f"out/K_full_{tag}.c", "w", encoding="utf-8").write(full)
open(f"out/K_bench_{tag}.c", "w", encoding="utf-8").write(bench)
np.savetxt(f"out/A_code_{tag}.txt", D['Ac'], fmt="%+.6e"); np.savetxt(f"out/B_code_{tag}.txt", D['Bc'], fmt="%+.6e")
print(full); print(bench)
