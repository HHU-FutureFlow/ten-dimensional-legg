"""固件 balance.c 的五连杆正解（逐行复刻），用于核对标定与反解锁定位姿。"""
import numpy as np
A, B = 0.21, 0.25
K0 = 2 * 31.17 * np.pi / 180

def leg(alpha, beta):
    c = np.array([A * np.sin(alpha), -A * np.cos(alpha)])
    b = np.array([A * np.sin(beta + alpha), -A * np.cos(beta + alpha)])
    bc = c - b; p = (b + c) / 2; BC = np.linalg.norm(bc)
    PD = np.sqrt(B * B - (BC / 2)**2); k = PD / BC
    d = p + np.array([-k * bc[1], k * bc[0]])
    return np.linalg.norm(d), np.arctan2(d[0], -d[1])

def right(big, small):  return leg(big - 0.67311, -(big - small) + K0 + 0.56)
def left(big, small):   return leg(big + 2.71234, -(big - small) - 0.7599 + K0)   # 输入为取负后的值

def beta_for_length(l):
    lo, hi = 0.01, np.pi - 0.01
    for _ in range(100):
        m = (lo + hi) / 2
        if leg(0, m)[0] > l: lo = m
        else: hi = m
    return (lo + hi) / 2

if __name__ == '__main__':
    print("右 轮心正下方", right(0.12531, -0.43469))
    print("左 轮心正下方", left(-3.25151, -2.49161))
    print("右 docx板凳", right(0.02766, -0.07801))
    print("右 代码锁定", right(-0.03529, -0.13294))
    print("左 代码锁定(取负)", left(2.77733, -2.31765), " 左 代码锁定(不取负)", left(-2.77733, 2.31765))
