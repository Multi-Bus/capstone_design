"""Consensus 편대 제어.

u_i = k_f * sum_j (p_j - p_i - (d_j - d_i)) + k_g * (c_ref + d_i - p_i)
  d_i   : 대형에서 i의 자리 (편대 중심 기준 오프셋)
  c_ref : 편대 중심이 지금 따라갈 점 (A* 웨이포인트)
"""
import numpy as np


def formation_vel(p_i, d_i, nbrs, c_ref, k_f=0.5, k_g=0.6, vmax=1.0):
    """nbrs: [(p_j, d_j), ...] 이웃 예측 위치와 대형 오프셋. 원하는 속도를 돌려준다."""
    p_i, d_i = np.asarray(p_i, float), np.asarray(d_i, float)
    u = k_g * (np.asarray(c_ref, float) + d_i - p_i)
    for p_j, d_j in nbrs:
        u = u + k_f * (np.asarray(p_j, float) - p_i - (np.asarray(d_j, float) - d_i))
    n = np.linalg.norm(u)
    return u * (vmax / n) if n > vmax else u


class PathFollower:
    """편대 중심용 웨이포인트 추종. 중심이 웨이포인트 반경 안에 들어오면 다음으로 넘어간다."""

    def __init__(self, waypoints, reach=0.3):
        self.wp = [np.asarray(w, float) for w in waypoints]
        self.i = 0
        self.reach = reach

    def ref(self, center):
        while self.i < len(self.wp) - 1 and np.linalg.norm(self.wp[self.i] - center) < self.reach:
            self.i += 1
        return self.wp[self.i]

    def done(self, center):
        return self.i == len(self.wp) - 1 and np.linalg.norm(self.wp[-1] - center) < self.reach
