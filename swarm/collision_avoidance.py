"""순간 회피: CBF-QP와 APF (E6 비교용).

CBF 조건 (이웃 j, 장애물은 v_j = 0):
  h = |p_i - p_j|^2 - D^2
  (p_i - p_j) . v_i >= (p_i - p_j) . v_j - 0.5 * gamma * h
QP: min |v - v_des|^2  s.t.  A v >= b,  |v_x|, |v_y| <= vmax
"""
import itertools

import numpy as np

OK, INFEASIBLE = 0, 1


def solve_qp(v_des, A, b, tol=1e-9):
    """min |v - v_des|^2 s.t. A v >= b 를 활성 제약 조합 전수 확인으로 푼다.

    변수 d개면 최적해는 최대 d개 제약이 활성인 면 위의 사영이다. 그 사영 중
    실행 가능한 것 가운데 가장 가까운 것이 최적해다. 반복이 없어 시간이 일정하다.
    ponytail: O(m^d), d = 2~3, m ~ 20 이면 충분. 이웃이 많아지면 OSQP로 교체.
    """
    v_des = np.asarray(v_des, float)
    if len(b) == 0 or np.all(A @ v_des >= b - tol):
        return v_des, OK
    d = len(v_des)
    best, best_dist = None, np.inf
    for k in range(1, d + 1):
        for rows in itertools.combinations(range(len(b)), k):
            As, bs = A[list(rows)], b[list(rows)]
            M = As @ As.T
            if abs(np.linalg.det(M)) < 1e-12:
                continue
            v = v_des + As.T @ np.linalg.solve(M, bs - As @ v_des)
            if np.all(A @ v >= b - 1e-7):
                dist = np.sum((v - v_des) ** 2)
                if dist < best_dist:
                    best, best_dist = v, dist
    if best is None:
        return np.zeros(d), INFEASIBLE
    return best, OK


def cbf_rows(p_i, others, D, gamma):
    """others: [(p_j, v_j) 또는 (p_j, v_j, D_j), ...]. 이웃 하나당 제약 한 줄. D_j가 있으면 그 값을 쓴다."""
    A, b = [], []
    for o in others:
        p_j, v_j = o[0], o[1]
        Dj = o[2] if len(o) > 2 else D
        n = np.asarray(p_i, float) - np.asarray(p_j, float)
        h = n @ n - Dj * Dj
        A.append(n)
        b.append(n @ np.asarray(v_j, float) - 0.5 * gamma * h)
    return A, b


def cbf_qp(v_des, p_i, others, D, gamma=1.0, vmax=1.0):
    """안전 속도와 상태(OK / INFEASIBLE). 해가 없으면 정지 명령(0)을 낸다."""
    d = len(v_des)
    A, b = cbf_rows(p_i, others, D, gamma)
    for k in range(d):  # 상자 제약 |v_k| <= vmax
        e = np.zeros(d)
        e[k] = 1.0
        A += [e, -e]
        b += [-vmax, -vmax]
    return solve_qp(v_des, np.array(A), np.array(b))


def apf(v_des, p_i, others, rho0, k_rep=0.5, vmax=1.0):
    """원하는 속도 + 이웃 척력. rho0 안쪽에서만 밀어낸다."""
    v = np.asarray(v_des, float).copy()
    for o in others:
        n = np.asarray(p_i, float) - np.asarray(o[0], float)
        dist = np.linalg.norm(n)
        r = dist - (o[2] if len(o) > 2 else 0.0)  # D_j가 있으면 표면까지 거리
        if dist > 1e-6 and 1e-6 < r < rho0:
            v += k_rep * (1.0 / r - 1.0 / rho0) / (r * r) * (n / dist)
    return np.clip(v, -vmax, vmax)


class Deadlock:
    """교착 탐지 · 완화. 목표에서 먼데 명령 속도가 거의 0인 상태가 hold초 이어지면 교착이다.
    완화: escape초 동안 원하는 속도를 오른쪽으로 angle만큼 돌린다 (모두 같은 방향이라 대칭이 깨진다).
    ponytail: 교란 방식만 넣었다. 우선순위, 목표 재배정은 이걸로 안 풀릴 때 추가.
    """

    def __init__(self, hold=1.0, escape=1.5, angle_deg=60.0, v_min=0.05, far=0.5):
        self.hold, self.escape, self.v_min, self.far = hold, escape, v_min, far
        c, s = np.cos(np.radians(-angle_deg)), np.sin(np.radians(-angle_deg))
        self.R = np.array([[c, -s], [s, c]])
        self.stall = 0.0
        self.until = -1.0
        self.count = 0

    def adjust(self, v_des, u_prev, dist_goal, t, dt):
        """원하는 속도를 돌려준다(교착 탈출 중이면 회전한 값)."""
        if t < self.until:
            return self.R @ v_des
        stalled = dist_goal > self.far and np.linalg.norm(u_prev) < self.v_min
        self.stall = self.stall + dt if stalled else 0.0
        if self.stall >= self.hold:
            self.stall, self.until = 0.0, t + self.escape
            self.count += 1
            return self.R @ v_des
        return v_des


def safe_distance(base, eps_self, eps_nbr, v_rel_max, age, clock_err=0.0):
    """안전거리 = 기본 + 내 오차 + 이웃 오차 + 상대속도 상한 x 정보 나이 + 시계 오차 (01 문서 2.2)."""
    return base + eps_self + eps_nbr + v_rel_max * age + clock_err
