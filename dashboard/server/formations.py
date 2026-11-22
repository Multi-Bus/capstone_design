"""편대 형태(자리 좌표)와 자리 배정. 좌표는 편대 중심 기준 (x, y) m."""
import itertools
import math

import numpy as np

FORMATIONS = ("line", "triangle", "circle")

# 시작 위치: 2행 3열, 간격 1.0 m, 중심 (0, 0)
HOME = np.array([(-1, -0.5), (0, -0.5), (1, -0.5), (-1, 0.5), (0, 0.5), (1, 0.5)], float)


def slots(name, n=6, spacing=1.0):
    """편대 이름의 자리 좌표 (n, 2). 중심(평균)이 (0, 0)."""
    if name == "line":
        s = np.c_[np.arange(n) * spacing, np.zeros(n)]
    elif name == "triangle":
        if n != 6:
            raise ValueError("triangle은 6대 전용")
        h = spacing * math.sqrt(3) / 2
        s = np.array([((k - r / 2) * spacing, -r * h) for r in range(3) for k in range(r + 1)])
    elif name == "circle":
        a = np.arange(n) * 2 * math.pi / n
        s = np.c_[np.cos(a), np.sin(a)] * spacing
    else:
        raise ValueError(f"모르는 편대: {name}")
    return s - s.mean(axis=0)


def assign(current, targets):
    """current[i]가 갈 자리를 돌려준다. 총 이동거리가 최소인 배정이라 직선 경로끼리 교차하지 않는다.
    ponytail: 순열 전수 조사(6대 720가지). 9대 이상이면 scipy linear_sum_assignment로 바꾼다.
    """
    current, targets = np.asarray(current, float), np.asarray(targets, float)
    d = np.linalg.norm(current[:, None] - targets[None], axis=-1)
    rows = np.arange(len(current))
    best = min(itertools.permutations(rows), key=lambda p: d[rows, p].sum())
    return targets[list(best)]
