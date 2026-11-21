"""A* 경로 계획 (격자 맵, 8방향).

grid: bool 배열, True = 장애물. 좌표는 (행, 열) = (y, x) 칸 번호.
plan()은 m 단위 좌표를 받아 m 단위 웨이포인트를 돌려준다.
"""
import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


def inflate(grid, r_cells):
    """장애물을 반경 r_cells 칸만큼 부풀린다 (편대 반경 + 안전거리)."""
    out = grid.copy()
    h, w = grid.shape
    ys, xs = np.nonzero(grid)
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dy * dy + dx * dx > r_cells * r_cells:
                continue
            y, x = ys + dy, xs + dx
            ok = (y >= 0) & (y < h) & (x >= 0) & (x < w)
            out[y[ok], x[ok]] = True
    return out


def astar(grid, start, goal):
    """칸 경로 리스트를 돌려준다. 길이 없으면 None."""
    h, w = grid.shape
    if grid[start] or grid[goal]:
        return None

    def heur(c):
        dy, dx = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dy + dx) + (SQRT2 - 2.0) * min(dy, dx)  # octile

    g = {start: 0.0}
    parent = {start: None}
    pq = [(heur(start), start)]
    done = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        if cur in done:
            continue
        done.add(cur)
        for dy, dx, cost in MOVES:
            y, x = cur[0] + dy, cur[1] + dx
            if not (0 <= y < h and 0 <= x < w) or grid[y, x]:
                continue
            if dy and dx and (grid[cur[0] + dy, cur[1]] or grid[cur[0], cur[1] + dx]):
                continue  # 모서리 끼어들기 금지
            ng = g[cur] + cost
            if ng < g.get((y, x), math.inf):
                g[(y, x)] = ng
                parent[(y, x)] = cur
                heapq.heappush(pq, (ng + heur((y, x)), (y, x)))
    return None


def line_free(grid, a, b):
    """칸 a에서 b까지 직선이 장애물을 지나지 않으면 True (반 칸 간격 표본)."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for t in np.linspace(0.0, 1.0, n + 1):
        y = int(round(a[0] + (b[0] - a[0]) * t))
        x = int(round(a[1] + (b[1] - a[1]) * t))
        if grid[y, x]:
            return False
    return True


def shortcut(grid, path):
    """막히지 않는 웨이포인트를 건너뛰어 지그재그를 없앤다."""
    if not path:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(grid, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan(grid, res, start_xy, goal_xy, inflate_m=0.0):
    """m 좌표 (x, y) 시작점에서 목표점까지 웨이포인트 [(x, y), ...]. 길이 없으면 None."""
    g = inflate(grid, int(math.ceil(inflate_m / res))) if inflate_m > 0 else grid

    def cell(p):
        return (int(round(p[1] / res)), int(round(p[0] / res)))

    path = astar(g, cell(start_xy), cell(goal_xy))
    if path is None:
        return None
    return [(c[1] * res, c[0] * res) for c in shortcut(g, path)]
