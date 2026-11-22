"""가상 모드 시뮬: 한 단계씩 진행하는 드론 6대.

평면(x, y)은 swarm/의 consensus 편대 속도 + CBF-QP 안전 필터 + 교착 완화,
운동은 swarm/simulator.py와 같은 1차 지연 (v' = (u - v) / tau).
편대 중심은 A* 경로(swarm.path_planning)를 PathFollower로 따라간다.
고도 z는 별도로 climb m/s로 올리고 내린다.
ponytail: 이웃 참값 사용, 통신 지연과 잡음 없음. 발표용 화면이라 충분,
필요하면 swarm/simulator.py의 이벤트 트리거를 붙인다.
"""
from collections import deque

import numpy as np

from swarm import collision_avoidance as avoid
from swarm import formation_control as consensus
from swarm import path_planning as astar

from . import formations

AIRBORNE = ("takeoff", "hover")
ROOM = 15.0  # 방 한 변 (m), 중심 (0, 0)
RES = 0.1  # A* 격자 (m)
PILLAR_R = 0.5  # 기둥 반경 (m)
FIXED_PILLARS = [(-3.5, 2.5), (3.0, -3.5), (1.0, 4.5)]


def _cell(p):
    h = ROOM / 2
    return int(round((p[1] + h) / RES)), int(round((p[0] + h) / RES))


def _point(c):
    h = ROOM / 2
    return round(c[1] * RES - h, 3), round(c[0] * RES - h, 3)


def _flood(g, start):
    """start 칸에서 막히지 않은 칸으로 8방향 이동해 닿는 칸들 (bool 배열)."""
    seen = np.zeros_like(g)
    if g[start]:
        return seen
    seen[start] = True
    q = deque([start])
    h, w = g.shape
    while q:
        y, x = q.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and not g[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((ny, nx))
    return seen


class SimSwarm:
    def __init__(self, n=6, dt=0.02, ctrl_dt=0.1, tau=0.3, vmax=1.0, D=0.4, gamma=1.0,
                 hover_z=1.0, climb=0.5):
        self.n, self.dt, self.tau, self.vmax, self.D, self.gamma = n, dt, tau, vmax, D, gamma
        self.hover_z, self.climb = hover_z, climb
        self.every = int(round(ctrl_dt / dt))
        self.p = formations.HOME[:n].copy()
        self.v = np.zeros_like(self.p)
        self.u = np.zeros_like(self.p)
        self.z = np.zeros(n)
        self.slot = self.p.copy()
        self.phase = ["ground"] * n
        self.formation = None
        self.dl = [avoid.Deadlock() for _ in range(n)]
        self.k = 0
        self.obstacles = [{"x": x, "y": y, "r": PILLAR_R, "fixed": True} for x, y in FIXED_PILLARS]
        self.center_ref = np.zeros(2)  # 편대 중심이 머무를 점
        self.goal = None  # (x, y) 또는 None
        self.path = []  # 남은 웨이포인트 [(x, y), ...]
        self.follower = None

    @property
    def t(self):
        return self.k * self.dt

    def _up(self):
        return [i for i in range(self.n) if self.phase[i] in AIRBORNE]

    def center(self):
        up = self._up()
        return self.p[up].mean(axis=0) if up else self.p.mean(axis=0)

    # 명령
    def start(self):
        self.phase = ["takeoff" if ph == "ground" else ph for ph in self.phase]
        if self.goal is None:
            self._fit()  # 지상에서 고른 편대가 이 자리에 안 들어가면 들어가는 곳으로

    def stop(self):
        self.phase = ["landing" if ph != "ground" else ph for ph in self.phase]
        self._arrive(self.center())

    def set_formation(self, name):
        if name not in formations.FORMATIONS:
            raise ValueError(f"모르는 편대: {name}")
        prev = self.slot
        self.slot = formations.assign(self.p - self.center(), formations.slots(name, self.n))
        if self._up() and not self._room_for().any():
            self.slot = prev
            raise ValueError("이 편대가 들어갈 공간이 없습니다")
        self.formation = name
        if self.goal is not None:
            self._replan()
        else:
            self._fit()

    def home(self):
        self.slot = formations.HOME[:self.n].copy()
        self.formation = None
        if self._up():
            try:
                self.goto(0.0, 0.0)
            except ValueError:  # 경로가 없으면 곧장 간다. CBF가 기둥과의 충돌은 막는다.
                self._arrive((0.0, 0.0))

    def goto(self, x, y):
        """편대 중심을 A* 경로로 (x, y)까지 보낸다. 편대가 들어갈 수 없는 점이면 갈 수 있는 가장 가까운 점으로
        옮기고 알림 문자열을 돌려준다. 갈 곳이 전혀 없으면 ValueError."""
        if not self._up():
            raise ValueError("먼저 시작하세요")
        h = ROOM / 2
        if not (abs(x) <= h and abs(y) <= h):
            raise ValueError("방 밖입니다")
        old = self.goal
        self.goal = (float(x), float(y))
        res = self._replan()
        if not res:
            self.goal = old
            if old is not None:
                self._replan()
            raise ValueError("경로 없음")
        return "편대가 들어갈 수 있는 가장 가까운 지점으로 옮겼습니다" if res == "snapped" else None

    def add_obstacle(self, x, y):
        """기둥을 놓는다. 이동 중이면 경로를 다시 계획하고, 막히면 그 자리에 멈추고 알림 문자열을 돌려준다."""
        h = ROOM / 2 - PILLAR_R
        if not (abs(x) <= h and abs(y) <= h):
            raise ValueError("방 밖입니다")
        if (np.linalg.norm(self.p - (x, y), axis=1) < PILLAR_R + 0.5).any():
            raise ValueError("드론과 너무 가깝습니다")
        if any(np.hypot(o["x"] - x, o["y"] - y) < 2 * PILLAR_R + 0.1 for o in self.obstacles):
            raise ValueError("다른 기둥과 겹칩니다")
        self.obstacles.append({"x": float(x), "y": float(y), "r": PILLAR_R, "fixed": False})
        if self.goal is not None and not self._replan():
            self._arrive(self.center())
            return "경로가 막혀 제자리에 멈췄습니다"
        if self.goal is None:
            self._fit()
        return None

    def clear_obstacles(self):
        self.obstacles = [o for o in self.obstacles if o["fixed"]]
        if self.goal is not None:
            self._replan()

    # 경로
    def _fit(self):
        """떠 있는 편대가 지금 자리(벽, 기둥)에 들어가지 않으면 가장 가까운 들어가는 점으로 옮긴다."""
        if not self._up():
            return
        _, gi = self._grid(self.center())
        if gi[_cell(self.center_ref)]:
            self.goal = tuple(map(float, self.center_ref))
            if not self._replan():
                self._arrive(self.center_ref)

    def _room_for(self):
        """지금 편대가 지금 위치에서 닿을 수 있고, 벽과 기둥에 걸리지 않고 설 수 있는 칸들 (bool 배열)."""
        g, gi = self._grid(self.center())
        return _flood(g, _cell(self.center())) & ~gi

    def _grid(self, start):
        """(시작점 주변을 비운 격자, 부풀린 격자). 장애물과 벽을 편대 반경 + 안전거리만큼 부풀린다."""
        size = int(round(ROOM / RES)) + 1
        yy, xx = np.mgrid[0:size, 0:size] * RES - ROOM / 2
        raw = np.zeros((size, size), bool)
        raw[[0, -1], :] = raw[:, [0, -1]] = True  # 벽
        for o in self.obstacles:
            raw |= (xx - o["x"]) ** 2 + (yy - o["y"]) ** 2 <= o["r"] ** 2
        r_form = np.linalg.norm(self.slot, axis=1).max()
        gi = astar.inflate(raw, int(np.ceil((r_form + self.D) / RES)))
        g = gi.copy()
        near = (xx - start[0]) ** 2 + (yy - start[1]) ** 2 <= (r_form + self.D) ** 2
        g[near] = raw[near]  # 기둥 옆에서 다시 계획할 때 시작 칸이 막히지 않게
        return g, gi

    def _replan(self):
        """경로를 다시 계획한다. True, 목표를 옮겼으면 "snapped", 갈 곳이 없으면 False."""
        start = self.center()
        g, _ = self._grid(start)
        ok = self._room_for()
        if not ok.any():
            return False
        goal_c = _cell(self.goal)
        snapped = not ok[goal_c]
        if snapped:  # 편대가 들어갈 수 없는 목표: 닿을 수 있는 가장 가까운 칸으로
            ys, xs = np.nonzero(ok)
            k = np.argmin((ys - goal_c[0]) ** 2 + (xs - goal_c[1]) ** 2)
            goal_c = (int(ys[k]), int(xs[k]))
            self.goal = _point(goal_c)
        cells = astar.astar(g, _cell(start), goal_c)
        if cells is None:
            return False
        self.path = [_point(c) for c in astar.shortcut(g, cells)]
        self.path[-1] = self.goal
        self.follower = consensus.PathFollower(self.path)
        return "snapped" if snapped else True

    def _arrive(self, point):
        self.center_ref = np.asarray(point, float)
        self.goal, self.path, self.follower = None, [], None

    # 진행
    def _control(self):
        up = self._up()
        u_prev = self.u.copy()  # 교착 판정은 직전 명령 속도로 한다
        self.u[:] = 0.0
        if self.follower is not None:
            c = self.center()
            ref = self.follower.ref(c)
            self.path = self.path[-(len(self.follower.wp) - self.follower.i):]
            if self.follower.done(c):
                self._arrive(self.goal)
                ref = self.center_ref
        else:
            ref = self.center_ref
        pillars = [(np.array([o["x"], o["y"]]), np.zeros(2), o["r"] + self.D) for o in self.obstacles]
        for i in up:
            js = [j for j in up if j != i]
            v_des = consensus.formation_vel(self.p[i], self.slot[i], [(self.p[j], self.slot[j]) for j in js],
                                            ref, vmax=self.vmax)
            v_des = self.dl[i].adjust(v_des, u_prev[i], np.linalg.norm(ref + self.slot[i] - self.p[i]), self.t,
                                      self.every * self.dt)
            # 벽: 가장 가까운 벽 위의 점을 멈춘 장애물로 본다 (여유 D/2). 1.5 m 안쪽 벽만.
            h = ROOM / 2
            px, py = self.p[i]
            walls = [(np.array(q), np.zeros(2), self.D / 2) for q, d in
                     (((h, py), h - px), ((-h, py), h + px), ((px, h), h - py), ((px, -h), h + py)) if d < 1.5]
            others = [(self.p[j], self.v[j], self.D) for j in js] + pillars + walls
            self.u[i], _ = avoid.cbf_qp(v_des, self.p[i], others, self.D, self.gamma, self.vmax)

    def step(self):
        if self.k % self.every == 0:
            self._control()
        self.v += (self.u - self.v) / self.tau * self.dt
        self.p += self.v * self.dt
        for i, ph in enumerate(self.phase):
            if ph == "takeoff":
                self.z[i] = min(self.hover_z, self.z[i] + self.climb * self.dt)
                if self.z[i] >= self.hover_z:
                    self.phase[i] = "hover"
            elif ph == "landing":
                self.z[i] = max(0.0, self.z[i] - self.climb * self.dt)
                if self.z[i] <= 0.0:
                    self.phase[i] = "ground"
                    self.v[i] = 0.0
        self.k += 1

    def run_for(self, seconds):
        for _ in range(int(round(seconds / self.dt))):
            self.step()

    def state(self):
        path = [self.center().tolist()] + [list(w) for w in self.path] if self.path else []
        return {"mode": "virtual", "formation": self.formation, "t": round(self.t, 3), "room": ROOM,
                "goal": list(self.goal) if self.goal else None, "path": path,
                "obstacles": [dict(o) for o in self.obstacles],
                "drones": [{"id": i + 1, "x": float(self.p[i, 0]), "y": float(self.p[i, 1]),
                            "z": float(self.z[i]), "phase": self.phase[i]} for i in range(self.n)]}
