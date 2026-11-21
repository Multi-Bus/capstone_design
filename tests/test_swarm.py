"""트랙 A 모듈 점검. 실행: python tests/test_swarm.py  (또는 python -m pytest -q tests)"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from swarm import collision_avoidance as avoid  # noqa: E402
from swarm import event_trigger as trigger  # noqa: E402
from swarm import formation_control as consensus  # noqa: E402
from swarm import path_planning as astar  # noqa: E402
from swarm import simulator as sim  # noqa: E402


def test_astar_goes_around_wall():
    g = np.zeros((20, 20), bool)
    g[0:15, 10] = True  # 아래쪽만 열린 벽
    path = astar.astar(g, (0, 0), (0, 19))
    assert path and path[-1] == (0, 19)
    assert all(not g[c] for c in path)
    assert any(c[0] >= 15 for c in path)  # 벽 끝을 돌아간다
    short = astar.shortcut(g, path)
    assert len(short) < len(path)
    assert all(astar.line_free(g, a, b) for a, b in zip(short, short[1:]))


def test_astar_blocked():
    g = np.zeros((10, 10), bool)
    g[:, 5] = True
    assert astar.astar(g, (0, 0), (0, 9)) is None


def test_plan_inflation_keeps_clearance():
    res = 0.1
    g = np.zeros((51, 51), bool)
    g[20:31, 20:31] = True  # 1 m 블록
    wp = astar.plan(g, res, (0.5, 2.5), (4.5, 2.5), inflate_m=0.5)
    assert wp is not None
    for (x0, y0), (x1, y1) in zip(wp, wp[1:]):
        for t in np.linspace(0, 1, 50):
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            dx = max(2.0 - x, 0, x - 3.0)
            dy = max(2.0 - y, 0, y - 3.0)
            assert np.hypot(dx, dy) >= 0.4  # 부풀림 0.5 m, 격자 오차 0.1 m


def test_cbf_doc_example():
    # docs/소프트웨어/01 2.2 계산 예: 원하는 (1, 0) -> 안전 (-0.201, 0)
    v, st = avoid.cbf_qp([1.0, 0.0], [0.0, 0.0], [([1.2, 0.0], [-0.5, 0.0])], D=0.85, gamma=1.0, vmax=2.0)
    assert st == avoid.OK
    assert abs(v[0] - (-0.2010)) < 1e-3 and abs(v[1]) < 1e-9


def test_cbf_passes_when_safe():
    v, st = avoid.cbf_qp([0.5, 0.0], [0, 0], [([5.0, 0.0], [0.0, 0.0])], D=0.5)
    assert st == avoid.OK and np.allclose(v, [0.5, 0.0])


def test_cbf_infeasible_stops():
    # 상자 제약(vmax 0.1)과 충돌하는 큰 회피 요구 -> 해 없음, 정지
    v, st = avoid.cbf_qp([0, 0], [0, 0], [([0.1, 0], [-1.0, 0]), ([-0.1, 0], [1.0, 0])], D=1.0, vmax=0.1)
    assert st == avoid.INFEASIBLE and np.allclose(v, 0)


def test_qp_matches_bruteforce():
    rng = np.random.default_rng(1)
    for _ in range(30):
        A = rng.normal(size=(5, 2))
        b = rng.normal(size=5) - 1.0
        vd = rng.normal(size=2) * 2
        v, st = avoid.solve_qp(vd, A, b)
        grid = np.stack(np.meshgrid(np.linspace(-6, 6, 241), np.linspace(-6, 6, 241)), -1).reshape(-1, 2)
        feas = grid[np.all(grid @ A.T >= b, axis=1)]
        if st == avoid.OK and len(feas):
            assert np.sum((v - vd) ** 2) <= np.min(np.sum((feas - vd) ** 2, axis=1)) + 1e-6


def test_consensus_zero_when_in_formation():
    offs = [np.array([1.0, 0]), np.array([-1.0, 0])]
    c = np.array([3.0, 3.0])
    u = consensus.formation_vel(c + offs[0], offs[0], [(c + offs[1], offs[1])], c_ref=c)
    assert np.allclose(u, 0)


def test_trigger():
    tr = trigger.EventTrigger(delta=0.1, max_gap=1.0)
    assert tr.should_send([0, 0], [1, 0], 0.0)
    tr.sent([0, 0], [1, 0], 0.0)
    assert not tr.should_send([0.5, 0.05], [1, 0], 0.5)  # 예측 (0.5, 0)과 0.05 차이
    assert tr.should_send([0.5, 0.2], [1, 0], 0.5)
    assert tr.should_send([1.0, 0.0], [1, 0], 1.0)  # 최소 1 Hz
    assert abs(trigger.effective_distance(0.4, 0.1) - 0.6) < 1e-12


def test_sim_swap_cbf_no_collision():
    r = sim.run("swap", "cbf", t_end=30)
    assert r["collisions"] == 0 and r["min_sep"] >= 0.35 and r["arrive_t"] is not None


def test_sim_formation_reaches_goal():
    r = sim.run("formation", "cbf", t_end=60)
    assert r["collisions"] == 0 and r["arrive_t"] is not None


def test_event_trigger_sends_less():
    a = sim.run("swap", "cbf", comm="period", t_end=30)
    b = sim.run("swap", "cbf", comm="event", delta=0.1, t_end=30)
    assert b["msgs_per_s"] < a["msgs_per_s"] and b["collisions"] == 0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
