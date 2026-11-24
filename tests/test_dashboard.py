"""대시보드(가상 모드) 점검. 실행: python -m pytest -q tests/test_dashboard.py"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gcs.dashboard.server import app  # noqa: E402
from gcs.dashboard.server import formations  # noqa: E402
from gcs.dashboard.server import sim  # noqa: E402


def test_slots_shapes():
    for name in formations.FORMATIONS:
        s = formations.slots(name)
        assert s.shape == (6, 2)
        assert np.allclose(s.mean(axis=0), 0, atol=1e-9)
        d = np.linalg.norm(s[:, None] - s[None], axis=-1) + np.eye(6) * 9
        assert d.min() >= 1.0 - 1e-6
    try:
        formations.slots("hexagon")
        assert False
    except ValueError:
        pass


def test_assign_keeps_close_slots():
    cur = formations.HOME.copy()
    tgt = formations.HOME[::-1].copy()  # 같은 자리 집합, 순서만 뒤집음
    out = formations.assign(cur, tgt)
    assert np.allclose(out, cur)  # 각자 제자리를 받는다
    assert sorted(map(tuple, out)) == sorted(map(tuple, tgt))


def _crossings(a, b):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    n = len(a)
    return sum(side(a[i], b[i], a[j]) * side(a[i], b[i], b[j]) < 0 and side(a[j], b[j], a[i]) * side(a[j], b[j], b[i]) < 0
               for i in range(n) for j in range(i + 1, n))


def test_assign_paths_never_cross():
    # 탐욕 배정은 이 전환들에서 직선 경로 3개를 교차시켰다. 총거리 최소 배정은 0이어야 한다.
    for a in formations.FORMATIONS + ("home",):
        cur = formations.HOME if a == "home" else formations.slots(a)
        for b in formations.FORMATIONS:
            out = formations.assign(cur, formations.slots(b))
            assert _crossings(cur, out) == 0, (a, b)


def _min_sep(s, seconds):
    m = 9.0
    for _ in range(int(seconds / s.dt)):
        s.step()
        d = np.linalg.norm(s.p[:, None] - s.p[None], axis=-1) + np.eye(s.n) * 9
        m = min(m, d.min())
    return m


def _sorted_rows(a):
    a = np.round(np.asarray(a, float), 9)
    return a[np.lexsort(a.T)]


def test_sim_takeoff_formations_converge():
    s = sim.SimSwarm()
    s.start()
    s.run_for(4)
    assert all(ph == "hover" for ph in s.phase) and np.allclose(s.z, s.hover_z)
    for name in ["line", "triangle", "circle"]:
        s.set_formation(name)
        assert _min_sep(s, 20) >= 0.3
        assert np.linalg.norm(s.p - s.slot, axis=1).max() < 0.2
        assert np.allclose(_sorted_rows(s.slot), _sorted_rows(formations.slots(name)))


def test_sim_stop_lands_everyone():
    s = sim.SimSwarm()
    s.start()
    s.run_for(1)  # 이륙 중
    s.stop()
    s.run_for(5)
    assert all(ph == "ground" for ph in s.phase) and np.allclose(s.z, 0)


def test_sim_formation_on_ground_applies_at_start():
    s = sim.SimSwarm()
    s.set_formation("circle")
    s.run_for(2)
    assert np.allclose(s.z, 0)
    s.start()
    s.run_for(20)
    assert np.linalg.norm(s.p - s.slot, axis=1).max() < 0.2


def test_sim_retarget_midway_and_home():
    s = sim.SimSwarm()
    s.start()
    s.set_formation("line")
    s.run_for(1)
    s.set_formation("circle")  # 이동 중 바꿈
    s.start()  # 떠 있을 때 시작 재입력: 무시
    assert _min_sep(s, 20) >= 0.3
    assert np.linalg.norm(s.p - s.slot, axis=1).max() < 0.2
    s.home()
    s.run_for(20)
    assert np.linalg.norm(s.p - formations.HOME, axis=1).max() < 0.2
    st = s.state()
    assert st["formation"] is None and all(d["phase"] == "hover" for d in st["drones"])


def test_sim_unknown_formation():
    try:
        sim.SimSwarm().set_formation("hexagon")
        assert False
    except ValueError:
        pass


def test_handle_command():
    s = sim.SimSwarm()
    assert app.handle_command(s, '{"cmd": "start"}') is None and s.phase[0] == "takeoff"
    assert app.handle_command(s, '{"cmd": "formation", "name": "circle"}') is None
    assert s.formation == "circle"
    for bad in ['nope', '{"cmd": "fly"}', '{"cmd": "formation", "name": "hexagon"}', '{"cmd": "formation"}', '[]']:
        assert isinstance(app.handle_command(s, bad), str)


def _next_with(ws, key, ok=lambda m: True, tries=200):
    for _ in range(tries):
        m = ws.receive_json()
        if key in m and ok(m):
            return m
    raise AssertionError(key)


def test_ws_state_two_clients_and_reconnect():
    from fastapi.testclient import TestClient
    with TestClient(app.create_app()) as client:
        assert client.get("/").status_code == 200
        with client.websocket_connect("/ws") as a, client.websocket_connect("/ws") as b:
            for ws in (a, b):
                st = ws.receive_json()
                assert st["mode"] == "virtual" and len(st["drones"]) == 6
            a.send_text("garbage")
            assert "error" in _next_with(a, "error")
        with client.websocket_connect("/ws") as c:  # 앞 연결이 닫힌 뒤 다시 연결
            c.send_text('{"cmd": "start"}')
            m = _next_with(c, "drones", lambda m: m["drones"][0]["phase"] != "ground")
            assert all(d["phase"] != "ground" for d in m["drones"])


def test_web_offline_and_vendored():
    web = Path(__file__).resolve().parents[1] / "gcs" / "dashboard" / "web"
    for f in ["index.html", "main.js", "style.css", "vendor/three.module.js",
              "vendor/addons/controls/OrbitControls.js", "vendor/addons/renderers/CSS2DRenderer.js"]:
        assert (web / f).is_file(), f
    for f in ["index.html", "main.js", "style.css"]:
        text = (web / f).read_text(encoding="utf-8")
        assert "http://" not in text and "https://" not in text, f


def _fly_to(s, x, y, seconds=40, check=None):
    """목표까지 날리며 드론끼리, 드론과 기둥 사이 최소 거리를 잰다."""
    s.goto(x, y)
    near_drone, near_obst = 9.0, 9.0
    for _ in range(int(seconds / s.dt)):
        s.step()
        if check:
            check(s)
        d = np.linalg.norm(s.p[:, None] - s.p[None], axis=-1) + np.eye(s.n) * 9
        near_drone = min(near_drone, d.min())
        for o in s.obstacles:
            near_obst = min(near_obst, (np.linalg.norm(s.p - (o["x"], o["y"]), axis=1) - o["r"]).min())
        if s.goal is None:
            break
    return near_drone, near_obst


def _airborne():
    s = sim.SimSwarm()
    s.start()
    s.run_for(3)
    return s


def test_goto_goes_around_obstacles():
    s = _airborne()
    s.add_obstacle(2.5, 0.0)  # 시작 위치와 목표 사이를 막는다
    near_drone, near_obst = _fly_to(s, 5.5, 0.0)
    assert s.goal is None and not s.path  # 도착
    assert np.linalg.norm(s.p.mean(axis=0) - (5.5, 0.0)) < 0.3
    assert near_drone >= 0.3 and near_obst >= 0.3


def test_obstacle_on_route_replans():
    s = _airborne()
    s.goto(5.5, 0.0)
    before = [tuple(w) for w in s.path]
    s.add_obstacle(3.0, 0.0)
    assert [tuple(w) for w in s.path] != before
    near_drone, near_obst = _fly_to(s, 5.5, 0.0)
    assert s.goal is None and near_obst >= 0.3


def test_goto_rejections():
    s = sim.SimSwarm()
    for call in [lambda: s.goto(3, 3)]:  # 지상: 먼저 시작
        try:
            call()
            assert False
        except ValueError as e:
            assert "시작" in str(e)
    s = _airborne()
    fx = next(o for o in s.obstacles if o["fixed"])
    assert isinstance(s.goto(fx["x"], fx["y"]), str)  # 기둥 위: 옆으로 옮기고 알림
    assert np.hypot(s.goal[0] - fx["x"], s.goal[1] - fx["y"]) > fx["r"]
    for call in [lambda: s.goto(20, 0),                     # 방 밖
                 lambda: s.add_obstacle(0, 0),              # 드론 위
                 lambda: s.add_obstacle(fx["x"], fx["y"]),  # 다른 기둥 위
                 lambda: s.add_obstacle(9, 9)]:             # 방 밖
        try:
            call()
            assert False
        except ValueError:
            pass


def test_clear_keeps_fixed_and_home_returns():
    s = _airborne()
    n_fixed = len(s.obstacles)
    s.add_obstacle(2.5, 0.0)
    s.clear_obstacles()
    assert len(s.obstacles) == n_fixed and all(o["fixed"] for o in s.obstacles)
    _fly_to(s, 5.5, 0.0)
    s.home()  # A* 경로로 시작 위치까지 간 뒤 원래 자리로 선다
    assert s.goal == (0.0, 0.0)
    for _ in range(int(40 / s.dt)):
        s.step()
        if s.goal is None:
            break
    s.run_for(10)
    assert np.linalg.norm(s.p - formations.HOME, axis=1).max() < 0.2


def test_state_has_scene():
    s = _airborne()
    s.goto(5.5, 0.0)
    st = s.state()
    assert st["room"] == 15.0 and st["goal"] == [5.5, 0.0] and len(st["path"]) >= 2
    assert all({"x", "y", "r", "fixed"} <= o.keys() for o in st["obstacles"])


def test_handle_scene_commands():
    s = _airborne()
    assert app.handle_command(s, '{"cmd": "obstacle", "x": 2.5, "y": 0}') is None
    assert app.handle_command(s, '{"cmd": "goal", "x": 5.5, "y": 0}') is None
    assert app.handle_command(s, '{"cmd": "clear_obstacles"}') is None
    for bad in ['{"cmd": "goal", "x": "a", "y": 0}', '{"cmd": "goal"}', '{"cmd": "obstacle", "x": 99, "y": 0}',
                '{"cmd": "goal", "x": true, "y": 0}']:
        assert isinstance(app.handle_command(s, bad), str)


def test_no_false_deadlock_on_open_floor():
    # 장애물 없는 직선 이동에서 교착 완화(60도 회전)가 켜지면 안 된다.
    # 버그: 제어 직전에 u를 0으로 지운 뒤 '직전 속도'로 넘겨 1초마다 교착으로 판정했다.
    s = _airborne()
    s.goto(4.0, 0.0)
    _fly_to(s, 4.0, 0.0)
    assert sum(d.count for d in s.dl) == 0


def _wall_overrun(s):
    return np.abs(s.p).max() - sim.ROOM / 2


def test_goal_near_wall_snaps_inside():
    # 일렬(반경 2.5 m)로 벽 가까이를 찍으면 거부하지 말고 갈 수 있는 가장 가까운 점으로 옮긴다.
    s = _airborne()
    s.set_formation("line")
    s.run_for(10)
    note = s.goto(7.0, 0.0)
    assert isinstance(note, str) and s.goal is not None and s.goal[0] < 7.0
    worst = [0.0]
    _fly_to_goal(s, check=lambda s: worst.__setitem__(0, max(worst[0], _wall_overrun(s))))
    assert s.goal is None and worst[0] <= 0.0


def _fly_to_goal(s, seconds=40, check=None):
    for _ in range(int(seconds / s.dt)):
        s.step()
        if check:
            check(s)
        if s.goal is None:
            break


def test_formation_change_near_wall_moves_inward():
    s = _airborne()
    _fly_to(s, 5.5, 0.0)  # 기본 대형으로 벽 가까이
    s.set_formation("line")  # 일렬은 이 자리에서 벽을 넘는다
    worst = [0.0]
    for _ in range(int(25 / s.dt)):
        s.step()
        worst[0] = max(worst[0], _wall_overrun(s))
    assert worst[0] <= 0.0
    assert np.linalg.norm(s.p - (s.center_ref + s.slot), axis=1).max() < 0.2


def test_obstacle_inside_hover_footprint_moves_formation():
    s = _airborne()
    s.set_formation("circle")
    s.run_for(10)
    s.add_obstacle(1.6, 0.9)  # 드론과는 떨어졌지만 편대 반경 + D 안쪽이다
    s.run_for(25)
    assert np.linalg.norm(s.p - (s.center_ref + s.slot), axis=1).max() < 0.2


def test_formation_that_cannot_fit_is_rejected():
    # 일렬(폭 5 m)이 들어갈 공간이 없게 기둥을 세우면 편대 변경을 거부하고 지금 대형을 유지한다.
    s = _airborne()
    for x, y in [(-2.2, 2.6), (2.2, 2.6), (-2.2, -2.6), (2.2, -2.6), (0.0, 3.4), (0.0, -3.4),
                 (-5.0, 0.0), (5.0, 0.0), (-5.4, 4.6), (5.4, 4.6), (-5.4, -4.6), (5.4, -4.6)]:
        try:
            s.add_obstacle(x, y)
        except ValueError:
            pass
    before = s.slot.copy()
    try:
        s.set_formation("line")
        assert False
    except ValueError as e:
        assert "공간" in str(e)
    assert np.allclose(s.slot, before) and s.formation is None


def test_start_with_formation_that_hits_pillar_moves_over():
    # 지상에서 일렬을 골라 두고 기둥 옆에서 이륙하면 편대가 들어가는 곳으로 옮겨 선다.
    s = sim.SimSwarm()
    s.add_obstacle(3.0, 0.3)  # 일렬 오른쪽 끝 자리와 겹친다
    s.set_formation("line")
    s.start()
    _fly_to_goal(s, 30)
    s.run_for(10)
    assert np.linalg.norm(s.p - (s.center_ref + s.slot), axis=1).max() < 0.2


def test_random_scenarios_arrive_safely():
    # 무작위 기둥, 목표, 이동 중 편대 변경. 모두 도착하고, 드론끼리, 드론과 기둥 0.3 m, 벽 안쪽을 지킨다.
    for seed in range(6):
        rng = np.random.default_rng(seed)
        s = _airborne()
        for _ in range(rng.integers(0, 6)):
            try:
                s.add_obstacle(*np.round(rng.uniform(-6, 6, 2), 1))
            except ValueError:
                pass
        for _ in range(3):
            try:
                s.goto(*np.round(rng.uniform(-6.5, 6.5, 2), 1))
            except ValueError:
                continue
            switch = rng.uniform(1, 6)
            t0 = s.t
            while s.goal is not None:
                assert s.t - t0 < 60, (seed, s.goal)
                if switch and s.t - t0 > switch:
                    try:
                        s.set_formation(str(rng.choice(formations.FORMATIONS)))
                    except ValueError:
                        pass
                    switch = 0
                s.step()
                d = np.linalg.norm(s.p[:, None] - s.p[None], axis=-1) + np.eye(s.n) * 9
                assert d.min() >= 0.3, seed
                for o in s.obstacles:
                    assert (np.linalg.norm(s.p - (o["x"], o["y"]), axis=1) - o["r"]).min() >= 0.3, seed
                assert np.abs(s.p).max() <= sim.ROOM / 2, seed
