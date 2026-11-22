# 편대 리모트 대시보드 (가상 모드) 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 노트북에서 FastAPI 서버를 켜고 브라우저를 열면, 파란 그리드 무대 위 3D 쿼드콥터 6대가 시작, 중지, 편대(일렬, 삼각, 원), 복귀 버튼대로 움직이는 가상 모드 대시보드를 만든다.

**Architecture:** `dashboard/server`가 한 단계씩 도는 가상 시뮬(`SimSwarm`)을 들고 있고, `swarm/`의 `formation_vel`, `cbf_qp`, `Deadlock`으로 평면 운동을, 별도 상승·하강으로 고도를 계산한다. FastAPI가 50 Hz로 시뮬을 진행시키고 20 Hz로 상태를 WebSocket에 뿌린다. `dashboard/web`은 레포에 복사한 three.js로 상태를 그리고 버튼 명령을 같은 WebSocket으로 보낸다.

**Tech Stack:** Python 3.12, numpy, FastAPI, uvicorn, httpx(테스트용 TestClient), pytest, three.js 0.170.0(ES 모듈, import map), 순수 HTML/CSS/JS(빌드 도구 없음).

**Spec:** `docs/superpowers/specs/2026-11-22-formation-dashboard-design.md`

## Global Constraints

- 첫 버전은 가상 모드만. BLE, MAVLink, `bleak`, `pymavlink` 코드와 import를 넣지 않는다.
- `swarm/simulator.py`와 `swarm/` 안 파일은 수정하지 않는다. import만 한다.
- 외부 URL(CDN, Google Fonts 등)을 쓰지 않는다. 인터넷 없이 동작해야 한다.
- 화면 색은 파란색 계열 한 색조. 그라데이션 없음. 예외는 서버 연결 점의 초록(연결)과 빨강(끊김)뿐.
- 드론 위에는 번호만 표시한다. 배터리를 표시하지 않는다.
- 레포 규칙: 커밋과 푸시는 사용자가 요청할 때만 한다. 커밋 메시지에 Claude 표기를 넣지 않는다.
- 작업 브랜치는 `origin/main`에서 새로 만든 `feat/dashboard`. 현재 브랜치(`docs/readme-swarm-uwb`, PR #4)에서 작업하지 않는다.
- 테스트 스타일은 `tests/test_swarm.py`와 같다: 맨 위에서 `sys.path`에 레포 루트를 넣고, 일반 `assert`를 쓰며, `python -m pytest -q`로 돈다.

## Review Focus

1. 버튼 연타와 동작 중 명령: 이륙 중 중지, 편대 이동 중 다른 편대 선택, 떠 있을 때 시작 재입력. 예외 없이 마지막 명령대로 수렴해야 한다. Task 2 테스트로 고정한다.
2. 잘못된 WebSocket 메시지: JSON이 아님, 모르는 `cmd`, 모르는 편대 이름. 서버는 죽지 않고 `{"error": ...}`만 돌려줘야 한다. Task 3 테스트로 고정한다.
3. 발표 중 브라우저 새로고침이나 탭 닫기: 서버 시뮬은 계속 돌고, 다시 연결한 화면이 현재 상태를 받아야 한다. 끊긴 소켓 때문에 전송 루프가 멈추면 안 된다. Task 3 테스트로 고정한다.
4. 화면 두 개(발표자 노트북과 프로젝터 창): 두 연결 모두 같은 상태를 받아야 한다. Task 3 테스트로 고정한다.
5. 인터넷 없는 발표장: `web/`의 어떤 파일도 `http://`나 `https://`로 자원을 불러오지 않아야 한다. Task 4 테스트로 고정한다.

---

### Task 0: 작업 브랜치

- [ ] **Step 1:** `git fetch origin` 후 `git switch -c feat/dashboard origin/main`. 설계 문서(`docs/superpowers/specs/2026-11-22-formation-dashboard-design.md`)와 이 계획 파일은 아직 추적되지 않은 파일이라 그대로 따라온다.
- [ ] **Step 2:** `pip install fastapi uvicorn httpx` 실행. 확인: `python -c "import fastapi, uvicorn, httpx"`가 오류 없이 끝난다.

### Task 1: 편대 형태와 자리 배정

**Files:**
- Create: `dashboard/__init__.py`(빈 파일), `dashboard/server/__init__.py`(빈 파일), `dashboard/server/formations.py`
- Test: `tests/test_dashboard.py`

**Interfaces:**
- Produces:
  - `FORMATIONS: tuple[str, ...] = ("line", "triangle", "circle")`
  - `HOME: np.ndarray` 형태 (6, 2). 시작 위치. 2행 3열, 간격 1.0 m, 중심 (0, 0): `[(-1,-0.5),(0,-0.5),(1,-0.5),(-1,0.5),(0,0.5),(1,0.5)]`
  - `slots(name: str, n: int = 6, spacing: float = 1.0) -> np.ndarray` 형태 (n, 2). 중심(평균)이 (0, 0). 모르는 이름이면 `ValueError`.
    - `line`: x축 위 일렬, 이웃 간격 `spacing`.
    - `triangle`: 정삼각 격자 1, 2, 3개 행(위에서 아래), 이웃 간격 `spacing`. n=6 전용, 다른 n이면 `ValueError`.
    - `circle`: 반지름 `spacing`, 각도 `i * 360/n`도.
  - `assign(current: np.ndarray, targets: np.ndarray) -> np.ndarray` 형태 (n, 2). `targets`의 행을 재배열해 `current[i]`가 갈 자리를 돌려준다. 탐욕 배정: 남은 (드론, 자리) 쌍 중 거리가 가장 짧은 쌍부터 확정.

- [ ] **Step 1: 실패하는 테스트 작성** (`tests/test_dashboard.py`, `tests/test_swarm.py`와 같은 머리 부분)

```python
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


def test_assign_greedy_keeps_close_slots():
    cur = formations.HOME.copy()
    tgt = formations.HOME[::-1].copy()  # 같은 자리 집합, 순서만 뒤집음
    out = formations.assign(cur, tgt)
    assert np.allclose(out, cur)  # 각자 제자리를 받는다
    assert sorted(map(tuple, out)) == sorted(map(tuple, tgt))
```

- [ ] **Step 2:** `python -m pytest -q tests/test_dashboard.py` 실행. 기대: `ModuleNotFoundError: No module named 'dashboard'`로 FAIL.
- [ ] **Step 3:** `dashboard/server/formations.py`에 위 인터페이스 구현.
- [ ] **Step 4:** 같은 명령 실행. 기대: 2 passed.
- [ ] **Step 5: 커밋(사용자 승인 후)** `git add dashboard/__init__.py dashboard/server tests/test_dashboard.py` 후 `git commit -m "feat(dashboard): 편대 형태와 자리 배정"`

### Task 2: 가상 시뮬 `SimSwarm`

**Files:**
- Create: `dashboard/server/sim.py`
- Test: `tests/test_dashboard.py`(추가)

**Interfaces:**
- Consumes: Task 1의 `FORMATIONS`, `HOME`, `slots`, `assign`. `swarm.formation_control.formation_vel(p_i, d_i, nbrs, c_ref, vmax=...)`, `swarm.collision_avoidance.cbf_qp(v_des, p_i, others, D, gamma, vmax) -> (u, status)`, `swarm.collision_avoidance.Deadlock().adjust(v_des, u_prev, dist_goal, t, dt)`.
- Produces:
  - `class SimSwarm(n=6, dt=0.02, ctrl_dt=0.1, tau=0.3, vmax=1.0, D=0.4, gamma=1.0, hover_z=1.0, climb=0.5)`
  - `start() -> None`: 지상(`ground`) 드론만 `takeoff`로. 이미 떠 있으면 아무 일 없음.
  - `stop() -> None`: 떠 있는 모든 드론을 `landing`으로.
  - `set_formation(name: str) -> None`: `name`은 `FORMATIONS` 중 하나, 아니면 `ValueError`. 현재 위치 기준 `assign`으로 자리를 다시 배정하고 `formation = name`. 지상이어도 자리는 바꿔 둔다(시작하면 그 형태로 선다).
  - `home() -> None`: 자리 = `HOME`(드론 i는 `HOME[i]`, 재배정 없음), `formation = None`.
  - `step() -> None`: `dt`만큼 진행.
  - `run_for(seconds: float) -> None`: `step()` 반복.
  - `state() -> dict`: `{"mode": "virtual", "formation": str | None, "t": float, "drones": [{"id": 1..n, "x": float, "y": float, "z": float, "phase": "ground"|"takeoff"|"hover"|"landing"}, ...]}`
  - 속성 `p: np.ndarray (n,2)`, `z: np.ndarray (n,)`, `slot: np.ndarray (n,2)`, `phase: list[str]` (테스트가 읽는다)
- 동작 규칙(구현자가 정하지 못하는 것):
  - 초기 상태: `p = HOME`, `z = 0`, 모두 `ground`, `slot = HOME`, `formation = None`.
  - 평면 제어는 `ctrl_dt`마다(`dt`의 5배), 떠 있는 드론(`takeoff`, `hover`)에만 적용: `v_des = formation_vel(p[i], slot[i], nbrs, c_ref=(0,0), vmax=vmax)` (`nbrs`는 떠 있는 다른 드론의 `(p[j], slot[j])`), `Deadlock.adjust`로 보정(`dist_goal = |slot[i] - p[i]|`), `cbf_qp`로 안전 속도(`others`는 떠 있는 다른 드론의 `(p[j], v[j], D)`). 이웃 정보는 참값을 쓴다(통신 모델 없음).
  - 지상과 `landing` 드론의 평면 명령은 0. 운동은 `simulator.py`와 같은 1차 지연 `v += (u - v)/tau*dt; p += v*dt`.
  - 고도: `takeoff`는 `climb` m/s로 올라가 `hover_z`에 닿으면 `hover`. `landing`은 `climb` m/s로 내려가 0에 닿으면 `ground`이고 평면 속도도 0.
  - `ponytail:` 주석으로 "이웃 참값 사용, 통신 지연과 잡음 없음. 발표용 화면이라 충분, 필요하면 simulator.py의 이벤트 트리거를 붙인다"를 남긴다.

- [ ] **Step 1: 실패하는 테스트 작성**

```python
def _min_sep(s, seconds):
    m = 9.0
    for _ in range(int(seconds / s.dt)):
        s.step()
        d = np.linalg.norm(s.p[:, None] - s.p[None], axis=-1) + np.eye(s.n) * 9
        m = min(m, d.min())
    return m


def test_sim_takeoff_formations_converge():
    s = sim.SimSwarm()
    s.start()
    s.run_for(4)
    assert all(ph == "hover" for ph in s.phase) and np.allclose(s.z, s.hover_z)
    for name in ["line", "triangle", "circle"]:
        s.set_formation(name)
        assert _min_sep(s, 20) >= 0.3
        assert np.linalg.norm(s.p - s.slot, axis=1).max() < 0.2
        assert np.allclose(s.slot[np.lexsort(s.slot.T)], formations.slots(name)[np.lexsort(formations.slots(name).T)])


def test_sim_stop_lands_everyone():
    s = sim.SimSwarm()
    s.start()
    s.run_for(1)          # 이륙 중
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
    s.set_formation("circle")   # 이동 중 바꿈
    s.start()                   # 떠 있을 때 시작 재입력: 무시
    assert _min_sep(s, 20) >= 0.3
    assert np.linalg.norm(s.p - s.slot, axis=1).max() < 0.2
    s.home()
    s.run_for(20)
    assert np.linalg.norm(s.p - formations.HOME, axis=1).max() < 0.2
    assert s.state()["formation"] is None and all(d["phase"] == "hover" for d in s.state()["drones"])


def test_sim_unknown_formation():
    try:
        sim.SimSwarm().set_formation("hexagon")
        assert False
    except ValueError:
        pass
```

- [ ] **Step 2:** `python -m pytest -q tests/test_dashboard.py` 실행. 기대: `sim` import 실패로 새 테스트 FAIL.
- [ ] **Step 3:** `dashboard/server/sim.py`에 `SimSwarm` 구현.
- [ ] **Step 4:** 같은 명령 실행. 기대: 7 passed. 수렴 테스트가 20초 안에 안 들어오면 `formation_vel`의 `k_g`나 `vmax`가 아니라 먼저 교착(`Deadlock.count`)과 `cbf_qp` 상태를 확인하고, 원인을 보고한다. 테스트 기준(20초, 0.2 m, 0.3 m)은 설계 문서 값이라 바꾸지 않는다.
- [ ] **Step 5:** `python -m pytest -q tests`로 기존 `tests/test_swarm.py`도 통과하는지 확인.
- [ ] **Step 6: 커밋(사용자 승인 후)** `git commit -m "feat(dashboard): 가상 시뮬 SimSwarm"`

### Task 3: FastAPI 서버와 WebSocket

**Files:**
- Create: `dashboard/server/app.py`, `dashboard/__main__.py`, `dashboard/requirements.txt`
- Test: `tests/test_dashboard.py`(추가)

**Interfaces:**
- Consumes: Task 2의 `SimSwarm`.
- Produces:
  - `handle_command(sim: SimSwarm, msg: str) -> str | None`: 오류 문자열 또는 `None`. 메시지 형식: `{"cmd": "start"}`, `{"cmd": "stop"}`, `{"cmd": "home"}`, `{"cmd": "formation", "name": "line"|"triangle"|"circle"}`. JSON이 아니거나, 모르는 `cmd`이거나, 모르는 이름이면 예외를 던지지 않고 오류 문자열을 돌려준다.
  - `create_app(sim: SimSwarm | None = None, sim_hz: float = 50, send_hz: float = 20) -> FastAPI`
    - lifespan에서 백그라운드 작업 하나: `1/sim_hz`초마다 `sim.step()`(실시간 진행), `1/send_hz`초마다 연결된 모든 WebSocket에 `sim.state()`를 JSON으로 전송. 전송에 실패한 소켓은 목록에서 빼고 루프는 계속 돈다.
    - `GET /`와 정적 파일: `dashboard/web`을 `StaticFiles(directory=..., html=True)`로 `/`에 마운트. 라우트 `/ws`를 먼저 등록한다.
    - `WS /ws`: 연결 즉시 `sim.state()` 한 번 전송, 이후 받은 텍스트마다 `handle_command`. 오류면 `{"error": "..."}` 전송.
  - `python -m dashboard [--port 8000]`: uvicorn으로 `127.0.0.1`에서 실행하고, 서버가 뜬 뒤 `webbrowser.open("http://127.0.0.1:<port>/")`.
  - `dashboard/requirements.txt`: `numpy`, `fastapi`, `uvicorn`, `httpx`(테스트 전용이라고 주석).

- [ ] **Step 1: 실패하는 테스트 작성**

```python
def test_handle_command():
    s = sim.SimSwarm()
    assert app.handle_command(s, '{"cmd": "start"}') is None and s.phase[0] == "takeoff"
    assert app.handle_command(s, '{"cmd": "formation", "name": "circle"}') is None
    assert s.formation == "circle"
    for bad in ['nope', '{"cmd": "fly"}', '{"cmd": "formation", "name": "hexagon"}', '{"cmd": "formation"}', '[]']:
        assert isinstance(app.handle_command(s, bad), str)


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
        with client.websocket_connect("/ws") as c:   # 앞 연결이 닫힌 뒤 다시 연결
            c.send_text('{"cmd": "start"}')
            assert any(d["phase"] != "ground" for d in _next_with(c, "drones", lambda m: m["drones"][0]["phase"] != "ground")["drones"])


def _next_with(ws, key, ok=lambda m: True, tries=200):
    for _ in range(tries):
        m = ws.receive_json()
        if key in m and ok(m):
            return m
    raise AssertionError(key)
```

`test_ws_state_two_clients_and_reconnect`는 `dashboard/web/index.html`이 있어야 `GET /`가 200이다. 이 Task에서는 `index.html`을 한 줄짜리 자리 표시 파일로 만들고 Task 4에서 덮어쓴다.

- [ ] **Step 2:** `python -m pytest -q tests/test_dashboard.py` 실행. 기대: `app` import 실패로 새 테스트 FAIL.
- [ ] **Step 3:** `app.py`, `__main__.py`, `requirements.txt`, 자리 표시 `web/index.html` 작성.
- [ ] **Step 4:** 같은 명령 실행. 기대: 9 passed.
- [ ] **Step 5:** `python -m dashboard --port 8001`을 띄우고 브라우저가 열리는지, 종료(Ctrl+C)가 깔끔한지 확인.
- [ ] **Step 6: 커밋(사용자 승인 후)** `git commit -m "feat(dashboard): FastAPI 서버와 WebSocket"`

### Task 4: 3D 화면

**Files:**
- Create: `dashboard/web/index.html`, `dashboard/web/style.css`, `dashboard/web/main.js`, `dashboard/web/vendor/three.module.js`, `dashboard/web/vendor/addons/controls/OrbitControls.js`, `dashboard/web/vendor/addons/renderers/CSS2DRenderer.js`, `dashboard/web/vendor/LICENSE`
- Test: `tests/test_dashboard.py`(추가)

**Interfaces:**
- Consumes: Task 3의 `/ws` 상태 JSON과 명령 JSON.
- Produces: 화면. 다른 Task가 쓰는 인터페이스 없음.

- [ ] **Step 1: three.js 복사.** 임시 폴더(스크래치)에서 `npm pack three@0.170.0` 후 압축을 풀고, `package/build/three.module.js`, `package/examples/jsm/controls/OrbitControls.js`, `package/examples/jsm/renderers/CSS2DRenderer.js`, `package/LICENSE`를 위 경로로 복사한다. 0.170.0은 `three.module.js` 한 파일짜리라 `three.core.js`가 필요 없다.
- [ ] **Step 2: 실패하는 테스트 작성**

```python
def test_web_offline_and_vendored():
    web = Path(__file__).resolve().parents[1] / "dashboard" / "web"
    for f in ["index.html", "main.js", "style.css", "vendor/three.module.js",
              "vendor/addons/controls/OrbitControls.js", "vendor/addons/renderers/CSS2DRenderer.js"]:
        assert (web / f).is_file(), f
    for f in ["index.html", "main.js", "style.css"]:
        text = (web / f).read_text(encoding="utf-8")
        assert "http://" not in text and "https://" not in text, f
```

- [ ] **Step 3:** 실행해서 FAIL 확인(`main.js` 없음).
- [ ] **Step 4: `index.html`, `style.css`, `main.js` 작성.** 정해진 것:
  - import map: `"three": "./vendor/three.module.js"`, `"three/addons/": "./vendor/addons/"`. `main.js`는 `type="module"`.
  - 색 토큰(`style.css`의 `:root`): 바탕 `#050b1f`, 그리드 선 `#1e6bff`, 밝은 선과 글자 `#7fb2ff`, 연결 `#2ecc71`, 끊김 `#e74c3c`. 장면 배경색도 `#050b1f`, 안개와 그라데이션 없음.
  - 무대: 바닥 `GridHelper`(20 m, 40칸)와 뒷벽 `GridHelper`(바닥 뒤 끝에 세로로 세움). 둘 다 그리드 선 색.
  - 좌표 변환: three `(x, y, z) = (sim.x, sim.z, -sim.y)`.
  - 드론 `makeDrone(id)`: 몸체 상자, X자 팔 2개(대각선 상자), 프로펠러 원판 4개. 크기는 팔 끝 사이 약 0.35 m. 위에 `CSS2DObject` 번호 라벨(`1`~`6`). `phase`가 `ground`가 아니면 프로펠러가 돈다. 위치는 받은 상태로 매 프레임 보간(`lerp`).
  - 카메라: 시작 시 편대 중심을 비스듬히 위에서 약 6 m 거리로 본다. `OrbitControls`의 target이 편대 중심을 부드럽게 따라간다. 마우스로 돌려볼 수 있다.
  - 버튼(HTML, 장면 위에 고정): 왼쪽 세로 `일렬`, `삼각`, `원`(`{"cmd":"formation","name":"line"|"triangle"|"circle"}`), 아래 중앙 `시작`, `중지`, `복귀`. 현재 편대 버튼은 강조 표시.
  - 위쪽 막대: 연결 점, `가상 모드`, 현재 편대 이름(`일렬`/`삼각`/`원`, 없으면 `대기`).
  - WebSocket: `ws://${location.host}/ws`. 끊기면 화면 가운데에 `서버 연결 끊김` 표시, 1초마다 다시 연결, 연결되면 표시를 지운다.
  - 화면 크기가 바뀌면 렌더러와 카메라 비율을 맞춘다.
- [ ] **Step 5:** `python -m pytest -q tests` 실행. 기대: 전부 통과.
- [ ] **Step 6: 눈으로 확인.** `python -m dashboard`를 띄우고 브라우저 창에서: 그리드 무대와 드론 6대 보임 → `시작`으로 이륙, 프로펠러 회전 → `일렬`, `삼각`, `원` 각각 형태가 됨 → `복귀`로 시작 위치 위로 → `중지`로 착륙 → 서버를 끄면 `서버 연결 끊김`, 다시 켜면 사라짐 → 개발자 도구 Network에서 외부 요청 없음. 스크린샷 하나를 사용자에게 보낸다.
- [ ] **Step 7: 커밋(사용자 승인 후)** `git commit -m "feat(dashboard): 3D 화면"`

### Task 5: 실행 안내

**Files:**
- Create: `dashboard/README.md`
- Modify: `README.md`(레포 구조 블록, 문서 표)

- [ ] **Step 1:** `dashboard/README.md`에 설치(`pip install -r dashboard/requirements.txt`), 실행(`python -m dashboard`), 테스트(`python -m pytest -q tests/test_dashboard.py`), 버튼 설명, "첫 버전은 가상 모드만. 실기체 모드는 설계 문서의 다음 단계 조건이 갖춰지면" 한 줄, 설계 문서 링크를 적는다. 명령은 `powershell` 코드 블록 하나씩.
- [ ] **Step 2:** 루트 `README.md` 레포 구조에 `dashboard/        발표용 편대 리모트 대시보드(가상 모드): FastAPI 서버 + three.js 화면` 한 줄을 `uwb/` 줄 아래에 넣고, 문서 표의 `swarm`, `uwb` 행에 `[dashboard](dashboard/README.md)`를 더한다. PR #4가 아직 merge되지 않았다면 그 변경과 충돌하지 않게 PR #4 merge 뒤 `origin/main`을 다시 받아 넣는다.
- [ ] **Step 3: 커밋(사용자 승인 후)** `git commit -m "docs: dashboard 실행 안내와 README 반영"`
