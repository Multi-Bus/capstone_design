"""swarm/c (C99 포팅)가 swarm/*.py 와 같은 결과를 내는지 비교한다.
double 로 빌드한 공유 라이브러리를 ctypes 로 불러 무작위 입력에 대해 비교한다. gcc 가 없으면 건너뛴다.
실행: python tests/test_swarm_c.py  (또는 python -m pytest -q tests/test_swarm_c.py)
"""
import ctypes as C
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm import collision_avoidance as avoid  # noqa: E402
from swarm import event_trigger as trigger  # noqa: E402
from swarm import formation_control as consensus  # noqa: E402
from swarm import path_planning as astar  # noqa: E402

CELLS = 22801


class V2(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double)]


class Nbr(C.Structure):
    _fields_ = [("p", V2), ("v", V2), ("d", C.c_double)]


class FNbr(C.Structure):
    _fields_ = [("p", V2), ("d", V2)]


class Cell(C.Structure):
    _fields_ = [("y", C.c_int16), ("x", C.c_int16)]


class Dead(C.Structure):
    _fields_ = [("hold", C.c_double), ("escape", C.c_double), ("v_min", C.c_double), ("far", C.c_double),
                ("R", C.c_double * 4), ("stall", C.c_double), ("until", C.c_double), ("count", C.c_int)]


class Trig(C.Structure):
    _fields_ = [("delta", C.c_double), ("max_gap", C.c_double), ("has_last", C.c_int),
                ("p", V2), ("v", V2), ("t", C.c_double)]


def load():
    gcc = shutil.which("gcc")
    if gcc is None:
        return None
    out = Path(tempfile.mkdtemp()) / "swarm_c.dll"
    src = [str(p) for p in (ROOT / "swarm" / "c" / "src").glob("*.c")]
    subprocess.run([gcc, "-std=c99", "-O2", "-shared", "-DSWARM_DOUBLE", "-I", str(ROOT / "swarm" / "c" / "include"),
                    "-o", str(out), *src, "-lm"], check=True)
    lib = C.CDLL(str(out))
    lib.swarm_solve_qp.argtypes = [V2, C.POINTER(V2), C.POINTER(C.c_double), C.c_int, C.POINTER(V2)]
    lib.swarm_cbf_qp.argtypes = [V2, V2, C.POINTER(Nbr), C.c_int, C.c_double, C.c_double, C.c_double, C.POINTER(V2)]
    lib.swarm_apf.argtypes = [V2, V2, C.POINTER(Nbr), C.c_int, C.c_double, C.c_double, C.c_double]
    lib.swarm_apf.restype = V2
    lib.swarm_deadlock_init.argtypes = [C.POINTER(Dead)] + [C.c_double] * 5
    lib.swarm_deadlock_adjust.argtypes = [C.POINTER(Dead), V2, V2, C.c_double, C.c_double, C.c_double]
    lib.swarm_deadlock_adjust.restype = V2
    lib.swarm_formation_vel.argtypes = [V2, V2, C.POINTER(FNbr), C.c_int, V2, C.c_double, C.c_double, C.c_double]
    lib.swarm_formation_vel.restype = V2
    lib.swarm_trigger_init.argtypes = [C.POINTER(Trig), C.c_double, C.c_double]
    lib.swarm_trigger_should_send.argtypes = [C.POINTER(Trig), V2, V2, C.c_double, C.c_int]
    lib.swarm_trigger_sent.argtypes = [C.POINTER(Trig), V2, V2, C.c_double]
    lib.swarm_astar.argtypes = [C.POINTER(C.c_uint8), C.c_int, C.c_int, Cell, Cell, C.POINTER(Cell), C.c_int,
                                C.POINTER(C.c_int)]
    lib.swarm_inflate.argtypes = [C.POINTER(C.c_uint8), C.POINTER(C.c_uint8), C.c_int, C.c_int, C.c_int]
    lib.swarm_plan.argtypes = [C.POINTER(C.c_uint8), C.c_int, C.c_int, C.c_double, V2, V2, C.c_double,
                               C.POINTER(V2), C.c_int, C.POINTER(C.c_int)]
    return lib


LIB = load()
RNG = np.random.default_rng(7)


def v2(a):
    return V2(float(a[0]), float(a[1]))


def arr(v):
    return np.array([v.x, v.y])


def test_qp_and_cbf_match_python():
    for _ in range(400):
        n = int(RNG.integers(0, 9))
        p_i = RNG.uniform(-1, 1, 2)
        others = [(p_i + RNG.uniform(-1, 1, 2), RNG.uniform(-0.5, 0.5, 2)) for _ in range(n)]
        v_des = RNG.uniform(-1, 1, 2)
        D, gamma, vmax = 0.4, 1.0, 1.0
        want, st = avoid.cbf_qp(v_des, p_i, others, D, gamma, vmax)
        nb = (Nbr * max(n, 1))(*[Nbr(v2(o[0]), v2(o[1]), 0.0) for o in others])
        out = V2()
        rc = LIB.swarm_cbf_qp(v2(v_des), v2(p_i), nb, n, D, gamma, vmax, C.byref(out))
        assert rc == st
        assert np.allclose(arr(out), want, atol=1e-6), (arr(out), want)


def test_qp_per_neighbor_distance():
    p_i = np.array([0.0, 0.0])
    others = [(np.array([0.5, 0.0]), np.zeros(2), 0.8)]
    want, _ = avoid.cbf_qp(np.array([1.0, 0.0]), p_i, others, 0.4)
    nb = (Nbr * 1)(Nbr(V2(0.5, 0.0), V2(0, 0), 0.8))
    out = V2()
    LIB.swarm_cbf_qp(V2(1.0, 0.0), V2(0, 0), nb, 1, 0.4, 1.0, 1.0, C.byref(out))
    assert np.allclose(arr(out), want, atol=1e-6)


def test_apf_deadlock_formation_trigger_match():
    for _ in range(200):
        n = int(RNG.integers(0, 6))
        p_i = RNG.uniform(-1, 1, 2)
        others = [(p_i + RNG.uniform(-0.8, 0.8, 2), np.zeros(2), float(RNG.uniform(0, 0.3))) for _ in range(n)]
        v_des = RNG.uniform(-1, 1, 2)
        want = avoid.apf(v_des, p_i, others, 1.0)
        nb = (Nbr * max(n, 1))(*[Nbr(v2(o[0]), v2(o[1]), o[2]) for o in others])
        got = LIB.swarm_apf(v2(v_des), v2(p_i), nb, n, 1.0, 0.5, 1.0)
        assert np.allclose(arr(got), want, atol=1e-6)

        nbrs = [(RNG.uniform(-2, 2, 2), RNG.uniform(-1, 1, 2)) for _ in range(n)]
        d_i, c_ref = RNG.uniform(-1, 1, 2), RNG.uniform(-3, 3, 2)
        want = consensus.formation_vel(p_i, d_i, nbrs, c_ref)
        fn = (FNbr * max(n, 1))(*[FNbr(v2(a), v2(b)) for a, b in nbrs])
        got = LIB.swarm_formation_vel(v2(p_i), v2(d_i), fn, n, v2(c_ref), 0.5, 0.6, 1.0)
        assert np.allclose(arr(got), want, atol=1e-9)

    py = avoid.Deadlock()
    cd = Dead()
    LIB.swarm_deadlock_init(C.byref(cd), 1.0, 1.5, 60.0, 0.05, 0.5)
    for k in range(60):
        t, dt = k * 0.1, 0.1
        v_des = np.array([1.0, 0.2])
        u_prev = np.zeros(2) if k % 17 < 14 else np.array([0.5, 0.0])
        want = py.adjust(v_des, u_prev, 5.0, t, dt)
        got = LIB.swarm_deadlock_adjust(C.byref(cd), v2(v_des), v2(u_prev), 5.0, t, dt)
        assert np.allclose(arr(got), want, atol=1e-9), k
    assert cd.count == py.count

    et, ct = trigger.EventTrigger(0.1), Trig()
    LIB.swarm_trigger_init(C.byref(ct), 0.1, 1.0)
    p, v = np.zeros(2), np.array([1.0, 0.0])
    for k in range(80):
        t = k * 0.05
        p = p + v * 0.05 + RNG.normal(0, 0.03, 2)
        want = et.should_send(p, v, t)
        got = bool(LIB.swarm_trigger_should_send(C.byref(ct), v2(p), v2(v), t, 0))
        assert got == want, k
        if want:
            et.sent(p, v, t)
            LIB.swarm_trigger_sent(C.byref(ct), v2(p), v2(v), t)


def to_buf(g):
    return g.astype(np.uint8).ravel().ctypes.data_as(C.POINTER(C.c_uint8)), g


def path_cost(path):
    return sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:]))


def test_inflate_and_astar_match_python():
    for _ in range(40):
        h = w = int(RNG.integers(15, 60))
        g = RNG.random((h, w)) < 0.22
        g[0, 0] = g[h - 1, w - 1] = False
        r = int(RNG.integers(0, 3))
        want_g = astar.inflate(g, r) if r else g
        buf = np.ascontiguousarray(g.astype(np.uint8))
        out = np.zeros_like(buf)
        LIB.swarm_inflate(buf.ctypes.data_as(C.POINTER(C.c_uint8)), out.ctypes.data_as(C.POINTER(C.c_uint8)), h, w, r)
        assert np.array_equal(out.astype(bool), want_g)

        gg = want_g.copy()
        gg[0, 0] = gg[h - 1, w - 1] = False
        want = astar.astar(gg, (0, 0), (h - 1, w - 1))
        gb = np.ascontiguousarray(gg.astype(np.uint8))
        path = (Cell * CELLS)()
        n = C.c_int(0)
        rc = LIB.swarm_astar(gb.ctypes.data_as(C.POINTER(C.c_uint8)), h, w, Cell(0, 0), Cell(h - 1, w - 1), path, CELLS,
                             C.byref(n))
        if want is None:
            assert rc == 3  # SWARM_NO_PATH
            continue
        assert rc == 0
        got = [(path[i].y, path[i].x) for i in range(n.value)]
        assert got[0] == (0, 0) and got[-1] == (h - 1, w - 1)
        assert all(not gg[c] for c in got)
        # 같은 비용의 다른 경로는 허용 (동점 처리가 다르다)
        assert abs(path_cost(got) - path_cost(want)) < 1e-9


def test_plan_matches_python_room():
    res, size = 0.1, 15.0
    h = w = int(size / res) + 1
    yy, xx = np.mgrid[0:h, 0:w] * res
    g = (xx - 7.5) ** 2 + (yy - 7.5) ** 2 <= 1.0
    want = astar.plan(g, res, (2.0, 7.5), (13.0, 7.5), inflate_m=1.2)
    gb = np.ascontiguousarray(g.astype(np.uint8))
    out = (V2 * CELLS)()
    n = C.c_int(0)
    rc = LIB.swarm_plan(gb.ctypes.data_as(C.POINTER(C.c_uint8)), h, w, res, V2(2.0, 7.5), V2(13.0, 7.5), 1.2, out,
                        CELLS, C.byref(n))
    assert rc == 0
    got = [(out[i].x, out[i].y) for i in range(n.value)]
    assert np.allclose(got[0], want[0]) and np.allclose(got[-1], want[-1])
    assert abs(path_cost(got) - path_cost(want)) < 0.2  # 지름길 선택이 달라도 길이는 거의 같다
    gi = astar.inflate(g, int(math.ceil(1.2 / res)))
    cells = [(int(round(y / res)), int(round(x / res))) for x, y in got]
    assert all(astar.line_free(gi, a, b) for a, b in zip(cells, cells[1:]))


if __name__ == "__main__":
    if LIB is None:
        print("skip: gcc 없음")
    else:
        for name, fn in list(globals().items()):
            if name.startswith("test_"):
                fn()
                print("ok", name)
