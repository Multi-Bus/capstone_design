"""2D 다기체 시뮬 (E1, E2, E3, E6 시뮬 값).

모델: 속도 명령 u에 1차 지연 tau로 따라가는 기체 (v' = (u - v) / tau).
제어 10 Hz, 적분 50 Hz. 위치 측정 잡음, 통신 지연, 송신 방식(주기 / 이벤트)을 바꿀 수 있다.
ponytail: 이상 모델이다. Gazebo SITL과 실측 tau로 교체해 다시 확인한다.

예:
  python -m swarm.simulator --scenario swap --avoid cbf
  python -m swarm.simulator --scenario formation --avoid apf --comm event --delta 0.1
"""
import argparse
import math

import numpy as np

from . import collision_avoidance as avoid
from . import event_trigger as trigger
from . import formation_control as consensus
from . import path_planning as astar

COLLISION = 0.3  # 8차 회의: 0.3 m 미만 접근을 충돌로 센다


def scenario_swap(n=6, radius=2.0):
    """원 위 n대가 정반대편으로 간다. 대형 없음."""
    ang = np.arange(n) * 2 * math.pi / n
    start = np.c_[np.cos(ang), np.sin(ang)] * radius
    return dict(start=start, goals=-start, offsets=None, path=None, obstacles=[])


def scenario_formation(n=6, ring=0.8, size=15.0, res=0.1, pillar=(7.5, 7.5, 1.0), D=0.4):
    """육각 대형이 가운데 기둥을 A*로 돌아 15 m 방을 가로지른다."""
    h = w = int(size / res) + 1
    yy, xx = np.mgrid[0:h, 0:w] * res
    grid = (xx - pillar[0]) ** 2 + (yy - pillar[1]) ** 2 <= pillar[2] ** 2
    path = astar.plan(grid, res, (2.0, 7.5), (13.0, 7.5), inflate_m=ring + D)
    ang = np.arange(n) * 2 * math.pi / n
    offsets = np.c_[np.cos(ang), np.sin(ang)] * ring
    start = np.array([2.0, 7.5]) + offsets
    return dict(start=start, goals=None, offsets=offsets, path=path,
                obstacles=[(np.array(pillar[:2]), np.zeros(2), pillar[2] + D / 2)])


def run(scenario="swap", avoid_mode="cbf", comm="period", delta=0.1, delay=0.0,
        tau=0.3, noise=0.01, D=0.4, gamma=1.0, vmax=1.0, t_end=40.0, seed=0, n=6):
    rng = np.random.default_rng(seed)
    sc = scenario_swap(n) if scenario == "swap" else scenario_formation(n, D=D)
    p = sc["start"].astype(float).copy()
    v = np.zeros_like(p)
    u = np.zeros_like(p)
    dt, ctrl_dt = 0.02, 0.1
    every = int(round(ctrl_dt / dt))
    follower = consensus.PathFollower(sc["path"]) if sc["path"] else None

    # 통신: 각 기체가 마지막으로 받은 이웃 상태 (p, v, t_src)
    rec = [[None] * n for _ in range(n)]
    inflight = []  # (도착 시각, 보낸 기체, p, v, t_src)
    trig = [trigger.EventTrigger(delta) for _ in range(n)]
    msgs = 0
    dl = [avoid.Deadlock() for _ in range(n)]

    # 안전거리: 이벤트면 예측 오차 2*delta, 지연이면 상대속도 상한 x 지연
    D_eff = D + (2 * delta if comm == "event" else 0.0) + 2 * vmax * delay

    min_sep, infeasible, close_pairs, collisions = math.inf, 0, set(), 0
    form_err, arrive_t = [], None
    for step in range(int(t_end / dt)):
        t = step * dt
        if step % every == 0:
            meas = p + rng.normal(0, noise, p.shape)
            # 송신
            for i in range(n):
                send = comm == "period" or trig[i].should_send(meas[i], v[i], t)
                if send:
                    trig[i].sent(meas[i], v[i], t)
                    inflight.append((t + delay, i, meas[i].copy(), v[i].copy(), t))
                    msgs += 1
            # 수신
            keep = []
            for m in inflight:
                if m[0] <= t + 1e-9:
                    for j in range(n):
                        if j != m[1]:
                            rec[j][m[1]] = m[2:]
                else:
                    keep.append(m)
            inflight = keep
            # 제어
            center = meas.mean(axis=0)
            c_ref = follower.ref(center) if follower else None
            for i in range(n):
                js = [j for j in range(n) if j != i and rec[i][j] is not None]
                nb = [(trigger.predict(*rec[i][j], t), rec[i][j][1]) for j in js]
                if sc["offsets"] is not None:
                    nb_form = [(q, sc["offsets"][j]) for (q, _), j in zip(nb, js)]
                    v_des = consensus.formation_vel(meas[i], sc["offsets"][i], nb_form, c_ref, vmax=vmax)
                else:
                    v_des = consensus.formation_vel(meas[i], np.zeros(2), [], sc["goals"][i], k_f=0.0, vmax=vmax)
                goal = sc["path"][-1] if sc["path"] else sc["goals"][i]
                v_des = dl[i].adjust(v_des, u[i], np.linalg.norm(np.asarray(goal) - meas[i]), t, ctrl_dt)
                others = [(q, w, D_eff) for q, w in nb] + sc["obstacles"]
                if avoid_mode == "cbf":
                    u[i], st = avoid.cbf_qp(v_des, meas[i], others, D_eff, gamma, vmax)
                    infeasible += st == avoid.INFEASIBLE
                elif avoid_mode == "apf":
                    u[i] = avoid.apf(v_des, meas[i], others, rho0=D_eff * 2, vmax=vmax)
                else:
                    u[i] = v_des
        # 운동
        v += (u - v) / tau * dt
        p += v * dt
        # 지표 (참값 위치)
        diff = p[:, None, :] - p[None, :, :]
        dist = np.linalg.norm(diff, axis=-1) + np.eye(n) * 1e9
        min_sep = min(min_sep, dist.min())
        now_close = {(i, j) for i, j in zip(*np.nonzero(dist < COLLISION)) if i < j}
        collisions += len(now_close - close_pairs)
        close_pairs = now_close
        if sc["offsets"] is not None:
            form_err.append(np.mean(np.linalg.norm(p - p.mean(axis=0) - sc["offsets"], axis=1)))
            done = follower.done(p.mean(axis=0))
        else:
            done = np.all(np.linalg.norm(p - sc["goals"], axis=1) < 0.2)
        if done and arrive_t is None:
            arrive_t = t
            break
    return dict(min_sep=round(float(min_sep), 3), collisions=collisions, infeasible=infeasible,
                deadlocks=sum(d.count for d in dl),
                arrive_t=None if arrive_t is None else round(arrive_t, 2), msgs_per_s=round(msgs / (t + dt), 1),
                form_err=round(float(np.mean(form_err)), 3) if form_err else None, D_eff=round(D_eff, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=["swap", "formation"], default="swap")
    ap.add_argument("--avoid", choices=["cbf", "apf", "none"], default="cbf")
    ap.add_argument("--comm", choices=["period", "event"], default="period")
    ap.add_argument("--delta", type=float, default=0.1)
    ap.add_argument("--delay", type=float, default=0.0)
    ap.add_argument("--tau", type=float, default=0.3)
    ap.add_argument("--noise", type=float, default=0.01)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    print(run(a.scenario, a.avoid, a.comm, a.delta, a.delay, a.tau, a.noise, seed=a.seed))


if __name__ == "__main__":
    main()
