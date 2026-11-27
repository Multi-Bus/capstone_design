"""앵커 3대 차례 맞추기 시뮬레이터 (보드 없이 src/main.c 의 규칙만 흉내).

    python tools/sched_sim.py            # 여러 시작 조건으로 돌려서 겹침/측정 빈도 확인

모델
- 앵커마다 시계 오차 ±20 ppm, 켜지는 시각은 무작위 (0~3 s).
- 처음 두 바퀴 + 난수 동안 듣기만 함. 내 교환(약 2 ms) 중에는 남의 Poll 을 못 들음.
- 남의 Poll 을 들으면: 내 다음 Poll = 들은 시각 - 0.2 ms + ((내번호 - j) mod 3) x 칸.
- 교환 뒤: 다음 = 지난 예정 + 한 바퀴. 최근 세 바퀴 동안 아무도 못 들었으면 0~칸/2 난수를 더함.
- 두 교환이 시간상 겹치면 '충돌' (둘 다 실패로 셈).
"""
import random
import sys

SLOT = 20000.0          # us
CYCLE = 3 * SLOT
EXCHANGE = 2000.0       # Poll~Final 약 1.8 ms
LATENCY = 200.0
PREP = 300.0
SYNC_LOST = 3 * CYCLE


class Anchor:
    def __init__(self, ident, boot, ppm, rng):
        self.id = ident
        self.boot = boot
        self.rate = 1.0 + ppm * 1e-6        # 실제 1 us 당 내 시계 진행
        self.rng = rng
        self.next_local = 2 * CYCLE + rng.uniform(0, SLOT)
        self.last_heard_local = None
        self.busy_until = -1.0              # 실제 시각
        self.tx_times = []

    def local(self, t):
        return (t - self.boot) * self.rate

    def real(self, local):
        return self.boot + local / self.rate

    def synced(self, t):
        return self.last_heard_local is not None and self.local(t) - self.last_heard_local < SYNC_LOST

    def hear(self, j, t_end):
        if t_end < self.boot or t_end < self.busy_until or j == self.id:
            return
        now = self.local(t_end)
        self.last_heard_local = now
        self.next_local = now - LATENCY + ((self.id + 3 - j) % 3) * SLOT


def simulate(seed, seconds=30.0, boot_spread=3e6):
    rng = random.Random(seed)
    anchors = [Anchor(i, rng.uniform(0, boot_spread), rng.uniform(-20, 20), rng) for i in (1, 2, 3)]
    t_end = max(a.boot for a in anchors) + seconds * 1e6
    events = []      # (start, end, id)
    while True:
        a = min(anchors, key=lambda x: x.real(x.next_local))
        t = a.real(a.next_local)
        if t > t_end:
            break
        end = t + EXCHANGE
        events.append((t, end, a.id))
        a.tx_times.append(t)
        a.busy_until = end
        for other in anchors:
            other.hear(a.id, t + LATENCY)
        sync = a.synced(end)
        a.next_local += CYCLE
        if not sync:
            a.next_local += a.rng.uniform(0, SLOT / 2)
        if a.next_local - a.local(end) < PREP:
            a.next_local = a.local(end) + CYCLE
    events.sort()
    collided = set()
    for i in range(len(events) - 1):
        for k in range(i + 1, len(events)):
            if events[k][0] >= events[i][1]:
                break
            collided.add(i)
            collided.add(k)
    settle = max(a.boot for a in anchors) + 2e6          # 마지막 앵커가 켜진 뒤 2 s 이후만 평가
    late = [i for i, e in enumerate(events) if e[0] >= settle]
    late_coll = sum(1 for i in late if i in collided)
    gaps = [events[i + 1][0] - events[i][0] for i in late[:-1]]
    rate = {a.id: sum(1 for t in a.tx_times if t >= settle) / ((t_end - settle) / 1e6) for a in anchors}
    return len(collided), late_coll, len(late), min(gaps) if gaps else 0, rate


def main():
    worst = 0
    for seed in range(200):
        total, late_coll, late_n, min_gap, rate = simulate(seed)
        worst = max(worst, late_coll)
        if seed < 5 or late_coll:
            print(f"seed {seed:3d}: 충돌 전체 {total:3d} | 안정 뒤 충돌 {late_coll}/{late_n} | "
                  f"안정 뒤 최소 간격 {min_gap/1000:.2f} ms | 앵커별 Hz " +
                  " ".join(f"A{k}={v:.1f}" for k, v in rate.items()))
    print(f"200개 시작 조건 중 안정 뒤 최대 충돌 수: {worst}")
    sys.exit(1 if worst else 0)


if __name__ == "__main__":
    main()
