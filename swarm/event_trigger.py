"""이벤트 트리거 송신.

이웃은 마지막으로 받은 상태를 등속으로 외삽해 내 위치를 예측한다.
실제 위치와 그 예측의 차이가 delta를 넘거나, 경로가 바뀌었거나,
마지막 송신 뒤 max_gap이 지나면 보낸다 (FS_GCS 3 s 대비 최소 1 Hz).
두 기체 모두 예측 오차가 최대 delta이므로 유효 안전거리는 D + 2*delta.
"""
import numpy as np


def predict(p, v, t_src, t):
    return np.asarray(p, float) + np.asarray(v, float) * (t - t_src)


class EventTrigger:
    def __init__(self, delta, max_gap=1.0):
        self.delta = delta
        self.max_gap = max_gap
        self.last = None  # (p, v, t) 마지막으로 보낸 상태

    def should_send(self, p, v, t, path_changed=False):
        if self.last is None or path_changed or t - self.last[2] >= self.max_gap:
            return True
        err = np.linalg.norm(np.asarray(p, float) - predict(*self.last, t))
        return err > self.delta

    def sent(self, p, v, t):
        self.last = (np.asarray(p, float).copy(), np.asarray(v, float).copy(), t)


def effective_distance(D, delta):
    return D + 2.0 * delta
