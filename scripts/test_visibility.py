"""visibility.py 자체 점검 (A 가이드 §13 체크리스트). 실행: python3 test_visibility.py"""
import math
from visibility import visible_drones

PI = math.pi


def vis(target, cam=(0.0, 0.0, 0.0), yaw=0.0, **kw):
    return visible_drones(0, cam, {0: cam, 1: target}, yaw, **kw)


# 정면·거리 안쪽 -> 포함
assert vis((3, 0, 0)) == [1]
assert vis((3, 1, 0.5)) == [1]
# 뒤쪽 / FOV 밖 / 거리 밖 -> 제외
assert vis((-3, 0, 0)) == []
assert vis((1, 3, 0)) == []          # 약 71.6도, FOV(+-45도) 밖
assert vis((11, 0, 0)) == []
assert vis((8, 0, 8)) == []          # 3차원 거리 11.3m
# 자기 자신 제외
assert visible_drones(0, (0, 0, 0), {0: (0, 0, 0)}, 0.0) == []
assert visible_drones(0, (0, 0, 0), {0: (1, 0, 0), 1: (3, 0, 0)}, 0.0) == [1]
# 경계 포함: 거리 10m 정확히, FOV 경계(45도) 정확히
assert vis((10, 0, 0)) == [1]
assert vis((1, 1, 0)) == [1]
assert vis((1, -1, 0)) == [1]
assert vis((10, 0, 0.001)) == []     # 거리 경계 바로 밖
# yaw +-pi 통과
assert vis((-3, 0.5, 0), yaw=PI) == [1]
assert vis((-3, -0.5, 0), yaw=-PI) == [1]
assert vis((-3, 0.5, 0), yaw=-PI) == [1]   # 같은 방향의 다른 표현
assert vis((3, 0, 0), yaw=PI) == []
assert vis((-1, 1, 0), yaw=PI) == [1]      # 경계(45도) + 각도 wrap
# 카메라 위치/yaw 적용
assert vis((2, 5, 0), cam=(2, 1, 0), yaw=PI / 2) == [1]
# 같은 수평 위치의 바로 위/아래 -> 제외
assert vis((0, 0, 2)) == []
assert vis((0, 0, -2)) == []
# 여러 대는 정렬된 목록
assert visible_drones(0, (0, 0, 0), {2: (4, 0, 0), 1: (3, 0, 0), 0: (0, 0, 0)}, 0.0) == [1, 2]

print("test_visibility: all passed")
