"""visible_drones: 시야 판정.

A 담당 최종본이 오면 교체하는 참조 구현(A 가이드 §13).
입력은 같은 시각의 ENU 실제 좌표. 대상은 중심점으로 근사하고 수직 FOV·가림은 판정하지 않는다.
거리/FOV 경계는 포함, 자기 자신은 제외.
"""
import math


def visible_drones(observer_id, camera_position_enu, positions_enu,
                   camera_yaw_enu, horizontal_fov_rad=math.pi / 2,
                   max_range_m=10.0):
    ox, oy, oz = camera_position_enu
    result = []
    for drone_id, (x, y, z) in positions_enu.items():
        if drone_id == observer_id:
            continue
        dx, dy, dz = x - ox, y - oy, z - oz
        if math.sqrt(dx*dx + dy*dy + dz*dz) > max_range_m:
            continue
        if math.hypot(dx, dy) < 1e-9:
            continue
        angle = math.atan2(dy, dx) - camera_yaw_enu
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= horizontal_fov_rad / 2:
            result.append(drone_id)
    return sorted(result)
