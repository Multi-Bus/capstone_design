# 정면 카메라 장착 명세 (B 제공)

- 모델: `sensors/front_camera/model.sdf` (URI `model://front_camera`)
- 센서: camera, 640x480, 15Hz, 토픽 `front_camera/image`
- 수평 FOV: 기본 90도 (1.5708 rad). 변경은 `scripts/set_fov.sh <도>`
- 방향: 링크 +x가 정면 (기체 몸체 +x 앞쪽과 일치시킨다)
- 링크 이름: `link` (질량 0.01kg, 중력 비활성)

## 장착 (A가 본체 SDF에서 수행)
- 본체 장착 링크에 fixed joint로 고정한다. include만으로는 고정되지 않는다.
- 권장 장착 위치(본체 base_link 기준, 앞 x, 왼쪽 y, 위 z): (0.1, 0, 0.05) 미터 [미확인: 실제 기체 사양으로 조정]
- 이 위치는 B의 `get_truth`가 카메라 위치(`camera_position_enu_m`)를 계산할 때 쓰는 값이다. 바꾸면 B에게 알린다.

## 검증
- 장착 후 Image Display로 `/front_camera/image`를 열어 기체가 움직일 때 영상이 같이 움직이는지 확인한다.
