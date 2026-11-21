# 기체별 올라간 버전

기체에 코드를 올리거나 바꾸면 이 표를 같이 고친다. 해시는 SHA-256이다.

| 기체 | 부품 | 버전 | 해시 / 비고 | 날짜 |
| --- | --- | --- | --- | --- |
| 1호기 | NUCLEO 브리지 | n6_bridge (trusted) | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |
| 4호기 | NUCLEO 브리지 | n6_bridge (trusted) | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |
| 1호기, 4호기 | FC (ArduCopter) | 4.7.3, `fc/base/n6_base.param` 기준 | 4호기 `FLOW_ORIENT_YAW 18000` | 2026-11-21 |
| 4호기 | XIAO BLE 릴레이 | v1 (`xiao/n6_ble_relay_v1/`) | BLE 알림 44개 중 80%가 4바이트 이하, 최대 22바이트라서 v1로 판단 (`tools/hover_test/xiao_version.py`) | 2026-11-21 |
| 1, 2, 3, 5, 6호기 | XIAO BLE 릴레이 | 확인 안 됨 | 기체를 켜고 `xiao_version.py N6-DRONE-N`으로 확인 | - |

시험 기록은 `docs/호버링_시험_절차.md`에 있다.

`xiao/n6_ble_relay_v2/`는 올라가지 않았다. FC에서 오는 데이터가 쪼개져 일부 유실되는 문제를 고친 버전이고, 올리기 전에 따로 시험해야 한다.
