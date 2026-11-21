"""BLE 스캔: 주변에서 N6-DRONE-* (XIAO) 가 광고 중인지 확인 (연결·쓰기 없음).

    cd tools && source .venv/bin/activate
    python dev/ble_scan.py

- "SCAN ERROR" → PC 블루투스 어댑터 문제 (QUICKSTART_KO.md 5장)
- 광고 장치는 보이는데 N6-* 가 없음 → XIAO 전원, 또는 다른 기기가 이미 연결 중 (XIAO 파란 LED 켜짐)
"""
import asyncio

from bleak import BleakScanner

NUS_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"   # Nordic UART Service


async def main():
    try:
        found = await BleakScanner.discover(timeout=8.0, return_adv=True)
    except Exception as e:  # noqa: BLE001
        print("SCAN ERROR:", repr(e))
        return
    print(f"광고 장치 {len(found)}개 발견")
    hits = [(d, a) for d, a in found.values()
            if (a.local_name or d.name or "").startswith("N6-") or NUS_SVC in [u.lower() for u in a.service_uuids]]
    for d, a in hits:
        print(f"  {a.local_name or d.name}  {d.address}  RSSI {a.rssi} dBm")
    if not hits:
        print("  N6-* 장치 없음")


asyncio.run(main())
