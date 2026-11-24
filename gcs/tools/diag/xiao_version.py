"""읽기 전용: BLE로 연결해 FC가 보내는 알림 크기로 XIAO 릴레이 버전을 구분한다.
v1은 알림이 1~4바이트로 잘게 쪼개지고, v2는 최대 MTU-3(보통 244B)까지 묶어 보낸다.
사용: xiao_version.py N6-DRONE-4 [초]"""
import asyncio
import sys

from bleak import BleakClient, BleakScanner

NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # XIAO -> PC


async def main(name, secs):
    dev = await BleakScanner.find_device_by_name(name, timeout=10)
    if dev is None:
        sys.exit(f"{name} 광고 없음")
    sizes = []
    async with BleakClient(dev) as c:
        await c.start_notify(NUS_TX, lambda _, d: sizes.append(len(d)))
        await asyncio.sleep(secs)
    if not sizes:
        sys.exit("알림 없음 (FC 전원/연결 확인)")
    n, mx = len(sizes), max(sizes)
    small = sum(s <= 4 for s in sizes) / n
    print(f"알림 {n}개, 최대 {mx}B, 4B 이하 {small:.0%} -> {'v1(쪼개짐)' if small > 0.5 else 'v2(묶음)'}")


asyncio.run(main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 8))
