import asyncio
from bleak import BleakScanner

NUS = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"


async def main():
    found = await BleakScanner.discover(timeout=8.0, return_adv=True)
    print("devices:", len(found))
    hits = [(d, a) for d, a in found.values()
            if (a.local_name or d.name or "").startswith("N6-") or NUS in [u.lower() for u in a.service_uuids]]
    for d, a in hits:
        print("HIT", a.local_name or d.name, d.address, "RSSI", a.rssi)
    if not hits:
        print("no N6-* advertising")


asyncio.run(main())
