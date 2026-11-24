"""2026-11-24 A1 보정 후 거리 비교표를 measurement_windows.csv에서 다시 계산한다.

    python analyze.py

앵커 이름 = 펌웨어 ID (A1 = ID 1, A2 = ID 2, A3 = ID 3).
레이저 거리는 태그(1호기 EVB)에서 각 앵커까지 직접 잰 값이다.
"""
import csv
import statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent

POINT2 = {1: 3.41, 2: 4.97, 3: 5.14}
POINT3_UP = {1: 2.50, 2: 5.91, 3: 5.22}
POINT3 = {1: 2.43, 2: 5.96, 3: 5.33}
POINT4 = {1: 2.94, 2: 5.51, 3: 5.14}
WINDOWS = {
    "E1": ("지점 2, A1 보정 후", POINT2),
    "F1": ("지점 3, 12 cm 올림, 1차", POINT3_UP),
    "F2": ("지점 3, 12 cm 올림, 2차", POINT3_UP),
    "F3": ("지점 3, 12 cm 올림, 3차", POINT3_UP),
    "G1": ("지점 3, 원래 높이, 1차", POINT3),
    "G2": ("지점 3, 원래 높이, 2차", POINT3),
    "G3": ("지점 3, 원래 높이, 3차", POINT3),
    "H1": ("지점 4, 1차", POINT4),
    "H2": ("지점 4, 2차", POINT4),
    "H3": ("지점 4, 3차", POINT4),
}


def main():
    data = {w: {1: [], 2: [], 3: []} for w in WINDOWS}
    total = {w: {1: 0, 2: 0, 3: 0} for w in WINDOWS}
    with open(HERE / "measurement_windows.csv", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            window, anchor = row["window"], int(row["anchor_id"])
            total[window][anchor] += 1
            if row["distance_valid"] == "1":
                data[window][anchor].append((int(row["distance_mm"]) / 1000,
                                             int(row["poll_rssi"]) / 256, int(row["poll_first_path"]) / 256))
    for window, (label, laser) in WINDOWS.items():
        print(f"{window} {label}")
        for anchor in (1, 2, 3):
            rows = data[window][anchor]
            d = sorted(r[0] for r in rows)
            n = len(d)
            median = st.median(d)
            ok = n / total[window][anchor] * 100
            gap = st.median(r[1] for r in rows) - st.median(r[2] for r in rows)
            print(f"    A{anchor}: 레이저 {laser[anchor]:.2f} m  UWB {median:.3f} m  차이 {(median - laser[anchor]) * 100:+5.1f} cm  "
                  f"성공 {ok:5.1f}%  5~95% {d[n // 20]:.3f}~{d[n * 19 // 20]:.3f} m  rssi-fp {gap:4.1f} dB")


if __name__ == "__main__":
    main()
