"""2026-11-24 3대1 거리 확인 결과표를 measurement_windows.csv에서 다시 계산한다.

    python analyze.py

앵커 이름 = 펌웨어 ID (A1 = ID 1, A2 = ID 2, A3 = ID 3).
레이저 거리는 태그(1호기 EVB)에서 각 앵커까지 직접 잰 값이다.
"""
import csv
import statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent

# 구간: (설명, 앵커별 레이저 거리 m). A2만 S2에서 옮겼다.
WINDOWS = {
    "S0": ("지점 1, 가림 있을 때", {1: 4.31, 2: 5.36, 3: 6.43}),
    "S1a": ("지점 1, 1차", {1: 4.31, 2: 5.36, 3: 6.43}),
    "S1b": ("지점 1, 2차", {1: 4.31, 2: 5.36, 3: 6.43}),
    "S1c": ("지점 1, 3차", {1: 4.31, 2: 5.36, 3: 6.43}),
    "S2a": ("A2 3.70 m로 옮김, 1차", {1: 4.31, 2: 3.70, 3: 6.43}),
    "S2b": ("A2 3.70 m로 옮김, 2차", {1: 4.31, 2: 3.70, 3: 6.43}),
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
            median = st.median(d)
            ok = len(d) / total[window][anchor] * 100
            gap = st.median(r[1] for r in rows) - st.median(r[2] for r in rows)
            print(f"    A{anchor}: 레이저 {laser[anchor]:.2f} m  UWB {median:.3f} m  차이 {(median - laser[anchor]) * 100:+5.1f} cm  "
                  f"성공 {ok:5.1f}%  흔들림 {st.pstdev(d) * 100:5.1f} cm  rssi-fp {gap:4.1f} dB")


if __name__ == "__main__":
    main()
