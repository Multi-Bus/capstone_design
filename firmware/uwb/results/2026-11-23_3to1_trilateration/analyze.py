"""2026-11-23 3대1 삼변측량 결과표를 measurement_windows.csv에서 다시 계산한다.

    python analyze.py

방법론 3-1: 태그 높이를 아는 값으로 두고 3D 거리를 수평 거리로 바꾼 뒤,
원 3개를 가우스-뉴턴 최소제곱으로 푼다. 거리는 구간별 중앙값을 쓴다.
"""
import csv
import math
import statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent

# 자리별 좌표 (m). 펌웨어 ID와 자리가 다르다: A1 자리 = ID 2, A2 자리 = ID 1.
ANCHORS = {"A1": (2, (0.0, 0.0, 2.0)), "A2": (1, (4.5, 0.0, 1.7)), "A3": (3, (2.25, 4.5, 1.5))}

# 지점: (설명, 줄자 위치 x, y, 태그 높이 z) [m]
POINTS = {
    "P1": ("정중앙", 2.25, 2.25, 0.09),
    "P2": ("A2 쪽", 3.15, 2.25, 0.09),
    "P3": ("A1 쪽", 1.35, 2.25, 0.09),
    "P4a": ("A3 쪽 1차", 2.25, 3.15, 0.09),
    "P4b": ("A3 쪽 2차", 2.25, 3.15, 0.09),
    "P5": ("A3 반대쪽", 2.25, 1.35, 0.09),
    "P6a": ("정중앙 82 cm 1차", 2.25, 2.25, 0.82),
    "P6b": ("정중앙 82 cm 2차", 2.25, 2.25, 0.82),
}


def solve(distances, tag_z):
    rho = {k: math.sqrt(max(distances[k] ** 2 - (tag_z - a[2]) ** 2, 0.0)) for k, (_, a) in ANCHORS.items()}
    x, y = 2.25, 2.25
    for _ in range(100):
        rows = []
        for k, (_, (ax, ay, _)) in ANCHORS.items():
            h = math.hypot(x - ax, y - ay)
            rows.append(((x - ax) / h, (y - ay) / h, rho[k] - h))
        a11 = sum(j0 * j0 for j0, _, _ in rows)
        a12 = sum(j0 * j1 for j0, j1, _ in rows)
        a22 = sum(j1 * j1 for _, j1, _ in rows)
        b1 = sum(j0 * e for j0, _, e in rows)
        b2 = sum(j1 * e for _, j1, e in rows)
        det = a11 * a22 - a12 * a12
        dx, dy = (a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det
        x, y = x + dx, y + dy
        if abs(dx) + abs(dy) < 1e-9:
            break
    return x, y


def main():
    data = {p: {1: [], 2: [], 3: []} for p in POINTS}
    total = {p: {1: 0, 2: 0, 3: 0} for p in POINTS}
    with open(HERE / "measurement_windows.csv", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            point, anchor = row["point"], int(row["anchor_id"])
            total[point][anchor] += 1
            if row["distance_valid"] == "1":
                data[point][anchor].append((int(row["distance_mm"]) / 1000,
                                            int(row["poll_rssi"]) / 256, int(row["poll_first_path"]) / 256))
    for point, (label, tx, ty, tz) in POINTS.items():
        tag = (tx, ty, tz)
        medians, cells = {}, []
        for name, (anchor_id, coord) in ANCHORS.items():
            rows = data[point][anchor_id]
            d = [r[0] for r in rows]
            medians[name] = st.median(d)
            error = (medians[name] - math.dist(tag, coord)) * 100
            ok = len(d) / total[point][anchor_id] * 100
            gap = st.median(r[1] for r in rows) - st.median(r[2] for r in rows)
            cells.append(f"{name}(ID{anchor_id}) {error:+5.1f} cm {ok:5.1f}% sd {st.pstdev(d) * 100:4.1f} cm  rssi-fp {gap:4.1f} dB")
        x, y = solve(medians, tz)
        print(f"{point} {label}: 줄자 ({tx * 100:.0f}, {ty * 100:.0f}, {tz * 100:.0f}) -> UWB ({x * 100:.1f}, {y * 100:.1f}) "
              f"dx {(x - tx) * 100:+.1f} dy {(y - ty) * 100:+.1f} 수평 {math.hypot(x - tx, y - ty) * 100:.1f} cm")
        for cell in cells:
            print("   ", cell)


if __name__ == "__main__":
    main()
