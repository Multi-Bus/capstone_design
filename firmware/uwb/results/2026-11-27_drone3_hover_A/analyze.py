"""3호기 A 지점 호버 UWB 분석 (2026-11-27). 표준 라이브러리만 씀.

    python analyze.py                  # 이 폴더의 *.raw.log.gz 를 모두 분석

호버 구간: SD 기록의 C 줄(FC 값)에서 시동 ON + GUIDED + FC 높이 1.1 m 이상인 구간.
위치: 앵커가 모두 같은 높이(2.6 m)라 거리식 3개를 서로 빼면 x, y 2x2 선형식이 된다(높이 불필요).
      앵커가 2개만 있으면 FC 높이를 태그 높이로 두고 두 원의 교점 중 A에 가까운 쪽을 쓴다(참고용).
"""
import glob
import gzip
import math
import os
import statistics as st

# 앵커 좌표 (m). 앵커1 원점, 앵커2 -x, 앵커3 +y. 높이 모두 2.6 m (줄자: 1-2 9.98, 2-3 6.34, 1-3 7.41)
ANCHORS = {1: (0.0, 0.0), 2: (-9.98, 0.0), 3: (-5.727, 4.702)}
ANCHOR_Z = 2.6
POINT_A = (-7.303, 0.007)            # 높이 1.3 m 에서 줄자 거리 7.44 / 3.00 / 5.12 m 로 푼 점
REF_A = {1: 7.44, 2: 3.00, 3: 5.12}  # A, 높이 1.3 m 줄자 거리


def read(path):
    C, R = [], {}
    with gzip.open(path, "rt", encoding="ascii", errors="replace") as f:
        for line in f:
            p = line.rstrip("\n").split(",")
            try:
                if p[0] == "C" and len(p) >= 15:
                    # frame, x, y, h(위 +), yaw(deg), mode, armed
                    C.append((int(p[1]), int(p[2]) / 1000, int(p[3]) / 1000, -int(p[4]) / 1000,
                              int(p[10]) / 100, int(p[13]), int(p[14])))
                elif p[0] == "R" and len(p) >= 8 and p[4] == "1":
                    rssi, fp = int(p[6]), int(p[7])
                    R.setdefault(int(p[1]), {})[int(p[2])] = (int(p[3]) / 1000, None if rssi == 32767 else fp - rssi)
            except ValueError:
                pass
    return C, R


def hovers(C):
    segs, cur = [], None
    for c in C:
        if c[6] != 1:
            continue
        if cur is None or c[0] - cur[-1][0] > 50:
            cur = []
            segs.append(cur)
        cur.append(c)
    out = []
    for s in segs:
        hv = [c for c in s if c[5] == 4 and c[3] >= 1.1]
        if hv and (hv[-1][0] - hv[0][0]) * 0.02 >= 15:
            out.append((s, hv))
    return out


(_x1, _y1), (_x2, _y2), (_x3, _y3) = ANCHORS[1], ANCHORS[2], ANCHORS[3]
_A11, _A12, _A21, _A22 = 2 * (_x2 - _x1), 2 * (_y2 - _y1), 2 * (_x3 - _x1), 2 * (_y3 - _y1)
_DET = _A11 * _A22 - _A12 * _A21


def solve3(r1, r2, r3):
    b1 = r1 ** 2 - r2 ** 2 + _x2 ** 2 - _x1 ** 2 + _y2 ** 2 - _y1 ** 2
    b2 = r1 ** 2 - r3 ** 2 + _x3 ** 2 - _x1 ** 2 + _y3 ** 2 - _y1 ** 2
    return (b1 * _A22 - _A12 * b2) / _DET, (_A11 * b2 - _A21 * b1) / _DET


def solve2(ra, rb, a, b, h):
    pa, pb = ANCHORS[a], ANCHORS[b]
    dz2 = (ANCHOR_Z - h) ** 2
    r1, r2 = ra ** 2 - dz2, rb ** 2 - dz2
    d = math.dist(pa, pb)
    ux, uy = (pb[0] - pa[0]) / d, (pb[1] - pa[1]) / d
    t = (r1 - r2 + d * d) / (2 * d)
    hh = r1 - t * t
    if r1 <= 0 or r2 <= 0 or hh < 0:
        return None
    hh = math.sqrt(hh)
    bx, by = pa[0] + t * ux, pa[1] + t * uy
    return min(((bx - hh * uy, by + hh * ux), (bx + hh * uy, by - hh * ux)), key=lambda q: math.dist(q, POINT_A))


def pos_stats(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    mx, my = st.median(xs), st.median(ys)
    d = sorted(math.dist(p, (mx, my)) for p in pts)
    return mx, my, math.dist((mx, my), POINT_A), st.pstdev(xs), st.pstdev(ys), d[int(0.95 * (len(d) - 1))]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    for path in sorted(glob.glob(os.path.join(here, "*.raw.log.gz"))):
        C, R = read(path)
        for seg, hv in hovers(C):
            f0, f1 = hv[0][0], hv[-1][0]
            hs, yw = [c[3] for c in hv], [c[4] for c in hv]
            print(f"\n== {os.path.basename(path)} frames {f0}-{f1} ({(f1 - f0) * 0.02:.1f} s)")
            print(f"   FC 높이 {st.mean(hs):.2f} m ({min(hs):.2f}~{max(hs):.2f}), 요 변화 {max(yw) - min(yw):.0f}°")
            hr = {f: r for f, r in R.items() if f0 <= f <= f1}
            for a in (1, 2, 3):
                v = sorted(r[a][0] for r in hr.values() if a in r)
                if not v:
                    print(f"   A{a}: 기록 없음")
                    continue
                print(f"   A{a}: n {len(v)} 중앙값 {st.median(v):.3f} sd {st.pstdev(v):.3f} "
                      f"(5~95% {v[int(.05 * (len(v) - 1))]:.3f}~{v[int(.95 * (len(v) - 1))]:.3f}) "
                      f"기준 {REF_A[a]:.2f} 차이 {st.median(v) - REF_A[a]:+.3f}")
            full = [r for f, r in sorted(hr.items()) if all(k in r for k in (1, 2, 3))]
            if full:
                s = pos_stats([solve3(r[1][0], r[2][0], r[3][0]) for r in full])
                print(f"   위치 (앵커 3개) n {len(full)}: ({s[0]:+.3f}, {s[1]:+.3f}), A와 {s[2]:.3f} m, "
                      f"sd x {s[3]:.3f} y {s[4]:.3f}, r95 {s[5]:.3f}")
                good = [r for r in full if r[3][1] is not None and r[3][1] >= -1000]
                if good and len(good) < len(full):
                    s = pos_stats([solve3(r[1][0], r[2][0], r[3][0]) for r in good])
                    print(f"   위치 (앵커3 첫 신호 약한 값 제외, FP-RSSI >= -1000) n {len(good)}: ({s[0]:+.3f}, {s[1]:+.3f}), "
                          f"A와 {s[2]:.3f} m, sd x {s[3]:.3f} y {s[4]:.3f}, r95 {s[5]:.3f}")
            else:
                ck = [c[0] for c in C]
                import bisect
                pts = []
                for f, r in sorted(hr.items()):
                    if 1 in r and 3 in r:
                        h = C[max(bisect.bisect_right(ck, f) - 1, 0)][3]
                        q = solve2(r[1][0], r[3][0], 1, 3, h)
                        if q:
                            pts.append(q)
                if pts:
                    s = pos_stats(pts)
                    print(f"   위치 (앵커 1·3 + FC 높이, 참고용) n {len(pts)}: ({s[0]:+.3f}, {s[1]:+.3f}), A와 {s[2]:.3f} m, "
                          f"sd x {s[3]:.3f} y {s[4]:.3f}, r95 {s[5]:.3f}")


if __name__ == "__main__":
    main()
