"""UWB airtime 예산 계산 (CBF-Link 논문 통신 방식의 구현 가능성 F2).

20 ms 제어 프레임 하나에 다음이 들어가는지 본다.
  앵커 측위: 드론 N대 x 앵커 A개 DS-TWR, 측위율 f_loc
  heartbeat: 드론마다 T_h마다 1회 (상태, ACK, 점수)
  피어 교환: 프레임당 K개 DS-TWR (상태 또는 약속 payload)

프레임 길이는 DW3000 BPRF 6.8 Mb/s, PRF 64 MHz 기준 근사식이다.
  프리앰블 심볼 1.0176 us, SFD 8 심볼, PHR 19비트 (850 kb/s, 1.0256 us/비트),
  데이터 0.1282 us/비트, Reed-Solomon 330비트당 48비트 추가.
ponytail: 근사식이다. U1에서 DS-TWR 1회 시간을 실측하면 --twr-us로 그 값을 넣는다.

예:
  python airtime_budget.py
  python airtime_budget.py --twr-us 2800 --loc-hz 20
"""
import argparse
import math

T_PSYM = 1.0176  # us, PRF 64 MHz 프리앰블 심볼
T_PHR_BIT = 1.0256  # us
T_DATA_BIT = 0.1282  # us, 6.8 Mb/s
MAC_OVERHEAD = 9 + 2  # 802.15.4 짧은 주소 헤더 9 B + FCS 2 B


def frame_us(payload, preamble=64, sfd=8):
    bits = (payload + MAC_OVERHEAD) * 8
    bits += math.ceil(bits / 330) * 48  # Reed-Solomon
    return (preamble + sfd) * T_PSYM + 19 * T_PHR_BIT + bits * T_DATA_BIT


def ds_twr_us(payload_poll, payload_resp, payload_final, turnaround, preamble=64):
    """Poll, Response, Final 세 프레임 + 응답 전환 두 번."""
    return (frame_us(payload_poll, preamble) + frame_us(payload_resp, preamble)
            + frame_us(payload_final, preamble) + 2 * turnaround)


def budget(n=6, anchors=3, loc_hz=10.0, hb_period=0.1, hb_payload=51, k=3, ex_payload=40,
           turnaround=300.0, guard=100.0, frame_ms=20.0, preamble=64, twr_us=None):
    """프레임 하나(frame_ms)에 쓰는 평균 airtime(ms)과 판정."""
    loc_twr = twr_us or ds_twr_us(4, 4, 12, turnaround, preamble)
    ex_twr = twr_us or ds_twr_us(ex_payload, ex_payload, ex_payload, turnaround, preamble)
    frames_per_s = 1000.0 / frame_ms
    loc = n * anchors * (loc_twr + guard) * loc_hz / frames_per_s / 1000.0
    hb = n * (frame_us(hb_payload, preamble) + guard) / (hb_period * frames_per_s) / 1000.0
    beacon = (frame_us(8, preamble) + guard) / 1000.0
    ex = k * (ex_twr + guard) / 1000.0
    total = loc + hb + beacon + ex
    return dict(loc_ms=round(loc, 2), hb_ms=round(hb, 2), beacon_ms=round(beacon, 2), ex_ms=round(ex, 2),
                total_ms=round(total, 2), frame_ms=frame_ms, fits=total <= frame_ms,
                loc_twr_us=round(loc_twr), ex_twr_us=round(ex_twr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--anchors", type=int, default=3)
    ap.add_argument("--loc-hz", type=float, default=10.0)
    ap.add_argument("--hb-period", type=float, default=0.1, help="heartbeat 주기 T_h (s)")
    ap.add_argument("--hb-payload", type=int, default=51, help="test_uwb_protocol 직렬화 최대 크기 (ACK 8, 입찰 3)")
    ap.add_argument("--k", type=int, default=3, help="프레임당 피어 교환 수 K")
    ap.add_argument("--ex-payload", type=int, default=40)
    ap.add_argument("--turnaround", type=float, default=300.0, help="RX 끝에서 TX 시작까지 (us)")
    ap.add_argument("--preamble", type=int, default=64)
    ap.add_argument("--twr-us", type=float, default=None, help="실측 DS-TWR 1회 시간 (us). 주면 근사식 대신 사용")
    a = ap.parse_args()
    print("frame us (payload 0/40/72/116 B):",
          [round(frame_us(p, a.preamble)) for p in (0, 40, 72, 116)])
    r = budget(a.n, a.anchors, a.loc_hz, a.hb_period, a.hb_payload, a.k, a.ex_payload,
               a.turnaround, preamble=a.preamble, twr_us=a.twr_us)
    print(r)
    print("판정:", "20 ms 안에 들어간다" if r["fits"] else "20 ms를 넘는다")


if __name__ == "__main__":
    main()
