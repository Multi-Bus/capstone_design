#!/bin/bash
# 두 드론 GPS-off 광류 점검 (A 가이드 연결 경로): 헤드리스 Gazebo -> 드론 0,1 스폰 -> SITL 2개 + MAVProxy --out
# (제어 14550/14560, 센서 브리지 15550/15560) -> scripts/two_flow_check.py -> 로그 요약 -> 내 프로세스 그룹만 정리.
# 사용: scripts/run_two_flow_check.sh      (bash로 실행. 환경변수: PART 기본 agentM, RUN 기본 agentM, LOGD 기본 /tmp/agentM/$RUN)
source ~/drone_project/env.sh
PART=${PART:-agentM}; RUN=${RUN:-agentM}; LOGD=${LOGD:-/tmp/agentM/$RUN}
export GZ_PARTITION=$PART
cd ~/drone_project
PY=~/venv-ardupilot/bin/python
export MAVPROXY_CMD=~/venv-ardupilot/bin/mavproxy.py  # sim_vehicle이 PATH에서 mavproxy.py를 찾는다
CTL=(14550 14560); BRG=(15550 15560)
mkdir -p $LOGD runs/${RUN}_0 runs/${RUN}_1
rm -f $LOGD/*.log
PIDS=""

cleanup() {  # 내가 띄운 프로세스 그룹만 정리 (pkill 금지)
  for p in $PIDS; do kill -- -$p 2>/dev/null; done; sleep 3
  for p in $PIDS; do kill -9 -- -$p 2>/dev/null; done; sleep 1
}

setsid gz sim -s -r -v4 --headless-rendering worlds/room.sdf > $LOGD/gz.log 2>&1 &
PIDS="$!"
for _ in $(seq 40); do gz topic -l 2>/dev/null | grep -q dynamic_pose/info && break; sleep 1; done

echo "== 스폰"
if ! $PY scripts/spawn_drones.py 0 1 > $LOGD/spawn.log 2>&1; then cat $LOGD/spawn.log; cleanup; echo "스폰 실패"; exit 1; fi
cat $LOGD/spawn.log
for t in /drone_0/down_camera/image /drone_0/down_range/scan /drone_1/down_camera/image /drone_1/down_range/scan; do
  for _ in $(seq 30); do gz topic -l 2>/dev/null | grep -qx "$t" && break; sleep 1; done
  gz topic -l 2>/dev/null | grep -qx "$t" && echo "토픽 OK $t" || echo "토픽 없음 $t"
done

echo "== SITL 2개 (MAVProxy --out)"
for i in 0 1; do
  (cd runs/${RUN}_$i && exec setsid $PY ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
     -I$i --sysid $((i + 1)) --no-rebuild -w --add-param-file=$HOME/drone_project/config/flow_test.parm --no-extra-ports \
     --out=udp:127.0.0.1:${CTL[$i]} --out=udp:127.0.0.1:${BRG[$i]} --mavproxy-args=--daemon \
     < /dev/null > $LOGD/sitl_$i.log 2>&1) &
  PIDS="$PIDS $!"
done
sleep 12

echo "== 센서 브리지 2개"
for i in 0 1; do
  setsid $PY scripts/sensor_bridge.py --cam /drone_$i/down_camera/image --range /drone_$i/down_range/scan \
    --mav udpin:127.0.0.1:${BRG[$i]} > $LOGD/bridge_$i.log 2>&1 &
  PIDS="$PIDS $!"
done
sleep 3

echo "== 시험 (two_flow_check.py)"
timeout 900 $PY -u scripts/two_flow_check.py > $LOGD/two_flow.log 2>&1
RC=$?
tail -80 $LOGD/two_flow.log
echo "== bridge (시작 줄, 마지막 flow/range 줄)"
for i in 0 1; do grep -m1 "bridge 시작" $LOGD/bridge_$i.log; grep "flow n=" $LOGD/bridge_$i.log | tail -1; done
echo "== gz 오류"; grep -iE "error|fail" $LOGD/gz.log | head -5
cleanup
echo "== SITL STATUSTEXT (MAVProxy tlog: EKF/flow/failsafe/PreArm)"
for i in 0 1; do
  echo "-- SITL $i (sysid $((i + 1)))"
  $PY - runs/${RUN}_$i/mav.tlog <<'PYEOF'
import sys
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], dialect="ardupilotmega")
keys = ("EKF3", "optical", "relative aiding", "ailsafe", "PreArm")
while True:
    msg = m.recv_match(blocking=False)
    if msg is None:
        break
    if msg.get_type() == "STATUSTEXT" and any(k in msg.text for k in keys):
        print("  ", msg.text)
PYEOF
done
PG=$(echo $PIDS | xargs | tr ' ' ',')
LEFT=$(pgrep -g "$PG" 2>/dev/null)
echo "== leftover (내 프로세스 그룹 $PG): ${LEFT:-none}"
echo "rc=$RC"
exit $RC
