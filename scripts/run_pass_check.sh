#!/bin/bash
# 2대 스침 시험 일괄 실행: 헤드리스 Gazebo -> 드론 2대 스폰 -> SITL 2개(GPS ON) -> pass_check.py -> csv_logger.py, 끝나면 정리.
# 사용: scripts/run_pass_check.sh        (결과 CSV: $OUT)
# 환경변수: IDS(드론 id 둘, 기본 "0 1") PART(gz 파티션, 기본 pass_check) RUN(runs/<RUN>_<id> 실행 폴더, 기본 pass_check)
#           LOGD(로그 폴더, 기본 /tmp/$RUN) OUT(CSV, 기본 $LOGD/pass_check.csv) DUR(로그 시뮬 초, 기본 70)
# 첫 id: (-4,-0.75) yaw 0 에서 동쪽, 둘째 id: (4,+0.75) yaw pi 에서 서쪽으로 0.5m/s, 각 8m (scripts/pass_check.py).
# SITL 인스턴스 = id, sysid = id+1, TCP 5760+10*id, 플러그인 포트 9002+10*id.
source ~/drone_project/env.sh
IDS=${IDS:-"0 1"}; PART=${PART:-pass_check}; RUN=${RUN:-pass_check}; LOGD=${LOGD:-/tmp/$RUN}; DUR=${DUR:-70}
OUT=${OUT:-$LOGD/pass_check.csv}
set -- $IDS; A=$1; B=$2
export GZ_PARTITION=$PART
cd ~/drone_project
PY=~/venv-ardupilot/bin/python
mkdir -p $LOGD
PIDS=""

setsid gz sim -s -r -v4 --headless-rendering worlds/room.sdf > $LOGD/gz.log 2>&1 &
GZ=$!; PIDS="$GZ"
for _ in $(seq 40); do gz topic -l 2>/dev/null | grep -q dynamic_pose/info && break; sleep 1; done
echo "== 스폰"
$PY scripts/spawn_drones.py $A $B --pos $A=-4,-0.75,0.2,0 --pos $B=4,0.75,0.2,3.14159265 || { echo "스폰 실패"; }

for i in $A $B; do
  d=runs/${RUN}_$i; mkdir -p $d
  (cd $d && exec setsid $PY ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
     -I$i --sysid $((i + 1)) --no-mavproxy --no-rebuild -w --add-param-file=$HOME/drone_project/config/frame.parm \
     < /dev/null > $LOGD/sitl_$i.log 2>&1) &
  PIDS="$PIDS $!"
done

echo "== pass_check"
$PY -u scripts/pass_check.py $A $B > $LOGD/pass_check.log 2>&1 &
PC=$!
for _ in $(seq 600); do grep -q LOG_START $LOGD/pass_check.log && break; kill -0 $PC 2>/dev/null || break; sleep 1; done
if grep -q LOG_START $LOGD/pass_check.log; then
  cd scripts && $PY csv_logger.py --ids $A $B --world room --out $OUT --duration $DUR --run-id pass_${A}_${B} > $LOGD/logger.log 2>&1 &
  LG=$!; cd ..
fi
wait $PC; RC=$?
[ -n "$LG" ] && wait $LG
echo "== pass_check 결과(rc=$RC)"; tail -25 $LOGD/pass_check.log
echo "== logger"; cat $LOGD/logger.log 2>/dev/null
echo "== SITL 메시지"; grep -hE "no JSON|failsafe|PreArm" $LOGD/sitl_*.log | sort | uniq -c | head
for p in $PIDS; do kill -- -$p 2>/dev/null; done; sleep 3
# gz 서버가 SIGTERM에 끝나지 않고 남는 경우가 있었다(1회 관찰). 남은 그룹은 강제 종료
for p in $PIDS; do kill -9 -- -$p 2>/dev/null; done; sleep 1
echo "== leftover (내 프로세스 그룹)"; pgrep -a -g $(echo $PIDS | tr ' ' ',') | head -5 || true
echo "CSV: $OUT"
exit $RC
